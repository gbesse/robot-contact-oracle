"""Black-box contact response checks for Gazebo Sim."""

from __future__ import annotations

import argparse
import json
import math
import os
from pathlib import Path
import re
import shutil
import signal
import subprocess
import sys
import tempfile
import time
import uuid


MESSAGES = {
    "fr": {
        "pass": "réussi",
        "fail": "échec",
        "inconclusive": "indéterminé",
        "missing_gz": "Gazebo Sim est introuvable : commande « gz » absente.",
        "invalid": "Entrée invalide",
        "summary": "Bilan",
        "sample": "échantillon",
        "error": "raison",
    },
    "en": {
        "pass": "pass",
        "fail": "fail",
        "inconclusive": "inconclusive",
        "missing_gz": "Gazebo Sim was not found: the “gz” command is missing.",
        "invalid": "Invalid input",
        "summary": "Summary",
        "sample": "sample",
        "error": "reason",
    },
    "es": {
        "pass": "correcto",
        "fail": "fallo",
        "inconclusive": "indeterminado",
        "missing_gz": "No se encontró Gazebo Sim: falta el comando « gz ».",
        "invalid": "Entrada no válida",
        "summary": "Resumen",
        "sample": "muestra",
        "error": "motivo",
    },
}


class InputError(ValueError):
    pass


class CaptureError(RuntimeError):
    pass


def _number(value: object, label: str) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise InputError(f"{label}: expected a finite number")
    number = float(value)
    if not math.isfinite(number):
        raise InputError(f"{label}: expected a finite number")
    return number


def load_manifest(path: Path) -> dict:
    try:
        raw = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as exc:
        raise InputError(str(exc)) from exc
    if not isinstance(raw, dict) or raw.get("schema_version") != 1:
        raise InputError("schema_version must be 1")
    cases = raw.get("cases")
    if not isinstance(cases, list) or not cases:
        raise InputError("cases must be a nonempty array")
    ids = set()
    checked = []
    for index, case in enumerate(cases):
        if not isinstance(case, dict):
            raise InputError(f"cases[{index}] must be an object")
        case_id = case.get("id")
        if not isinstance(case_id, str) or not case_id or case_id in ids:
            raise InputError(f"cases[{index}].id must be unique and nonempty")
        ids.add(case_id)
        for key in ("world", "world_name", "pose_topic", "entity_name"):
            if not isinstance(case.get(key), str) or not case[key]:
                raise InputError(f"{case_id}.{key} must be a nonempty string")
        world = (path.parent / case["world"]).resolve()
        if not world.is_file() or world.suffix.lower() != ".sdf":
            raise InputError(f"{case_id}.world must point to an SDF file")
        thresholds = case.get("sample_after_iterations")
        if (
            not isinstance(thresholds, list)
            or not thresholds
            or any(isinstance(n, bool) or not isinstance(n, int) or n <= 0 for n in thresholds)
            or thresholds != sorted(set(thresholds))
        ):
            raise InputError(f"{case_id}.sample_after_iterations must be increasing positive integers")
        low = case.get("min_z_m")
        high = case.get("max_z_m")
        if low is None and high is None:
            raise InputError(f"{case_id} needs min_z_m or max_z_m")
        low = None if low is None else _number(low, f"{case_id}.min_z_m")
        high = None if high is None else _number(high, f"{case_id}.max_z_m")
        if low is not None and high is not None and low > high:
            raise InputError(f"{case_id}: min_z_m exceeds max_z_m")
        timeout = _number(case.get("timeout_s", 30), f"{case_id}.timeout_s")
        if timeout <= 0:
            raise InputError(f"{case_id}.timeout_s must be positive")
        checked.append({**case, "world": str(world), "min_z_m": low, "max_z_m": high, "timeout_s": timeout})
    return {"schema_version": 1, "cases": checked}


def parse_iterations(message: str) -> int:
    match = re.search(r"(?m)^\s*iterations:\s*(\d+)\s*$", message)
    if not match:
        raise CaptureError("stats message has no iterations field")
    return int(match.group(1))


def parse_pose_z(message: str, entity_name: str) -> float:
    # The fixture publishes one gz.msgs.Pose, not a Pose_V bundle.
    names = re.findall(r'(?m)^\s*name:\s*"([^"]+)"\s*$', message)
    if names != [entity_name]:
        raise CaptureError(f"pose entity mismatch: expected {entity_name!r}, got {names!r}")
    position = re.search(r"(?m)^\s*position\s*\{([^{}]*)\}", message, re.DOTALL)
    if not position:
        raise CaptureError("pose message has no position")
    match = re.search(r"(?m)^\s*z:\s*([-+\d.eE]+)\s*$", position.group(1))
    # Protobuf text format may omit default-valued coordinates.
    z = 0.0 if not match else float(match.group(1))
    if not math.isfinite(z):
        raise CaptureError("pose z is not finite")
    return z


def _topic_once(topic: str, env: dict, timeout_s: float) -> str:
    try:
        result = subprocess.run(
            ["gz", "topic", "-e", "-n", "1", "-t", topic],
            env=env,
            capture_output=True,
            text=True,
            timeout=max(0.1, min(3.0, timeout_s)),
            check=False,
        )
    except subprocess.TimeoutExpired as exc:
        raise CaptureError(f"topic {topic} timed out") from exc
    if result.returncode != 0 or not result.stdout.strip():
        raise CaptureError(f"topic {topic} returned no message")
    return result.stdout


def _stop_process(process: subprocess.Popen) -> None:
    if process.poll() is not None:
        return
    try:
        os.killpg(process.pid, signal.SIGTERM)
        process.wait(timeout=3)
    except (ProcessLookupError, subprocess.TimeoutExpired):
        if process.poll() is None:
            os.killpg(process.pid, signal.SIGKILL)
            process.wait(timeout=3)


def capture_case(case: dict) -> dict:
    env = os.environ.copy()
    env["GZ_PARTITION"] = f"contact-oracle-{uuid.uuid4().hex}"
    deadline = time.monotonic() + case["timeout_s"]
    stats_topic = f'/world/{case["world_name"]}/stats'
    evidence = {"id": case["id"], "samples": []}
    with tempfile.TemporaryFile(mode="w+t", encoding="utf-8") as log:
        try:
            process = subprocess.Popen(
                ["gz", "sim", "-s", "-r", "-v", "2", case["world"]],
                env=env,
                stdout=log,
                stderr=subprocess.STDOUT,
                start_new_session=True,
            )
        except OSError as exc:
            return {**evidence, "error": f"simulator launch failed: {exc}"}
        try:
            baseline = None
            while baseline is None and time.monotonic() < deadline:
                if process.poll() is not None:
                    raise CaptureError(f"simulator exited with code {process.returncode}")
                try:
                    baseline = parse_iterations(_topic_once(stats_topic, env, deadline - time.monotonic()))
                except CaptureError:
                    time.sleep(0.1)
            if baseline is None:
                raise CaptureError("no simulator statistics before timeout")
            evidence["baseline_iterations"] = baseline
            for offset in case["sample_after_iterations"]:
                target = baseline + offset
                while time.monotonic() < deadline:
                    if process.poll() is not None:
                        raise CaptureError(f"simulator exited with code {process.returncode}")
                    try:
                        iterations = parse_iterations(_topic_once(stats_topic, env, deadline - time.monotonic()))
                    except CaptureError:
                        continue
                    if iterations >= target:
                        pose = _topic_once(case["pose_topic"], env, deadline - time.monotonic())
                        evidence["samples"].append({"iterations": iterations, "z_m": parse_pose_z(pose, case["entity_name"])})
                        break
                    time.sleep(0.05)
                else:
                    raise CaptureError(f"target iteration {target} not reached")
        except CaptureError as exc:
            evidence["error"] = str(exc)
        finally:
            _stop_process(process)
    return evidence


def evaluate_case(case: dict, evidence: dict) -> dict:
    samples = evidence.get("samples")
    result = {"id": case["id"], "status": "inconclusive", "samples": samples if isinstance(samples, list) else []}
    if evidence.get("error"):
        result["reason"] = str(evidence["error"])
        return result
    if not isinstance(samples, list) or len(samples) != len(case["sample_after_iterations"]):
        result["reason"] = "missing pose samples"
        return result
    baseline = evidence.get("baseline_iterations")
    if isinstance(baseline, bool) or not isinstance(baseline, int) or baseline < 0:
        result["reason"] = "missing baseline iteration"
        return result
    previous = -1
    for sample, offset in zip(samples, case["sample_after_iterations"]):
        if not isinstance(sample, dict) or isinstance(sample.get("iterations"), bool) or not isinstance(sample.get("iterations"), int):
            result["reason"] = "invalid sample iteration"
            return result
        if sample["iterations"] <= previous or sample["iterations"] < baseline + offset:
            result["reason"] = "sample iteration is before its required threshold"
            return result
        previous = sample["iterations"]
        try:
            z = _number(sample.get("z_m"), "z_m")
        except InputError:
            result["reason"] = "invalid sample height"
            return result
        low, high = case["min_z_m"], case["max_z_m"]
        if (low is not None and z < low) or (high is not None and z > high):
            result["status"] = "fail"
            result["reason"] = f"z={z:g} m outside expected range [{low}, {high}]"
            return result
    result["status"] = "pass"
    return result


def evaluate(manifest: dict, evidence: dict) -> dict:
    if not isinstance(evidence, dict) or evidence.get("schema_version") != 1 or not isinstance(evidence.get("cases"), list):
        raise InputError("evidence needs schema_version 1 and a cases array")
    by_id = {}
    for item in evidence["cases"]:
        if not isinstance(item, dict) or not isinstance(item.get("id"), str) or item["id"] in by_id:
            raise InputError("evidence case IDs must be unique strings")
        by_id[item["id"]] = item
    expected = {case["id"] for case in manifest["cases"]}
    if set(by_id) != expected:
        raise InputError("evidence case IDs do not match the manifest")
    results = [evaluate_case(case, by_id[case["id"]]) for case in manifest["cases"]]
    status = "fail" if any(r["status"] == "fail" for r in results) else "inconclusive" if any(r["status"] == "inconclusive" for r in results) else "pass"
    return {"status": status, "cases": results}


def _show(report: dict, locale: str, as_json: bool) -> None:
    if as_json:
        print(json.dumps(report, ensure_ascii=False, indent=2))
        return
    msg = MESSAGES[locale]
    for case in report["cases"]:
        print(f'{case["id"]}: {msg[case["status"]]}')
        for sample in case["samples"]:
            print(f'  {msg["sample"]} {sample.get("iterations", "?")}: z={sample.get("z_m", "?")} m')
        if case.get("reason"):
            print(f'  {msg["error"]}: {case["reason"]}')
    print(f'{msg["summary"]}: {msg[report["status"]]}')


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description="Gazebo contact response oracle / oracle de contact / oráculo de contacto")
    parser.add_argument("command", choices=["run", "check"])
    parser.add_argument("manifest", type=Path)
    parser.add_argument("evidence", nargs="?", type=Path, help="JSON evidence path for check")
    parser.add_argument("--evidence-out", type=Path, help="write captured JSON evidence")
    parser.add_argument("--locale", choices=MESSAGES, default="en")
    parser.add_argument("--json", action="store_true")
    args = parser.parse_args(argv)
    try:
        manifest = load_manifest(args.manifest)
        if args.command == "run":
            if args.evidence is not None:
                raise InputError("run does not accept an evidence input path")
            if shutil.which("gz") is None:
                print(MESSAGES[args.locale]["missing_gz"], file=sys.stderr)
                return 3
            evidence = {"schema_version": 1, "cases": [capture_case(case) for case in manifest["cases"]]}
            if args.evidence_out:
                args.evidence_out.write_text(json.dumps(evidence, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
        else:
            if args.evidence is None or args.evidence_out is not None:
                raise InputError("check needs an evidence input path and no output path")
            evidence = json.loads(args.evidence.read_text(encoding="utf-8"))
        report = evaluate(manifest, evidence)
    except (InputError, OSError, json.JSONDecodeError) as exc:
        print(f'{MESSAGES[args.locale]["invalid"]}: {exc}', file=sys.stderr)
        return 2
    _show(report, args.locale, args.json)
    return {"pass": 0, "fail": 1, "inconclusive": 3}[report["status"]]


if __name__ == "__main__":
    raise SystemExit(main())
