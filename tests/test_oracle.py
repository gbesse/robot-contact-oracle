import json
import os
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch
import xml.etree.ElementTree as ET

from robot_contact_oracle import (
    CaptureError,
    evaluate,
    load_manifest,
    main,
    parse_iterations,
    parse_pose_z,
)


ROOT = Path(__file__).resolve().parents[1]


class OracleTests(unittest.TestCase):
    def setUp(self):
        self.manifest = load_manifest(ROOT / "manifest.json")

    def test_reference_worlds_are_valid_xml(self):
        for case in self.manifest["cases"]:
            world = ET.parse(case["world"])
            self.assertEqual(world.getroot().find("world").attrib["name"], case["world_name"])

    def test_passing_and_ignored_collision_evidence(self):
        good = json.loads((ROOT / "fixtures/pass.json").read_text())
        bad = json.loads((ROOT / "fixtures/ignored-collision.json").read_text())
        self.assertEqual(evaluate(self.manifest, good)["status"], "pass")
        report = evaluate(self.manifest, bad)
        self.assertEqual(report["status"], "fail")
        self.assertEqual([case["status"] for case in report["cases"]], ["fail", "pass"])

    def test_missing_observations_are_inconclusive(self):
        evidence = json.loads((ROOT / "fixtures/pass.json").read_text())
        evidence["cases"][0]["samples"].pop()
        self.assertEqual(evaluate(self.manifest, evidence)["status"], "inconclusive")

    def test_observation_must_reach_required_iteration(self):
        evidence = json.loads((ROOT / "fixtures/pass.json").read_text())
        evidence["cases"][0]["samples"][0]["iterations"] = 1000
        self.assertEqual(evaluate(self.manifest, evidence)["status"], "inconclusive")

    def test_gazebo_text_parsing(self):
        self.assertEqual(parse_iterations("sim_time { sec: 2 }\niterations: 2000\n"), 2000)
        pose = 'name: "ball"\nposition {\n  z: 0.1501\n}\norientation { w: 1 }\n'
        self.assertAlmostEqual(parse_pose_z(pose, "ball"), 0.1501)
        with self.assertRaises(CaptureError):
            parse_pose_z(pose, "other")

    def test_runner_protocol_with_fake_gazebo_cli(self):
        with tempfile.TemporaryDirectory() as directory:
            temp = Path(directory)
            gz = temp / "gz"
            gz.write_text(
                "#!/usr/bin/env python3\n"
                "import os, pathlib, sys, time\n"
                "root = pathlib.Path(os.environ['FAKE_GZ_ROOT'])\n"
                "part = os.environ['GZ_PARTITION']\n"
                "marker = root / part\n"
                "if sys.argv[1] == 'sim':\n"
                "    marker.write_text(pathlib.Path(sys.argv[-1]).name)\n"
                "    while True: time.sleep(0.1)\n"
                "if sys.argv[1] == 'topic':\n"
                "    while not marker.exists(): time.sleep(0.01)\n"
                "    topic = sys.argv[-1]\n"
                "    if topic.endswith('/stats'):\n"
                "        counter = root / (part + '.count')\n"
                "        value = int(counter.read_text()) + 500 if counter.exists() else 500\n"
                "        counter.write_text(str(value))\n"
                "        print(f'iterations: {value}')\n"
                "    else:\n"
                "        z = 0.15 if marker.read_text() == 'support.sdf' else -10.0\n"
                "        print(f'name: \\\"ball\\\"\\nposition {{\\n  z: {z}\\n}}')\n"
            )
            gz.chmod(0o755)
            output = temp / "evidence.json"
            env = {"PATH": str(temp) + os.pathsep + os.environ["PATH"], "FAKE_GZ_ROOT": str(temp)}
            with patch.dict(os.environ, env):
                self.assertEqual(main(["run", str(ROOT / "manifest.json"), "--evidence-out", str(output), "--json"]), 0)
            evidence = json.loads(output.read_text())
            self.assertEqual(len(evidence["cases"]), 2)
            self.assertEqual(evaluate(self.manifest, evidence)["status"], "pass")


if __name__ == "__main__":
    unittest.main()
