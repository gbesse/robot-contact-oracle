# Robot Contact Oracle

## Français

`robot-contact-oracle` vérifie en boîte noire qu'une collision **produit bien une réponse physique** dans Gazebo Sim. Les deux scènes SDF fournies ne diffèrent que par la collision de la plateforme : dans la première, une sphère doit rester dessus ; dans la seconde, elle doit la traverser. L'outil lance chaque scène sans interface graphique, attend des nombres précis d'itérations, lit la hauteur publiée par Gazebo et rend un verdict utilisable en CI.

Ce contrôle répond au type de régression décrit dans [une collision ignorée sans erreur](https://github.com/gazebosim/gz-sim/issues/4005). Il ne reproduit pas ce grand modèle précis et ne teste pas encore [les collisions fantômes après suppression](https://github.com/gazebosim/gz-sim/issues/3979).

### Démarrer

Installer [Gazebo Harmonic](https://gazebosim.org/docs/harmonic/install/) et Python 3.11 ou plus. Sur une machine sans Gazebo, les fichiers JSON fournis permettent de vérifier l'analyseur.

```sh
python3 -m venv .venv
.venv/bin/pip install -e .
.venv/bin/python -m unittest discover -s tests -v
.venv/bin/robot-contact-oracle check manifest.json fixtures/pass.json --locale fr
.venv/bin/robot-contact-oracle check manifest.json fixtures/ignored-collision.json --locale fr
.venv/bin/robot-contact-oracle run manifest.json --evidence-out evidence.json --locale fr
```

La dernière commande exige `gz` et exécute réellement les mondes SDF. Le fichier `evidence.json` contient les itérations et hauteurs mesurées. `--json` produit un rapport JSON sur stdout. Codes de sortie : `0` réussite, `1` échec physique observé, `2` entrée invalide, `3` résultat indéterminé (Gazebo absent, publication manquante ou délai dépassé).

Pour tester un autre robot ou simulateur Gazebo, ajouter une scène SDF et un cas dans `manifest.json`. Chaque cas indique le monde, le sujet suivi, les itérations à attendre relativement au premier message `/stats`, et une borne minimale ou maximale pour sa hauteur. Les bornes doivent être choisies d'après la géométrie et les tolérances physiques du cas. Une hauteur de repos vérifie une **réponse de support** ; elle ne mesure pas directement les forces de contact et ne prouve pas la sûreté d'un robot réel. Le job GitHub Actions installe Gazebo Harmonic et conserve les observations en artefact.

## English

`robot-contact-oracle` checks, as a black box, whether a collision **produces a physical response** in Gazebo Sim. The two bundled SDF worlds differ only in the platform's collision geometry: in the first, a ball must rest on it; in the second, the ball must pass through. The tool starts each world headlessly, waits for specified simulation iterations, reads the height published by Gazebo, and returns a CI-friendly verdict.

This check addresses the kind of regression reported in [a silently ignored collision](https://github.com/gazebosim/gz-sim/issues/4005). It does not reproduce that particular large model and does not yet test [ghost colliders after removal](https://github.com/gazebosim/gz-sim/issues/3979).

### Quick start

Install [Gazebo Harmonic](https://gazebosim.org/docs/harmonic/install/) and Python 3.11 or newer. On a machine without Gazebo, the bundled JSON files exercise the evaluator.

```sh
python3 -m venv .venv
.venv/bin/pip install -e .
.venv/bin/python -m unittest discover -s tests -v
.venv/bin/robot-contact-oracle check manifest.json fixtures/pass.json --locale en
.venv/bin/robot-contact-oracle check manifest.json fixtures/ignored-collision.json --locale en
.venv/bin/robot-contact-oracle run manifest.json --evidence-out evidence.json --locale en
```

The last command requires `gz` and runs the SDF worlds. `evidence.json` contains measured iterations and heights. `--json` writes a JSON report to stdout. Exit codes: `0` pass, `1` observed physical failure, `2` invalid input, `3` inconclusive result (Gazebo missing, no published sample, or timeout).

To check another robot or Gazebo simulator, add an SDF world and a case to `manifest.json`. Each case names the world, tracked entity, iteration offsets relative to the first `/stats` message, and a lower or upper height bound. Select bounds from the case geometry and physical tolerances. A resting height checks a **support response**; it does not directly measure contact forces or prove real-robot safety. The GitHub Actions job installs Gazebo Harmonic and retains the observations as an artifact.

## Español

`robot-contact-oracle` comprueba como caja negra si una colisión **produce una respuesta física** en Gazebo Sim. Los dos mundos SDF incluidos solo se diferencian en la geometría de colisión de la plataforma: en el primero, una esfera debe quedar apoyada; en el segundo, debe atravesarla. La herramienta inicia cada mundo sin interfaz gráfica, espera determinadas iteraciones de simulación, lee la altura publicada por Gazebo y devuelve un resultado apto para CI.

Esta prueba aborda el tipo de regresión descrita en [una colisión ignorada sin error](https://github.com/gazebosim/gz-sim/issues/4005). No reproduce ese modelo grande concreto y todavía no comprueba [colisiones fantasma tras eliminar entidades](https://github.com/gazebosim/gz-sim/issues/3979).

### Inicio rápido

Instala [Gazebo Harmonic](https://gazebosim.org/docs/harmonic/install/) y Python 3.11 o superior. En un equipo sin Gazebo, los archivos JSON incluidos permiten probar el evaluador.

```sh
python3 -m venv .venv
.venv/bin/pip install -e .
.venv/bin/python -m unittest discover -s tests -v
.venv/bin/robot-contact-oracle check manifest.json fixtures/pass.json --locale es
.venv/bin/robot-contact-oracle check manifest.json fixtures/ignored-collision.json --locale es
.venv/bin/robot-contact-oracle run manifest.json --evidence-out evidence.json --locale es
```

La última orden requiere `gz` y ejecuta los mundos SDF. `evidence.json` contiene las iteraciones y alturas medidas. `--json` escribe un informe JSON en stdout. Códigos de salida: `0` correcto, `1` fallo físico observado, `2` entrada no válida, `3` resultado indeterminado (falta Gazebo, no se publica una muestra o se agota el tiempo).

Para comprobar otro robot o simulador Gazebo, añade un mundo SDF y un caso a `manifest.json`. Cada caso indica el mundo, la entidad observada, los desplazamientos de iteración respecto al primer mensaje `/stats` y un límite inferior o superior para la altura. Elige los límites según la geometría y las tolerancias físicas del caso. La altura en reposo comprueba una **respuesta de apoyo**; no mide directamente las fuerzas de contacto ni demuestra la seguridad de un robot real. El job de GitHub Actions instala Gazebo Harmonic y conserva las observaciones como artefacto.

MIT — [LICENSE](LICENSE).
