# EngineLab

Simulateur moteur/audio natif Windows en C++20 et JUCE. Le projet vise une
illusion sonore crédible pilotée par une physique simplifiée cohérente — pas un
outil de calcul moteur 1:1.

## État actuel

Le squelette produit une application desktop JUCE avec :

- simulation 4 cylindres essence 4 temps sur une thread dédiée à 240 Hz ;
- événements d'allumage `1-3-4-2` transmis par file SPSC sans verrou ;
- physique, ECU, échappement en graphe, diagnostics et sérialisation séparés ;
- callback audio sans allocation et générateur de pulses minimal ;
- visualisation 2D native des pistons, bielles et vilebrequin ;
- import/export métier JSON et YAML ;
- tests du démarrage, de l'ordre d'allumage, de la file temps réel et des codecs.

Cette version ajoute également :

- cinq moteurs de base sélectionnables : I2, I4, I5, V6 et V8 ;
- arbres à cames paramétrés par durée, centre et levée, avec soupapes animées ;
- injection idéalisée (débit d'air, rendement volumétrique et masse injectée) ;
- synthèse audio procédurale stéréo multicouche avec résonance d'échappement ;
- banc freiné automatique, calcul couple/puissance et comparaison colorée des runs ;
- suppression individuelle des anciennes courbes.

Le moteur utilise désormais un bilan de couple explicite (indiqué,
frottement, charge, démarreur et efforts alternatifs), une intégration en
unités SI, un remplissage MAP/VE, des masses d'air/carburant cohérentes, des
cartographies ECU interpolées et un modèle thermique. Le contact seul ne peut
plus lancer un moteur arrêté : le démarreur doit atteindre le régime de
combustion autonome.

Le collecteur d'admission conserve maintenant la masse d'air dans un plénum
paramétrable (`plenum_volume_l`, `throttle_diameter_mm`). Les ratés retirent une
vraie impulsion de couple cylindre par cylindre, l'AFR affiché inclut les
corrections de carburant et la thermique dépend du débit d'énergie par seconde.
Les moteurs en V exposent également leur angle de banc.

L'application expose un éditeur JSON complet, l'import/export JSON/YAML,
l'export CSV enrichi des passages au banc et un historique RPM/MAP/température.
Les événements audio sont placés à l'échantillon et complétés par des couches
admission, mécanique, distribution et démarreur.

Le rendu audio reste entièrement synthétique et temps réel : il est sensiblement
plus riche que le générateur de pulses initial, sans prétendre remplacer une
bibliothèque de prises moteur enregistrées en studio.

## Commandes clavier

- `S` maintenu : démarreur ;
- `W` : quart de gaz ;
- `E` : mi-gaz ;
- `R` : plein gaz ;
- `D` : démarrer ou arrêter le banc de puissance.
- `F1` à `F5` : sélectionner directement les cinq moteurs de base ;
- `Espace` : pause/reprise de la simulation ;
- `1`, `2`, `3` : vitesse de simulation 0,5×, 1× ou 2×.

La barre supérieure permet d'éditer tous les paramètres, d'importer ou
d'exporter un moteur et d'exporter la courbe sélectionnée en CSV.

## Prérequis Windows

- Visual Studio 2022 ou 2026 avec **Desktop development with C++** ;
- CMake 3.24 ou supérieur ;
- Git ;
- Windows 10/11 64 bits.

Les versions de JUCE, nlohmann/json et yaml-cpp sont épinglées dans le
`CMakeLists.txt` racine. CMake les télécharge à la première configuration.

## Compiler

```powershell
cmake -S . -B build -G "Visual Studio 18 2026" -A x64
cmake --build build --config Release --target EngineLabApp
ctest --test-dir build -C Release --output-on-failure
```

Ou avec les presets reproductibles :

```powershell
cmake --preset windows-vs2022
cmake --build --preset windows-release --parallel
ctest --preset windows-release
```

Avec Visual Studio 2022, remplacez simplement le générateur par
`"Visual Studio 17 2022"`.

Validation mémoire instrumentée (MSVC AddressSanitizer) :

```powershell
cmake -S . -B build -DENGINELAB_ENABLE_SANITIZERS=ON
cmake --build build --config Debug --target EngineLabCoreTests
ctest --test-dir build -C Debug --output-on-failure
```

Exécutable attendu :

```text
build/src/app/EngineLabApp_artefacts/Release/EngineLab.exe
```

## Contrats importants

- L'audio ne prend aucun mutex, n'alloue pas et n'appelle jamais l'UI.
- La simulation ne dépend ni de JUCE, ni du pilote audio, ni de l'UI.
- L'UI écrit des commandes atomiques et lit des snapshots ; elle ne calcule pas
  le moteur.
- Chaque nouveau cycle ou carburant arrive comme nouvelle stratégie derrière
  les interfaces existantes, pas comme une cascade de conditions dans le 4T.
- Les fichiers moteur sont versionnés (`schema_version`).

Consultez [docs/architecture.md](docs/architecture.md) et
[docs/realtime-audio.md](docs/realtime-audio.md), ainsi que
[docs/simulation-model.md](docs/simulation-model.md) pour les équations et les
limites du modèle.
