# Socle, corrections et validation

Le nom de ce fichier est conservé pour les liens historiques. Il décrit le
socle de fiabilité actuel, pas une promesse de « phase terminée » indépendante
des tests.

## Corrections structurelles intégrées

La normalisation de configuration considère maintenant les topologies
explicites comme autoritaires. Les anciennes propriétés globales ne doivent pas
écraser silencieusement crankshafts, chemins d'admission ou chemins
d'échappement déjà décrits. Les presets V8 et flat-six ont été remis en
cohérence avec leurs journaux, offsets et volumes de chemin.

La cinématique partage une référence calculée et couvre les bielles
conventionnelles, maîtresses et articulées. Les positions de PMH et volumes de
chambre tiennent compte de la géométrie réellement configurée. Les frottements
à très basse vitesse conservent un sens résistant.

Le réseau gazeux utilise :

- une pression dynamique directionnelle signée ;
- les sections caractéristiques réelles des volumes et restrictions ;
- un transfert borné par l'équilibre de pression ;
- une résolution simultanée des flux lors du croisement de soupapes ;
- la conservation contrôlée des espèces, de l'énergie et des deux composantes
  de quantité de mouvement.

Les événements et trames audio sont produits aux vrais sous-pas, avec leur
temps de simulation. Le reset vide les échantillons anciens. La télémétrie par
cycle provient des franchissements de cycle observés, pas d'une estimation
figée sur la durée d'un appel UI.

Le runtime et l'audio ont aussi été durcis sur les files bornées, l'horloge
producteur/consommateur, la publication du rapport engagé, la fin d'un banc qui
ne converge pas et les changements de réponses impulsionnelles entre threads.
Le signal continu reste associé à son chemin d'échappement et les constantes
DSP suivent la fréquence d'échantillonnage réelle.

Enfin, le harnais de comparaison active explicitement la production de trames
de pression. Ses WAV ne peuvent plus réussir en étant entièrement silencieux.

## Suite CTest

Après une configuration avec les tests et harnais activés par défaut :

```powershell
cmake --build build --config Release
ctest --test-dir build -C Release --output-on-failure
```

Les tests enregistrés sont :

| Test | Objet principal |
|---|---|
| `EngineLab.Core` | intégration simulation, gaz, combustion, transmission, sérialisation, catalogue et audio de base |
| `EngineLab.PhysicsRegression` | topologie autoritaire, presets, pression dynamique signée, flux et cinématique |
| `EngineLab.Calibration` | types, interpolation, validation, transactions, concurrence, JSON et watcher fichier |
| `EngineLab.EcuCalibration` | application immédiate des tables et du rupteur par l'ECU |
| `EngineLab.Exhaust` | validation du DAG, pertes série/parallèle, propriétés de débit agrégées, routes, migration et round-trips JSON/YAML |
| `EngineLab.ExhaustSimulation` | effet du K auteur sur pression/débit/couple et indépendance vis-à-vis de la géométrie legacy |
| `EngineLab.Scripting` | unités, diagnostics, base/include, cycles et conservation du dernier script valide |
| `EngineLab.RenderSnapshot` | scène bornée, layout 3D et interpolation d'angle |
| `EngineLab.RealtimeRegression` | timing, fréquence audio, buffers, isolation des chemins et transmission du DAG |
| `EngineLab.CatalogPhysics` | scénarios déterministes et paliers chargés régulés par frein PI sur tout le catalogue |
| `EngineLab.AudioRender` | rendus multi-moteurs à régime normalisé, finitude, dynamique, plafond du limiteur et stabilité longue |

`EngineLab.AudioRender` émet seulement un avertissement lorsque les signatures
spectrales sont trop proches ; ce seuil n'est pas encore une cause d'échec.
Cette nuance est volontairement documentée : le test empêche les sorties
invalides ou écrasées, mais ne prouve pas une bonne différenciation perceptuelle.

## Harnais de comparaison

Le harnais déterministe produit, pour I4, V8, V-twin et radial :

- une trace CSV par scénario ;
- un WAV mono dérivé des pressions intra-cycle ;
- `summary.csv` avec régime, couple, AFR, carburant, pression, IMEP,
  résonance runner, RMS et crête audio ;
- une porte de fonctionnement sur tous les moteurs du catalogue.

```powershell
cmake --build build --config Release --target EngineLabComparisonHarness
build\tools\Release\EngineLabComparisonHarness.exe `
  --output comparison-output `
  --catalog-root .
```

Le WAV doit avoir une énergie et une crête non nulles. Les seuils catalogue
contrôlent démarrage/régime, suivi AFR et FMEP sur un palier chargé à 65 %
du rupteur, ainsi que le spool d'un turbo configuré.

Pour comparer à un résultat EngineLab de référence :

```powershell
build\tools\Release\EngineLabComparisonHarness.exe `
  --output comparison-candidate `
  --catalog-root . `
  --reference comparison-baseline\summary.csv
```

Les tolérances sont relatives : 5 % pour régime moyen et AFR, 2 % pour la
fréquence de résonance, 10 % pour couple, carburant, pression et IMEP. Les
métriques WAV sont enregistrées et vérifiées non nulles, mais ne participent
pas encore à cette comparaison numérique de référence.

Le renderer JUCE complet peut être lancé séparément :

```powershell
cmake --build build --config Release --target EngineLabAudioRenderHarness
build\tools\Release\EngineLabAudioRenderHarness.exe --output audio-render-output
```

## Sanitizers et analyse statique

Sur une arborescence dédiée :

```powershell
cmake -S . -B build-asan -G "Visual Studio 17 2022" -A x64 `
  -DENGINELAB_ENABLE_SANITIZERS=ON
cmake --build build-asan --config Debug
ctest --test-dir build-asan -C Debug --output-on-failure
```

MSVC active AddressSanitizer ; les toolchains GCC/Clang activent AddressSanitizer
et UndefinedBehaviorSanitizer. Une autre arborescence peut activer
`-DENGINELAB_ENABLE_CLANG_TIDY=ON` si `clang-tidy` est disponible.

Les builds sanitizers sont plus lents, surtout `EngineLab.Core` et les harnais.
Une absence d'erreur sur une exécution ne remplace pas l'analyse de race dédiée
ni une campagne longue sur plusieurs périphériques audio.

## Ce que ces validations démontrent — et ne démontrent pas

Elles démontrent des invariants numériques, des round-trips, des transactions
atomiques, des sorties bornées et des tendances attendues sur les scénarios
livrés. Elles rendent les régressions visibles.

Elles ne démontrent pas l'exactitude d'une courbe de couple réelle, la fidélité
d'un échappement particulier, la stabilité sur toutes les configurations
possibles ou une parité perceptuelle avec ES2D. Ces affirmations nécessitent des
données externes, des protocoles A/B et des références physiques absentes du
dépôt actuel.
