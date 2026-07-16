# Architecture

EngineLab sépare les modèles physiques, leur orchestration, le temps réel et la
présentation. Les bibliothèques de calcul restent en C++ standard ; JUCE est
confiné à l'audio et à l'application desktop.

## Modules et dépendances

```text
foundation
  ├── calibration ───────────────┐
  ├── events                     ├──► ecu
  ├── physics                    │
  ├── exhaust ──► events         │
  ├── serialization              │
  ├── diagnostics                │
  └── render                     │
                                 ▼
events + physics + ecu + exhaust ──► simulation
                                        │
                                        ▼
                                     runtime ──► audio (JUCE/DSP)

foundation + serialization ──► scripting
catalog + diagnostics + serialization + scripting
       + calibration + render + runtime + audio ──► app (JUCE)
```

| Cible | Responsabilité |
|---|---|
| `EngineLabFoundation` | Types de configuration et d'état, validation, normalisation et SPSC générique |
| `EngineLabCalibration` | Axes/unités typés, scalaires/courbes/tables, validation, snapshots immuables et JSON ECU |
| `EngineLabEvents` | Événements d'allumage et trames de pression intra-cycle |
| `EngineLabPhysics` | Gaz conservatif, cinématique, P·dV, injection, flamme, knock, distribution et admission Helmholtz |
| `EngineLabEcu` | Interpolation des cartes, enrichissements, corrections et limiteur |
| `EngineLabExhaust` | Compilation de la topologie, routes, propriétés gazeuses agrégées et métadonnées acoustiques |
| `EngineLabSimulation` | Orchestration déterministe des modèles et intégration de l'état ; aucun thread |
| `EngineLabRuntime` | Thread de simulation, transmission/véhicule, banc, snapshots et pont vers l'audio |
| `EngineLabAudio` | Renderer stéréo temps réel, guides d'onde, voix, FDN et convolution par chemin |
| `EngineLabSerialization` | Round-trip JSON/YAML en schéma moteur 2 et migration des fichiers v1 |
| `EngineLabScripting` | Compilation sûre du DSL `.els` et watcher de dépendances en arrière-plan |
| `EngineLabCatalog` | Chargement des moteurs et pièces livrés |
| `EngineLabDiagnostics` | Diagnostics explicatifs à destination de l'UI |
| `EngineLabRender` | Scène 3D indépendante d'une API, snapshots fixes et interpolation |
| `EngineLabApp` | Composition JUCE, fichiers, tuner ECU, concepteur d'échappement, rendu 2D et interactions |

## Propriété des états et threads

`EngineSimulator` possède les états physiques et avance uniquement quand son
appelant invoque `step`. `EngineRuntime` en est le propriétaire applicatif et
fait tourner la simulation sur un `std::jthread`. Il publie un `EngineState`
protégé pour l'UI, des télémétries lentes par atomiques et deux flux SPSC : les
événements discrets et les trames continues de pression.

Le callback JUCE consomme ces flux. Il ne compile aucun script, ne lit aucun
fichier et ne prend aucun mutex. Le décodage des réponses impulsionnelles, les
allocations et la préparation DSP ont lieu avant le rendu.

`EngineScriptHotReloader` possède un worker distinct. Il surveille le script
racine, ses `include` et son éventuel `base`, compile hors du thread UI puis
publie un état immuable. L'UI ne remplace le runtime qu'après une compilation
et une validation complètes.

Le watcher de calibration est plus léger : le timer de la fenêtre ECU détecte
une modification du fichier, parse et valide le document, puis appelle
`CalibrationStore::publish`. Les lecteurs ECU obtiennent le snapshot par
chargement atomique au début d'une trame externe et ne voient jamais un
brouillon partiel. Un epoch par lecteur garde les snapshots remplacés en vie
jusqu'à leur acquittement ; leur destruction a donc lieu sur le thread de
publication, pas sur celui de simulation. Cette stratégie évite un mutex lecteur
mais ne suppose pas que `atomic<shared_ptr>` soit matériellement lock-free.

## Frontières transactionnelles

### Calibration ECU

Un éditeur travaille sur un `CalibrationDraft`, qui peut être temporairement
invalide. `publish` valide l'ensemble et vérifie éventuellement
`expectedRevision`. En cas de succès, un nouveau `CalibrationSnapshot` est
échangé atomiquement. En cas d'erreur ou de conflit, le pointeur actif et sa
révision ne changent pas.

Cette opération ne recrée ni `EngineRuntime`, ni `EngineSimulator`. Elle est
adaptée aux paramètres que l'ECU lit à chaque évaluation : AFR, avance et
rupteur dans l'intégration actuelle.

### Configuration structurelle

JSON, YAML et `.els` produisent un `EngineConfig` complet. Les décodeurs
acceptent les schémas moteur 1 et 2 ; la normalisation migre la v1 en mémoire
vers la v2, et tout nouvel export émet la v2. La normalisation
rend les topologies explicites autoritaires, puis la validation contrôle les
identifiants, plages, affectations de cylindres, fréquence du solveur et graphes
acycliques. Une configuration acceptée sert à construire un nouveau runtime.
L'ancien est arrêté seulement après que son remplacement a pu être créé.

Le remplacement est sûr mais n'est pas une migration d'état : régime,
températures, films de carburant, phases de combustion, transmission et files
audio repartent de l'état initial. Pour les rechargements structurels du même
moteur (script live, éditeur JSON, concepteur d'échappement), le nouveau runtime
réutilise le même `CalibrationStore` : cartes, tuner et watcher ECU survivent.
Le choix ou l'import explicite d'un autre moteur crée son jeu par défaut.

## Contrats physiques

- `GasCell` conserve espèces, énergie interne, volume et quantité de mouvement
  2D. `ConservativeGasSystem` est l'unique point de transfert entre cellules.
- `MechanicalKinematics` fournit les positions, vitesses, accélérations et bras
  de levier, y compris la géométrie maître/articulée ; simulation et scène de
  rendu partagent ainsi la même référence.
- `FlamePhysicsModel` calcule délai, vitesse et progression sans dépendre de
  l'UI ou de l'audio. Sa fermeture géométrique actuelle est un volume effectif
  cylindrique `πr²h`, pas une géométrie 3D de flamme résolue.
- `IndicatedWorkModel` intègre les boucles signées P·dV comme télémétrie. Le
  vilebrequin reçoit le couple instantané issu de la pression, pas un couple
  moyen réinjecté.
- `ExhaustGraph` transforme une configuration en routes bornées. Pour un réseau
  auteur, il compile la gorge d'entrée de chaque cylindre ainsi que le volume,
  la longueur moyenne et la conductance de sortie de chaque chemin ; ces valeurs
  alimentent les `GasCell` et `FlowParameters` agrégés de la simulation. Il ne
  discrétise toutefois pas le réseau composant par composant. Le graphe publie
  aussi par cylindre une transmission, un délai et des métriques combinées pour
  l'audio continu et événementiel. Le renderer conserve ensuite la séparation
  des chemins.

## Préparation du rendu 3D

Le module `render` ne dépend ni de JUCE, ni d'OpenGL. `RenderSnapshotBuilder`
convertit `EngineConfig` et `EngineState` en pièces avec identifiant stable,
parent, transformation XYZ, activité et température. Les stations de cylindres
sont placées sur Z, alors que la cinématique du solveur reste résolue dans son
plan XY. Un tableau fixe borne à 256 le nombre de pièces publiées.

`RenderSnapshotInterpolator` conserve deux trames et interpole les
transformations, dont les angles avec le chemin le plus court.
`IEngineRenderer` définit seulement `prepare`, `resize`, `render` et `release`
avec une caméra et un viewport neutres.

Ce socle rend une intégration OpenGL possible sans modifier la physique, mais
il ne constitue pas encore un renderer. Il reste notamment à :

- écrire le backend et gérer le contexte OpenGL ;
- définir meshes, matériaux, éclairage, shaders et cache de ressources ;
- choisir la synchronisation avec le thread de rendu ;
- brancher la caméra et les interactions sur le backend ;
- étendre la scène aux collecteurs, admissions, accessoires et effets ;
- remplacer ou adapter la vue JUCE 2D, qui lit encore directement l'état de
  simulation malgré la production parallèle d'un `RenderSnapshot`.

## Règle d'extension

Un moteur deux temps ou diesel, un vilebrequin torsionnel, un solveur
acoustique 1D ou un backend OpenGL doivent entrer par un modèle ou une interface
explicite. Ils ne doivent pas contourner la validation, les invariants de
conservation ou les frontières temps réel.
