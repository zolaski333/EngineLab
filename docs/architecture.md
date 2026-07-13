# Architecture native

## Direction des dépendances

```text
foundation
  ├── events
  ├── physics
  ├── ecu
  └── exhaust ──► events
          │
          ▼
      simulation
          │
          ▼
        runtime ── SPSC + atomiques ──► audio (JUCE + JUCE DSP)
          │                                  │
          └──────────── snapshot ────────────┴──► app/UI (JUCE)

foundation ──► diagnostics
foundation ──► serialization (JSON/YAML)
```

JUCE reste confiné à `audio` et `app`. Les lois de gaz, de flamme, d'injection,
de frottement, d'ECU et de simulation restent en C++ standard testable.

## Responsabilités

| Cible | Responsabilité |
|---|---|
| `EngineLabFoundation` | Types, configuration, télémétrie, file SPSC |
| `EngineLabPhysics` | Gaz conservatif, cinématique partagée, travail P·dV, distribution variable, résonance Helmholtz, propagation de flamme, knock end-gas et injection |
| `EngineLabEvents` | Timing et payloads d'allumage |
| `EngineLabEcu` | Commandes carburant/allumage et limiteur |
| `EngineLabExhaust` | Topologie, contre-pression, délai/résonance et routage audio |
| `EngineLabSimulation` | Intégration mécanique, gaz, combustion et thermique |
| `EngineLabRuntime` | Thread 240 Hz, chaîne énergétique embrayage/boîte/pneu/véhicule, banc, snapshots et pont audio |
| `EngineLabAudio` | Voix temps réel, conditionnement et convolution par chemin |
| `EngineLabSerialization` | Codecs JSON/YAML versionnés |
| `EngineLabCatalog` | Presets et parts réutilisables |
| `EngineLabDiagnostics` | Règles explicatives utilisateur |
| `EngineLabApp` | Composition, fichiers audio et rendu desktop |

## Contrats physiques

- `GasCell` possède les espèces, l'énergie interne, le volume et le momentum
  2D. `ConservativeGasSystem` est seul responsable des transferts.
- `FlamePhysicsModel` transforme des conditions thermodynamiques en avancement
  de front sans connaître l'horloge, l'UI ou l'audio.
- `FuelInjectionModel` transforme une commande en carburant mesuré/vaporisé et
  refroidissement de charge ; son film liquide est un état par cylindre.
- `IndicatedWorkModel` intègre la boucle P·dV de chaque cylindre sans intervenir
  dans la dynamique instantanée du vilebrequin.
- `ValveTrainModel` possède la réponse continue VVT/VVL et transforme les
  profils levée/débit en restrictions gazeuses. `HelmholtzRunnerModel` possède
  l'état acoustique agrégé de chaque runner.
- `EngineSimulator` orchestre ces modèles et utilise la pression cylindre comme
  source de couple. Il ne crée aucun thread.
- `EngineRuntime` est l'unique propriétaire du thread de simulation et de
  `DrivelineModel`. Ce dernier possède les états de rapport, embrayage, roue et
  véhicule ; il ne dépend ni de JUCE ni de l'UI.
- `FiringEvent` transporte les impulsions discrètes. `CylinderPressureSample`
  transporte les pressions intra-cycle par une seconde SPSC ; les télémétries
  lentes utilisent des atomiques lock-free.
- `RealtimeConvolutionBank` alloue et prépare hors callback, puis ne fait que du
  traitement borné dans le callback.

## Configuration

YAML et JSON restent les formats canoniques. Les nouveaux paramètres de rail,
film, vaporisation, chaleur latente, refroidissement DI/port, friction
Stribeck, calibration de combustion, courbes levée/débit, VVT/VVL, acoustique
runner et transmission énergétique sont sérialisés et validés. Les anciens fichiers port-injection sans
ces champs reçoivent des valeurs physiques rétrocompatibles ; aucune branche
spécifique à un moteur n'est ajoutée au solveur.

## Extensions futures

Un moteur deux temps, diesel, une dynamique torsionnelle multi-vilebrequin ou un réseau
acoustique 1D doivent arriver sous forme de nouveaux modèles explicites. Ils ne
doivent pas contourner les contrats de conservation, le pont SPSC ou la
validation de configuration.
