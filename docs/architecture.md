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
        runtime ──SPSC + atomiques──► audio (JUCE)
          │                 │
          └──── snapshot ───┴──► app/UI (JUCE)

foundation ──► diagnostics
foundation ──► serialization (JSON/YAML)
```

Les flèches indiquent les dépendances de compilation. `foundation` à
`simulation` sont du C++ standard. JUCE est confiné à `audio` et `app`.

## Modules

| Cible | Responsabilité | Interdit |
|---|---|---|
| `EngineLabFoundation` | types, configuration, file SPSC | JUCE, UI |
| `EngineLabEvents` | timing et payloads d'allumage | audio, thread |
| `EngineLabPhysics` | combustion et contraintes simplifiées | horloge, UI |
| `EngineLabEcu` | commandes carburant/allumage | audio |
| `EngineLabExhaust` | graphe et contre-pression | rendu UI |
| `EngineLabSimulation` | intégration de l'état | création de thread |
| `EngineLabRuntime` | thread 240 Hz, snapshots, transport | DSP |
| `EngineLabAudio` | consommation RT et synthèse | verrou, allocation |
| `EngineLabSerialization` | codecs versionnés JSON/YAML | état live |
| `EngineLabDiagnostics` | explications utilisateur | callback audio |
| `EngineLabApp` | composition et rendu desktop 2D | lois moteur |

## Extension 2T et diesel

Le type `EngineCycle` annonce les formats possibles, mais le modèle initial
refuse explicitement autre chose que `fourStroke + gasoline`. Un 2T ajoutera un
`TwoStrokeEventGenerator` (cycle 360°) et sa physique. Un diesel ajoutera une
physique de combustion et un modèle ECU adaptés. Le runtime et l'audio
continueront à consommer le même `FiringEvent`.

## Composition

`EngineRuntime` est la racine de composition actuelle : il construit les
stratégies concrètes et les injecte dans `EngineSimulator`. Quand plusieurs
familles seront disponibles, une fabrique sélectionnera cette composition à
partir du fichier moteur sans déplacer cette décision dans l'UI.

## Données

`EngineConfig` est une donnée de conception, chargée hors temps réel et validée
avant la création du runtime. `RealtimeAudioState` ne contient que cinq valeurs
atomiques triviales ; il transporte les couches continues sans donner au
callback accès au snapshot verrouillé de l'UI.
`EngineControls` contient les commandes utilisateur. `EngineState` est le
snapshot observable. `FiringEvent` est petit, trivialement copiable et constitue
le contrat stable entre simulation, audio, échappement et visualisations.
