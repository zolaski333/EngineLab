# Audio temps réel

## Règles du callback

`RealtimeEngineAudio::render` n'effectue aucune allocation, attente,
entrée/sortie, journalisation ou prise de mutex. Les voix et le tampon de
planification sont fixes ; les événements arrivent par
`SpscQueue<FiringEvent, 2048>`.

Les événements conservent leur temps interpolé dans le pas de simulation. Le
renderer applique 20 ms de latence contrôlée et les déclenche au bon
échantillon, même lorsque la simulation et le périphérique utilisent des blocs
différents.

Chaque allumage produit une couche bloc moteur immédiate et une couche
échappement retardée par le chemin complet port-collecteur-silencieux-sortie.
Le délai de réflexion provient lui aussi de cette géométrie. Les couches
continues suivent le facteur de vitesse de simulation afin de rester accordées
aux événements.

```text
FiringEvent ──► combustion ──► délai/résonance échappement
       RealtimeAudioState ──► admission + mécanique + distribution + démarreur
```

Les coefficients de filtre et retards sont convertis depuis des fréquences ou
durées lors de `prepare`, donc restent cohérents à 44,1, 48 ou 96 kHz. Les
phases sont repliées pour éviter une perte de précision après une longue
session.

Les assets futurs devront être décodés et ré-échantillonnés hors callback. Les
changements de graphe utiliseront des snapshots immuables échangés atomiquement
avec crossfade. Les compteurs événements perdus, événements tardifs, overruns
simulation, voix volées et saturation du tampon de planification rendent les
régressions observables.
