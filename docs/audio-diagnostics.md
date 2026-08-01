# Exports de diagnostic audio

Quand l'option de stems est activee, l'export offline ecrit :

- `master.wav` ;
- six buses sources (`stem_combustion`, `stem_exhaust_dry`,
  `stem_exhaust_ir`, `stem_intake`, `stem_forced_induction`,
  `stem_mechanical`) ;
- `premaster.wav`, somme des six buses dans l'ordre ci-dessus ;
- `master_processing_delta.wav`, difference entre le master livre et le
  premaster ;
- `engine-order-map.csv`.

La relation auditable est :

`master = premaster + master_processing_delta`

Le manifeste schema 2 enregistre l'erreur absolue maximale des deux
reconstructions dans le domaine float avant encodage WAV. En PCM24, chaque
fichier est quantifie independamment : une reconstruction relue depuis les WAV
peut donc differer de quelques LSB, ce qui est une limite de representation et
non une source audio manquante. Utiliser `float32` pour une analyse numerique
sans cette ambiguite.

La carte d'ordres emploie des fenetres de 1/12 s, recouvrees a 50 %, avec une
fenetre de Hann et une evaluation de Goertzel aux ordres 0.5 a 24 par pas de
0.5. Chaque ligne contient le temps central, le regime moyen mesure, l'ordre,
sa frequence et son niveau dBFS. Le regime vient de la simulation pendant le
rendu ; il n'est pas estime a partir du son.
