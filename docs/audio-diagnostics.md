# Exports de diagnostic audio

Le paquet Windows livre aussi `tools/EngineLabOfflineAudioExporter.exe`. Depuis
la racine extraite, un export directement exploitable s'obtient avec :

```powershell
.\tools\EngineLabOfflineAudioExporter.exe --catalog-root . --engine K20 --output .\exports\k20 --format float32 --stems
```

Quand l'option de stems est activee, l'export offline ecrit :

- `master.wav` ;
- six buses sources (`stem_combustion`, `stem_exhaust_dry`,
  `stem_exhaust_ir`, `stem_intake`, `stem_forced_induction`,
  `stem_mechanical`) ;
- `premaster.wav`, somme des six buses dans l'ordre ci-dessus ;
- `master_processing_delta.wav`, difference entre le master livre et le
  premaster ;
- `engine-order-map.csv`.

Deux sous-stems diagnostiques supplémentaires décomposent le bus sec sans être
ajoutés une seconde fois au prémaster :

- `stem_exhaust_pressure_wave.wav` contient l'onde rayonnée par le réseau
  (blowdown, réflexions et éventuel afterfire) ;
- `stem_exhaust_jet.wav` contient uniquement la turbulence de sortie, après le
  gain de jet auteur.

Leur somme reconstruit `stem_exhaust_dry.wav`. Le manifeste publie l'erreur
float dans `exhaust_dry_decomposition.sum_to_exhaust_dry_max_abs_error`. Cette
séparation permet de décider à l'écoute si un excès d'aigu vient de la source
de pression ou du bruit de jet sans changer le master.

La relation auditable est :

`master = premaster + master_processing_delta`

Le manifeste schema 3 enregistre l'erreur absolue maximale des deux
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

Le bloc `audio_physics` du manifeste distingue configuration et activite
mesuree : COV/correlation auteurs, afterfire active, rupteur humide, minimum et
maximum des multiplicateurs de cycle, nombre d'echantillons de variation,
chaleur afterfire maximale, masse de carburant reellement brulee et nombre de
silencieux poreux. Cela evite de conclure qu'une case cochee a produit un effet
quand la ligne etait froide ou qu'aucun carburant imbrule n'etait disponible.
