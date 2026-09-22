# Validation du graphe acoustique d'échappement — 2026-08-01

## Verdict

La haute bande ne réduit plus un échappement auteur à un tube équivalent par
chemin. `AcousticExhaustNetwork` compile le DAG complet produit par
`ExhaustNetworkLayout`. Ce constat repose sur le binaire de test Release, pas
sur la seule lecture du code.

## Preuves exécutables

Commande exécutée sur le commit `6f78c73` :

```powershell
cmake --build out/build/windows-vs2022 --config Release `
  --target EngineLabRealtimeRegressionTests -- /nr:false /m:1
ctest --test-dir out/build/windows-vs2022 -C Release `
  -R "EngineLab.RealtimeRegression" --output-on-failure
```

Résultat : `1/1` test passé, `0` échec, en `3,02 s`.

Ce test regroupe notamment les oracles suivants :

- `branchedAcousticTopologyRegression` compile quatre primaires, un merge avec
  tronc, un splitter avec tronc et deux sorties différentes. Il exige huit
  conduits et deux charges de sortie, puis prouve qu'une source atteint les
  sorties sans divergence ; retirer uniquement les longueurs des deux branches
  fait disparaître exactement deux conduits ;
- `branchTrunkDelayRegression` compare le même collecteur avec et sans tronc de
  400 mm. L'arrivée causale doit se décaler de `L/c` à moins de trois
  échantillons près ;
- `areaStepScatteringRegression` compare les transmissions directes de rapports
  de section 2, 4 et 9 à la solution passive fermée
  `T(m) = 4m/(1+m)^2`, avec une tolérance absolue de 0,05 ;
- `ductMediumRegression` prouve que le délai de chaque conduit suit son propre
  état thermodynamique et non une température unique prise à la soupape ;
- `freeFieldObserverRegression` vérifie la décroissance exacte en `1/r` et la
  directivité avant/arrière d'une terminaison à bride.

## Câblage production vérifié

`RealtimeEngineAudio` construit le réseau depuis `ExhaustGraph`, le prépare hors
callback, lui transmet les états gazeux par conduit à chaque bloc, puis appelle
`process` à chaque échantillon. Chaque sortie possède sa position, son axe, sa
charge de rayonnement et son observateur champ libre. Le bruit de jet est ajouté
à la sortie et ne réinjecte pas d'énergie dans le guide.

## Limites honnêtes

Le réseau haute bande reste un modèle linéaire à ondes planes : pas de modes
transverses, de coudes 3D ni de CFD acoustique. Les sorties rayonnent vers une
paire de microphones commune. Les réflexions haute fréquence ne sont pas
renvoyées dans le cylindre 0D ; le retour physique de basse fréquence appartient
au solveur gazeux non linéaire.

Conclusion pratique : réimplémenter un second « graphe complet » serait une
duplication risquée. Les prochaines améliorations utiles sont la calibration
d'écoute, l'afterfire de décélération conditionné par l'ECU et la validation
mesurée des sources de blowdown/rayonnement.
