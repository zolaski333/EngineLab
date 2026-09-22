# Baseline audio-first bout en bout — 2026-07-29

Cette mesure complète le banc de capacité simulation-only. Elle exécute en
concurrence :

- le vrai thread `EngineRuntime` à 240 Hz ;
- les vraies files SPSC d'événements et de pression cylindre ;
- `RealtimeEngineAudio`, avec le graphe d'échappement, l'admission acoustique,
  le rayonnement structurel, la suralimentation et les IR explicitement
  configurées ;
- un consommateur de blocs de 256 échantillons à 48 kHz.

La mesure ne remplace pas une carte son Windows. Elle évite volontairement de
dépendre du périphérique sélectionné sur la machine de test. En mode
`--free-run`, le consommateur suit le temps simulé accéléré : il rend exactement
le nombre de blocs correspondant aux fenêtres physiques produites, tout en
mesurant chaque durée de callback contre l'échéance réelle de 5,333 ms.

## Modification de l'instrument

`EngineLabRealtimeBudgetHarness --with-audio` rend un point invalide si la
fenêtre mesurée contient :

- un rendu audio plus long qu'un bloc ;
- une perte d'événement ou de pression ;
- une frontière SI invalide ;
- un échantillon du chemin procédural historique ;
- une valeur non finie ;
- une intervention du leveler de sûreté.

Le p99 imprimé est le pourcentage de l'échéance occupé par le callback.

Le gate d'acquisition du régime utilise désormais une fenêtre complète de
1,5 seconde simulée, avec les mêmes critères que le banc dyno validé :

- moyenne dans ±2 % de la cible ;
- excursion ≤4 % ;
- dérive entre demi-fenêtres ≤1 % ;
- écart-type ≤1,5 % de la cible.

L'ancien gate exigeait qu'aucun poll ne sorte jamais de ±2 %. Il refusait K20,
LS3, Merlin et Flat-6 alors que leur moyenne était sur la cible, parce qu'un
poll pouvait échantillonner le ripple d'allumage. La nouvelle fenêtre refuse
toujours une rampe ou un régime oscillant ; elle ne confond plus ripple cyclique
et absence de maintien.

## Commande

```powershell
out/build/windows-vs2022/tools/Release/EngineLabRealtimeBudgetHarness.exe `
  --catalog-root . `
  --free-run --rpm 7000 --warmup 3 --seconds 6 `
  --with-audio --audio-rate 48000 --audio-block 256
```

Les valeurs ci-dessous sont une baseline locale, pas une référence réutilisable
pour une future optimisation. Toute comparaison de code devra reconstruire les
deux variantes et les alterner dans la même heure.

## Catalogue

| Moteur | Cible tr/min | Facteur capacité complet | Callback p99 | Pertes/fallback/leveler | État |
|---|---:|---:|---:|---:|---|
| K20A-like I4 | 7000 | 1,516× | 32 % | 0 | valide |
| 2JZ-GTE-like I6 Turbo | 6650 | 1,537× | 43 % | 0 | valide |
| LS3-like V8 | 6270 | 1,172× | 53 % | 0 | valide |
| EJ25-like Flat-4 Turbo | 6460 | 1,927× | 40 % | 0 | valide |
| Audi I5-like Turbo | 6745 | 1,722× | 40 % | 0 | valide |
| Hayabusa-like I4 | 7000 | 1,785× | 32 % | 0 | valide |
| Big Twin-like V2 | 5320 | 3,324× | 27 % | 0 | valide |
| Merlin-like V12 | 3040 | **1,080×** | **67 %** | 0 | valide, pire cas |
| Aircooled-like Flat-6 | 7000 | 1,339× | 45 % | 0 | valide |
| Radial-like R5 | 2280 | 2,404× | 32 % | 0 | valide |
| Yamaha CP2 | 7000 | 2,945× | 23 % | 0 | valide |
| Yamaha CP3 | 7000 | 2,198× | 27 % | 0 | valide |
| Yamaha CP4 | 7000 | 1,825× | 31 % | 0 | valide |
| VW EA288-like TDI | 4750 | 2,337× | 35 % | 0 | valide au passage répété |

Le premier passage plein du VW a observé un unique rendu supérieur à 5,333 ms,
sans perte de file ni autre anomalie, sur environ 1 125 blocs. La répétition
isolée de même durée a produit zéro dépassement ; sa ligne ci-dessus vient de
ce passage. Les sorties brutes sont :

- `out/validation/realtime-audio-baseline-2026-07-29-v2.txt` ;
- `out/validation/realtime-audio-baseline-ea288-retry-2026-07-29.txt`.

Ces fichiers restent des artefacts locaux ignorés par Git.

## Conclusion de décision

Le catalogue passe la capacité complète simulation + audio. Le coût du renderer
ne justifie donc pas de rouvrir immédiatement l'admission 1D.

Le Merlin ne possède cependant plus la marge de 14 % observée par le banc
simulation-only : 1,080× correspond à environ 7,4 % de marge sur l'échéance.
Les futurs ajouts audio devront être mesurés sur ce témoin. Une nouvelle couche
qui le fait tomber sous 1,0 sera refusée, déplacée vers le rendu HQ hors ligne
ou financée par une optimisation mesurée.

La prochaine étape n'est pas une optimisation CPU spéculative. Elle est la
séparation diagnostique des stems et l'écoute A/B, sans modifier le master.
