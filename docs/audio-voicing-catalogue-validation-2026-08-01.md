# Baseline locale et profils de voicing catalogue

Validation du 1er aout 2026, commit de depart `199df2b`, sur la machine Windows
12 threads courante. Aucun chiffre absolu d'une session ou machine precedente
n'est utilise comme reference.

## Budget avant modification

Commande, six passages consecutifs :

```powershell
out/build/windows-vs2022/tools/Release/EngineLabRealtimeBudgetHarness.exe `
  --free-run --rpm 7000 --seconds 6
```

Le brief demande le minimum du temps sur six essais. Comme le harness publie
`simulatedSeconds / wallSeconds`, le minimum du temps correspond au maximum du
facteur. Le pire passage observe est conserve separement pour montrer le jitter.

| Moteur | Workers | facteur temps-min | pire observe | marge temps-min | overruns |
|---|---:|---:|---:|---:|---:|
| K20A I4 | 2 | 1,625 | 1,547 | 38,5 % | 0 |
| 2JZ I6 turbo | 2 | 1,635 | 1,596 | 38,8 % | 0 |
| LS3 V8 | 2 | 1,236 | 1,063 | 19,1 % | 0 |
| EJ25 flat-4 turbo | 2 | 2,084 | 1,855 | 52,0 % | 0 |
| Audi I5 turbo | 2 | 1,836 | 1,589 | 45,5 % | 0 |
| Hayabusa I4 | 2 | 1,912 | 1,869 | 47,7 % | 0 |
| Big Twin V2 | 0 | 3,469 | 3,387 | 71,2 % | 0 |
| Merlin V12 | 3 | 1,187 | 1,086 | 15,8 % | 0 |
| Flat-6 refroidi par air | 2 | 1,428 | 1,336 | 30,0 % | 0 |
| Radial R5 | 2 | 2,609 | 2,520 | 61,7 % | 0 |
| Yamaha CP2 | 0 | 3,092 | 3,015 | 67,7 % | 0 |
| Yamaha CP3 | 2 | 2,367 | 2,280 | 57,8 % | 0 |
| Yamaha CP4 | 2 | 1,933 | 1,877 | 48,3 % | 0 |
| VW EA288 TDI | 2 | 2,513 | 2,417 | 60,2 % | 0 |
| MT-07 Full System | 0 | 3,217 | 3,158 | 68,9 % | 0 |
| Audio Physics Lab | 0 | 3,123 | 3,018 | 68,0 % | 0 |

Les 96 points sont valides et sans overrun. Le Merlin est le goulot avec 15,8 %
de marge sur le temps minimal. Cette table autorise le travail audio mais pas
une depense CPU non bornee. Les alertes de contre-pression 2JZ/EJ25/Audi restent
visibles et ne sont pas masquees.

## Corpus et limite de calibration

`references/real-engine-audio/fetch-corpus.ps1` a reverifie dix fichiers CC0 :
10/10 SHA-256 et decodages passent. Le manifeste reste en schema 2 et ne porte
pas de fenetre auditionnee par condition. Les deux packs ont donc ete rendus
avec `--allow-unmatched-reference-window`; leurs references sont marquees
`WINDOW NOT CONDITION-MATCHED` et aucun verdict de qualite n'en est publie.

Les seize overrides de `voicing/engines/` restent explicitement des profils
`estimatedFamily`. Ils sont assez differents pour tester le produit mais ne sont
pas etiquetes comme calibrations d'un constructeur ou d'un microphone.

## Preuve avant/apres dans le vrai renderer

Les deux packs utilisent la meme trajectoire, la meme graine `20260801`, 48 kHz
et une sonie de -20 LUFS par segment :

- `out/validation/listening-baseline-2026-08-01` avant profils ;
- `out/validation/listening-profiled-2026-08-01` apres profils.

Comparaison echantillon par echantillon des 20 WAV EngineLab, apres nivellement :

- delta RMS minimal `0,001478` ;
- delta RMS moyen `0,006794` ;
- delta RMS maximal `0,015506` ;
- aucun segment bit-identique ;
- mouvement spectral mesure de `-4,04 dB` a `+2,37 dB` selon bande et moteur.

Exemples : LS3 ralenti `-4,04 dB` dans la bande haute, Hayabusa ralenti
`+2,37 dB`, Big Twin rev `+1,48 dB` dans la bande basse. La normalisation par
segment empeche d'expliquer ces deltas par un simple avantage de niveau.

## Garde-fous automatises

```text
EngineLab.AudioVoicing   PASS
EngineLab.AudioWorkshop  PASS
```

Le premier test charge les seize moteurs et exige seize profils effectivement
resolus. Le second clique les deux actions A/B et verifie que les champs caches
sont neutralises puis restaures exactement. L'application Release est egalement
reliee avec les memes sources JUCE.

## Ce que ce lot ne prouve pas

- Une difference mesurable n'est pas une preference humaine.
- Les prises non appariees ne permettent pas de regler une reponse cible.
- Le voicing est une presentation non physique ; la prochaine correction
  structurelle reste la propagation haute bande noeud par noeud dans le graphe
  d'echappement.
