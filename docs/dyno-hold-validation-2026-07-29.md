# Validation du maintien au banc — 2026-07-29

## Problème mesuré

Le précédent contrôleur de banc agissait sur une charge normalisée. La moyenne
d'une fenêtre pouvait être proche de la consigne alors que le moteur la
traversait en oscillant. L'instrumentation ajoutée avant la correction a
notamment mesuré les excursions brutes suivantes :

| Point | écart crête-à-crête / consigne | dérive sur la fenêtre | écart-type |
|---|---:|---:|---:|
| Yamaha CP3, 7 000 tr/min | 29,280 % | -12,527 % | 624 tr/min |
| Yamaha CP4, 9 000 tr/min | 16,642 % | +9,411 % | 472 tr/min |
| 2JZ-GTE, 4 000 tr/min | 10,193 % | -6,282 % | 139 tr/min |
| K20A, 7 000 tr/min | 8,012 % | +3,428 % | 179 tr/min |

Ces moyennes n'étaient donc pas des points de couple ou de puissance
exploitables.

## Correction

`DynoAbsorberController` est désormais le contrôleur unique du runtime et des
harnais. Il travaille en couple de freinage unidirectionnel et combine :

- une avance par le couple moyen de cycle, afin de ne pas poursuivre chaque
  explosion ;
- une boucle PI en tr/min et un amortissement par accélération filtrée ;
- une capacité de frein dimensionnée par la cylindrée ;
- une mise en contact progressive ;
- une intégration limitée à la bande de contact. La descente initiale depuis
  le régime libre ne peut donc plus laisser un couple intégral mémorisé sous la
  consigne.

La fenêtre de mesure conserve deux vitesses différentes :

- `rpm_min/max/stddev/drift` décrit le vilebrequin brut, y compris son
  ondulation cyclique ;
- `held_rpm_*` décrit la vitesse d'arbre filtrée vue par le banc.

La moyenne brute doit rester à ±2 % de la consigne. La tenue exige en plus, sur
la seconde complète, une excursion filtrée ≤4 %, une dérive entre demi-fenêtres
≤1 % et un écart-type ≤1,5 %. Le premier point dispose de cinq secondes de
stabilisation ; les suivants sont amorcés à chaud.

## Résultat constructeur

Commande :

```powershell
out/build/windows-vs2022/tools/Release/EngineLabDynoSweepHarness.exe `
  --catalog-root . `
  --reference-file tests/reference-data/catalog-manufacturer-points.csv
```

Après stabilisation, l'ancien réglage CP3 mesurait 116,9 Nm à 7 000 tr/min
(+25,7 %) : l'ancienne moyenne avait masqué une vraie surévaluation. Sa
turbulence de chambre effective a été réancrée localement, sans multiplicateur
de couple et sans modifier les six autres familles.

| Famille | couple simulé / référence | erreur | puissance simulée / référence | erreur | tenue |
|---|---:|---:|---:|---:|---|
| Honda K20A | 208,0 / 206 Nm | +0,988 % | 149,3 / 162 kW | -7,868 % | PASS |
| Toyota 2JZ-GTE | 369,6 / 427 Nm | -13,433 % | 223,3 / 239 kW | -6,578 % | PASS |
| GM LS3 | 597,9 / 575 Nm | +3,989 % | 290,5 / 321 kW | -9,493 % | PASS |
| Yamaha CP2 | 62,3 / 68 Nm | -8,408 % | 60,1 / 54 kW | +11,358 % | PASS |
| Yamaha CP3 | 102,6 / 93 Nm | +10,293 % | 82,4 / 87,5 kW | -5,818 % | PASS |
| Yamaha CP4 | 111,1 / 111 Nm | +0,090 % | 119,4 / 118 kW | +1,216 % | PASS |
| VW EA288 | 348,9 / 320 Nm | +9,027 % | 111,7 / 110 kW | +1,535 % | PASS |

Le gate a ensuite été étendu à cinq variantes dont la géométrie et les chiffres
constructeur sont traçables :

| Famille ajoutée | couple simulé / référence | erreur | puissance simulée / référence | erreur | tenue |
|---|---:|---:|---:|---:|---|
| Subaru EJ257 | 364,8 / 393 Nm | -7,184 % | 198,9 / 231 kW | -13,888 % | PASS |
| Audi EA855 Evo Sport | 437,0 / 500 Nm | -12,604 % | 253,3 / 294 kW | -13,853 % | PASS |
| Suzuki GSX1300R 1999 | 147,2 / 138,2 Nm | +6,487 % | 143,7 / 128,7 kW | +11,690 % | PASS |
| Harley-Davidson 117 Classic | 171,5 / 162,7 Nm | +5,409 % | 81,1 / 73 kW | +11,029 % | PASS |
| Porsche 964 Carrera 2 | 344,0 / 310 Nm | +10,958 % | 159,6 / 184 kW | -13,270 % | PASS |

Les **24/24** points respectent l'enveloppe indépendante de ±15 % et les
**24/24** fenêtres sont déclarées `PASS`. Le log brut de cette exécution est
`out/validation/catalog-reference-24-point-2026-07-29.log`
(artefact local ignoré par Git).

## Tests non vacuitaires

- `EngineLab.CatalogReference` échoue si l'un des 24 points sort de ±15 % **ou** si
  la consigne n'est qu'une moyenne oscillante.
- `EngineLab.IntakeTuning` et `EngineLab.GasExchange` valident maintenant la
  fenêtre filtrée complète et continuent d'imprimer l'ondulation brute.
- `EngineLab.Core` exerce le même contrôleur dans le banc du runtime.

Les anciens chiffres absolus de temps CPU ne sont pas utilisés : cette
correction porte uniquement sur la validité physique du point de mesure.
