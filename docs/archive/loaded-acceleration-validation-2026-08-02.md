# Validation des accélérations chargées — 2 août 2026

## Objet

Ce lot traite les chutes brèves de couple observées à plein gaz sur presque
tout le catalogue, souvent environ 2 000 tr/min avant le rupteur. Il ne change
pas le choix produit `tyre_grip_limit_enabled: false` : l'absence de limite
d'adhérence reste volontaire afin de faciliter les essais moteur sous charge.

La machine de mesure est le nouveau poste fixe i5-11600. Aucun chiffre absolu
historique n'a servi de référence. Les deux mesures de capacité ci-dessous ont
été faites avant et après le lot, sur cette machine et dans la même session.

## Reproduction instrumentée

Le nouveau `EngineLabLoadedAccelerationHarness` démarre un moteur réel du
catalogue, engage directement le rapport demandé, embraye progressivement,
puis maintient le plein gaz jusqu'au rupteur. Son CSV expose notamment :

- couple instantané et moyen de cycle, régime et accélération ;
- glissement/torque d'embrayage et inertie ramenée au vilebrequin ;
- commandes ECU de carburant, allumage et rupteur ;
- AFR, capacité injecteur, probabilités et événements de raté ;
- nombre et phase des commandes d'étincelle et des combustions terminées pour
  chaque cylindre.

Une chute est comptée avant la zone rupteur (`rpm < limite - 300`) lorsque le
couple de cycle tombe sous 55 % de la médiane des cinq cycles précédents puis
remonte au-dessus de 80 % dans les quatre cycles suivants. Une baisse
progressive de la courbe de couple n'est donc pas confondue avec un sursaut.

Commandes de reproduction :

```powershell
out/build/windows-vs2022/tools/Release/EngineLabLoadedAccelerationHarness.exe --gear 2
out/build/windows-vs2022/tools/Release/EngineLabLoadedAccelerationHarness.exe --gear 3
```

## Causes prouvées et corrections

1. Le couplage lent moteur/véhicule réinjectait, une trame plus tard, la
   réaction complète d'un embrayage verrouillé. Cette réaction contient déjà
   le couple nécessaire à accélérer les inerties véhicule/roues ; son retard
   transformait l'ondulation de combustion en alternance de couples opposés.
   Le modèle transmet désormais le couple routier équivalent et l'inertie
   réfléchie séparément. Sa prédiction interne du vilebrequin est actualisée à
   chaque sous-pas mécanique.
2. Sans limite d'adhérence, le pneu restait auparavant un ressort de glissement
   arbitrairement raide. Ce mode est maintenant une contrainte de roulement
   rigide : la masse du véhicule est ramenée en inertie à la roue, sans
   réintroduire une limite de grip.
3. La cible d'avance à l'allumage bougeait à chaque sous-pas. En accélération,
   elle pouvait franchir le vilebrequin entre deux évaluations et supprimer une
   étincelle pendant tout un cycle. La phase est maintenant mémorisée au début
   de la compression, puis consommée exactement une fois.
4. Sur l'Audi I5, l'AFR réel aux cylindres restait sain (12,0 à 13,2) et
   l'injecteur conservait environ 62 % de marge, mais un indicateur transitoire
   d'inventaire carburant ajoutait une seconde pénalité au calcul de raté. Le
   modèle de raté dépend désormais du mélange réellement présent et de sa
   flammabilité ; une vraie sous-alimentation continue de dégrader cet AFR et
   reste donc détectée physiquement.

## Résultat catalogue à rapport fixe

Les Merlin et radial n'ont pas les rapports demandés et sont signalés
`n/a`. Les 14 moteurs routiers ont tous atteint la fenêtre de mesure.
`min couple` est le plus petit rapport à la médiane précédente avant rupteur.

| Moteur | 2e sursauts | 2e min couple | 3e sursauts | 3e min couple |
|---|---:|---:|---:|---:|
| K20A I4 | 0 | 96,0 % | 0 | 94,9 % |
| 2JZ-GTE I6 Turbo | 0 | 89,3 % | 0 | 93,8 % |
| LS3 V8 | 0 | 97,3 % | 0 | 97,8 % |
| EJ25 Flat-4 Turbo | 0 | 89,0 % | 0 | 90,4 % |
| Audi I5 Turbo | 0 | 89,8 % | 0 | 58,7 % |
| Hayabusa I4 | 0 | 94,8 % | 0 | 94,7 % |
| Big Twin V2 | 0 | 93,6 % | 0 | 94,8 % |
| Flat-6 3.6 | 0 | 91,3 % | 0 | 90,7 % |
| Yamaha CP2 | 0 | 96,7 % | 0 | 97,4 % |
| Yamaha CP3 | 0 | 93,9 % | 0 | 94,8 % |
| Yamaha CP4 | 0 | 92,4 % | 0 | 88,1 % |
| VW TDI | 0 | 85,2 % | 0 | 89,1 % |
| CP2 Full System | 0 | 97,4 % | 0 | 97,6 % |
| Audio Physics Lab | 0 | 90,2 % | 0 | 92,9 % |

Il n'y a également aucun fuel cut, spark cut ni glissement d'embrayage dans
les fenêtres pré-rupteur. Les coupures volontaires observées dans la garde des
300 tr/min du rupteur ne sont pas classées comme défaut.

## Budget temps réel A/B local

Commande identique avant et après :

```powershell
out/build/windows-vs2022/tools/Release/EngineLabRealtimeBudgetHarness.exe --free-run --rpm 7000 --seconds 6
```

| Moteur | Avant | Après |
|---|---:|---:|
| K20A I4 | 1,373× | 1,521× |
| 2JZ-GTE I6 Turbo | 1,510× | 1,354× |
| LS3 V8 | 1,132× | 1,122× |
| EJ25 Flat-4 Turbo | 1,983× | 1,844× |
| Audi I5 Turbo | 1,759× | 1,627× |
| Hayabusa I4 | 1,557× | 1,845× |
| Big Twin V2 | 3,328× | 3,399× |
| Merlin V12 | 1,151× | 1,148× |
| Flat-6 3.6 | 1,404× | 1,394× |
| Radial R5 | 2,524× | 2,501× |
| Yamaha CP2 | 2,979× | 3,048× |
| Yamaha CP3 | 2,327× | 2,290× |
| Yamaha CP4 | 1,907× | 1,638× |
| VW TDI | 2,520× | 2,475× |
| CP2 Full System | 3,190× | 3,028× |
| Audio Physics Lab | 3,084× | 3,072× |

Les deux passages font 16/16 sans overrun. Le pire cas reste le LS3 et passe
de 1,132× à 1,122×, soit -0,9 %, bien plus représentatif de l'absence de
régression globale que les variations moteur par moteur de deux runs courts.
Le lot conserve donc environ 10,9 % de temps avant l'échéance sur le pire cas.

## Tests de non-régression

Commande Release ciblée :

```powershell
ctest --test-dir out/build/windows-vs2022 -C Release -R "EngineLab\.(Core|VehicleDynamics|EcuCalibration|AudioShiftTransientNA|AudioShiftTransientBoosted|LoadedAccelerationCP2|LoadedAccelerationAudi)$" --output-on-failure
```

Résultat : **7/7 passés** en 73,91 s. Les deux nouveaux témoins permanents
verrouillent le CP2 en 2e et l'Audi I5 en 3e. Les tests existants confirment en
parallèle le comportement ECU, la dynamique véhicule, les transitoires audio
de changement de rapport et la vraie sous-alimentation pauvre du test cœur.

## Limites explicites

- Le harnais qualifie les chutes brèves pré-rupteur ; il ne prétend pas valider
  la justesse absolue de chaque courbe de couple.
- Plusieurs moteurs publient encore des pas limités par le solveur à haut
  régime. Ils ne coïncident pas avec les sursauts corrigés et constituent une
  télémétrie de résolution distincte à surveiller.
- Les avertissements de contre-pression turbo restent un chantier séparé.
- Le banc utilisateur et le diagnostic AFR diesel sont les étapes suivantes ;
  ce document ne les marque pas terminés.
