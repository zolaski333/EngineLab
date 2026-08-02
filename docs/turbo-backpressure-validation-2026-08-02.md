# Validation turbine / wastegate et diagnostic de pression — 2026-08-02

## Problème reproduit

Le modèle calculait déjà la restriction d'échappement avec deux passages en
parallèle, turbine et wastegate. L'audio partageait aussi le débit selon leurs
aires effectives. En revanche, la puissance de turbine recevait encore le débit
d'échappement total, puis appliquait un coefficient empirique
`1 - ouverture_wastegate * 0,94`.

Cette opération n'était ni conservative ni cohérente avec la géométrie. Avec
une turbine de 700 mm², une wastegate de 520 mm² et une ouverture de 75 %, la
part physique de turbine vaut 64,2 %, alors que l'ancien coefficient n'en
retenait que 29,5 %. Le solveur devait alors créer une pression motrice
artificielle pour fournir la puissance demandée.

## Correction

`partitionTurboExhaustFlow()` est désormais l'unique partage conservatif :

- aire turbine = aire configurée ;
- aire wastegate = aire configurée × ouverture ;
- débit de chaque branche proportionnel à son aire effective ;
- somme turbine + wastegate exactement égale au débit mesuré.

Le même partage alimente la puissance d'axe et l'aéroacoustique. Le coefficient
empirique a été supprimé.

Le diagnostic a été séparé du cas atmosphérique :

- atmosphérique : moyenne supérieure à l'ambiante de plus de 40 kPa ;
- turbo après la zone de spool : rapport pression motrice / MAP supérieur à
  2,0 ;
- avant spool : pas de verrouillage sur un pic transitoire.

Le seuil 2,0 est une limite conservatrice de diagnostic EngineLab, pas une loi
universelle. Les moteurs stock mesurés ci-dessous sont entre 1,11 et 1,39.

## A/B chargé, même machine et même session

Protocole : `EngineLabLoadedAccelerationHarness`, troisième rapport, plein gaz,
passages automatiques et limiteur d'adhérence désactivés. Moyenne prise dans les
500 derniers tr/min avant le rupteur en excluant les échantillons de coupure.

| Moteur | Pression avant | Pression après | MAP après | Rapport après | Couple moyen avant/après |
|---|---:|---:|---:|---:|---:|
| 2JZ 3.0 I6 | 279,7 kPa | 267,9 kPa | 192,4 kPa | 1,39 | 242,9 / 240,7 Nm |
| EJ25 2.5 F4 | 253,2 kPa | 231,6 kPa | 209,2 kPa | 1,11 | 249,3 / 237,7 Nm |
| Audi 2.5 I5 | 300,5 kPa | 278,5 kPa | 247,6 kPa | 1,12 | 269,8 / 287,7 Nm |
| EA288 2.0 TDI | 280,4 kPa | 292,1 kPa | 226,6 kPa | 1,29 | 179,6 / 178,1 Nm |

Les trois essence baissent de 11,8 à 22,0 kPa. Le TDI augmente de 11,7 kPa à
cause du nouvel équilibre axe/wastegate, mais reste à un rapport sain de 1,29 et
conserve ses références constructeur. La correction n'est donc pas présentée
comme une baisse forcée sur tous les moteurs : elle rétablit d'abord la
conservation de masse et laisse le solveur trouver son équilibre.

## Références constructeur, même fenêtre A/B

| Point | Avant | Après | Résultat après |
|---|---:|---:|---:|
| 2JZ couple 4 000 tr/min | -13,519 % | -10,498 % | PASS |
| 2JZ puissance 5 600 tr/min | -5,227 % | -3,672 % | PASS |
| EJ25 couple 4 000 tr/min | -7,486 % | -5,873 % | PASS |
| EJ25 puissance 6 000 tr/min | -4,078 % | -0,202 % | PASS |
| Audi couple 4 000 tr/min | -10,857 % | -9,567 % | PASS |
| Audi puissance 5 600 tr/min | -12,120 % | -8,257 % | PASS |
| TDI couple 2 000 tr/min | +9,027 % | +8,224 % | PASS |
| TDI puissance 4 000 tr/min | +1,535 % | -0,688 % | PASS |

Tous ces points restent dans l'enveloppe ±15 %.

## Tests automatiques

`EngineLab.VehicleDynamics` vérifie désormais : wastegate fermée, partage par
aires, conservation exacte de masse, absence d'alerte à 265/190 kPa, alerte à
385/190 kPa et absence de verrouillage avant spool.

`EngineLab.RealtimeRegression` protège le chemin audio qui utilise le même
partage. Les deux tests passent en Release : 2/2 en 3,02 s.

## Interprétation utilisateur

Un gros tube après le turbo réduit les pertes du cat-back, mais ne supprime pas
la pression nécessaire en amont de la turbine. Garrett décrit explicitement la
wastegate comme un contournement qui augmente la capacité de débit de la
turbine et limite sa vitesse ; son guide sur les carters relie aussi une petite
section A/R à davantage de contre-pression à haut régime. BorgWarner dimensionne
de même la wastegate par aire de débit et rapport d'expansion turbine.

Sources :

- https://www.garrettmotion.com/news/newsroom/article/turbo-tech-turbine-housings-101-garrett-performance/
- https://www.garrettmotion.com/knowledge-center-category/racing-and-performance/what-does-an-external-wastegate-do/
- https://www.borgwarner.com/docs/default-source/iam/boosting-technologies/efr_technical_training_book.pdf

