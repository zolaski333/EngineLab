# Investigation PMEP échappement — 2026-07-28

Cette note prolonge `validation-2026-07-28.md`. Elle ne change aucun réglage de
production : elle isole les contributions à la PMEP haut régime avec des
ablations reproductibles du banc `EngineLabGasExchangeTests`.

Les nombres sont des comparaisons locales réalisées sur le même binaire et la
même machine. Ils ne sont pas des références de performance CPU.

## Point de départ

Cas : quatre-cylindres par défaut, pleine charge, absorbeur dyno à 6 000 tr/min,
quatre secondes d'établissement puis une seconde de mesure.

| Variante | \|PMEP\| | échappement | admission | pression échappement |
|---|---:|---:|---:|---:|
| Production, couplage 125 µs | 1,475 bar | 1,023 bar | 0,452 bar | 169,230 kPa |
| Oracle, couplage à chaque sous-pas | 1,410 bar | 0,963 bar | 0,447 bar | 172,243 kPa |

L'oracle ne reprend que 0,065 bar, soit 4,4 % du point de production. Le défaut
restant n'est donc pas créé par le sous-échantillonnage du couplage.

Commandes :

```text
EngineLabGasExchangeTests --point-rpm 6000
EngineLabGasExchangeTests --point-rpm 6000 --oracle-coupling
```

## Convergence du maillage physique d'échappement

| Cellule cible | \|PMEP\| à 6 000 tr/min |
|---:|---:|
| 300 mm, production | 1,480 bar |
| 150 mm | 1,537 bar |
| 75 mm | 1,542 bar |

Raffiner ne rapproche pas le résultat de la cible de 0,750 bar. La maille
physique de 300 mm ne crée donc pas la perte par diffusion numérique ; la
raffiner augmenterait le CPU tout en dégradant légèrement le résultat.

```text
EngineLabGasExchangeTests --exhaust-cell-mm 150
EngineLabGasExchangeTests --exhaust-cell-mm 75
```

## Ablation de la géométrie aval

Toutes les lignes suivantes utilisent un seul point à 6 000 tr/min.

| Variante | \|PMEP\| | part échappement | pression échappement |
|---|---:|---:|---:|
| Production | 1,475 | 1,023 | 169,230 kPa |
| Coefficient de sortie 1,0 | 1,461 | — | — |
| Sortie 100 mm et coefficient 1,0 | 1,536 | — | — |
| Restriction silencieux 0 | 1,465 | — | — |
| Collecteur 100 mm | 1,282 | — | — |
| Collecteur 8 L | 1,461 | — | — |
| Tout ouvert ci-dessus | 1,152 | 0,703 | 130,706 kPa |
| Quatre lignes entièrement séparées | 1,413 | 0,961 | 147,957 kPa |
| Quatre lignes séparées et tout ouvert | 1,146 | 0,698 | 125,479 kPa |

Le diamètre/mélange du collecteur est le seul scalaire sensible, mais même une
géométrie volontairement irréaliste ne suffit pas. Ni le silencieux, ni le
coefficient de sortie, ni le volume de collecteur, ni la jonction quatre-vers-un
ne sont à eux seuls la cause racine.

Exemples :

```text
EngineLabGasExchangeTests --point-rpm 6000 --collector-mm 100
EngineLabGasExchangeTests --point-rpm 6000 --independent-paths
EngineLabGasExchangeTests --point-rpm 6000 --outlet-mm 100 --outlet-cd 1 \
  --muffler-restriction 0 --collector-mm 100 --collector-litres 8
```

## Plancher soupape/timing et contribution du réseau

`--ideal-exhaust-reservoir` remet volontairement le réseau à l'ambiante avant
chaque couplage. Ce n'est pas un modèle utilisable ; c'est une borne inférieure
qui supprime toute mémoire de pression aval.

| Variante oracle | \|PMEP\| | échappement | admission | pression échappement |
|---|---:|---:|---:|---:|
| Réservoir idéal, aire 1× | 0,965 | 0,520 | 0,444 | 112,336 kPa |
| Réservoir idéal, aire échappement 2× | 0,590 | 0,167 | 0,423 | — |
| Réservoir idéal, aires admission et échappement 2× | 0,643 | 0,172 | 0,471 | — |
| Production, aire échappement 6× | 1,135 | 0,709 | 0,426 | 176,783 kPa |

À 6× sous réservoir idéal, l'invariant de piégeage sort de sa bande
(`VE=1,230`, `deliveredVE=1,173`, ratio `1,049`) : ce résultat est rejeté.

Deux contributions indépendantes sont prouvées :

1. la mémoire de pression du réseau vaut environ un demi-bar sur ce cas ;
2. une limite de capacité effective soupape/port demeure même avec un aval
   artificiellement idéal.

Cela ne justifie pas de doubler l'aire en production. Les travaux expérimentaux
sur les soupapes à clapet montrent que le coefficient de débit dépend au
minimum de la levée, du rapport de pression et du sens de l'écoulement :

- [SAE 962527](https://doi.org/10.4271/962527), *Relationship Between
  Discharge Coefficients and Accuracy of Engine Simulation* ;
- [SAE 2001-01-1798](https://doi.org/10.4271/2001-01-1798), *Maps of Discharge
  Coefficients for Valves, Ports and Throttles* ;
- [SAE 2019-01-0041](https://doi.org/10.4271/2019-01-0041), *Valve Flow
  Coefficients under Engine Operation Conditions: Pressure Ratios, Pressure and
  Temperature Levels*.

Un multiplicateur global peut diagnostiquer la sensibilité, mais ne peut pas
remplacer une carte de coefficient défendable.

```text
EngineLabGasExchangeTests --point-rpm 6000 --oracle-coupling \
  --ideal-exhaust-reservoir
EngineLabGasExchangeTests --point-rpm 6000 --oracle-coupling \
  --ideal-exhaust-reservoir --exhaust-valve-area-x 2
```

## Frontière atmosphérique réfutée

Une version expérimentale a utilisé directement le flux physique de l'état
caractéristique au lieu de le repasser dans le solveur de Riemann :

| Formulation | \|PMEP\| | pression échappement |
|---|---:|---:|
| Fantôme caractéristique de production | 1,475 bar | 169,230 kPa |
| Flux caractéristique direct | 1,476 bar | 170,287 kPa |

La modification n'apporte rien et a été retirée avant commit. La frontière
atmosphérique n'est pas le prochain levier.

## Décision

- Conserver le maillage d'échappement de production à 300 mm.
- Ne pas ouvrir artificiellement collecteur, silencieux ou soupapes.
- Ne pas remplacer la frontière atmosphérique actuelle.
- Garder les options ci-dessus strictement diagnostiques et absentes par
  défaut.
- Prochaine expérience : remplacer le plafond constant
  `maximumHeadAreaFraction` par une carte d'aire/coefficient dépendant de
  `L/D` et du rapport de pression, puis la confronter à un banc de débit
  automatisé avant toute promotion en production.
