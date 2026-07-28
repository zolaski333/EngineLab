# Investigation PMEP échappement — 2026-07-28

Cette note prolonge `validation-2026-07-28.md`. Elle part d'ablations
reproductibles du banc `EngineLabGasExchangeTests`, puis consigne la cause
physique confirmée et promue en production : le collecteur 0-D détruisait toute
la quantité de mouvement dirigée reçue de ses branches.

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

## Cause confirmée : quantité de mouvement annulée dans le collecteur

Le modèle 0-D du collecteur conservait masse et énergie, mais remettait sa
quantité de mouvement à zéro après chaque sous-pas. Il se comportait donc comme
une chambre de mélange stagnante : l'énergie cinétique dirigée des primaires
était intégralement thermalisée avant le tube aval.

La correction fait évoluer la quantité de mouvement selon l'axe du graphe. Pour
chaque ouverture, la pression statique propre au volume est équilibrée par la
paroi non résolue ; seul le résidu
`flux de quantité de mouvement - pression_collecteur × aire` accélère le gaz.
Cette soustraction est essentielle : à pression uniforme et vitesse nulle, elle
garantit exactement zéro force créée.

| I4 à 6 000 tr/min | Collecteur mélangé historique | Collecteur dirigé |
|---|---:|---:|
| \|PMEP\| production | 1,475 bar | **1,046 bar** |
| Pression moyenne échappement | 169,230 kPa | **111,953 kPa** |

Le contrôle `--well-mixed-junctions` conserve l'ancien comportement dans les
trois bancs (`GasExchange`, `ExhaustFlowBench`, `DynoSweepHarness`) et dans le
banc temps réel. Il permet de reproduire l'A/B sans modifier le source.

### Banc stationnaire, géométrie identique

Cas K20, réservoirs cylindres à 135 kPa et 1 050 K, atmosphère à 101,325 kPa,
deux secondes d'établissement :

| Mesure | Mélangé historique | Dirigé |
|---|---:|---:|
| Débit sortie | 0,139 kg/s | **0,295 kg/s** |
| Pression collecteur relative | 3,732 kPa | 15,658 kPa |
| Vitesse collecteur | **0,000 m/s** | **128,851 m/s** |
| `K mesuré / K géométrique` | 5,549 | **1,494** |
| Puissance totale entrante / sortante | 148,855 / 148,855 kW | 312,376 / 312,376 kW |

Les résultats à 0,5 s et 2,0 s sont identiques aux chiffres imprimés. Le bilan
de masse et le bilan d'énergie ferment, le repos à l'ambiante reste invariant,
et un test unitaire interdit désormais de réintroduire une force ou une énergie
au repos.

### Régression PMEP après correction

L'oracle de couplage donne :

| Régime | \|PMEP\| |
|---:|---:|
| 2 000 | 0,256 bar |
| 2 500 | **0,187 bar** |
| 3 000 | 0,276 bar |
| 3 500 | 0,322 bar |
| 4 000 | 0,355 bar |
| 4 500 | 0,471 bar |
| 5 000 | 0,610 bar |
| 5 500 | 0,716 bar |
| 6 000 | **0,990 bar** |
| 6 500 | 0,913 bar |

Le critère intermédiaire à 2 500 tr/min (`≤ 0,350 bar`) est donc réparé. Le
critère haut régime à 6 000 tr/min (`≤ 0,750 bar`) ne l'est pas : l'écart
résiduel est explicite et ne doit pas être maquillé.

## Limite résiduelle soupape/port

Avec le nouveau collecteur et l'oracle, relever seulement le plafond d'aire
effective en fraction de l'aire de tête donne 0,994 bar à 0,51, 0,911 à 0,60,
0,877 à 0,70, puis un plateau à 0,869 pour 0,80 et 1,00. Le plafond constant
n'est donc pas à lui seul la cause du résidu.

Un multiplicateur d'aire global ferait artificiellement passer la cible
(0,789 bar à 1,5×, 0,737 à 1,75×), mais dépasse la géométrie de rideau/tête et
contredit les travaux cités plus haut : le coefficient dépend de la levée, du
rapport de pression et du sens du débit. Il reste un outil de sensibilité,
jamais un réglage de production.

## Effet catalogue et décision

Le pompage corrigé a invalidé une ancienne compensation du CP2 à haut régime.
Sa calibration de turbulence de chambre, locale au moteur, est réancrée à
`0,80`. Le gate constructeur complet repasse **14/14** ; le CP2 mesure
−10,094 % au point de couple et +12,365 % au point de puissance, dans
l'enveloppe ±15 %.

- Promouvoir la quantité de mouvement dirigée pour les collecteurs moteur.
- Conserver le maillage d'échappement de production à 300 mm.
- Ne pas ouvrir artificiellement collecteur, silencieux ou soupapes.
- Ne pas remplacer la frontière atmosphérique actuelle.
- Conserver `--well-mixed-junctions` et les multiplicateurs d'aire comme
  contrôles diagnostiques seulement.
- Prochaine correction physique : une carte d'aire/coefficient de soupape
  dépendant de `L/D`, du rapport de pression et du sens, validée sur un banc de
  débit avant promotion.
