# X-pipe directionnel — implémentation et preuves du 22 août 2026

## Résultat du lot

Le LS3 n'emploie plus un `merge` suivi d'un `splitter` pour prétendre représenter
un X-pipe. Cette écriture formait un plénum commun : après la première jonction,
les deux bancs partageaient le même état acoustique et perdaient définitivement
leur identité. Le schéma moteur 9 possède maintenant un vrai composant compact
à quatre ports appariés, consommé par le graphe, le layout gaz, le guide d'onde,
les sérialisateurs et le concepteur graphique.

Le correctif reste volontairement parcimonieux : aucun CFD 3D, aucune nouvelle
voix audio, aucune allocation dans le callback et aucune ligne à retard cachée
dans l'intersection. Les quatre tubes mesurables autour du X portent les délais ;
le X ne fait que le scattering directionnel et une perte moyenne explicitement
authorée.

Ce document établit des contrats internes et des mesures produit. Il ne prétend
pas valider le son face à un enregistrement réel.

## Défaut reproduit avant correction

Un fixture déterministe injecte une même réaction successivement dans les deux
bancs d'un échappement symétrique et mesure les deux sorties. Avec le pseudo-X
historique `merge -> splitter`, les deux réponses étaient identiques à la
précision flottante :

```text
reaction pseudo-X spatialisation:
  bankA_lr_db=0
  bankB_lr_db=0
  afterX_lr_db=10.2896
  bank_identity_delta_db=-318.813
```

Le problème n'était donc pas un manque de gain ou une nuance subjective : la
topologie supprimait mathématiquement l'information « banc gauche / banc droit ».
Une source locale d'afterfire correctement publiée ne pouvait plus être située
une fois passée par ce nœud commun.

## Modèle retenu

### Contrat auteur

Un `crossover` possède exactement deux entrées et deux sorties. Les connexions
indiquent explicitement `to_port: 0|1` côté entrée et `from_port: 0|1` côté
sortie. Chaque port doit être utilisé une fois.

Le composant lui-même est compact :

- `length_mm = 0` ;
- `volume_l = 0` ;
- `outlet_diameter_mm = 0` ;
- `resonance_hz = 0` ;
- `diameter_mm` décrit les sections qui se croisent ;
- `restriction` est la perte moyenne authorée ;
- `crossover_coupling = k`, entre 0 et 1, règle le couplage croisé.

Les quatre longueurs physiques sont des composants voisins. Cette contrainte
évite de cacher une longueur dans le X ou de compter deux fois une portion de
tube. Une connexion directe à un cylindre, une autre jonction de branche ou un
second crossover est rejetée. Les ports sont auto-attribués par le concepteur
graphique, affichés dans la liste de connexions, puis contrôlés par la validation
centrale.

### Scattering acoustique passif

Les pressions incidentes sont converties en coordonnées normalisées par la
puissance :

```text
q_i = sqrt(Y_i) p_i
t   = sqrt(1 - k²)
```

Dans l'ordre entrée 0, entrée 1, sortie 0, sortie 1 :

```text
q'_e0 =  t q_s0 + k q_s1
q'_e1 = -k q_s0 + t q_s1
q'_s0 =  t q_e0 - k q_e1
q'_s1 =  k q_e0 + t q_e1
```

Cette matrice est réelle, symétrique et orthogonale. Elle est donc réciproque et
conserve exactement `sum(Y p²)`, y compris avec quatre admittances différentes.
`k=0` donne deux conduits appariés indépendants ; la puissance droite vaut
`1-k²` et la puissance croisée `k²`. Il n'existe ni gain arbitraire, ni filtre
correctif, ni sommation à phase identique dans un plénum commun.

Une réaction ayant lieu dans le volume gaz du X est une source interne et
rayonne vers ses quatre ports. Une réaction située dans l'un des quatre tubes
reste attachée à ce tube et conserve donc sa signature spatiale.

Les racines d'admittance `sqrt(Y)` sont calculées à la frontière de bloc depuis
les états gazeux cibles, puis interpolées continûment par échantillon. Le
premier prototype recalculait quatre impédances et quatre racines carrées à
48 kHz ; cette dépense n'ajoutait aucune information puisque la télémétrie gaz
n'est publiée qu'une fois par bloc. La normalisation de puissance reste faite à
chaque échantillon avec les valeurs interpolées, donc la matrice demeure passive
pendant une transition de température au lieu de subir un saut au rythme des
blocs.

### Représentation gaz basse bande

Le solveur conservatif garde une jonction bien mélangée pour l'écoulement moyen,
la contre-pression, les espèces et la réaction basse bande. Sans volume auteur,
son volume numérique est dérivé de deux sections qui se croisent :

```text
V_x = 2 A d
L_CFL,x = V_x / sum(A_port) = d / 2
```

Pour le LS3 à `d=80 mm`, la longueur CFL de l'intersection vaut donc `40 mm`.
Ce volume dérivé n'est pas réintroduit comme compliance acoustique : ce serait
recréer le plénum commun que le composant directionnel remplace.

Une première implémentation avec `V=A d` a été volontairement soumise au gate
produit. Elle a doublé la fréquence de sous-pas, de 26 879 à 53 755 Hz, et fait
chuter le facteur temps réel à `0,758`. Elle a été rejetée. `2 A d` représente
les deux sections, ramène le sous-pas à 26 880 Hz et respecte le budget.

### Métrologie CFL corrigée dans le même lot

Le diagnostic historique ne regardait que la plus petite cellule de conduit.
Il ignorait la borne `V/sum(A_port)` des jonctions, pourtant utilisée par le
solveur. Le layout publie maintenant :

- `minimumCellLengthM()`, diagnostic des conduits seulement ;
- `minimumCflLengthM()`, minimum complet entre `dx` et `V/sum(A_port)`.

La politique de discrétisation temps réel — cible 360 mm, une cellule minimum,
64 cellules par conduit et 1 024 au total — est centralisée dans
`realtimeExhaustFeedbackDiscretisation()`. Le simulateur, l'éditeur et les deux
harnesses lisent la même fonction. Avant cette correction, l'éditeur compilait
son conseil avec la politique générique 20 mm/deux cellules et ne décrivait pas
le coût réel du produit.

Le banc de coût mesure désormais le LS3 livré ainsi :

```text
ducts 16  junctions 3  cells 26  longueur_CFL 40.00 mm
```

## Migration du LS3 livré

L'ancien tronçon commun était :

```text
banc 0 --\
          merge 301 (140 mm) -> splitter 302 (140 mm) -> deux silencieux
banc 1 --/
```

Le schéma 9 livre maintenant :

```text
banc 0 -> tube 211, 140 mm -> port 0 \
                                      X 303 -> port 0 -> tube 311, 140 mm
banc 1 -> tube 212, 140 mm -> port 1 /       -> port 1 -> tube 312, 140 mm
```

Chaque route conserve 280 mm autour de l'intersection. Le coefficient
`k=0,30` est explicitement commenté comme estimation, pas comme calibration
mesurée. Le coefficient de perte total de l'ancien couple de jonctions est
conservé sous la forme `restriction: 0.06` sur le X.

## Compatibilité et persistance

Le schéma courant passe de 8 à 9. JSON, YAML et chargeur de catalogue :

- reconnaissent `type: crossover` ;
- conservent `crossover_coupling` ;
- n'émettent les ports que lorsqu'ils sont présents ;
- conservent le sentinelle interne pour les anciennes arêtes à deux champs ;
- rejettent un port autre que 0 ou 1 avant conversion vers `uint8_t`.

Les schémas 1 à 8 restent lisibles. Comme l'énumération `crossover` est ajoutée
après les types historiques, leurs valeurs internes ne changent pas. Un
composant autre qu'un X ne peut pas porter silencieusement un couplage ou des
ports.

## Oracles déterministes après correction

### Matrice quatre ports

Le test direct vérifie :

- conservation de puissance avec quatre admittances inégales ;
- réciprocité de chaque paire de ports ;
- passage parfaitement séparé à `k=0` ;
- finitude de toutes les sorties.

Le résumé de routes du graphe vérifie exactement, pour `k=0,30`, les parts de
puissance `0,91 / 0,09` pour le banc 0 et `0,09 / 0,91` pour le banc 1. Un
graphe programmatique qui contourne la validation et omet un port retombe sur
un partage générique conservatif, jamais sur une multiplication de puissance.

### Réaction et identité des bancs

```text
reaction X spatialisation:
  legacy_identity_db=-318.813
  directional_identity_db=3.0189
  bankA_lr_db=8.68156
  bankB_lr_db=-7.49687
  afterX_lr_db=25.2891
  legacy_afterX_lr_db=10.2896
```

Le test ne demande pas une couleur sonore arbitraire. Il exige simplement que
deux injections miroir ne deviennent pas le même signal et que leur biais
gauche/droite s'inverse.

### Graphe, layout et sérialisation

Les tests ciblés imposent également :

- deux entrées/deux sorties, ports uniques et compacité du X ;
- rejet d'un port absent, dupliqué, hors plage ou placé sur un autre type ;
- conservation du type, de `k` et des ports dans le graphe compilé ;
- volume gaz `2 A d` et ports conservés dans le layout ;
- distinction entre cellule de conduit `180 mm` et borne de jonction `30 mm`
  sur le fixture dédié ;
- round-trip LS3 complet en JSON et YAML ;
- rejet explicite de `to_port: 256` avant narrowing entier.

Le 22 août, après reconstruction Release :

```text
EngineLab.Exhaust             Passed
EngineLab.GasDynamics         Passed
EngineLab.Core                Passed
EngineLab.RealtimeRegression  Passed
4/4 ciblés, zéro échec
```

## Rendu produit complet, sans référence externe

Le harness de sensibilité utilise le vrai `EngineSimulator`, le vrai runtime,
les queues de pression/réaction et `RealtimeEngineAudio`, à 48 kHz. À 5 000
tr/min commandés, quatre secondes par variante :

| Variante LS3 | RPM final | Forme mix vs X | Forme échappement vs X | CFL | Pression observateur crête | Garde niveau |
|---|---:|---:|---:|---:|---:|---:|
| X directionnel livré | 5 078 | référence | référence | 40,0 mm | 35,0 Pa | 0 |
| deux 4-en-1 sans section commune | 5 067 | 6,32 dB | 7,98 dB | 80,0 mm | 27,6 Pa | 0 |
| tubes indépendants | 5 077 | 8,08 dB | 15,15 dB | 40,9 mm | 28,6 Pa | 0 |

L'AGC reste à 1,000 et aucun échantillon n'est limité. Ces nombres prouvent que
la topologie atteint la sortie du produit et possède une autorité mesurable ;
ils ne disent pas laquelle sonne comme un véhicule réel.

## Budget temps réel frais

Scénario identique avant/après : LS3 à 6 270 tr/min, 48 kHz, blocs de 256,
trois secondes de chauffe, six secondes mesurées, mode libre avec audio.

| État | Facteur | DSP moyen | DSP p99 | Sous-pas échappement | Violations audio |
|---|---:|---:|---:|---:|---:|
| pseudo-X, baseline même session | 1,078 | 34,0 % | 54 % | 26 879 Hz | 0 |
| premier X, avant optimisation | 1,005 / 0,994 | 35,7 / 35,6 % | 56 % | 26 880 Hz | 0 |
| X, admittances calculées au bloc | 1,025 / 1,025 | 35,0 / 34,9 % | 56 % | 26 880 Hz | 0 |

Le premier X a révélé un passage à `0,994×` après échauffement : le fait qu'il
passait encore le seuil 0,97 ne suffisait pas, puisqu'il était sous le temps
réel. Cette version n'a pas été retenue. Le calcul des admittances à la bonne
cadence remonte deux passages consécutifs à `1,025×`. La capacité finale baisse
donc d'environ 4,9 % face au pseudo-X de la baseline même session, tout en
gardant 2,5 % au-dessus du temps réel et 5,5 points au-dessus du gate. Cette
marge reste une contrainte à préserver, pas une réserve disponible. Couple,
puissance et VE restent respectivement à environ 424,58 Nm, 278,85 kW et
0,7941 ; aucune perte de queue, voix volée, frontière invalide, non-finitude,
garde de niveau ou tranche audio hors budget n'est relevée.

Le supplément vient des quatre conduits physiques voisins nécessaires pour
préserver les deux chemins, pas d'une maille audio ajoutée à l'intérieur du X.

## Limites honnêtes

1. `k=0,30` est une estimation. Aucune géométrie d'angle ou de recouvrement ne
   permet encore de le dériver.
2. Le scattering est compact et indépendant de la fréquence. Un X réel devient
   dispersif lorsque ses dimensions ne sont plus petites devant la longueur
   d'onde.
3. Le gaz traite l'intersection comme un volume bien mélangé. Il conserve masse,
   espèces et énergie mais ne résout pas deux jets directionnels en 3D.
4. Le modèle acoustique haute bande est linéaire et plan ; il n'inclut ni modes
   transverses, ni coudes, ni biais complet dû à l'écoulement moyen.
5. Le DAG interdit les boucles ; un H-pipe littéral n'est donc toujours pas
   représentable par une branche transversale bidirectionnelle.
6. L'identité des bancs et l'autorité du graphe sont maintenant démontrées dans
   le simulateur. La fidélité perceptive absolue demanderait des dimensions et
   des enregistrements conditionnés qui ne font pas partie de ce lot.

## Validation finale avant publication

Une première suite a terminé à 42/42 CTest en 729,13 s et a motivé
l'optimisation du callback. Après cette dernière modification :

- reconstruction Release intégrale, application et outils compris : passée ;
- suite autoritaire : **42/42 CTest**, zéro échec, en **727,16 s** ;
- rampes dyno produit CP2 et LS3 : comprises dans ce passage et vertes ;
- `EngineLab.exe` : 8 878 592 octets, SHA-256
  `17BC98CCA3687B7F926CE89BE1493988316CDECCF38BF5FE1BDCFE5543C7D93F` ;
- smoke caché : processus encore vivant après cinq secondes, puis arrêté
  volontairement ;
- budget LS3 final : deux passages à **1,025×**, tous les compteurs audio à
  zéro et gate `--enforce 0.97` passé.
