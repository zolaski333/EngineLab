# Audio temps réel

Le mix livré par `EngineRuntime` assemble des renderers en unités SI pour
l’échappement, l’admission et le rayonnement modal du bloc/culasses. La
suralimentation est pilotée par le solveur mais son rendement acoustique reste
semi-empirique. La description physique complète et ses limites se trouvent dans
[thermoacoustic-architecture.md](thermoacoustic-architecture.md).

## Flux simulation → callback

Deux files SPSC séparent les responsabilités :

- `CylinderPressureSample` porte à chaque sous-pas la pression chambre et la
  frontière d’échappement (`p`, `ṁ`, `ρ`, `c`, `CdA`, chemin, validité) ;
- `FiringEvent` porte le phasage combustion, les ratés, le knock et les sources
  non liées à l’échappement physique.

Les débits sont signés en kg/s. Une valeur négative représente une réversion
réelle du réseau vers le cylindre. Les horodatages utilisent le temps de
simulation ; une PLL logicielle lente compense la dérive entre horloge producteur
et horloge de la carte son sans déplacer brutalement les événements.

Lorsqu’un graphe thermoacoustique valide est compilé, il possède la sortie dès le
premier échantillon et propage le silence jusqu’à la première frontière. Si la
télémétrie devient ensuite invalide, les guides physiques se vident
naturellement ; le synthétiseur historique n’apparaît ni au démarrage ni après
une perte de producteur.

## Source physique

Le renderer interpole monotoniquement les trames adaptatives. Une moyenne lente
sépare l’état de fonctionnement des perturbations acoustiques, puis :

```text
Zc = ρc/A
U′ = ṁ′/ρ
p+ = (p′ + ZcU′)/2
p− = (p′ - ZcU′)/2
```

La résistance différentielle de la soupape est dérivée de la loi d’orifice
autour du débit courant ; elle fixe le coefficient de réflexion du port. Les
caractéristiques sont ensuite propagées dans des lignes aller/retour dont les
délais viennent des longueurs physiques et de la célérité locale. Les jonctions
de collecteur emploient les admittances `A/(ρc)` et conservent le croisement
entre cylindres d’un même chemin.

Les paramètres historiques `exhaustPreset`, bruit basse/haute fréquence,
transmission audio du DAG, FDN, jitter, saturation de collecteur et voix de
blowdown ne participent pas à ce calcul. Des tests de régression vérifient cette
invariance.

## Sortie, rayonnement et niveau

Chaque sortie aboutit à une charge causale passive d’ouverture circulaire libre
ou bridée. L’onde réfléchie retourne au collecteur ; l’accélération de vitesse
de volume donne une pression monopolaire de référence. `FreeFieldObserver`
propage ensuite séparément vers les microphones gauche et droit avec distance
exacte, décroissance `1/r`, retard `r/c` et directivité dépendante de `ka`.

Le signal reste en pascals jusqu’à la conversion de monitoring. Puisque les
dBFS décrivent une chaîne électrique/numérique et non une pression universelle,
la pleine échelle SPL du micro/préampli est publiée explicitement dans
`RealtimeAudioState::acousticFullScaleSplDb`. Par défaut :

```text
141,589 Pa crête = 100,237 Pa RMS = 134 dB SPL = 0 dBFS
```

La conversion utilise toujours la référence acoustique de 20 µPa. Modifier
cette calibration simule le gain/la marge de la chaîne de capture ; cela ne
modifie ni la pression physique calculée, ni la propagation, ni le volume
d’écoute. Le pic de pression SI observé et l’activité du leveler sont exposés
séparément aux harnais de validation.

Les positions/axes 3D des sorties et le couple de microphones sont publiés par
le schéma moteur 4. La stéréo provient donc des différences physiques de trajet,
pas d’un panoramique. Une IR stéréo mesurée peut ajouter la pièce ou la cabine en
aval.

## Réponses impulsionnelles

`RealtimeConvolutionBank` conserve jusqu’à huit convolutions partitionnées,
préallouées hors callback. L’application ne charge une IR que si le chemin
déclare explicitement `impulse_response` dans JSON/YAML. Sans fichier déclaré,
le résultat est le champ libre calculé.

Il n’existe plus de sélection automatique par preset, d’IR générique par défaut
ni d’IR synthétisée depuis les longueurs et restrictions. Une IR explicite est
considérée comme une mesure de propagation aval ; elle ne doit pas doubler la
réponse du tube déjà simulée.

L’application rend maintenant ce contrat visible :

- lorsqu’un DAG d’échappement physique est compilé, le sélecteur historique
  `Street / Open / Turbo / Long tube / Moto` est remplacé par
  **GRAPHE PHYSIQUE** et désactivé ; changer le son passe par la géométrie du
  concepteur d’échappement, pas par un preset procédural sans effet ;
- la commande de bruit aigu héritée est signalée comme non applicable et ne
  bouge plus en mode physique ;
- une IR explicitement déclarée mais absente, vide, corrompue ou illisible
  produit une erreur avec le chemin concerné. Le mixer affiche le nombre d’IR
  effectivement chargées. Le champ libre reste utilisable, mais il n’est plus
  un fallback silencieux.

Le décodage hors callback est centralisé dans `ImpulseResponseLoader`. Les tests
ouvrent une IR livrée et refusent explicitement les cas fichier absent et WAV
corrompu.

## Autres couches

`AcousticIntakeNetwork` propage les débits de soupapes dans les runners, le
plénum, le papillon et l’entrée d’air. `StructuralModalRadiator` reçoit forces
gazeuses, inerties et réactions de paliers en SI. La suralimentation emploie
puissance d’arbre, ordres de pales/lobes et débits de wastegate/dump valve ; ses
niveaux absolus restent semi-empiriques tant qu’aucune mesure composant ne les
remplace. Ses sources de bruit sont indépendantes, ses marches de télémétrie
sont reconstruites à cadence audio et le débit total est conservé lorsqu'il se
partage entre turbine et wastegate. Voir les §21–24 du document d’architecture.

## Contrat du callback

`RealtimeEngineAudio::render` :

- n’effectue ni allocation, ni I/O, ni journalisation, ni verrou ;
- respecte exactement `startSample` et `sampleCount` ;
- alloue guides, buffers d’observateur et convolutions dans `prepare()` ;
- dimensionne les lignes depuis la géométrie publiée et compte toute troncature ;
- neutralise les denormals ;
- reste invariant à 48, 96 et 192 kHz pour les constantes physiques testées.

Après le mix, un shelf utilisateur, un bloqueur DC, un filtre de reconstruction,
un leveler de sûreté lent et un limiteur doux suréchantillonné protègent le
périphérique. Ces traitements ne servent pas à créer la signature de
l’échappement. Au voicing normal, le leveler doit rester à gain unité ; ses
compteurs sont observables par les harnais.

## Tests

`EngineLab.RealtimeRegression` couvre notamment :

- passivité et causalité du rayonnement ;
- déterminisme du rendu SI ;
- influence du signe du débit ;
- silence d’une frontière SI stationnaire ;
- invariance aux presets, événements et bruits hérités ;
- dépendance aux longueurs de tailpipe ;
- cohérence de délai à 48 et 96 kHz ;
- exactitude de la conversion SPL ↔ pression ↔ dBFS ;
- absence de réactivation du chemin procédural après verrouillage physique.

`EngineLab.Core` vérifie en plus qu’une frontière SI finie est publiée à chaque
sous-pas mécanique malgré le couplage multirate du réseau non linéaire. Il
refuse aussi l'annulation compresseur/turbine, une discontinuité de puissance à
la frontière de télémétrie et toute duplication de débit
turbine/wastegate. `EngineLab.RealtimeRegression` conserve les tests d'ordre de
pales, de racine de puissance, de silence sans débit et de rayonnement des
valves réellement ouvertes.

La suite transitoire Release est séparée du rendu stationnaire :

- `EngineLab.AudioTransients` rend la chaîne de production pendant un
  démarrage, un ralenti stabilisé, un coup de gaz, le retour au ralenti, un
  lever/reprise de papillon sous boost avec dump valve, puis une entrée dans la
  zone de coupure du rupteur ;
- `EngineLab.AudioShiftTransientNA` et
  `EngineLab.AudioShiftTransientBoosted` rendent un passage de rapport
  clutchless WOT complet, vérifient le verrouillage de l'embrayage et écrivent
  les WAV de preuve ;
- chaque cas refuse les valeurs non finies, les pertes d'événements ou de
  pression, le vol de voix, le fallback physique, le leveler ou le limiteur
  utilisés comme cache-misère et une discontinuité isolée.

Le détecteur de clic compare un pas à la distribution locale des pas pendant
le même événement. Comparer une dump valve active au seul plateau WOT
pré-transitoire est invalide : le bruit de jet large bande attendu augmente
précisément pendant le passage et ferait échouer un signal continu.

## Limites connues

- huit chemins et 32 cylindres maximum ;
- acoustique plane et linéaire pour la bande audio ;
- pas de correction de rayonnement par écoulement moyen ;
- modes structurels estimés lorsque aucune mesure NVH n’est fournie ;
- rendement acoustique de suralimentation encore semi-empirique ;
- corrélation multi-microphone réelle encore à effectuer moteur par moteur.
