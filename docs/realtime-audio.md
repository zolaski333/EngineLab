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

## Stems de diagnostic

`RealtimeEngineAudio::renderWithStems` peut observer six bus stéréo pré-master
sans les réinjecter dans la sortie :

- combustion ;
- échappement sec ;
- retour de l’IR d’échappement ;
- admission ;
- induction forcée ;
- mécanique/structure.

Les taps se trouvent avant le shelf commun, le bloqueur DC, le filtre de
reconstruction, le volume, le leveler et le limiteur. L’échappement sec et l’IR
restent séparés pour ne pas confondre la source moteur avec la pièce ou la
cabine. Les buffers sont fournis par l’appelant et remplis sans allocation.

Export ciblé :

```powershell
out/build/windows-vs2022/tools/Release/EngineLabAudioRenderHarness.exe `
  --stems K20 `
  --output out/validation/audio-stems-k20-v2-2026-07-29
```

Preuve Release du 29 juillet 2026 :

| Stem K20 | RMS | Pic |
|---|---:|---:|
| combustion directe | 0,000000 | 0,000000 |
| échappement sec | 0,030013 | 0,316187 |
| IR échappement | 0,003014 | 0,015993 |
| admission | 0,030953 | 0,522270 |
| induction forcée | 0,000000 | 0,000000 |
| mécanique/structure | 0,027226 | 0,114039 |

Les deux zéros sont attendus : le K20 est atmosphérique et, dès que la frontière
physique prend la main, la pression cylindre excite l’échappement et la
structure au lieu d’être doublée par une voix procédurale directe. Le test
`EngineLab.RealtimeRegression` rend deux instances déterministes, l’une avec
stems et l’autre sans, puis exige l’égalité **bit à bit de chaque échantillon du
master**. Il vérifie aussi les bornes du buffer et le silence des bus inactifs.

## Turbulence au débouché d’échappement

`ExhaustJetNoise` ajoute au débouché une source large bande causale dérivée du
débit, de la densité, de la célérité et du diamètre déjà résolus. Sa puissance
suit la loi subsonique en `U^8` et son centre spectral `St = 0,2`. La modulation
audio vient du débit volumique de la charge de rayonnement ; elle n’est jamais
réinjectée dans le réseau gaz.

La source traverse le même observateur stéréo et la même IR que le débouché.
Elle est déterministe et ne fait aucune allocation dans le callback. Son
coefficient moteur demeure semi-empirique et doit encore passer un vote
d’écoute aveugle. Modèle, valeurs authored, essais rejetés, A/B cinq familles et
coût temps réel sont consignés dans
[`audio-lot3-outlet-turbulence-2026-07-29.md`](audio-lot3-outlet-turbulence-2026-07-29.md).

Contrôle A/B sonore :

```powershell
out/build/windows-vs2022/tools/Release/EngineLabAudioRenderHarness.exe `
  --exhaust-jet-comparison "LS3" --output out/validation/jet-ls3
```

Contrôle A/B CPU, même binaire :

```powershell
out/build/windows-vs2022/tools/Release/EngineLabRealtimeBudgetHarness.exe `
  --filter Merlin --warmup 3 --seconds 10 --free-run --with-audio `
  --disable-exhaust-jet-noise
```

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
- loi en `U^8`, Strouhal, niveau RMS et silence à débit nul du jet de sortie ;
- influence du signe du débit ;
- silence d’une frontière SI stationnaire ;
- invariance aux presets, événements et bruits hérités ;
- dépendance aux longueurs de tailpipe ;
- cohérence de délai à 48 et 96 kHz ;
- exactitude de la conversion SPL ↔ pression ↔ dBFS ;
- absence de réactivation du chemin procédural après verrouillage physique.

Le banc de capacité peut également exécuter le renderer en concurrence avec le
vrai thread runtime :

```powershell
out/build/windows-vs2022/tools/Release/EngineLabRealtimeBudgetHarness.exe `
  --catalog-root . --free-run --rpm 7000 --warmup 3 --seconds 6 `
  --with-audio --audio-rate 48000 --audio-block 256
```

En `--free-run`, le consommateur suit le temps simulé accéléré et compare chaque
durée de rendu à l'échéance réelle du bloc. Il refuse pertes de files, frontière
invalide, fallback historique, sortie non finie, callback hors budget et
intervention du leveler. La baseline locale est consignée dans
[audio-first-baseline-2026-07-29.md](audio-first-baseline-2026-07-29.md).

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
- modes structurels estimés lorsque aucune section NVH sourcée n’est fournie ;
- aucune mesure NVH réelle livrée dans le catalogue à ce jour, malgré le chemin
  `measured` désormais configurable ;
- rendement acoustique de suralimentation encore semi-empirique ;
- corrélation multi-microphone réelle encore à effectuer moteur par moteur.
