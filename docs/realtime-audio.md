# Audio temps réel

Le chemin d’échappement livré par `EngineRuntime` est un renderer
thermoacoustique en unités SI. Les autres familles sonores — admission,
mécanique, distribution, démarreur et suralimentation — restent hybrides ou
procédurales. La description physique complète et ses limites se trouvent dans
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

Lorsque la première frontière thermoacoustique valide est consommée,
`RealtimeEngineAudio` verrouille `physicalExhaustActive_`. Il supprime toutes les
voix d’échappement procédurales déjà actives ou planifiées. Si la télémétrie
devient ensuite invalide, les guides physiques se vident naturellement ; le
synthétiseur historique ne revient pas.

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

Chaque chemin aboutit à une charge causale passive d’ouverture circulaire non
bridée. L’onde réfléchie retourne au collecteur ; l’accélération de vitesse de
volume donne la pression monopolaire à 1 m. Un retard supplémentaire `r/c` dans
l’air conserve sa phase par rapport aux sources mécaniques.

Le signal reste en pascals jusqu’à la conversion de monitoring. Puisque les
dBFS décrivent une chaîne électrique/numérique et non une pression universelle,
la pleine échelle SPL du micro/préampli est publiée explicitement dans
`RealtimeAudioState::acousticFullScaleSplDb`. Par défaut :

```text
448,275 Pa crête = 316,979 Pa RMS = 144 dB SPL = 0 dBFS
```

La conversion utilise toujours la référence acoustique de 20 µPa. Modifier
cette calibration simule le gain/la marge de la chaîne de capture ; cela ne
modifie ni la pression physique calculée, ni la propagation, ni le volume
d’écoute. Le pic de pression SI observé et l’activité du leveler sont exposés
séparément aux harnais de validation.

Les sorties ne possédant pas encore de position 3D publiée, le champ libre est
mono et co-localisé. Une IR stéréo mesurée peut spatialiser ce signal en aval.

## Réponses impulsionnelles

`RealtimeConvolutionBank` conserve jusqu’à huit convolutions partitionnées,
préallouées hors callback. L’application ne charge une IR que si le chemin
déclare explicitement `impulse_response` dans JSON/YAML. Sans fichier déclaré,
le résultat est le champ libre calculé.

Il n’existe plus de sélection automatique par preset, d’IR générique par défaut
ni d’IR synthétisée depuis les longueurs et restrictions. Une IR explicite est
considérée comme une mesure de propagation aval ; elle ne doit pas doubler la
réponse du tube déjà simulée.

## Autres couches

La pression chambre alimente encore une couche combustion/structure
conditionnée. Admission, mécanique, distribution, démarreur et suralimentation
utilisent les modèles historiques. Ils sont séparés du bus d’échappement et de
son étalonnage SI, mais ils empêchent encore de qualifier le mix complet de
modèle acoustique entièrement physique.

Le remplacement honnête de la mécanique requiert un modèle modal réduit du
bloc, de la culasse et des carters. Voir la section « Compatibilité physique » du
document d’architecture.

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
sous-pas mécanique malgré le couplage multirate du réseau non linéaire.

## Limites connues

- huit chemins et 32 cylindres maximum ;
- acoustique plane et linéaire pour la bande audio ;
- pas de correction de rayonnement par écoulement moyen ;
- pas de position ni directivité 3D par sortie ;
- pas de modèle structurel modal ;
- pas de validation perceptuelle ni de corrélation multi-microphone achevée.
