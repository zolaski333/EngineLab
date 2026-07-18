# Audio temps réel

L'audio d'EngineLab est un renderer hybride : les formes continues viennent de
la simulation thermodynamique, les événements apportent les transitoires et la
spatialisation, et les couches DSP représentent la propagation et la couleur
perceptuelle. Il ne s'agit pas d'une simulation acoustique CFD.

## Pont simulation-audio

Le runtime publie deux flux SPSC distincts :

- `FiringEvent` transporte étincelle, raté, pression, intensité, cylindre,
  panorama, chemin, délai, résonance et transmission d'échappement ;
- `CylinderPressureSample` transporte, à chaque sous-pas produit lorsque
  l'audio est actif, la pression cylindre, la pression runner, le débit
  d'échappement, l'ouverture de soupape et l'indice de chemin de chaque
  cylindre.

Les horodatages utilisent le temps de simulation publié par le producteur. Le
renderer les convertit dans sa propre chronologie audio, limite le look-ahead
et compte les événements tardifs ou abandonnés. Une réinitialisation du
simulateur vide les trames de pression restantes pour éviter de rejouer un état
ancien.

Le look-ahead vaut au minimum 20 ms et augmente jusqu'à couvrir un bloc hôte
complet plus 5 ms de marge. La file de pression conserve plus de 85 ms au débit
maximal de publication (96 kHz), ce qui couvre les blocs testés jusqu'à 2 048
échantillons à 48, 96 et 192 kHz.

Les atomiques de `RealtimeAudioState` publient les grandeurs lentes : régime,
papillon, charge, pression ambiante, boost, résonance admission, cinématique,
température des gaz, sections acoustiques, gains de mix et paramètres par
chemin.

## Contrat du callback

`RealtimeEngineAudio::render` n'effectue aucune entrée/sortie, journalisation,
attente ou acquisition de mutex. Voix, tableaux de délais, buffers de guides
d'onde, FDN et buffers de convolution sont alloués pendant la préparation.
Chaque bloc respecte le segment `startSample/numSamples` fourni par JUCE.

`prepare()` alloue et lit la géométrie statique publiée par le constructeur de
`EngineRuntime`, qui précède toujours celui du renderer. Un changement structurel
de moteur reconstruit les deux objets, si bien que ce dimensionnement n'observe
jamais un moteur périmé.

Les coefficients dépendant du temps sont convertis à la fréquence réelle du
périphérique. Les constantes de décroissance, filtres et délais ne sont donc
pas supposés valables uniquement à 48 kHz.

### Dimensionnement des lignes à retard

Les lignes sont allouées dans `prepare()`, à la taille du moteur réellement
chargé : nombre de cylindres et de chemins publiés, et longueur déduite des
délais effectivement publiés par le runtime. Elles ne sont plus des tableaux
fixes dimensionnés au pire cas et payés par tous les moteurs.

Chaque délai physique est étiré au rendu par
`acousticDelayScale = c_référence / c_échappement / timeScale`, borné par les
clamps du rendu à `900 / 289,8 / 0,25 = 12,42`. Les lignes intègrent ce pire cas,
si bien qu'un échappement froid écouté au quart de vitesse ne les atteint jamais.

Le runtime borne les délais publiés à 80 ms, mais c'est une clamp de sécurité et
non une borne physique : 80 ms de primaire correspondraient à un tube de 41 m.
Dimensionner sur cette valeur gaspillerait environ 87×, et l'ancien
dimensionnement fixe était de toute façon **trop court** pour son propre pire cas
(0,080 × 12,42 × 192 000 = 190 764 pour 131 072 alloués) : les lignes tronquaient
en silence dans ce coin, comme les lignes FDN.

Les longueurs sont des puissances de deux, ce qui remplace le modulo par échantillon
par un masque. Une troncature résiduelle — un délai qui grandirait après
`prepare()` — est comptée par `delayTruncationCount()` et fait échouer
`EngineLab.AudioRender`. Elle n'est plus silencieuse.

Les maxima de compilation (huit chemins, 32 cylindres) ne dimensionnent plus que
de petits tableaux de travail par bloc.

## Sources et mixage

La trame de pression est interpolée vers la fréquence audio. Le signal de
chambre est recentré, dérivé et filtré ; la pression runner et son débit
alimentent le blowdown. La bande passante de reconstruction suit la cadence
adaptative réellement publiée par le solveur et reste sous sa zone de Nyquist,
au lieu de forcer un filtre fixe sur une source sous-échantillonnée.
Contrairement à un synthétiseur uniquement déclenché à l'étincelle, un raté
conserve ici la forme de pression réellement calculée.

Les `FiringEvent` ajoutent des voix bornées pour le corps moteur, le choc
d'échappement, les composantes de knock et la spatialisation par cylindre. Le
renderer limite et compte les vols de voix lorsque le pool est saturé. La
transmission du DAG ne module que la voix d'échappement ; elle ne change pas
l'intensité de combustion portée par l'événement.

Les autres couches utilisent :

- bruit et mode Helmholtz mesuré pour l'admission ;
- harmoniques liées au régime, accélération piston et charge pour la mécanique ;
- activité de distribution pour le haut du spectre ;
- couche dédiée au démarreur ;
- sifflement ou composante mécanique de suralimentation selon le type configuré.

Les gains combustion, échappement, admission et mécanique restent séparés
jusqu'au mix final stéréo.

## Traitement par chemin d'échappement

Jusqu'à huit chemins physiques sont conservés. Une trame de cylindre et un
événement n'excitent que le chemin auquel leur cylindre est affecté ; le signal
continu n'est plus rabattu arbitrairement vers le chemin zéro.

Chaque chemin possède :

- les lignes aller/retour de ses primaires et une jonction de collecteur à
  admittances pondérées par les sections réelles ;
- un délai de réflexion et un filtrage de sortie ;
- un réseau FDN de silencieux ;
- un jitter fractionnaire dépendant du débit, du bruit d'air et un corps
  résonant ;
- son gain, son ouverture et sa réponse impulsionnelle.

La réflexion côté port varie avec l'ouverture de la soupape. Le retour de la
ligne de sortie rejoint réellement la jonction des primaires, au lieu d'être
simulé par une boucle instantanée. Les vitesses d'onde suivent
`c = √(γRT)` avec la température d'échappement simulée, et tous les délais
physiques suivent aussi `timeScale`.

Cette vitesse d'onde provient d'une source unique
(`enginelab/foundation/ExhaustGasAcoustics.hpp`) partagée avec le compilateur de
topologie : les lignes à retard audio et les modes de résonance du graphe ne
peuvent plus supposer des célérités différentes. Les propriétés retenues
(γ ≈ 1,33, R ≈ 287 J/kg/K) sont un point de fonctionnement représentatif du gaz
chaud, distinct du γ par cellule et dépendant de la composition que calcule le
solveur thermodynamique.

Le réseau FDN de silencieux utilise une matrice de Hadamard 4×4 normalisée à
`H/2` (facteur 0,5), donc réellement sans perte : le coefficient de rebouclage
du preset est désormais le gain de boucle effectif qui fixe la décroissance.
L'ancienne normalisation à 0,25 appliquait `H/4`, dont les valeurs singulières
valent 0,5 : elle divisait l'énergie par deux à chaque tour, et le gain de preset
ne valait que la moitié de sa valeur affichée.

Le réseau FDN et le corps résonant font partie du modèle d'échappement et ne
suivent plus le potentiomètre `convolution`, qui ne règle que le mélange de la
réponse impulsionnelle. Baisser l'IR ne coupe donc plus la simulation de
silencieux comme auparavant ; le niveau humide par preset intègre la valeur par
défaut de ce potentiomètre, si bien que le rendu par défaut est inchangé.

Ce guide d'onde est un modèle DSP agrégé primaire/collecteur. Il ne résout pas
encore chaque coude, changement de section ou composant comme une cellule 1D
non linéaire. Pour un DAG personnalisé, le runtime publie néanmoins par
cylindre le gain, la longueur primaire et les modes combinés des routes. La
route la plus énergétique fournit le délai scalaire historique ; moyenne,
première/dernière arrivée et dispersion RMS restent disponibles. Le signal
continu traverse primaire, collecteur puis sortie, tandis que la voix
événementielle déjà retardée n'est pas propagée une seconde fois. Les branches
restent ramenées à un chemin acoustique par sortie configurée, pas à plusieurs
sources spatialisées issues d'un même splitter.

## Réponses impulsionnelles

`RealtimeConvolutionBank` contient huit convolutions partitionnées JUCE
indépendantes. L'application essaie, dans l'ordre :

1. le WAV déclaré par le chemin moteur ;
2. l'IR du preset acoustique courant ;
3. l'IR générique `assets/ir/exhaust_default.wav` ;
4. une IR déterministe générée depuis la géométrie si aucun asset n'est lisible.

Les WAV mono ou stéréo sont décodés hors callback, limités à 262 144
échantillons et ré-échantillonnés par la convolution à la fréquence du
périphérique. Le preset **Street/Open/Turbo/Long tube/Moto** règle aussi la
matière et l'ouverture du modèle ; il ne change pas l'affectation physique des
cylindres.

## Conditionnement de sortie

Le chemin sec et la convolution sont combinés avant un véritable shelf aigu,
un bloqueur DC et le filtre de reconstruction. Le volume maître est appliqué
avant un leveler de sécurité lent, puis un unique limiteur doux à seuil est
exécuté en suréchantillonnage 2×. En régime normal, le collecteur, les guides et
le FDN restent linéaires ; leurs anciennes saturations successives ont été
retirées. Le limiteur conserve une marge inter-échantillon et la dernière borne
ne sert qu'à empêcher une valeur non finie ou pathologique d'atteindre le
buffer.

## Audit d'honnêteté du voicing

Quatre compensations « fragiles » étaient signalées comme risquant de rendre les
paramètres menteurs (un réglage qui ne fait pas ce qu'il prétend). Chacune a été
**mesurée** au réglage `convolution` par défaut via `AudioRenderHarness` avant
toute intervention, selon la règle « measure before you fix » de `CLAUDE.md`. Le
résultat : trois des quatre sont déjà résolues dans le code actuel, la quatrième
est hors du chemin audio. La preuve métrique est donnée pour chacune.

| Compensation | Cause racine | État mesuré | Preuve |
|---|---|---|---|
| Flamme turbulente ×1.12 | Fermeture de vitesse de flamme intégrale (∝ u', pas √(u'/S_L)) dans `FlamePhysicsModel`. Physique de combustion, **pas** le chemin audio. | Load-bearing et in-spec : le calage est dans la fenêtre MBT. La « corriger » isolément décalerait le phasing sans gain observable. Hors périmètre voicing. | `EngineLab.CombustionPhasing` PASS (LPP 18–22° ATDC, CA50 9–11°). Non modifiée. |
| AGC de sécurité masquant un offset de niveau | Leveler lent en aval de `physicalReferenceLevel = 3.50`. Le doute : le niveau serait trop chaud et l'AGC le rabattrait en silence. | **Faux au voicing par défaut** : le leveler reste à l'identité exacte. Les pics (0,29–0,70) restent sous le seuil 0,78 ; l'AGC ne fait aucun travail de gain permanent. C'est réellement un filet de sécurité. | `minLevelGain = 1.0000`, `levelLimited = 0` sur les 4 moteurs, le turbo et les 25 s de stabilité. |
| Presets de silencieux réglés sur une matrice FDN mal normalisée | Hadamard normalisée `H/4` (0,25) divisant l'énergie par deux à chaque tour → le gain de preset valait la moitié de sa valeur affichée. | Corrigée en `H/2` (0,5, sans perte) ; les valeurs `fdn` ont été divisées par deux et `presetFdngain_` est désormais le **vrai** gain de boucle. | RT60 Schroeder par preset stable et fini (street 0,123 s, turbo 0,136 s, long-tube 0,131 s). |
| Couplage `convolution` ↔ modèle de silencieux | Baisser l'IR coupait aussi le FDN et l'excitation du corps. | Découplé : `fdnWet = processMufflerFdn(...) * presetWet_` **sans** facteur `convolution` ; `irMix = convolution·0,5` ne pilote que le mélange d'IR. Résiduel : les `presetWet_` sont calibrés à la main en intégrant le défaut 0,45 — c'est une provenance de valeur, plus un couplage runtime. | Le harness encadre la décroissance dry (FDN seul) vs full (IR) par preset ; baisser `convolution` ne change plus la queue du FDN. |

**Contribution concrète de cette session.** L'AGC était affirmée « safety-only »
par un commentaire, sans preuve. Elle est désormais **mesurée et gardée** :
`RealtimeEngineAudio` expose `levelLimitedSampleCount()` et
`minObservedLevelGain()` (observateurs en lecture seule, publiés une fois par
bloc, sans effet sur l'audio), et `EngineLab.AudioRender` échoue si le leveler
s'engage au voicing par défaut. Vérifié non-vacuous : porter
`physicalReferenceLevel` à 30 engage le leveler (`minLevelGain ≈ 0,40`) et fait
échouer le gate ; le retour à 3,50 le repasse à l'identité. Le paramètre déclare
donc maintenant ce qu'il est, au lieu qu'un commentaire l'affirme.

**Preuve d'absence de changement de son.** L'instrumentation ci-dessus est
purement observatrice : les métriques audio (RMS, pic, crête, bandes, RT60) sont
**bit-identiques** au baseline capturé avant modification (diff vide au réglage
par défaut). Aucun refactor supplémentaire n'a été fait sur les trois
compensations déjà résolues : leurs valeurs de preset sont calibrées (non des
multiples ronds), et les « ré-honnêtiser » cosmétiquement risquerait un écart de
voicing pour zéro gain mesurable — précisément ce que `CLAUDE.md` interdit.

## Validation

`EngineLab.RealtimeRegression` couvre notamment le routage continu par chemin,
les limites de segment, les blocs 1 024/2 048 à 48/96/192 kHz, la transmission
du DAG, l'atmosphère et les valeurs finies.

`EngineLab.AudioRender` rend plusieurs moteurs hors ligne à un pourcentage de
régime commun. Les propriétés de sûreté — finitude, dépassement de pleine
échelle, temps passé au plafond, plateaux de clipping — sont vérifiées sur
**toute** la durée rendue, y compris les 25 s du test de stabilité longue ; une
divergence en milieu de course est donc un échec. L'empreinte spectrale, le RMS,
le facteur de crête et l'offset DC sont mesurés sur la fenêtre stationnaire
finale. Les deux canaux sont analysés séparément, et une corrélation L/R trop
proche de 1 échoue : un effondrement de l'image stéréo vers le mono est une
régression. Une similarité spectrale trop forte entre moteurs échoue également.

Le harnais mesure aussi le temps de réverbération de l'échappement par
intégration inverse de Schroeder, sur la queue d'une impulsion injectée dans un
renderer par ailleurs silencieux.

L'impulsion est injectée comme **télémétrie de pression runner**, pas comme
`FiringEvent`, et ce choix est structurel. Une voix d'événement est sommée
directement sur le bus d'échappement et dans l'IR du chemin : elle n'entre jamais
dans le collecteur. Seul le flux de pression atteint le guide d'onde, la jonction
de collecteur, la ligne de réflexion de sortie et le FDN. Une impulsion
événementielle ne mesure donc que l'enveloppe de la voix, et donne une
décroissance identique pour tous les presets — l'erreur est silencieuse et
crédible, d'où cette note.

L'amplitude est dimensionnée sous le seuil du leveler et le coude du limiteur,
qui restent à l'identité : la décroissance mesurée est celle du modèle acoustique
et non celle d'un compresseur. La mesure est vérifiée amplitude-invariante.

Deux valeurs sont rapportées par preset, avec et sans la couche `convolution`.
Ce n'est pas une isolation du FDN mais un encadrement : `convolution` pilote à la
fois le mix d'IR, le niveau humide du FDN et l'excitation du corps d'échappement,
si bien qu'aucun réglage actuel ne permet d'écouter le modèle sans l'IR.

C'est l'instrument de calibration du FDN de silencieux ; il ne constitue pas
encore une porte, la valeur de référence par preset n'étant pas établie. Le test
échoue seulement si une queue devient non mesurable ou non finie.

Le harnais de comparaison produit également un WAV directement depuis les trames
de pression afin de détecter une source silencieuse.

Le harnais partage avec l'application la fonction `publishAudioFrame`, seule
définition du mappage simulation-vers-audio par trame ; il pilote le simulateur
lui-même pour rester déterministe, sans dupliquer ce mappage.

Ces portes sont des régressions techniques. Elles ne remplacent pas :

- des tests d'écoute en aveugle et à sonie égalisée ;
- des enregistrements multi-microphones de référence ;
- des mesures de latence bout en bout sur plusieurs périphériques ;
- une validation des signatures de collecteurs réels.

## Limites

- huit chemins et 32 cylindres maximum dans le renderer courant ;
- acoustique d'habitacle et position micro réduites aux IR fournies ;
- pas de modèle thermoacoustique 1D maillé ou de rayonnement extérieur ;
- pas d'enregistrement WAV depuis l'interface ;
- aucune revendication actuelle de supériorité perceptuelle prouvée sur ES2D.
