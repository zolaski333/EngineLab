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

Les coefficients dépendant du temps sont convertis à la fréquence réelle du
périphérique. Les constantes de décroissance, filtres et délais ne sont donc
pas supposés valables uniquement à 48 kHz. Les lignes de 131 072 échantillons
conservent les 80 ms physiques à 192 kHz, y compris au ralenti temporel 0,25×
et pendant un échappement froid.

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

## Validation

`EngineLab.RealtimeRegression` couvre notamment le routage continu par chemin,
les limites de segment, les blocs 1 024/2 048 à 48/96/192 kHz, la transmission
du DAG, l'atmosphère et les valeurs finies. `EngineLab.AudioRender` rend plusieurs
moteurs hors ligne à un pourcentage de régime commun et vérifie énergie,
dynamique, offset DC, plateaux de clipping, équilibre spectral, temps passé au
plafond du limiteur et finitude. Une similarité spectrale trop forte entre
moteurs est désormais un échec. Le harnais de comparaison produit également un
WAV directement depuis les trames de pression afin de détecter une source
silencieuse.

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
