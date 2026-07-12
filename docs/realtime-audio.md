# Audio temps réel

## Contrat du callback

`RealtimeEngineAudio::render` n'effectue aucune entrée/sortie, journalisation,
attente ou prise de mutex. Les voix, événements en attente, lignes de retard et
buffers de convolution sont préparés avant le démarrage. Les événements
arrivent par `SpscQueue<FiringEvent, 2048>` et conservent leur temps interpolé.

## Sources sonores

Chaque allumage crée une couche bloc moteur immédiate et une couche échappement
retardée par le chemin port-collecteur-silencieux-sortie. Pression cylindre,
vitesse de flamme, débit, pression runner, géométrie, banque et chemin
d'échappement modulent attaque, durée, spectre et panorama. Les couches
continues admission, mécanique, distribution et démarreur suivent les atomiques
du runtime.

Le signal d'échappement combine :

- ondes aller/retour et délai géométrique ;
- réseau FDN du silencieux ;
- bandes de choc, jet et turbulence ;
- mélange du signal de pression et de sa dérivée ;
- jitter à retard fractionnaire dépendant du débit ;
- bruit d'air filtré ;
- leveler attaque/relâchement borné ;
- filtre anti-alias après les non-linéarités.

## Convolution par chemin

`RealtimeConvolutionBank` fournit jusqu'à huit convolutions partitionnées JUCE
DSP indépendantes. `FiringEvent::exhaustPathIndex` route chaque impulsion vers
la bonne réponse. Un WAV mono ou stéréo est décodé hors callback, conservé
jusqu'à 262 144 échantillons, puis ré-échantillonné par le moteur de convolution
à la fréquence du périphérique.

Quand aucun fichier n'est fourni, l'application génère une IR déterministe à
partir de la longueur primaire, du diamètre collecteur et de la restriction du
silencieux. Chaque chemin conserve donc une signature propre sans dépendre des
cinq presets génériques. Les presets restent un réglage de matériau/ouverture,
pas un remplacement des IR moteur.

## Observabilité

Les compteurs d'événements tardifs, événements perdus, voix volées et saturation
du planning restent exposés. Les tests vérifient le routage par chemin, la
différence entre IR, le respect des tranches de buffer, les gains du mixeur et
l'absence de sortie non finie.

## Limites

- huit chemins convolutifs maximum dans le renderer courant ;
- 262 144 échantillons maximum par fichier chargé depuis l'UI ;
- acoustique de l'habitacle et position micro réduites à l'IR fournie ;
- pas encore d'enregistrement WAV depuis l'interface.
