# Observabilité honnête de la chaîne de sortie — 22 août 2026

## Objet

Ce lot ne cherche pas à rendre le son « meilleur » en ajoutant un autre effet.
Il répond d'abord à une question indispensable avant toute retouche : une
différence produite par le moteur ou le graphe d'échappement arrive-t-elle au
master sans être saturée, nivelée, limitée ou clampée ?

La réponse mesurée sur les chemins physiques testés est oui. Le lot corrige
toutefois plusieurs défauts d'instrumentation qui empêchaient de le démontrer,
ainsi qu'une erreur métrologique dans le harness de géométrie. Les ajouts sont
des compteurs et des lectures atomiques par bloc ; aucun nouvel effet ni
traitement par échantillon n'est introduit.

## Défauts trouvés

### Le panneau produit attribuait l'AGC au limiteur

`levelLimitedSampleCount()` compte les frames pendant lesquelles le gain du
garde-niveau lent est inférieur à l'unité. L'interface l'affichait sous le
libellé « Limiter samples ». Les vrais compteurs du soft-limiter et du clamp
final existaient déjà dans le moteur audio, mais n'étaient pas visibles. Une
saturation pouvait donc être diagnostiquée comme un limiteur, et un limiteur
réel pouvait rester invisible.

Le panneau montre maintenant séparément :

- gain minimal et frames de l'AGC ;
- frames traitées par la saturation de voicing ;
- échantillons au-dessus du seuil du soft-limiter ;
- frames non finies ou au-delà du clamp final ;
- crête d'échantillon de sortie au taux hôte.

### Les gates ne couvraient pas toute la chaîne

Les harnesses temps réel, afterfire, rendu, géométrie et changement de rapport
rejetaient l'activité de l'AGC, mais pas systématiquement la saturation de
voicing, le soft-limiter et le clamp dur. Une comparaison physique pouvait donc
être déclarée valide après une non-linéarité non signalée.

Les quatre étages sont désormais publiés, affichés et inclus dans les contrats.
Le mode `physical_reference` exige leur inactivité. Cette règle est aussi testée
dans le cœur et dans l'export offline.

### Une crête d'échantillon était appelée « true peak »

`maximumTruePeakMagnitude()` ne reconstruisait pas les crêtes inter-échantillon
selon une méthode normalisée. Il relevait seulement le maximum des échantillons
au taux hôte après le limiteur. L'API devient
`maximumPostLimiterSampleMagnitude()` et le manifeste JSON publie
`maximum_post_limiter_sample_magnitude`. Le schéma d'export passe de 3 à 4 pour
rendre ce changement explicite.

Il n'y a donc toujours pas de mesureur true-peak normalisé dans le callback, et
le projet ne prétend plus le contraire.

### Le harness de géométrie comparait deux grandeurs incompatibles

La valeur appelée « niveau physique » était le maximum, sur tout le run-up et
le hold, de n'importe quel chemin de pression. Elle était comparée au RMS stéréo
sur la fenêtre stabilisée puis l'écart était attribué à la chaîne aval. Cette
comparaison mélangeait :

- maximum et RMS ;
- chemin individuel et bus agrégé ;
- transitoire complet et fenêtre stabilisée.

Le maximum de pression est conservé sous le nom `runPeakPa`, uniquement comme
diagnostic de stabilité. Les dynamiques et les comparaisons de silencieux sont
maintenant calculées entre RMS du master ou RMS du bus échappement sur la même
fenêtre stabilisée de deux secondes. Le harness refuse aussi le résultat si un
des quatre étages de sortie est actif.

## Chaîne réellement exécutée

Dans l'ordre, la sortie produit traverse :

1. la somme des sources sèches et de la convolution ;
2. éventuellement la saturation `pre_shelf` ;
3. le shelf de voicing, la largeur stéréo, le coupe-continu et le filtre de
   bande ;
4. le garde-niveau lent, dont la cible ne descend que si l'enveloppe dépasse
   0,78 ;
5. dans le domaine suréchantillonné 2×, la saturation `post_shelf` éventuelle
   puis le soft-limiter ;
6. au taux hôte, le garde-fou non-fini et le clamp à ±0,999.

Deux garde-fous évitent de confondre physique et coloration :

- `physical_reference` force le drive de saturation à zéro ainsi que les gains
  artistiques pertinents ;
- les seize voicings livrés déclarent tous `post_shelf`, donc leur saturation
  optionnelle partage déjà le domaine 2× du limiteur.

Le chemin `pre_shelf` reste disponible pour une configuration personnalisée et
n'est pas suréchantillonné. Le doubler maintenant ajouterait du coût à tous les
callbacks pour un mode absent du catalogue livré. Il est conservé comme limite
connue plutôt que masqué par un second suréchantillonneur.

## Résultats reproductibles

### Rendu produit LS3

Le rendu frais du `LS3-like 6.2 Crossplane V8`, avec graphe complet et moniteur
physique, donne :

| Mesure | Résultat |
|---|---:|
| RMS gauche/droite | 0,0016 / 0,0016 |
| pic gauche/droite | 0,1530 / 0,1434 |
| maximum avant limiteur | 0,1469 |
| maximum d'échantillon de sortie | 0,1530 |
| gain AGC minimal | 1,0000 |
| frames AGC | 0 |
| frames de saturation | 0 |
| échantillons soft-limités | 0 |
| frames clampées | 0 |
| pertes/retards/troncatures audio | 0 |

Le pic de sortie peut être légèrement supérieur au relevé pré-limiteur à cause
du filtre de reconstruction du domaine 2×. Il reste loin du clamp et n'implique
aucune activité du limiteur.

Le WAV de référence avant instrumentation et celui rendu après instrumentation
sont bit à bit identiques :

`CD1C1B72F18B8437FF6D473DCF5012D54B3A83C3C09A01E9DE19A8AF0CB136EC`

Les deux captures sont conservées hors Git dans
`out/audit-2026-08-22-output-chain-ls3/` et
`out/audit-2026-08-22-output-chain-ls3-v2/`.

### Reprise correcte de la sensibilité du silencieux

Le LS3 a été rejoué à 4 000 tr/min pendant 3 s. Les niveaux utilisent la même
fenêtre stabilisée de deux secondes :

| Variante à un facteur | RMS bus échappement | Niveau relatif | Écart de forme |
|---|---:|---:|---:|
| référence | 0,002425 | référence | référence |
| corps retiré | 0,003361 | +2,84 dB | 14,28 dB |
| garnissage A/B | 0,002056 | −1,43 dB | 1,37 dB |
| volume annulaire ×2,25 | 0,001898 | −2,13 dB | 6,40 dB |
| volume annulaire ×5,3 | 0,000936 | −8,27 dB | 11,29 dB |

Pour le corps retiré, le master passe seulement de 0,019440 à 0,019584, soit
+0,06 dB. Toutes les douze variantes ont `AGC=1,000` et les quatre compteurs à
zéro. La faible influence large bande du silencieux dans le master est donc le
résultat actuel des sources et de leur balance, pas une réduction cachée après
le graphe. Le changement spectral de 14,28 dB montre néanmoins que la variante
n'est pas équivalente à un simple gain.

Cette correction annule l'ancienne conclusion `+9,41 dB`, issue du ratio entre
deux pics `runPeakPa` incompatibles avec la fenêtre RMS. Elle ne remplace pas ce
chiffre par une prétention sur un silencieux réel : le catalogue ne décrit pas
encore les chicanes, chambres multiples, perforations détaillées et tubes de
liaison d'un modèle déterminé.

### Afterfire avec la chaîne complète

Le moteur `Audio Physics Lab 689 Twin` a été mesuré exactement avec sa
calibration cataloguée, après 60 s de chauffe puis 8 s de coupure des gaz :

| Mesure | Résultat |
|---|---:|
| régime au lever de pied | 4 291 tr/min |
| paroi au lever / minimum overrun | 544,5 / 532,3 °C |
| événements de réaction | 25 |
| masse brûlée / livrée | 72,092 / 178,070 mg |
| espacement min / max / écart-type | 183,3 / 512,5 / 94,7 ms |
| pic thermique | 10,547 kW |
| sources acoustiques distribuées | 1 080 |
| énergie de réaction publiée | 5 175,180 J |
| pic audio overrun / p99,9 | 0,00457 / 0,00292 |
| coût audio moyen / p99 / max | 13,7 / 15,0 / 16,8 % du bloc |

Le chemin physique et le graphe compilé sont actifs. Pertes d'événements,
pressions limitées, retards, voix volées, troncatures, frontières invalides,
legacy, AGC, saturation, soft-limit et clamp restent tous à zéro. L'afterfire
ne dépend donc pas d'un écrêtage pour créer ses événements, et ceux-ci ne sont
pas supprimés par la chaîne de sortie.

### Budget temps réel contraignant

Le LS3 à 95 % du régime rouge, avec audio produit et graphe directionnel, donne :

| Mesure | Résultat |
|---|---:|
| facteur temps réel | 1,022× |
| charge audio moyenne | 35,5 % du bloc |
| p99 audio | 55 % du bloc |
| cadence réseau gaz | 26 880 sous-pas/s |
| couple / puissance | 424,62 Nm / 278,80 kW |
| VE | 0,7940 |
| violations de contrat | 0 |

Cette marge d'environ 2,2 % est faible et doit rester un gate. L'instrumentation
n'ajoute pas de DSP au callback ; elle ne justifie donc pas de dépenser cette
marge dans un second suréchantillonnage ou un analyseur spectral temps réel.

## Validations du lot

La reconstruction Release de l'application et de tous les outils passe. Les
tests ciblés exécutés après les changements passent :

- `EngineLab.Core` : 46,95 s ;
- `EngineLab.OfflineAudioExport` : 7,06 s ;
- `EngineLab.AudioRender` : 103,31 s ;
- total ciblé : 3/3 en 157,33 s.

Le manifeste offline schéma 4 teste désormais les quatre compteurs à zéro et
une crête d'échantillon de sortie inférieure à 0,999. La fixture cœur vérifie
explicitement qu'un rendu `physical_reference` ne passe ni par la saturation,
ni par l'AGC, ni par le soft-limiter, ni par le clamp final.

Après la dernière reconstruction, la suite autoritaire complète termine à
**42/42 CTest**, zéro échec, en **730,07 s**. Elle inclut `AudioRender`, les
régressions temps réel et thermiques, l'autorité turbo et les rampes dyno produit
complètes CP2 et LS3.

## Hypothèses examinées puis réfutées

- La saturation n'est pas obligatoire dans le rendu physique : le mode
  `physical_reference` force déjà son drive à zéro.
- Le coefficient de perte des jonctions n'est pas appliqué deux fois dans le
  solveur gaz actuel : il est porté par le côté amont orienté de la jonction.
- La faible différence de niveau entre géométries n'est pas causée par l'AGC ou
  le limiteur sur les runs reproduits : tous les compteurs restent à zéro.

Ces points ne sont donc pas transformés en modifications de DSP.

## Limites et suite raisonnable

- Il manque encore un analyseur offline normalisé de true peak, THD et repliement
  spectral. Il peut être ajouté hors callback si une campagne de voicing le
  demande ; aucun coût temps réel n'est nécessaire.
- La saturation `pre_shelf` personnalisée n'est pas suréchantillonnée. Les
  voicings livrés n'emploient pas ce mode.
- Le silencieux perforé actuel est un modèle passif réduit. Sans dimensions de
  chicanes, tubes, chambres et perforations, le rendre arbitrairement plus fort
  serait un réglage de cible, pas une correction physique.
- Une géométrie peut changer fortement le bus échappement tout en restant peu
  audible dans le master. Les futurs réglages doivent donc conserver les stems,
  la mesure par bande et les quatre compteurs, au lieu d'évaluer seulement le
  niveau global.

La bonne suite n'est pas d'ajouter un compresseur ou de la saturation : c'est de
continuer à authorer des architectures d'échappement explicites quand leurs
dimensions sont connues, puis de vérifier chaque lot avec ces mesures et le gate
temps réel contraignant.
