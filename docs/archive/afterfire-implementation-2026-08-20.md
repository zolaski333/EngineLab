# Afterfire local, audible et borné — implémentation du 20 août 2026

## Résultat livré

Le commit `9f4fa96` corrige le chemin afterfire sans sample, oscillateur, EQ de
masquage ni hausse arbitraire du gain. Le carburant imbrûlé reste transporté et
brûlé dans le réseau gaz quasi-1D ; sa chaleur devient maintenant une source
thermoacoustique compacte au nœud et à la position où la réaction a réellement
eu lieu, puis traverse le DAG d’échappement commun au blowdown.

Le cas produit `Audio Physics Lab 689 Twin` donne désormais, après un vrai lever
de pied à ligne chaude :

- zéro réaction pendant l’intervalle chargé qui précède le lever ;
- 27 excursions de chaleur sur 8 s, 63,532 mg brûlés et 4 579,559 J publiés ;
- une crête audio de 0,00518 et un crest factor de 6,41 ;
- zéro perte de queue, zéro voix volée, zéro limite de pression, zéro garde de
  niveau et zéro bloc audio hors budget ;
- seulement +0,5 point de budget moyen et +0,7 point au p99 face au contrôle où
  les mêmes réactions ne sont pas copiées vers l’audio.

Cette preuve établit que le chemin logiciel est causal, audible, non vide et
compatible temps réel. Elle ne constitue pas une identification acoustique sur
un véhicule déterminé.

## Pourquoi l’ancien afterfire ne « poppait » pas

Le défaut n’était pas unique. Plusieurs erreurs se renforçaient.

### 1. Le filtre audio supprimait la bande de la réaction

La source précédente calculait bien une pression depuis la puissance chimique,
mais lui appliquait deux passe-haut dont la coupure valait
`couplingFrequency × 0,42`, soit environ 3 à 6 kHz dans les scénarios mesurés.
Une libération d’énergie de 8 ms possède son contenu principal vers 125 Hz : le
filtre supprimait donc presque tout le front qu’il devait transmettre.

Le même chemin divisait déjà l’expression par deux avant de l’envoyer dans le
réseau, alors que le réseau partage lui-même un saut de pression entre ses deux
ondes voyageuses. L’amplitude était divisée deux fois. Trois exponentielles
étaient également recalculées dans la boucle par échantillon, même quand aucune
réaction n’était active.

Mesure avant correction, avec physique identique et événements audio
activés/coupés : composante directe à **−57,747 dB** du signal complet, avec au
maximum un seul LSB de différence dans le WAV 16 bits. La chimie pouvait donc
réagir sans produire un pop pratiquement observable.

### 2. Le délai d’induction écrit n’était pas le délai exécuté

Le solveur accumulait :

```text
dt_induction = dt × clamp((T_source - T_ignition) / 450 K, 0, 1)
```

La configuration présentait pourtant `induction_time_s` comme une durée. Avec
4 ms écrits, un site situé 1 K au-dessus du seuil demandait environ 1,8 s et un
site 10 K au-dessus environ 180 ms. Cette rampe de 450 K n’était ni exposée ni
documentée ; elle rendait la frontière d’allumage presque inutilisable.

Le nouveau contrat est direct : chaque durée passée à ou au-dessus du seuil
s’ajoute à l’induction. Le test gaz couvre explicitement un site seulement 1 K
au-dessus du seuil et vérifie qu’il s’allume après les 4 ms écrits.

### 3. L’origine « paroi » était réinterprétée après combustion

Le compteur `wallIgnited` comparait la température du gaz après que la réaction
l’avait elle-même chauffé. Un noyau allumé par le gaz pouvait donc être classé
comme allumé par la paroi, ou l’inverse selon l’ordre des mises à jour. L’origine
est maintenant figée à l’instant précis où le noyau passe à l’état `burning`,
puis remise à zéro au quench ou à l’épuisement des réactifs.

### 4. Une calibration afterfire rendait la chimie permanente

Le simulateur appelait `reactUnburnedFuel` à chaque avance du réseau dès que le
moteur **possédait** une stratégie afterfire ou un rupteur humide. La chimie
tournait donc aussi pendant la chauffe à pleine charge. À constante rapide,
elle pouvait produire des sources proches de la borne de sécurité avant même le
lever de pied.

La chimie est désormais activée seulement dans l’un des états suivants :

```text
afterfire retenant du carburant ET DFCO actif
OU
rupteur retenant du carburant ET spark-cut humide / limite dure active
```

Les fenêtres fermées d’un pulse afterfire continuent à réagir parce que le DFCO
reste actif pendant le transport du paquet. Le calcul est au contraire bypassé
à pleine charge ordinaire, ce qui corrige la sémantique et réduit le coût CPU.

### 5. Le harness ne mesurait pas le geste annoncé

Deux défauts d’instrumentation ont retardé le diagnostic :

- le défaut du harness remplaçait silencieusement la calibration cataloguée
  (800 K, 8 ms à ce moment) par un cas forcé à 900 K, 10 ms et réaction
  continue ; `--force-demo` est maintenant la seule façon explicite de demander
  cet ancien cas de laboratoire ;
- la phase dite « coast down » fermait déjà la pédale plusieurs secondes avant
  l’instant étiqueté `lift-off`. La mesure commençait donc au milieu d’un
  overrun, pas sur son front.

Le protocole actuel freine sous 28 % de pédale, exécute ensuite 750 ms de charge
normale et mesure les 500 dernières millisecondes comme contrôle négatif. Une
seule vraie chute 28 % → 0 % ouvre la fenêtre afterfire. Tout événement de
réaction dans le contrôle chargé rend le processus invalide.

Le rapport distingue aussi trois masses qui étaient auparavant confondues :

- carburant ECU mesuré ;
- carburant effectivement livré aux cylindres après le film de paroi ;
- carburant réellement brûlé dans l’échappement.

Dans la preuve finale, ces valeurs sont respectivement 965,068 mg, 177,850 mg
et 63,532 mg. Le ratio utile pour la chimie locale est donc 35,7 % du carburant
livré brûlé, pas 6,6 % de la commande ECU brute.

## Source thermoacoustique retenue

Pour une source de chaleur compacte dans un conduit uniforme, l’intégration de
l’équation d’énergie linéarisée donne le saut de pression caractéristique :

```text
Delta p = (gamma - 1) Qdot / (A c)
```

avec :

- `Qdot` : puissance de réaction conservative en watts ;
- `A` : section locale du conduit en mètres carrés ;
- `c` : célérité locale en mètres par seconde ;
- `gamma - 1 = 0,34` dans le modèle d’échappement actuel.

`AcousticExhaustNetwork` partage ce saut entre les caractéristiques droite et
gauche. Chaque onde émise reçoit donc :

```text
p_plus = p_minus = (gamma - 1) Qdot / (2 A c)
```

Le signal arrive à la cadence du couplage gaz, pas à 48 kHz. Le nouveau
`ThermoacousticHeatReleaseSource` applique :

1. le filtre de reconstruction anti-imaging LR8 déjà utilisé pour les
   frontières physiques ;
2. deux bloqueurs continus à 25 Hz qui retirent la composante thermique
   quasi-stationnaire déjà portée par le solveur d’écoulement moyen ;
3. l’injection au `nodeId`, `sourceComponentId` et à la position axiale de
   l’événement.

La clé de voix inclut désormais le composant source en plus du nœud. Deux
branches partageant un nœud de jonction ne s’écrasent donc plus mutuellement.
Les coefficients exponentiels ne sont recalculés que lorsque la cadence de
couplage change de plus de 1 %. Une voix éteinte conserve une courte queue pour
laisser les filtres revenir à zéro, puis est libérée. Le tableau reste borné à
64 voix et n’alloue rien dans le callback.

## Borne du domaine linéaire et télémétrie

Le réseau conserve une borne de dernier recours de ±100 kPa par source compacte.
Ce n’est pas un limiteur de timbre : chaque échantillon qui la dépasserait
incrémente `reactionPressureLimitedSampleCount`.

Ce compteur est :

- visible dans `PHYSICS DEBUG` sous `Limite pression AF` ;
- publié par `RealtimeEngineAudio` ;
- inclus dans `EngineLabRealtimeBudgetHarness` ;
- inclus et invalidant dans `EngineLabAfterfireHarness`.

La preuve finale atteint 54,708 kPa au maximum et garde ce compteur à zéro. Une
future calibration qui dépasserait le domaine linéaire échouera donc au lieu de
sembler valide grâce à un clamp silencieux.

## Calibration du moteur laboratoire

Le catalogue et `DEMO AUDIBLE` passent de 8 ms à 2 ms. Cette modification ne
change ni la masse de carburant, ni l’énergie chimique, ni le seuil de 800 K, ni
le délai d’induction de 4 ms. Elle concentre la même libération conservative
dans un front résolu :

| Calibration | Pic chaleur | Pic compact calculé | Pic audio | Crest audio | Limite 100 kPa |
|---|---:|---:|---:|---:|---:|
| 8 ms, vrai lever | 6,565 kW | 13,057 kPa | 0,00298 | 3,90 | 0 |
| 2 ms, vrai lever final | 9,244 kW | 54,708 kPa | 0,00518 | 6,41 | 0 |

La valeur générique de configuration n’est pas changée. Seul le moteur
d’écoute explicitement expérimental et le bouton qui reproduit sa calibration
emploient 2 ms. Le slider UI avait déjà 2 ms comme borne minimale.

## A/B final strict

Commande de référence :

```text
EngineLabAfterfireHarness.exe
  --engines "Audio Physics Lab 689 Twin"
  --audio <dossier>
  --warmup-seconds 60
  --overrun-seconds 8
  --liftoff-rpm 4000
```

Le contrôle ajoute uniquement `--no-reaction-acoustics`. Les deux sorties
confirment la même physique :

| Mesure | Réaction audio OFF | Réaction audio ON |
|---|---:|---:|
| régime au lever | 4 291 tr/min | 4 291 tr/min |
| paroi au lever | 544,5 °C | 544,5 °C |
| minimum paroi overrun | 532,2 °C | 532,2 °C |
| événements chargés avant lever | 0 | 0 |
| puissance chaleur crête | 9,244 kW | 9,244 kW |
| événements chaleur | 27 | 27 |
| carburant brûlé | 63,532 mg | 63,532 mg |
| énergie source publiée | 4 579,559 J | 4 579,559 J |
| crête audio | 0,00348 | 0,00518 |
| percentile 99,9 audio | 0,00262 | 0,00291 |
| crest factor | 4,48 | 6,41 |
| limite pression / pertes / leveler | 0 | 0 |

La crête gagne **3,46 dB**. La soustraction échantillon par échantillon des WAV
de même longueur donne, sur les 8 s d’overrun :

```text
RMS signal ON       26,457 LSB
RMS composante AF    6,320 LSB
AF / mix complet   -12,436 dB
différence max      124 LSB
échantillons changés 204 200
```

Les artefacts sont conservés sous :

- `out/audit-2026-08-20/afterfire-final-timed-on/` ;
- `out/audit-2026-08-20/afterfire-final-timed-off/`.

## Budget CPU

### Source active pendant l’overrun

Le harness chronomètre seulement l’appel au renderer, sans l’écriture WAV ni la
simulation. À 48 kHz, un bloc de 200 échantillons dispose de 4,167 ms :

| Chemin | Blocs | Moyen | P99 | Maximum | Hors budget |
|---|---:|---:|---:|---:|---:|
| événements copiés OFF | 1 920 | 13,1 % | 13,9 % | 16,9 % | 0 |
| source thermoacoustique active | 1 920 | 13,6 % | 14,6 % | 21,6 % | 0 |

Le surcoût observé est donc de 0,5 point moyen et 0,7 point au p99 sur ce moteur.

### Pires scénarios produit hors afterfire

Le harness temps réel complet a été rejoué en Release, libre, 48 kHz / 256,
20 s mesurées :

| Moteur | Régime | Facteur capacité | DSP moyen | DSP p99 | Violations |
|---|---:|---:|---:|---:|---:|
| LS3-like V8 | 6 270 tr/min | 1,053× | 34,5 % | 54 % | 0 |
| Merlin-like V12 | 3 040 tr/min | 1,449× | 53,1 % | 81 % | 0 |

Les références immédiatement antérieures étaient 1,018× et 1,367× sur des
passages plus courts. La variation ne permet pas de revendiquer un gain précis,
mais elle exclut une régression observable. Le retrait des exponentielles par
échantillon et le bypass chimique hors DFCO expliquent aussi pourquoi ce lot
n’ajoute pas une charge permanente.

## Tests ajoutés ou durcis

- `EngineLab.GasDynamics` vérifie le délai d’induction réel près du seuil et
  l’origine gaz/paroi mémorisée ;
- `EngineLab.RealtimeRegression` vérifie l’échelle analytique de la source, sa
  linéarité lorsque la puissance double, son front bipolaire, son retour à zéro
  et le silence d’une chaleur continue ;
- `EngineLab.AudioWorkshop` épingle la valeur 2 ms du moteur laboratoire et du
  bouton de démonstration ;
- `EngineLab.Core` couvre les contrats ECU/simulation existants avec le nouveau
  chemin relié ;
- les deux harness rendent désormais la limite de pression invalidante ;
- le harness afterfire rend également invalidants une réaction pré-lever et un
  rendu actif plus long que son bloc.

Validation ciblée avant la suite intégrale :

```text
EngineLab.GasDynamics          passé, 0,64 s
EngineLab.RealtimeRegression   passé, 3,52 s
EngineLab.AudioWorkshop        passé, 0,54 s
EngineLab.Core                 passé, 47,70 s
EngineLabAfterfireHarness      valide, code retour 0
EngineLabRealtimeBudgetHarness valide sur LS3 et Merlin
EngineLab.exe                  reconstruit en Release
```

### Défauts latents révélés par la reconstruction intégrale

Le correctif transversal correspondant est publié dans `2465157`.

Le premier passage complet, après reconstruction de **tous** les exécutables,
n’a pas été déclaré vert : 39/42 tests seulement ont passé. Cette étape a
révélé trois défauts qui avaient été masqués par des binaires de test anciens ou
des gates devenus marginaux :

1. `TurboDownstreamAuthority` agrandissait le noyau perforé du silencieux en
   même temps que la sortie, sans agrandir le volume du corps. La géométrie
   devenait impossible (`volume corps < volume noyau`) et l’exception non
   interceptée finissait en `0xc0000409`. Le sweep ne modifie désormais que le
   composant `outlet`, ce qui est aussi le contrat annoncé ; un catch de niveau
   harness rend toute future exception lisible. Le test isolé passe en 93,47 s
   et conserve un span de pression turbine de 28,3 kPa ainsi que tous ses signes.
2. Le Big Twin publiait 84,871 kW à 4 600 tr/min contre 73 kW, soit +16,261 %
   pour une borne de 15 %. L’ancien résultat du 11 août était déjà marginal à
   +14,291 %. Un balayage du coefficient explicite de turbulence de chambre a
   ramené les deux ancrages à −3,477 % en couple et +3,769 % en puissance avec
   `0,42`, sans modifier la référence ni sa tolérance. Le test complet des 24
   points passe de nouveau en 65,38 s.
3. `AudioRender` appelait « steady-state » la dernière seconde d’un rendu de
   3 s alors que le dyno ne démarre qu’à 1,2 s. Sur le diesel EA288, le
   remplissage admission traversait encore cette fenêtre et donnait un crest de
   18,42. Couper seulement l’admission le ramenait à 6,44 ; attendre 4 s avec le
   mix complet donne 7,21. Les rendus catalogue durent maintenant 4 s, tandis
   que les scénarios transitoires dédiés restent inchangés. `AudioRender` complet
   passe en 103,48 s.

Ces corrections ne modifient pas le gain afterfire. Elles durcissent toutefois
la crédibilité de sa validation : un lot n’est pas considéré livré sur la base
d’un CTest vert si les exécutables concernés n’ont pas d’abord été reliés avec
les bibliothèques courantes.

Après correction et reconstruction des cibles concernées, le second passage
autoritaire donne :

```text
42/42 CTest passés
0 échec
temps total : 735,23 s
```

Il inclut notamment les rampes dyno produit CP2/LS3, l’export audio, le rendu
catalogue 4 s, les transitoires, la physique catalogue, les 24 points fabricant,
le sweep turbo et les régressions thermiques d’overrun.

## Limites restantes, explicitement conservées

1. La réaction demeure une relaxation exponentielle conservative. Elle n’est
   pas un solveur cinétique détaillé multi-espèces.
2. L’induction est persistante par volume de contrôle, mais un noyau de flamme
   n’est pas encore une espèce advectée distincte entre les cellules.
3. Le réseau acoustique est un guide d’onde 1-D linéaire. Une source qui
   atteindrait 100 kPa sortirait de ce domaine ; elle est détectée et refusée,
   pas simulée comme une onde de choc.
4. Le tableau de 64 voix est volontairement borné. Toute perte amont ou tout
   vol de voix reste instrumenté et invalide les mesures.
5. La constante 2 ms et les autres valeurs du moteur laboratoire sont des
   estimations d’ingénierie. Les tests prouvent leur effet et leur stabilité,
   pas leur correspondance à un échappement déterminé.
6. La couche afterfire est désormais identifiable dans le mix, mais le timbre
   final dépend encore des dimensions, pertes et terminaisons écrites dans le
   DAG. Il n’existe volontairement pas de « son de pop » commun à tous les
   moteurs.

## Ordre recommandé après ce lot

Le prochain chantier physique ne doit pas multiplier les voix ou les mailles.
Les améliorations à meilleur rapport précision/coût sont :

1. rendre l’induction sensible à une intégrale thermochimique explicitement
   paramétrée, sans constante cachée ;
2. transporter un faible état de noyau/allumage avec l’inventaire existant si
   les essais de slugs montrent que l’état par cellule reste trop local ;
3. ajouter des fixtures de spatialisation réactionnelle sur X-pipe et branches
   multiples, avec les mêmes compteurs de perte ;
4. conserver les gates LS3/Merlin et le contrôle A/B strict à chaque évolution.

Ces étapes ajoutent quelques scalaires par volume réactif ou des tests hors
temps réel ; elles n’exigent ni nouvelle grille 3-D, ni convolution par réaction,
ni banque de samples.

## Mise à jour du 22 août 2026

Le premier point de l'ordre ci-dessus est livré dans le schéma moteur 8. Le
timer plat a été remplacé, pour les configurations qui l'authorisent, par une
intégrale `dt/tau(T,p,phi)` dont délai de référence, pression de référence,
`Ea/R`, exposants pression/richesse et temps de décroissance sont tous exposés.
Les schémas 1 à 7 gardent exactement le timer historique grâce à des exposants
nuls.

Le sweep isolé passe de `4/4/4/4 ms` à `3,95/0,20/2,25/5,90 ms` sur les cas
référence/chaud/pressurisé/pauvre. Le geste produit reste audible, borné et très
loin du budget bloc ; il n'a reçu aucun gain compensatoire lorsque son crest a
baissé. Le détail, la provenance et l'A/B même binaire sont dans
[`afterfire-induction-implementation-2026-08-22.md`](afterfire-induction-implementation-2026-08-22.md).

Le transport d'un noyau distinct reste conditionnel : il ne sera ajouté que si
un oracle de slug met en évidence une dépendance fautive au maillage ou une
localisation incorrecte. La fixture X-pipe/multi-branche devient donc le prochain
instrument afterfire utile, pas une hausse du nombre de voix.
