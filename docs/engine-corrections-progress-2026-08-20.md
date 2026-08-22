# Corrections moteur, dyno et acoustique — journal du 20 août 2026

Ce document complète l'audit du 10 août. Il décrit ce qui est effectivement
livré sur `codex/engine-corrections-roadmap`, les preuves exécutées et les
limites qui restent ouvertes. Un test vert signifie ici que le contrat décrit
est respecté ; il ne transforme pas une hypothèse acoustique en donnée mesurée.

## Points de contrôle publiés

| Commit | Contenu | État |
|---|---|---|
| `bad7d4b` | graphe d'échappement canonique, source afterfire locale, débit signé par sortie, terminaison passive et moniteurs physiques | poussé |
| `bc47b1d` | compteurs de pertes complets et oracle de transfert déterministe | poussé |
| `dc6f022` | échantillon de frein autoritaire par cycle moteur et estimateur dyno commun | poussé |
| `be93990` | rampe dyno rééchantillonnée sur une grille exacte de 50 tr/min | poussé |
| `c3faf6e` | sessions dyno immuables, archive persistante, statuts et calibration épinglée | poussé |
| `91b5ca8` | compliance passive des volumes compacts de jonction | poussé |
| `5cf2d55` | conduits coniques distribués sous budget global symétrique | poussé |
| `9c4ff1f` | branches latérales résonantes passives, bornées à huit | poussé |
| `8b4f980` | monolithes de catalyseur homogénéisés | poussé |
| `f865a7b` | noyau perforé, volume annulaire et garnissage passif | poussé |
| `afbc3e2` | preuves, budget et limites du silencieux passif | poussé |
| `7db0b8e` | harness afterfire aligné sur la calibration réellement authorée | poussé |
| `9f4fa96` | induction cohérente, source thermoacoustique audible, afterfire borné et instrumenté | poussé |
| `2465157` | fixtures de mesure périmées réparées sans relâcher les seuils | poussé |
| `778c70f` | preuves et limites de l'afterfire physique documentées | poussé |
| `cdb8ba2` | énergie d'arbre turbo conservée et autorité aval mesurée | poussé |
| `f008e2c` | induction afterfire thermochimique explicitement paramétrée | poussé |
| `4c74f61` | X-pipe directionnel passif à quatre ports, schéma 9 et métrologie CFL complète | poussé |

## Dyno livré

La mesure n'est plus reconstruite depuis une valeur instantanée de fin de
frame. `EngineSimulator` publie chaque cycle 720 degrés terminé avec son travail
de frein, son temps, son angle intégré et son régime moyen. Le produit et les
harness consomment le même événement.

Le chemin de rampe utilise ensuite une fenêtre roulante causale de 0,25 s et
interpole sur une grille RPM explicite. Les fenêtres non chevauchantes de
0,25–0,80 s, qui rendaient la densité dépendante du nombre de cylindres, ne sont
plus le cadenceur de l'enregistrement.

Résultats Release du chemin produit complet :

| Moteur | Plage | Pas | Points valides | Résultat |
|---|---:|---:|---:|---|
| CP2-like Twin | 1 800–9 500 tr/min | 50 tr/min | 155 | grille exacte, aucun bin invalide |
| LS3-like V8 | 1 120–6 270 tr/min | 50 tr/min | 104 | grille exacte, aucun bin invalide |

Les runs portent maintenant leur mode, leur protocole, la révision de
calibration, leur statut terminal et leur raison d'arrêt. L'archive appartient
à l'application, donc les identifiants et les runs survivent au remplacement de
`EngineRuntime`. La calibration ECU acceptée est épinglée pendant la session ;
une édition concurrente ne peut plus fabriquer une courbe hybride.

## Volume compact des jonctions

### Défaut reproduit

`CompiledExhaustJunction::volumeM3` alimentait le réseau de gaz, mais n'était
jamais lu par `AcousticExhaustNetwork`. Deux splitters ayant les mêmes tubes,
sorties, source, milieu et observateur, avec respectivement 0,25 L et 2,0 L,
produisaient donc des buffers audio identiques.

Le nouveau cas de `EngineLab.ExhaustTransfer` a d'abord échoué avec :

```text
changing only the compact junction volume did not change its acoustic transfer
```

### Modèle retenu

Une jonction compacte reçoit une compliance acoustique passive :

```text
C  = V / (rho c²)
Yc = 2 C / T
```

`Yc` est le port de compliance trapézoïdal du scattering en ondes. Il conserve
un seul scalaire d'état par jonction. Il n'ajoute ni maille gaz, ni ligne de
délai, ni sous-pas, ni allocation dans le callback.

Un tronc de merge/splitter ayant une longueur explicite est déjà un guide
d'onde distribué. Son volume balayé `A × L` est donc soustrait du volume de
contrôle avant de créer la compliance ; sinon le même tube serait compté une
fois comme délai et une seconde fois comme cavité. Seul le volume compact
résiduel reste au nœud many-port. Les volumes de jonctions réellement
coïncidentes s'additionnent.

### Oracle après correction

À 48 kHz, source fixe d'un pascal, milieu et observateur figés :

| Contrat | Mesure |
|---|---:|
| délai ajouté par 600 mm | 54,000 samples, attendu 53,832 |
| forme chambre d'expansion contre tube | 5,823 dB |
| forme split asymétrique contre tube | 6,568 dB |
| forme splitter 0,25 L contre 2,0 L | 6,434 dB |

Les six réponses testées restent finies, non vides, sous le plafond passif du
harness et avec une queue finale inférieure à 1 % de leur énergie observée.
Deux rendus frais du bypass restent bit-identiques.

### Budget temps réel frais

Scénario produit 48 kHz / 256 samples, physique 240 Hz, régime tenu à 55 % du
redline, 8 s de chauffe et 20 s mesurées :

| Moteur | Facteur | DSP moyen | DSP p99 | Violations du contrat audio |
|---|---:|---:|---:|---:|
| LS3-like V8 | 1,000 | 30,4 % | 33 % | 0 |
| Merlin-like V12 | 1,000 | 41,9 % | 50 % | 0 |

Le contrat compte séparément pertes firing/pression/acoustique/réaction,
événements tardifs/pending, voix volées, délais tronqués, frontières invalides,
chemin legacy, garde-niveau, non-finitude et rendu hors budget. Tous les
compteurs sont restés à zéro sur ces deux passages.

## Tapers distribués sous budget explicite

### Défaut reproduit

Le layout conservait bien l'aire d'entrée, l'aire de sortie et l'aire moyenne,
mais l'audio créait une seule ligne uniforme. La variation de section
n'existait qu'aux deux admittances terminales : aucun scattering intermédiaire
ne distinguait un cône fini d'une transformation concentrée.

Le nouveau fixture 42→84 mm sur 750 mm exige plusieurs sections acoustiques,
un délai total conservé et une réponse passive. Il échouait d'abord avec :

```text
a finite taper must contain distributed acoustic sections, not only endpoint areas
```

### Approximation retenue

Le rayon est interpolé le long du cône et chaque section conserve exactement le
volume de son frustum. Les sections se raccordent par les mêmes jonctions
d'admittance passives que le reste du graphe. Une ligne constante reste une
seule ligne. Un taper trop court pour des tronçons de 25 mm n'est pas sur-maillé.

Le maximum local est quatre sections, mais le budget est global : au plus seize
lignes supplémentaires par réseau. Les raffinements sont distribués par tours
complets entre les branches équivalentes. Ainsi douze sorties identiques ont
toutes deux sections ; le compilateur ne raffine jamais les premières sorties
en laissant les dernières différentes.

Ce budget vient d'une mesure, pas d'une estimation : quatre sections sur chacun
des douze stacks du Merlin faisaient monter le DSP moyen de 41,9 % à 61,4 % et
ont produit 3 puis 1 blocs hors budget sur deux passages. Cette version a été
rejetée. Avec deux sections symétriques par stack, le passage final de 20 s
mesure 48,4 % moyen, 54 % p99, facteur 1,000 et zéro violation.

Oracle fixe après correction :

| Contrat | Mesure |
|---|---:|
| taper 42→84 mm contre tube de contrôle | 1,123 dB de forme |
| sections du taper isolé | 4 |
| graphe 12 sorties | 25 lignes : 1 tronc + 12 × 2 |
| violations temps réel Merlin final | 0 |

Les injections de réaction conservent leur position axiale : la coordonnée
globale choisit désormais la section puis sa position locale. Un smoke produit
afterfire/audio a gardé `physical=yes`, `compiled=yes` et tous les compteurs de
livraison à zéro.

## Résonateur terminal passif

### Sémantique du graphe

Un composant `resonator` avec une entrée et une sortie garde son comportement
historique : c'est un conduit inline, dont la géométrie est résolue par le guide
d'onde. Le même composant avec exactement une connexion entrante et aucune
sortie est désormais une branche latérale fermée. Elle est exclue du DAG de
débit moyen, parce qu'une branche scellée ne transporte aucun débit permanent,
mais elle est raccordée au nœud acoustique aval du composant source.

La branche ajoute une seule ligne de délai bidirectionnelle. Avec `volume_l =
0`, son extrémité a une réflexion de pression `+1`, soit une branche quart
d'onde fermée. Un volume positif ajoute à l'extrémité le même port de compliance
WDF passif que les volumes compacts. Il n'y a ni cellule gaz, ni sous-pas, ni
allocation dans le callback.

`resonance_hz = 0` conserve la longueur géométrique écrite. Une valeur positive
définit une longueur acoustique de référence `c_ref/(4 f)` ; le milieu local
chaud ou froid déplace ensuite naturellement l'accord, sans oscillateur ni EQ
posé après coup. Si un volume terminal est aussi renseigné, la valeur règle la
longueur du col et la cavité reste un second élément physique : elle ne promet
donc pas que le minimum final restera exactement à la fréquence saisie.

### Preuves de non-vacuité et de coût borné

Le fixture a d'abord échoué parce que le résonateur pendant entrait encore dans
le graphe gaz et déclenchait un fallback de topologie. Après compilation dédiée,
deux branches de même diamètre et même longueur, dont seule la consigne passe de
la longueur écrite à 500 Hz, diffèrent de **11,015 dB** sur la forme du transfert
fixe. Chacune ajoute exactement une ligne acoustique, reste finie et sous le
plafond passif de l'oracle. À longueur et diamètre constants, ajouter une cavité
terminale de 0,35 L change la forme de **11,825 dB** : le port de compliance
n'est donc pas un champ sérialisé sans effet.

Un test de graphe séparé vérifie qu'ajouter une branche ne change ni les nœuds,
ni les arêtes, ni les routes, ni le volume de contrôle, ni l'aire de sortie, ni
la restriction du solveur de débit moyen. Deux attaches ou un simple tube
pendant restent invalides : seule cette forme explicite de `resonator` obtient
la sémantique de branche. Le nombre est borné globalement à huit par moteur ; un
document validé au-delà est refusé, et le compilateur défensif tronque à huit en
émettant `acousticBranchLimitReached`. Le coût ne peut donc pas croître jusqu'à
la limite générale de 256 composants.

## Monolithe de catalyseur homogénéisé

### Défaut reproduit

Le type `catalyst` conservait sa longueur et sa perte locale dans le réseau de
gaz, mais ne portait aucune géométrie de substrat. À longueur, diamètre et
restriction identiques, activer une densité de 400 cpsi et une aire ouverte de
0,80 ne changeait d'abord aucun échantillon du transfert passif : le nouvel
oracle a échoué avec `authored catalyst cell geometry did not change the passive
transfer`.

### Modèle et conservation

Le substrat carré est défini par `catalyst_cell_density_cpsi`,
`catalyst_open_area_ratio` et une capacité thermique volumique effective. Le
pas vaut `0,0254/sqrt(cpsi)`, la largeur et le diamètre hydraulique du canal
valent `pas*sqrt(aire_ouverte)`. Le faisceau est homogénéisé en un conduit : sa
section de volume est la section du boîtier multipliée par l'aire ouverte,
tandis que le diamètre hydraulique du canal pilote le frottement de paroi gaz
et la perte thermo-visqueuse acoustique.

L'état de paroi par cellule axiale agrège la surface mouillée de tous les
canaux, la capacité du solide `(A_boîtier-A_ouverte)*L*Cvol` et celle de
l'enveloppe métallique. Sans cette agrégation, le nouveau diamètre hydraulique
aurait donné à tout le faisceau la masse thermique d'un seul micro-canal et
aurait artificiellement armé l'afterfire presque instantanément.

Ce cas a révélé un deuxième défaut, plus général : le flux de Riemann d'une
interface était comptabilisé avec l'aire extérieure du raccord alors que la
cellule aval pouvait avoir une aire plus faible. L'inventaire observé et la
masse réellement ajoutée divergeaient. Les frontières internes, soupapes et
sorties bornent maintenant le flux par le minimum de l'ouverture de raccord et
de l'aire de face. Le test de propagation à travers le catalyseur referme de
nouveau espèces et énergie contre les transferts aux frontières.

### Preuves et budget

À source, milieu, boîtier, longueur et `restriction` identiques, le monolithe
400 cpsi / 0,80 change la forme du transfert de **7,062 dB**. Les réponses
restent finies, non vides et sous le plafond passif. Un test de layout exige en
même temps : aire de débit `A*0,80`, diamètre hydraulique du canal, ouvertures
de boîtier inchangées, même nombre de conduits et même nombre total de cellules
gaz que le bypass.

Il n'existe ni ligne par canal, ni maille par cellule réelle, ni nouveau filtre
par échantillon : le filtre de perte de paroi déjà présent reçoit seulement le
diamètre hydraulique physique. Les trois champs à zéro gardent le chemin ancien
exact. Aucun moteur du catalogue ne les renseigne encore ; ce lot n'impose donc
pas une couleur arbitraire aux presets existants.

Mesure produit fraîche après reconstruction complète, 48 kHz / 256 samples,
8 s de chauffe et 20 s mesurées à 55 % du redline :

| Moteur | Facteur | DSP moyen | DSP p99 | Violations du contrat |
|---|---:|---:|---:|---:|
| LS3-like V8 | 1,000 | 30,6 % | 36 % | 0 |
| Merlin-like V12 | 1,000 | 48,3 % | 58 % | 0 |

Le Merlin a compté deux réveils tardifs du scheduler, mais aucun rendu n'a
dépassé la durée d'un bloc (`renderOver=0`) et aucun événement n'a été perdu.
La reconstruction Release intégrale puis les **42/42 CTest** ont passé en
842,87 s, y compris les rampes dyno produit CP2 et LS3.

## Silencieux à noyau perforé et volume annulaire

Le silencieux garni confondait son volume brut avec le conduit de débit : un
noyau de 76 mm dans un corps de 142 mm devenait un tube gaz proche de 142 mm.
Le layout conserve maintenant le noyau comme section quasi-1D, soustrait son
volume balayé du corps brut et publie la différence comme volume annulaire
acoustique. Le résumé du chemin exclut lui aussi ce volume scellé.

La fraction ouverte couple ce volume à une compliance passive répartie aux
deux extrémités du noyau ; Delany-Bazley reste la perte matérielle indépendante.
Le modèle réutilise les états WDF des jonctions et n'ajoute ni cellule gaz, ni
ligne de délai. Faire varier seulement le corps de 4 à 8 L change la forme du
transfert de **5,138 dB** avec un nombre de conduits identique.

Sur le LS3 à 4 000 tr/min, retirer seulement le silencieux fait passer la
pression observateur de 30,1 à 89,1 Pa, soit **+9,41 dB**. Doubler puis
quintupler environ le volume donne respectivement 6,83 et 13,49 dB d'écart de
forme sur la couche échappement. Le mix complet ne bouge presque pas en niveau
large bande parce que les autres couches masquent 16/28 bandes ; ce résultat
est documenté comme un problème séparé, pas compensé par un gain arbitraire.

La correction du vrai noyau augmente le coût CFL du LS3 à ~26 880 sous-pas/s.
Un profil a isolé un prédicat `std::isfinite` à 18,74 % du réseau gaz. Son
équivalent exact par masque d'exposant IEEE-754 fait passer l'A/B immédiat de
0,932/0,935 à 1,102/1,108×, sans changer la cadence, les seuils physiques ou les
résultats publiés. Avec audio produit au régime haut, le LS3 atteint 1,018× et
le Merlin 1,367× ; le contrat temps réel complet reste à zéro violation. La
marge LS3 de 4,8 % reste un gate à préserver, pas un budget à dépenser. Les
détails, équations, oracles et limites sont dans
`docs/passive-muffler-implementation-2026-08-20.md`.

Le smoke afterfire qui suit ce lot garde la topologie physique active et tous
les compteurs à zéro. Le lot suivant a toutefois découvert que le harness
imposait silencieusement 900 K alors que le moteur catalogué écrit 800 K : la
première mesure à 327,8 °C après 30 s n'était donc pas une mesure du « seuil
produit ». Avec la calibration réellement écrite et 60 s de charge, la paroi
atteint environ 517 °C ; 5,0 à 5,6 mg brûlent en six à sept excursions, mais le
chemin audio ne gagne qu'environ 0,8 dB au pic et 0,2 dB au percentile 99,9 face
au contrôle sans carburant. Le contrôle instrumental à 520 K livre bien dix
réactions et 22,523 mg brûlés jusqu'à l'observateur, mais ce seuil forcé n'est
pas une correction proposée. Il sépare seulement le chemin de livraison sain
du problème d'allumage et de rayonnement acoustique encore ouvert.

La reconstruction Release a relié l'application et les harness ; les
**42/42 CTest** ont passé en 860,90 s après ce lot.

## Afterfire local, audible et borné

L’ancien chemin calculait une source depuis la chaleur, puis la passait deux
fois dans un passe-haut à environ 3–6 kHz. Il supprimait ainsi presque toute la
bande d’une réaction de quelques millisecondes et divisait en plus l’amplitude
par deux avant que le réseau ne partage déjà le saut entre deux ondes. L’A/B
WAV strict plaçait la composante directe à −57,747 dB du mix, avec un maximum
d’un LSB de différence.

La nouvelle source emploie le saut compact
`Delta p = (gamma - 1) Qdot / (A c)`, une reconstruction anti-imaging LR8 et
un bloqueur continu à 25 Hz. Elle reste au lieu de réaction et traverse le DAG
passif. Les coefficients ne sont plus recalculés par échantillon ; seules les
voix actives filtrent, dans un tableau fixe de 64 entrées.

Deux erreurs de chimie ont été corrigées en même temps : le délai d’induction
n’est plus multiplié par une rampe cachée de 450 K, et l’origine paroi/gaz est
mémorisée à l’allumage. Le simulateur n’appelle plus la chimie simplement parce
qu’une calibration existe : il faut un DFCO afterfire ou un rupteur humide
réellement actif. Le contrôle chargé de 500 ms avant lever mesure désormais
zéro événement et devient invalidant sinon.

Sur le Twin laboratoire catalogué à 2 ms, 60 s de chauffe puis 8 s d’overrun :

| Mesure | Sources audio supprimées | Sources audio actives |
|---|---:|---:|
| trajectoire / chaleur / carburant | identiques | identiques |
| événements avant lever | 0 | 0 |
| événements chaleur | 27 | 27 |
| carburant brûlé | 63,532 mg | 63,532 mg |
| crête audio | 0,00348 | 0,00518 |
| crest factor | 4,48 | 6,41 |
| source directe / mix overrun | — | −12,436 dB |
| limite 100 kPa / pertes / leveler | 0 | 0 |

Le rendu actif consomme 13,6 % du budget moyen d’un bloc de 200 samples et
14,6 % au p99, contre 13,1 % et 13,9 % pour le contrôle ; aucun des 1 920 blocs
n’a dépassé sa durée. Les gates produit de 20 s restent valides à 1,053× sur le
LS3 et 1,449× sur le Merlin, avec zéro violation. Le détail des causes, équations,
artefacts et limites se trouve dans
`docs/afterfire-implementation-2026-08-20.md`.

La reconstruction intégrale a aussi exposé trois défauts latents : le sweep
turbo fabriquait un silencieux perforé impossible au lieu de ne changer que la
sortie ; le Big Twin était devenu marginalement hors de son enveloppe fabricant
avec la mesure dyno par cycle ; et le rendu catalogue diesel appelait stabilisée
une fenêtre encore traversée par le remplissage admission. Les trois corrections
passent isolément sans desserrer les gates : sortie seule et exceptions lisibles,
turbulence Big Twin 0,50 → 0,42 (erreurs finales −3,48 % / +3,77 %), puis fenêtre
catalogue 3 → 4 s (crest EA288 18,42 → 7,21).

Après reconstruction des exécutables touchés, le second passage autoritaire
termine à **42/42 CTest**, zéro échec, en **735,23 s**. Les deux rampes dyno
produit CP2/LS3 passent en fin de suite.

## Arbre turbo sans puits d'énergie caché

Le turbo tronquait l'arbre à 1,16 fois sa vitesse de conception puis tronquait
sa loi de tête à 1,12 fois. Sur le 2JZ stressé, 14,9 à 23,7 kW disparaissaient
ainsi en continu : 150 800 tr/min et 2,154 de boost restaient exactement fixes
alors que la turbine passait de 32,71 à 41,60 kW.

Le bilan est maintenant intégré directement en énergie cinétique : turbine
moins travail compresseur moins pertes de palier. La tête centrifuge continue
avec le carré de la vitesse et n'est plus multipliée directement par le
papillon. Vitesse relative, trois puissances et puissance nette sont visibles
dans `EngineState`, le panneau debug et le harness.

En stock, les quatre points restent à 1,000–1,006 fois la vitesse de conception,
1,921–1,932 de boost et 0,02–0,07 kW de bilan net. Le stress explicitement hors
calibration traverse l'ancien clamp à 1,618–1,626 fois, retrouve 0,022 de
sensibilité au downstream et ferme encore son bilan à 0,01–0,09 kW. Ce stress
prouve la conservation, pas la précision d'une carte compresseur inexistante.

Les tests `Core`, `GasExchange`, `TurboDownstreamAuthority`,
`CatalogReference`, `AudioRender`, `AudioTransients` et
`AudioShiftTransientBoosted` passent. Le 2JZ tient **1,426x** temps réel avec
audio, DSP p99 45 % et tous les compteurs à zéro. Le détail se trouve dans
`docs/turbo-shaft-energy-implementation-2026-08-20.md`.

La reconstruction Release intégrale puis la suite autoritaire terminent à
**42/42 CTest**, zéro échec, en **765,38 s**. Les tests audio et transitoires
ainsi que les rampes dyno produit CP2/LS3 sont compris dans ce passage.

## Induction d'afterfire thermochimique et explicite

L'induction locale était encore un chronomètre : une fois le seuil franchi,
901 K ou 1 150 K, 101 ou 180 kPa, `phi=1` ou `phi=0,5` attendaient tous
exactement 4,00 ms. Le schéma 8 peut maintenant authorer une intégrale de
Livengood-Wu normalisée, avec délai et pression de référence, `Ea/R`, exposants
pression/richesse et durée de décroissance explicites. Les anciens schémas
migrent avec des exposants nuls et restent bit-sémantiquement plats.

Le sweep final mesure respectivement **3,95 / 0,20 / 2,25 / 5,90 ms**. Sur le
Twin laboratoire, l'A/B même binaire `--flat-induction` mesure une plage locale
0,001–6,543 ms au lieu de 4,000 ms fixe, 72,092 mg brûlés au lieu de 63,532 mg
et 10,547 kW au lieu de 9,244 kW. Le crest audio baisse honnêtement de 6,41 à
5,67 sans gain compensatoire ; la source demeure identifiable, sans limite de
pression, perte de queue, leveler ni bloc hors budget. Le pas physique mesure
20,3 % / 23,4 % du budget moyen / p99 contre 20,2 % / 23,5 % pour le timer
plat ; la corrélation n'ajoute donc aucun coût discernable dans ce passage. Le
détail et les limites de provenance se trouvent dans
`docs/afterfire-induction-implementation-2026-08-22.md`.

La reconstruction Release intégrale passe, application comprise, puis la suite
autoritaire termine à **42/42 CTest**, zéro échec, en **719,78 s**. Les tests
dyno produit CP2/LS3, audio, overrun thermique, catalogue et turbo sont compris
dans ce passage.

## X-pipe directionnel et identité des bancs

Le LS3 utilisait encore un `merge -> splitter` comme pseudo-X. L'oracle de
réaction a prouvé que cette écriture détruisait complètement l'identité des
bancs : les deux injections miroir devenaient identiques, avec un écart de
`-318,813 dB` à la précision numérique.

Le schéma 9 ajoute un `crossover` compact avec deux entrées, deux sorties,
appariement explicite des ports et couplage de puissance `k`. Le guide d'onde
emploie une matrice quatre ports réelle, réciproque et orthogonale dans les
coordonnées `sqrt(Y) p`. À `k=0,30`, les routes conservent exactement les parts
de puissance `0,91 / 0,09`, en miroir entre les deux bancs. Le nouvel oracle
mesure `3,0189 dB` d'identité résiduelle, avec des biais gauche/droite opposés de
`+8,68156 / -7,49687 dB` selon le banc excité.

Le solveur gaz garde un seul contrôle bien mélangé de volume `2 A d`. La
métrologie coût inclut désormais la vraie borne de jonction `V/sum(A_port)` en
plus du `dx` des conduits, et le moteur, l'éditeur et les harnesses partagent une
seule politique de maille temps réel. Le LS3 livré expose 16 conduits, 3
jonctions, 26 cellules et une longueur CFL limitante de 40 mm.

Le premier callback recalculait quatre racines d'admittance à chaque échantillon
et a révélé un passage chaud à `0,994×`. Il n'a pas été retenu. Les cibles sont
maintenant calculées une fois par bloc puis interpolées dans le domaine
`sqrt(Y)`, ce qui conserve le scattering passif et remonte deux passages
consécutifs à **1,025×**. DSP moyen `34,9–35,0 %`, p99 `56 %`, sous-pas gaz
`26 880 Hz`, et tous les compteurs audio restent à zéro. Le gate 0,97 passe,
mais les 2,5 % de marge sur le temps réel restent une contrainte.

Le rendu produit à 5 000 tr/min, sans référence externe, mesure 6,32 dB de
forme mix et 7,98 dB sur l'échappement entre le X livré et les deux 4-en-1 sans
section commune ; les tubes indépendants atteignent 8,08 / 15,15 dB. AGC 1,000,
zéro échantillon limité. Cela prouve l'autorité du graphe dans la chaîne livrée,
pas une fidélité absolue au réel.

Après l'optimisation finale, la reconstruction Release intégrale passe et la
suite autoritaire termine à **42/42 CTest**, zéro échec, en **727,16 s**. Les
deux rampes dyno produit sont incluses. `EngineLab.exe` reste vivant après un
smoke caché de cinq secondes. Toutes les équations, mesures intermédiaires et
limites sont consignées dans
[`x-pipe-directionnel-implementation-2026-08-22.md`](x-pipe-directionnel-implementation-2026-08-22.md).

## Livraison Windows du lot X-pipe

Le ZIP CPack final est reconstruit après le journal et vérifié depuis une
extraction neuve, pas depuis l'arbre de build. Le contrôle exige :

- 121 entrées, dont les 16 fichiers moteur ;
- `EngineLab.exe`, les deux outils CLI et les catalogues de pièces/voicings ;
- le document X-pipe détaillé dans `docs/` ;
- un seul exécutable principal, dont le SHA-256 doit être identique à celui de
  l'arbre Release ;
- processus extrait encore vivant après cinq secondes, puis arrêté
  volontairement.

La copie de ce journal embarquée dans le ZIP omet volontairement la taille et le
SHA-256 finaux du ZIP : les ajouter puis reconstruire changerait le hash à
l'infini. La copie de travail externe les fixe après la dernière génération.

Artefact externe final :

- ZIP : `out/build/windows-vs2022/EngineLab-0.1.0-win64.zip` ;
- taille : **6 571 930 octets** ;
- SHA-256 :
  `F7D0AB914EA857CB96499CF29CBF9880E0C33F003017CAABB2E7D0C6FE838DDB` ;
- SHA-256 de l'exécutable extrait :
  `17BC98CCA3687B7F926CE89BE1493988316CDECCF38BF5FE1BDCFE5543C7D93F` ;
- extraction contrôlée dans `out/validation-2026-08-22-xpipe-final/` ;
- smoke extrait vivant après cinq secondes, puis arrêt volontaire.

## Matrice actuelle des champs du graphe

Cette matrice évite de confondre un champ sérialisé avec une influence physique
effective.

| Champ/type | Gaz | Acoustique physique | État |
|---|---|---|---|
| longueur | volume/CFL et frottement distribué | délai et pertes de paroi ; tronc de merge/splitter conservé | actif |
| diamètre entrée/sortie | sections et faces quasi-1D | taper distribué 1–4 sections sous budget global | corrigé dans ce lot |
| volume d'un conduit | section interne de chambre | section interne et deux sauts d'aire | actif |
| volume d'une jonction | contrôle bien mélangé | compliance compacte résiduelle | corrigé dans ce lot |
| packing + perforation | noyau seul ; annulus scellé exclu | perte poreuse + compliance annulaire passive | corrigé dans ce lot |
| position/axe/terminaison de sortie | frontière de débit indirecte | délai, directivité, radiation et perte de lèvre | actif |
| restriction | perte de charge locale | effet indirect par pression/débit, pas une impédance complexe | libellé clarifié |
| `acousticGain` | aucun | fallback reconstruit seulement, jamais le guide d'onde SI | libellé legacy explicite |
| `resonanceHz` | aucun | accorde la longueur de référence d'un `resonator` terminal seulement | corrigé sans oscillateur |
| type `catalyst` | aire ouverte, diamètre hydraulique, frottement et perte locale | même conduit passif, pertes de canal thermo-visqueuses | corrigé sans maille par canal |
| type `resonator` | conduit inline, ou aucun débit s'il est terminal | conduit inline, ou branche fermée quart d'onde avec cavité optionnelle | corrigé dans ce lot |
| type `crossover` | jonction bien mélangée `2 A d`, perte locale authorée et ports conservés | matrice directionnelle passive quatre ports, sans plénum commun | corrigé, `k` catalogue encore estimé |

## Prochain ordre de travail

1. Composer les silencieux plus complexes avec chambres et branches explicites
   lorsque leurs dimensions sont connues ; le noyau perforé de base est livré.
2. Ne pas simuler un H-pipe par un X : une branche transversale littérale exige
   une topologie cyclique et un solveur acoustique adapté. Ce chantier ne se
   justifie que lorsqu'une géométrie à authorer ou mesurer est disponible.
3. Ne remplacer la loi de similitude turbo par une carte compresseur que si les
   lignes débit/vitesse/rendement possèdent une provenance ; l'UI marque déjà
   toute vitesse au-dessus du point de conception.
4. Rejouer les oracles, les tests produit et le budget temps réel à chaque lot,
   puis produire l'artefact exécutable final.
