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

## Prochain ordre de travail

1. Composer les silencieux plus complexes avec chambres et branches explicites
   lorsque leurs dimensions sont connues ; le noyau perforé de base est livré.
2. Reprendre l'afterfire maintenant que ces transferts existent : distribution spatiale,
   variabilité de l'allumage et énergie locale, sans échantillon de pop.
3. Rejouer les oracles, les tests produit et le budget temps réel à chaque lot.
