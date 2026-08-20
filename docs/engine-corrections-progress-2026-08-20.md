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

## Matrice actuelle des champs du graphe

Cette matrice évite de confondre un champ sérialisé avec une influence physique
effective.

| Champ/type | Gaz | Acoustique physique | État |
|---|---|---|---|
| longueur | volume/CFL et frottement distribué | délai et pertes de paroi ; tronc de merge/splitter conservé | actif |
| diamètre entrée/sortie | sections et faces quasi-1D | taper distribué 1–4 sections sous budget global | corrigé dans ce lot |
| volume d'un conduit | section interne de chambre | section interne et deux sauts d'aire | actif |
| volume d'une jonction | contrôle bien mélangé | compliance compacte résiduelle | corrigé dans ce lot |
| packing + perforation | ignoré volontairement | perte poreuse distribuée opt-in | actif |
| position/axe/terminaison de sortie | frontière de débit indirecte | délai, directivité, radiation et perte de lèvre | actif |
| restriction | perte de charge locale | effet indirect par pression/débit, pas une impédance complexe | libellé clarifié |
| `acousticGain` | aucun | fallback reconstruit seulement, jamais le guide d'onde SI | libellé legacy explicite |
| `resonanceHz` | aucun | accorde la longueur de référence d'un `resonator` terminal seulement | corrigé sans oscillateur |
| type `catalyst` | perte géométrique et locale | aucun monolithe acoustique dédié | à corriger |
| type `resonator` | conduit inline, ou aucun débit s'il est terminal | conduit inline, ou branche fermée quart d'onde avec cavité optionnelle | corrigé dans ce lot |

## Prochain ordre de travail

1. Modéliser le monolithe de `catalyst` comme une impédance passive bornée, avec
   un coût indépendant du nombre de canaux réels.
2. Construire les silencieux comme petits assemblages passifs composables
   (chambres, noyau perforé, branches accordées), sans augmenter la maille gaz.
3. Reprendre l'afterfire seulement après ces transferts : distribution spatiale,
   variabilité de l'allumage et énergie locale, sans échantillon de pop.
4. Rejouer les oracles, les tests produit et le budget temps réel à chaque lot.
