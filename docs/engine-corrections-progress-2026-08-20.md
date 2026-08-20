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

## Matrice actuelle des champs du graphe

Cette matrice évite de confondre un champ sérialisé avec une influence physique
effective.

| Champ/type | Gaz | Acoustique physique | État |
|---|---|---|---|
| longueur | volume/CFL et frottement distribué | délai et pertes de paroi ; tronc de merge/splitter conservé | actif |
| diamètre entrée/sortie | sections et faces quasi-1D | admittances terminales, rayon moyen et chambre | actif, taper encore approché |
| volume d'un conduit | section interne de chambre | section interne et deux sauts d'aire | actif |
| volume d'une jonction | contrôle bien mélangé | compliance compacte résiduelle | corrigé dans ce lot |
| packing + perforation | ignoré volontairement | perte poreuse distribuée opt-in | actif |
| position/axe/terminaison de sortie | frontière de débit indirecte | délai, directivité, radiation et perte de lèvre | actif |
| restriction | perte de charge locale | atténuation de source réduite, pas une impédance complexe | à clarifier |
| `acousticGain` | aucun | gain de source authorisé | actif mais non géométrique |
| `resonanceHz` | aucun | métadonnée/fallback procédural, pas le guide d'onde produit | champ trompeur à corriger |
| type `catalyst` | perte géométrique et locale | aucun monolithe acoustique dédié | à corriger |
| type `resonator` | conduit inline | aucun branchement Helmholtz/quart d'onde dédié | à corriger |

## Prochain ordre de travail

1. Donner une sémantique non trompeuse à `resonator`, `resonanceHz`,
   `catalyst`, `restriction` et `acousticGain`, avec tests A/B non vacants.
2. Construire les silencieux comme petits assemblages passifs composables
   (chambres, noyau perforé, branches accordées), sans augmenter la maille gaz.
3. Reprendre l'afterfire seulement après ces transferts : distribution spatiale,
   variabilité de l'allumage et énergie locale, sans échantillon de pop.
4. Rejouer les oracles, les tests produit et le budget temps réel à chaque lot.

