# Authoring, calibration et préparation graphique

Le nom de ce fichier est conservé pour les liens historiques. Il résume les
outils d'auteur ajoutés autour du simulateur et sépare clairement ce qui est
opérationnel de ce qui reste à construire.

## Livré : calibration ECU transactionnelle

`EngineLabCalibration` introduit des métadonnées, unités et quantités d'axes
typées, des scalaires, courbes 1D et tables 2D. Les interpolations sont clampées
aux bords ; une coordonnée de mauvaise quantité ou unité est refusée.

Un brouillon mutable n'est jamais lu directement par le runtime. Après
validation complète, `CalibrationStore` publie un snapshot immuable versionné
par échange atomique. Le contrôle de révision empêche deux éditeurs de
s'écraser. Une transaction invalide conserve exactement le pointeur actif. Les
epochs lecteurs reportent la destruction des snapshots remplacés sur le thread
de publication.

La fenêtre JUCE **ECU** permet d'éditer les cartes AFR/avance et le rupteur,
affiche la cellule proche du point de fonctionnement et charge/enregistre le
schéma `.ecu.json`. Une modification de cellule ou de fichier valide est
appliquée sans recréer le moteur. Les axes de charge normalisée couvrent jusqu'à
400 % pour ne pas écraser les zones de suralimentation sur la colonne 100 %.

Voir [ecu-tuning.md](ecu-tuning.md) pour le format, les effets actifs et les
limites.

## Livré : DSL moteur et watcher de dépendances

`EngineLabScripting` compile un langage déclaratif avec :

- presets ou base JSON/YAML ;
- inclusions relatives et suivi déterministe des dépendances ;
- variables, expressions simples et unités contrôlées ;
- affectations globales ou par identifiant de cylindre ;
- remplacement de la courbe d'allumage ;
- diagnostics fichier/ligne/colonne, limites de taille/profondeur et détection
  des cycles.

Le watcher compile sur son worker et publie un `EngineScriptReloadState`
immuable. Une erreur conserve la dernière configuration valide. L'application
importe `.els` et `.engine`, surveille le script, ses inclusions et son fichier
de base puis applique les nouvelles révisions.

Ce reload remplace le runtime : il évite de relancer l'application, pas de
réinitialiser le moteur. Le détail et la comparaison avec `.mr` sont dans
[engine-scripting.md](engine-scripting.md).

## Livré : scène prête pour un backend 3D

`EngineLabRender` convertit une configuration et un état en scène bornée, sans
dépendance graphique. Vilebrequin, journaux, cylindres, pistons, bielles et
soupapes reçoivent identifiants et transformations 3D. Les cylindres sont
distribués sur l'axe Z et un interpolateur à deux snapshots traite les angles
circulaires correctement.

L'application construit déjà ces snapshots à son rythme UI. Toutefois, la vue
visible reste le dessin JUCE 2D existant et ne consomme pas encore
`IEngineRenderer`. Aucun contexte, shader, mesh ou appel OpenGL n'est présent.

La base rend le passage en 3D raisonnable : le futur backend pourra rester un
consommateur de snapshots au lieu d'accéder au simulateur. Elle n'élimine pas
le travail graphique listé dans
[architecture.md#préparation-du-rendu-3d](architecture.md#préparation-du-rendu-3d).

## Livré : séparation audio renforcée

Les cylindres, événements et trames de pression conservent leur indice de
chemin. Chaque chemin possède ses états de guide d'onde, réflexion, FDN et
convolution. Pression ambiante, fréquence du périphérique et temps producteur
sont pris en compte explicitement. Des tests de régression isolent les chemins
et comparent les délais à 48, 96 et 192 kHz.

Le résultat technique est plus robuste que l'ancienne sommation globale, mais
la supériorité sonore ne sera considérée acquise qu'après un corpus de référence
et des écoutes à niveau égalisé. Voir [realtime-audio.md](realtime-audio.md).

## Livré : topologie d'échappement auteur

Chaque chemin peut désormais porter un DAG sérialisable de pipes, merges,
splitters, résonateurs, silencieux, catalyseurs et sorties. La validation impose
des identifiants uniques, les cardinalités de jonction, l'absence de cycle,
l'accessibilité de tous les composants et une sortie pour chaque route.

Le compilateur calcule les pertes en série/parallèle et les métriques de chaque
route cylindre-sortie. Les fichiers moteur v1 restent acceptés et migrés ; les
exports utilisent le schéma v2. Le mode géométrique historique reste le fallback
quand aucun graphe auteur n'est présent.

Ces pertes servent à l'atténuation audio et à la physique. Un DAG auteur est
réduit en gorge d'entrée par cylindre, volume/longueur par chemin et conductance
de sorties parallèles corrigée par son `K` ; ces valeurs reparamètrent les
transferts conservatifs. La résolution reste toutefois agrégée à une cellule
runner par cylindre et une cellule collecteur par chemin.

La fenêtre **ECHAP. PRO** édite une copie de travail : génération depuis les
champs historiques, ajout et paramétrage des sept types de composant,
connexions, affectations cylindre et vue du flux. Une validation complète est
requise avant l'application, qui remplace le runtime comme tout changement
structurel. JSON/YAML reste le format de persistance et le moyen de gérer les
chemins, leurs cylindres et leurs IR. Le format, le workflow et ses limites
sont décrits dans [custom-exhaust.md](custom-exhaust.md).

## Encore limité

### ECU

Seules deux tables et un scalaire pilotent actuellement le runtime. Il manque
les cartes VE, lambda en boucle fermée, démarrage, enrichissements, VVT/VVL,
boost, wastegate, torque management, datalogging, sélection multi-cellules,
lissage et historique/undo d'un tuner complet.

### Script

Le DSL ne crée pas encore une bibliothèque de types composables comparable à
Piranha. Il ne possède ni nœuds utilisateurs, ni fonctions, boucles,
conditions, tableaux de pièces ou génération arbitraire de topologie. Les
fichiers `.mr` ne sont pas importables.

### Audio et échappement

Le renderer reste un modèle perceptuel agrégé. Il manque un solveur 1D par
segments/composants, le glisser-déposer et l'undo/redo dans le concepteur, une
bibliothèque de pièces mesurées, des positions de microphone et un workflow A/B
automatisé. Gain, délai, ouverture et réflexion du signal continu suivent les
métriques combinées du DAG, mais ses branches restent agrégées avant un unique
guide d'onde et une unique IR par chemin.

### Rendu

Le contrat de scène ne contient pas encore admissions, collecteurs, turbo,
accessoires, matériaux ou effets. Aucun backend GPU n'est livré et les
performances d'une future scène 3D ne sont pas mesurées.

## Critère de sortie honnête

Ces modules rendent l'architecture extensible et les modifications utilisateur
plus sûres. Ils ne suffisent pas à conclure qu'EngineLab égale ES2D sur son
écosystème de scripts, son catalogue ou son rendu sonore. La grille de
comparaison et les jalons restants sont maintenus dans
[es2d-targeted-gap-closure.md](es2d-targeted-gap-closure.md).
