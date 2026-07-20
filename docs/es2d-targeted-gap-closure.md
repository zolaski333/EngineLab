# Comparaison EngineLab / ES2D et trajectoire de parité

> **État antérieur à la refonte thermoacoustique.** Les tableaux audio ci-dessous
> servent de trace d’audit, pas de description du renderer actuel.

Date de l'analyse : 15 juillet 2026.

Cette comparaison porte sur EngineLab et sur le code présent dans
`C:\Users\Noah\Desktop\es2d`. Elle est volontairement plus prudente qu'une
comparaison de listes de fonctionnalités.

## Méthode et limite majeure

EngineLab a été inspecté, compilé et exercé par sa suite CTest et ses harnais.
Le dossier ES2D local contient son code, ses fichiers `.mr` et ses assets, mais
ses cinq répertoires de submodules (`piranha`, moteur de rendu, solveur de
contraintes, etc.) sont vides et le dossier n'est pas un checkout Git complet.
Une compilation ou un rendu A/B reproductible d'ES2D n'a donc pas été possible.

Les conclusions sur ES2D sont une analyse statique du code local. Aucun score
de performance, de précision ou de qualité sonore comparée n'est inventé. Le
README d'ES2D indique lui-même que le projet vise le son et la réponse moteur,
pas un usage scientifique.

## Comparaison point par point

| Critère | EngineLab actuel | ES2D local | Conclusion honnête |
|---|---|---|---|
| Architecture | Bibliothèques C++20 séparées, interfaces de modèles, runtime, SPSC, calibration, scripts et rendu neutre | Application plus monolithique, avec bibliothèques externes et un sous-système Piranha puissant | Avantage EngineLab pour la modularité et les frontières temps réel |
| Cinématique | Géométrie analytique commune, journaux explicites, bielles conventionnelles/maîtresses/articulées, multi-crank rigide | Corps 2D et contraintes mécaniques explicites pour crank/piston/rod | Approches différentes ; ES2D garde plus de maturité sur l'assemblage arbitraire, EngineLab est plus déterministe |
| Gaz | Volumes 0D, espèces, énergie, momentum 2D, pression dynamique signée, débit compressible, équilibre borné et croisement simultané | `GasSystem` 0D avec moles, énergie, momentum 2D et débit compressible | EngineLab a davantage d'invariants et de propriétés de mélange, mais aucun des deux n'est CFD/1D maillé |
| Couple et travail | Couple instantané de pression ; P·dV signé séparé pour IMEP/puissance | Forces chambre appliquées au système de contraintes et dyno intégré | EngineLab offre une observabilité énergétique plus explicite ; parité physique réelle non mesurée |
| Combustion | Délai d'allumage, vitesse laminaire/turbulente, volume brûlé `πr²h`, résiduels, parois et knock Livengood-Wu | Modèle de chambre/fuel paramétrable, fonctions de turbulence, rendement et délai dans les scripts | Avantage de profondeur apparente EngineLab, sans étalonnage expérimental suffisant pour conclure sur la précision |
| Injection | Port avec film persistant et DI avec refroidissement, rail et fenêtre | Modèle carburant/chambre plus agrégé | Avantage fonctionnel EngineLab |
| Suralimentation | Bilan de puissance turbine/compresseur, inertie et wastegate | Les fichiers inspectés ne montrent pas un modèle énergétique équivalent | Avantage EngineLab sur le noyau inspecté, à valider sur données |
| Transmission/véhicule | Embrayage thermique, boîte, différentiel, roue, pneu longitudinal, route et bilan d'énergie | Transmission, véhicule, dyno et contraintes intégrés, éprouvés dans l'UI | EngineLab est plus instrumenté ; l'expérience ES2D reste plus mature |
| ECU | Tables AFR et avance 2D jusqu'à 400 % de charge, rupteur, trims neutres, corrections, tuner et snapshots atomiques | Ignition module, governor et paramètres scriptés, sans éditeur de cartes comparable dans ce code | Avantage net EngineLab pour le tuning live |
| Reload ECU | Modification cellule/fichier sans recréer le moteur | Pas de frontière équivalente identifiée | Avantage EngineLab |
| Scripts moteur | DSL sûr `.els` : preset/base/include/let/set, unités, diagnostics et watcher automatique | Piranha `.mr` : nœuds, objets, fonctions, imports, bibliothèques de pièces, moteurs, véhicules, thèmes | Avantage net ES2D en expressivité et écosystème |
| Reload structurel | Surveillance automatique de toutes les dépendances, dernière version valide conservée, puis remplacement du runtime | `Entrée` recompile manuellement `assets/main.mr` puis recharge moteur/véhicule/transmission | EngineLab détecte et sécurise mieux ; les deux reconstruisent l'état dynamique |
| Échappement auteur | Jusqu'à huit chemins ; DAG pipe/merge/splitter/résonateur/silencieux/catalyseur/sortie, pertes série-parallèle, gorge/volume/conductance gazeux agrégés, concepteur validé et JSON/YAML | Un objet `ExhaustSystem` par chemin, facilement composé en `.mr`, avec longueur, flows, volume et IR | Outil visuel et données de topologie EngineLab plus riches, mais aucun des deux modèles comparés n'est ici un solveur 1D par composant ; ES2D garde un authoring textuel plus composable grâce au langage et à sa bibliothèque |
| Source audio | Pressions sous-pas + runner/flow + événements + admission/mécanique/distribution/démarreur/suralimentation | Débit échappement retardé par cylindre vers un canal de synthèse | EngineLab possède une source plus riche sur le papier |
| Traitement audio | Stéréo, routage par chemin, guides aller/retour, jonction, FDN, jitter, bruits, leveler et convolution partitionnée par chemin | Dérivée/signal brut, jitter, bruit d'air, anti-alias, convolution et leveler, avec un thread audio dédié | Architecture EngineLab plus avancée ; supériorité perceptuelle non démontrée et son actuel encore perfectible |
| Temps réel | Callback JUCE borné sans mutex/I/O/allocation, files SPSC et atomiques ; compteurs d'overflow/retard | Ring buffers, mutex/condition variables et thread de rendu audio | Avantage EngineLab en contrat temps réel vérifiable |
| Affichage | Vue JUCE 2D fonctionnelle ; snapshots 3D neutres produits mais inutilisés visuellement | Vue 2D dédiée, objets moteur et shaders intégrés au framework graphique | Avantage ES2D pour le rendu actuel ; aucune des bases inspectées ne livre la 3D demandée |
| Préparation 3D | Transformations XYZ, IDs, bounds, interpolation et `IEngineRenderer`, sans OpenGL | Rendu fortement lié au framework graphique historique | EngineLab est mieux préparé architecturalement, mais un backend complet reste à écrire |
| Catalogue | Presets intégrés et catalogue YAML de dix moteurs avec bibliothèques de pièces | Vaste bibliothèque `.mr` de moteurs, pièces, IR et thèmes | Avantage net ES2D en contenu et communauté |
| Formats et migration | JSON/YAML moteur v2, migration v1, calibration JSON v1, validation stricte | `.mr` riche mais dépendant de Piranha et de ses bibliothèques | EngineLab facilite les données structurées ; ES2D facilite la composition |
| Tests | Tests unitaires/intégration, transactions concurrentes, régressions audio, catalogue et harnais déterministes | Trois sources de tests visibles, mais dépendances manquantes localement | Avantage EngineLab dans l'arbre inspecté |
| Validation réelle | Invariants et tendances, aucune campagne banc/microphone certifiée | Projet explicitement non scientifique ; aucune campagne locale reproductible | Égalité : aucune preuve suffisante de précision absolue |
| Performance | Cadence adaptative et tests de stabilité ; pas de benchmark comparatif | Fréquence de simulation réglable ; exécutable local indisponible | Indéterminé |

## Où EngineLab est déjà devant techniquement

Les points les plus solides ne sont pas des impressions sonores :

- publication ECU transactionnelle sans reset ;
- diagnostics et dernier état valide pour les scripts ;
- conservation et télémétrie énergétique testées ;
- séparation multi-chemin jusqu'à la convolution ;
- callback audio avec contrat temps réel explicite ;
- formats structurés validés et migrés ;
- scène neutre prête à alimenter un futur renderer.

Ces qualités réduisent les bugs et rendent le projet plus facile à étendre.
Elles ne garantissent pas que l'utilisateur préférera immédiatement le son.

## Où ES2D reste devant

ES2D conserve trois avantages déterminants :

1. son langage `.mr` est un système de composition complet, pas seulement un
   langage de surcharge ;
2. sa bibliothèque de moteurs, pièces, IR et exemples est beaucoup plus vaste ;
3. son rendu et sa signature sonore bénéficient de davantage de recul et
   d'itérations publiques.

Le niveau global « égal ou supérieur » ne sera atteint que lorsque ces écarts
d'usage seront fermés, pas simplement lorsque le nombre d'équations sera plus
grand.

## Évaluation de la préparation OpenGL

Le passage à la 3D est possible dans l'état actuel sans refondre la simulation.
Le solveur ne dépend pas de pixels, et le module `render` transforme déjà son
état en scène 3D interpolable. C'est le bon découplage pour ajouter un backend
OpenGL ultérieurement.

Le projet n'est cependant pas « prêt à activer OpenGL » par une option CMake.
Il manque le contexte, les shaders, les meshes, matériaux, ressources, caméra,
sélection, synchronisation GPU et de nombreuses pièces de scène. La vue 2D
accède encore directement à `EngineState`. L'état correct est donc :

```text
architecture préparée : oui
backend OpenGL présent : non
migration sans toucher à la physique : probable
travail graphique restant : important
```

## Feuille de route vers un niveau égal ou supérieur

### 1. Prouver la stabilité actuelle

- figer des baselines déterministes par plateforme ;
- faire passer Release et sanitizers sur CI ;
- ajouter des stress tests longs de files, reloads et périphériques 44,1/48/96 kHz ;
- transformer la différenciation spectrale audio en critère mesuré et justifié,
  pas seulement un avertissement.

Critère de sortie : aucune valeur non finie, aucun clic de reload reproductible,
pas de drop dans le budget supporté et régressions quantifiées.

### 2. Rendre la qualité audio mesurable

- constituer un corpus I2/I4/I5/V6/V8/flat/radial/moto, atmosphérique et turbo,
  avec plusieurs charges et positions micro ;
- égaliser la sonie avant toute écoute A/B ;
- mesurer latence, bruit, aliasing, enveloppe, ordre moteur et différenciation ;
- organiser des écoutes en aveugle face à ES2D et aux enregistrements.

Critère de sortie : préférence et défauts documentés sur un protocole
reproductible. Sans cela, « audio supérieur » reste une intention.

### 3. Faire de l'échappement un vrai outil utilisateur

- faire évoluer le concepteur actuel avec glisser-déposer, undo/redo,
  édition des chemins/IR et diagnostics localisés en direct ;
- proposer une bibliothèque versionnée de tubes, jonctions, catalyseurs,
  résonateurs et silencieux ;
- ajouter comparaison, presets, écoute rapide et import d'IR ;
- faire évoluer l'acoustique agrégée vers des segments 1D avec impédances,
  température, pertes fréquentielles et jonctions physiques.

Critère de sortie : un utilisateur peut concevoir, valider, sauvegarder,
comparer et entendre deux échappements distincts sans éditer manuellement un
grand JSON. Le concepteur actuel couvre déjà la création et la validation du
DAG, mais pas encore tout ce workflow.

### 4. Approcher l'écosystème `.mr`

- permettre au DSL de construire banques, journaux, cames, chemins et pièces,
  pas seulement de les modifier ;
- ajouter des bibliothèques composables avec API versionnée ;
- fournir davantage d'exemples et de diagnostics d'éditeur ;
- décider explicitement entre un import `.mr`, un convertisseur partiel ou une
  incompatibilité assumée et documentée.

Critère de sortie : les familles de moteurs ES2D courantes peuvent être décrites
avec une quantité de code et un temps d'itération comparables.

### 5. Étendre le tuner ECU

- rendre actives les cartes VE, VVT/VVL, boost, wastegate, démarrage,
  température et enrichissement transitoire ;
- ajouter sélection multi-cellules, interpolation/lissage, undo, historique,
  diff de révisions et datalogger synchronisé ;
- conserver la publication atomique et définir une migration de calibration
  lors d'un reload structurel compatible.

Critère de sortie : workflow comparable à un tuner logiciel, tout en restant
clairement un outil de simulation.

### 6. Valider et calibrer la physique

- importer des courbes de flowbench et de cames mesurées ;
- comparer pression cylindre, MAP, lambda, EGT, spool et courbes de banc ;
- publier les incertitudes et distinguer presets démonstratifs et moteurs
  calibrés ;
- benchmarker objectivement EngineLab et un checkout ES2D complet.

Critère de sortie : erreurs et domaine de validité documentés sur plusieurs
moteurs, au lieu d'une simple plausibilité visuelle.

### 7. Ajouter la 3D après stabilisation du contrat de scène

- implémenter le backend OpenGL sans dépendance inverse vers la simulation ;
- créer les assets, le cache GPU, les matériaux et les contrôles caméra ;
- étendre progressivement les `RenderPartKind` ;
- mesurer le temps GPU/CPU et préserver le budget audio.

La 3D améliorera la présentation mais ne doit pas retarder la validation du son
et des outils d'auteur, qui déterminent davantage l'objectif face à ES2D.

## Conclusion

EngineLab possède désormais une base logicielle plus propre et plusieurs
capacités absentes du code ES2D inspecté, notamment le tuning ECU sans reset et
le graphe d'échappement structuré. Il n'est pas encore globalement supérieur :
ES2D garde l'avantage du langage, du contenu et du recul perceptuel. Le prochain
risque n'est plus l'architecture ; c'est de confondre sophistication interne et
qualité prouvée. La feuille de route ci-dessus transforme précisément cette
sophistication en résultats mesurables et en outils simples pour l'utilisateur.
