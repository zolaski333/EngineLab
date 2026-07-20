# EngineLab

EngineLab est un simulateur de moteur à combustion interne quatre temps et un
synthétiseur audio temps réel écrits en C++20 avec JUCE. Le projet vise une
simulation cohérente, observable et agréable à écouter. Il ne remplace ni un
logiciel de calcul thermodynamique validé, ni un banc moteur, ni un outil de
calibration destiné à un véhicule réel.

## État actuel

Le moteur de simulation relie un réseau gazeux à volumes de contrôle, la
cinématique bielle-manivelle, l'injection, la propagation de flamme, le couple
issu de la pression cylindre, les pertes, le turbocompresseur et la chaîne
cinématique jusqu'au véhicule. La cadence interne s'adapte au régime et à la
résolution angulaire configurée.

L'échappement audio part d'un réseau gazeux quasi-1D conservatif, d'un débit SI
signé aux soupapes, de guides caractéristiques et d'une charge de rayonnement
passive. Aucun preset, bruit ou oscillateur de blowdown n'est mélangé à ce chemin
physique. Admission, distribution, mécanique et démarreur restent des couches
hybrides distinctes. Le callback audio ne réalise ni accès fichier, ni attente,
ni allocation dynamique.

L'application fournit également :

- un catalogue de moteurs et des imports/exports JSON ou YAML ;
- un concepteur **ECHAP. PRO** pour éditer des graphes validés avec branches,
  jonctions, résonateurs, silencieux, catalyseurs et sorties ;
- un DSL déclaratif et typé par unités (`.els` ou `.engine`) avec surveillance
  automatique des dépendances ;
- un tuner ECU pour les tables AFR et avance ainsi que le rupteur, appliqués à
  chaud par snapshots transactionnels ;
- un banc automatique avec historique, courbes et export CSV ;
- une vue JUCE 2D et un contrat de scène 3D indépendant du backend graphique.

OpenGL n'est pas implémenté. Le module `render` prépare les transformations 3D,
les identifiants stables, les limites de scène, l'interpolation de snapshots et
l'interface `IEngineRenderer`. La vue actuelle reste un rendu 2D direct. Voir
[l'architecture](docs/architecture.md#préparation-du-rendu-3d) pour le travail
qui reste avant un backend OpenGL.

## Démarrage rapide

Prérequis Windows : Visual Studio 2022 ou 2026 avec le workload C++ desktop,
CMake 3.24 ou plus récent et Git. JUCE, nlohmann-json et yaml-cpp sont récupérés
par CMake à des révisions épinglées.

```powershell
cmake -S . -B build -G "Visual Studio 17 2022" -A x64
cmake --build build --config Release --target EngineLabApp
```

Avec Visual Studio 2026, utiliser `-G "Visual Studio 18 2026"`. L'exécutable est
produit dans `build/src/app/EngineLabApp_artefacts/Release/EngineLab.exe`.

Pour compiler et exécuter toute la validation :

```powershell
cmake --build build --config Release
ctest --test-dir build -C Release --output-on-failure
```

Les détails des tests, des harnais déterministes et des builds avec sanitizers
sont dans [docs/phase-0-1-2.md](docs/phase-0-1-2.md).

## Créer et modifier un moteur

Trois niveaux sont volontairement séparés :

1. JSON/YAML décrit toute la structure moteur. L'appliquer remplace l'instance
   de simulation et réinitialise son état dynamique.
2. Un script `.els` choisit un preset ou un fichier JSON/YAML de base, puis
   applique des modifications avec unités. L'application surveille le script,
   ses inclusions et son fichier de base. Une sauvegarde valide remplace le
   runtime ; une sauvegarde invalide laisse tourner la dernière configuration
   valide et affiche les diagnostics.
3. Le tuner ECU publie une calibration sans remplacer le runtime. Une cellule
   validée devient visible par l'ECU au prochain calcul, sans arrêter le moteur.
   Les documents `.ecu.json` chargés ou enregistrés sont ensuite surveillés.

Le bouton **ECHAP. PRO** travaille sur une copie du moteur. Il permet de générer
un réseau de départ, d'ajouter et paramétrer ses composants, de relier les
nœuds et d'affecter chaque cylindre. **VALIDER ET APPLIQUER** refuse les cycles,
les branches incomplètes et les cardinalités incohérentes. Comme la topologie
d'échappement appartient à la structure moteur, une application valide remplace
le runtime et réinitialise son état ; elle est refusée pendant un passage au
banc. Utiliser ensuite **EXPORTER** pour persister le résultat en JSON ou YAML.

Cette distinction est importante : le hot reload ECU conserve le moteur, son
régime et ses états thermiques ; un changement structurel de cylindrée,
topologie ou géométrie nécessite une nouvelle instance et repart de son état
initial. Les remplacements issus du script live, de l'éditeur JSON et du
concepteur d'échappement réutilisent toutefois le même magasin ECU : les cartes,
la fenêtre tuner et son fichier surveillé restent actifs. Choisir ou importer
un autre moteur crée volontairement sa calibration par défaut.

- [Guide du DSL EngineLab](docs/engine-scripting.md)
- [Guide du tuner et du format ECU](docs/ecu-tuning.md)
- [Guide de l'échappement personnalisé](docs/custom-exhaust.md)
- [Modèle de simulation](docs/simulation-model.md)
- [Architecture thermoacoustique physique](docs/thermoacoustic-architecture.md)
- [Architecture audio temps réel](docs/realtime-audio.md)
- [Livraison et mesures des phases 0 à 3](docs/phase-0-3-delivery.md)

Un exemple de script prêt à importer est disponible dans
`examples/street-turbo.els`.

## Commandes principales

Les touches par défaut sont modifiables depuis le bouton **TOUCHES**. Le fichier
`keybindings.json` refuse les actions inconnues, les doublons et les raccourcis
réservés.

| Entrée | Action par défaut |
|---|---|
| `A` / `S` maintenu | contact / démarreur |
| `Q`, `W`, `E`, `R` | papillon 1 %, 10 %, 20 %, 100 % |
| `D` / `H` | banc automatique / maintien de régime |
| `P` / `Tab` | pause / écran suivant |
| `1` à `5` | temps 0,25×, 0,5×, 1×, 2×, 4× |
| flèches haut/bas/gauche | rapport supérieur, inférieur, frein de roue |
| `Y` ou `Shift` maintenu | débrayer ; `T`/`U` ajustent la consigne |
| `;` | preset acoustique d'échappement suivant |
| molette / glisser / double-clic | zoom, déplacement et recentrage de la vue moteur |

Les modificateurs `G`, `Z`, `X`, `C`, `V`, `B`, `J`, `K`, `L`, `O`, `N` et
`Espace` associés à la molette règlent respectivement le maintien de régime, le
volume, la convolution, les bandes/bruits, les couches du mix, la vitesse de
simulation et le papillon fin.

## Positionnement face à ES2D

EngineLab possède maintenant un socle plus transactionnel et plus instrumenté,
mais il ne revendique pas encore une supériorité globale sur ES2D. ES2D conserve
un avantage net en maturité du langage `.mr`, en richesse de bibliothèque et
en recul perceptuel sur le son. EngineLab dispose d'un réseau gazeux et d'une
télémétrie plus détaillés, d'un audio multi-chemin moderne et d'un vrai hot
reload ECU, mais ces avantages techniques doivent encore être étalonnés contre
des mesures et des écoutes contrôlées.

La comparaison critère par critère, ses conditions et la feuille de route vers
un niveau égal ou supérieur sont dans
[docs/es2d-targeted-gap-closure.md](docs/es2d-targeted-gap-closure.md).

## Limites à connaître

- essence quatre temps uniquement dans le runtime actuel ;
- chambres cylindres 0D et réseau d'échappement quasi-1D basse bande, pas CFD 3D ;
- propagation audible linéaire par caractéristiques agrégées par chemin : les
  modes transverses, les coudes 3D et la correction de rayonnement par écoulement
  moyen ne sont pas résolus ;
- chimie globale et modèles semi-empiriques de flamme, knock et transferts
  thermiques ;
- huit chemins d'échappement audio au maximum ;
- les branches sont conservées dans le solveur gaz, mais leurs sorties ne
  possèdent pas encore des positions audio 3D indépendantes ;
- concepteur d'échappement sans glisser-déposer, undo/redo, audition A/B ni
  sélection d'IR ; les chemins et cylindres sont gérés dans l'interface, tandis
  que l'IR reste éditable en JSON/YAML ;
- admission et bruit structurel/mécanique encore procéduraux ; pas de modèle
  modal réduit du bloc ni de validation sur banc/multi-microphones ;
- pas de backend 3D, d'enregistrement WAV depuis l'interface, ni de diagnostic
  OBD destiné à une ECU réelle.

## Licence

Consultez [CODE_OF_CONDUCT.md](CODE_OF_CONDUCT.md) pour les règles de
contribution. Le projet est distribué sous licence MIT ; voir [LICENSE.md](LICENSE.md).
