# Concevoir un échappement personnalisé

Chaque `ExhaustPathConfig` peut contenir un `graph` optionnel. Ce graphe décrit
un réseau orienté acyclique de composants entre les cylindres et une ou
plusieurs sorties. Il est sérialisé en JSON et YAML et compilé par
`ExhaustGraph` en routes utilisées pour la contre-pression, le délai, le gain et
la résonance des événements. Le réseau est aussi réduit en propriétés physiques
agrégées injectées dans le solveur gazeux ; ce n'est pas encore la résolution
d'un volume distinct par composant.

Le bouton **ECHAP. PRO** ouvre un concepteur graphique pour les opérations les
plus courantes. JSON/YAML reste le format de persistance et permet encore les
modifications structurelles que l'interface ne couvre pas. Un script `.els`
peut utiliser ce fichier comme `base`, mais le DSL ne crée pas encore les nœuds
du graphe lui-même.

## Utiliser le concepteur

La fenêtre charge une copie de travail : ajouter, supprimer ou modifier un
composant ne touche pas le moteur qui tourne tant que la copie n'est pas
appliquée.

1. Choisir un chemin dans la liste. **+ CHEMIN** extrait le cylindre choisi avec
   sa branche de graphe ; **- CHEMIN** transfère ses cylindres vers une
   destination avant suppression. Le sélecteur **DEPLACER** réaffecte un
   cylindre entre deux chemins sans créer de doublon. L'IR reste dans le fichier
   moteur.
2. Si le chemin n'a pas encore de graphe, sélectionner **GENERER DEPUIS
   LEGACY**. Un primaire par cylindre, une jonction, un silencieux et une sortie
   sont créés à partir de `geometry`.
3. Ajouter les composants, puis éditer leur type, ID, dimensions, restriction,
   résonance, gain et coefficient de décharge. Le canevas utilise un placement
   automatique et un clic sur un nœud sélectionne le composant correspondant.
4. Créer les connexions orientées dans l'ordre du flux et affecter chaque
   cylindre à son premier composant.
5. Sélectionner **VALIDER ET APPLIQUER**. La validation complète décrite plus
   bas s'exécute avant toute modification du moteur.
6. Utiliser **EXPORTER** dans la fenêtre principale pour enregistrer la
   configuration appliquée en JSON ou YAML.

Une erreur conserve la copie pour correction et laisse le runtime courant
inchangé. Une application valide est un changement structurel : elle remplace
le runtime, réinitialise le régime et les états thermiques, et n'est pas
autorisée pendant un passage au banc. Ce n'est donc pas le hot reload sans reset
du tuner ECU.

## Exemple YAML

Le bloc suivant remplace `engine.exhaust_paths` pour un I4 dont les cylindres
ont les identifiants 1 à 4 :

```yaml
exhaust_paths:
  - id: 1
    cylinder_ids: [1, 2, 3, 4]
    impulse_response: "assets/ir/exhaust_default.wav"
    audio_volume: 1.0
    geometry:
      primary_length_mm: 480
      primary_diameter_mm: 42
      collector_diameter_mm: 60
      muffler_restriction: 0.25
      outlet_diameter_mm: 70
      collector_volume_l: 2.5
      outlet_discharge_coefficient: 0.78
    graph:
      components:
        - { id: 101, type: pipe,     length_mm: 480, diameter_mm: 42, restriction: 0.00, resonance_hz: 0, acoustic_gain: 1.00 }
        - { id: 102, type: pipe,     length_mm: 480, diameter_mm: 42, restriction: 0.00, resonance_hz: 0, acoustic_gain: 1.00 }
        - { id: 103, type: pipe,     length_mm: 480, diameter_mm: 42, restriction: 0.00, resonance_hz: 0, acoustic_gain: 1.00 }
        - { id: 104, type: pipe,     length_mm: 480, diameter_mm: 42, restriction: 0.00, resonance_hz: 0, acoustic_gain: 1.00 }
        - { id: 201, type: merge,    length_mm: 0,   diameter_mm: 60, restriction: 0.03, resonance_hz: 0, acoustic_gain: 1.00 }
        - { id: 301, type: catalyst, length_mm: 180, diameter_mm: 60, outlet_diameter_mm: 68, restriction: 0.18, resonance_hz: 0, acoustic_gain: 0.92 }
        - { id: 401, type: muffler,  length_mm: 520, diameter_mm: 65, volume_l: 8.0, restriction: 0.20, resonance_hz: 95, acoustic_gain: 0.82 }
        - { id: 501, type: outlet,   length_mm: 120, diameter_mm: 70, restriction: 0.00, resonance_hz: 0, acoustic_gain: 1.00, discharge_coefficient: 0.78 }
      cylinder_connections:
        - { cylinder_id: 1, to_component_id: 101 }
        - { cylinder_id: 2, to_component_id: 102 }
        - { cylinder_id: 3, to_component_id: 103 }
        - { cylinder_id: 4, to_component_id: 104 }
      connections:
        - { from_component_id: 101, to_component_id: 201 }
        - { from_component_id: 102, to_component_id: 201 }
        - { from_component_id: 103, to_component_id: 201 }
        - { from_component_id: 104, to_component_id: 201 }
        - { from_component_id: 201, to_component_id: 301 }
        - { from_component_id: 301, to_component_id: 401 }
        - { from_component_id: 401, to_component_id: 501 }
```

Les mêmes clés existent en JSON sous `engine.exhaust_paths[].graph`.

### Garnissage poreux mesure

ECHAP PRO expose maintenant les trois champs ci-dessous lorsqu'un composant
`muffler` est selectionne. **GARNISSAGE DEMO** remplit 24 000 Pa.s/m2, 35 mm et
0,28 ; **BYPASS GARNISSAGE** remet les trois champs a zero. Une mise a jour est
un changement structurel et redemarre le moteur lors de l'application du graphe.

Un composant `muffler` peut maintenant decrire son absorption avec trois
mesures independantes de la perte de charge :

- `packing_flow_resistivity_pa_s_m2`, resistivite au flux du materiau poreux ;
- `packing_thickness_mm`, epaisseur radiale du garnissage ;
- `perforated_open_area_ratio`, fraction ouverte du tube perfore, entre 0 et 1.

Les trois champs doivent etre strictement positifs ensemble et ne sont valides
que sur un `muffler`. En leur absence, le filtre compile en identite exacte :
EngineLab n'invente pas une absorption a partir de `restriction`, de
`acoustic_gain` ou du volume du corps. Lorsqu'ils sont renseignes, l'impedance
de surface suit Delany-Bazley et la perte de propagation reste passive. Ces
valeurs doivent provenir de la fiche du materiau ou d'une mesure du silencieux.

Les memes trois champs existent sur une geometrie scalaire sans `graph`, avec
les noms `muffler_packing_flow_resistivity_pa_s_m2`,
`muffler_packing_thickness_mm` et `muffler_perforated_open_area_ratio`. Ils sont
copies dans le vrai noeud muffler lors de la compilation ou de **GENERER DEPUIS
LEGACY**. `cp2_full_system` et `cp2_absorptive_lab` ont la meme geometrie et la
meme restriction ; seul le second renseigne ces valeurs estimees pour une A/B
d'ecoute. EngineLab n'en deduit jamais depuis le nom d'un silencieux.

## Types de composant

| Type | Rôle compilé |
|---|---|
| `pipe` | longueur, diamètre, perte géométrique et résonance quart d'onde |
| `merge` | rassemble au moins deux entrées vers une sortie |
| `splitter` | partage une entrée vers au moins deux branches |
| `resonator` | longueur et résonance explicite ou estimation de Helmholtz avec `volume_l` |
| `muffler` | perte, longueur, gain et résonance de silencieux |
| `catalyst` | perte de charge concentrée avec longueur physique |
| `outlet` | termine une route et applique diamètre/coefficient de décharge |

Chaque composant possède :

- `id`, unique à l'intérieur du chemin ;
- `length_mm`, `diameter_mm`, éventuellement `outlet_diameter_mm` et
  `volume_l` ;
- `restriction`, coefficient de perte additionnel sans dimension ;
- `resonance_hz`, où zéro demande une estimation lorsque le type le permet ;
- `acoustic_gain`, multiplié le long de la route ;
- `discharge_coefficient`, principalement utilisé par la sortie.

La restriction finale additionne la perte géométrique calculée et
`restriction`. Modifier seulement `acoustic_gain` change la transmission
acoustique des événements et du signal continu, pas la contre-pression.

Le `discharge_coefficient` de la sortie est un coefficient de contraction
d'écoulement. Il est appliqué une seule fois, comme section effective `A·Cd` de
la sortie, et n'entre donc plus comme facteur `1/Cd²` dans la restriction : ce
double comptage réduisait une seconde fois la même section effective et
sous-estimait le débit de sortie.

### Section variable

`diameter_mm` est le diamètre à l'entrée du composant.
`outlet_diameter_mm` est facultatif : zéro conserve une section constante,
tandis qu'une valeur non nulle décrit un raccord conique dont le rayon varie
linéairement. Le volume implicite est celui du tronc de cône,

```text
V = pi L (r_entree² + r_entree r_sortie + r_sortie²) / 3
```

et non une moyenne arbitraire des diamètres. Le maillage quasi-1D emploie les
aires exactes de chaque face, pondère les flux conservatifs par ces aires et
ajoute le terme géométrique `p dA/dx` à l'équation de quantité de mouvement.
Une pression uniforme au repos reste donc un équilibre exact. Les aires
d'entrée et de sortie sont aussi conservées jusqu'au réseau acoustique pour que
chaque jonction utilise sa propre admittance `A/(rho c)`.

Le concepteur graphique expose les deux diamètres. Les documents antérieurs
qui ne possèdent pas `outlet_diameter_mm` gardent exactement leur conduit
cylindrique historique.

## Règles de connexion

La validation impose :

- un à huit chemins, chaque cylindre affecté exactement une fois ;
- 1 à 256 composants et au plus 1 024 connexions par graphe ;
- au plus 4 096 routes développées entre cylindres et sorties ;
- un ID de composant unique et des arêtes uniques sans auto-boucle ;
- un mapping unique pour chaque cylindre du chemin ;
- une entrée et une sortie pour `pipe`, `resonator`, `muffler` et `catalyst` ;
- au moins deux entrées et exactement une sortie pour `merge` ;
- exactement une entrée et au moins deux sorties pour `splitter` ;
- au moins une entrée et aucune sortie pour `outlet` ;
- aucun cycle, aucun composant inaccessible et chaque branche terminée par une
  sortie.

Une erreur fait échouer l'import complet ; l'application ne lance pas un
réseau partiellement valide.

## Diagnostics du compilateur

`ExhaustGraph::makeForEngine` rend toujours un graphe exploitable, y compris
lorsqu'il reçoit une topologie que la validation applicative aurait refusée : le
solveur gazeux et l'audio doivent rester sûrs en toute circonstance. Mais chaque
repli qu'il prend écarte une partie de l'intention de l'auteur, et il le signale
désormais au lieu de substituer un défaut en silence. `ExhaustGraph::diagnostics()`
est vide quand la topologie a été compilée telle qu'elle est écrite.

| Diagnostic | Sens | `relatedId` |
|---|---|---|
| `topologyRejected` | un cylindre n'est pas couvert exactement une fois ; les chemins écrits ont été remplacés par un chemin unique généré | le cylindre fautif |
| `routeLimitReached` | la limite de 4 096 routes est atteinte ; les routes suivantes ne sont pas compilées | le cylindre en cours |
| `unresolvedRestriction` | une route n'a pas de restriction équivalente finie (branche pendante ou cycle) et s'est vu imputer le maximum | le cylindre concerné |
| `nodeIdSpaceExhausted` | l'espace d'ID générés est épuisé | 0 |

`topologyRejected` est le cas à surveiller : il fait disparaître tout un
échappement personnalisé au profit d'un collecteur générique. À l'oreille, c'est
indiscernable d'une conception simplement décevante.

## Valeurs non finies

Un champ non fini (NaN, infini) issu d'un fichier mal formé ne rend plus le pire
résultat possible. Une `restriction` non finie retombe sur « aucune restriction
additionnelle » et laisse la perte géométrique seule, au lieu d'imputer le
maximum et de museler la ligne sans symptôme. Un `acoustic_gain` non fini est
traité comme un silence, jamais comme un gain maximal.

## Séries, branches et routes

Pour la perte de charge, les composants communs sont en série. Les branches
aval d'un splitter sont combinées en parallèle avec :

```text
K_parallèle = 1 / (Σ 1 / √K_branche)²
```

La restriction équivalente de chaque cylindre contribue à la conductance
physique du chemin et à l'atténuation de l'événement. Toutes les routes
cylindre-sortie sont également énumérées. Leur délai utilise
`c = √(γRT)` à une température d'échappement de référence (ambiante + 405 °C
par défaut, ou une température explicitement fournie au compilateur). Chaque
route conserve jusqu'à huit modes : modes impairs quart d'onde de la route,
résonateurs et silencieux locaux. Les modes proches sont fusionnés et classés
par énergie, au lieu de retenir simplement la fréquence maximale rencontrée.

À une séparation, l'énergie est répartie selon l'admittance aval approximée
`A / √(1 + K_aval)`. L'amplitude de la branche reçoit la racine de cette part
d'énergie. Les produits de gains sont accumulés en domaine logarithmique et
bornés à 8 afin de rester finis même sur un grand DAG.

Pour un DAG auteur, `ExhaustNetworkLayout` conserve chaque composant physique :

- tubes, catalyseurs, silencieux, résonateurs et sorties deviennent des conduits
  quasi-1D avec longueur, volume, section, diamètre hydraulique et perte ;
- merges et splitters deviennent des volumes de jonction finis ;
- chaque soupape et chaque sortie garde sa section et son coefficient de
  décharge ;
- les interfaces partagent un unique flux de Riemann, donc une branche ne peut
  créer ni masse ni énergie selon l'ordre d'itération.

Le solveur basse bande calcule directement pression, température, composition,
débit et réversion dans chaque composant. Les anciennes conductances agrégées et
la fermeture analytique de contre-pression ne sont plus utilisées par
`EngineSimulator`.

Pour l'audio physique, les routes servent à construire longueurs et sections des
guides caractéristiques. Les métriques historiques `audio_volume`,
`sound_attenuation`, gains de composants, modes de preset et transmission de
`FiringEvent` ne colorent pas la frontière SI. Firing order, pression, débit,
température et géométrie suffisent à produire les caractéristiques acoustiques.

Une branche complète n'est toutefois pas encore propagée nœud par nœud dans la
haute bande : les runners se rencontrent dans une jonction de collecteur par
chemin, puis un guide rejoint la sortie. Le réseau volumes finis conserve la
topologie détaillée pour la contre-pression ; le réseau audio en conserve une
réduction caractéristique passive.

## Chemins multiples

Un V ou un flat peut déclarer deux `exhaust_paths`, chacun avec ses cylindres,
son graphe et, facultativement, une IR mesurée explicite. Le gaz et l'audio
conservent ces chemins séparés.
Les indices runtime suivent l'ordre du tableau, tandis que les IDs auteur
restent les références des banques et de la sérialisation.

## Compatibilité avec les configurations existantes

`graph` est optionnel. En son absence, EngineLab compile les anciens champs de
géométrie en primaires, merge, silencieux et sortie. Les fichiers moteur de
schémas 1 et 2 restent lisibles et sont migrés en mémoire vers le schéma 3. Tout
nouvel export JSON/YAML porte `schema_version: 3`. Les fichiers historiques du
catalogue restent volontairement des fixtures de migration.

Même avec un graphe, le bloc `geometry` reste utile : il fournit les valeurs de
secours nécessaires à la compilation physique d'une ancienne configuration.
L'absence de WAV signifie désormais champ libre ; aucune IR n'est générée.

## Ce que le graphe ne simule pas

Le solveur non linéaire résout bien chaque composant, mais seulement dans la
bande nécessaire au débit et à la contre-pression temps réel. La haute bande
audio reste linéaire, plane et agrégée par chemin. Elle ne résout pas les modes
transverses, les coudes 3D, la directivité ni la correction du rayonnement par
écoulement moyen. Deux sorties d'un splitter ne possèdent pas encore des
positions micro indépendantes.

Les réflexions haute fréquence du guide ne reviennent pas dans le cylindre 0D ;
le retour physique est fourni par le réseau non linéaire basse bande. Les IR ne
sont chargées que par `impulse_response` explicite et doivent représenter une
mesure aval. Voir [thermoacoustic-architecture.md](thermoacoustic-architecture.md)
pour les équations, invariants et fichiers propriétaires.
