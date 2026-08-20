# Concevoir un échappement personnalisé

Chaque `ExhaustPathConfig` peut contenir un `graph` optionnel. Ce graphe décrit
un réseau orienté acyclique de composants entre les cylindres et une ou
plusieurs sorties. Il est sérialisé en JSON et YAML et compilé par
`ExhaustGraph` en routes utilisées par les outils de diagnostic historiques.
En production, `ExhaustNetworkLayout` conserve chaque composant dans le solveur
gazeux quasi-1D et `AcousticExhaustNetwork` compile le même DAG en guides
bidirectionnels pour la haute bande. Il n'existe donc plus de réduction globale
du réseau à une seule restriction ou à un seul tube acoustique par chemin.

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
        - { id: 301, type: catalyst, length_mm: 180, diameter_mm: 60, outlet_diameter_mm: 68, restriction: 0.18, catalyst_cell_density_cpsi: 400, catalyst_open_area_ratio: 0.80, catalyst_substrate_volumetric_heat_capacity_j_m3_k: 2000000, resonance_hz: 0, acoustic_gain: 1.00 }
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

### Monolithe de catalyseur homogénéisé

Un composant `catalyst` peut décrire explicitement son substrat cellulaire avec
trois champs :

- `catalyst_cell_density_cpsi`, densité de cellules par pouce carré ;
- `catalyst_open_area_ratio`, fraction de surface frontale réellement ouverte ;
- `catalyst_substrate_volumetric_heat_capacity_j_m3_k`, capacité thermique du
  solide par volume de substrat occupé.

Les trois valeurs doivent être nulles ensemble ou positives ensemble. Zéro
conserve exactement le catalyseur historique : un conduit ordinaire portant la
longueur, le diamètre et le coefficient `restriction`. Quand le substrat est
renseigné, la validation accepte 25 à 5 000 cpsi, une aire ouverte de 0,05 à
0,99 et exige qu'au moins un pas de cellule tienne dans le diamètre du boîtier.
ECHAP PRO propose 400 cpsi, 0,80 et 2,0 MJ/m3/K comme point de départ éditable ;
ces nombres ne sont pas déduits du nom du moteur ni appliqués aux anciens
catalogues.

Le modèle suppose des canaux carrés. Avec `N` en cpsi et `phi` comme fraction
ouverte :

```text
pas        = 0,0254 / sqrt(N)
largeur    = pas * sqrt(phi)
Dh canal   = largeur
aire débit = aire boîtier * phi
```

Le faisceau complet reste **un seul conduit quasi-1D**. L'aire ouverte fixe
l'admittance de débit et le diamètre hydraulique d'un canal fixe le frottement
distribué et les pertes thermo-visqueuses acoustiques. Le nombre de cellules du
solveur gaz et le nombre de lignes audio restent donc identiques au bypass,
quelle que soit la valeur cpsi. Chaque cellule axiale agrège aussi la surface
mouillée de tous les canaux, la capacité du solide et celle de l'enveloppe
métallique dans un unique état thermique. Ce modèle ne prétend pas calculer la
chimie de dépollution, la conduction radiale interne ni chaque canal réel.

## Types de composant

| Type | Rôle compilé |
|---|---|
| `pipe` | longueur, diamètre, perte géométrique et résonance quart d'onde |
| `merge` | rassemble au moins deux entrées vers une sortie |
| `splitter` | partage une entrée vers au moins deux branches |
| `resonator` | conduit inline s'il possède une sortie ; branche acoustique fermée s'il n'en possède aucune |
| `muffler` | chambre/conduit inline et, si renseigné, garnissage poreux distribué |
| `catalyst` | conduit physique et, si renseigné, substrat cellulaire homogénéisé passif |
| `outlet` | termine une route et applique diamètre/coefficient de décharge |

Chaque composant possède :

- `id`, unique à l'intérieur du chemin ;
- `length_mm`, `diameter_mm`, éventuellement `outlet_diameter_mm` et
  `volume_l` ;
- `restriction`, coefficient de perte additionnel sans dimension ;
- `resonance_hz`, accord de référence d'un `resonator` terminal ; zéro conserve
  sa longueur géométrique ;
- `acoustic_gain`, conservé pour le rendu audio de secours historique, mais
  jamais appliqué au guide d'onde physique passif ;
- `discharge_coefficient`, principalement utilisé par la sortie.

La restriction finale additionne la perte géométrique calculée et
`restriction`. Elle change le débit et la pression calculés, donc peut modifier
indirectement la source acoustique physique ; elle n'est pas encore une
impédance acoustique complexe. Modifier seulement `acoustic_gain` ne change pas
le réseau physique de production. Ce champ n'agit que si le rendu doit utiliser
son ancien chemin reconstruit de secours.

### Résonateur terminal

Un `resonator` relié depuis un composant, sans aucune sortie et sans mapping de
cylindre, est une branche latérale scellée. Elle n'ajoute aucune route de débit
moyen. L'audio ajoute une ligne bidirectionnelle avec une réflexion de pression
`+1` à son extrémité. Un `volume_l` positif transforme cette extrémité en cavité
compliant passive ; zéro donne une branche quart d'onde rigide.

Avec `resonance_hz = 0`, `length_mm` est la longueur acoustique. Une fréquence
positive remplace cette longueur, à la température de référence du graphe, par
`c/(4f)`. Le milieu simulé continue ensuite à faire varier la vitesse du son et
donc l'accord. Il n'y a ni oscillateur ajouté, ni filtre correctif. Si une
cavité est également configurée, la fréquence saisie accorde la longueur du col
et non la résonance finale de l'ensemble col-cavité.

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
- au plus huit `resonator` terminaux sur l'ensemble du moteur, borne explicite
  du nombre de lignes de délai acoustiques supplémentaires ;
- un ID de composant unique et des arêtes uniques sans auto-boucle ;
- un mapping unique pour chaque cylindre du chemin ;
- une entrée et une sortie pour `pipe`, `muffler`, `catalyst` et un
  `resonator` inline ;
- exactement une connexion composant entrante, aucun mapping cylindre et
  aucune sortie pour un `resonator` utilisé comme branche latérale ;
- au moins deux entrées et exactement une sortie pour `merge` ;
- exactement une entrée et au moins deux sorties pour `splitter` ;
- au moins une entrée et aucune sortie pour `outlet` ;
- aucun cycle, aucun composant inaccessible et chaque branche de débit terminée
  par une sortie (les `resonator` terminaux sont les seules feuilles scellées).

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
| `acousticBranchLimitReached` | plus de huit branches acoustiques ont été fournies sans passer par la validation ; les suivantes sont omises | premier composant omis |

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

Pour l'audio physique, le DAG complet fournit les longueurs et sections des
guides caractéristiques. Les métriques historiques `audio_volume`,
`sound_attenuation`, gains de composants, modes de preset et transmission de
`FiringEvent` ne colorent pas la frontière SI. Firing order, pression, débit,
température et géométrie suffisent à produire les caractéristiques acoustiques.

Chaque composant de longueur finie devient une ligne à retard bidirectionnelle.
Les interfaces directes, merges et splitters utilisent une dispersion passive
par admittance ; les longueurs de tronc portées par une branche restent des
conduits, et chaque sortie conserve sa propre charge de rayonnement, sa position
et son axe. Un 4-1 et un 4-2-1 ne sont donc pas réduits au même chemin dès lors
que leurs géométries diffèrent. Les preuves causales et analytiques sont dans
`tests/RealtimeRegressionTests.cpp` et résumées dans
[exhaust-audio-topology-validation-2026-08-01.md](exhaust-audio-topology-validation-2026-08-01.md).

## Chemins multiples

Un V ou un flat peut déclarer deux `exhaust_paths`, chacun avec ses cylindres,
son graphe et, facultativement, une IR mesurée explicite. Le gaz et l'audio
conservent ces chemins séparés.
Les indices runtime suivent l'ordre du tableau, tandis que les IDs auteur
restent les références des banques et de la sérialisation.

## Compatibilité avec les configurations existantes

`graph` est optionnel. En son absence, EngineLab compile les anciens champs de
géométrie en primaires, merge, silencieux et sortie. Les fichiers moteur des
schémas 1 à 6 restent lisibles et sont migrés en mémoire vers le schéma 7. Tout
nouvel export JSON/YAML porte `schema_version: 7`. Les fichiers historiques du
catalogue restent volontairement des fixtures de migration ; l'absence des
trois champs de substrat conserve le bypass exact.

Même avec un graphe, le bloc `geometry` reste utile : il fournit les valeurs de
secours nécessaires à la compilation physique d'une ancienne configuration.
L'absence de WAV signifie désormais champ libre ; aucune IR n'est générée.

## Ce que le graphe ne simule pas

Le solveur non linéaire résout bien chaque composant, mais seulement dans la
bande nécessaire au débit et à la contre-pression temps réel. La haute bande
audio conserve le DAG, mais reste linéaire et plane. Elle ne résout pas les
modes transverses, les coudes 3D ni la correction complète du rayonnement par
écoulement moyen. La directivité simple de terminaison et les positions/axes
indépendants des sorties sont résolus vers une paire de microphones commune ;
le modèle n'est pas un champ acoustique 3D ni une simulation de local.

Les réflexions haute fréquence du guide ne reviennent pas dans le cylindre 0D ;
le retour physique est fourni par le réseau non linéaire basse bande. Les IR ne
sont chargées que par `impulse_response` explicite et doivent représenter une
mesure aval. Voir [thermoacoustic-architecture.md](thermoacoustic-architecture.md)
pour les équations, invariants et fichiers propriétaires.
