# Scripts moteur `.els`

Le DSL EngineLab est un langage déclaratif sûr qui compile vers un
`EngineConfig`. Il sert à choisir une base et à surcharger des valeurs avec des
unités explicites. Les extensions reconnues par l'application sont `.els` et
`.engine`.

Ce langage n'est pas compatible avec les fichiers `.mr` d'ES2D et n'essaie pas
encore d'en reproduire toute l'expressivité.

## Exemple complet minimal

```text
# Les commentaires commencent par # ou //.
preset inline_four
name "Street Turbo I4"

let runner = 30 cm + 20 mm
let primary = 48 cm

set idle_rpm = 900 rpm
set engine.redline_rpm = 7200 rpm
set ignition.rev_limit_rpm = 7350 rpm
set intake.runner_length_mm = runner
set intake.runner_diameter_mm = 42 mm
set exhaust.primary_length_mm = primary
set exhaust.outlet_diameter_mm = 70 mm

set forced_induction.enabled = true
set forced_induction.type = turbo
set forced_induction.pressure_ratio = 1.35
set cylinder.all.bore_mm = 86 mm
set cylinder.2.ignition_offset_deg = -2 deg

ignition clear
ignition point 800 rpm, 10 deg
ignition point 3500 rpm, 29 deg
ignition point 7350 rpm, 28 deg
```

Le dépôt contient aussi `examples/street-turbo.els`.

## Choisir une base

Sans instruction de base, le compilateur part d'un quatre-cylindres en ligne.
Les presets intégrés sont :

- `inline_two`/`i2`, `inline_four`/`i4`, `inline_five`/`i5` ;
- `v6`, `v8` ;
- `flat_six`/`boxer_six` ;
- `radial_five`.

Un moteur complet existant peut remplacer cette base :

```text
base "../engines/my-engine.yaml"
name "Variante piste"
set engine.redline_rpm = 8200 rpm
```

`base` accepte un JSON ou YAML moteur v1 ou v2. Une base v1 est migrée en
mémoire et le résultat compilé utilise le schéma courant v2. Le chemin est
relatif au fichier qui contient l'instruction.

`include` lit un autre script dans le même contexte de variables et de
configuration :

```text
include "shared/intake.els"
include "shared/track-ignition.els"
```

L'ordre des instructions est significatif : un `preset` ou `base` rencontré
plus tard remplace la configuration accumulée jusque-là.

## Variables, expressions et unités

`let` définit une variable une seule fois. Les noms ne sont pas sensibles à la
casse. Les expressions acceptent `+`, `-`, `*`, `/`, les parenthèses, les
signes unaires et `pi`.

L'addition et la soustraction exigent la même dimension. La multiplication
n'accepte qu'un facteur dimensionnel et un facteur sans dimension. Une division
par une valeur de même dimension produit un ratio ; les unités composées
arbitraires ne sont pas inférées.

Unités reconnues :

| Grandeur | Symboles |
|---|---|
| ratio | `ratio`, `%`, `percent`, `pct` |
| longueur / volume | `mm`, `cm`, `m` / `l`, `ml`, `cc`, `cm3` |
| masse | `mg`, `g`, `kg` |
| pression / température | `pa`, `kpa`, `bar` / `c`, `degc`, `celsius` |
| angle / temps | `deg`, `degree`, `rad` / `us`, `ms`, `s`, `sec` |
| fréquence / régime | `hz`, `khz` / `rpm` |
| section | `mm2`, `cm2`, `m2` |
| débit massique | `mg_s`, `mgps`, `mg_per_s`, `g_s`, `kg_s` |
| énergie massique | `j_kg`, `kj_kg` |
| vitesse / force | `mm_s`, `m_s`, `mps` / `n` |
| frottement / inertie | `ns_m` / `kg_m2` |
| puissance / couple | `w`, `kw` / `nm` |

Par exemple `20 MPa` est refusé car `MPa` n'appartient pas à cette liste ;
écrire `200 bar` ou `20000 kpa`.

## Propriétés modifiables

`set idle_rpm = ...` est un raccourci pour `set engine.idle_rpm = ...`.
Les familles actuellement prises en charge couvrent :

- moteur : ralenti, rupteur mécanique, inertie, friction, octane, ambiance,
  refroidissement, angle de banc, layout et nom ;
- admission et échappement géométriques globaux ;
- limiteur et courbe d'allumage ;
- mode, fenêtre, rail, débit, film et refroidissement d'injection ;
- fréquence, sous-pas et résolution du solveur ;
- activation et paramètres principaux de suralimentation ;
- géométrie, masses, friction, journal, banque et atténuation par cylindre.

Une cible `cylinder.all` modifie tous les cylindres. Une cible numérique utilise
l'identifiant du cylindre, jamais sa position dans le tableau :

```text
set cylinder.all.compression_ratio = 10.5 ratio
set cylinder.7.exhaust_primary_length_mm = 620 mm
set cylinder.7.connecting_rod_type = articulated
```

Une propriété inconnue est une erreur ; elle n'est pas ignorée. La liste
exécutable de référence se trouve dans les tables de
`src/scripting/src/EngineScriptCompiler.cpp`.

## Diagnostics et validation

Chaque erreur contient un code `ESxxx`, le fichier, la ligne et la colonne. Le
compilateur contrôle notamment :

- unités incompatibles, division par zéro et résultat non fini ;
- identifiant de cylindre absent et propriété inconnue ;
- cycle d'inclusion et profondeur maximale de 32 fichiers ;
- source supérieure à 2 Mio ;
- lecture et décodage d'un fichier de base ;
- normalisation et validation finale complète de `EngineConfig`.

Le runtime actuel refuse finalement les moteurs qui ne sont pas des quatre
temps essence, même si les symboles réservés `two_stroke` ou `diesel` existent
dans le parseur pour préparer une extension future.

## Hot reload dans l'application

Après import d'un `.els` ou `.engine`, un worker surveille toutes les
dépendances environ toutes les 250 ms. Une sauvegarde déclenche une compilation
hors du thread UI :

- si elle réussit, une nouvelle révision immuable est publiée et l'application
  construit un nouveau runtime ;
- si elle échoue, la révision et le pointeur vers la dernière configuration
  valide sont conservés, et les diagnostics sont affichés.

Le watcher reste actif après une reconfiguration réussie. Il détecte aussi la
modification d'un `include` ou du JSON/YAML chargé par `base`.

Ce hot reload est structurel. Il évite de relancer l'application, mais il
remplace le moteur simulé et réinitialise régime, températures, combustion,
transmission et audio. Le `CalibrationStore` courant est partagé avec le nouveau
runtime : les modifications live et le fichier `.ecu.json` surveillé restent
actifs sans repasser par un chargement. Pour modifier AFR, avance ou rupteur sans
réinitialiser les états dynamiques, utiliser directement le
[tuner ECU](ecu-tuning.md).

## Limites face au `.mr` d'ES2D

Le langage ne permet pas encore de déclarer de nouveaux types de nœuds, des
fonctions utilisateur, boucles, conditions, collections de pièces ou une
topologie complète à partir de zéro. Il surcharge un preset ou une
configuration canonique. C'est plus borné et facile à valider, mais nettement
moins expressif que l'écosystème Piranha/`.mr` d'ES2D.
