# Tuner ECU et hot reload

Le bouton **ECU** ouvre un éditeur de cartographies inspiré des outils de
calibration : sélection de table, axes régime/charge, cellules colorées, point
de fonctionnement surligné, limites et révision active.

Ce tuner agit uniquement sur la simulation EngineLab. Ses fichiers et valeurs
ne doivent jamais être injectés dans une ECU de véhicule réel.

## Paramètres réellement actifs

Le jeu créé avec chaque moteur contient :

| Identifiant | Forme | Effet |
|---|---|---|
| `fuel.target_afr` | table 2D régime × charge | cible AFR interpolée bilinéairement |
| `ignition.advance_deg` | table 2D régime × charge | avance absolue avant corrections dynamiques |
| `limits.rev_rpm` | scalaire | seuil du limiteur |

La charge est actuellement `MAP / pression_ambiante`, bornée entre 0 et 4. Les
tables par défaut couvrent 0 à 400 %, avec des points supplémentaires au-dessus
de 100 % pour les moteurs suralimentés. Les coordonnées hors des axes sont
clampées sur la première ou dernière cellule.

Le slider AFR de la fenêtre principale est un trim explicite de -3 à +3 AFR,
neutre à 0. Le slider d'allumage ajoute un trim en degrés à la table. Démarrage,
température, enrichissement transitoire, knock et logique de limiteur
s'appliquent ensuite.

Les bornes canoniques sont partagées par la validation, les métadonnées générées
et l'ECU : 10,5 à 18 pour AFR, -10° à 55° pour l'avance, et 600 à 25 000 tr/min
pour le rupteur. Une cellule acceptée par le tuner n'est donc plus silencieusement
re-clampée sur une plage différente. Les corrections dynamiques restent bornées
dans ce même domaine après application des trims.

Le framework définit des clés réservées à de futures cartes de rendement
volumétrique, VVT/VVL, boost ou wastegate. Elles ne sont pas instanciées dans
la calibration par défaut et ne sont pas consommées par l'ECU actuelle.

## Publication transactionnelle

La fenêtre édite un `CalibrationDraft`. Quand une cellule perd le focus ou que
la touche Entrée est pressée :

1. le texte est converti en nombre fini ;
2. le brouillon complet est validé ;
3. `expectedRevision` empêche d'écraser une modification concurrente ;
4. un nouveau `CalibrationSnapshot` immuable est publié par échange atomique.

L'ECU charge ce pointeur une fois au début de chaque trame externe de simulation.
Elle observe donc soit l'ancienne calibration complète, soit la nouvelle, jamais
un mélange de cellules au milieu des sous-pas. Le lecteur ne prend pas le mutex
des écrivains. Un epoch par lecteur retient les anciens snapshots jusqu'à ce que
la simulation ait accusé réception de la nouvelle révision ; leur destruction
reste ainsi sur le thread qui publie. `atomic<shared_ptr>` n'est toutefois pas
présenté comme obligatoirement lock-free sur toutes les bibliothèques standard.
Une limite dépassée, un axe invalide ou un conflit conserve la révision
précédente et recharge l'éditeur depuis la source active.

Cette publication ne remplace pas le runtime : régime, températures, cycle de
combustion, film de carburant et transmission continuent sans reset.

## Charger, enregistrer et surveiller

**CHARGER** accepte un fichier `.ecu.json` ou `.json` de 2 Mio maximum, le
valide et le publie en une transaction. **ENREGISTRER** écrit le snapshot
courant. Dans les deux cas, ce chemin devient surveillé.

Tant que la fenêtre tuner existe, son timer vérifie le fichier dix fois par
seconde. Une sauvegarde externe valide publie une nouvelle révision ; un JSON
malformé ou hors limites affiche l'erreur et laisse la dernière révision valide
en service.

Le watcher ne suit pas un fichier avant un premier chargement ou
enregistrement. Les rechargements structurels déclenchés par le script live,
l'éditeur JSON ou le concepteur d'échappement conservent le même magasin, la
fenêtre et son watcher. Choisir ou importer explicitement un autre moteur ferme
la fenêtre et crée le jeu par défaut de ce moteur.

## Format JSON de calibration

Le schéma actuel vaut `1`. Une calibration est un `scalar`, une `curve_1d` ou
une `table_2d`. Cet exemple réduit montre une table AFR 2 × 2 et le rupteur :

```json
{
  "schema_version": 1,
  "name": "Calibration exemple",
  "description": "Carte minimale",
  "calibrations": [
    {
      "kind": "table_2d",
      "metadata": {
        "id": "fuel.target_afr",
        "display_name": "AFR cible",
        "description": "Régime et charge",
        "unit": "afr",
        "display_precision": 2,
        "live_editable": true,
        "limits": { "minimum": 10.5, "maximum": 18.0 }
      },
      "x_axis": {
        "id": "rpm",
        "display_name": "Régime",
        "quantity": "engine_speed",
        "unit": "rpm",
        "breakpoints": [1000, 6000]
      },
      "y_axis": {
        "id": "load",
        "display_name": "Charge",
        "quantity": "normalized_load",
        "unit": "ratio",
        "breakpoints": [0.0, 2.0]
      },
      "values": [14.7, 14.2, 13.8, 12.8]
    },
    {
      "kind": "scalar",
      "metadata": {
        "id": "limits.rev_rpm",
        "display_name": "Limiteur",
        "description": "Seuil régime",
        "unit": "rpm",
        "display_precision": 0,
        "live_editable": true,
        "limits": { "minimum": 600, "maximum": 25000 }
      },
      "value": 7200
    }
  ]
}
```

Dans une table 2D, `values` est organisé par lignes : toutes les valeurs de
l'axe X pour le premier point Y, puis toutes celles du deuxième point Y, etc.
Le nombre de valeurs doit donc être `x_count × y_count`.

Les axes doivent contenir des nombres finis strictement croissants. Leur unité
doit correspondre à leur quantité. Pour les clés ECU connues, le contrat est
plus strict afin d'éviter une carte valide mais ignorée : régime en
`engine_speed/rpm`, puis charge en `normalized_load/ratio` pour une table 2D.
Les limites sont dures : une seule cellule hors plage invalide tout le document.

## API C++

`CalibrationStore` est volontairement générique. Les lecteurs peuvent demander
un scalaire, échantillonner une courbe linéaire ou une table bilinéaire avec des
`AxisCoordinate` typés. Une unité ou quantité incompatible renvoie l'absence de
valeur plutôt qu'une conversion implicite.

`CalibrationJson::parse` produit un brouillon, `publishJson` combine parse,
validation et publication, et `makeDraft` permet de repartir d'un snapshot.
Les tests `EngineLab.Calibration` et `EngineLab.EcuCalibration` couvrent les
dimensions, interpolations, limites, conflits de révision et lectures
concurrentes.

## Limites et prochaines extensions

- pas encore d'édition d'axes, sélection multi-cellules, lissage, undo/redo ou
  comparaison visuelle de deux révisions ;
- pas de datalogger synchronisé, de trace historique du point actif ou de
  fonction d'auto-tune des tables ;
- seules AFR, avance et limiteur ont un effet runtime ;
- la correction lambda par cylindre compense le transport du carburant, mais il
  n'existe pas encore de modèle détaillé de sondes, de trims court/long terme,
  de stratégies OBD ou de torque management comparable à une ECU de production ;
- un changement explicite vers un autre moteur ne transpose pas automatiquement
  les cartes vers de nouveaux axes ; il démarre avec les valeurs de ce moteur.
