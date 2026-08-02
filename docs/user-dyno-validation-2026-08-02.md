# Validation du banc utilisateur — 2 août 2026

## Défaut reproduit

Le banc produit imposait immédiatement sa première consigne
(`max(1000, idle_rpm)`) dès l'appui sur D. Si le moteur tournait déjà plusieurs
milliers de tours plus haut, le frein unidirectionnel recevait un écart brutal
et pouvait traverser la zone de ralenti jusqu'au calage. Le test historique ne
voyait pas ce défaut car il lançait toujours le banc moteur arrêté.

Une seconde limite concernait les twins : essayer de mesurer WOT exactement au
ralenti (1 200–1 400 tr/min selon le moteur) provoquait un accrochage/décrochage
du frein avant le premier point.

## Chemin produit corrigé

- Un moteur arrêté conserve le démarrage automatique existant.
- Un moteur déjà au-dessus du premier point commence par une préparation
  visible. La consigne du frein part du régime réel puis descend à
  700 tr/min/s ; sa bande de contact est élargie uniquement pendant cette
  approche.
- Le maintien scientifique historique reprend ensuite avec sa bande précise de
  60 tr/min. Il n'a pas été remplacé par le contrôleur inertiel expérimental :
  ce dernier tenait le CP2 mais ralentissait excessivement les hauts régimes et
  a été retiré.
- L'entrée du balayage vaut `max(1000, idle_rpm + 400)` ; les moteurs à un ou
  deux cylindres démarrent au minimum à 1 800 tr/min. Ce sont des points WOT,
  pas une mesure de ralenti.
- Si un point décroche réellement sous 55 % du ralenti, le frein se libère et
  le runtime redémarre puis réapproche la même consigne. Le compteur est exposé
  en télémétrie ; la validation finale n'a eu besoin d'aucune récupération.
- Le temps maximal produit passe de 30 à 60 secondes, valeur utilisée par le
  passage complet du catalogue.

L'écran affiche désormais `PRÉPARATION PROGRESSIVE`, la consigne, l'avancement
et un bouton `ANNULER PRÉPA`. La courbe visible reçoit un filtre triangulaire
sur trois points, comparable au filtrage d'affichage d'un banc réel. Les
`DynoRun`, les maxima et les exports conservent les échantillons bruts.

## Nouveau harnais

`EngineLabUserDynoHarness` lance chaque moteur en roue libre jusqu'à un régime
nettement supérieur à l'entrée, appuie alors sur D par l'API produit et mesure :

- observation obligatoire de la préparation ;
- régime minimum et absence de calage ;
- premier point à ±100 tr/min de l'entrée ;
- zéro arrêt prématuré ;
- progression ≥99 % et dernier point à moins de 100 tr/min du plafond ;
- nombre de points et récupérations.

Commande complète :

```powershell
out/build/windows-vs2022/tools/Release/EngineLabUserDynoHarness.exe --complete
```

## Résultat complet du catalogue

| Moteur | Déclenchement | Minimum | Premier | Dernier | Points | Récup. |
|---|---:|---:|---:|---:|---:|---:|
| K20A I4 | 2 689 | 1 254 | 1 343 | 8 159 | 29 | 0 |
| 2JZ-GTE I6 Turbo | 2 122 | 1 091 | 1 177 | 6 640 | 23 | 0 |
| LS3 V8 | 2 240 | 966 | 1 151 | 6 258 | 22 | 0 |
| EJ25 Flat-4 Turbo | 2 268 | 1 157 | 1 241 | 6 451 | 22 | 0 |
| Audi I5 Turbo | 2 379 | 1 143 | 1 201 | 6 736 | 24 | 0 |
| Hayabusa I4 | 3 409 | 1 485 | 1 643 | 10 632 | 37 | 0 |
| Big Twin V2 | 3 537 | 1 629 | 1 799 | 5 316 | 16 | 0 |
| Merlin V12 | 2 113 | 1 093 | 1 221 | 3 026 | 9 | 0 |
| Flat-6 3.6 | 2 184 | 1 057 | 1 210 | 7 018 | 25 | 0 |
| Radial R5 | 1 980 | 956 | 1 057 | 2 266 | 6 | 0 |
| Yamaha CP2 | 3 295 | 1 503 | 1 801 | 9 489 | 32 | 0 |
| Yamaha CP3 | 3 373 | 1 505 | 1 724 | 10 438 | 36 | 0 |
| Yamaha CP4 | 3 164 | 1 436 | 1 707 | 11 392 | 40 | 0 |
| VW TDI | 2 185 | 1 076 | 1 196 | 4 740 | 16 | 0 |
| CP2 Full System | 3 230 | 1 577 | 1 804 | 9 489 | 32 | 0 |
| Audio Physics Lab | 3 803 | 1 518 | 1 800 | 9 489 | 32 | 0 |

Résultat : **16/16 passages complets**, **zéro récupération**, **zéro calage**.
Le test CTest permanent `EngineLab.UserDynoHighRpmStart` verrouille le CP2 à
chaque suite Release.

## Réancrage après correction d'étincelle

La correction des étincelles manquées a rendu obsolètes quatre calibrations qui
compensaient implicitement ces cycles perdus. Elles ont été réancrées par les
paramètres physiques locaux existants (avance et turbulence de chambre), sans
multiplicateur de couple :

| Famille | Couple simulé / référence | Erreur | Puissance simulée / référence | Erreur |
|---|---:|---:|---:|---:|
| Audi I5 | 445,7 / 500 Nm | -10,9 % | 258,4 / 294 kW | -12,1 % |
| Big Twin | 176,8 / 162,7 Nm | +8,6 % | 83,9 / 73 kW | +14,9 % |
| Porsche Flat-6 | 325,4 / 310 Nm | +5,0 % | 204,2 / 184 kW | +11,0 % |
| Yamaha CP4 | 106,2 / 111 Nm | -4,3 % | 133,5 / 118 kW | +13,1 % |

Le gate constructeur complet repasse **24/24 valeurs dans ±15 %** et
**24/24 fenêtres stables**. Les autres familles n'ont pas été recalibrées.

## Régressions

- `EngineLab.Core` : démarrage automatique, points, restauration des commandes
  et diagnostics de préparation.
- `EngineLab.IntakeTuning` et `EngineLab.GasExchange` : maintien physique
  inchangé.
- `EngineLab.CatalogReference` : 24 références constructeur et tenue filtrée.
- `EngineLab.UserDynoHighRpmStart` : activation haute vitesse CP2.

La matrice Release ciblée finale passe **11/11 en 243,02 s**, avec également
`VehicleDynamics`, `EcuCalibration`, les deux transitoires audio et les deux
accélérations chargées CP2/Audi. L'application JUCE Release a été reliée avec
succès dans la même passe.

## Limites

- La courbe produit est un balayage freiné par paliers, pas une simulation de
  rouleaux avec pertes de transmission.
- Le filtre visuel n'altère pas les CSV ; un export brut peut donc paraître
  plus ondulé que le graphe à l'écran.
- La justesse absolue reste limitée aux 24 points constructeur disponibles ;
  le harnais complet prouve la continuité et l'achèvement, pas une vérité
  mesurée à chaque régime.
