# Validation finale de la base simulation — 2 août 2026

## Périmètre

Ce lot part des défauts observés dans le produit, pas d'une refonte théorique :

- chutes répétées de couple environ 2 000 tr/min avant le rupteur ;
- banc utilisateur qui cale ou ne termine pas sa courbe ;
- Diesel annoncé pauvre sans autorité claire de sa quantité injectée ;
- pression motrice turbo signalée excessive malgré un cat-back agrandi ;
- cylindres du radial qui suivaient visuellement les bielles ;
- nécessité de conserver assez de CPU pour la chaîne sonore.

Les commits de code correspondants sont :

- `81482ab` — suppression des sursauts pré-rupteur ;
- `97b49a3` — banc utilisateur progressif ;
- `0b58421` — commande physique de quantité Diesel ;
- `6e11cea` — partage conservatif turbine/wastegate ;
- `7f8d9f3` — priorité fidèle de la sonde audio ;
- `c0e93dc` — chemises radiales ancrées au carter.

## 1. Sursauts de couple

Trois causes cumulatives ont été corrigées : couplage moteur/roues trop rigide,
calage d'étincelle recalculé pendant le cycle et pénalité de raté appliquée deux
fois à un manque de carburant déjà mesuré.

Le harnais charge le moteur en deuxième puis en troisième, plein gaz, sans
passage automatique ni limite d'adhérence. Il traverse obligatoirement la zone
pré-rupteur puis atteint le limiteur.

| Rapport | Moteurs routiers | Rupteur atteint | Sursauts | Coupures parasites | Glissement embrayage |
|---|---:|---:|---:|---:|---:|
| 2 | 14/14 | 14/14 | 0 | 0 | 0 |
| 3 | 14/14 | 14/14 | 0 | 0 | 0 |

Merlin et Radial n'ont pas de rapport 2/3 et sont explicitement `n/a`. Les
compteurs de résolution angulaire restent non nuls sur K20, Hayabusa, CP3 et
CP4 à très haut régime ; ils ne produisent plus de chute de couple mais restent
une limite numérique distincte à surveiller.

## 2. Banc utilisateur

Le frein rejoint désormais progressivement le premier palier, stabilise le
moteur, mesure plusieurs cycles, puis balaie jusqu'à 95 % de la plus petite
limite mécanique/ECU. Une récupération existe mais n'a pas été utilisée dans la
validation finale.

`EngineLabUserDynoHarness --complete` : **16/16 courbes terminées**, 6 à 40
points, **zéro calage**, **zéro récupération**. Les derniers points vont de
2 268 tr/min pour le Radial à 11 390 tr/min pour le CP4 et restent à moins de
100 tr/min du plafond propre à chaque moteur.

## 3. Diesel

La table `fuel.diesel_quantity_mg_per_cycle` pilote maintenant directement la
quantité à plein régime/charge. `fuel.target_afr` est explicitement une limite
fumée riche, pas une cible stœchiométrique essence. Le diagnostic ne considère
plus un Diesel normalement pauvre comme défectueux.

À 2 000 tr/min plein gaz :

| Cas | Quantité commandée | AFR | Couple | Faux diagnostic pauvre |
|---|---:|---:|---:|---:|
| Dégradé | 34,15 mg/cycle | 26,90 | 216,96 Nm | non |
| Catalogue | 52,55 mg/cycle | 17,62 | 346,82 Nm | non |
| Demande au-dessus fumée | 63,06 mg/cycle | 17,22 | 353,37 Nm | non |

Le dernier cas est bien plafonné par la limite fumée 16,99. Détails :
`docs/diesel-control-validation-2026-08-02.md`.

## 4. Turbo et pression motrice

La wastegate et la turbine sont deux branches parallèles. Leur débit est
désormais partagé par aire effective, conservé exactement, puis réutilisé par
la puissance d'axe et l'audio. L'ancien coefficient empirique non conservatif a
été supprimé.

En troisième, dans les 500 derniers tr/min avant rupteur :

| Moteur | Pression avant | Pression après | Rapport pression/MAP après |
|---|---:|---:|---:|
| 2JZ | 279,7 kPa | 267,9 kPa | 1,39 |
| EJ25 | 253,2 kPa | 231,6 kPa | 1,11 |
| Audi I5 | 300,5 kPa | 278,5 kPa | 1,12 |
| EA288 TDI | 280,4 kPa | 292,1 kPa | 1,29 |

Le TDI monte légèrement avec son nouvel équilibre, mais son rapport reste sain.
Le diagnostic turbo compare maintenant pression motrice et MAP après spool ;
un cas 265/190 kPa ne déclenche pas, un cas 385/190 kPa déclenche toujours.
Détails et sources Garrett/BorgWarner :
`docs/turbo-backpressure-validation-2026-08-02.md`.

## 5. Références constructeur

`EngineLabDynoSweepHarness` valide **24/24** points dans l'enveloppe ±15 % et
toutes les fenêtres de régime. Les bords les plus proches sont :

- Big Twin puissance : +14,905 % ;
- CP4 puissance : +13,147 % ;
- CP2 puissance : +12,345 % ;
- CP3 couple : +12,062 %.

Le projet ne prétend pas pour autant garantir ±15 % sur un moteur utilisateur
arbitraire sans calibration.

## 6. Catalogue temps réel sur l'i5-11600

Mesure finale indépendante, machine revenue à 7,6 % CPU moyen avant lancement :
`EngineLabRealtimeBudgetHarness --free-run`, 5 000 tr/min demandés et plafond
à 95 % de la plage valide. Aucun chiffre de l'ancien portable n'est utilisé.

| Moteur | Facteur physique | Overruns | Alerte pression |
|---|---:|---:|---:|
| K20A I4 | 2,045× | 0 | non |
| 2JZ I6 Turbo | 1,929× | 0 | non |
| LS3 V8 | 1,293× | 0 | non |
| EJ25 Flat-4 Turbo | 2,267× | 0 | non |
| Audi I5 Turbo | 2,098× | 0 | non |
| Hayabusa I4 | 2,308× | 0 | non |
| Big Twin V2 | 3,517× | 0 | non |
| Merlin V12 | 1,101× | 0 | non |
| Flat-6 | 1,694× | 0 | non |
| Radial R5 | 2,559× | 0 | non |
| CP2 | 3,751× | 0 | non |
| CP3 | 2,928× | 0 | non |
| CP4 | 2,381× | 0 | non |
| EA288 TDI | 2,494× | 0 | non |
| CP2 Full System | 3,893× | 0 | non |
| Audio Physics Lab | 3,681× | 0 | non |

Le premier passage Merlin en fin de catalogue est le bord bas : 1,101×, soit
environ 9,2 % avant l'échéance. Trois répétitions au repos donnent 1,124×,
1,145× et 1,135× ; la médiane correspond à environ 11,9 % de marge.

## 7. Budget avec renderer audio de production

La première sonde de test utilisait un thread normal alors que la physique est
`ABOVE_NORMAL`. Elle pouvait être préemptée pendant un bloc complet et compter
un faux dépassement malgré un p99 inférieur à 58 %. La sonde utilise désormais
un thread JUCE `high`, sans promouvoir le processus en temps réel et sans
relâcher le contrat : un seul rendu supérieur aux 5,33 ms reste éliminatoire.

Après correction, les neuf répétitions Audi/Flat-6/moteur labo passent 9/9, puis
le catalogue complet passe 16/16 à 48 kHz / 256 échantillons :

| Moteur | Facteur avec audio | Callback moyen | Callback p99 |
|---|---:|---:|---:|
| K20A | 1,869× | 24,0 % | 35 % |
| 2JZ | 1,751× | 31,7 % | 47 % |
| LS3 | 1,323× | 37,7 % | 58 % |
| EJ25 | 2,104× | 30,4 % | 45 % |
| Audi I5 | 2,006× | 27,3 % | 42 % |
| Hayabusa | 2,081× | 22,7 % | 35 % |
| Big Twin | 3,389× | 19,0 % | 31 % |
| Merlin | 1,117× | 47,9 % | 73 % |
| Flat-6 | 1,509× | 35,4 % | 51 % |
| Radial | 2,147× | 25,9 % | 36 % |
| CP2 | 3,310× | 16,1 % | 25 % |
| CP3 | 2,734× | 18,7 % | 29 % |
| CP4 | 2,252× | 21,8 % | 34 % |
| TDI | 2,378× | 24,2 % | 38 % |
| CP2 Full System | 3,598× | 15,2 % | 25 % |
| Audio Physics Lab | 3,590× | 14,9 % | 25 % |

Tous ont `miss=0`, `dropP=0`, `late=0`, `legacy=0`, `leveler=0`, zéro
overrun physique et aucune sortie non finie.

## 8. Radial

La chemise utilisait auparavant le wrist pin mobile comme centre de dessin.
Le centre fixe du cylindre est maintenant calculé depuis l'angle de banc ; le
piston et la bielle utilisent seuls les points cinématiques mobiles. Le binaire
Release se lie et démarre. La capture Windows.Graphics de la fenêtre JUCE a
échoué avec `0x80004002`, donc aucune fausse preuve visuelle n'est publiée ; ce
point reste à confirmer à l'œil dans le livrable.

## 9. Build, tests et paquet

- build Release complet : vert ;
- tentative `/m:4` : trois `C1060` LTO, puis reconstruction `/m:1` verte comme
  prescrit par le brief ;
- `ctest` Release : **35/35** en **679,20 s** ;
- après les derniers changements app/harness : RenderSnapshot,
  VehicleDynamics et RealtimeRegression **3/3** en 3,00 s ;
- package CPack : 110 entrées avant ajout de ce rapport, 16 moteurs ;
- exécutable extrait lancé depuis son propre dossier : toujours vivant après
  6 secondes, puis fermé proprement.

Le hash du ZIP final est volontairement communiqué hors du ZIP pour éviter une
auto-référence à chaque reconstruction.

## Limites honnêtes

- La chambre reste un modèle thermochimique 0D résolu à l'angle vilebrequin. Les
  sursauts venaient du couplage, de l'étincelle et du carburant ; passer la
  chambre en quasi-1D n'aurait pas corrigé ces causes et n'a pas été entrepris.
- La résolution angulaire atteint encore sa limite sur quatre très hauts
  régimes, sans sursaut observé.
- Le Merlin est le vrai bord CPU : environ 10–12 % de marge au repos, moins en
  fin de longue charge chaude.
- Les tests prouvent stabilité, physique bornée et contrat temps réel. Ils ne
  remplacent pas le verdict d'écoute sur casque/enceintes ni la comparaison à
  des enregistrements réels.

