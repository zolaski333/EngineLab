# Livraison audio et physique — validation finale du 1er août 2026

Ce document fixe les preuves du lot exécuté sur la machine actuelle. Aucun
chiffre absolu de performance provenant d'une ancienne session ou d'une autre
machine n'est utilisé comme référence. La table temps réel vient d'un seul
passage neuf du binaire MSVC Release final de cette session.

## Verdict

- Les 15 configurations du catalogue tiennent le temps réel, sans overrun.
- Le pire cas est le Merlin V12 à 1,128 fois le temps réel : 12,8 % de capacité
  brute supplémentaire, soit 11,3 % de marge avant l'échéance
  (`1 - 1 / 1,128`). Il reste donc dans la zone demandée de 10 à 15 %.
- La suite Release passe 31 tests sur 31 en 603,46 s. Après l'ajout de l'exemple
  d'écoute, le test Scripting ciblé repasse également.
- L'application extraite du ZIP reste vivante après six secondes et l'exporteur
  emballé charge les 15 moteurs puis produit un vrai rendu K20 de 15,3 s.
- L'ancien rendu par défaut est bit-identique. Les nouvelles physiques sont
  opt-in : elles ne changent pas silencieusement le catalogue.

## Catalogue complet — mesure locale prescrite

Commande exécutée le 1er août 2026 :

```powershell
out/build/windows-vs2022/tools/Release/EngineLabRealtimeBudgetHarness.exe `
  --free-run --rpm 7000 --seconds 6
```

Le régime demandé est borné à 95 % de la plage valide de chaque moteur. Le
facteur est la quantité de temps simulé produite par seconde murale ; 1,0 est
la rupture exacte du temps réel.

| Moteur | Cyl. | Cible | Moyenne | Workers | Facteur | Overruns | P éch. moyenne | Alerte BP |
|---|---:|---:|---:|---:|---:|---:|---:|:---:|
| K20A-like 2.0 I4 VTEC | 4 | 7 000 | 7 002 | 2 | 1,463 | 0 | 100,7 kPa | non |
| 2JZ-GTE-like 3.0 I6 Turbo | 6 | 6 650 | 6 650 | 2 | 1,580 | 0 | 283,5 kPa | oui |
| LS3-like 6.2 Crossplane V8 | 8 | 6 270 | 6 272 | 2 | 1,202 | 0 | 111,8 kPa | non |
| EJ25-like 2.5 Flat-4 Turbo | 4 | 6 460 | 6 461 | 2 | 1,949 | 0 | 252,0 kPa | oui |
| Audi I5-like 2.5 Turbo | 5 | 6 745 | 6 746 | 2 | 1,763 | 0 | 302,0 kPa | oui |
| Hayabusa-like 1.3 I4 | 4 | 7 000 | 7 004 | 2 | 1,818 | 0 | 101,7 kPa | non |
| Big Twin-like 1.9 V2 | 2 | 5 320 | 5 320 | 0 | 3,177 | 0 | 102,6 kPa | non |
| Merlin-like 19.8 V12 Scaled | 12 | 3 040 | 3 042 | 3 | **1,128** | 0 | 93,1 kPa | non |
| Aircooled-like 3.6 Flat-6 | 6 | 7 000 | 7 001 | 2 | 1,374 | 0 | 107,9 kPa | non |
| Radial-like 6.5 R5 | 5 | 2 280 | 2 279 | 2 | 2,487 | 0 | 100,3 kPa | non |
| Yamaha CP2 MT-07-like 689 Twin | 2 | 7 000 | 7 002 | 0 | 2,975 | 0 | 93,8 kPa | non |
| Yamaha CP3 MT-09-like 890 Triple | 3 | 7 000 | 7 002 | 2 | 2,246 | 0 | 87,4 kPa | non |
| Yamaha CP4 MT-10-like 998 Crossplane I4 | 4 | 7 000 | 7 000 | 2 | 1,869 | 0 | 93,0 kPa | non |
| VW EA288-like 2.0 TDI I4 | 4 | 4 750 | 4 751 | 2 | 2,524 | 0 | 256,7 kPa | non |
| MT-07-like 689 Twin Full System | 2 | 7 000 | 7 001 | 0 | 3,212 | 0 | 96,2 kPa | non |

La limite d'adhérence reste volontairement désactivée pour les essais sous
charge. Les alertes turbo ne sont pas masquées : leur seuil est maintenant
testé sur la pression moyenne hors pic de blowdown et tient compte du boost.
Elles signalent encore trois calibrations à examiner, pas une panne temps réel.

## Changements prouvés

### Mesures et éditeur

- Les diagnostics de richesse à charge partielle distinguent commande,
  carburant effectivement brûlé et état de film ; ils ne publient plus une
  richesse fictive.
- Le banc attend une fenêtre stable et mesure un point physique au lieu de
  confondre ondulation instantanée et régime tenu.
- L'éditeur d'échappement préserve la géométrie scalaire lors de ses
  conversions, ce qui évite une modification silencieuse du moteur.
- Le témoin NVH sourcé mesure 1 234,5 Hz, amortissement 0,025, masse modale
  4,2 kg et réponse RMS 1,36069 Pa à travers la chaîne pression-structure.

### Audio modifiable sans recompilation

Le voicing est résolu par couches `default -> famille -> moteur`, validé
strictement et rechargé à chaud hors callback audio. Il expose gains de buses,
EQ, largeur stéréo, jet de sortie et saturation. Le bypass par défaut est exact :

```text
K20 showcase 48 kHz float32, SHA-256 avant/après
851B708DC02385D7A141746484002EED63B140425324FF778E10FF93648AE221
```

Le paquet contient `voicing/default.yaml` et les outils
`EngineLabOfflineAudioExporter.exe` et `EngineLabAbClipRenderer.exe`.

### Diagnostic reconstructible

L'export K20 exécuté depuis le paquet a produit 1 468 800 frames à 96 kHz,
soit 15,3 s, avec les six buses, le premaster, le delta de traitement, le
master et la carte des ordres moteur. Preuves du manifeste :

- somme des six stems vers le premaster : erreur absolue maximale 0 ;
- premaster + delta vers le master : 1,4901161193847656e-08 ;
- carte d'ordres : 17 569 lignes ;
- troncatures de délai : 0 ; frontières invalides : 0 ; télémétries perdues : 0.

### Nouvelles sources physiques opt-in

- Le revêtement poreux de silencieux utilise trois paramètres mesurables et
  une impédance Delany-Bazley passive. Le gain ne peut pas ajouter d'énergie.
- La variabilité de combustion est un processus AR(1) déterministe, indépendant
  par cylindre, appliqué à la vitesse de flamme ou au mélange diesel. Un test
  direct observe 258 cycles, un multiplicateur de 0,88 à 1,12 et 68,04 bar de
  pression maximale.
- L'afterfire consomme réellement carburant imbrûlé et oxygène dans les cellules
  chaudes du réseau quasi-1D, conserve la masse et injecte l'énergie dans le gaz.
  Le témoin chaud mesure 129 volumes réactifs, 32,8595 mg de carburant,
  115,296 mg d'oxygène et 1 445,82 J ; le témoin froid reste inchangé.
- `examples/physical-audio-lab.els` permet d'essayer la variabilité et
  l'afterfire sans modifier les moteurs du catalogue.

## Validation et artefact

La compilation Release complète est verte. La commande suivante passe 31/31 :

```powershell
ctest --test-dir out/build/windows-vs2022 -C Release --output-on-failure -j 1
```

Le ZIP extrait contient l'application, les 15 moteurs, les assets, exemples,
voicing, documentation et outils. Depuis la racine extraite :

```powershell
.\tools\EngineLabOfflineAudioExporter.exe --catalog-root . --list-engines
```

retourne les 15 moteurs. `EngineLab.exe` reste vivant après le smoke test de
six secondes. Son SHA-256 est :

```text
D52029B9FAE1E08E7ECFC8094D6AFEBA9E9711125BCE0340CD47BCCE929180CA
```

Le hash du ZIP final est communiqué hors du ZIP afin de ne pas créer une
dépendance circulaire en l'inscrivant dans un document lui-même emballé.

## Limites restantes

1. Les preuves démontrent la cohérence, la reconstructibilité et le passage en
   temps réel ; elles ne remplacent pas un jugement humain. La prochaine étape
   prioritaire est l'écoute A/B aveugle par condition, avec références réelles.
2. Les valeurs de variabilité, de matériau poreux et d'afterfire sont des
   paramètres physiques, mais le catalogue les laisse neutres tant qu'ils ne
   sont pas calibrés par des mesures ou des références défendables.
3. Les avertissements de contre-pression du 2JZ, de l'EJ25 et de l'Audi I5 sont
   maintenant honnêtes mais restent à expliquer ou recalibrer par A/B physique.
4. EngineLab ne revendique toujours pas une supériorité sonore sur ES2D ou
   Engine Simulator 3D sans campagne d'écoute contrôlée.
