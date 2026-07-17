# Audit physique — baseline mesurée et plan de correction

Ce document fige la **baseline instrumentée** du moteur physique avant les
correctifs d'audit, et sert de référence anti-régression. Les valeurs
proviennent des instruments déterministes du dépôt, **jamais** d'une intuition
de lecture du code : conformément à `CLAUDE.md`, plusieurs « bugs » apparents
sont des compensations calibrées, et seule la mesure tranche.

## Reproduire la baseline

```
cmake --build out/build/windows-vs2022 --config Release
ctest --test-dir out/build/windows-vs2022 -C Release -R CombustionPhasing -V
ctest --test-dir out/build/windows-vs2022 -C Release -R AudioRender -V
ctest --test-dir out/build/windows-vs2022 -C Release            # bar complète (13 tests)
```

Les cibles d'exécutables (pièges de nom rappelés dans `CLAUDE.md`) :
`EngineLabCombustionPhasingTests`, `EngineLabAudioRenderHarness`,
`EngineLabCoreTests`, `EngineLabPhysicsRegressionTests`,
`EngineLabRealtimeRegressionTests`.

## Baseline combustion (`EngineLab.CombustionPhasing`)

Moteur `inline4`, cylindre 1. Le pic de pression (LPP) et CA50 sont la vérité
physique indépendante du modèle interne ; le balayage régime distingue un
modèle correct d'un modèle compensé.

| cible rpm | rpm réel | Pmax (bar) | LPP (deg) | CA10/50/90 (deg) | IMEP (bar) | burn |
| --------- | -------- | ---------- | --------- | ---------------- | ---------- | ---- |
| 2200      | 2341.39  | 74.86      | 18.00 ✓   | -3 / 11 / 25 ✓   | 13.87      | 1.00 |
| 3000      | 3085.02  | 78.07      | 18.00 ✓   | -5 / 9 / 18 ✓    | 14.58      | 1.00 |
| 4200      | 4230.84  | 70.91      | 22.00 (hors cible, toléré) | -7 / 10 / 20 ✓ | 14.45 | 1.00 |

Fenêtre attendue : LPP 18-22 deg ATDC, CA50 9-11 deg ATDC, stable sur
2300-4200 rpm (fenêtre MBT).

## Baseline audio (`EngineLab.AudioRender`, défaut `convolution`)

IR chargée : `assets/ir/exhaust_default.wav` (33705 échantillons @ 44100 Hz).
Rendus 3.0 s :

| moteur       | rms L/R        | peak L/R       | crest | brightness | bands % (bas/med/haut) | LRcorr |
| ------------ | -------------- | -------------- | ----- | ---------- | ---------------------- | ------ |
| EL-20 I4     | 0.0833/0.0829  | 0.5577/0.5886  | 6.70  | 0.127      | 23.0 / 74.6 / 2.4      | 0.645  |
| EL-50 V8     | 0.0811/0.0827  | 0.3688/0.3390  | 4.55  | 0.122      | 11.4 / 86.8 / 1.7      | 0.734  |
| EL-09 I2     | 0.1273/0.1170  | 0.5260/0.5952  | 3.29  | 0.051      | 57.8 / 42.0 / 0.2      | 0.726  |
| EL-R5 Radial | 0.0508/0.0507  | 0.2894/0.3856  | 5.70  | 0.098      | 86.2 / 13.4 / 0.4      | 0.647  |

Différenciation spectrale (similarité cosinus, plus bas = plus distinct) —
**similarité max = 0.803** (I4 vs V8), c'est le seuil de PASS :

| paire              | similarité |
| ------------------ | ---------- |
| inline4 vs v8      | 0.803      |
| inline4 vs inline2 | 0.659      |
| inline4 vs radial5 | 0.551      |
| v8 vs inline2      | 0.603      |
| v8 vs radial5      | 0.440      |
| inline2 vs radial5 | 0.534      |

Stabilité long-run (inline4, 25 s) : rms 0.1102/0.1004, crest 6.23,
`dc=+0.000`, `nearFS=0.0000`, `finite=yes`, 0 dropped/late/stolen/truncated.

RT60 échappement (Schroeder) par preset :

| preset       | rt60 noIR/FDN (s) | rt60 full (s) |
| ------------ | ----------------- | ------------- |
| street       | 0.027             | 0.123         |
| openHeaders  | 0.021             | 0.123         |
| turboMuffled | 0.141             | 0.136         |
| longTube     | 0.134             | 0.131         |
| motorcycle   | 0.022             | 0.122         |

## État de la bar complète

13/13 tests au vert au moment de l'audit. `EngineLab.Core` (107 s) contient le
test de verrouillage embrayage (`tests/CoreTests.cpp`) mais **ne vérifie pas
encore** le résidu énergétique du couplage crank/driveline (voir Phase 2).

## Constat d'architecture (non écrit ailleurs dans le dépôt)

Il existe **deux modèles de combustion** en parallèle. Le couple qui anime le
vilebrequin vient de la **pression-chambre résolue** par le solveur 0-D
(`EngineSimulator.cpp`, `gasIndicatedTorque`). Le modèle « mean-value »
(`SimplifiedGasolinePhysics::evaluateCombustion`) ne pilote **pas** la
dynamique : son couple indiqué n'est que de la télémétrie (`meanWorkTorqueNm`,
`pressureBlend` figé à 1.0). Il reste utile pour :

- `combustionQuality` (= afrEfficiency x timingEfficiency) -> amplitude audio ;
- `heatOutput` -> température de paroi cylindre -> transfert thermique ;
- `airMassMgPerCycle` -> affichage `breathingQuality`.

Le même schéma « modèle moyen calculé, modèle résolu qui gagne » vaut pour le
raté d'allumage et le cliquetis. Corriger la thermodynamique de
`SimplifiedGasolinePhysics` ne change donc **pas** le couple.

## Plan de correction (résumé)

| Phase | Cible | Verdict | Gate instrumenté | État |
| ----- | ----- | ------- | ---------------- | ---- |
| 0 | Figer cette baseline | — | aucun | ✅ fait |
| 1 | Bielle articulée : `rodAngle` calculé puis jeté (`MechanicalKinematics.cpp`) | ~~bug~~ → code mort | PhysicsRegression | ✅ fait |
| 2 | Embrayage verrouillé : couplage retardé une frame | ~~bug~~ → non-problème | Core (garde catalogue) | ✅ fait |
| 3 | Double comptage frottement paliers/piston | ~~bug~~ → non-problème | Core (garde FMEP) | ✅ fait |
| 4 | Clarté doc (γ, modèle moyen vestigial, `bearingFriction`) | clarté | build vert, baselines identiques | ✅ fait |
| 5 | Fraction volumique + facteur 1.12 (couplés) | hack calibré | CombustionPhasing + AudioRender | ⏸ fermée par défaut |

Ordre suivi : 0 → 1 → 2 → 3 → 4. La Phase 5 reste fermée : `CLAUDE.md` prévient
que corriger ces deux hacks isolément régresse la calibration MBT, et aucune
mesure n'a révélé de problème observable qui la justifierait.

## Phase 1 — résultat (hypothèse renversée par la mesure)

L'hypothèse de lecture (« le maneton articulé secondaire est probablement faux
car `rodAngle` est calculé puis jeté ») a été **infirmée par la mesure**. Un
balayage 0-720 deg du maneton articulé (cylindres 2-5 du radial) donne :

- distance au centre vilebrequin `pinR` ∈ [29.50, 97.50] mm = exactement
  `[throw-R, throw+R]` = `[63.5-34, 63.5+34]` — le maneton balaie toute la bande
  d'articulation attendue ;
- trace continue (pas max 1.19 mm/deg), strictement 360 deg-périodique ;
- course piston 127.04 mm ≈ course nominale 127 mm.

La géométrie est donc **correcte** : la rotation encodée par `rodAngle` est déjà
appliquée, en ligne, sous forme matricielle `R * Rot(A) * û` dans le calcul du
point d'articulation. `rodAngle` n'était que du **code mort**.

Actions livrées (comportement inchangé, baselines identiques, 13/13 vert) :

- suppression du code mort `rodAngle` + `(void)rodAngle` (`MechanicalKinematics.cpp`) ;
- ajout d'un test de régression verrouillant les invariants ci-dessus
  (`tests/PhysicsRegressionTests.cpp`) — les tests existants ne couvraient que
  le PMH, pas la trace complète. Non-vacuité prouvée : en sabotant la rotation
  d'articulation, le test échoue sur « must actually sweep the articulation band ».

## Phase 2 — résultat (hypothèse renversée par la mesure)

Hypothèse : le couplage vilebrequin<->embrayage verrouillé est retardé d'une
frame (la driveline sous-cycle la roue à 1 ms mais garde `engineOmega` figé sur
tout l'`advance()`), donc pour un volant léger la loi Karnopp stick/kinetic
pourrait entrer en cycle limite.

Mesure décisive : en pilotant **chaque moteur du catalogue avec sa propre
configuration** (inertie, capacité embrayage, rapports) en prise, à la cadence
de couplage la plus grossière expédiée (timeScale 4x -> pas public 1/60 s), le
couple de couplage en régime établi a une amplitude crête-à-crête **< 1 Nm** et
un glissement stabilisé dans la bande de verrouillage (~33 rpm) pour **tous** les
moteurs, EL-09 I2 (le plus léger, I=0.131) compris (0.20 Nm).

Le « chatter » initial (±capacité, 2000+ Nm) n'apparaissait que dans un banc de
test **mal apparié** : la transmission/véhicule de l'inline4 couplée à une
inertie vilebrequin étrangère artificiellement faible. Aucun moteur réel ne se
trouve dans ce régime. **Il n'y a donc pas de bug expédiable** ; le couplage
retardé d'une frame est un no-op sur les configurations réelles.

Action livrée : **aucune modification du solveur** (ne pas toucher un sous-système
en cours de mise au point pour un non-problème). Ajout d'un **test de garde
catalogue** (`tests/CoreTests.cpp`) : chaque preset, en prise, à dt=1/60 s, doit
converger sans chatter (ripple couplage < 25 Nm) et rester dans sa bande de
verrouillage. Le test existant ne couvrait qu'une seule inertie (I=0.41) à
dt=1/200 s ; celui-ci couvre tout le catalogue à la pire cadence.

## Phase 3 — résultat (hypothèse renversée par la mesure)

Hypothèse : `bearingFriction` (polynomiale empirique) et le frottement piston
Stribeck résolu couvrent le même mécanisme -> double comptage possible.

Mesure : balayage FMEP total (`state.frictionMeanEffectivePressureBar`) sur
inline4 en WOT avec charge, du ralenti au rupteur :

| rpm  | 750  | 1750 | 2750 | 3250 | 4750 | 6250 | 7250 |
| ---- | ---- | ---- | ---- | ---- | ---- | ---- | ---- |
| FMEP | 0.56 | 0.77 | 0.99 | 1.11 | 1.48 | 1.92 | 2.14 |

Courbe FMEP **de manuel** : montée quasi linéaire de ~0.6 bar au ralenti à
~2.1 bar au rupteur, en plein dans la bande essence (idle 0.5-0.8, mid 1.0-1.3,
haut 1.8-2.5). Décomposition à 3000 rpm : polynomiale ~0.59 bar, piston résolu
~0.46 bar — **complémentaires, pas de recouvrement**. Le total est bien calibré ;
re-répartir sans références dyno régresserait une courbe correcte.

Actions livrées (comportement inchangé) :

- **aucun changement de calcul** ;
- commentaire clarifiant que `bearingFriction` est en fait le FMEP non-piston
  agrégé (paliers + distribution + accessoires), le piston étant résolu à part
  (`EngineSimulator.cpp`) ;
- test de garde FMEP (`tests/CoreTests.cpp`) : la courbe FMEP inline4 doit rester
  dans l'enveloppe littérature et croître avec le régime. Non-vacuité prouvée :
  en doublant le frottement, le test échoue sur l'enveloppe.

## Bilan de l'audit (Phases 1-3)

Les trois hypothèses de lecture — maneton articulé faux, embrayage qui broute,
double comptage de frottement — ont **toutes été infirmées par la mesure**. C'est
exactement l'avertissement de `CLAUDE.md` : « This codebase reads as if it is full
of bugs. Several of them are not. » Aucun bug expédiable trouvé dans ces zones ;
la valeur livrée est la **couverture de régression** (trace articulée, anti-chatter
catalogue, enveloppe FMEP) qui verrouille le comportement mesuré-correct, plus le
retrait d'un code mort et des clarifications de commentaires (Phase 4). Reste, si
besoin produit : Phase 5 (fraction volumique/1.12, fermée par défaut).

## Phase 4 — clarté (comportement strictement inchangé)

Corrections de commentaires seulement, pour que le prochain lecteur (humain ou
agent) ne reparte pas sur une fausse piste :

- `ConservativeGasSystem` : le commentaire disait « blend linéaire » alors que le
  code applique l'identité exacte `γ = 1 + R/Cv_eff` ; corrigé dans le `.cpp` et
  le `.hpp`. Ce n'est pas une interpolation en fraction brûlée : c'est le Cv du
  mélange qui porte la non-linéarité.
- `SimplifiedGasolinePhysics.hpp` : ajout d'un bloc « IMPORTANT for maintainers »
  qui dit noir sur blanc que le couple indiqué de ce modèle **ne pilote pas** le
  vilebrequin (c'est la pression-chambre résolue qui le fait), et liste ce que le
  modèle moyen alimente réellement (audio, thermique, affichage). C'est le piège
  d'audit numéro un du dépôt.
- `EngineSimulator` : commentaire clarifiant que `bearingFriction` est le FMEP
  non-piston agrégé (déjà livré en Phase 3).

---

# Guide pour les futurs agents IA

Cette section est écrite pour l'agent (ou l'humain) qui reprendra la physique de
ce dépôt. Elle condense ce qui a fait perdre du temps, la méthode qui marche ici,
et où sont les pièges. Lis-la avant de « corriger » quoi que ce soit.

## 1. La règle d'or : mesurer avant de corriger

Ce code **se lit comme s'il était plein de bugs. Il ne l'est pas.** Beaucoup de
« bugs » évidents à la lecture sont des compensations calibrées, vérifiées par
mesure. Pendant cet audit, **trois** hypothèses de lecture confiantes ont été
posées puis **toutes infirmées par la mesure** :

1. « Le maneton articulé est faux car `rodAngle` est jeté. » -> Faux : la trace
   mesurée balaie exactement `[throw-R, throw+R]`, continue et périodique. C'était
   du code mort, la rotation étant déjà inlinée.
2. « L'embrayage verrouillé broute (couplage retardé). » -> Faux : chaque moteur
   avec sa vraie config a un ripple < 1 Nm. Le chatter n'existait que dans un banc
   mal apparié (config inline4 + inertie étrangère).
3. « Le frottement paliers double-compte le frottement piston. » -> Faux : la
   courbe FMEP mesurée (0.56 -> 2.14 bar) est de manuel ; les deux termes sont
   complémentaires.

La leçon : une hypothèse de lecture n'est **pas** un constat. Tant que tu n'as pas
un chiffre d'un instrument déterministe, tu n'as rien. Le coût d'un correctif à
l'aveugle ici est une régression de calibration silencieuse.

## 2. Où vit la physique, et QUI pilote réellement le couple

Carte mentale indispensable — il y a **deux** modèles de combustion :

- **Le solveur qui compte** : `ConservativeGasSystem` (volumes de contrôle 0-D,
  masse/énergie/quantité de mouvement conservées) résout la pression-chambre de
  chaque cylindre à chaque sous-pas. C'est `gasIndicatedTorque` dans
  `EngineSimulator::step` qui **intègre le vilebrequin**. Toute la dynamique
  (couple, régime, accélération) vient de là.
- **Le modèle moyen télémétrique** : `SimplifiedGasolinePhysics::evaluateCombustion`
  a l'air d'être le cœur thermodynamique (rendement, avance optimale, pression
  idéale…) mais son couple indiqué **ne bouge pas le vilebrequin**. Il finit dans
  `state_.meanWorkTorqueNm` (affichage). Il alimente seulement : l'**amplitude
  audio** (`combustionQuality`), la **température de paroi** (`heatOutput`), et
  l'affichage. `pressureBlend` est figé à 1.0 (potard mort).

Corollaire : **retoucher la thermo de `SimplifiedGasolinePhysics` ne changera pas
le couple au banc.** Le même schéma « modèle moyen calculé, modèle résolu qui
gagne » vaut aussi pour le raté d'allumage et le cliquetis (résolus par cylindre
dans `EngineSimulator` / `EndGasKnockModel`).

Frontières des couches :
`IPhysicsModel` (politique pure, sans horloge/thread/UI) -> `EngineSimulator`
(orchestration : gaz, frottements, thermique, intégration couple) -> `EngineRuntime`
(temps réel, driveline, dyno, publication) -> app/audio.

## 3. Les instruments (ta seule source de vérité)

| Instrument | Cible ctest | Ce qu'il mesure |
| ---------- | ----------- | --------------- |
| Combustion phasing | `EngineLab.CombustionPhasing` | LPP, CA10/50/90, IMEP sur balayage régime |
| Audio render | `EngineLab.AudioRender` | RMS/crest/DC/bandes spectrales, RT60, différenciation presets |
| Physics regression | `EngineLab.PhysicsRegression` | gaz conservatif, cinématique (dont trace articulée) |
| Core | `EngineLab.Core` | driveline, embrayage, **garde FMEP**, **garde anti-chatter** |
| Catalog physics | `EngineLab.CatalogPhysics` | tous les moteurs tournent, finis, stables, audio non-silencieux |

Les valeurs de référence de ces gates viennent de la **littérature moteur/DSP**,
jamais de la sortie actuelle du simulateur. Ne resserre jamais un gate sur le
comportement courant : ça re-calibrerait le test sur ce qu'il est censé attraper.

## 4. Procédure pour instrumenter une hypothèse (ce qui a marché ici)

1. **Isoler** la grandeur physique observable (trace géométrique, ripple de
   couple, FMEP…) — pas une intuition de code.
2. **Sonder** : ajouter un bloc de diagnostic temporaire dans le test qui a déjà
   accès aux bons objets (`PhysicsRegressionTests` pour la cinématique/gaz,
   `CoreTests` pour la chaîne complète moteur+driveline), builder la **bonne**
   cible, lire les chiffres.
3. **Décider** avec un critère externe (littérature), pas relatif au code.
4. Si correction nécessaire : l'écrire, puis **prouver la non-vacuité** — saboter
   volontairement et vérifier que le test échoue, rebuild la bonne cible,
   restaurer. Si aucune correction : reverter la sonde, garder un **test de garde**
   qui verrouille le comportement mesuré-correct.
5. Rebuild complet + `ctest` complet. Comparer aux baselines de ce document.

## 5. Pièges concrets (qui ont coûté du temps, y compris pendant cet audit)

- **Binaire périmé.** Une cible CMake mal nommée échoue (`MSB1009`) en ne
  construisant rien, et `ctest` relance alors l'**ancien** binaire — un test peut
  « passer » contre du code non compilé. Vécu pendant cet audit : un
  warning-as-error (`C4456`, variable masquée) a fait échouer le build, et le
  « passed » venait du binaire périmé. **Toujours** vérifier que la cible a
  relinké (`... .exe` dans la sortie de build) avant de croire un résultat. Ne
  filtre pas la sortie de build au point de cacher un `MSB`/`warning C`.
- **Warnings = erreurs.** Une variable inutilisée, une déclaration masquée : build
  cassé. Nettoie tes sondes.
- **Bancs mal appariés.** Piloter un modèle avec des paramètres qui ne
  correspondent à aucune config réelle fabrique de faux bugs (cf. le « chatter »
  embrayage). Toujours tester avec les configs **du catalogue**
  (`makeBaseEnginePresets()`), appariées.
- **Noms trompeurs.** `bearingFriction` = FMEP non-piston agrégé (pas seulement
  les paliers). Le modèle « moyen » n'est pas le modèle principal. Lis les
  commentaires ajoutés en Phase 4 avant de conclure.

## 6. Zones à NE PAS toucher sans mesure ni raison produit

- **Le voicing audio par défaut** (réglage `convolution`) : vérifié bit-identique
  avant/après les corrections de cet audit. Un changement audio doit être un
  changement de voicing **assumé**, prouvé par `AudioRender`.
- **Fraction volumique de flamme + facteur 1.12** (`FlamePhysicsModel`) : deux
  hacks **couplés** qui se compensent et tiennent la fenêtre MBT mesurée. Les
  corriger isolément régresse `CombustionPhasing`. Phase 5, fermée par défaut.
- **Le solveur embrayage** (`DrivelineModel`) : en cours de mise au point sur la
  branche `fix/clutch-lockup`. Stable pour toutes les configs réelles.

## 7. Idées de suivi (non urgentes, mesure-gated)

- Exposer la décomposition FMEP (paliers/piston/vilebrequin) en télémétrie pour
  auditer le split sans sonde temporaire.
- Ajouter un audit d'énergie inter-couches (le résidu existe déjà côté roue dans
  `DrivelineModel::energyResidualJoules` ; l'étendre au couplage crank<->embrayage).
- Quand des courbes dyno de référence seront disponibles, valider le couple/
  puissance absolus (aujourd'hui `CatalogPhysics` vérifie la stabilité, pas les
  valeurs absolues).
