# Audit physique — baseline mesurée et plan de correction

> **Archive de provenance.** Les sections audio de cet audit décrivent le chemin
> antérieur au réseau gaz quasi-1D et au rayonnement passif. Les décisions encore
> actives sont documentées dans `thermoacoustic-architecture.md`.

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
| 6 | Modèle moyen vestigial : ~150 lignes de « thermo morte » à extraire/supprimer | ~~code mort~~ → thermo vivante | Core + AudioRender + Catalog | ✅ fait (quarantaine doc) |

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

## Phase 6 — résultat (hypothèse renversée par la mesure)

Hypothèse : `SimplifiedGasolinePhysics::evaluateCombustion` (modèle moyen) contient
~150 lignes de thermo morte — **avance optimale**, **rendement de timing**,
**pression idéale par les moles brûlées** — qui « ne pilotent rien » et pourraient
être extraites/supprimées.

Mesure décisive : grep exhaustif de **tous** les consommateurs de `CombustionResult`
(la struct n'a que deux sites de lecture en prod, `EngineSimulator` et
`FourStrokeEventGenerator`, plus le gate `EngineLab.Core`), croisé avec le graphe
de dépendances du modèle. Carte de consommation des 13 champs :

| champ | lu en prod | lu par test Core | rôle |
| ----- | ---------- | ---------------- | ---- |
| `combustionEnabled` | ✅ | ✅ | gate injection/combustion/event-gen (vivant) |
| `combustionQuality` | ✅ (inconditionnel) | — | amplitude audio (vivant) |
| `heatOutput` | ✅ | — | température de paroi (vivant) |
| `airMassMgPerCycle` | ✅ | — | `breathingQuality` (affichage) |
| `misfireProbability` | ✅ (fallback) | — | fallback event-gen |
| `pressureEstimateBar` | ✅ (fallback) | ✅ | fallback event-gen + caractérisé |
| `indicatedTorqueNm` | ✅ → `meanWorkTorqueNm` | ✅ | télémétrie d'affichage |
| `actualAirFuelRatio` | ❌ | ✅ | verrouillé par le gate Core |
| `heatPowerKw` | ❌ | ✅ | verrouillé par le gate Core |
| `knockLevel` | ❌ | ❌ | constante 0.0 neutralisée |
| `volumetricEfficiency` | ❌ | ❌ | intermédiaire exposé (→ `airMassMg`) |
| `fuelMassMgPerCycle` | ❌ | ❌ | intermédiaire exposé (→ `heatOutput`) |
| `thermalEfficiency` | ❌ | ❌ | intermédiaire exposé (→ `indicatedTorque`) |

Les trois calculs nommés comme « morts » alimentent en réalité des champs
consommés : `optimumAdvance` → `timingEfficiency` → **`combustionQuality`** (audio) ;
`idealPressureBar` → **`pressureEstimateBar`** (fallback event-gen + test Core).
**Il n'y a pas de bloc de ~150 lignes mort** — 4ᵉ hypothèse de lecture infirmée
par la mesure. Le seul mort prouvé est 4 champs de struct, dont 3 restent des
intermédiaires nécessaires ; `actualAirFuelRatio`/`heatPowerKw` casseraient le gate
Core. La suppression/extraction régresserait l'audio et un gate pour un gain nul.

Action livrée (option A — quarantaine documentaire, comportement strictement
inchangé, baselines identiques, 13/13 vert) :

- carte de consommation annotée sur `CombustionResult` (`EngineTypes.hpp`), un
  commentaire par champ pointant son lecteur unique ;
- `return` du modèle réécrit en initialiseurs désignés C++20 groupés/annotés
  (`SimplifiedGasolinePhysics.cpp`), valeurs et ordre identiques ;
- **test de caractérisation** (`tests/CoreTests.cpp`) qui fige le câblage audio :
  sans état cylindre résolu, l'intensité de tir doit être linéaire en
  `combustionQuality` et la pression doit retomber sur `pressureEstimateBar`.
  Non-vacuité prouvée : en débranchant `combustionQuality` de l'event-gen, le test
  échoue sur « intensity must scale linearly with combustionQuality ».

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
- **`CylinderState::runnerPressureKpa` est le runner d'ÉCHAPPEMENT.** Le nom ne
  le dit pas, et un `grep "runnerPressureKpa ="` ne trouve **aucune** écriture :
  le champ est rempli **positionnellement** par l'initialiseur agrégé de
  `EngineSimulator.cpp` (~l. 1740, `exhaustRunnerPressureKpa_[index]`). Le côté
  admission est `intakeRunnerChargePressureKpa` (colonne `irp_kpa` de la trace).
  Coût vécu : la colonne s'appelait `runner_kpa` dans `--trace` ; lue comme le
  port d'admission elle donne 112-176 kPa et on conclut que le runner
  d'admission est 30 kPa au-dessus de l'ambiante, alors qu'il est à ±5 kPa
  d'elle — diagnostic inversé. La colonne s'appelle maintenant
  `exh_runner_kpa`. Corollaire général : dans ce dépôt, un champ sans écriture
  visible au grep est probablement rempli par un initialiseur positionnel —
  compte les champs, ne fais pas confiance au nom.

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


## Étage fautif du remplissage : la soupape d'admission n'étrangle pas (mesuré)

Mesuré au `EngineLabPhysicsPerfHarness` (plein gaz, moteur tenu au régime), LS3
à 5940 tr/min. Chaque étage de la chaîne de remplissage a été relâché **seul**,
et le seul critère est le déplacement de la VE.

| variante | VE | couple | EGT |
|---|---|---|---|
| référence | 0.328 | 92 Nm | 948 °C |
| soupape adm. x1.4 (section x2) | **0.326** | 94 Nm | 949 °C |
| levée adm. x1.5 | 0.347 | 109 Nm | 925 °C |
| durée adm. +60° | 0.340 | 110 Nm | 894 °C |
| soupape éch. x1.4 | 0.340 | 110 Nm | 931 °C |
| échappement grand ouvert | 0.341 | 144 Nm | 876 °C |
| papillon + plénum x2 | 0.343 | 103 Nm | 930 °C |
| **runner adm. x1.6** | **0.439** | 138 Nm | 808 °C |
| **runner adm. x2.0** | **0.613** | 29 Nm | 422 °C |

**Le diagnostic est dans l'asymétrie, pas dans un chiffre isolé.** Doubler la
section de la soupape d'admission ne change **rien** (0.328 → 0.326), et ne
change toujours rien quand on l'ajoute par-dessus un runner doublé
(0.613 → 0.610). Doubler le diamètre du runner fait **+87 %** de VE, sans
saturation.

Or dans un moteur réel à 5900 tr/min la soupape *est* la restriction : c'est
l'ouverture la plus petite et la plus brève de tout le conduit. Ici, pour le
LS3, la section de passage moyenne à la soupape (~775 mm² sur l'événement,
55 mm de diamètre, 12.8 mm de levée, Cd 0.70) est **inférieure à la moitié** de
la section du runner (1963 mm² pour 50 mm) — et pourtant c'est le runner qui
mesure le débit. **Le rapport de restriction est inversé.**

### Ce que ça coûte, en chaîne

VE 0.33 (au lieu de 0.85-1.05) → IMEP 3.5 bar (au lieu de 9-12) → **91 Nm là où
le moteur modélisé en fait ~520** → la chaleur qui aurait dû devenir du travail
part à l'échappement, d'où **EGT 1256 °C** (au lieu de 800-950).

Côté audio, deux conséquences distinctes :

1. **Désaccord global.** c = sqrt(gamma R T) : 770 m/s à 1256 °C contre 674 à
   900 °C. Tous les retards du guide d'ondes sont 14 % trop courts, donc tout
   l'échappement sonne ~2.3 demi-tons trop haut. L'erreur est du même ordre au
   ralenti (15 %), donc c'est un désaccord quasi constant.
2. **Excitation inversée.** L'IMEP chute d'un facteur 3 du ralenti à la zone
   rouge, et c'est la détente qui excite tout le guide d'ondes. Un vrai
   échappement durcit en montant ; celui-ci s'éteint. C'est probablement une
   part importante du « ça manque de rage en haut » et du « ils se ressemblent »,
   puisque tous les moteurs convergent vers une excitation faible et similaire
   là où ils devraient le plus se distinguer.

### Deux fausses pistes écartées par la mesure

- **`state_.exhaustPressureKpa` est un `max` sur tous les ports** (le pic de
  détente d'un cylindre quelconque), pas une moyenne. Ça *semble* être la cause
  de la contre-pression apparente de 0.87 à 1.74 bar. Remplacé par une moyenne :
  la pression rapportée baisse, **la VE, le couple et l'EGT ne bougent pas d'un
  chiffre**. C'est de la télémétrie ; le remplissage vient du réseau de gaz
  résolu, pas de cette valeur.
- **Le plafond apparent à VE ~0.43** ("tout relâché" n'allait pas plus haut)
  n'existe pas : cette variante n'ouvrait le runner qu'à x1.6. À x2.0 la VE
  passe à 0.613 sans saturer.

### Limite de cette localisation (levée depuis — voir la section suivante)

La localisation ci-dessus est **empirique** : elle dit *quel réglage déplace la
VE*, pas *pourquoi*. Le mécanisme a été trouvé ensuite, et il corrige la lecture
« rapport de restriction inversé » : le runner ne mesure pas le débit parce
qu'il serait trop étroit, mais parce que sa **quantité de mouvement** était
fausse. Le diamètre du runner était le seul levier efficace parce que c'est le
seul qui change la vitesse du gaz dans la maille.

## Mécanisme du défaut de remplissage : la quantité de mouvement du jet (mesuré)

Le traceur `--trace` du `EngineLabPhysicsPerfHarness` (une ligne tous les ~2°
vilebrequin, avec l'état de la charge des **deux** côtés de la soupape) a montré
que la panne n'est ni une pression ni une section, mais une **densité** :

| LS3 plein gaz | ralenti 786 | zone rouge 5949 |
|---|---|---|
| pression runner admission | 100.7 kPa | 95.7 kPa |
| **température charge runner** | **71 °C** | **333 °C** (pics à 753) |
| vitesse gaz runner, moyenne | 35 m/s | **172 m/s** |
| masse piégée | 798 mg | 366 mg |

La continuité donne ~54 m/s de moyenne pendant l'événement d'admission pour ce
runner. Le modèle en portait **172**, et ne descendait jamais sous 115 m/s
**même soupape fermée**.

`injectJetMomentum()` ajoutait `masse_transférée x v_jet` à la maille à chaque
appel de flux — **en plus** de l'advection de quantité de mouvement que
`transfer()` fait déjà. Une maille traversée comme un runner voit passer
plusieurs fois sa propre masse pendant un seul événement de soupape, donc les
incréments s'accumulent jusqu'à `(débit traversant / masse de la maille)` fois
la valeur physique — ici x3.2, ce qui colle au 172 / 54 mesuré.

Chaîne de conséquences, chacune mesurée dans le traceur :

1. quantité de mouvement gonflée → `dynamicPressureKpa()` gonflée (7 à 23 kPa
   fictifs) → **elle s'oppose au remplissage plénum → runner** ;
2. le runner tombe sous l'ambiante (85 kPa mini) ;
3. à l'IVO, le cylindre est donc systématiquement au-dessus du runner et y
   refoule du gaz à ~1000 °C : la charge du runner monte à 753 °C et ne
   redescend jamais sous 289 °C avant l'admission suivante ;
4. le moteur respire de l'air à 333 °C sous 96 kPa — **densité 2.6x trop
   faible**. C'est toute la VE manquante, en un seul chiffre.

**Le correctif** (`injectJetMomentum`) fait *relaxer* chaque maille vers la
vitesse de continuité du jet au lieu de lui ajouter un incrément, au taux de la
fraction de sa propre masse échangée dans le pas. C'est la formulation en volume
de contrôle de la même physique, et elle coïncide avec l'ancien incrément tant
que cette fraction est petite : une maille en régime établi s'arrête à la
vitesse du jet au lieu de la dépasser.

### Ce que la correction délivre (mesuré)

| LS3 plein gaz | avant | après | littérature |
|---|---|---|---|
| vitesse runner (5949) | 172 m/s | **48 m/s** | ~54 (continuité) |
| température runner (5949) | 333 °C | **84 °C** | 40-80 |
| VE ralenti / 3640 / 5990 | 0.75 / 0.47 / 0.33 | **0.94 / 0.78 / 0.59** | 0.85-1.05 |
| couple 5990 | 91 Nm | **278 Nm** | ~520 |
| EGT 5990 | 1256 °C | **1019 °C** | 800-950 |

Et à la sortie du rendu audio livré, pression au micro à 1 m :

| moteur | avant | après |
|---|---|---|
| LS3 V8 | 32.6 Pa | **255.2 Pa** (+17.9 dB) |
| K20 I4 | 99.8 Pa | 207.6 Pa |
| 2JZ I6 turbo | 105.8 Pa | 222.3 Pa |
| **Merlin V12 (compresseur)** | 101.6 Pa | 129.6 Pa |

**C'est la signature du défaut** : le V12 est suralimenté (MAP 130-176 kPa), donc
le plénum poussait la charge quoi qu'il arrive et il bouge à peine. Tous les
atmosphériques étaient affamés. C'est exactement le rapport d'écoute qui a lancé
cette recherche (« le V12 sonne très bien, les autres restent assez moyens »).

Reste : la VE tombe encore à 0.59 en zone rouge au lieu de rester plate, et
l'EGT reste ~100 °C trop haute. La panne suivante est en aval de celle-ci, pas
résolue par elle.

### Garde-fou

`EngineLab.PhysicsRegression` contient désormais une sonde d'écoulement établi
qui exige qu'une maille de conduit ne dépasse pas sa vitesse de continuité, et
surtout que **passer 8x plus de masse à travers elle ne la fasse pas tourner
plus vite**. La référence est la continuité — une identité, pas un étalonnage —
donc ce garde-fou ne peut pas être re-calibré sur la sortie du simulateur.
Vérifié non vacu : sans le correctif il échoue.

## Le verrou suivant : la frontière de sortie du réseau (mesuré)

Après le correctif de quantité de mouvement, la VE tombait encore de 0.94 à
0.59 entre le ralenti et la zone rouge, et l'EGT restait à 1019 °C. Le traceur
donne la cause en une image : le cylindre arrive au PMH d'échappement à
**6.1 bar et 1942 K**, et garde donc **87 mg de résiduels** là où un moteur réel
en garde ~25. Détendus, ces 87 mg occupent **62 % de la cylindrée** (13 % au
ralenti) : il ne reste pas de place pour l'air.

Le cylindre ne se vide pas parce que le runner d'échappement est à 2.1 bar. Et
cette contre-pression n'est ni une géométrie ni une soupape :

| étage relâché (LS3, 5940) | VE |
|---|---|
| référence | 0.68 |
| primaires x2 | 0.70 |
| collecteur x2.6 | 0.69 |
| sortie x3.7 | 0.71 |
| soupape d'échappement 40.4 → 52 mm | 0.72 |

Le débit passant la soupape colle d'ailleurs à ±20 % de la capacité d'un orifice
sonique de sa section instantanée : elle n'est pas bridée. La mesure décisive est
la pression **le long** du réseau :

| rpm | port | sortie | chute interne | vitesse en sortie |
|---|---|---|---|---|
| 824 | 119.4 | 117.6 | 1.8 kPa | 17 m/s |
| 5994 | 210.1 | **180.1** | 30 kPa | **68 m/s** |

**La sortie elle-même est 79 kPa au-dessus de l'ambiante en ne débitant qu'à
68 m/s**, alors qu'une détente libre vers 101.3 kPa en donnerait 699. Seuls 30
des 109 kPa se perdent dans les tuyaux. Confirmation directe : en allégeant le
réservoir ambiant d'un facteur 17 à pression inchangée, la sortie tombe à
117 kPa. **C'était la condition aux limites, pas la tuyauterie.**

L'atmosphère à un bout ouvert est un *réservoir*. Résoudre un problème de Riemann
entre la dernière maille du conduit et une *maille* d'air ambiant oblige
l'échappement à pousser une colonne semi-infinie de gaz froid et dense : le flux
est alors borné par l'impédance acoustique **de l'ambiante**, `rho_a*c_a`, qui
pour de l'air à 22 °C dépasse celle du gaz d'échappement chaud. C'est faux
acoustiquement pour la même raison : un bout ouvert doit renvoyer une compression
en dépression, et contre un réservoir plus dense il réfléchissait avec le signe
d'un bout **fermé** (r ≈ +0.15).

Le correctif applique le traitement caractéristique standard : imposer la
pression statique du réservoir au plan de sortie et prendre tout le reste de
l'invariant sortant `u + 2c/(gamma-1)`, seule information que l'intérieur envoie
à la frontière tant que l'écoulement y est subsonique ; au blocage sonique
l'état intérieur reste tel quel.

| LS3 plein gaz | jet seul | + bout ouvert | littérature |
|---|---|---|---|
| VE ralenti / 3640 / 6070 | 0.94 / 0.78 / 0.59 | **0.95 / 0.83 / 0.67** | 0.85-1.05 |
| couple | 621 / 497 / 278 Nm | **626 / 570 / 367 Nm** | ~470 / 575 / 520 |
| EGT | 478 / 800 / 1019 °C | **426 / 712 / 876 °C** | 800-950 |
| contre-pression crête | 127 / 201 / 231 kPa | **110 / 139 / 154 kPa** | ~110-150 |

L'EGT en haut est désormais **dans la fenêtre de la littérature**, et le couple
de pointe tombe à 570 Nm contre 575 pour la référence.

### Deux départs ratés, gardés en commentaire dans le code

Les deux ont fait **caler tout le catalogue sauf un moteur**, ce qui est la seule
raison pour laquelle ils sont documentés plutôt que oubliés :

- **Prendre le flux physique de l'état frontière directement.** Correct sur le
  papier (c'est la formulation NSCBC), mais la maille terminale n'est pas
  uniformément à l'état de sortie : une détente de blowdown la vide entièrement
  en un sous-pas. La forme *ghost cell* repasse par le solveur de Riemann, qui
  garde ses vitesses d'onde et ses garde-fous de positivité.
- **Prolonger l'invariant à travers le contact au reflux.** L'invariant
  `u + 2c/(gamma-1)` ne vaut que dans un même gamma et une même entropie.
  L'évaluer sur la vitesse du son du gaz chaud (~700 m/s) tout en lui attribuant
  la densité de l'ambiante produisait **+2 km/s dans la branche censée modéliser
  une entrée d'air**. Au reflux, le réservoir ambiant au repos est la bonne
  réponse, simplement.

Une fausse alerte de perf aussi, et c'est exactement le piège que ce dépôt
documente : le coût semblait exploser jusqu'à ce que la comparaison soit refaite
contre une base de la **même session**. À géométrie et session identiques le
surcoût est de **+3.7 %** (LS3 à 6070 : 4565 → 4734 µs). La trame dépassait déjà
son budget avant ce changement ; c'est le problème connu et distinct du fil
physique.

### Deux fausses pistes de plus, écartées par la mesure

- **La contre-pression d'échappement.** Le runner d'échappement ne descendait
  jamais sous 164 kPa à 5949 tr/min (contre 105 au ralenti), et le cylindre
  était encore à 4.4 bar / 1964 °C à l'ouverture de l'admission. Hypothèse
  évidente : le cylindre ne se vide pas, donc il refoule à l'admission. Testé en
  ouvrant tout l'échappement (primaires x2, collecteur x2.6, sortie x3.7) : la
  contre-pression tombe à 109 kPa, la pression à l'IVO à 2.30 bar, et la masse
  piégée **ne bouge pas** (366.6 → 365.9 mg). Le runner d'admission chauffe même
  un peu plus. Ce n'était pas la cause.
- **Le recouvrement de soupapes.** Testé proprement (durée 260 → 210 *et*
  centrale 110 → 135, donc IVO retardé de 50° avec **IVC inchangé**) : plus
  aucun recouvrement, pression à l'IVO 1.16 bar au lieu de 4.37 — et le runner
  monte à 405 °C au lieu de 333. Le recouvrement n'était pas la source de
  chaleur. (Un premier essai qui ne bougeait que la centrale était confondu :
  il déplaçait l'IVC en même temps.)

## Le plafond haut-régime : pas de suralimentation par inertie/résonance (mesuré)

Le « verrou aval » que les deux sections précédentes signalent sans le résoudre
(« la VE tombe encore à 0.59-0.67 en zone rouge au lieu de rester plate ») a été
localisé et quantifié en portant au catalogue trois moteurs moto à haut régime
— Yamaha CP2/CP3/CP4 (`engines/11-13`, calage crossplane authentique via
`crank_offset_deg`) — et en les passant au banc plein-gaz
(`EngineLabDynoSweepHarness`, balayage régime→couple, réutilise le contrôleur
d'absorbeur de `measurePoint`).

### Symptôme livré (banc vs données constructeur)

| moteur | puissance pic sim / réel | couple pic sim / réel | régime couple pic sim / réel |
|---|---|---|---|
| CP2 (MT-07) | 36 / 55 kW (**66 %**) | 66 / 67 Nm (99 %) | **3500** / 6500 |
| CP3 (MT-09) | 51 / 87 kW (**58 %**) | 81 / 93 Nm (87 %) | **3500** / 7000 |
| CP4 (MT-10) | 56 / 118 kW (**48 %**) | 82 / 111 Nm (74 %) | **5000** / 9000 |

Le couple de pointe est presque juste quand il tombe bas dans les tours (CP2,
99 %) mais **le pic est systématiquement 1500-4000 tr/min trop tôt**, et la
puissance plafonne à 48-66 % parce que le couple s'est effondré avant d'atteindre
le régime de puissance. La VE décroît de façon **monotone dès le plus bas régime**
(CP4 : 0.82 à 2000 → 0.69 à 11500) au lieu de culminer haut. Plus un moteur
dépend de sa respiration à haut régime, plus le simulateur le sous-évalue.

### Localisation (méthode « relâcher chaque étage », CP4 à 10800 tr/min)

| variante | VE | Δ |
|---|---|---|
| référence | 0.722 | — |
| soupape adm. Ø ×1.6 | 0.744 | +0.022 |
| runner Ø ×1.6 | 0.749 | +0.027 |
| **longueur runner ÷2** | **0.719** | **~0** |
| runner + soupape ×1.6 | 0.789 | +0.067 |

Contrairement à l'audit LS3 d'**avant** le correctif de quantité de mouvement (où
le runner seul donnait +87 %), **la géométrie n'est plus le levier dominant** :
×2.5 de section (runner+soupape) ne rend que +0.067 de VE, et le plafond persiste.
Surtout, **la longueur de runner est inerte** (0.719 vs 0.722) — or c'est LE
levier d'accord d'admission d'un vrai moteur (il déplace le pic de VE de
±2000 tr/min). Un accord d'admission sans effet sur le remplissage est la
signature du défaut.

### Deux fausses pistes écartées par la mesure

- **Résolution du solveur.** Hypothèse : la cadence livrée sous-résout l'échange
  gazeux à haut régime (la trace `--trace` à sous-pas ×48 remplit mieux). Testé
  proprement : `gas_substeps` 1→8 et `maximum_crank_deg_per_step` 2.0→0.8
  (144 → 361 sous-pas à 10800). La VE **ne bouge pas** (0.722 → 0.722 puis 0.716,
  soit très légèrement pire). Le rolloff est un comportement du **modèle**, pas
  un artefact numérique ; le meilleur remplissage de la trace était une différence
  de définition de VE, pas de physique. À noter au passage : la validation
  plafonne `maximum_mechanical_frequency_hz` à 100 kHz, ce qui à 12500 tr/min de
  rupteur borne déjà la résolution à ~0.8°/pas.
- **Surchauffe de charge à bas régime.** À 3000 tr/min la charge runner est à
  25 °C, résiduels 1.5 %, le cylindre atteint la pression collecteur : le bas
  régime respire correctement (couple ≈ réel). La surchauffe n'apparaît qu'en
  montant (runner 40-100 °C à 9000) et reste secondaire.

### Mécanisme (code + trace)

Le cylindre se remplit jusqu'à la densité **statique** du collecteur, pas au-delà.
Aucune suralimentation par inertie ni par résonance n'atteint le remplissage :

1. La seule résonance d'admission (`HelmholtzRunnerModel`) ne module que le Cd du
   flux **plénum→runner** (`EngineSimulator.cpp`, `0.78 * flowAdmittance`). Elle
   n'atteint jamais le flux **runner→cylindre** (la soupape). D'où l'inertie
   mesurée de la longueur de runner : la fréquence de résonance en dépend, mais
   son seul effet est en amont du remplissage.
2. Le terme de pression dynamique (ram) existe mais est borné par le solveur
   d'équilibre par sous-pas (`ConservativeGasSystem::pressureEquilibriumMoles`),
   qui interdit au cylindre de dépasser la pression **effective** du runner dans
   un pas — donc pas de remplissage au-dessus du statique collecteur.

Conséquence à l'IVC (trace CP4 3000 tr/min) : la masse piégée culmine à 304 mg
puis **retombe à 289** avant fermeture — la charge est refoulée faute de colonne
d'admission en mouvement pour la retenir. La VE ne peut donc ni dépasser le
plafond densité-collecteur (~0.80-0.85) ni **culminer** à haut régime.

### Portée et plan (mesure-gaté, non implémenté)

C'est une limite **générale**, pas propre aux motos : tout moteur dont la courbe
réelle de VE repose sur l'accord admission/échappement pour respirer haut
(sportives, moteurs de course, motos) sera sous-évalué de ~50 % en puissance de
pointe avec un pic de couple trop bas. Séquence de correction proposée, chaque
étage gaté (littérature, jamais la sortie du simulateur) :

0. **Gate d'abord.** Promouvoir `EngineLabDynoSweepHarness` en régression VE(rpm)
   / BMEP(rpm) : pour un moteur haut-régime de référence, la VE doit rester
   ≥ 0.90 jusqu'à ~85 % du rupteur, et le **pic de VE doit se déplacer ≥ 1000
   tr/min quand la longueur de runner double** (cibles Heywood, accord
   inertiel/acoustique).
1. **Router la pression de résonance vers le remplissage.** Ajouter l'amplitude
   `HelmholtzRunnerModel` à la pression effective du runner **vue par la soupape**
   (flux runner→cylindre), phasée au cycle — rend la longueur de runner active et
   laisse la VE culminer au régime accordé.
2. **Autoriser le sur-remplissage inertiel après le PMB.** Assouplir le plafond
   `pressureEquilibriumMoles` sur la soupape d'admission pour que la quantité de
   mouvement de la colonne (pression dynamique) pousse la charge au-dessus du
   statique collecteur jusqu'à l'IVC. À faire sous garde de conservation/positivité
   (ce plafond a été ajouté pour la stabilité).
3. **Inertance de runner physique.** Modéliser la colonne de gaz du runner comme
   une vraie inertance 1D (`L·dṁ/dt = ΔP·A`, ou un runner à deux mailles), version
   robuste du point 2 et voie standard pour obtenir VE > 1.
4. **Secondaire.** Une fois le ram en place, revoir la prise de chaleur runner et
   la FMEP au-delà de 8000 tr/min contre la littérature (FMEP CP4 ≈ 2.2 bar à
   9000) pour que la VE corrigée ne soit pas re-mangée.

Risque : ces points touchent le solveur gaz cœur ; l'historique de ce document
montre qu'un changement isolé y régresse la calibration (voicing audio, fenêtre
MBT). D'où gate-d'abord et étage-par-étage, exactement comme les correctifs de
remplissage précédents.

### Localisation fine : le résonateur est bien accordé, il est mal forcé (mesuré)

La section ci-dessus dit « la résonance ne module que le Cd amont ». C'est vrai
mais incomplet, et la partie manquante change le correctif. Avec la trace
instrumentée (`res_kpa` = amplitude `HelmholtzRunnerModel`, `res_hz` = sa
fréquence propre), CP4 à 9013 tr/min :

| grandeur | mesure | attendu (littérature) |
|---|---|---|
| `res_hz` fréquence propre | **138.8 Hz** | — |
| fréquence d'événement soupape (rpm/120) | 75.1 Hz | — |
| **nombre de Helmholtz** (res/événement) | **1.85** | ~2 (optimum Engelman) |
| `res_kpa` amplitude ram délivrée | **−2.4 … +4.8 kPa** | ±25-40 kPa |
| `irp_kpa` pression au port d'admission | **95.4 … 105.8** | ±25-40 kPa d'excursion |
| `irp_kpa` à l'IVC | **101.3 kPa (= ambiante exactement)** | 110-135 kPa au régime accordé |
| forçage `(plénum − runner)` | **−4.0 … +6.4 kPa** | — |

**L'accord est bon** (nombre de Helmholtz 1.85, en plein dans l'optimum) : ce
n'est donc pas le modèle de fréquence qui est en cause. Ce qui manque, c'est
l'amplitude, et elle est bornée par le **signal de forçage** : le modèle était
forcé par `(plénum − runner)`, qui ne vaut lui-même que ±5 kPa. Raison
topologique et non paramétrique : un runner 0-D est relié au plénum par un
orifice de **pleine section** (`0.78 * runnerAreaM2`), donc il s'égalise avec lui
presque instantanément — d'où un port à exactement l'ambiante à l'IVC, c'est-à-dire
zéro ram, quelle que soit la valeur de `coupling_gain`.

**Piège majeur, et c'est celui qui coûte le plus de temps ici :** monter
`coupling_gain` *a l'air* de calibrer quelque chose. Sur CP4 le pic de VE passe
bien de 0.843 (gain 0.45) à 0.887 (gain 1.2), et à gain 2.0 / amortissement 0.10
on obtient même une VE de 1.04 avec un pic qui se déplace correctement avec la
longueur de runner. **Ce n'est pas une calibration, c'est l'amplification d'un
signal que la topologie a déjà aplati** : la grandeur amplifiée n'a pas la
dépendance physique en régime (le ram croît en rpm², pas linéairement), et le
même gain appliqué au LS3 gonfle son couple à 2000 tr/min (596 → 618 Nm) là où il
est **déjà au-dessus** du réel, en laissant son pic de couple à 2000 tr/min au
lieu de 4600. On déplace le niveau, jamais la forme de la courbe.

### Le forçage inertiel via le résonateur : implémenté, mesuré PIRE, annulé

Le forçage qui *paraît* correct se dérive de l'équation de quantité de mouvement
1-D de la colonne, sans gain libre :

    rho * l_eff * du/dt = P_plénum − P_port    =>    dP_port = − rho * l_eff * du/dt

Sur le papier tout est bon : bon zéro (débit permanent → `du/dt = 0` → pas de
ram), bon signe (décélération à l'IVC → ram positif), bon ordre de grandeur
(`rho*c*u = 1.2*343*65 = 27 kPa`, ~6× l'ancien forçage, magnitude des relevés de
port réels), bonne dépendance en régime (`u ∝ rpm`, temps ∝ `1/rpm`, donc
`du/dt ∝ rpm²`), et la longueur de runner devient un levier deux fois (dans
`l_eff` du forçage *et* dans la fréquence propre). Implémenté avec `u` =
`intakeRunnerGas_.bulkVelocityMps()`, `coupling_gain` 1.0, admittance retirée.

**Mesuré : c'est pire.** CP4, échelle de longueur de runner, balayage complet :

| variante | VE @2000 | pic VE | VE @9000 | VE @11000 | ch pic |
|---|---|---|---|---|---|
| **référence (forçage `plénum−runner`, gain 0.45)** | 0.819 | 0.843 @5000-6000 | **0.790** | **0.725** | **80.7** |
| forçage inertiel, runner 95 mm | 0.821 | 0.846 @4000-5000 | 0.754 | 0.697 | 75.4 |
| forçage inertiel, runner 190 mm | 0.825 | 0.852 @4000 | 0.759 | 0.694 | 75.3 |
| forçage inertiel, runner 48 mm | 0.816 | 0.836 @5000 | 0.767 | 0.697 | 77.1 |
| `runner_acoustics.enabled: false` | 0.821 | 0.839 @4000 | 0.767 | 0.704 | 76.5 |

Trois enseignements, tous contre l'hypothèse :

1. **Le pic de VE ne se déplace pas** avec la longueur (95 → 190 mm : pic
   4000-5000 → 4000). Test de non-vacuité échoué, le levier d'accord reste inerte.
2. **Le forçage inertiel est net négatif** : 0.754 (ram actif) contre 0.767 (ram
   coupé) à 9000 tr/min.
3. **L'admittance n'était pas un double comptage.** La retirer coûte 0.790 → 0.767
   de VE à 9000 (les deux avec l'amplitude routée à la soupape). Elle *ressemble*
   à un substitut redondant du ram et n'en est pas un — cas d'école de la règle
   « demande ce qui compense déjà en aval » de `CLAUDE.md`.

**Pourquoi ça échoue**, lu dans la trace (`res_kpa` contre `lift_mm`) — le signal
est cohérent, pas bruité (4 changements de signe sur le cycle, sinusoïde propre),
donc l'échec n'est pas numérique. Deux causes de fond :

- **Phase.** Le résonateur a une période de 7.2 ms (138.8 Hz) alors que
  l'événement d'admission dure 3.3 ms à 9000 tr/min. Un terme **instantané** passé
  dans un passe-bas d'ordre 2 en ressort retardé d'un angle énorme, et ici
  carrément inversé : `res_kpa` vaut **−3.40 kPa à pleine levée**, où il s'oppose
  au remplissage, et **+0.02 kPa à l'IVC**, où le ram devrait culminer.
- **Signal.** `intakeRunnerGas_.bulkVelocityMps()` est une **moyenne de maille**
  qui ne s'effondre pas quand la soupape se ferme (40.9 → 40.3 m/s à l'IVC). Or
  c'est précisément cet arrêt de la colonne qui *crée* le ram inertiel. L'événement
  physique est absent de `du/dt`, donc aucune reformulation du forçage à partir de
  ce signal ne produira un ram correctement phasé.

**Conséquence pour la suite.** Le ram correctement phasé demande soit une vitesse
au **plan de soupape** (dérivée du débit massique de la soupape, qui lui s'annule
bien à l'IVC), soit une vraie inertance de runner (maille double / `L·dṁ/dt`) —
pas un résonateur side-car sur une moyenne de maille. Le point 3 du plan reste
donc le seul chemin viable, et les points 1-2 ne suffisent pas seuls. Annulé :
forçage, `coupling_gain` (revenu à 0.45), suppression d'admittance. **Conservé** :
la capacité de biais dans le solveur (`FlowParameters::biasKpa0/1` + passage dans
`pressureEquilibriumMoles`), no-op documenté par défaut, et qui est ce qui rend
VE > 1 atteignable — vérifié : à `coupling_gain` 2.0 la VE du CP4 culmine à 1.04,
et avec un runner de 190 mm le pic se déplace à 6000 tr/min pour 1.095. Le
mécanisme d'application est bon ; c'est la **source** du signal qui manque.

### Le routage à la soupape casse le ralenti (mesuré, retiré)

Câbler l'amplitude du résonateur sur le biais de la soupape d'admission — même à
`coupling_gain` 0.45, même sans toucher au forçage — fait **échouer
`EngineLab.IdleStabilityRegression`** sur le **Radial R5** : 417 tr/min de minimum
pour une cible de 640, écart-type 52.3 (les autres moteurs : 2 à 18). Isolé
proprement : biais remis à 0.0 aux deux sites d'appel, tout le reste inchangé →
**la porte repasse**. Le routage est donc bien la cause, pas une coïncidence.

> **Correction (mesurée après coup).** Cette section affirmait d'abord que le
> forçage `(plénum − runner)` « atteint son maximum au ralenti » et valait « 11 %
> de perturbation par cycle contre 35 kPa de MAP ». **C'est faux.** Mesuré au vrai
> ralenti avec `--trace 1 --idle` (voir plus bas pourquoi `--throttle` ne suffit
> pas), `res_kpa` vaut **0.16 kPa** sur le R5 et 0.19 kPa sur le CP4, et il
> *croît* légèrement avec le régime (1.09 kPa au CP4 à 9000). Le chiffre de 11 %
> était une estimation, pas une mesure. Le **fait** que le routage cassait la porte
> reste établi (le remettre à zéro la réparait) ; le **mécanisme** de cet échec,
> lui, n'est pas établi. Ne pas réutiliser cette explication comme si elle l'était.

**Critère pour la prochaine tentative :** tout forçage candidat doit être vérifié
**petit au ralenti et grand au régime accordé** *avant* d'être câblé — c'est un
test à deux points, pas un seul. Et surtout : ne pas câbler puis aller chasser
l'échec de ralenti en aval dans le régulateur de ralenti. Ce serait précisément le
motif « corriger un symptôme en aval de sa cause » que ce document documente
depuis le début.

État livré : **17/17 portes vertes**, routage retiré, capacité solveur conservée.

### L'inertance de runner : deux formulations de plus, mesurées, réfutées

Suite directe de la section précédente, avec le test à deux points enfin fait pour
de vrai. Trois sources candidates pour le biais de remplissage ont maintenant été
mesurées ; **les trois sont fausses**, chacune pour une raison différente et
instructive. À lire avant d'en proposer une quatrième.

#### D'abord : un instrument manquait, et son absence invalidait le test

Le test « petit au ralenti » que j'avais posé comme critère **ne pouvait pas être
fait** avec les instruments existants. `--trace 1300` n'est pas un ralenti : c'est
du **plein gaz en sous-régime**. Le contrôleur de charge du banc commande la charge
qui tient le régime cible, l'ECU répond à cette charge en rouvrant le papillon via
`effectiveThrottle`, et la MAP se retrouve **à 1 kPa de la valeur pleine charge**
(mesuré : 98.3 kPa au « ralenti » CP4 contre 99.1 à 9000 tr/min). Ajouter
`--throttle 0.06` ne change rien, pour la même raison.

D'où **`--trace 1 --idle`** : papillon fermé, charge nulle, démarreur jusqu'à
l'accrochage — exactement la phase 2 de `EngineLab.IdleStabilityRegression`. Il
produit enfin de vraies dépressions (CP4 33.0 kPa, LS3 17.3 kPa). Toute mesure de
ralenti faite autrement est à jeter.

Note au passage : le **Radial R5 idle à 95.6 kPa de MAP**, quasiment l'ambiante,
là où tous les autres moteurs du catalogue idlent sous vraie dépression. Son
ralenti n'est donc pas stabilisé par la dépression collecteur, ce qui explique
qu'il soit le seul moteur sensible aux termes d'admission — mais **pas** qu'il
échoue : sa ligne de base est saine (voir plus bas).

#### Formulation 2 : la réaction inertielle −ρ·L·du/dt, appliquée en biais

C'est le terme du manuel, dérivé de l'équation de quantité de mouvement de la
colonne, sans gain libre. Pris **au plan de soupape** (`u = ṁ_valve/(ρ·A_runner)`)
il corrige le défaut de la tentative précédente : cette vitesse-là **s'effondre
bien** à l'IVC — 82.5 → −2.8 m/s, soit 3 % du pic, contre 40.9 → 40.3 pour la
vitesse de maille. Et le test à deux points passe largement :

| CP4 | ram fin d'événement (pondéré levée) | vitesse colonne, pic |
| --- | --- | --- |
| vrai ralenti, 1288 tr/min, MAP 33 kPa | **+0.04 kPa** (0.12 % de la MAP) | 45.4 m/s |
| 9000 tr/min pleine charge | **+6.43 kPa** | 82.5 m/s |

Rapport 160×, au-delà de la loi `rpm²` prédite parce que la densité du runner
chute aussi. LS3 au vrai ralenti : +0.03 kPa. Le signal a donc exactement la forme
qu'un ram doit avoir.

**Et pourtant c'est instable par construction.** Un orifice fixe déjà le débit à
partir de ΔP ; ajouter un ΔP dérivé de la dérivée *de ce même débit*
**sur-détermine** le système. Le gain de la boucle vaut ~`ρL/dt` et **croît quand
le sous-pas diminue** — ce n'est donc pas réglable, c'est structurel. Prédit puis
mesuré :

```
first 26 ram (CP4 9000):  35.0 -35.0 35.0 -35.0 35.0 -35.0 ...
saturé à la borne 34.2 % des échantillons, 38 % de changements de signe
vitesse colonne 82 -> 140 m/s
```

`EngineLab.IdleStabilityRegression` : **onze moteurs calent purement et
simplement** (0 tr/min), pas un ralenti qui chasse. C'est la signature d'une
divergence numérique, pas d'un terme trop fort.

**Leçon générale, qui dépasse ce terme :** dans ce solveur, un biais de pression ne
peut jamais être une dérivée de la grandeur qu'il pilote. Seul un **état** est
admissible.

#### Formulation 3 : la tête de stagnation ρu²/2 (un état, donc stable)

Une colonne arrivant à `u` peut charger le cylindre jusqu'à sa pression d'arrêt,
pas seulement jusqu'à la pression statique du port. C'est un **état**, donc pas de
gain en `1/dt` — et effectivement **aucune oscillation** : 0 % de saturation, 0
changement de signe, pic 4.00 kPa au CP4 à 9000 (conforme au calcul à la main),
0.29 kPa au ralenti R5.

Elle échoue sur la **phase**, et le diagnostic est net :

| Radial R5, porte de ralenti | moyenne | min | écart-type | verdict |
| --- | --- | --- | --- | --- |
| ligne de base (biais isolé à 0) | 637 | 546 | **27.7** | passe |
| avec la tête de stagnation | 567 | 431 | **52.6** | échoue |

Le **signe** de l'erreur est le tell : la moyenne **baisse de 70 tr/min**. Un terme
de remplissage ne peut qu'ajouter du couple. Il en perd, donc le mécanisme est
faux, pas son dosage. La cause : `u` suit l'aire de soupape, donc la tête culmine
**à pleine levée** et ne vaut plus que 3 % du pic à l'assise. Un sur-remplissage de
milieu d'événement est **réversible** — quand la tête retombe, la borne d'équilibre
redevient statique et le solveur rechasse l'excédent par la soupape encore
ouverte, avec les pertes du passage. Bilan net négatif.

Et cela réfute au passage l'argument « le R5 est de toute façon marginal » :
**mesuré, sa ligne de base est saine** (écart-type 27.7, la porte passe
confortablement). La régression est bien la mienne. Les 12 autres moteurs passaient
avec cette formulation — n'en conclure ni que le terme est presque bon, ni que le
R5 est fautif.

#### Ce qui reste, et pourquoi c'est la seule voie identifiée

La formulation 3 démontre que **la physique manquante n'est pas une pression du
tout**. Le ram réel est irréversible parce que la colonne **arrive encore** au
moment où la soupape se ferme : c'est la fermeture qui **piège** la surpression.
Il faut donc que le **débit retarde** sur sa valeur quasi-statique, c'est-à-dire un
état de quantité de mouvement porté par la colonne :

```
ṁ ← ṁ + (ṁ_quasi-statique − ṁ)·(1 − exp(−dt/τ)),   τ = L_eff/u  (temps de transit)
```

Propriétés, toutes vérifiables avant câblage :

- **Inconditionnellement stable** : le facteur de mélange est borné dans [0, 1],
  contrairement au gain `ρL/dt` de la formulation 2.
- **τ est dérivé, pas ajusté** : la linéarisation de `ρ(L/A)·dQ/dt = ΔP − K·Q|Q|`
  donne `τ = ρ·L·ṁ/(2·A·ΔP)`, fini même au repos.
- **La bonne échelle en régime** : estimé `τ/t_event` ≈ 0.4 % au ralenti CP4 contre
  ≈ 9 % à 9000 tr/min.
- **La bonne phase** : `u` reste élevée quand la soupape se ferme, donc la charge
  arrive encore à l'IVC — c'est exactement ce qui manque à la formulation 3.

Ce n'est plus un terme ajouté : cela **remplace** une partie de la loi de débit, et
demande de traiter l'inversion de sens à partir du signe de l'état (une colonne
lancée continue contre un gradient adverse). C'est donc un changement de topologie
du solveur gaz, avec un **risque réel sur le voicing audio** — la pression cylindre
est la seule excitation de la chaîne d'échappement — et sur la fenêtre MBT.

**Non implémenté.** L'arbre est rendu vert avec les instruments et les mesures, et
ce choix appartient au produit.

État livré : **17/17 portes vertes** (352.7 s), aucun câblage, capacité solveur
conservée, trois formulations réfutées et documentées. Gain net d'instruments :
`--idle`, `--throttle`, et les colonnes `ram_kpa` / `col_mps` de la trace.

## Le diagnostic architectural : l'admission n'a pas de dimension (topologie, vérifiée)

Après quatre formulations mesurées et réfutées, la question a changé : ce n'est
plus « quel terme manque ? » mais « pourquoi chaque terme candidat échoue-t-il
d'une façon différente ? ». La réponse est une asymétrie de **topologie**, lisible
dans le code sans ambiguïté :

- **Échappement** : un vrai solveur gaz-dynamique 1-D
  (`src/gas-dynamics/FiniteVolumeDuct.hpp`) — volumes finis second ordre,
  reconstruction TVD monotonisée, flux de Riemann HLLC, SSP-RK2, sous-pas CFL
  avec rejet des états non physiques, 4 espèces, γ variable spatialement —
  assemblé en réseau globalement couplé (`ExhaustGasNetwork.hpp`), où la soupape
  est une **condition limite de Riemann** (`CylinderValveBoundary`).
- **Admission** : `intakePlenumGas_` = **une** cellule 0-D ; `intakeRunnerGas_` =
  **une** cellule 0-D par cylindre (`EngineSimulator.hpp:155-157`) ; puis un
  orifice quasi-statique avec clamp d'équilibre par sous-pas.
- Le détail qui achève le diagnostic : `AcousticIntakeNetwork` existe — **dans le
  module audio**. L'admission est modélisée comme réseau d'ondes pour produire du
  *son*, jamais pour faire passer de l'*air*.

L'accord d'admission est un phénomène d'**ondes**. Une cellule unique n'a aucun
délai de propagation : rien ne peut arriver « à la bonne phase » parce que rien
ne voyage. Les quatre formulations tentaient de contrefaire une onde par une
pression algébrique, et c'est pour cela qu'elles ont échoué de quatre façons
différentes (instabilité, phase, réversibilité) : il ne manque pas un
coefficient, il manque une **dimension**. La seule dépendance au régime d'un
orifice quasi-statique est « moins de temps pour remplir à travers une
restriction fixe », donc la VE ne peut que décroître de façon monotone — ce qui
prédit exactement la signature banc : couple 74-99 % (un modèle localisé remplit
bien à bas régime), pic 1500-4000 tr/min trop tôt (aucune bosse de résonance à
placer), puissance 48-66 % (la bosse est précisément ce qui fait le haut du
régime). Trois symptômes, un seul terme absent.

### L'instrument écrit AVANT le fix : `EngineLab.IntakeTuning` (mesuré)

`tests/IntakeTuningTests.cpp` balaye l'EL-20 I4 à pleins gaz sous absorbeur de
banc (le contrôleur exact du DynoSweepHarness) et évalue quatre critères ancrés
littérature — jamais sur la sortie du simulateur — figés avant tout changement
du solveur : pic de VE dans [0.85, 1.15] (Heywood) ; pic à ≥ 40 % du rupteur ;
VE à ~85 % du rupteur ≥ 0.80 × pic ; et le critère décisif, **doubler la
longueur de runner doit déplacer le pic de VE vers le bas d'au moins 15 %**
(Helmholtz/quart d'onde : la vitesse accordée varie en 1/√L à 1/L, soit
−29 à −50 % physiques ; 15 % ne présuppose aucun modèle d'accord précis).

Mesuré le 2026-07-25 (23 s, invariants verts, MAP ~100-101 kPa donc le WOT est
réel) :

| Critère | Mesure | Verdict |
|---|---|---|
| 1. pic VE ∈ [0.85, 1.15] | 0.886 | OK — le remplissage bas régime est sain |
| 2. pic à ≥ 2880 tr/min | **2000 (plancher du balayage)** | échec — la courbe ne sait que décroître |
| 3. VE(6000) ≥ 0.709 | 0.769 | OK sur ce moteur modeste |
| 4. décalage du pic à 2×L ≥ 15 % | **0.000 %** | échec — la longueur n'existe pas pour le remplissage |

Le critère 4 est la preuve directe : la courbe 2×L est légèrement *plus haute*
partout à bas régime (0.897 contre 0.886 à 2000) — pur effet de volume tampon de
la cellule agrandie — et son pic ne bouge pas d'un tour/minute. Les critères
sont **rapportés** par défaut et gatés sous `--enforce-tuning` (vérifié
non-vacuux : exit 1 aujourd'hui, sur le critère 2). Quand l'admission 1-D
arrivera, l'enregistrement ctest gagne le drapeau et les critères deviennent des
portes dures. **Ne pas affaiblir un critère pour faire passer cette promotion** :
ils encodent le comportement d'un moteur réel, pas un objectif que le simulateur
ait jamais atteint.

### Le plan, gaté par étapes

- **A (fait)** : l'instrument ci-dessus, écrit et mesuré avant le changement.
- **B** : spike CPU (`tools/IntakeDuctBench.cpp`) — un `FiniteVolumeDuct` en
  conditions d'admission (froid, 0.20-0.30 m, excitation à cadence de soupape),
  chiffré contre le budget 240 Hz **sur machine au repos** ; le thread physique
  est déjà à ~99 % du budget sur le V8 à 6500, donc le verdict porte sur le
  budget restant et sur la nécessité du parallélisme par cylindre. Go/no-go.
- **C** : `IntakeGasNetwork` réutilisant la machinerie existante (un conduit 1-D
  par runner, plénum en jonction, papillon en `localLossCoefficient`, frontière
  réservoir déjà corrigée, `CylinderValveBoundary` tel quel), remplaçant les
  cellules localisées et le clamp statique. **Changement de voicing déclaré** —
  la pression cylindre est la seule excitation de la chaîne d'échappement.
- **D** : recalibration du catalogue et des 17 portes, re-banc CP2/CP3/CP4
  contre constructeur, gains indépendants (chauffage de charge 55 K → BDC,
  ~15 % de densité) repris au passage.

### Stage B mesuré : le spike CPU dit GO, avec le parallélisme par cylindre pour les gros moteurs

`EngineLabIntakeDuctBench`, machine au repos, trois runs à ±2 % (budget d'une
frame 240 Hz = 4.167 ms ; excitation volontairement dure — tirage de 23 kPa
soutenu 2 frames sur 5, donc ces chiffres ne sont pas flattés) :

| cellules | L (m) | ms/runner | sous-pas | ×4 runners | ×8 | ×12 |
|---|---|---|---|---|---|---|
| 8 | 0.20 | 0.42 | 133 | 41 % | 81 % | 122 % |
| 8 | 0.30 | 0.29 | 90 | 27 % | 55 % | 82 % |
| 12 | 0.20 | 0.97 | 200 | 93 % | 186 % | 279 % |
| 12 | 0.30 | 0.66 | 134 | 64 % | 128 % | 191 % |

Lecture :

- **8 cellules par runner est le bon défaut.** La résonance qui fait la VE est
  le fondamental quart-d'onde (λ ≈ 4L), donc 8 cellules ≈ 32 cellules par
  longueur d'onde — largement au-dessus des 10-20 requis au second ordre. Passer
  à 12 cellules coûte 2.3× pour ne raffiner que des harmoniques secondaires.
- **GO série pour les moteurs cibles** (CP2/CP3/CP4, 2-4 cylindres) :
  0.6-1.7 ms ajoutées, alors que le cas liant du budget est le V8 à 6500
  (~99 %) — les petits moteurs ont la marge.
- **Les V8/V12 exigent le parallélisme par cylindre**, qui existe déjà : la
  section Jacobi `decoupleSharedVolumes` (`EngineSimulator.cpp:1608-1622`)
  avance chaque cylindre contre un plénum gelé dans un scratch de worker et
  commet ensuite des flux conservatifs à N voies. Un conduit de runner par
  cylindre entre exactement dans ce motif ; sur 8 cœurs, 8 runners en parallèle
  ≈ le coût d'un seul + le commit, soit ~8-10 % de budget pour le V8.
- Deux biais de mesure, en sens opposés, non chiffrés : l'excitation du banc est
  plus dure que les dépressions réelles de port (moins de sous-pas en vrai), et
  le réseau réel ajoute jonction plénum + papillon + frontières de Riemann aux
  soupapes (plus de coût que N conduits indépendants). Décisionnel quand même :
  l'ordre de grandeur est net et la voie parallèle est éprouvée.

## Stage C : l'admission 1-D est câblée, la porte d'accord passe (mesuré)

`EngineSimulator` remplace la cellule-runner 0-D et l'orifice de soupape
quasi-statique par **un réseau `ExhaustGasNetwork` mono-conduit par cylindre**
(assemblé programmatiquement via `ExhaustNetworkLayout::assemble`, nouveau) :
soupape = port cylindre de Riemann à l'entrée du conduit, embouchure = « outlet »
dont le réservoir d'ambiance est le plénum du chemin, avancé en deux
demi-sous-pas symétriques autour du couplage échappement, exactement comme
l'orifice qu'il remplace. Le plénum reste 0-D (une compliance est un élément
localisé — c'est le runner qui est le tuyau d'orgue). L'injection port dépose la
vapeur et son refroidissement dans les 3 cellules côté soupape
(`injectSpeciesAtPort`, tout-ou-rien) ; le modèle de film est inchangé. Le
Helmholtz ne module plus aucun débit : télémétrie seulement.

### Deux défauts de frontière, trouvés par un test stationnaire, pas par relecture

Le premier essai livrait des courbes de VE PIRES qu'avant (0.886→0.829 au
plancher, courbes base/2×L identiques au-dessus de 4000 : un étranglement
indépendant de la longueur). L'instrument décisif : un tirage **stationnaire**
à travers une soupape ouverte, comparé à la tuyère isentropique du même ΔP
(`testSteadyDrawMatchesIsentropicValveFlow`).

1. **L'aspiration à une extrémité ouverte était comptée à l'impédance
   acoustique.** La branche backflow de `openEndBoundaryPrimitive` renvoyait le
   réservoir *au repos* comme fantôme ; le flux de Riemann contre une cellule
   statique impose u ≈ ΔP/(ρc). Mesuré : le conduit devait s'affaisser de
   10.5 kPa sous un réservoir à 101.3 pour tirer 26 m/s là où Bernoulli demande
   0.4 kPa — déficit stationnaire de 13 %. C'est le pendant côté aspiration du
   défaut « une extrémité ouverte est un réservoir » déjà corrigé côté
   refoulement ; l'échappement ne le voyait pas (réversion brève), l'admission
   y vit en permanence.
2. **Le remplacement « évident » était aussi faux.** Utiliser la tuyère
   compacte (`compressibleValveFlux`) à pleine section impose la contrainte
   d'un jet libre (P_exit + ρu²_exit, u_exit dimensionné par tout le ΔP) sur
   une face dont l'intérieur ne porte qu'une fraction de cette vitesse :
   mesuré, l'intérieur se faisait pomper à 111 kPa AU-DESSUS d'un réservoir à
   101.3 (ratio de débit 1.064). Une entrée de conduit n'est pas une ouverture
   compacte.

La forme correcte garde la structure fantôme→Riemann de la sortie : **le
fantôme d'aspiration est le gaz du réservoir accéléré isentropiquement jusqu'à
la pression statique intérieure** (bornée au rapport critique). La pression est
la seule grandeur intérieure continue à travers le contact, donc la lire ne
répète pas la catastrophe « invariant à travers le contact » (2 km/s, onze
moteurs calés) documentée plus haut ; le saut d'entropie/composition reste dans
le solveur de Riemann. Mesuré : ratio 0.963 (le manque est l'entrée + friction,
physique), intérieur à 0.3 kPa sous le réservoir = Bernoulli exact. Le test
verrouille [0.90, 1.05] avec les deux modes d'échec en dehors.

### Résultat : les quatre critères littérature passent, la porte est promue

| Critère (gelé avant le changement) | Avant (0-D) | Après (1-D) |
|---|---|---|
| pic VE ∈ [0.85, 1.15] | 0.886 | **1.012** — suralimentation par résonance réelle |
| pic à ≥ 40 % du rupteur | plancher (2000) | **5142 tr/min = 71 %** |
| VE(85 % rupteur) ≥ 0.80×pic | OK (courbe plate) | OK (0.923) |
| décalage du pic à 2×L ≥ 15 % | 0.000 % | **23.7 %**, pic 2×L à 4000 avec VE 1.113 |

La courbe a la physionomie d'une vraie courbe d'accord : creux
d'anti-résonance à ~2450, bosse au régime accordé, chute au-delà. La longueur
de runner est enfin un paramètre de conception qui agit. `EngineLab.IntakeTuning`
est enregistrée avec `--enforce-tuning` : portes dures désormais.

### Ouvert (étage D) : trois ralentis de gros cylindres

CP2/CP3/CP4/Hayabusa/LS3 tournent au ralenti (σ 9-20). Échouent : Big Twin
(chasse), R5 (chasse, recentré après l'étalement de l'injection sur 3
cellules), et le **Merlin V12 qui cale — mesuré au `--watch` (nouveau) : ce
n'est pas une panne pauvre mais une noyade verrouillée**. Séquence : le moteur
attrape (AFR 9.3, 1000 tr/min), le papillon se ferme, la charge chute 4×
(1700→470 mg), l'inventaire de carburant du démarrage reste debout près de la
soupape, le cycle suivant l'avale entier (AFR ~3), misfire, et la réversion à
l'IVO recycle l'imbrûlé vers le port — piège circulant que la rétention de 45 %
de la charge entretient. Réfuté par la mesure : ni le refroidissement de charge
(sonde à zéro : noyade identique), ni le film mural (le catalogue applique déjà
0.22 ; identique). Le runner 0-D survivait au même excédent parce que sa
cellule unique était un tampon de dilution numérique. C'est un nœud
calibration-dosage (l'enrichissement était réglé sur un transport retardé) —
étage D, avec `--watch` comme instrument.

### Le déverrouillage du ralenti : une asymétrie de comptabilité, pas un défaut du réseau

Trois sondes ont réduit le nœud (chacune mesurée, deux réfutées) :
le refroidissement de charge mis à zéro → noyade identique ; le film mural
(0.22 via le chargeur catalogue) → identique ; puis la trace `--watch` a montré
`fuel req/del = 106/201 mg` pour une charge réelle de ~500 : **la requête
elle-même doublait**.

La cause : `TransientChargeEstimator` prend `max(O2 résolu du cylindre,
prédiction vitesse-densité)`. Ce plancher suppose que la charge précédente a
brûlé — moteur allumé, l'O2 résiduel ≈ 0 et le plancher ne mord jamais ; après
un raté, le cylindre garde son air imbrûlé ET son carburant imbrûlé, mais le
dosage ne comptait le carburant que soupape ouverte. L'air noyé comptait
toujours, le carburant noyé presque jamais : sur-requête → AFR 3 → raté →
rétention (45 % de la charge) → verrou.

Deux corrections couplées, changées ENSEMBLE comme l'exige la règle des
compensations appariées :

1. Le plancher est conservé — il est aussi un enrichissement anti-calage réel :
   sa suppression seule a fait plonger les ralentis des deux turbos (2JZ min
   567, Audi min 596), mesuré avant d'être annulé.
2. `trappedCylinderFuel` compte désormais le carburant du cylindre aussi
   **quand le cycle précédent a raté** (`cylinderMisfires_`), symétriquement à
   l'air que le plancher compte.

Résultat mesuré (porte de ralenti) : Merlin **σ = 4.0 à 796/800** (il calait à
0), 2JZ σ 12.1, Audi σ 9.9, Big Twin σ 27.6 — tous verts. Le gate d'injection
sur rotation réelle (`rpm > 20`) a aussi été posé : une phase gelée dans la
fenêtre d'injection modélisait un injecteur coincé ouvert sur moteur arrêté.

### Le test zéro-levée mesurait un artefact du 0-D

`EngineLab.Core` exigeait la MAP instantanée à ±10 Pa de l'ambiante soupapes
fermées. Deux effets **physiques** du runner résolu la déplacent : (1) la
vapeur de carburant injectée pendant le lancement s'équilibre par l'embouchure
ouverte — ~10 mg dans ~3 L = ~67 Pa de pression partielle réelle (la cellule
0-D la piégeait, voilà pourquoi 10 Pa tenaient) ; (2) le mode de Helmholtz
runner-plénum sonne quasi non amorti au repos (±140 Pa sur la MAP) — un
échantillon instantané lit la phase d'une onde, pas un inventaire. Le test
assert désormais la **moyenne temporelle à ±0.25 kPa** : l'intention (pas de
drainage du collecteur, un défaut à l'échelle du kPa) est conservée, l'acoustique
résolue ne déclenche plus.

## Le vrai défaut du ralenti : l'anti-windup regardait le mauvais signal (mesuré)

Ce qui suit annule et remplace la section R5 ci-dessous, deux « correctifs » que
j'avais posés, et l'idée que le ralenti du catalogue était sain avant.

### La porte de ralenti mesurait la phase d'une oscillation, pas une stabilité

`EngineLab.IdleStabilityRegression` moyenne une fenêtre de 4 s (t = 10-14 s).
Onze moteurs y « passaient ». La trace montre qu'ils **oscillaient tous** à
~0.2 Hz avec 130-180 tr/min crête-à-crête, faiblement amorties, encore vivantes
à t = 14 s. Passer ou échouer dépendait de la phase que la fenêtre attrapait.

C'est ce qui m'a fait tourner en rond : j'ai attribué à mes propres
modifications une régression 2JZ/Audi/Big Twin qui n'était qu'un **déplacement
de phase** d'un défaut déjà présent. Preuve : désactiver l'un OU l'autre de mes
deux changements faisait échouer le 2JZ avec des chiffres quasi identiques
(682/559 σ52.8 et 688/560 σ45.8) alors que désactiver les deux le faisait
passer — signature d'un attracteur fragile, pas de deux causes.

Le simulateur est déterministe (deux exécutions séquentielles identiques au
bit ; l'exécution concurrente aussi) : la confusion ne venait pas du bruit.

### La cause : l'intégrale se dévidait pendant que le plancher tenait la vanne

L'ouverture réellement livrée vaut
`max(postStartAir, max(clamp(PI,0,1), dashpot) * driverOverride)`.
La garde d'anti-windup, elle, testait **la commande propre du gouverneur**
contre 0 et 1. Or cette commande est en plein milieu de sa plage pendant que le
plancher post-démarrage possède l'actionneur : la garde ne voyait donc jamais
la saturation.

Conséquence, mesurée sur le 2JZ (colonnes `iac`/`postSt`/`integ` ajoutées à
`--trace`) : de t = 5.0 s à t = 9.25 s, **`iac` égale `postSt` à trois
décimales** — le gouverneur ne bouge rien — pendant que le flare tient le
régime au-dessus de la cible, que l'erreur reste négative et que l'intégrale
descend jusqu'à sa butée `-0.20`, où elle reste collée deux secondes. Le
plancher passe ensuite sous la commande et rend la main à un régulateur sans
autorité : le moteur s'affaisse de 847 à 571 tr/min pendant que l'intégrale
remonte à 0.11/s, dépasse, et sonne à 0.2 Hz jusqu'à la fin.

Ce n'était pas qu'un transitoire : la butée imposait un **biais permanent**.
Presque tout le catalogue tournait *sous* sa cible — LS3 665 pour 720,
Aircooled 706 pour 780, R5 685 pour 800.

### Le correctif : tester la position livrée, pas la commande

Règle d'anti-windup classique, appliquée au vrai actionneur : quand autre chose
que le gouverneur possède la vanne, celui-ci est saturé BAS et ne peut intégrer
que dans le sens qui le ramène aux commandes (`normalizedError > 0`, moteur sous
la cible) ; sinon il **conserve** son autorité. Le passage de témoin devient
continu par construction.

Mesuré, porte de ralenti complète (cible / moyenne / min / σ) :

| moteur | avant | après |
|---|---|---|
| LS3 (720) | 665 / 595 / 30.6 | **721 / 714 / 3.3** |
| Aircooled (780) | 706 / 630 / 37.3 | **776 / 763 / 6.0** |
| 2JZ (760) | 728 / 709 / 12.1 | **755 / 748 / 3.0** |
| Audi (780) | 760 / 737 / 9.9 | 778 / 744 / 9.4 |
| R5 (640) | échec | **639 / 553 / 27.2** |
| K20A (950) | 944 / 931 / 5.1 | 954 / 946 / 3.9 |

13/13 moteurs au vert, chacun **à** sa cible et non plus dessous.

### Deux de mes propres correctifs, réfutés par cette mesure

- **Le clamp d'avance au démarrage (`rpm < 500 → avance ≤ 4°`) : retiré.** Le
  vrai remède du coup de recul du R5 était le dimensionnement du démarreur ; le
  clamp mesuré seul coûtait au 2JZ 728/709 → 682/559. Un seuil en régime nu est
  *dans* l'enveloppe normale d'un moteur qui ralentit bas : il mordait sur un
  creux de ralenti ordinaire et coupait le couple juste quand il fallait le
  rendre. Un vrai retard de démarrage est conditionné à l'état run/start.
- **Le `idle_rpm: 640 → 800` du R5 : annulé.** J'avais écrit que « le régime
  naturel papillon fermé était monté à 770-800 et qu'un gouverneur qui ne sait
  qu'ajouter de l'air ne peut pas réguler en dessous ». Faux : c'était la butée
  d'intégrale. Avec l'anti-windup corrigé le R5 tient **639 pour 640**. La
  config est revenue à l'identique.

Ce qui reste vrai et gardé : le démarreur dimensionné sur la cylindrée
**unitaire** (pic de compression), vérifié encore nécessaire — sans lui le R5
rechasse (σ 47.9).

### Le test « levée nulle » passait pour deux mauvaises raisons à la fois

Piège à signaler tel quel aux futurs agents. `EngineLab.Core` affirmait
« zero valve lift must result in zero volumetric efficiency ». Vert depuis
toujours. Il ne testait rien :

1. **Le moteur n'était pas à levée nulle.** Le test ne mettait à zéro que
   `config.camshafts` ; or `activeCamshaft()` préfère la came du **banc** dès
   que le cylindre appartient à un banc. Les bancs gardaient leur levée.
2. **L'assertion était satisfaite par un garde-fou, pas par la physique.** Le
   démarreur d'alors était trop faible pour entraîner ce moteur (mesuré :
   rpm 0.00), et `volumetricEfficiency` est forcé à 0 sous 20 tr/min. Zéro
   parce que rien ne tournait, pas parce que rien n'entrait.

Le démarreur redimensionné a fait tourner le vilebrequin (correct : un cylindre
scellé est un ressort à gaz qui restitue le travail), les bancs encore levés ont
respiré, et l'assertion est tombée — révélant les deux défauts d'un coup.
Corrigé : levée annulée sur la came globale **et** sur chaque banc, plus une
garde de non-vacuité `rpm > 20` pour que l'assertion ne puisse plus jamais
passer moteur à l'arrêt.

### Défaut réel mis au jour au passage : la VE compte l'oxygène *piégé*, pas l'air *admis*

`state_.volumetricEfficiency` se calcule sur `trappedAirMassMgLastCycle_`,
c'est-à-dire l'oxygène présent dans la chambre à l'IVC. Pour un moteur sain les
deux coïncident (les gaz résiduels sont brûlés, sans O2). Ils divergent dès que
la chambre garde de l'oxygène imbrûlé : raté d'allumage, cylindre entraîné, ou
came sans levée. Mesuré sur le moteur à levée nulle : **VE = 0.596 alors que
l'admission vaut exactement 0** — la charge initiale, jamais renouvelée, est
comptée comme respiration.

Non corrigé volontairement, et il faut savoir pourquoi avant d'y toucher :
- `trappedAirMassMgLastCycle_` est aussi la référence de dosage, et là
  l'oxygène piégé est la **bonne** grandeur (c'est ce qui peut brûler).
- `EngineLab.IntakeTuning`, la porte durcie de l'étage A, est calibrée sur
  cette VE. Le remplaçant naturel (`intakeFlowMgPerCycle_`) est une masse
  **totale**, carburant vaporisé compris : en injection indirecte cela décale
  la VE d'environ +6.7 % et déplacerait la porte.

Ce qui est fait : `EngineState::inductedChargeMassMgPerCycle` expose la masse
qui a réellement franchi les soupapes d'admission, ce qui rend le test
ci-dessus non vacuous sans toucher à la définition porteuse. Séparer proprement
« air frais admis » de « masse totale admise » est un chantier d'étage D.

### La porte durcie : dérive, fenêtre allongée, exemption supprimée

La porte teste maintenant aussi la **dérive** (moyenne de la seconde moitié de
la fenêtre moins celle de la première). Un écart-type seul ne distingue pas un
ralenti stabilisé d'un ralenti encore en train de balayer ; la dérive, elle, est
sensible à la phase. Elle a immédiatement pris un vrai défaut : douze moteurs
entre -7.5 et +10.6 tr/min, et le **Big Twin à -66.8** — encore en descente
pendant toute la fenêtre.

Cause mesurée : le schedule d'air post-démarrage décroît en exponentielle
(τ 2.5-6 s) et n'est coupé qu'en dessous de 1e-4, donc sa traîne dépasse 20 s.
À 8 s de ralenti libre la fenêtre « stabilisée » mesurait encore le transitoire
de démarrage des gros cylindres. Phase portée à **16 s** — durcissement net, pas
relâchement, puisqu'elle est appariée à l'assertion de dérive. Résultat : les
treize moteurs convergent **sur** leur cible à quelques tr/min près.

| moteur | cible | moyenne | σ | dérive |
|---|---|---|---|---|
| K20A | 950 | 950 | 3.4 | -0.1 |
| 2JZ | 760 | 760 | 2.7 | +0.2 |
| LS3 | 720 | 720 | 2.4 | -0.1 |
| Merlin V12 | 800 | 800 | 0.4 | -0.0 |
| Big Twin | 760 | 764 | 17.7 | -3.3 |
| R5 | 640 | 641 | 28.7 | -10.0 |
| CP2 / CP3 / CP4 | 1400 / 1300 / 1300 | idem | 16.4 / 14.8 / 20.7 | ≤ 2.9 |

Le Big Twin passe de σ 45.1 à 17.7 : il n'avait pas un défaut de régulation,
il n'avait pas fini de converger. Et l'exemption `knownBlipStallAllow` du
Merlin est **supprimée** — le V12 récupère désormais son ralenti après un coup
d'accélérateur sans caler, donc la garder ne masquait plus rien et empêchait
seulement de voir une régression future.

### ~~Reste ouvert : le ralenti du R5 radial~~ (PÉRIMÉ — résolu par l'anti-windup)

> Conservé pour la trace du raisonnement. La conclusion « la vanne de ralenti
> perd son autorité » était la bonne observation avec la mauvaise cause :
> l'autorité était perdue parce que l'intégrale était collée à sa butée `-0.20`,
> pas parce que la MAP traversait l'ambiante. Le R5 tient 639 pour 640 depuis la
> correction d'anti-windup, sans toucher ni à `idle_bypass_area_mm2` ni à
> `idle_rpm`. Voir la section ci-dessus.

AFR sain (~13.5), carburant stable — le mélange n'est plus en cause. Chasse
lente ~0.25 Hz, ±180 tr/min, avec la **MAP oscillant 86→103 kPa papillon
fermé** : la réversion des cinq gros cylindres atteint maintenant physiquement
le plénum (les radiaux réels rotent dans leur admission), la MAP traverse
l'ambiante et la vanne de ralenti perd son autorité (aucun ΔP). Le 0-D retenait
cette masse dans sa cellule. La config porte un `idle_bypass_area_mm2: 180`
(un trou de 15 mm — dimensionné pour faire respirer l'ancien modèle) : c'est la
recalibration catalogue de l'étage D, à mesurer à la porte de ralenti.

## L'absorbeur du banc ne tenait pas ses points, et balayait dans le rupteur (mesuré)

Ce qui suit **annule et remplace** tous les chiffres haut-régime que j'ai
publiés dans cette session avant cette section, dont deux « falaises de
combustion » que j'ai annoncées et qui n'existent pas. Les deux venaient du
même instrument.

### Le défaut 1 : le gain intégral ne pouvait pas se charger

Les quatre instruments WOT du dépôt (`DynoSweepHarness`, les deux points de
`PhysicsPerfHarness`, `IntakeTuningTests`) partagent le même contrôleur copié :

```
dynoIntegral = clamp(dynoIntegral + speedError * dt * 1.20, 0.0, 0.95);
load         = clamp(dynoIntegral + speedError * 0.70, 0.0, 1.0);
```

Avec `dt = 1/240 s` et une erreur stationnaire de 7 %, l'intégrale n'atteint que
~0.25 en deux secondes de stabilisation, soit ~119 Nm de frein — moins que le
couple d'un 2 L à 7000 tr/min. **L'absorbeur ne sature jamais : il ne se charge
jamais.** Aux bas régimes il converge (peu de charge suffit) ; au-delà de
~5000 tr/min le moteur dérive vers le haut jusqu'à ce que son propre couple
s'écroule ou que le rupteur l'attrape.

Mesuré sur le I4 par défaut, cible 6500 tr/min : le point rendu était à
**6958 tr/min**, bande [6914, 7001], rupteur à 7200. La garde de l'époque
tolérait 15 % d'écart, donc elle passait. Gain porté à 12.0 et plafond à 1.0 :
tous les points tiennent à moins de 0.5 %.

### Le défaut 2 : le sommet du balayage EST le rupteur

`maxRpm = min(redline, revLimit)` place la dernière cible **exactement sur** le
rupteur. Or le rupteur de `SimpleEcuModel` est verrouillé avec hystérésis (coupe
à `revLimit`, ne relâche qu'à `revLimit - 180`), et `EngineSimulator` fait
`flameEvents_[i] = {}` dès que l'étincelle manque — ce qui **met à zéro
l'efficacité de combustion publiée** à chaque cycle coupé.

Isolation décisive, même point à 7000 tr/min tenu à 0.04 % :

| rupteur | combEff | IMEP net | trap | VE livrée |
|---|---|---|---|---|
| 7200 (balayage sur le rupteur) | 0.167 | 4.81 bar | 1.165 | 0.826 |
| 9000 (rupteur écarté) | **0.911** | **13.02 bar** | 1.006 | 0.922 |

Un constructeur annonce sa puissance nominale **sous** le rupteur ; un balayage
WOT doit donc s'arrêter sous lui. Les deux instruments plafonnent maintenant à
`0.95 * min(redline, revLimit)`. Signature à reconnaître dans un ancien CSV : la
dernière ligne d'un moteur donne un couple absurde (le 2JZ y lisait 2.15 Nm et
1.56 kW).

### Les quatre hypothèses réfutées en route

Chacune paraissait solide et chacune est tombée sur une mesure directe. La
falaise à 6958/7080 tr/min mettait `combEff` à 0.337 puis 0.129 :

| hypothèse | prédiction | mesure | verdict |
|---|---|---|---|
| coupure d'allumage / ratés | `misfireRate` élevé | 0.000 et 0.007 | réfutée |
| dilution par les gaz résiduels | résiduel ≫ 7 % | 0.011-0.016 partout | réfutée |
| saturation d'injecteur | `injectorCapacityRatio` < 1 | 1.000 et 0.999 | réfutée |
| mélange faux à l'étincelle | φ très écarté de 1.08 | 1.136 vs 1.139 | réfutée |

Le résiduel mérite une note : j'ai d'abord conclu que `burnedGasFraction`
n'était **jamais** affecté, parce qu'un grep par nom ne trouvait rien hors des
tests. Faux — il est rempli **positionnellement** dans l'agrégat
`FlameConditions` (`burnedMoles / totalMoles`). Le terme de dilution est bien
vivant. Un grep par nom ne prouve rien sur un agrégat initialisé par position.

### Ce que le fractionnement de la VE a rendu visible

`volumetricEfficiency` est une efficacité de **piégeage** (air équivalent-oxygène
présent à la fermeture admission). La définition de Heywood est une efficacité
de **livraison**. `deliveredVolumetricEfficiency` publie maintenant la seconde,
sur la même base oxygène, donc leur rapport est un rendement de piégeage.

Le rapport vaut 1.002-1.008 sur toute la plage saine — les deux comptabilités,
qui n'empruntent pas le même chemin, se recoupent à moins de 1 %. Et il **passe
au-dessus de 1** exactement là où l'étincelle est coupée : l'oxygène imbrûlé
reste en chambre et gonfle le chiffre piégé, tandis que le chiffre livré, lui,
dit la vérité (0.973 contre 0.810 au point rupté). Le rapport trap > 1 est donc
un **détecteur de combustion incomplète**, et c'est la garde toujours active du
nouvel instrument.

### PMEP : le chiffre corrigé

L'excès de pompage reste réel mais il est **plus petit que ce que j'ai annoncé**.
Mes « 3 à 4× » incluaient des points contaminés par le rupteur. Mesuré propre
sur le I4 par défaut :

| régime | PMEP | littérature (Heywood ch. 13) |
|---|---|---|
| 2000 | -0.350 bar | -0.2 à -0.4 |
| 2500 | -0.525 bar | -0.2 à -0.4 |
| 6000 | -1.522 bar | -0.4 à -0.6 |

Soit ~1.5× en milieu de plage et ~2.0× près du régime nominal, pas 3-4×.
`EngineLab.GasExchange` est enregistrée **report-only** : les deux plafonds
littérature sont gelés, les gardes de signe/croissance/réconciliation
(`net == gross + pmep`) sont actives. La promotion est l'ajout de
`--enforce-gas-exchange` dans `tests/CMakeLists.txt`, jamais l'assouplissement
d'un plafond.

### Ce qui reste non tenu, et donc non promu

La bande de régime **dans** un point n'est pas encore gatée : l'absorbeur reste
sous-amorti en milieu de plage (le point 4500 tr/min oscille sur [4074, 4990]
autour d'une moyenne juste de 4518). La moyenne est bonne, le signal n'est pas
établi. C'est le même piège que la porte de ralenti (§ ci-dessus) : une fenêtre
moyennée ne voit pas un signal non établi. La bande est **rapportée** ;
resserrer le contrôleur est un changement distinct de rendre la dérive visible.

## D.1 — le catalogue contre les constructeurs, mesuré proprement (mesuré)

Balayage à pas de 250 tr/min, absorbeur corrigé, plafond sous le rupteur
(`dyno_fix5`). Les trois Yamaha sont les moteurs demandés ; K20A, LS3 et 2JZ sont
des **témoins** écrits bien avant eux, donc ils datent d'avant tout ce travail.

| moteur | couple sim | couple réel | puiss. sim | puiss. réelle | % puiss. |
|---|---|---|---|---|---|
| Yamaha CP2 | 72.7 @4750 | 67 @6500 | 57.4 @9243 | 54.0 @8750 | **106 %** |
| Yamaha CP3 | 85.1 @4000 | 93 @7000 | 75.4 @10004 | 87.5 @10000 | 86 % |
| Yamaha CP4 | 85.6 @5750 | 111 @9000 | 76.7 @11009 | 118 @11500 | 65 % |
| Honda K20A | 222.0 @3249 | 206 @7000 | 134.4 @7495 | 162 @8000 | 83 % |
| GM LS3 | 624.4 @2500 | 575 @4600 | 281.3 @5253 | 321 @5900 | 88 % |
| Toyota 2JZ | 437.7 @2750 | 427 @4000 | 198.6 @5488 | 239 @5600 | 83 % |

Ce qui a changé depuis le début de la session, et ce qui reste :

* **Le régime de puissance maximale est maintenant juste.** Le CP3 tombe à
  10004 tr/min contre 10000 annoncés, le CP4 à 11009 contre 11500. Avant les
  correctifs le CP3 culminait à 7178. C'est le gain du plateau de port, des
  diamètres de soupape et de l'interpolation du déphaseur.
* **Le couple maximal arrive toujours 1750-3750 tr/min trop tôt, sur les six
  moteurs**, témoins compris. C'est le défaut principal restant et il n'est pas
  dans les fichiers moteur : il est cohérent avec l'excès de pompage, qui croît
  avec le régime et rabat donc la BMEP haut-régime.
* **La valeur du couple maximal est bonne à 93-109 %.** Le simulateur sait faire
  la bonne quantité de couple ; il ne sait pas encore la placer au bon régime.

### Les deux correctifs qui ont produit ces gains

**Plateau de port.** L'aire efficace de soupape était
`min(courtine, 0.48 x aire_de_tête)` **puis** multipliée par le Cd en aval : la
contraction au siège était donc comptée deux fois. Elle vaut maintenant
`min(Cd x courtine, plateau x aire_de_tête)` avec 0.58 (adm.) et 0.51 (éch.),
milieu de la fourchette de production (Heywood ch. 6), et le Cd qui voyage en
aval devient 1.0. Gain seul, mesuré : CP2 39.0 -> 41.7 kW, CP3 56.4 -> 60.2,
CP4 62.9 -> 66.7, K20A 129.2 -> 134.8, et les pics de VE remontent en régime.

**Interpolation du déphaseur.** Une table `continuous_control` non rectangulaire
tombait dans une pondération inverse-distance (Shepard), qui n'est pas monotone
entre voisins : la table `sport_na` du K20A demande 28 deg d'avance admission à
6000 tr/min et 12 deg à 8500, et l'ancien repli livrait encore 26 deg à
5000 tr/min sans jamais redescendre. Remplacé par un linéaire par morceaux en
régime (la charge ne départage que les points de même régime). Effet mesuré :
le régime de puissance maximale monte sur **tous** les moteurs (K20A
7331 -> 7705, LS3 5149 -> 5419, 2JZ 5287 -> 5576) sans changer la puissance
maximale — un gain de forme de courbe, pas de niveau.

### Couplage à connaître : le plateau tient le ralenti du Big Twin

`EngineLab.IdleStabilityRegression` échoue maintenant sur **un** moteur : le
Merlin cale en revenant d'un coup de gaz. Isolation mesurée :

| état | Merlin | Big Twin |
|---|---|---|
| plateau corrigé (actuel) | cale après un blip | sain |
| plateau d'origine (0.48, Cd en aval) | sain | **cale à ralenti établi** (4 symptômes) |

Aucun des deux états n'est vert. Au dernier passage vert (01:46) le Big Twin
tenait 764 tr/min pour une cible de 760 avec un minimum à 725 : il passait, mais
il était déjà fragile. Le ralenti des gros cylindres du catalogue est donc
**marginal**, et le plateau ne crée pas le défaut : il déplace le moteur qui
bascule, parce que son surplus d'aire est ce qui maintient aujourd'hui le Big
Twin en vie.

Le plateau **ne doit pas** être annulé pour faire passer la porte : c'est une
correction juste (double comptage supprimé), mesurée, et qui vaut ~7 % de
puissance sur tout le catalogue. Ce qui doit être corrigé, c'est le ralenti des
gros cylindres — le « verrouillage par noyade » déjà ouvert sur le Merlin. Le
déphaseur n'y est pour rien : `v12_aircraft` et `v_twin_torque` n'ont pas de
table `continuous_control`, et la fonction sort avant tout calcul quand la table
est vide.

### Deux observations que le nouveau télémètre rend lisibles

**Le rapport de piégeage est sain partout.** `ve / delivered_ve` vaut 1.003-1.017
sur les six moteurs et sur toute leur plage. Les deux comptabilités n'empruntent
pas le même chemin jusqu'à `EngineState`, donc leur accord est un contrôle vivant
du bilan d'oxygène, et aucun moteur ne montre la signature > 1.05 d'une
combustion incomplète.

**Le 2JZ suralimenté a la pire boucle de pompage du catalogue**, ce qui est
physiquement à l'envers : -2.32 bar à 5000 tr/min et -3.12 bar à 6000, contre
-1.76 pour le K20A atmosphérique au même régime relatif. Un moteur suraliminté
paie bien la contre-pression de turbine à l'échappement, mais la suralimentation
pousse le piston pendant l'admission, donc sa PMEP nette au WOT doit être petite,
voire positive. Un déficit cinq fois trop grand sur le seul moteur boosté du
catalogue désigne le couplage compresseur/plénum, pas les soupapes. À instrumenter
séparément — ce n'est pas le même défaut que l'excès de pompage atmosphérique.

## L'excès de pompage est le temps d'échappement, et quatre causes sont écartées

`pumpingMeanEffectivePressureBar` disait « ce moteur pompe trop » sans dire de
quel côté. La boucle est maintenant scindée en ses deux temps
(`exhaustStrokeMeanEffectivePressureBar`, l'admission étant le reste), et la
réponse est nette. Moteur de référence I4, pleine charge :

| tr/min | PMEP | moitié échappement | moitié admission |
|---|---|---|---|
| 2000 | -0.351 | -0.148 | -0.203 |
| 3000 | -0.627 | -0.392 | -0.235 |
| 4000 | -0.682 | -0.560 | -0.122 |
| 5000 | -1.108 | -0.902 | -0.207 |
| 6000 | -1.522 | -1.214 | -0.308 |
| 6500 | -1.584 | -1.280 | -0.304 |

Littérature (Heywood ch. 13), atmosphérique pleine charge : PMEP **totale** -0.2
à -0.4 bar en milieu de plage, -0.4 à -0.6 près du régime nominal. **L'admission
est dans la bande.** Tout l'excès est sur le temps d'échappement, et il croît en
rpm^1.8 (facteur 8.65 pour un rapport de régime de 3.25) — une signature de
limitation de débit, pas d'un décalage statique. C'est cohérent avec le défaut de
forme de courbe : une perte qui croît comme le carré du régime rabat la BMEP haut
régime et fait tomber le pic de couple trop tôt.

Ces chiffres sont **inchangés au millième** depuis la scission, à travers les
quatre commits du rework : rien n'a touché à l'échange gazeux, et les courbes
Yamaha ont été redressées *malgré* ce défaut, pas en le corrigeant.

Quatre hypothèses ont été mesurées puis écartées. Ne pas les reprendre :

1. **La géométrie d'échappement est trop restrictive.** Réfutée par
   `EngineLabExhaustFlowBench`, banc d'écoulement stationnaire sur le réseau
   réellement compilé (même maillage que le simulateur). À 0.139 kg/s — le K20A
   en fait 0.154 à 8000 tr/min — l'orifice ne demande que ~34 kPa relatifs et le
   silencieux tient 2.7 kPa. Bilans de masse et d'énergie bouclés, résultats
   identiques au bit à deux temps de stabilisation. *Réserve honnête : un banc
   stationnaire au débit **moyen** ne dit rien du pic de vidange, qui vaut
   plusieurs fois la moyenne. Il réfute « le tuyau est trop petit », pas « le
   tuyau n'encaisse pas l'impulsion ».*
2. **L'aire efficace de soupape d'échappement est trop petite.** Réfutée : porter
   le plateau de 0.51 à 0.95 (diagnostic, annulé depuis) ne retire que 12 % à
   6000 tr/min. Beaucoup à 2000 (57 %), rien là où le défaut vit.
3. **La loi de débit de soupape sous-débite.** Réfutée par la mesure, et c'est la
   plus utile : avec `exh_cda_mm2` et `exh_valve_gps` publiés par
   `EngineLabPhysicsPerfHarness --trace`, le rapport du débit réel à la capacité
   isentropique quasi-stationnaire calculée depuis les mêmes états amont/aval et
   la même aire vaut **≈ 1.00 aux fortes ouvertures**. La soupape délivre ce que
   la thermodynamique permet.
4. **De l'énergie est créée à la jonction du collecteur.** Réfutée : le bilan
   d'énergie du banc boucle exactement (E_orifices == E_sortie à chaque point).
   La jonction du K20A est à 1147 K contre un réservoir à 1050 K au repos, et
   cette répartition reste inexpliquée, mais rien n'est créé. Une affirmation de
   violation de conservation a été émise puis **retirée** sur cette base.

Une cause partielle est confirmée : le **couplage multi-cadence** moyenne l'état
du cylindre sur son intervalle puis calcule un seul flux depuis cette moyenne.
Le flux étant concave en l'écart de pression, cela sous-estime le transfert
partout où l'état bouge dans l'intervalle — donc pendant la vidange.
`EngineLabGasExchangeTests --oracle-coupling` (réseau avancé à chaque sous-pas)
retire **23-29 %** de la perte au-dessus de 2500 tr/min, et un peu *plus* mauvais
à 2000. Cumulé avec l'aire, cela fait ~35 % : il reste 2 à 3 fois d'inexpliqué.

Le symptôme brut, au trace K20A 8000 tr/min à travers le temps d'échappement : le
cylindre reste entre 1.8 et 3.9 bar absolus contre un primaire à 1.5 bar, et la
pression **remonte** de 1.79 bar à 260° à 3.86 bar à 334° alors que la soupape est
encore ouverte de 13 à 4 mm et que le piston chasse. Le gaz est comprimé au lieu
d'être évacué. La vidange n'est pas non plus terminée au PMB (~3.1 bar à 180°
contre 1.64 bar dans le tube).

### La soupape est définitivement hors de cause (mesuré, expérience décisive)

L'oracle mesuré sur la **moitié échappement** seule, et non plus sur la PMEP
totale, retire un **23-26 % constant au-dessus de 2500 tr/min** (et 11 % de
*plus* de perte à 2000, où le couplage n'est pas le problème) :

| tr/min | production | oracle | oracle + aire soupape x6 |
|---|---|---|---|
| 2000 | -0.148 | -0.165 | -0.107 |
| 4000 | -0.560 | -0.398 | -0.377 |
| 6000 | -1.214 | -0.933 | -0.893 |
| 6500 | -1.280 | -0.988 | -0.941 |

La troisième colonne est l'expérience décisive : plateau d'échappement porté de
0.51 à **3.00** de l'aire de tête — six fois, géométriquement impossible — *avec*
l'oracle actif. À 6000 tr/min cela ne retire que **4 %**. À 2000 tr/min cela en
retire 35 %, ce qui confirme au passage que l'aire gouverne bien le bas régime,
là où le défaut n'est pas.

La preuve positive vient avec : à couplage identique, ouvrir la soupape six fois
ne fait pas *baisser* la pression du tube primaire, elle la fait légèrement
**monter** (170.3 -> 172.6 kPa de crête à 6000). Le cylindre ne peut pas
descendre sous son tube, et la pression du tube est fixée par le réseau, pas par
la soupape.

**Conclusion : la perte de pompage du temps d'échappement est fixée en aval de la
soupape.** Les trois hypothèses « côté cylindre » (aire, loi de débit, phasage de
la vidange) sont toutes tombées. Le banc stationnaire n'avait réfuté qu'une
restriction au débit **moyen** ; le pic de vidange vaut plusieurs fois la moyenne
et le banc ne peut pas le voir.

### Ce qui reste à faire, dans l'ordre

1. **Corriger la quadrature du couplage.** Les 23-26 % sont réels et ce n'est pas
   un artefact d'oracle : c'est la bonne intégrale contre une moyenne. Avancer le
   réseau à chaque sous-pas est trop cher (le réseau vaut 29-52 % du pas), mais
   la *formule d'orifice* est bon marché. Sous-échantillonner le **flux** à
   chaque sous-pas mécanique, accumuler masse et énergie transférées, et ne
   remettre au réseau qu'une seule fois par intervalle de couplage donne la même
   quadrature sans le coût. Attention : cela change la voix d'échappement, donc
   c'est un changement de sonorité assumé, à mesurer au harnais audio.
   *Ne pas reprendre la pondération par la conductance de soupape : elle a été
   mesurée et ne change rien (-1.522 -> -1.525 bar à 6000). Le biais est sur
   l'état moyenné, pas sur l'aire moyennée.*
2. **Instrumenter le réseau en régime pulsé**, pas stationnaire. C'est le seul
   endroit où le facteur ~3.5 restant peut vivre.

Le critère de réussite est la promotion de `EngineLab.GasExchange` en bloquant
(`--enforce-gas-exchange` dans `tests/CMakeLists.txt`) : ses deux plafonds sont
gelés depuis la littérature et **ne doivent jamais être élargis** pour obtenir du
vert. État actuel, pour mémoire : crit3 0.525 contre 0.350, crit4 1.522 contre
0.750.

## Le simulateur tournait au ralenti, et le son en découlait (2026-07-26)

Plainte utilisateur : « plus il y a de cylindres, moins ça va » ; le Merlin V12 et
le LS3 V8 « presque aucun son » ; « il y a un décalage entre mes inputs et ce qui
se passe » ; et deux messages permanents dans l'encart diagnostic. La consigne
était de ne **pas** toucher à la génération du son avant d'avoir regardé tout ce
qui la précède. C'était la bonne consigne : rien de ce qui suit n'est dans la
chaîne audio.

### La quantité qui manquait : le facteur temps réel

`EngineRuntime::run` avance un pas **fixe** de `1/240 s` de temps simulé par
itération, puis dort jusqu'à une échéance d'horloge murale. Il n'y a pas
d'accumulateur, et au-delà de quatre pas de retard la boucle fait
`deadline = now`, ce qui **abandonne la dette** au lieu de la rattraper. Dès
qu'une itération coûte plus de temps mural qu'elle n'avance de temps simulé, la
simulation entière passe au ralenti, définitivement.

Aucun instrument existant ne pouvait le voir. `PhysicsPerfHarness` mesure le CPU
par pas sans savoir si le pas tenait dans son créneau ; le compteur d'`overruns`
dit qu'une échéance a été manquée, pas de combien de travail. D'où
`tools/RealtimeBudgetHarness.cpp` (`EngineLabRealtimeBudgetHarness`), qui fait
tourner le **vrai** thread `EngineRuntime` et rapporte
`simulationTimeSeconds / temps mural`.

Mesuré sur un portable 16 threads, maintien dyno à 5000 tr/min, avant correction :

| moteur | cyl | facteur | moteur | cyl | facteur |
|---|---|---|---|---|---|
| CP2 twin | 2 | 1.000 | Audi I5 | 5 | 0.416 |
| CP3 triple | 3 | 1.000 | 2JZ I6 | 6 | 0.402 |
| Big Twin | 2 | 0.998 | Flat-6 | 6 | 0.399 |
| Hayabusa I4 | 4 | 0.822 | Merlin V12 | 12 | 0.286 |
| CP4 I4 | 4 | 0.824 | **LS3 V8** | 8 | **0.266** |
| K20A I4 | 4 | 0.437 | | | |

Le V8 produisait **un quart** de temps réel. Les deux pires moteurs du tableau
sont exactement les deux que l'utilisateur avait nommés comme silencieux, et il
les avait nommés avant que la mesure existe. Le mécanisme est direct : la
télémétrie de pression cylindre est la *seule* excitation de la chaîne acoustique
d'échappement ; produite à 0.27x la cadence à laquelle le thread audio la
consomme, elle affame le modèle. **Un V8 muet est un symptôme du thread physique,
pas un défaut audio.**

### Le parallélisme coûtait plus qu'il ne rapportait

`SubstepParallelExecutor` dispatchait un fork-join **par sous-pas gaz**, soit
~19 000 barrières par seconde sur un V8 à 5940 tr/min. Quatre variantes mesurées
(facteur temps réel, LS3 / Merlin) :

| variante | LS3 | Merlin |
|---|---|---|
| origine : `notify_all` + spin 2048 `yield` | 0.266 | 0.286 |
| réveil ciblé par worker + spin | 0.249 | 0.256 |
| réveil ciblé, sans spin | 0.282 | 0.336 |
| **en ligne (retenu)** | **0.343** | **0.396** |

Toutes les variantes filées perdent. Mon hypothèse initiale — que `notify_all`
réveillait les non-participants, qui brûlaient ensuite 2048 `yield()` (des appels
système sous Windows) — était **fausse** : la corriger a *dégradé* le résultat. Le
spin volait bien des cycles au thread même que la barrière attendait, mais le
supprimer ne suffit pas non plus. Le problème est la cadence : aucune barrière à
condition-variable ne s'amortit sur un corps aussi petit.

L'exécuteur est retiré, ainsi que son test. Vérifié **bit-identique** sur le LS3 :
couple 557.732 / 539.704 / 423.614, IMEP 11.964 / 12.130 / 10.306, VE
0.895 / 0.839 / 0.805, air 6598.852 / 6179.919 / 5934.833 — tous les chiffres
imprimés inchangés. Le coût du pas à 5940 tr/min passe de 15 062 à 10 541 us.

`decoupledSharedVolumeCylinderThreshold` (8) survit et n'est **plus** lié au
threading : c'est uniquement le choix Jacobi / Gauss-Seidel sur le volume
partagé. Le déplacer déplace pour de vrai la calibration des gros moteurs.

Une vraie parallélisation devrait garder les workers résidents **à travers** les
sous-pas, en ne se synchronisant que là où le plénum partagé est touché. C'est une
autre architecture, pas un réglage de celle-ci.

### Où part réellement le temps : l'admission 1-D, à 80 %

Instrumentation par bloc à l'intérieur du sous-pas (K20A / LS3 / Merlin / CP4) :

| bloc | K20A | LS3 | Merlin | CP4 |
|---|---|---|---|---|
| **admission 1-D** | **83.5 %** | **81.9 %** | **83.6 %** | **75.3 %** |
| physique cylindre | 7.3 % | 7.3 % | 4.9 % | 10.3 % |
| échappement 1-D | 4.2 % | 7.1 % | 8.9 % | 6.4 % |
| ECU | 0.4 % | 0.2 % | 0.1 % | 0.6 % |
| reste | 4.6 % | 3.8 % | 2.5 % | 7.4 % |

Ceci **remplace** le profil 41 % cylindres / 29 % échappement / 30 % reste qui
figurait dans `CLAUDE.md` : ce profil précède les runners 1-D et n'est plus vrai.

Ce n'est pas un emballement CFL : `accepted/call` vaut 1.09 et `rejected/call`
0.000. C'est **8.1 us de coût largement fixe** pour avancer un runner de 6 à 12
cellules, payé deux fois par sous-pas mécanique et par cylindre. Répartition
interne d'un `advance` : flux 47 % (deux étages RK2), parois 24 %, préparation des
primitives 12 %, pas stable 4 %, setup 1 %.

Le maillage runner est en cellules de ~30 mm, dont la limite CFL propre autorise
~66 us, alors que le sous-pas mécanique l'appelle tous les ~26 us : **le réseau
est sollicité ~2.5x plus finement que sa propre stabilité ne l'exige.**

Piste principale restante, par ordre de préférence :

1. **Réduire le coût fixe par appel** — bit-identique si c'est fait proprement,
   donc sans risque pour les attracteurs de ralenti. Les parois (24 %) ont une
   constante de temps en *secondes* (masse d'aluminium) et sont intégrées
   ~38 000 fois par seconde ; leur sous-cyclage est presque gratuit
   physiquement. À mesurer, pas à supposer.
2. **Ne pas** multirater l'admission comme l'échappement sans mesure : cela
   importerait exactement le biais de moyennage documenté plus haut, du côté qui
   fixe la VE.

### Deux fausses alarmes dans l'encart diagnostic

**« Contre-pression d'échappement excessive »** : le test lisait
`state.exhaustPressureKpa` et le comparait à une tolérance dimensionnée pour une
*moyenne*. Or ce champ est le `max` sur cylindres de la pression **instantanée**
du tube — une enveloppe de pic de vidange, ce que `CLAUDE.md` documentait déjà
sans que le diagnostic en tienne compte. Un LS3 sain culmine à 172 kPa contre
~101 ambiants **en vidangeant librement** : le message s'allumait par
construction et ne s'éteignait jamais. Un vrai champ moyen est désormais publié,
`exhaustBackPressureKpa` (moyenne sur les ports, amortie sur ~3 périodes
d'allumage : ce que lit un manomètre sur un piquage de collecteur), et le seuil
vient de la littérature — ~15-30 kPa de contre-pression moyenne pour un
atmosphérique à la puissance nominale, donc 40 kPa au-dessus de l'ambiant est
franchement excessif ; 90 kPa pour un turbo, dont la turbine est une restriction
voulue. Noter que la variable locale s'appelle toujours `collectorPressureKpa`
alors qu'elle ne vaut pas une valeur de collecteur.

**« Force longitudinale limitée par l'adhérence »** : `tractionLimited` était un
OU **collant** sur les sous-pas mécaniques de la transmission. Un seul sous-pas
écrêté sur cinq allumait le voyant pour toute la trame — et avec un ressort de
pneu en *vitesse* de glissement à `normalForce * 7.5` N/(m/s), un sous-pas écrêté
arrive à chaque passage de rapport. Il faut maintenant une majorité de sous-pas.
Le champ n'est lu que par l'affichage, donc l'affiner ne touche à aucune physique.

### La moto n'avait pas de réduction primaire

Une moto a **trois** réductions : vilebrequin vers cloche d'embrayage (1.6-2.0),
boîte, puis chaîne. `TransmissionConfig` n'a pas de champ primaire, il faut donc
le replier dans `final_drive_ratio` — « tout ce qui est en dehors de la boîte ».
`motorcycle_6_speed` sortait à `2.62 x 2.75 = 7.21` en première alors qu'un MT-07
est à `1.925 x 2.846 x 2.688 = 14.73`.

Reconnaissable à l'arithmétique, pas au ressenti : avec le rapport livré, la
**première** atteignait 141 km/h à 9000 tr/min (réel : 69) et la sixième 314 km/h
(réel : ~189). Toutes les motos étaient surmultipliées de 1.6 à 2.0x dans chaque
rapport, donc le couple à la roue divisé par deux partout — exactement la plainte
« très lent, peu importe le rapport, même avec le CP2 ». Le `cruiser_6_speed`
avait le même défaut.

**Reste ouvert** : le transfert de charge longitudinal n'est pas modélisé.
`tractionLimit = mu * m * g * drivenAxleWeightFraction` avec la fraction figée à
0.55. Une moto en accélération franche transfère à ~0.9 et plus sur l'arrière,
donc l'adhérence arrière est sous-estimée d'un facteur ~1.7 ; une propulsion
aussi. L'ajouter demande empattement et hauteur de centre de gravité, que
`VehicleConfig` ne porte pas.

### Ce qui n'est PAS encore expliqué

Le facteur temps réel après retrait du fork-join reste très en dessous de 1.0 :
LS3 0.357, Merlin 0.388, K20A 0.433, 2JZ 0.398, Flat-6 0.396, TDI 0.679,
Hayabusa 0.803, CP4 0.809, et seuls les 2 et 3 cylindres tiennent le temps réel.
Le fork-join ne valait qu'environ un tiers du déficit du V8. **Le reste est
l'admission 1-D**, et tant qu'il n'est pas traité, le son des gros moteurs restera
affamé. Aucune correction de la chaîne audio ne peut compenser ça.
