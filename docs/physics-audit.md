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

| Phase | Cible | Gravité | Gate instrumenté |
| ----- | ----- | ------- | ---------------- |
| 0 | Figer cette baseline | — | aucun | ✅ fait |
| 1 | Bielle articulée : `rodAngle` calculé puis jeté (`MechanicalKinematics.cpp`) | ~~bug~~ → nettoyage | PhysicsRegression | ✅ fait |
| 2 | Embrayage verrouillé : couplage retardé une frame | ~~bug~~ → non-problème | Core (garde catalogue) | ✅ fait |
| 2 | Embrayage verrouillé : couplage retardé 2 ticks, pas une contrainte | bug/à mesurer | Core + assert résidu énergétique |
| 3 | Double comptage frottement paliers/piston | à mesurer | garde FMEP + CombustionPhasing IMEP |
| 4 | Nettoyages doc (γ, modèle moyen vestigial) | clarté | build vert, baselines identiques |
| 5 | Fraction volumique + facteur 1.12 (couplés) | hack, gain-gated | CombustionPhasing + AudioRender |

Ordre recommandé : 0 → 1 → 2 → 4 → 3 → (5 seulement si gain observable).
Détails et méthode « mesurer avant de corriger » : voir l'historique d'audit.

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

Actions livrées (comportement inchangé, baselines identiques, 13/13 vert) :

- suppression du code mort `rodAngle` + `(void)rodAngle` (`MechanicalKinematics.cpp`) ;
- ajout d'un test de régression verrouillant les invariants ci-dessus
  (`tests/PhysicsRegressionTests.cpp`) — les tests existants ne couvraient que
  le PMH, pas la trace complète. Non-vacuité prouvée : en sabotant la rotation
  d'articulation, le test échoue sur « must actually sweep the articulation band ».
