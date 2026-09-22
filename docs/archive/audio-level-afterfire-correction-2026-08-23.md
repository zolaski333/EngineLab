# Correction du niveau global et de l’afterfire — 23 août 2026

## Résultat

Ce lot corrige deux défauts distincts observés dans l’application :

1. toutes les sources physiques étaient atténuées d’environ **22 dB** par une
   calibration Pa→numérique dimensionnellement incorrecte ;
2. cocher l’afterfire sur un moteur propre sélectionnait en pratique un flux
   continu, puis le rendu prolongeait chaque tranche de réaction. Le résultat
   était une vague régulière de niveau/hauteur, pas des combustions séparées.

La chaîne courante conserve la pression physique, le graphe d’échappement et la
sécurité existants. Elle ne normalise pas chaque moteur, n’ajoute aucun sample
de pop, aucun bruit de crépitement et aucun compresseur. Le niveau est corrigé
par une propriété unique du moniteur ; les pops proviennent toujours de la masse
et de l’énergie réellement présentes dans les volumes finis du réseau.

Ce document décrit une correction d’ingénierie et ses mesures logicielles. Il ne
prétend pas identifier une calibration constructeur ni valider le son contre un
enregistrement réel. Le chantier dyno n’est pas modifié par ce lot.

## 1. Niveau global trop faible

### 1.1 Chemin relu

La vérification a suivi la chaîne complète : pression cylindre et échappement,
observateur acoustique, conversion Pa→FS, stems, mode de moniteur, master,
sécurité de sortie, callback de production et rendu catalogue. Les compteurs
AGC, saturation de voicing, soft-limit et clamp final ont été lus séparément.

Le signal physique en sortie de l’observateur est exprimé en pascals. La
conversion numérique est :

```text
p_rms_pleine_echelle = 20 µPa × 10^(SPL_FS / 20)
p_crête_pleine_echelle = sqrt(2) × p_rms_pleine_echelle
échantillon_FS = pression_Pa / p_crête_pleine_echelle
```

### 1.2 Cause racine

Le code employait `SPL_FS = 156 dB` alors que l’architecture et la documentation
de référence spécifiaient 134 dB. Les deux conversions sont :

| Calibration | Pa RMS à 0 dBFS | Pa crête à 0 dBFS | Gain relatif |
|---|---:|---:|---:|
| ancien code, 156 dB | 1 261,915 | 1 784,617 | 1,000 |
| contrat restauré, 134 dB | 100,237 | 141,757 | **12,589× / +22 dB** |

La valeur 156 dB avait été choisie à partir d’un ancien pic de démarrage du
Merlin. La décomposition en stems montre que ce pic venait de `starterSound`,
une couche synthétique déjà numérique, et non d’une pression en pascals. Cette
couche n’est donc pas multipliée par la calibration acoustique. Utiliser son pic
pour choisir `SPL_FS` atténuait toutes les pressions physiques sans réduire le
signal qui avait motivé l’atténuation.

Sur le passage Merlin relu, la couche mécanique numérique atteignait environ
0,214 au démarrage, tandis que les maxima SI observés restaient proches de
67 Pa à l’échappement et 69 Pa à l’admission. L’ancien réglage demandait
1 784,6 Pa crête pour atteindre 0 dBFS : il laissait donc inutilement plus de
28 fois la pression SI observée avant pleine échelle.

### 1.3 Deuxième défaut d’interface

Le mode `physical_reference` met volontairement les couches physiques à l’unité
afin de préserver la référence SI. Dans ce mode, modifier le fader échappement
ne changeait rien, mais le contrôle restait visuellement mobile. Un utilisateur
pouvait donc mettre « échappement » au maximum sans que la chaîne applique ce
gain.

Désormais, modifier un fader authoré (échappement, admission, mécanique,
combustion, shelves ou bruits) active visiblement `MODE CAPTURE`, le mode qui
honore ces choix. Le master et l’IR restent utilisables dans les deux modes. Le
bouton ne peut plus mentir sur l’état audio effectif.

### 1.4 Correction

- `AcousticMonitorCalibration::defaultFullScaleSplDb` passe de 156 à 134 dB ;
- le défaut atomique de `RealtimeAudioState` est aligné à 134 dB ;
- un test fixe explicitement le triplet 134 dB / 100,237 Pa RMS /
  141,757 Pa crête ;
- les contrôles de voicing activent visiblement le mode de capture qui les lit ;
- aucune constante de gain par moteur et aucun auto-leveler ne sont ajoutés.

### 1.5 Mesure catalogue fraîche

Le rendu Release `EngineLabAudioRenderHarness` a été rejoué sur les 16 moteurs.
Le tableau ci-dessous mesure les deux dernières secondes des WAV catalogue
fraîches, tous canaux confondus, après la correction à 134 dB :

| Moteur | RMS FS | dBFS RMS | Crête FS |
|---|---:|---:|---:|
| 2JZ-GTE I6 | 0,02694 | −31,39 | 0,14935 |
| Flat-6 refroidi par air | 0,02714 | −31,33 | 0,18863 |
| Audi I5 turbo | 0,03179 | −29,95 | 0,21228 |
| Audio Physics Lab Twin | 0,01750 | −35,14 | 0,08255 |
| Big Twin V2 | 0,02543 | −31,89 | 0,14459 |
| EJ25 Flat-4 turbo | 0,03437 | −29,28 | 0,32349 |
| Hayabusa I4 | 0,01529 | −36,31 | 0,13898 |
| K20A I4 | 0,02424 | −32,31 | 0,14713 |
| LS3 crossplane V8 | 0,02054 | −33,75 | 0,10471 |
| Merlin V12 | 0,04343 | −27,24 | 0,31369 |
| MT-07 Full System | 0,01831 | −34,75 | 0,09787 |
| Radial R5 | 0,01662 | −35,59 | 0,12167 |
| EA288 TDI | 0,01105 | −39,13 | 0,18314 |
| CP2 Twin | 0,01600 | −35,92 | 0,10199 |
| CP3 Triple | 0,01551 | −36,19 | 0,06934 |
| CP4 crossplane I4 | 0,01990 | −34,03 | 0,13678 |

Le scénario de callback/application mesure par ailleurs : K20 0,0441 RMS /
0,2277 crête, 2JZ 0,0266 / 0,1878, LS3 0,0546 / 0,2689 et Merlin 0,0815 /
0,4670. Le vrai ralenti du Big Twin reste plus calme, environ 0,0028 RMS pour
0,1779 de crête, sans être ramené artificiellement au niveau des moteurs rapides.

Sur les seize moteurs, `leveler=0`, `saturation=0`, `softLimit=0` et
`hardClamp=0`. La correction est un scalaire de moniteur et n’ajoute aucun DSP
par échantillon.

Les artefacts finaux sont sous `out/audit-2026-08-23-level-134-final/`.

## 2. Afterfire réduit à une vague régulière

### 2.1 Causes racines cumulées

Le symptôme ne venait pas d’un seul coefficient.

#### Toggle actif mais stratégie continue

Les défauts propres étaient 900 K, 10 ms, carburant 0, pulse 0 Hz et variation
nulle. Le callback interprète un pulse à 0 Hz comme `continuous_anti_lag`.
Cocher uniquement « afterfire » conservait ces valeurs : l’UI promettait des
pops alors que la stratégie demandait explicitement une combustion continue.

#### Fenêtre ECU trop longue

Le précédent profil de démonstration utilisait 4 Hz et 35 % de duty, soit une
fenêtre ouverte de 87,5 ms. À 4 000 tr/min, cette fenêtre couvre plusieurs
opportunités d’injection et plusieurs combustions sur un multicylindre. La
chimie mesurée restait active 76,6 % du temps sur le Twin et près de 100 % sur le
K20. Réduire seulement la constante chimique changeait le pic, pas cette forme.

#### Cadence trop régulière

Le hacheur global et sa variation de seulement 25 % formaient une modulation
facilement reconnaissable. L’induction locale pouvait ensuite varier, mais elle
recevait déjà une suite de longues fenêtres presque métronomiques.

#### Inventaire ancien au lever

La chimie avait été entièrement désactivée sous charge puis activée au lever.
Les traces d’hydrocarbures normales pouvaient donc traverser la chauffe de 60 s
sans oxydation et apparaître brusquement comme un réservoir d’afterfire. Cela
gonflait la première réaction et pouvait transformer l’overrun en longue vidange.

#### Énergie acoustique prolongée

Chaque tranche de réaction transporte une énergie `E` et une durée `dt`, donc
une puissance `E/dt`. Le renderer maintenait cette puissance pendant `2 × dt`,
avec un minimum de 0,5 ms. Il doublait ainsi l’énergie temporelle reconstruite
et fusionnait des noyaux voisins en une enveloppe lisse.

#### Métrique de carburant trompeuse

Le harness intégrait `deliveredFuelMgPerCycle`, valeur du dernier cycle achevé
conditionnée par l’état courant de l’ECU. Une fenêtre de quelques dizaines de
millisecondes peut s’ouvrir et se fermer entre deux frontières de cycle : cette
estimation sous-comptait fortement la masse. Le compteur cumulatif de carburant
réellement mesuré par l’injecteur est désormais l’autorité du contrat ;
l’ancienne estimation reste affichée et explicitement étiquetée diagnostic.

### 2.2 Corrections architecturales

- la première activation d’un moteur propre prépare `discrete_afterfire` ;
- le profil produit est 800 K, réaction 2 ms, carburant moyen 8 %, 2 Hz,
  duty 8 % et variation temporelle 40 % ;
- cette première activation installe la corrélation d’induction schéma 8
  (`Ea/R=13340 K`, exposant pression 0,989, exposant richesse −0,577, décroissance
  8 ms) ; une calibration existante n’est jamais écrasée ;
- la chimie maintient les espèces et l’énergie sous charge quand le modèle est
  authoré, mais ne publie les événements audio/télémétrie que pendant un vrai
  DFCO afterfire ou un rupteur humide ;
- la voix de réaction dure exactement le pas qui a produit son énergie ;
- le profil catalogue du Twin, `DEMO AUDIBLE` et le toggle propre partagent les
  mêmes valeurs ;
- le harness expose maintenant `--induction-ms` et `--correlated-induction`
  pour reproduire les deux contrats sans modifier un moteur.

Le modèle reste dormant sur les moteurs en `clean_dfco`. Le maintien chimique
ne coûte donc rien aux quinze moteurs propres du catalogue.

### 2.3 Sélection du profil

Les candidats ont été évalués après 60 s de chauffe et 8 s d’overrun. Le but
n’était pas le plus gros pic, mais une enveloppe discrète valable sur peu et
beaucoup de cylindres avec de la marge numérique.

| Carburant / duty / cadence | Twin | K20 | Décision |
|---|---|---|---|
| 18 % / 35 % / 4 Hz, ancien | duty chaleur 76,6 % | presque 100 % | flux trop continu |
| 5 % / 5 % / 2 Hz | 5 événements, trou maximal 3 996 ms | événements séparés | fenêtre trop courte pour le Twin |
| 5 % / 10 % / 2 Hz | aucune induction établie dans la fenêtre stabilisée | non retenu | paquet instantané trop pauvre |
| 10 % / 10 % / 2 Hz | 13 événements, saut 90,6 kPa | duty 70,3 %, saut 92,7 kPa | trop proche de la borne 100 kPa |
| **8 % / 8 % / 2 Hz** | **10 événements, saut 80,9 kPa** | **16 événements, saut 67,8 kPa** | **retenu** |

La fenêtre finale dure 40 ms, puis reste fermée environ 260 à 660 ms selon la
variation authorée. Fraction et duty étant égaux, l’injection instantanée n’est
pas diluée sous la borne de flammabilité ; sa masse moyenne reste inférieure de
plus de moitié à l’ancien profil 18 %.

### 2.4 A/B Twin strict

Les deux WAV utilisent exactement la même trajectoire, la même chimie et les
mêmes 5 419,634 J. Le contrôle désactive seulement la copie des événements vers
le réseau acoustique.

| Mesure stabilisée | Source réaction OFF | Source réaction ON |
|---|---:|---:|
| réactions avant lever | 0 | 0 |
| événements chaleur | 10 | 10 |
| masse brûlée | 89,373 mg | 89,373 mg |
| crête audio | 0,04764 | 0,05835 |
| P99,9 audio | 0,03310 | 0,03349 |
| crest overrun | 4,75 | 5,73 |
| coût audio moyen / p99 | 13,1 % / 14,2 % | 13,5 % / 15,9 % |
| coût simulation moyen / p99 | 20,3 % / 23,9 % | 20,4 % / 23,9 % |

La différence ON−OFF mesure 0,03415 de crête et 0,001365 RMS. Sur des fenêtres
de 5 ms, son RMS médian vaut 0 et son maximum 0,01229. Les dix maxima isolés
arrivent à 0,005, 0,755, 1,845, 2,305, 3,280, 3,900, 4,775, 5,820,
6,350 et 7,265 s dans la fenêtre analysée. Cette sparsité réfute directement
l’ancien comportement de vague continue.

Le Twin de laboratoire contient un absorbeur ; sa différence est logiquement
dominée par 120–500 Hz. Ce test valide un pop/gargouillis grave propagé par le
graphe, pas un claquement haute fréquence ajouté après le silencieux.

### 2.5 Corroboration K20 et LS3

| Mesure | K20 I4 | LS3 crossplane V8 |
|---|---:|---:|
| événements chaleur | 16 | 16 |
| espacement min–max | 270,8–733,3 ms | 287,5–708,3 ms |
| duty chaleur | 65,8 % | 68,3 % |
| énergie réaction | 24 932,415 J | 33 722,380 J |
| saut compact maximal | 67,847 kPa | 36,715 kPa |
| crête audio ON | 0,10551 | 0,06231 |
| coût simulation moyen / p99 | 40,0 % / 44,0 % | 71,2 % / 84,0 % |
| coût audio moyen / p99 | 19,6 % / 22,0 % | 33,9 % / 36,3 % |
| pas/blocs hors budget | 0 / 0 | 0 / 0 |

Sur le K20, l’A/B exact donne 0,05383 de crête différentielle. Le RMS médian
par fenêtre de 5 ms est 0,0000136, contre 0,02015 au maximum ; les seize maxima
sont espacés de 290 à 700 ms. La contribution est donc événementielle même si
la chaleur résiduelle reste non nulle entre plusieurs fronts.

Le LS3 est le cas de coût élevé. Une mesure courte de référence donnait 67,3 %
de budget simulation sans modèle afterfire et 70,5 % avec le premier candidat,
soit +3,2 points. Les pointes isolées déjà présentes dans la référence ne sont
pas attribuées à la chimie. Le passage final chaud ci-dessus reste sous budget à
chaque pas.

Tous les passages finaux publient `physical=yes`, `compiled=yes` et zéro pour
`dropF`, `dropP`, `dropA`, `dropR`, `reactLimit`, `late`, `pending`, `stolen`,
`trunc`, `boundary`, `legacy`, `leveler`, `saturation`, `softLimit` et
`hardClamp`.

Artefacts d’écoute principaux :

- `out/audit-2026-08-23-afterfire-final-authored-08/` ;
- `out/audit-2026-08-23-afterfire-twin-fuel08-duty08-no-source/` ;
- `out/audit-2026-08-23-afterfire-k20-fuel08-duty08/` ;
- `out/audit-2026-08-23-afterfire-k20-fuel08-duty08-no-source/` ;
- `out/audit-2026-08-23-afterfire-ls3-final-08/`.

## 3. Limites honnêtes

- 134 dB est une calibration de moniteur unique, pas une cible LUFS et pas une
  normalisation par moteur. Les vrais ralentis restent plus calmes que les
  accélérations ; `MODE CAPTURE` permet une présentation produite explicite.
- Le niveau final dépend encore du gain Windows, de l’amplificateur casque et de
  sa sensibilité. EngineLab ne mesure pas ces éléments externes.
- La réaction est une chimie globale à quelques espèces dans un réseau
  quasi-1D. Elle ne résout ni front de flamme 3D ni onde de choc non linéaire.
- Le pop du Twin absorptif est principalement grave/médium. Un « gunshot » très
  aigu demanderait soit une combustion plus rapide validée, soit un solveur
  non linéaire plus coûteux ; aucun bruit artificiel n’a été ajouté pour le
  fabriquer.
- Le profil 8 % est une calibration de démonstration robuste sur I2, I4 et V8,
  pas une cartographie OEM. Température de paroi, oxygène, richesse, régime et
  transport peuvent légitimement produire un événement nul.
- Le LS3 reste le scénario simulation le plus proche du budget dans ce lot. Le
  surcoût propre de l’afterfire est petit, mais l’optimisation générale du V8
  demeure un chantier séparé si l’on veut davantage de marge callback/UI.
- Le dyno n’a reçu aucun lissage ou changement opportuniste dans ce lot. Ses
  corrections de cadence et de métrologie continuent dans le plan dédié.

## 4. Fichiers modifiés

- `src/audio/include/enginelab/audio/AcousticMonitorCalibration.hpp` ;
- `src/runtime/include/enginelab/runtime/EngineRuntime.hpp` ;
- `src/app/src/AudioWorkshopWindow.cpp` ;
- `src/simulation/src/EngineSimulator.cpp` ;
- `src/audio/src/RealtimeEngineAudio.cpp` ;
- `engines/16_audio_physics_lab_689_twin.engine.yaml` ;
- `tools/AfterfireHarness.cpp` ;
- `tests/RealtimeRegressionTests.cpp` ;
- `tests/AudioWorkshopTests.cpp` ;
- `docs/realtime-audio.md` et `docs/physical-afterfire.md`.

## 5. Validation de livraison

La validation utilise exclusivement l’arbre Release autoritaire
`out/build/windows-vs2022`, après chargement de `scripts/vsenv.ps1` :

- reconstruction intégrale MSVC Release : succès ;
- tests ciblés `EngineLab.RealtimeRegression`, `EngineLab.AudioWorkshop` et
  `EngineLab.Core` : succès ;
- suite autoritaire finale : **42/42 CTest**, zéro échec, **728,10 s** ;
- les deux rampes dyno produit CP2 et LS3 sont incluses dans ces 42 tests ;
- `EngineLabAudioRenderHarness` final : `PASS`, similarité spectrale maximale
  0,800, seize moteurs finis, toutes les protections de sortie à zéro ;
- afterfire catalogué Twin : contrat physique/audio valide, aucun pas ou bloc
  hors budget ;
- afterfire K20 et LS3 : contrats valides, aucune perte/limitation, aucun pas ou
  bloc hors budget sur les passages finaux ;
- `EngineLab.exe` Release : processus encore vivant après cinq secondes de
  smoke caché, puis arrêté volontairement.

Le premier passage CTest avait correctement signalé que le fixture Core appelait
« non réactif » un moteur laissé avec `limiterKeepsFuel=true` près du rupteur.
Le test remet maintenant explicitement une calibration propre avant sa mesure de
cadence. Le test isolé passe en 46,49 s, puis la suite complète finale ci-dessus
passe 42/42. Aucun comportement produit n’a été désactivé pour satisfaire le
test : un vrai rupteur humide conserve son droit de publier ses réactions.

L’archive CPack est construite après cette copie embarquée du rapport, puis
extraite dans un dossier neuf. La taille et le SHA-256 externes sont consignés
dans le journal de progression et dans le compte rendu de livraison ; les écrire
dans le document avant de reconstruire l’archive créerait une dépendance de hash
circulaire.
