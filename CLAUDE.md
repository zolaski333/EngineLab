# EngineLab — guide pour agents

**Lis `VISION.md` avant toute tâche.** Ce fichier-ci dit *comment* travailler ;
`VISION.md` dit *pourquoi* et *vers quoi*. En cas de conflit, `VISION.md` gagne.

Ce fichier est volontairement court (plafond : ~200 lignes). Pour y ajouter une
règle, retire ou fusionne-en une autre. L'ancienne version complète (1 080
lignes, historique des mesures) est archivée dans
`docs/archive/notes-agents-2026-08.md` : c'est une source à consulter, pas une
consigne.

## Priorité actuelle

Seul endroit où la priorité est écrite. Ne travaille sur rien d'autre sans
accord explicite du propriétaire.

**Jalon 1 — le son du CP2 (MT-07, échappement Arrow).**

1. Enregistrements de référence faits par le propriétaire, régime connu.
2. Outil de comparaison simulation/réel au même régime (timbre par bande, ordres
   moteur, part de bruit), reproductible.
3. Réduire les écarts mesurés, par la méthode qui marche (physique, synthèse ou
   les deux), sans jamais exiger d'enregistrement par moteur.

Pistes connues, **non vérifiées**, à tester d'abord par la mesure :

- Afterfire : causes **mesurées** le 2026-09-23 (voir `docs/journal.md`). Le
  son d'une pétarade est plafonné à ~3,8 kHz par la reconstruction de la
  source (0,47 × cadence de couplage ≤ 8 kHz), et elle ne dépasse la crête du
  moteur que de 0 à +10 dB. La physique décide *quand* et *combien* ; la
  couche crack (`ReactionCrackSynthesiser`) fournit la bande > 4 kHz, non
  calibrée. Sur le CP2, rien ne brûle au seuil de 800 K : à mesurer d'abord.
- Au-delà de la coupure du mode plan (quelques kHz pour un tube de 50-80 mm),
  un modèle 1-D ne porte plus rien de physique : c'est la bande où la synthèse
  générique a le plus de chances d'aider.

## Règles de travail

- **Mesurer avant de corriger.** Ce code contient des compensations délibérées
  (fraction volumique de flamme compensée par un facteur 1,12 ; couche « voice »
  d'échappement qui sert de repli sans télémétrie…). « Corriger » l'une seule
  dégrade un comportement observable juste. Demande-toi ce qui compense en aval.
- **Les références viennent de la réalité** (enregistrements, littérature),
  jamais de la sortie actuelle du simulateur.
- **Une mesure qui change un seul facteur.** Une variante qui en change deux ne
  prouve rien.
- **Ne pas modifier le son par défaut** sauf si la tâche est explicitement un
  changement de son, vérifié au harness avant/après.
- **Pas de nouveau document daté.** Un résultat de mesure va dans
  `docs/journal.md` (10 lignes maximum par entrée, la plus récente en haut). Ce
  qui devient une règle durable va ici, en remplacement d'autre chose.
- **Jamais « validation finale ».** Écris ce qui a été mesuré et ce qui ne l'a
  pas été.
- **Une réfutation vaut une amélioration** : note-la dans la liste ci-dessous
  pour que personne ne la refasse.

## Build et tests

```
. scripts/vsenv.ps1                                                # À CHAQUE appel
cmake --build out/build/windows-vs2022 --config Release            # tout le projet
ctest  --test-dir out/build/windows-vs2022 -C Release              # tous les tests (~15 min)
```

- Les avertissements sont des erreurs. Build vert + `ctest` vert = le minimum
  pour tout changement.
- **`out/build/windows-vs2022` (Visual Studio) fait autorité** pour les commits
  et tous les chiffres de performance. `out/build/ninja-release` sert seulement
  à la boucle rapide ; ne jamais comparer un temps entre les deux arbres.
- Ninja : `-j 2` maximum (16 Go pour 12 threads → erreur `C1060` au-delà). Même
  cause pour MSBuild : en cas de `C1060`, relancer avec `/m:1`. Ce n'est pas un
  bug du code.
- **Ne jamais activer `ENGINELAB_ENABLE_COMPILER_CACHE`** : sccache casse le
  suivi des en-têtes de Ninja, et un en-tête modifié ne recompile plus rien.
- `scripts/vsenv.ps1` corrige un environnement cassé (VS 18 installé à côté de
  VS 2022, `vswhere` vide). Si CMake se met à viser VS 18, reconfigurer avec
  `-DCMAKE_GENERATOR_INSTANCE="C:/Program Files/Microsoft Visual Studio/2022/Community"`,
  y compris dans les `CMakeCache.txt` des sous-builds `_deps/*`.
- **Piège du binaire périmé** : un mauvais nom de cible (`MSB1009`) ne compile
  rien, et `ctest` relance l'ancien binaire. Exécutables de test :
  `EngineLabCoreTests`, `EngineLabExhaustTests`,
  `EngineLabPhysicsRegressionTests`, `EngineLabCombustionPhasingTests`,
  `EngineLabRealtimeRegressionTests`, `EngineLabComparisonHarness`,
  `EngineLabAudioRenderHarness`. Vérifie que la cible a bien été relinkée.
- Un nouveau test doit **échouer sans le correctif** (après rebuild de la bonne
  cible).
- Un `ctest` qui n'affiche plus rien après `Start N:` est **bloqué**, pas lent :
  vérifier avec `Get-Process ... | Select CPU,StartTime`. À l'inverse, un temps
  très court peut être un échec (`EngineLab.Core` : ~90 s s'il passe, ~8 s s'il
  avorte).

## Instruments

Déterministes, donc comparables d'un commit à l'autre :

- `EngineLabAudioRenderHarness` : rend le vrai chemin temps réel hors ligne ;
  RMS, facteur de crête, bandes spectrales, RT60 de l'échappement.
- `EngineLabGeometrySensitivityHarness` : un changement d'échappement change-t-il
  le timbre (`forme`), le niveau (`niveau`) ? Sépare échappement et couches.
- `EngineLabAbClipRenderer` : clips d'écoute ; chaque segment (ralenti, montée…)
  a son propre niveau. Une sonie unique sur ralenti + pleine charge rend le
  ralenti ~23 dB trop faible.
- `EngineLab.CombustionPhasing` : LPP, CA10/50/90, PMI sur un balayage de régime.
- `EngineLabAfterfireHarness` : forme du dégagement de chaleur de l'afterfire.
  Utiliser `--liftoff-rpm` (à 9 999 tr/min, le pompage masque tout).
- `EngineLabPhysicsPerfHarness --trace 1 --idle` : vrai ralenti.
  `--trace <bas régime>` sans `--idle` = pleine charge en sous-régime, pas un
  ralenti.
- `EngineLabIntakeDuctBench` : somme de contrôle bit à bit ; une optimisation
  qui la laisse inchangée ne touche pas à la physique.

Non déterministe :

- `EngineLabRealtimeBudgetHarness` (`--free-run` pour mesurer la capacité ; sans
  cette option le facteur plafonne à 1,0). **Comparaison valable uniquement
  dos à dos** dans la même heure (`git stash`, rebuild, mesure, `stash pop`),
  avec un moteur témoin que le changement ne peut pas affecter, en alternant la
  variante qui passe en premier et en ignorant un premier passage d'échauffement.
  Prendre le minimum de N passages, pas la moyenne.

## Pièges techniques vérifiés

- **Moteur silencieux ou commandes en retard = thread physique trop lent**, pas
  un problème audio. Lire le facteur temps réel affiché dans les diagnostics.
- **Un seul élément d'échappement court fixe le pas de temps de tout le réseau.**
  `ExhaustNetworkLayout::minimumCflLengthM()` donne le coût réel d'une géométrie.
- **`EngineRuntime` pèse ~7,4 Mo** : jamais deux sur la pile dans une même
  fonction (`0xC00000FD`, affiché par ctest comme `SegFault`). `std::make_unique`.
- **Les sélecteurs de moteur sont des sous-chaînes** (`Twin` en trouve trois).
- **Initialisation par position** : `CylinderState` est rempli positionnellement
  dans `EngineSimulator.cpp`. Ajouter les membres sous le commentaire marqueur et
  les affecter par nom. Un `grep` sur un nom de champ ne prouve pas qu'il est
  inutilisé.
- **Un réglage de haut niveau peut être masqué par banc ou par cylindre**
  (`activeCamshaft()` préfère l'arbre à cames du banc). Un moteur « single path »
  lit `config.exhaust`, pas `path.geometry`.
- **Une assertion sur une grandeur forcée à zéro sous un seuil ne prouve rien**
  tant qu'on n'a pas vérifié que le seuil est atteint.
- **Champs trompeurs** : `exhaustPressureKpa` est une crête, la contre-pression
  est `exhaustBackPressureKpa`. `residualGasFractionAtSpark` compte les seuls
  produits de combustion (× ~3,8 pour un taux de gaz résiduels de Heywood).
  `residualGasFraction` est instantané. `volumetricEfficiency` est un
  remplissage piégé ; `deliveredVolumetricEfficiency` est celui de Heywood.
- **Une ligne de balayage avec `ve / delivered_ve > 1,05` est contaminée**
  (limiteur ou raté). Les CSV antérieurs à l'arrêt à `0,95 × min(zone rouge,
  limiteur)` sont faux en haut de plage.
- **Toute moyenne sur une fenêtre qui mélange des niveaux très différents
  reflète la partie la plus forte** (sonie, seuils d'événements, moyennes de
  cycle). Idem pour une fenêtre fixe sur un signal non stabilisé : vérifier la
  dérive.
- **Les ralentis sont des attracteurs fragiles** : un changement algébriquement
  équivalent (un ULP) peut faire caler un moteur. Ne pas annuler l'optimisation :
  corriger ce qui rend le ralenti si sensible.
- **Phases qui bouclent** : un événement vilebrequin est une distance restante,
  pas un angle absolu. Un accumulateur ne doit pas recevoir
  `forwardPhaseDegrees` sans garde (au plus 180° par sous-pas).
- **Un biais de débit doit être un état, jamais une dérivée du débit** qu'il
  pilote (instabilité qui s'aggrave quand le pas diminue).
- **Barrière de threads** : compter les workers, pas les éléments (blocage de
  8 h 52 déjà vécu). Signature : un seul cœur à 100 %, les autres à l'arrêt.

## Réfuté — ne pas re-proposer sans preuve nouvelle

Le détail de chaque mesure est dans `docs/archive/`.

- Réécriture « conduit unifié » : le maillage de 300 mm porte bien la géométrie
  audible (le timbre bouge de 12 à 14 dB).
- Fork-join par cylindre : toutes les variantes threadées perdent.
- AVX / AVX2 : plus lent (latence de divisions dépendantes, pas de débit).
- Approfondir la chambre unique du silencieux : encoche de 16 dB sur le
  fondamental de l'EJ25.
- Accélérer la chimie de l'afterfire pour obtenir des salves : forme inchangée.
- « Le réseau 1-D ne peut pas porter le front raide d'une pétarade » : faux,
  le front audio monte en 0,2-0,8 ms. Le verrou est la bande passante (~3,8 kHz).
- Afterfire à carburant continu sous ~45 % de la demande normale : ne peut pas
  brûler (richesse sous la limite pauvre de 0,45). Seuls des paquets brûlent.
- Fermeture de variabilité cyclique par dilution : inerte (la variabilité existe
  déjà, 1,7-12,9 %).
- « Résidu piégé trop faible » : erreur d'unité, le résidu est sain.
- Quatre formulations d'inertance de runner : toutes instables ou déphasées.
- Cache de télémétrie pour accélérer l'écoute : n'aurait rien gagné.
- Amorcer la cellule de correction forte charge depuis celle du ralenti :
  sauve un démarrage 4 s après la mise en route mais fait caler 10 moteurs sur
  14 après 25 s de ralenti, et au coup de gaz (Hayabusa, CP3, Aircooled).
  Impulsion asynchrone au coup de pédale : ne sauve pas le 2JZ à 4 s, fait caler
  l'Aircooled à 8 s.

## Organisation des documents

- `VISION.md` : l'intention. `CLAUDE.md` (ce fichier) : la méthode.
  `AGENTS.md` renvoie ici.
- `docs/*.md` : comment fonctionne chaque partie (architecture, audio,
  échappement, ECU, DSL…). À tenir à jour quand le code change.
- `docs/journal.md` : résultats de mesure, entrées courtes.
- `docs/archive/` : tout l'historique (audits, validations, lots, plans). Lecture
  seule ; peut être périmé ou contradictoire.
