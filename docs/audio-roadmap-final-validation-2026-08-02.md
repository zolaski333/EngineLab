# Validation finale de la feuille de route audio — 2 août 2026

## Verdict

Le lot est livrable. Sur le PC fixe i5-11600, le catalogue Release exact du
commit de code `da33134` passe **16/16** avec
`EngineLabRealtimeBudgetHarness --free-run`, sans overrun ni point invalide.
Le pire cas est le Merlin à `1,167×`, soit environ **14,3 % du budget restant
avant l'échéance temps réel**. Il n'y a donc aucune justification mesurée pour
simplifier davantage l'admission dans ce lot.

La suite Release exacte passe **31/31 en 642,39 s**. Les changements audio sont
mesurables, mais ces nombres ne prouvent pas que le timbre est bon : la prochaine
étape reste une écoute humaine A/B aveugle avec des références appariées.

## Ce qui est livré

- 16 profils de voicing YAML, rechargeables par moteur, avec retour neutre dans
  AUDIO HQ et conservation des paramètres cachés du mix lors des A/B ;
- graphe acoustique d'échappement complet confirmé : branches, troncs partagés,
  changements de section, état de gaz par conduit, sorties et observateurs
  spatialisés ; aucune seconde implémentation parallèle n'a été ajoutée ;
- stratégie d'afterfire de décélération conditionnée par l'ECU : armement après
  demande conducteur, seuil RPM, papillon fermé, fraction de carburant bornée,
  spark cut explicite et réaction chimique physique en aval ;
- rupteur humide opt-in conservé par un signal ECU explicite, sans élargir
  l'injection à tous les cycles sans étincelle ;
- deux stems diagnostiques HQ : onde de pression
  (blowdown/réflexions/afterfire) et jet de sortie ;
- contrôle produit dans AUDIO HQ et moteur laboratoire
  `Audio Physics Lab 689 Twin` pour essayer la chaîne sans modifier les moteurs
  historiques par défaut.

## Table temps réel finale

Commande exécutée seule sur la machine :

```powershell
out/build/windows-vs2022/tools/Release/EngineLabRealtimeBudgetHarness.exe `
  --catalog-root . --free-run --rpm 7000 --seconds 6
```

`factor` est le nombre de secondes simulées produites par seconde murale. `1,0`
est la limite ; les chiffres absolus d'une autre machine ou d'une autre session
ne sont pas une référence.

| Moteur | Workers | RPM tenu | Facteur | Overruns | BP |
|---|---:|---:|---:|---:|:---:|
| K20A-like 2.0 I4 VTEC | 2 | 7003 | 1,531 | 0 | non |
| 2JZ-GTE-like 3.0 I6 Turbo | 2 | 6650 | 1,547 | 0 | oui |
| LS3-like 6.2 Crossplane V8 | 2 | 6271 | 1,233 | 0 | non |
| EJ25-like 2.5 Flat-4 Turbo | 2 | 6460 | 2,061 | 0 | oui |
| Audi I5-like 2.5 Turbo | 2 | 6747 | 1,791 | 0 | oui |
| Hayabusa-like 1.3 I4 | 2 | 7004 | 1,910 | 0 | non |
| Big Twin-like 1.9 V2 | 0 | 5319 | 3,461 | 0 | non |
| Merlin-like 19.8 V12 Scaled | 3 | 3042 | **1,167** | 0 | non |
| Aircooled-like 3.6 Flat-6 | 2 | 7000 | 1,406 | 0 | non |
| Radial-like 6.5 R5 | 2 | 2277 | 2,562 | 0 | non |
| Yamaha CP2 MT-07-like 689 Twin | 0 | 6999 | 3,079 | 0 | non |
| Yamaha CP3 MT-09-like 890 Triple | 2 | 7002 | 2,333 | 0 | non |
| Yamaha CP4 MT-10-like 998 Crossplane I4 | 2 | 7001 | 1,899 | 0 | non |
| VW EA288-like 2.0 TDI I4 | 2 | 4750 | 2,508 | 0 | non |
| MT-07-like 689 Twin Full System | 0 | 7004 | 3,188 | 0 | non |
| Audio Physics Lab 689 Twin | 0 | 7002 | 3,129 | 0 | non |

Les alertes de contre-pression 2JZ, EJ25 et Audi sont conservées : elles sont
des diagnostics physiques à expliquer, pas des erreurs à masquer pour rendre la
table verte.

## A/B et régressions découverts pendant la clôture

### Démarrage Big Twin contre injection trop large

La première table finale a été rejetée par le harness : Big Twin à `0 rpm`,
`INVALID: hold not stable within 60 sim s`. Le contrôle isolé a reproduit
l'échec. Le problème venait du contournement de la porte d'injection par tout
cycle `fuelEnabled`, même sans combustion ; les coupures ordinaires de
démarrage mouillaient alors le moteur.

Le correctif autorise le contournement seulement pour deux états ECU explicites :
afterfire de décélération ou rupteur humide opt-in. Résultat isolé après
correctif : Big Twin tenu à `5320 rpm`, facteur `3,351`, zéro overrun. La table
finale ci-dessus confirme `5319 rpm` et `3,461`. `EngineLab.Core` contient
désormais la régression catalogue qui exige l'accrochage du Big Twin.

### Référence dyno Audi

La première passe 31 tests avait exposé un point Audi à `424,088 Nm`, soit
`-15,182 %` face à la référence et `0,182` point hors tolérance. Un essai de
turbulence de chambre `1,50 -> 1,52` a été rejeté car il baissait encore le
couple (`422,818 Nm`). Un degré d'avance supplémentaire au point de 3400 rpm a
donné `437,036 Nm` (`-12,593 %`) et `254,537 kW` (`-13,423 %`). La référence
constructeur repasse donc sans relâcher la tolérance ; la suite finale valide
les 24 points.

## Preuves audio et physiques

### Voicing catalogue

Les 16 rendus A/B catalogue sont non identiques. Le delta RMS moyen mesuré est
`0,006794`, avec déplacement spectral, et le test `EngineLab.AudioVoicing`
verrouille le chargement YAML ainsi que le retour neutre.

### Afterfire de décélération

Le contrôle direct du moteur laboratoire mesure `121,608 kW` de pic et
`6479,05 mg` brûlés dans l'échappement sur `267` frames d'overrun actif. L'A/B
produit un delta RMS `0,0627879`, un delta peak `0,82579` et une plage de cycles
`0,858323..1,14009`. La fraction demandée reste exactement bornée ; l'ECU coupe
l'étincelle, mais la chimie aval conserve le dernier mot selon oxygène et
température.

### Décomposition pression / jet

Les stems diagnostiques satisfont :

```text
stem_exhaust_pressure_wave + stem_exhaust_jet == exhaust_dry
erreur float maximale : 2,32831e-10
erreur PCM24 maximale : 1,19209e-07
```

Ils ne sont pas sommés une seconde fois dans le premaster. Les six stems de mix
historiques continuent donc à reconstruire le master, tandis que ces deux stems
servent uniquement au diagnostic de la source d'échappement.

### Topologie acoustique haute bande

`EngineLab.RealtimeRegression` vérifie le DAG compilé, les troncs communs, le
pas de section analytique, les milieux par conduit et les sorties spatialisées.
La documentation qui parlait d'un unique guide par chemin était périmée et a
été corrigée ; aucune nouvelle approximation à 2190 Hz n'a été introduite dans
ce lot.

## Validation logicielle exacte

- build MSVC Release complet : réussi ;
- `ctest --test-dir out/build/windows-vs2022 -C Release --output-on-failure` :
  **31/31**, zéro échec, `642,39 s` ;
- tests ciblés après séparation des états humides : Core, EcuCalibration,
  OfflineAudioExport et AudioRender, **4/4 en 156,35 s** ;
- `EngineLab.CatalogReference` : 24/24 points constructeur dans la tolérance ;
- mesure finale : 16/16 valides, zéro overrun, zéro point sous le temps réel ;
- `git diff --check` : propre avant le commit documentaire.

## Commits du lot

| Commit | Objet |
|---|---|
| `6f78c73` | profils de voicing catalogue audibles |
| `906b0f7` | réconciliation de la topologie acoustique complète |
| `d04b0bd` | stratégie ECU d'afterfire conditionnée |
| `e0688dd` | exposition produit de l'afterfire de lever |
| `94ab9e4` | stems pression et jet séparés |
| `08b4976` | marge dyno Audi restaurée par A/B |
| `0bbadd4` | injection afterfire exclue des coupures de démarrage |
| `da33134` | rupteur humide explicite préservé |

## Limites honnêtes et suite

- Aucune écoute humaine aveugle n'a été effectuée par l'agent ; les différences
  de signal sont prouvées, la qualité subjective ne l'est pas.
- Les valeurs du moteur laboratoire sont des valeurs de démonstration estimées,
  pas une calibration Yamaha mesurée.
- Les références micro réelles ne sont pas encore appariées par moteur,
  condition, position, charge et chaîne de prise de son.
- La prochaine priorité est une écoute A/B des stems pression/jet et des profils
  catalogue, puis l'ajustement du voicing à partir de références réelles. Ne pas
  relancer une optimisation admission/pool tant qu'une nouvelle table locale ne
  montre pas un déficit.

