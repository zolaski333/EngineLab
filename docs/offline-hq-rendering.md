# Lot 5 — rendu audio hors ligne haute qualité (2026-07-29)

## Résultat livré

EngineLab possède désormais un seul chemin de rendu hors ligne partagé par
l'outil en ligne de commande et l'application :

```text
EngineSimulator -> publishAudioFrame -> RealtimeEngineAudio -> WAV
```

Il ne s'agit pas d'un synthétiseur parallèle. Le rendu HQ consomme les mêmes
événements d'allumage, échantillons de pression cylindre, graphes acoustiques,
sources forcées et contrôles de mix que l'application temps réel. Seules la
cadence de sortie et l'écriture disque changent.

Fonctions disponibles :

- 48, 96 ou 192 kHz ;
- WAV stéréo PCM 24 bits (code de format RIFF 1) ou float 32 bits IEEE
  (code 3) ;
- master plus six stems optionnels : combustion, échappement sec, retour IR,
  admission, suralimentation et mécanique ;
- scénario reproductible en JSON, versionné par `schema_version: 1` ;
- fondu de 20 ms au début et à la fin ;
- chargement des réponses impulsionnelles réellement écrites dans la
  configuration moteur, sans IR de remplacement cachée ;
- manifeste JSON avec format, nombre exact de frames, mix, fichiers, warnings
  et compteurs du chemin physique ;
- écriture d'abord en fichiers `.partial`, puis promotion seulement quand le
  rendu et les métadonnées sont complets ;
- annulation par callback et refus d'écraser un résultat existant par défaut.

L'API partagée est déclarée dans
`src/audio/include/enginelab/audio/OfflineAudioExporter.hpp`. L'outil est
`EngineLabOfflineAudioExporter`.

## Utilisation

Exemple recommandé pour travailler le son :

```powershell
.\EngineLabOfflineAudioExporter.exe `
  --engine K20 `
  --output .\exports\k20-96k `
  --sample-rate 96000 `
  --format pcm24 `
  --stems
```

Le profil par défaut est :

```text
démarrage -> ralenti -> montée à pleine charge -> limiteur -> décélération
```

Chaque export écrit sa copie canonique dans `scenario.json`. Elle peut être
modifiée puis rejouée :

```powershell
.\EngineLabOfflineAudioExporter.exe `
  --engine K20 `
  --output .\exports\k20-custom `
  --scenario .\mon-scenario.json `
  --sample-rate 192000 `
  --format float32 `
  --no-stems
```

Le schéma d'une étape contient :

```json
{
  "name": "rev_up",
  "duration_seconds": 3.2,
  "ignition": true,
  "starter": false,
  "governed": true,
  "throttle_start": 0.98,
  "throttle_end": 0.98,
  "load_start": 0.0,
  "load_end": 0.0,
  "target_rpm_start": 1600.0,
  "target_rpm_end": 7400.0
}
```

Avec `governed: true`, le banc hors ligne suit la cible RPM et ignore
`load_start/load_end`. Sans gouverneur, la charge est interpolée directement.
Les scénarios sont limités à 120 secondes pour rester dans un WAV RIFF 32 bits
et éviter une création accidentelle de plusieurs gigaoctets de stems.

## Preuves automatiques

Commande :

```powershell
cmake --build out\build\windows-vs2022 --config Release `
  --target EngineLabOfflineAudioExportTests EngineLabOfflineAudioExporter `
  --parallel 1 -- /nr:false /m:1 /v:minimal

.\out\build\windows-vs2022\tests\Release\
  EngineLabOfflineAudioExportTests.exe
```

Résultat :

```text
Offline audio export: PCM24 stems, float32 192 kHz, JSON scenario,
manifest and cancellation PASS
```

Le test ouvre les fichiers générés et vérifie, octet par octet :

- signature RIFF/WAVE ;
- code format 1 en PCM et 3 en float IEEE ;
- deux canaux ;
- fréquence et profondeur demandées ;
- taille du chunk data et taille physique du fichier ;
- master + six stems + métadonnées ;
- scénario JSON aller-retour ;
- manifeste indiquant le vrai chemin de rendu ;
- refus de 44,1 kHz ;
- annulation sans WAV final ;
- zéro troncature de délai et zéro télémétrie perdue.

## Deux rendus complets de contrôle

Ces chiffres prouvent ce run du 29 juillet 2026 sur la machine actuelle. Ce ne
sont ni une nouvelle référence absolue de performance, ni une comparaison de
qualité perceptive.

### K20, 96 kHz, PCM 24 bits, master + six stems

```text
988800 frames / 10.300000 s
peak 0.425443 / RMS 0.044991
physical exhaust active / compiled exhaust graph yes
delay truncations 0 / invalid boundaries 0
dropped firing events 0 / dropped pressure samples 0
```

### K20, 192 kHz, float 32 bits, master

```text
1977600 frames / 10.300000 s
peak 0.419566 / RMS 0.045786
physical exhaust active / compiled exhaust graph yes
delay truncations 0 / invalid boundaries 0
dropped firing events 0 / dropped pressure samples 0
```

Les manifests et WAV de preuve locaux se trouvent sous
`out/evidence/offline-hq-k20-96k-pcm24` et
`out/evidence/offline-hq-k20-192k-float`. `out/` reste volontairement hors Git :
les preuves versionnées sont le test, les commandes et ce relevé.

## Ce que cette étape ne prétend pas

- Monter à 192 kHz ne rend pas automatiquement un moteur plus réaliste. Cela
  fournit de la marge pour le travail spectral et évite que le débit audio
  temps réel bloque une écoute de laboratoire.
- Un test de structure WAV ne juge pas le timbre. Les stems permettent
  justement de diagnostiquer et d'écouter séparément les sources lors du lot
  atelier audio.
- La réussite hors ligne ne remplace pas la garde temps réel du catalogue.
  Celle-ci reste mesurée séparément avec
  `EngineLabRealtimeBudgetHarness --free-run`.
