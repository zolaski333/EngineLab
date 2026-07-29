# Corpus de références moteur réelles

Ce dossier décrit le corpus employé par `EngineLabAbClipRenderer`. Les cinq
sources sont de vrais enregistrements de terrain publiés sous **CC0 1.0**. Les
fichiers audio ne sont pas versionnés : le manifeste fixe leur URL de préécoute
HQ, leur SHA-256, leur auteur, leur licence et le niveau réel de correspondance
avec le moteur EngineLab.

Le niveau `exact-platform` signifie que le véhicule est explicitement identifié.
`platform-proxy`, `family-proxy` et `architecture-proxy` sont volontairement
moins forts : ils servent à juger un caractère sonore, pas à revendiquer une
corrélation exacte.

## Récupération vérifiée

Depuis la racine du dépôt :

```powershell
powershell -ExecutionPolicy Bypass -File references/real-engine-audio/fetch-corpus.ps1
```

Le script ne conserve un fichier que si son SHA-256 correspond au manifeste.
Les fichiers sont écrits sous `files/`, dossier ignoré par Git.

## Rendu A/B complet

```powershell
out/build/windows-vs2022/tools/Release/EngineLabAbClipRenderer.exe `
  --output out/validation/listening-current `
  --engines "2JZ,LS3,Hayabusa,Big Twin,EJ25" `
  --reference-manifest references/real-engine-audio/manifest.json `
  --require-references
```

L’outil décode WAV/AIFF/FLAC/OGG, rééchantillonne automatiquement à 48 kHz,
rogne les deux côtés à la même durée, applique les mêmes fondus et abaisse les
deux côtés ensemble si l’un dépasse le plafond de crête. La clé JSON conserve
la provenance, la licence, le taux d’échantillonnage original et le niveau de
correspondance de chaque référence.
