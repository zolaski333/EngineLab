# Protocole d’écoute A/B en aveugle

Une mesure spectrale ne prouve pas qu’un son est réaliste. Ce protocole produit
des paires aveugles entre le vrai chemin audio temps réel d’EngineLab et des
enregistrements de moteurs réels, à durée et sonie égalisées.

## Instrument

La cible `EngineLabAbClipRenderer` est définie par
[`tools/AbClipRenderer.cpp`](../tools/AbClipRenderer.cpp).

```powershell
cmake --build out/build/windows-vs2022 --config Release `
  --target EngineLabAbClipRenderer -- /m:1 /nr:false
```

Pour chaque moteur, l’outil :

1. rend `RealtimeEngineAudio` avec la télémétrie de pression cylindre, le graphe
   d’échappement physique et le voicing par défaut ;
2. suit `démarrage -> ralenti tenu -> montée au rupteur -> coupure -> frein
   moteur` ;
3. décode la référence WAV, AIFF, FLAC ou OGG avec JUCE ;
4. la rééchantillonne à 48 kHz si nécessaire ;
5. sélectionne la fenêtre définie par le manifeste et rogne les deux côtés à la
   même durée ;
6. applique les mêmes fondus de 20 ms ;
7. égalise chaque côté par la sonie intégrée ITU-R BS.1770 (K-weighting, porte
   absolue -70 LUFS, porte relative -10 LU) ;
8. si un côté dépasserait 0,98 en crête, atténue **les deux côtés du même
   facteur**. L’égalité de sonie est donc conservée sans écrêtage ;
9. randomise la position A/B avec une graine reproductible.

La comparaison ne révèle donc ni le côté par sa durée, ni par son volume, ni par
un écrêtage ajouté au moment de la normalisation.

## Corpus réel CC0

Le manifeste versionné est
[`references/real-engine-audio/manifest.json`](../references/real-engine-audio/manifest.json).
Il fixe l’URL, l’auteur, la licence, le SHA-256, le début de fenêtre et la force
réelle de la correspondance. Les fichiers téléchargés restent ignorés par Git.

```powershell
powershell -ExecutionPolicy Bypass `
  -File references/real-engine-audio/fetch-corpus.ps1
```

| Clé | Enregistrement réel | Licence | Correspondance déclarée |
|---|---|---|---|
| 2JZ | Toyota Supra turbo sur banc, editboy23 | CC0 1.0 | proxy de plateforme, moteur exact non indiqué |
| LS3 | V8 au ralenti et coups de gaz, overmedium | CC0 1.0 | proxy d’architecture, véhicule inconnu |
| Hayabusa | Suzuki GSX1300 Hayabusa, Heigh-hoo | CC0 1.0 | plateforme exacte |
| Big Twin | Harley-Davidson, allencote | CC0 1.0 | proxy de famille, cylindrée inconnue |
| EJ25 | Subaru Impreza WRX 2003 turbo-back, ulose2piranha | CC0 1.0 | proxy boxer/turbo, moteur exact non affirmé |

Ces niveaux sont volontairement conservateurs. Une Supra non documentée comme
2JZ ou une WRX 2003 non documentée comme EJ25 ne devient pas une référence
exacte par simple ressemblance.

## Run complet actuel — 2026-07-29

Commande :

```powershell
out/build/windows-vs2022/tools/Release/EngineLabAbClipRenderer.exe `
  --output out/validation/listening-current-2026-07-29 `
  --engines "2JZ,LS3,Hayabusa,Big Twin,EJ25" `
  --reference-manifest references/real-engine-audio/manifest.json `
  --require-references `
  --seed 20260729
```

Résultat :

| Paire | Durée | EngineLab LUFS | Référence LUFS | Pic max | État |
|---|---:|---:|---:|---:|---|
| 2JZ / Supra | 10,300 s | -20,000 | -20,000 | 0,710 | appariée |
| LS3 / V8 | 8,530 s | -20,000 | -20,000 | 0,467 | appariée, ref. 44,1 -> 48 kHz |
| Hayabusa | 10,300 s | -20,000 | -20,000 | 0,601 | appariée, ref. 44,1 -> 48 kHz |
| Big Twin / Harley | 10,300 s | -20,000 | -20,000 | 0,611 | appariée, ref. 44,1 -> 48 kHz |
| EJ25 / WRX | 10,300 s | -21,649 | -21,649 | 0,980 | appariée, plafond de crête commun |

Pour EJ25, la référence contient une crête forte par rapport à sa sonie moyenne.
Le plafond commun a donc abaissé les deux côtés de 1,649 dB. Ce n’est ni une
erreur ni une faveur accordée à un côté : les deux sorties restent égales à
moins de 0,000001 LU dans la clé mesurée.

Les dix WAV ont été écrits. Pour chaque paire, A et B ont exactement la même
taille. La clé complète se trouve hors du dossier remis aux auditeurs :
`out/validation/listening-current-2026-07-29/listening-key.json`.

## Disposition et clé

- Les fichiers aveugles sont `clips/pair_N_A.wav` et
  `clips/pair_N_B.wav`.
- La position EngineLab est tirée avec `--seed`. Si tous les moteurs tombent du
  même côté, le tirage est recommencé.
- `listening-key.json` reste hors de `clips/`. Il contient le nom du moteur, les
  côtés, la durée, les LUFS, les crêtes, la provenance, la licence, le SHA-256,
  le taux d’échantillonnage original et la qualité de correspondance.
- `--require-references` échoue avant tout rendu si un fichier manque ou ne peut
  pas être décodé.
- Une référence donnée manuellement avec `--ref clé=chemin` est marquée
  `MANUAL / UNVERIFIED` dans la clé : l’outil ne fabrique pas une provenance.

## Procédure humaine

1. Utiliser le même casque neutre ou les mêmes moniteurs et ne plus toucher au
   volume.
2. Donner uniquement le dossier `clips/` aux auditeurs.
3. Pour chaque paire, noter séparément :
   - réalisme A et B de 1 à 5 ;
   - préférence A et B de 1 à 5 ;
   - choix forcé « le plus réaliste » A/B ;
   - choix forcé « préféré » A/B ;
   - commentaire libre.
4. Utiliser idéalement au moins cinq auditeurs et une graine différente par
   session.
5. Dépouiller ensuite avec la clé. Un résultat qualitatif n’est annoncé qu’après
   ces écoutes ; le build et les métriques seules ne signifient pas « meilleur
   son ».

## Limites restantes

- Les gestes des enregistrements réels ne suivent pas exactement la trajectoire
  EngineLab. On juge le caractère moteur, pas un alignement cycle par cycle.
- Quatre références sur cinq sont des proxys explicitement étiquetés. Il faudra
  des prises documentées du véhicule/moteur exact pour une validation
  modèle-par-modèle.
- Les préécoutes HQ sont des transcodages OGG des prises citées. Pour une étude
  finale, remplacer les chemins par les WAV/AIFF originaux téléchargés et
  conserver leur nouveau SHA-256.
- Aucune préférence humaine n’est encore enregistrée dans le dépôt. Le corpus
  rend enfin le test possible, il ne remplace pas les auditeurs.
- `es2d` reste inutilisable localement parce que ses sous-modules sont vides ;
  aucune comparaison exécutable contre lui n’est revendiquée.
