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
Son schéma 2 rend obligatoires la provenance, les droits de redistribution, le
SHA-256, la nature de la prise, les conditions de fonctionnement, le statut des
RPM, le microphone, la qualité de l’asset et la force réelle de la
correspondance. Une donnée inconnue reste littéralement `unknown` ou `null` :
le corpus ne transforme pas une supposition en mesure.

Le script ne télécharge automatiquement que les entrées `redistributable`,
refuse les chemins absolus ou sortant du corpus, puis vérifie le SHA-256 avant
de remplacer le fichier final. Les assets restent ignorés par Git.

```powershell
powershell -ExecutionPolicy Bypass `
  -File references/real-engine-audio/fetch-corpus.ps1

out/build/windows-vs2022/tools/Release/EngineLabAbClipRenderer.exe `
  --reference-manifest references/real-engine-audio/manifest.json `
  --validate-manifest
```

La seconde commande relit les dix fichiers, recalcule leur SHA-256 et les
décode réellement avec JUCE. Le renderer répète cette vérification d’intégrité
avant chaque rendu A/B.

| Clé | Enregistrement réel | Conditions connues | Correspondance déclarée |
|---|---|---|---|
| 2JZ | Toyota Supra turbo sur banc, editboy23 | sweep chargé | proxy de plateforme, moteur exact non indiqué |
| LS3 | V8 inconnu en binaural, overmedium | ralenti, coups de gaz | proxy d’architecture |
| Hayabusa | Suzuki GSX1300, Heigh-hoo | démarrage, ralenti, départ | plateforme exacte |
| Big Twin | Harley-Davidson, allencote | démarrage, ralenti, coups de gaz | proxy de famille |
| EJ25 | Subaru Impreza WRX 2003 turbo-back, ulose2piranha | ralenti, coups de gaz | proxy boxer/turbo |
| K20 | Honda Civic 2012, thepodcastdoctor | démarrage, ralenti, coups de gaz | proxy I4 atmosphérique, moteur non identifié |
| Merlin | Merlin de Spitfire sur support statique, squashy555 | marche statique sans hélice | plateforme exacte, charge non appariée |
| Aircooled | Porsche 911 en rue, mharo | passage routier | proxy de plateforme, génération inconnue |
| CP3 | Yamaha MT-09/FZ-09 sur circuit, richwise | passage en charge | plateforme exacte |
| Radial | biplan à moteur radial, archive craigsmith | passage en vol | proxy radial d’archive, cylindres inconnus |

Ces niveaux sont volontairement conservateurs. Une Supra non documentée comme
2JZ, une WRX 2003 non documentée comme EJ25 ou une 911 de génération inconnue
ne devient pas une référence exacte par simple ressemblance.

## Run complet actuel — 2026-07-29

Commande :

```powershell
out/build/windows-vs2022/tools/Release/EngineLabAbClipRenderer.exe `
  --output out/validation/listening-corpus-v2-2026-07-29 `
  --engines "2JZ,LS3,Hayabusa,Big Twin,EJ25,K20,Merlin,Aircooled,CP3,Radial" `
  --reference-manifest references/real-engine-audio/manifest.json `
  --require-references `
  --seed 20260729
```

Résultat :

| Paire | Durée | EngineLab LUFS | Référence LUFS | Pic max | Correspondance |
|---|---:|---:|---:|---:|---|
| 2JZ / Supra | 10,300 s | -20,000 | -20,000 | 0,708 | plateforme proxy |
| LS3 / V8 | 8,530 s | -20,000 | -20,000 | 0,467 | architecture proxy |
| Hayabusa | 10,300 s | -20,000 | -20,000 | 0,601 | plateforme exacte |
| Big Twin / Harley | 10,300 s | -20,000 | -20,000 | 0,743 | famille proxy |
| EJ25 / WRX | 10,300 s | -21,649 | -21,649 | 0,980 | famille proxy |
| K20 / Civic | 10,300 s | -20,000 | -20,000 | 0,626 | architecture proxy |
| Merlin | 10,300 s | -20,000 | -20,000 | 0,284 | plateforme exacte |
| Flat-6 / 911 | 10,004 s | -20,000 | -20,000 | 0,674 | plateforme proxy |
| CP3 / MT-09 | 10,300 s | -20,441 | -20,441 | 0,980 | plateforme exacte |
| Radial / biplan | 10,300 s | -20,000 | -20,000 | 0,807 | architecture proxy |

EJ25 et CP3 rencontrent le plafond de crête commun. Les deux côtés de chaque
paire sont alors atténués du même facteur : la sonie reste égale, sans favoriser
EngineLab ni la référence.

Les vingt WAV ont été écrits. Pour chaque paire, A et B ont exactement la même
taille. La clé complète, incluant les métadonnées du schéma 2, se trouve hors du
dossier remis aux auditeurs :
`out/validation/listening-corpus-v2-2026-07-29/listening-key.json`.

## Disposition et clé

- Les fichiers aveugles sont `clips/pair_N_A.wav` et
  `clips/pair_N_B.wav`.
- La position EngineLab est tirée avec `--seed`. Si tous les moteurs tombent du
  même côté, le tirage est recommencé.
- `listening-key.json` reste hors de `clips/`. Il contient le nom du moteur, les
  côtés, la durée, les LUFS, les crêtes, la provenance, la licence, le SHA-256,
  le taux d’échantillonnage original, les conditions, le statut des RPM, le
  microphone, la qualité de l’asset et la qualité de correspondance.
- `--require-references` échoue avant tout rendu si un fichier manque, si son
  SHA-256 diffère ou s’il ne peut pas être décodé.
- `--validate-manifest` vérifie les métadonnées, l’intégrité et le décodage de
  tout le corpus sans lancer la simulation.
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

## Feuilles de réponse et dépouillement

`scripts/listening.py` (stdlib seule) fait les deux bouts.

```powershell
# 1. fabriquer les feuilles vierges (ne lit jamais la clé)
python scripts/listening.py sheets --pack out/validation/listening-pilot-2026-07-29 --listeners 5

# 2. après remplissage, dépouiller
python scripts/listening.py report --pack out/validation/listening-pilot-2026-07-29
```

`sheets` écrit `responses/listener_NN.csv`, une ligne par paire, colonnes
`pair, realism_A, realism_B, preference_A, preference_B, most_realistic,
preferred, comment`. Les consignes destinées aux auditeurs sont dans
`clips/CONSIGNES.md`, donc elles voyagent avec les clips ; la clé reste à la
racine du pack et ne doit pas être distribuée.

`report` relit `listening-key.json`, retourne chaque réponse A/B vers
« EngineLab » ou « référence » grâce à `enginelab_side`, et écrit
`listening-report.md` et `listening-report.json`. Il **refuse de noter** (code 2)
une feuille incomplète, hors échelle 1-5 ou dont un choix forcé n'est pas `A`/`B`,
en listant les lignes fautives ; `--allow-incomplete` note les lignes valides et
déclare combien ont été écartées. Une réponse manquante n'est jamais devinée.

Le rapport donne, par question et par famille : le taux de choix d'EngineLab, un
**intervalle de confiance de Wilson à 95 %** (correct à petit n, contrairement à
l'approximation normale — c'est exactement le régime d'un pilote à cinq
auditeurs), un **test des signes binomial exact**, les moyennes de réalisme et de
préférence des deux côtés, et tous les commentaires regroupés par paire.

## Ce que ce test décide — et ce qu'il ne décide pas

Il faut distinguer deux comparaisons que le plan confond facilement.

| | Contre quoi | À quoi ça sert | Seuil |
|---|---|---|---|
| **A. Diagnostic** | EngineLab contre **enregistrement réel** | classer les familles par déficit, pour savoir où porter l'effort | pas de seuil de réussite |
| **B. Non-régression** | candidate contre **baseline EngineLab** | accepter ou refuser un changement de timbre | ≥ 65 % de préférence |

Le pack actuel est le cas **A**. Le seuil de 65 % du plan appartient au cas
**B** et **ne s'y applique pas** : sept références sur dix sont des proxys, et
aucune n'a de trajectoire de régime ni de position micro appariées. Perdre
contre un vrai enregistrement est l'attendu, pas un échec.

Ce qu'on en tire légitimement :

1. **Le classement par écart de réalisme.** C'est la sortie principale. La
   famille la plus déficitaire est la prochaine cible de travail.
2. **L'écart réalisme / préférence.** Une famille jugée peu réaliste mais bien
   aimée n'a pas le même problème qu'une famille jugée fausse *et* déplaisante.
3. **Les commentaires, regroupés par couche.** « trop lisse » et « métallique »
   ne désignent pas le même étage de la chaîne : les stems du lot 2 permettent
   ensuite de vérifier lequel.
4. **Un point de départ daté**, auquel une version future se compare.

Ce qu'on n'en tire pas :

- aucune affirmation de supériorité sur ES2D — aucune comparaison exécutable
  n'existe, ses sous-modules sont vides ;
- aucun classement entre deux familles dont les intervalles de confiance se
  recouvrent : à cinq auditeurs, un écart de 1/5 sur un choix forcé n'est pas
  un résultat ;
- aucune conclusion sur une famille dont la référence est un proxy faible, si
  le commentaire ne dit pas *pourquoi*.

À cinq auditeurs et dix paires, un effet global se voit s'il est franc ; une
différence par famille reste indicative. C'est un pilote — il sert à orienter le
travail et à roder le protocole, pas à publier un chiffre.

## Limites restantes

- Les gestes des enregistrements réels ne suivent pas exactement la trajectoire
  EngineLab. On juge le caractère moteur, pas un alignement cycle par cycle.
- Sept références sur dix sont des proxys explicitement étiquetés. Même les
  trois plateformes exactes n’ont ni trajectoire de RPM ni position micro
  appariées. Il faudra des prises instrumentées pour une validation
  modèle-par-modèle.
- Les préécoutes HQ sont des transcodages OGG ou MP3 des prises citées. Pour une
  étude finale, remplacer les chemins par les WAV/AIFF originaux téléchargés,
  renseigner les paramètres de prise puis conserver leur nouveau SHA-256.
- Aucune préférence humaine n’est encore enregistrée dans le dépôt. Le corpus
  rend enfin le test possible, il ne remplace pas les auditeurs.
- `es2d` reste inutilisable localement parce que ses sous-modules sont vides ;
  aucune comparaison exécutable contre lui n’est revendiquée.
