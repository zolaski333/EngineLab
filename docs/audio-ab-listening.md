# Protocole d'écoute A/B en aveugle

La mesure objective (`docs/audio-ab-comparison.md`) ne prouve pas « sonne mieux ».
Ce document décrit un protocole d'écoute **en aveugle** et **à sonie égalisée**,
et l'outil qui produit les extraits appariés. Cette procédure ne change pas le
moteur audio : elle fournit l'instrument de jugement humain.

## Outil : `EngineLabAbClipRenderer`

Source : [`tools/AbClipRenderer.cpp`](../tools/AbClipRenderer.cpp).

```
cmake --build out/build/windows-vs2022 --config Release --target EngineLabAbClipRenderer
./out/build/windows-vs2022/tools/Release/EngineLabAbClipRenderer.exe --output listening-test
```

Ce qu'il fait, pour les trois archétypes présents des deux côtés (2JZ I6, LS V8,
Hayabusa I4) :

- rend le **vrai chemin temps réel** (`RealtimeEngineAudio`), voicing par défaut
  (`convolution = 0.45`), sans rien changer au moteur audio ;
- suit un **profil RPM unique et listenable** : ralenti tenu → montée en régime
  jusqu'au **rupteur** → lâcher de gaz et décélération en frein moteur. Le chemin
  hors-ligne n'a pas de boucle de ralenti ; un frein dyno gouverné impose la
  trajectoire (cf. `AudioAbHarness`) ;
- écrit un **WAV 48 kHz, 32 bits flottant, stéréo** (~10,3 s) par côté ;
- **égalise la sonie** de chaque extrait à une cible commune par une mesure de
  **loudness intégrée ITU-R BS.1770** (K-weighting + porte absolue −70 LUFS +
  porte relative −10 LU). Défaut : **−20 LUFS**. C'est le garde-fou central :
  sans cela, l'extrait le plus fort passe pour « meilleur ».

Options : `--target-lufs <v>` (cible de sonie), `--seed <n>` (tirage A/B),
`--ref <archétype>=<chemin.wav>` (enregistrement de référence, voir plus bas),
`--output <dir>`.

### Déterminisme et sonie vérifiés

Sur un run de référence (cible −20 LUFS) :

| Paire | Moteur | LUFS avant | LUFS après | Pic après |
|---|---|---|---|---|
| 1 | 2JZ I6 turbo | −14,87 | −20,00 | 0,55 |
| 2 | LS V8 | −21,02 | −20,00 | 0,63 |
| 3 | Hayabusa I4 | −20,40 | −20,00 | 0,45 |

La normalisation converge exactement à la cible (mesure → gain → re-mesure), et
les pics restent sous pleine échelle (aucun écrêtage introduit par la
normalisation). Le rendu est déterministe.

## Disposition en aveugle et clé

- Les extraits sont écrits dans `<output>/clips/` sous des noms **neutres** :
  `pair_1_A.wav`, `pair_1_B.wav`, `pair_2_A.wav`, …
- Pour chaque paire, le côté (A ou B) occupé par EngineLab est **tiré au sort**
  (graine reproductible via `--seed`). Le tirage évite le cas « tous du même
  côté » (deviner un extrait révélerait tous les autres).
- La **clé de réponse** `<output>/listening-key.json` est écrite **hors** du
  dossier `clips/`. **L'expérimentateur la garde à l'écart des auditeurs.** Elle
  indique, par paire : le moteur, le côté EngineLab, le côté référence, la source
  de référence et les LUFS après normalisation.

Exemple de clé :

```json
{ "target_lufs": -20, "seed": 20260718,
  "profile": "idle-hold -> rev-up into limiter -> throttle-off decel",
  "pairs": [ { "pair": 1, "engine": "2JZ-GTE-like 3.0 I6 Turbo",
               "enginelab_side": "A", "reference_side": "B",
               "reference_present": false } ] }
```

## Le côté « référence » : es2d ou enregistrements réels

**es2d n'a pas pu être compilé** localement (les cinq sous-modules
`delta-studio`, `piranha`, `simple-2d-constraint-solver`, `direct-to-video`,
`csv-io` sont vides). Aucun extrait es2d n'est donc produit. Le chemin de sortie
audio d'es2d existe pourtant (`Synthesizer::readAudioOutput` → PCM16) : si les
sous-modules sont un jour restaurés et le projet construit, un petit driver
appelant `Simulator::readAudioOutput` vers un WAV fournirait le côté B.

En attendant, le côté référence est un **emplacement documenté** à remplir avec
des **enregistrements de moteurs réels libres de droits** :

1. Récupérer un enregistrement du bon archétype (ex. un vrai 2JZ) avec un geste
   comparable (ralenti, montée, décélération), sous licence **CC0 / domaine
   public** (p. ex. sons CC0 de banques libres). Vérifier la licence.
2. Le **rééchantillonner à 48 kHz** (l'outil ne rééchantillonne pas ; la mesure
   BS.1770 et la lecture supposent 48 kHz — un fichier à 44,1 kHz déclenche un
   avertissement et fausserait la sonie).
3. Le passer en référence :
   ```
   ...EngineLabAbClipRenderer.exe --output listening-test \
       --ref 2JZ=refs/2jz_real_48k.wav --ref LS3=refs/ls_real_48k.wav \
       --ref Hayabusa=refs/busa_real_48k.wav
   ```
   L'outil charge la référence (WAV PCM16 ou flottant, mono/stéréo), l'**égalise
   à la même cible LUFS**, et la place sur le côté opposé à EngineLab.

Sans `--ref`, le côté référence reste un emplacement vide (`reference_present:
false`) : la paire n'est pas écoutable en A/B tant qu'un fichier n'est pas
déposé. Ne jamais comparer un extrait EngineLab à un extrait de sonie non
égalisée.

## Procédure d'écoute

1. **Matériel** : casque neutre ou moniteurs, même volume système pour toutes les
   paires. Ne pas ajuster le volume entre A et B.
2. **Aveugle** : l'auditeur ne voit pas la clé. Il écoute A puis B (autant de
   fois que voulu) pour chaque paire.
3. **Jugement**, par paire, sur deux axes (échelle 1–5) :
   - *Réalisme* : « lequel ressemble le plus à un vrai moteur ? »
   - *Préférence* : « lequel préférez-vous ? »
   plus un choix forcé A/B pour chaque axe et un champ commentaire libre.
4. **Plusieurs auditeurs** : idéalement ≥ 5, chacun avec une graine `--seed`
   différente pour re-randomiser les côtés.
5. **Dépouillement** : mapper les réponses via `listening-key.json`, compter les
   préférences EngineLab vs référence par moteur et agrégées. Un écart n'est
   probant que s'il dépasse le hasard (test binomial simple sur les choix forcés).

### Feuille de score (modèle)

| Paire | Réalisme A (1–5) | Réalisme B (1–5) | Plus réaliste (A/B) | Préféré (A/B) | Commentaire |
|---|---|---|---|---|---|
| 1 | | | | | |
| 2 | | | | | |
| 3 | | | | | |

## Limites

- Le profil RPM est imposé par un frein gouverné (pas un vrai ralenti trottoir) ;
  identique pour tous les extraits EngineLab, mais un enregistrement réel n'aura
  jamais exactement la même trajectoire — comparer le **caractère**, pas
  l'alignement temporel.
- L'outil ne rééchantillonne pas les références ; fournir du 48 kHz.
- La sonie est égalisée (BS.1770 intégré) mais pas le spectre ni la dynamique :
  c'est voulu, ce sont précisément les dimensions jugées à l'oreille.
- Sans enregistrements de référence déposés, ce protocole fournit l'instrument et
  les extraits EngineLab normalisés, pas encore un verdict d'écoute.
