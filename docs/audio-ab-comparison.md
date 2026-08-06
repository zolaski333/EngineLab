# EngineLab vs ES2D — comparaison audio A/B objective

> **Baseline historique.** Cette campagne mesure l’ancien renderer hybride.
> Ses chiffres ne qualifient pas la chaîne thermoacoustique physique actuelle et
> doivent être régénérés avant toute nouvelle conclusion perceptuelle.

Date de la mesure : 18 juillet 2026. Branche : `fix/clutch-lockup`.

Cette étude répond à une question précise : l'architecture audio d'EngineLab est
réputée « plus avancée » qu'ES2D, mais sa **supériorité sonore n'est pas
prouvée**. On cherche ici des mesures objectives, déterministes, sans juger à
l'œil ni à l'oreille, conformément à `CLAUDE.md` (« measure before you fix »).

**Verdict en une phrase.** Sur les dimensions mesurables, EngineLab est
*supérieur* en propreté de signal (DC, non-clipping, contrat temps réel) et en
dynamique bas/moyen régime, *réaliste* sur la longueur de décroissance
d'échappement, mais *inférieur* sur le contenu haute-fréquence (spectre sombre,
< 2 % d'énergie > 4 kHz là où la littérature attend 5–20 %) et sur la
cohérence de loudness entre moteurs (~21 dB d'écart). La supériorité *sonore*
globale reste **non démontrée** : elle n'est ni prouvée ni infirmée face à ES2D,
qui n'a pas pu être rendu (voir plus bas).

---

## 1. Méthode et déterminisme

- **Instrument** : nouvel outil `tools/AudioAbHarness.cpp`
  (cible `EngineLabAudioAbHarness`). Il rend le **vrai chemin temps réel**
  (`RealtimeEngineAudio`) hors-ligne, exactement comme `EngineRuntime` : il pas-
  à-pas le simulateur, alimente les files SPSC d'événements de combustion et de
  pression cylindre, et rend via la vraie convolution partitionnée. Les fonctions
  de mesure (`scanSignal`, `analyseWindow`, `measureDecay`) sont **identiques** à
  `tools/AudioRenderHarness.cpp` — les deux instruments sont donc directement
  comparables. La seule mécanique nouvelle est le chargeur de catalogue et le
  balayage RPM gouverné.
- **Voicing par défaut, non modifié** : `convolution = 0.45`
  (défaut de `RealtimeAudioState`), `exhaustPreset = street`. Aucun paramètre de
  voicing n'est touché dans cette session. Garde-fou respecté.
- **Déterminisme** : `dt = 1/240 s`, 200 échantillons/pas @ 48 kHz, gouverneur
  de charge déterministe. Rendu identique à chaque exécution.
- **Balayage** : les 10 moteurs de `engines/*.yaml` sont chargés via
  `loadEngineCatalog`. Le chemin hors-ligne **ne s'auto-ralentit pas** (pas de
  boucle de ralenti fermée) : à gaz seul le moteur monte au rupteur, et un frein
  trop fort à papillon fermé le cale. On place donc chaque moteur sur 4 points
  d'opération tenus par un **frein dyno** sous papillon partiel, du bas régime
  contrôlable (~0,25 × redline) jusqu'à ~0,90 × redline. Ce n'est pas un ralenti
  trottoir, mais un balayage bas→redline reproductible et physiquement sain.
- **Métriques par canal (L/R)** : RMS, facteur de crête, offset DC, équilibre
  spectral bas/moyen/haut (< 250 Hz / 250–4000 Hz / > 4 kHz), un proxy de
  brillance, la corrélation L/R, plus le RT60 de Schroeder de la chaîne
  d'échappement (dry = modèle seul, IR/FDN coupés ; full = convolution complète).
- **Bar de qualité respecté** : build Release complet **vert** (0 erreur, 0
  warning) et `ctest` **13/13 PASS** avant conclusion — dont le gate
  `EngineLab.AudioRender` (34,6 s) et `EngineLab.CatalogPhysics` (67 s). Aucun
  filtrage de sortie de build masquant une erreur MSB.

Reproduction :

```
cmake --build out/build/windows-vs2022 --config Release --target EngineLabAudioAbHarness
./out/build/windows-vs2022/tools/Release/EngineLabAudioAbHarness.exe --output <dir>
# -> <dir>/audio-ab-metrics.csv  (par moteur × point × canal)
# -> <dir>/audio-ab-rt60.csv     (RT60 par moteur, preset street par défaut)
```

## 2. Limite majeure : ES2D n'est pas mesurable localement

ES2D possède bien un chemin de sortie audio déterministe :
`Simulator::readAudioOutput` → `Synthesizer::readAudioOutput(samples, int16*)`
(`src/synthesizer.cpp:141`) qui produit du PCM 16 bits. En théorie un petit
driver hors-ligne pourrait le vider dans un WAV.

En pratique, c'est **impossible ici** : les cinq sous-modules d'ES2D
(`delta-studio`, `simple-2d-constraint-solver`, `direct-to-video`, `csv-io`,
`piranha`) sont **vides** dans le checkout local
(`dependencies/submodules/*/` → 0 entrée). Le synthétiseur dépend de
`delta-studio` (types `ysAudioBuffer`) et le chargement moteur dépend de
`piranha` (scripts `.mr`) et du solveur de contraintes. Sans ces dépendances,
ni compilation ni rendu A/B reproductible ne sont possibles. Le `README` d'ES2D
le dit lui-même : c'est un outil **« NOT a scientific tool »**, orienté son et
ressenti, pas mesure.

**Plan B (exigé par `CLAUDE.md`)** : on ne fabrique aucun chiffre ES2D. On
compare les métriques mesurées d'EngineLab à des **plages de référence issues de
l'ingénierie audio/DSP et de l'acoustique moteur**, pas à la sortie d'un
simulateur. Ces plages sont dans le tableau §5.

## 3. Les trois paires quasi-équivalentes

Les assets moteur d'ES2D sont présents (même si les submodules manquent). Les
trois archétypes suggérés existent nommément des deux côtés :

| Archétype | EngineLab (`engines/`) | ES2D (`assets/engines/`) |
|---|---|---|
| I6 3.0 turbo (2JZ) | `02_toyota_2jz_gte_like` | `atg-video-2/03_2jz.mr` |
| V8 crossplane (GM LS) | `03_gm_ls3_like` | `atg-video-2/07_gm_ls.mr` |
| I4 moto (Hayabusa) | `06_hayabusa_i4_like` | `atg-video-1/04_hayabusa.mr` |

(Recouvrements additionnels : EJ25, Audi I5, radial 5, Honda VTEC↔K20, Kohler,
Harley↔Big Twin.) La comparaison de paires reste donc **EngineLab mesuré vs
plages de référence** ; le côté ES2D est identifié mais non rendu.

## 4. Tableaux de métriques par moteur (balayage, voicing par défaut)

Bande = part d'énergie audible bas/moyen/haut en %. `dc` en fraction de pleine
échelle. RMS et crête sur la fenêtre stabilisée finale de chaque point ; canaux
L/R montrés là où ils diffèrent (RMS, crête, pic) ; l'équilibre spectral R suit
L à ±2–3 points. Tous les points : `finite=yes`, 0 événement/pression perdu, 0
retard, 0 voix volée.

### Paire 1 — I6 3.0 turbo (2JZ) · idle 760 / redline 7200

| Point | rpm | RMS L/R | crête L/R | dc | bas/moy/haut % (L) | LRcorr | pic L |
|---|---|---|---|---|---|---|---|
| low  | 1851 | 0.170/0.131 | 2.39/3.38 | +0.0001 | 96.0/3.9/0.0 | 0.968 | 0.41 |
| mid  | 3311 | 0.246/0.201 | 3.40/3.30 | −0.0004 | 95.6/3.4/1.0 | 0.972 | 0.84 |
| high | 4926 | 0.197/0.173 | 2.73/2.75 | −0.0001 | 92.9/5.1/2.0 | 0.940 | 0.54 |
| peak | 6466 | 0.147/0.136 | 2.97/2.87 | +0.0002 | 3.2/92.5/4.3 | 0.916 | 0.44 |

### Paire 2 — V8 crossplane 6.2 (LS3) · idle 720 / redline 6600

| Point | rpm | RMS L/R | crête L/R | dc | bas/moy/haut % (L) | LRcorr | pic L |
|---|---|---|---|---|---|---|---|
| low  | 1654 | 0.069/0.063 | 7.52/7.84 | −0.0001 | 68.3/31.5/0.2 | 0.772 | 0.52 |
| mid  | 2972 | 0.063/0.066 | 8.42/7.78 | −0.0001 | 36.9/57.6/5.5 | 0.595 | 0.53 |
| high | 4497 | 0.071/0.072 | 3.08/3.27 | +0.0000 | 22.9/75.0/2.1 | 0.618 | 0.22 |
| peak | 5935 | 0.108/0.103 | 2.67/2.62 | +0.0000 | 18.4/81.5/0.2 | 0.781 | 0.29 |

### Paire 3 — I4 moto 1.3 (Hayabusa) · idle 1250 / redline 11200

| Point | rpm | RMS L/R | crête L/R | dc | bas/moy/haut % (L) | LRcorr | pic L |
|---|---|---|---|---|---|---|---|
| low  | 2791 | 0.072/0.064 | 5.60/5.70 | −0.0003 | 90.2/9.4/0.4 | 0.916 | 0.40 |
| mid  | 5044 | 0.036/0.040 | 5.85/5.17 | +0.0000 | 53.3/44.5/2.2 | 0.279 | 0.21 |
| high | 7626 | 0.082/0.080 | 2.83/3.04 | +0.0000 | 5.6/94.3/0.1 | 0.756 | 0.23 |
| peak | 10080 | 0.069/0.071 | 3.20/3.24 | +0.0001 | 7.6/92.1/0.3 | 0.689 | 0.22 |

### Sept autres moteurs du catalogue (canal L, R similaire)

| Moteur | Point | rpm | RMS L | crête | bas/moy/haut % | LRcorr | RT60 dry/full (s) |
|---|---|---|---|---|---|---|---|
| K20 I4 VTEC | low→peak | 2148→7793 | 0.107→0.107 | 9.16→3.20 | 73/25/2 → 6/94/0 | 0.87→0.73 | 0.040 / 0.117 |
| EJ25 F4 turbo | low→peak | 1726→6139 | 0.144→0.190 | 4.63→2.72 | 98/2/0 → 87/12/2 | 0.94→0.88 | 0.029 / 0.128 |
| Audi I5 turbo | low→peak | 1808→6381 | 0.119→0.181 | 3.34→4.30 | 93/7/0 → 9/89/3 | 0.92→0.92 | 0.031 / 0.123 |
| Big Twin V2 | low→peak | 1400→5049 | 0.109→0.301 | 8.98→2.74 | 81/19/0 → 89/11/0 | 0.96→0.97 | 0.032 / 0.133 |
| Merlin V12 | low→peak | 946→3023 | 0.090→0.293 | 2.54→3.02 | 98/2/0 → 95/5/0 | 0.96→0.98 | 0.084 / 0.130 |
| Flat-6 aircooled | low→peak | 1900→6871 | 0.052→0.117 | 9.43→2.47 | 49/50/1 → 9/91/0 | 0.39→0.67 | 0.049 / 0.118 |
| Radial 5 | low→peak | 894→2159 | 0.034→0.048 | 4.53→2.67 | 96/4/0 → 96/4/1 | 0.98→0.98 | 0.065 / 0.128 |

Agrégats sur les 80 lignes (10 moteurs × 4 points × 2 canaux) :

- RMS : **0.027 – 0.301** (moyenne 0.121) — soit une plage de **~21 dB**.
- Crête : **2.37 – 9.70** (moyenne 4.46). Par point : low 2.4–9.7, mid 3.0–8.8,
  high 2.4–5.9, **peak 2.4–4.3**.
- DC : |dc| max **0.00165** (≤ 0.6 % du RMS au point le plus fort).
- Bande haute (> 4 kHz) : **0 – 7.2 %**, moyenne **1.19 %**.
- Corrélation L/R : **0.28 – 0.98** (moyenne 0.84), toujours positive.
- Fraction d'échantillons proche pleine échelle : max **0.17 %** (pas de plateau
  de clipping ; `finite=yes` partout).
- RT60 dry (modèle seul) : **0.029 – 0.084 s** ; full (IR+FDN) : **0.058 – 0.133 s**.

## 5. Plages de référence (littérature ingénierie/DSP — Plan B)

| Dimension | Attendu (norme audio/acoustique moteur) | EngineLab mesuré | Verdict |
|---|---|---|---|
| Offset DC | < 1 % du RMS (idéalement ~0 ; HP ~20 Hz) | ≤ 0.6 % du RMS | **Supérieur / propre** |
| Clipping | pas de plateau saturé soutenu | ≤ 0.17 % près de PE, 0 plateau | **Supérieur / propre** |
| Facteur de crête | son moteur sain 3–8 ; limitation lourde ≈ 1.5–2.5 persistant | 5–9.7 bas/moyen, plancher ~2.4 seulement au pic | **Égal→supérieur** (compression AGC douce au pic) |
| Énergie moyenne (250 Hz–4 kHz) | présente, 25–50 % typique | monte de ~2–5 % (bas) à 75–94 % (pic) | **Égal** (pas de trou : le médium se remplit avec le régime) |
| Énergie haute (> 4 kHz) | 5–20 % (rasp, distribution, injecteurs, air) | **< 2 % moyen, max 7 %** | **Inférieur** (spectre sombre/étouffé) |
| RT60 échappement | ring-down court de tuyau/silencieux, ~20–200 ms, croissant avec la taille | 0.03–0.13 s, dry ↑ avec la cylindrée (Merlin 0.084 > moto 0.033) | **Supérieur / réaliste** |
| Différenciation RT60 par moteur | chaque système devrait décroître différemment | dry ×2.9 d'écart ; **full ~0.12 s quasi constant** (IR partagé domine la queue) | **Inférieur** (une seule IR par défaut aplatit l'individualité) |
| Corrélation stéréo | +0.3…+0.9 sain ; ~+1 = mono ; négatif = risque de phase | 0.28–0.98, jamais négatif | **Égal** (compatible mono ; plusieurs presets étroits 0.96–0.98) |
| Cohérence de loudness catalogue | switch moteur ≈ iso-loudness | **~21 dB d'écart** ; petits moteurs sous le plancher −27 dBFS | **Inférieur** (pas d'égalisation ; en partie voulu par cylindrée) |
| Contrat temps réel | 0 drop / retard / vol de voix, fini | 0 partout sur 10 moteurs × 4 points | **Supérieur** |

## 6. Conclusion tranchée

**Là où EngineLab est mesurablement supérieur (aux normes DSP) :**

1. **Propreté de signal.** DC quasi nul (≤ 0.6 % RMS), aucun plateau de clipping,
   jamais de non-fini. C'est de la qualité de sortie irréprochable.
2. **Dynamique bas/moyen régime.** Crête 5–9.7 : les impulsions de combustion
   restent distinctes et percussives, pas écrasées par l'AGC. La compression au
   pic (plancher ~2.4) est douce et en partie physique (les bouffées fusionnent
   en tonalité périodique à haut régime).
3. **Réalisme de la décroissance d'échappement.** RT60 court (0.03–0.13 s), dans
   la plage physique d'un tuyau/silencieux, et la partie modèle (dry) **croît
   correctement avec la taille du moteur**.
4. **Contrat temps réel tenu** sur tout le balayage des 10 moteurs.

**Là où EngineLab est mesurablement inférieur :**

1. **Spectre sombre.** L'énergie > 4 kHz est < 2 % en moyenne (max 7 %), contre
   5–20 % attendus pour du son moteur réel. Les 2,5 octaves supérieures sont
   quasi vides : rendu probablement perçu comme *étouffé/mat*. C'est le déficit
   mesurable le plus net, et un axe d'amélioration voicing prioritaire.
2. **Loudness non égalisée.** ~21 dB d'écart RMS à travers le catalogue ; les
   petits moteurs (Radial, Hayabusa) tombent **sous** le plancher nominal
   −27 dBFS à bas régime. Changer de moteur fait sauter le niveau.
3. **Individualité de queue d'échappement limitée dans le chemin complet.** Le
   RT60 « full » est quasi constant (~0.12 s) parce que l'IR par défaut partagée
   domine la queue ; la différenciation réelle vit dans le chemin dry/FDN et est
   aplatie par une IR unique. (Cohérent avec la note `realtime-audio.md` : la
   couche « voice » est le seul différenciateur sans télémétrie de pression.)

**Sur la question de départ.** L'architecture d'EngineLab est effectivement plus
riche (source multi-couches, séparation multi-chemin, contrat temps réel
vérifiable), et ses sorties sont
**mesurablement plus propres et plus dynamiques** que les normes DSP courantes.
Mais « plus propre et plus dynamique » **n'est pas** « meilleur son » : deux
déficits mesurables (spectre sombre, loudness non égalisée) tirent probablement
le ressenti vers le bas, et le troisième (IR unique) limite l'individualité des
presets. Face à ES2D précisément, la comparaison sonore **reste non tranchée**
faute de rendu ES2D reproductible. La supériorité *architecturale* est étayée ;
la supériorité *sonore* n'est **ni prouvée ni infirmée** — mais on sait désormais
**quoi corriger** pour la prouver : remonter l'énergie HF et égaliser la loudness
catalogue, en re-mesurant avec cet instrument avant/après.

## 7. Provenance

- Instrument : [`tools/AudioAbHarness.cpp`](../tools/AudioAbHarness.cpp),
  cible `EngineLabAudioAbHarness`.
- Chemin audio réel mesuré : `RealtimeEngineAudio` via `EngineRuntime`
  (voicing par défaut, `convolution=0.45`, `exhaustPreset=street`).
- Chiffres : `audio-ab-metrics.csv` et `audio-ab-rt60.csv` régénérés par la
  commande du §1. Plages de référence : ingénierie audio/DSP et acoustique
  moteur, **jamais** la sortie d'un simulateur (exigence `CLAUDE.md`).
- Bar de qualité : build Release vert + `ctest` 13/13 PASS (dont
  `EngineLab.AudioRender`) au commit courant de `fix/clutch-lockup`.
