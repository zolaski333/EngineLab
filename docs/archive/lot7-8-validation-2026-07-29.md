# Lots 7 et 8 — décision admission et preuves de livraison (2026-07-29)

Ce document clôt le plan audio en huit lots. Il enregistre deux choses : la
**décision de ne pas exécuter le lot 7**, avec la mesure qui la justifie, et les
**preuves de livraison du lot 8**.

Point de départ : `b90c75f`, arbre propre, `main` synchronisé avec `origin/main`.
Machine : Intel i5-10600, 6 cœurs / 12 threads. Toolset MSVC **14.34.31933**.

---

## 0. Réparation de l'environnement de build (préalable, pas un changement de code)

Le build était cassé avant toute modification, pour deux raisons indépendantes
qui n'ont rien à voir avec le source. Les deux sont maintenant documentées en
tête de `CLAUDE.md`.

1. `cmake` n'est pas dans le `PATH` de l'agent.
2. Une installation **Visual Studio 18** cohabite avec 2022, `vswhere.exe` ne
   renvoie **rien** (le registre d'instances de l'installateur est endommagé), et
   CMake résolvait donc le générateur « Visual Studio 17 2022 » sur
   `.../Microsoft Visual Studio/18/Community`. L'arbre de build était
   contradictoire : `CMAKE_GENERATOR_INSTANCE` désignait VS 18 pendant que
   `CMAKE_LINKER` pointait toujours dans le `14.34.31933` de 2022.

Correction : reconfiguration avec
`-DCMAKE_GENERATOR_INSTANCE=".../Microsoft Visual Studio/2022/Community"`, **plus**
le retour de la même ligne dans les quatre caches de sous-build FetchContent
(`juce-subbuild`, `juce-build/tools`, `nlohmann_json-subbuild`,
`yaml_cpp-subbuild`), qui mémorisent leur propre instance. Les caches ont été
édités et non supprimés, pour ne pas relancer les téléchargements.

Le toolset retenu est celui que le cache référençait déjà : **c'est une
réparation d'environnement, pas un changement de chaîne de compilation.**

---

## 1. Lot 7 — admission : **non déclenché**

Le lot 7 était explicitement conditionnel :

> Si le banc complet prouve que simulation + audio ne tiennent plus le temps
> réel, alors seulement [simplifier l'admission]. […] Tant que le système complet
> passe, je ne prendrai pas le risque de rouvrir l'admission 95 mm déjà validée.

### Ce qui avait fait croire au déclenchement

Une série interrompue mesurait le Merlin à **0,998 / 0,985 / 0,985** pour 3 / 5 / 0
workers, plus un quatrième passage **INVALID** (2 échéances audio manquées). Lu
seul, cela dit « le V12 ne tient plus le temps réel ».

Ces chiffres ont été **écartés**, en application du protocole de `CLAUDE.md` :
ils ont été produits après plusieurs heures de builds et de suites de tests, et
la dérive de session documentée sur ce banc atteint 20 % sur tous les moteurs.
Comparer à eux serait exactement l'erreur que le protocole interdit.

### Mesure de reprise, binaire fraîchement relié, même heure

Cinq passages Merlin consécutifs, `--free-run --with-audio --rpm 7000 --warmup 3
--seconds 6`, audio de production 48 kHz / 256 :

| passage | 1 | 2 | 3 | 4 | 5 |
|---|---:|---:|---:|---:|---:|
| facteur | 1,106 | 1,130 | 1,130 | 1,118 | 1,117 |

Tous valides, `audP99` 68 %, et **zéro** sur chacun des compteurs `miss`,
`dropP`, `late`, `legacy`, `lvl`.

### Catalogue complet, bout en bout, audio actif

`out/validation/lot7-2026-07-29/catalog-endtoend-1.txt` — **14/14 points valides**.

| moteur | cyl | wrk | facteur | audMean | audP99 |
|---|---:|---:|---:|---:|---:|
| K20A-like 2.0 I4 VTEC | 4 | 2 | 1,548 | 21,0 % | 32 % |
| 2JZ-GTE-like 3.0 I6 Turbo | 6 | 2 | 1,545 | 28,9 % | 44 % |
| LS3-like 6.2 Crossplane V8 | 8 | 2 | 1,182 | 35,4 % | 54 % |
| EJ25-like 2.5 Flat-4 Turbo | 4 | 2 | 1,943 | 27,0 % | 41 % |
| Audi I5-like 2.5 Turbo | 5 | 2 | 1,716 | 26,6 % | 41 % |
| Hayabusa-like 1.3 I4 | 4 | 2 | 1,805 | 21,0 % | 32 % |
| Big Twin-like 1.9 V2 | 2 | 0 | 3,327 | 16,9 % | 28 % |
| **Merlin-like 19.8 V12 Scaled** | 12 | 3 | **1,120** | 45,1 % | 68 % |
| Aircooled-like 3.6 Flat-6 | 6 | 2 | 1,329 | 30,1 % | 46 % |
| Radial-like 6.5 R5 | 5 | 2 | 2,443 | 20,8 % | 33 % |
| Yamaha CP2 MT-07-like 689 Twin | 2 | 0 | 2,955 | 14,1 % | 23 % |
| Yamaha CP3 MT-09-like 890 Triple | 3 | 2 | 2,184 | 17,8 % | 27 % |
| Yamaha CP4 MT-10-like 998 Crossplane I4 | 4 | 2 | 1,847 | 20,8 % | 32 % |
| VW EA288-like 2.0 TDI I4 | 4 | 2 | 2,401 | 23,1 % | 35 % |

`overruns` 0, `maxLate` 0,0 ms, et `miss` / `dropP` / `late` / `legacy` / `lvl`
à zéro sur les quatorze lignes.

### Décision

Le système complet passe : pire cas **Merlin 1,120×**, soit **12 % de marge**
bout en bout avec le renderer de production actif. La condition d'ouverture du
lot 7 n'est pas remplie. **L'admission 95 mm n'est pas touchée.**

### Limite à énoncer honnêtement

12 % de marge est au-dessus du plancher de 10 % que le plan se fixait, mais en
dessous de la cible de 15 %, et c'est une marge mesurée **machine au repos**. Le
passage INVALID de la série écartée reste un fait : sous charge Windows externe,
le V12 peut manquer une échéance. Ce n'est pas un défaut d'admission, c'est
l'absence de test de charge — voir les limites restantes en fin de document.

---

## 2. Deux affirmations du plan étaient périmées

Le plan directeur avait été écrit à partir de documents qui ne décrivaient plus
le code livré. Corrigé dans cette session :

- **« Lever le plafond acoustique de 2,19 kHz » est déjà fait.**
  `EngineSimulator::step` utilise
  `maximumLowSpeedExhaustCouplingSeconds.value_or(125.0e-6)` : le cap est passé
  de 250 à **125 µs** le 2026-07-28. Nyquist physique **5 640 Hz** sur le LS3 et
  **3 480 Hz** sur le Merlin, contre 1 880 / 1 740 avant. Les mentions périmées
  ont été corrigées dans `next-session-brief.md` (priorité 4, marquée FAIT),
  `thermoacoustic-architecture.md` (§ multirate et §14) et `CLAUDE.md`.
- **Le profil « admission 1-D = 83 % du sous-pas » est périmé.** L'avance des
  runners est désormais conditionnée par `flushIntakeNetworks`, avec un couplage
  par défaut de 400 µs (`intakeCouplingIntervalSeconds`,
  `EngineSimulator.cpp` ~1749-1790, appelée à ~2105 et ~2242), plus une vidange
  forcée à la fermeture de soupape d'admission. Les runners ne sont donc plus
  avancés deux fois par sous-pas mécanique comme le décrit la note. Le passage
  est marqué **STALE** dans `CLAUDE.md` : il doit être **re-mesuré** avant de
  servir à justifier quoi que ce soit. Aucun nouveau profil n'est publié ici —
  il n'a pas été mesuré dans cette session, et l'inventer serait précisément la
  faute que ce dépôt documente.

---

## 3. Lot 8 — preuves de livraison

### Suite Release complète

`ctest --test-dir out/build/windows-vs2022 -C Release` :
**100 % — 27 tests sur 27**, 546,83 s.
Journal : `out/validation/lot7-2026-07-29/ctest-full.txt`.

Points notables : `EngineLab.CatalogReference` (les 24 points constructeur)
62,82 s, `EngineLab.CatalogPhysics` 100,06 s, `EngineLab.AudioRender` 91,25 s,
`EngineLab.IdleStabilityRegression` 82,88 s, `EngineLab.IntakeTuning` 81,49 s,
`EngineLab.Core` 47,64 s — cette dernière durée confirme un passage réel et non
l'abandon précoce (~8 s) décrit dans `CLAUDE.md`. Les deux gates de passage de
rapport et `EngineLab.AudioTransients` passent également.

### Paquet

- ZIP : `EngineLab-0.1.0-win64.zip`, 4 120 744 octets.
- SHA-256 :
  `450104F8BB0E13DB153890B15D957C98495D2C6BCF6D21E0AD3C3FABBB7207D6`
- Contenu extrait : **61 fichiers, 9 582 252 octets** — `EngineLab.exe`,
  `LICENSE.md`, `README.md`, `assets/ir`, `docs`, `engines`, `examples`, `parts`.
- Inventaire complet avec SHA-256 par fichier :
  `out/validation/lot7-2026-07-29/package-inventory-sha256.txt`.

### Test de démarrage après extraction

Le ZIP a été extrait dans un dossier vierge et `EngineLab.exe` lancé **depuis
l'arborescence extraite**, pas depuis l'arbre de build : processus vivant après
12 s, empreinte mémoire ~101 Mo, fermeture propre par la fenêtre avec **code de
sortie 0**.

---

## 4. Limites restantes, énoncées sans les masquer

Elles sont inchangées par cette session, qui n'a produit aucune nouvelle source
sonore.

1. **Aucun verdict humain.** Le corpus A/B de dix familles est constitué,
   vérifié par SHA-256, égalisé en sonie et rendu en paires — mais **aucune
   écoute n'a été enregistrée**. Le gate perceptuel du plan (5 auditeurs
   minimum, 65 % de préférence) n'a jamais été exécuté. Tant qu'il ne l'est pas,
   aucune affirmation de supériorité perceptuelle n'est soutenable, en
   particulier face à ES2D.
2. **Quatre des dix références restent des proxys**, honnêtement étiquetés comme
   tels dans le manifeste.
3. **Aucun mode NVH mesuré.** Le catalogue reste intégralement en
   `estimatedFamily`. Le schéma 5 accepte des modes sourcés ; personne n'en a
   fourni.
4. **Pas de test de charge longue durée ni de balayage de périphérique audio.**
   Le banc bout en bout tourne 6 s par moteur, sans interface, sans vrai
   périphérique, à un seul couple fréquence/buffer (48 kHz / 256). L'heure sur
   les pires cas, les tailles 64/128, les fréquences multiples et la charge
   Windows externe — le lot 6 du plan d'origine — restent à faire. C'est la
   lacune la plus directement liée à la marge de 12 % du Merlin.
5. **PMEP.** L'oracle reste à 0,990 bar contre une cible de 0,750 à 6 000 tr/min.
   Non touché ici.
6. **Les alertes de contre-pression** du 2JZ, de l'EJ25 et de l'Audi I5
   apparaissent toujours (`bpWarn = YES`) dans la table ci-dessus. Elles ne sont
   pas masquées et restent à instruire.
