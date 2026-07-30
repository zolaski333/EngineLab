# Boucle d'écoute : segmentation, appariement par condition, A/B de variante

2026-07-30. Priorité **P1** du plan de reprise : rendre la boucle de jugement
utilisable, parce que c'est le goulet d'étranglement du projet — le seul arbitre
valable du son est humain, et tout ce qui le précède ne sert qu'à lui présenter
quelque chose de jugeable.

Ce document enregistre ce qui a été mesuré, ce qui a été corrigé, et **une
hypothèse de départ qui était fausse et a été abandonnée avant d'être codée**.

## 1. L'hypothèse retirée : le cache de télémétrie

Le plan annonçait « un rendu rapide : cible < 30 s du changement au clip audible ;
aujourd'hui c'est un build complet », et proposait de mettre en cache la sortie
physique d'une trajectoire pour que seul l'étage audio soit re-rendu.

**Mesuré d'abord. La prémisse était fausse.** Bureau 12 threads, mesures
consécutives :

| Étape | Coût |
|---|---:|
| rendu d'une trajectoire, CP2 twin | 5,9 s |
| rendu, Radial R5 | 9,7 s |
| rendu, LS3 V8 | 12,4 s |
| rendu, Merlin V12 | 15,6 s |
| relink `EngineLabAbClipRenderer` après un `.cpp` audio | 11,0 s |
| relink après `RealtimeEngineAudio.**hpp**` | **115,1 s** |

Donc la boucle sur un `.cpp` est déjà de **17 à 27 s**, sous la cible, et la
physique n'y est pas dominante. Le coût réel est le **fan-out d'en-tête** : 115 s,
que **aucun cache de télémétrie n'adresse**. Le cache a été abandonné sans écrire
une ligne.

Ce qui retire réellement le build de la boucle est de faire du voicing une
**donnée** et non du code — c'est-à-dire la priorité P2, qui se trouve donc être
aussi le correctif de performance de la boucle. Les deux priorités n'en font
qu'une.

Leçon générique, et c'est la règle numéro un du projet appliquée à moi-même :
j'avais écrit « aujourd'hui c'est un build complet » sans l'avoir chronométré.

## 2. Le biais de niveau : 23,3 dB sur le ralenti

`AbClipRenderer` rendait une trajectoire de 15,3 s (démarrage → ralenti tenu →
montée → rupteur → lever de pied) et lui appliquait **un seul** gain BS.1770 pour
l'amener à −20 LUFS. La sonie intégrée est fixée par la partie forte, donc le
ralenti se retrouvait très bas dans le fichier remis.

Mesuré sur le CP2 (`--engines "Yamaha CP2"`) :

| | sonie propre | présenté avant | présenté après |
|---|---:|---:|---:|
| trajectoire entière | −21,55 LUFS | −20,00 | — |
| segment `idle` | −44,89 LUFS | ≈ **−43,3** | −20,00 |
| segment `rev` | −20,30 LUFS | ≈ −18,8 | −20,00 |

**23,3 dB** d'erreur de présentation sur le ralenti du CP2, **18,2 dB** sur celui
du full system. Le premier passage humain avait rapporté « le son au ralenti de
chaque moteur est trop faible et ne ressemble en aucun cas à la réalité » pour
*tous* les moteurs ; une grande partie de cette plainte était le protocole, pas le
moteur audio.

### Correctif

Une seule trajectoire est rendue, puis **découpée** (`listeningSegments`) :

| Segment | Fenêtre | Durée |
|---|---|---:|
| `idle` | les 3,5 dernières secondes du maintien de ralenti | 3,50 s |
| `rev` | montée + rupteur + lever de pied, d'un bloc | 6,80 s |

- La fenêtre de ralenti est prise à la **fin** : la surchauffe d'après-démarrage
  occupe le début, et une fenêtre qui l'attrape mesure un flare.
- La montée reste **un seul** clip : c'est un geste unique, et sa dynamique
  interne est ce qu'un auditeur juge. Le découpage ne remonte donc rien
  *à l'intérieur* d'un clip — ce serait un changement de voicing déguisé en mesure.
- Les segments viennent du **même** rendu, sinon un ralenti et une montée
  pourraient décrire deux états différents du moteur.

### Le rendu n'a pas changé — vérifié, pas déduit

La trajectoire entière mesurait **−21,5486 LUFS** avant le changement et
**−21,55** après, sur le même moteur, même heure, même machine. La segmentation
est une correction de mesure. La sonie de la trajectoire entière est désormais
imprimée et publiée dans la clé (`whole_trajectory_lufs`) précisément pour qu'une
modification involontaire du rendu se voie dans un seul chiffre.

## 3. Appariement par condition des références

Cinq prises locales, durées mesurées par `--validate-manifest` : 8,53 s (LS3),
22,49 s (Hayabusa), 35,21 s (EJ25), 35,47 s (2JZ), 47,01 s (Big Twin). Un
enregistrement porte son ralenti et sa montée à des **offsets différents**, donc un
unique `clip_start_seconds` ne peut pas apparier les deux conditions.

Schéma de manifeste **3** : `segment_windows`, un offset par condition.

| Cas | Comportement |
|---|---|
| segment déclaré dans `segment_windows` | apparié, `window_condition_matched: true` |
| segment absent | **aucune** référence, motif publié dans `control_error` |
| prise trop courte pour la fenêtre (< 2 s utiles) | refusée, motif publié |
| schéma 2 (sans fenêtres) | décodé et vérifié, mais n'apparie plus rien |
| `--allow-unmatched-reference-window` | apparie, marque `[WINDOW NOT CONDITION-MATCHED]`, non publiable |
| `--require-references` | échoue **avant** tout rendu |

Vérifié sur les quatre chemins. Le ralenti du LS3 (fichier de 8,53 s, une seule
condition possible) n'obtient **pas** de référence, et cela apparaît en clair au
lieu d'être substitué.

## 4. Un A/B en aveugle entre deux moteurs du catalogue

Le tableau A/B de `docs/audio-ab-listening.md` distingue depuis longtemps le
diagnostic (contre le réel, sans seuil) de la non-régression (contre une baseline
EngineLab, seuil 65 %). Le second cas **n'avait pas d'outil** : on ne pouvait
comparer le CP2 stock à sa variante full system qu'à l'oreille dans
l'application, sans appariement de sonie ni aveugle.

`--compare <baseline>,<candidat>` rend les deux moteurs sur la même trajectoire,
découpe les mêmes fenêtres, cale chaque côté à la même sonie et tire l'attribution
A/B. Rendu de référence : `out/validation/listening-cp2-variant-2026-07-30`.

| Paire | Condition | Candidat (full system) | Baseline (stock) | Niveau corrigé |
|---:|---|---|---|---:|
| 1 | idle | −41,27 → −20,00 LUFS, crête 0,525 | −44,89 → −20,00, crête 0,572 | +18,20 dB |
| 2 | rev | −21,92 → −20,00 LUFS, crête 0,557 | −20,30 → −20,00, crête 0,648 | −1,15 dB |

Le tirage a mis le candidat en B sur la paire 1 et en A sur la paire 2 : le
recommencement anti-« tout du même côté » fonctionne.

Garde-fous, chacun refusant au lieu d'ignorer :

- `--compare` avec un nombre d'arguments ≠ 2 → refus ;
- `--compare` avec `--engines` → refus (deux modes distincts) ;
- `--compare` avec une option de référence → refus. Un run qui aurait
  silencieusement laissé tomber `--reference-manifest` ressemblerait à une
  comparaison contre le réel dans l'historique du shell ;
- filtre ambigu → refus, avec la liste des moteurs correspondants. `Twin` désigne
  **trois** entrées du catalogue ; c'est le même piège de sous-chaîne que celui
  qui aurait tenu la variante d'échappement de course au couple homologué du
  MT-07 stock.

## 5. Deux défauts trouvés en exerçant le dépouillement

- **Une paire sans côté contrôle était notée comme une comparaison.**
  `scripts/listening.py` plantait dessus (`KeyError` sur `reference`) ; après
  correction du plantage, elle était **moyennée**, ce qui est pire : un auditeur
  qui note le côté absent note du silence. Ces jugements sont maintenant exclus de
  toutes les statistiques et listés à part avec leur motif.
- **La mise en garde « sept références sur dix sont des proxys » était écrite en
  dur** dans le générateur de rapport. Elle est maintenant comptée depuis la clé.
  Une mise en garde périmée est pire qu'aucune : elle est lue comme actuelle.

## 6. Portée — ce qui n'a pas été touché

Aucun code de bibliothèque n'a été modifié : les changements sont dans
`tools/AbClipRenderer.cpp` et `scripts/listening.py`. Le moteur audio, la
physique et le catalogue sont inchangés, et la voix par défaut est intacte
(vérifiée par la sonie de trajectoire ci-dessus).

## 7. Limites restantes

- Le corpus de références local est **de provenance non établie** et vit sous
  `out/` (git-ignoré) : il n'est ni commité ni empaqueté, et les manifestes de
  test écrits pour l'exercer le disent explicitement. Il ne peut servir à publier
  aucun verdict. Il faut des prises dont la licence et les conditions sont
  documentées.
- Les fenêtres `segment_windows` du manifeste de test n'ont **pas** été choisies à
  l'écoute : elles exercent le code d'appariement. Il faut auditionner chaque prise
  pour les renseigner.
- Aucune préférence humaine n'est encore enregistrée sur le pack segmenté. L'outil
  rend le test possible ; il ne remplace pas les auditeurs.
- Les stems offline ne reconstituent toujours pas leur master (max|diff| 0,527,
  soit −5,6 dBFS) : tout diagnostic par stem reste invalide, et il faut passer par
  des mutes successifs sur le master. Non corrigé.
- Le voicing reste du **code**, donc la boucle paie encore 115 s sur un changement
  d'en-tête. C'est la priorité P2.
