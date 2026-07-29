# Premier verdict d'écoute humaine, et ses causes mesurées (2026-07-29)

Premier retour humain jamais enregistré sur le son d'EngineLab. Un auditeur, sur
le pack aveugle `listening-pilot-2026-07-29` (10 paires, sonie égalisée
ITU-R BS.1770, graine 20260730). Aucune feuille CSV n'a été remplie : le verdict
est venu en texte libre, et il était suffisamment net pour être exploitable tel
quel.

## Le verdict, tel qu'il a été rapporté

1. Le son simulé est **trop aigu**, sur toutes les paires.
2. À la mise en gaz, il produit un **sifflement « comme un compresseur »**.
3. Le **ralenti de chaque moteur est trop faible** et « ne ressemble en aucun cas
   à la réalité ».
4. L'auditeur **n'a pas su dire à quel moteur réel** chaque extrait simulé
   correspondait, alors que la reconnaissance des vraies prises était immédiate.

Le même constat sur les dix familles : ce n'est pas une calibration par moteur,
c'est un défaut systématique de la chaîne.

## Pourquoi ce retour est exploitable malgré un seul auditeur

Un « whine de compresseur » a été rapporté sur le K20, le LS3, le Merlin, le Big
Twin et le Radial. Ces cinq moteurs sont **atmosphériques**. Le stem
`forced_induction` mesure exactement zéro sur eux. Le percept ne peut donc pas
être une propriété du moteur simulé : c'est un artefact, et un artefact se
localise par la mesure sans avoir besoin d'un panel.

## Cause 1 — l'aigu et le sifflement : la couche admission

Mesuré sur le **master** (pas sur les stems, voir la réserve plus bas), en
coupant la seule couche admission, fenêtre montée + limiteur :

| Moteur | énergie >5 kHz retirée en coupant l'admission |
|---|---:|
| LS3 V8 | **99,9 %** |
| K20 I4 | **97,7 %** |
| Big Twin V2 | **94,8 %** |
| Merlin V12 | 17,4 % |

Centroïde spectral du master : LS3 1707 → 1144 Hz sans admission, K20 1231 →
1277, Big Twin 1034 → 977.

Comparé aux **vraies prises que l'auditeur a jugées**, dans les mêmes clips :

| Paire | EngineLab | Réel | Rapport |
|---|---:|---:|---:|
| Hayabusa | 2532 Hz | 80 Hz | **31,5×** |
| LS3 | 1631 Hz | 165 Hz | **9,9×** |
| EJ25 | 914 Hz | 102 Hz | 9,0× |
| Big Twin | 1030 Hz | 116 Hz | 8,9× |
| Flat-6 | 939 Hz | 353 Hz | 2,7× |
| Radial | 572 Hz | 284 Hz | 2,0× |
| 2JZ | 738 Hz | 428 Hz | 1,7× |
| Merlin | 389 Hz | 274 Hz | 1,4× |
| CP3 | 1556 Hz | 1259 Hz | 1,2× |
| K20 | 1188 Hz | 3100 Hz | 0,4× |
| **moyenne** | **1149 Hz** | **616 Hz** | **1,86×** |

**Réserve à garder :** plusieurs références ont un aigu naturellement rabattu
(transcodages, prises lointaines, téléphone) — un centroïde de 80 Hz sur la réf
Hayabusa n'est pas normal non plus, et la réf K20 est *plus brillante* que la
simulation. Une part de l'écart vient donc du corpus. Mais un facteur 9 à 31 ne
s'explique pas par ça, et l'auditeur a conclu indépendamment des chiffres.

### Le mécanisme, dans le code

`AcousticIntakeNetwork` rayonne la bouche d'admission avec **trois** propriétés
qui se composent, toutes défavorables :

1. **Aucun élément filtrant n'est modélisé.** La compliance de boîtier et le
   conduit d'entrée existent, mais rien ne représente le filtre — qui est
   précisément ce qui, dans la réalité, dissipe l'aigu d'admission.
2. La bouche est placée à **l'origine du véhicule**, pas dans le compartiment
   moteur (`observer.prepare(..., {}, ...)`).
3. Son axe est `{0, 1, 0}` — **pointé droit sur le microphone** situé à y = +4 m
   — en terminaison `unflanged`, la géométrie qui rayonne le mieux.

Seuls **8 moteurs sur 14** déclarent un `airbox_volume_l`. Les six autres — K20,
LS3, Hayabusa, Harley, Porsche, Radial — tombent dans le repli où la jonction
papillon/plénum rayonne directement vers l'observateur, et ce sont exactement les
moteurs les plus atteints. Pour cinq d'entre eux c'est un **trou de données** (ces
moteurs ont tous un boîtier dans la réalité) ; pour le Radial, moteur d'avion à
écope et carburateur, l'absence de filtre est physiquement juste.

## Cause 2 — le ralenti : il n'y en avait pas

`AbClipRenderer.cpp` et `makeDefaultOfflineAudioScenario` tenaient tous deux
`max(idleRpm * 1.35, redlineRpm * 0.22)`. Le plancher `redline * 0.22` gagnait sur
**12 moteurs sur 14** :

| | ralenti réel | ralenti du clip | |
|---|---:|---:|---|
| Porsche flat-6 | 780 | 1672 | 2,14× |
| 2JZ | 760 | 1584 | 2,08× |
| LS3 | 720 | 1452 | 2,02× |
| K20 | 950 | 1892 | 1,99× |
| Harley | 760 | 1232 | 1,62× |

La cause structurelle : le gouverneur de ces deux chemins ne peut appliquer que
`controls.load`, **jamais le papillon**. Avec un papillon tenu à 12 %, il ne peut
que faire tirer le moteur contre un frein — il est incapable de produire un
ralenti. Le stage était un régime stabilisé au double, pas un ralenti.

Un **troisième défaut** est apparu avec la télémétrie ajoutée pour vérifier :
le stage `crank` cranke contre 20 % de papillon sans frein et **montait à
6723 tr/min** sur le K20 (settled 6119). C'était la première chose entendue.

Corrigé dans `2b719e1` : crank et idle non gouvernés, papillon fermé, régulateur
de ralenti de l'ECU laissé maître, exactement comme
`EngineLab.IdleStabilityRegression`. Ralenti stabilisé / ralenti catalogue :
K20 1,04×, LS3 1,00×, Big Twin 1,06×, Merlin 0,98×, Hayabusa 1,08×.

### Le « trop faible » ne se corrige pas dans le moteur audio

Après correction, l'écart ralenti → limiteur **s'aggrave de 4,4 à 5,3 dB** sur
trois moteurs sur quatre. C'est la preuve que la correction est juste : un vrai
ralenti à 950 tr/min produit moins d'énergie qu'un moteur tiré à 1892.

Le coupable du « trop faible » est le **protocole de clip** : un seul fichier
normalisé BS.1770 contenant un ralenti *et* une montée au rupteur cale la sonie
sur la partie forte et enterre le ralenti. Les références du corpus sont souvent
des prises de ralenti seul, donc normalisées *sur* le ralenti — la comparaison
était biaisée par construction. **Correctif : des clips séparés ralenti / montée,
chacun normalisé sur son propre contenu.** Ne pas remonter le niveau du ralenti.

## Cause 3 — la non-reconnaissance découle des deux premières

L'identité d'un moteur vit dans les ordres bas de la structure d'allumage
(`docs/thermoacoustic-architecture.md`, et la note de `CLAUDE.md` : « Character
lives in the firing-pattern envelope, not the steady spectrum »). Elle était
masquée par le souffle d'admission et jamais présentée au ralenti, là où elle est
la plus lisible.

## Défaut d'outillage trouvé au passage

**Les stems de l'exporteur hors-ligne ne reconstituent pas le master** : erreur
RMS −30,7 dBFS, et leur somme culmine 45 % au-dessus du master (0,564 contre
0,388). Le lot 2 avait prouvé cette propriété sur le chemin **temps réel**
uniquement. Tant que ce n'est pas corrigé, un diagnostic par stems est trompeur —
toutes les conclusions ci-dessus ont donc été validées sur le **master**, par
mute successif, jamais sur les stems.

## Ce qui reste sans cause identifiée

- **Le K20 ne perd que 2 % de centroïde** alors que son énergie >5 kHz
  s'effondre. Sa brillance vit entre 1 et 5 kHz et vient d'autre chose que
  l'admission. Aucune hypothèse mesurée à ce stade.
- **Le mi-aigu du Radial était l'artefact d'admission.** Une fois l'admission
  filtrée, `EngineLab.AudioRender` le refuse : plus de 98,5 % de son énergie
  passe sous 250 Hz. Son échappement ne génère quasiment rien au-dessus de son
  deuxième ordre. C'est le problème de bande passante « muffled » que `CLAUDE.md`
  décrit comme ouvert, et il était masqué. **Ne pas affaiblir le filtre pour
  faire passer ce gate** : ce serait calibrer sur l'artefact.

## Méthode : deux instruments jetés en route

- **Déduire le régime de l'audio est faux.** Un estimateur par produit
  harmonique s'est trompé d'octave sur le Big Twin (4560 tr/min lus pour un
  ralenti), dont le fondamental d'allumage est vers 13 Hz. Remplacé par la
  télémétrie simulateur `stage_speeds` publiée dans le manifeste de rendu.
- **Le premier prototype de mesure du sous-niveau d'admission passait par les
  stems**, et les stems sont faux (ci-dessus). Refait par mute sur le master.
