# Mesurer ES3D comme référence sonore

Écrit le 2026-08-04, après le verdict de la phase 0
(`docs/unified-duct-plan.md`). Ce document décrit **comment** transformer
« EngineLab devrait sonner comme ES3D » en un cahier des charges chiffré, et
**ce qui empêche** de le faire entièrement sans l'utilisateur.

## Pourquoi c'est nécessaire

La phase 0 a réfuté le diagnostic « le maillage 300 mm ne porte pas la
géométrie » : mesuré, l'échappement répond de 11,9 à 14,4 dB de forme. Le
projet a donc un chiffre pour lui-même et **aucun** pour sa référence. Viser
une cible jamais mesurée est exactement l'erreur que la phase 0 vient de
corriger, et la refaire sur ES3D coûterait autant.

## Ce que le prototype offre, et ce qu'il n'offre pas

`atg_es_prototype_v0.4.0a` est un binaire Vulkan fermé.

- **Pas de sources**, pas de drapeau en ligne de commande, pas de rendu hors
  ligne, pas d'export WAV. L'audio n'existe que comme sortie de carte son.
- **Pas d'automatisation dans le langage de script.** Une recherche de
  `record`, `wav`, `dyno`, `export`, `offline` dans toute la bibliothèque
  Piranha ne trouve que le mot « t**hold** » de `rev_limiter_activation`.
- **Rechargement à chaud, et c'est ce qui sauve le protocole** : « *While
  running, the ES3D demo listens for any changes in the `scripts/` folder. If
  it detects any, the engine being simulated updates immediately.* » Une
  variante se change donc **sans redémarrer**, sur la même instance, le même
  moteur et le même état de carte son — un contrôle à un seul facteur meilleur
  que de relancer entre deux prises.

## La chaîne d'instruments

### 1. `tools/WasapiLoopbackRecorder.cpp`

Capture en boucle de retour (WASAPI loopback) le mix de la sortie par défaut.
Ni « Stereo Mix » ni câble virtuel nécessaires. **Il ne fait pas partie de la
construction CMake** — c'est un outil de capture externe, autonome, compilé à
la demande :

```
cl /std:c++20 /EHsc /O2 /W4 /WX WasapiLoopbackRecorder.cpp /link ole32.lib
WasapiLoopbackRecorder.exe --seconds 12 --out capture.wav
```

Deux choix de conception valent d'être connus.

- **La boucle est cadencée sur l'horloge murale, jamais sur un nombre
  d'images.** Un endpoint inactif ne délivre *aucun* paquet — pas même des
  paquets silencieux — donc attendre un compte d'images se bloque indéfiniment
  quand rien ne joue. C'est précisément le cas qu'il faut rapporter, pas
  subir. La première version se bloquait ; le défaut est réel et corrigé.
- **Le manque entre audio capturé et temps mural est publié.** C'est le
  diagnostic qui sépare « la source était silencieuse » de « l'endpoint était
  inactif et n'a rien délivré ».

### 2. `EngineLabGeometrySensitivityHarness --wav-reference / --wav-variant`

Le harness de la phase 0 accepte maintenant des fichiers WAV externes et les
passe par **exactement** le même `analyse` / `distance` que les rendus
internes. C'est la seule raison pour laquelle les chiffres sont comparables :
une seconde implémentation des bandes produirait des nombres qui *paraissent*
comparables et ne le sont pas.

```
EngineLabGeometrySensitivityHarness --wav-reference ref.wav \
    --wav-variant sans_silencieux.wav [--wav-skip 1.0]
```

Le lecteur **refuse** un fichier qui n'est pas à 48 kHz plutôt que de le
rééchantillonner : les bords de bande sont mappés via la constante `audioRate`,
donc un autre taux serait binné contre les mauvaises fréquences et produirait
un spectre confiant et faux.

### 3. Validation de la chaîne

Non-vacuité prouvée avant tout usage, sur `tools/`-externe et sur le harness :

| test | résultat | attendu |
|---|---|---|
| aller-retour ton 1 kHz | pic capturé **0,3664** | source 12000/32768 = **0,3662**, gain unité |
| sonde gain × 0,5 | niveau **−6,02 dB**, forme **0,00 dB** | 20·log₁₀(0,5) = −6,021 ; un gain pur DOIT scorer zéro |
| sonde passe-bas 1 pôle | forme **9,74 dB**, pire bande −19,97 dB à 12,5 kHz | un timbre pur DOIT scorer gros |

Le test du gain est celui qui compte : si un changement de niveau scorait de la
forme, la métrique ne mesurerait pas ce qu'elle prétend.

## Le blocage : le lancement demande un bureau interactif

Un processus lancé par l'agent **ne peut pas démarrer ES3D**. Mesuré :

```
2026-08-04 03:11:05 | ERROR | PlatformObject::initialize::L17 | GetCursorPos failed
```
puis sortie `0xC0000005` (violation d'accès) en moins de 14 s.

Le processus est pourtant dans la **même session** que l'explorateur (16) —
ce n'est pas de l'isolation de session 0, mais un défaut d'accès au **bureau
interactif**, que `GetCursorPos` exige. L'application ne teste pas l'échec et
déréférence.

Un contournement existe (tâche planifiée interactive) mais c'est une
modification persistante de la configuration du système, hors de ce qu'un
agent doit faire sans accord explicite.

**Conséquence pratique : un seul lancement humain suffit.** Grâce au
rechargement à chaud, une fois le moteur au ralenti, toutes les variantes et
toutes les captures sont automatisables.

## Protocole

1. **(humain)** Lancer `bin/atg-es-prototype.exe`, démarrer le moteur, le
   laisser au ralenti, ne plus toucher au clavier ni au volume système.
2. **(agent)** Capturer la référence, `muffler_area` à sa valeur d'origine.
3. **(agent)** Éditer `muffler_area` → rechargement à chaud → capturer.
4. **(agent)** Revenir à la référence et recapturer — la répétabilité entre
   les deux prises de référence borne l'incertitude de tout le reste.
5. **(agent)** Analyser avec `--wav-reference` / `--wav-variant`.

Deux points de fonctionnement se définissent **sans consigne à tenir**, ce qui
évite le piège documenté dans `CLAUDE.md` (« un régime choisi par le contrôleur
n'est pas le point que vous croyez avoir demandé ») :

- **le ralenti**, fixé par `idle: 0.13` dans le script ;
- **le rupteur**, fixé par `rev_limiter_activation: 15500 rpm`.

## Les réserves, à lire avant d'interpréter un chiffre

- **La démo est délibérément sur-alimentée.** Son propre script, ligne 365 :
  « *I **cheated** this value in the video to compensate for the incomplete
  cylinder model and further highlight ES3D's superior audio production
  capabilities* », avec
  `fuel_energy_density: 1.75 * 57.5 MJ/kg` contre ~47,5 réels — un facteur
  **2,1**. Ce n'est pas un moteur calibré ; c'est une démonstration réglée pour
  impressionner.
- **Son silencieux n'a pas plus d'autorité théorique que le nôtre.** Sa section
  passe de 10 à 55 cm², soit **m ≈ 5,5**, contre m = 4,25 sur le 2JZ. Même
  régime de Munjal, donc même crête d'environ 7 dB — et sa transition est
  *graduelle* sur 80 % de la longueur, ce qui adapte les impédances et
  réfléchit **moins** qu'une marche franche. Physiquement c'est plus proche
  d'un mégaphone que d'un silencieux. **Si ES3D sonne satisfaisant, ce n'est
  donc pas parce que son silencieux atténue 20 dB.**
- **La capture est temps réel, donc non déterministe**, contrairement à tous
  les autres instruments du dépôt. Le README d'ES3D prévient lui-même des
  distorsions audio. Publier un écart type sur plusieurs prises, jamais un
  chiffre unique, et ne jamais transformer ce résultat en porte fine.
- **L'endpoint de cette machine est en 8 canaux.** Le mélange en mono moyenne
  tous les canaux ; comme le facteur est le même pour tous les fichiers, la
  *forme* et les écarts de *niveau* n'en sont pas affectés.
- **Le volume système ne doit pas bouger entre deux prises.** La capture
  mesurée est à gain unité ; un changement de volume se lirait comme une
  différence de niveau du moteur.

## Résultats mesurés (2026-08-04)

Moteur ES3D : la Hayabusa de `01_hayabusa_straight_pipe.mr`, au ralenti,
`idle: 0.13`. Facteur unique : `muffler_area` 15 → 55 cm² (m = 3,67), qui est
la bosse `.add_sample(0.8, muffler_area)` de ses quatre tubes d'échappement.

| comparaison | niveau | forme | pire bande |
|---|---|---|---|
| **témoin nul** (retour à la même config) | −0,09 dB | **1,14 dB** | +3,06 dB à 50 Hz |
| **silencieux 15 → 55 cm²** | **−2,72 dB** | **9,24 dB** | −23,31 dB à 80 Hz |

Le témoin nul borne le bruit de toute la chaîne — capture temps réel comprise —
à environ 1,1 dB de forme et 0,1 dB de niveau. L'effet du silencieux est donc
8 à 30 fois au-dessus du bruit : il est réel.

### Contre EngineLab, même moteur, même facteur unique

`EngineLabGeometrySensitivityHarness --filter Hayabusa`, variante
`sans-silencieux`, à 6 160 tr/min tenus :

| | perte d'insertion | forme du mix |
|---|---|---|
| **ES3D** (Hayabusa, ralenti) | **2,72 dB** | **9,24 dB** |
| **EngineLab** (Hayabusa, 6 160 tr/min) | **1,56 dB** | **3,34 dB** |
| EngineLab CP2 | 2,10 dB | — |
| EngineLab LS3 | 3,80 dB | — |
| **littérature** | **20 à 30 dB** | — |

Deux conclusions, de portées très différentes.

1. **Aucun des deux simulateurs ne silence, et l'écart entre eux est mineur
   devant l'écart à la réalité.** ES3D perd 2,72 dB en montant un silencieux,
   nous perdons 1,56 à 3,80 dB selon le moteur, quand la littérature donne 20 à
   30. Le grief « un échappement sans silencieux ne fait rien » **s'applique
   aussi à ES3D**. Ce n'est donc pas un défaut propre à EngineLab, et la cascade
   multi-chambres reste justifiée par la physique et par la littérature — pas
   par un retard sur la référence.
2. **ES3D change le TIMBRE environ 2,8× plus que nous pour ce même facteur**
   (9,24 contre 3,34 dB de forme sur le mix). Sa bosse de section agit à
   80 Hz avec 23 dB sur une bande. C'est le seul écart réel que la mesure
   établit, et c'est là qu'il faut chercher.

### La réserve qui limite la conclusion 2

**Les deux points de fonctionnement ne sont pas les mêmes** : ES3D était au
ralenti, notre harness tient 6 160 tr/min. C'est exactement le piège que
`CLAUDE.md` documente à répétition, et il n'est ici que partiellement évité —
le point ES3D est répétable (il est fixé par le script) mais il n'est pas
*le nôtre*. Le rapport 2,8× doit être reconfirmé à régime comparable avant de
fonder quoi que ce soit dessus. Le rapport de niveau, lui, est robuste : les
deux simulateurs sont dans le même ordre de grandeur et tous deux à ~10× de la
réalité.

## Décomposition du caractère (2026-08-04)

« Sonne mieux » n'est pas une quantité. Le harness mesure maintenant quatre axes
que la forme spectrale ne peut pas trancher, validés d'abord sur signaux
connus : bruit blanc crest **4,8 dB** (théorie 4,77) et pente **+3,10 dB/oct**
(exactement ce que donne un bruit blanc en tiers d'octave) ; ton pur crest
**3,0 dB** (√2 = 3,01), période détectée **1000,0 Hz**, périodicité **1,00**.

| axe | EngineLab Hayabusa, 6 300 tr/min tenus | ES3D Hayabusa, son ralenti |
|---|---|---|
| crest | 18,3 à 24,9 dB | **7,4 dB** |
| modulation | 13,9 à 16,3 dB | 19,0 dB |
| pente spectrale | **+2,4 dB/oct** | **−4,6 dB/oct** |
| **périodicité de l'enveloppe** | **0,38 à 0,50** | **0,92** |
| allumage détecté | 179 à 250 Hz (vrai : 207-211) | 28,7 Hz, propre |
| COV cycle à cycle | *inexploitable, voir ci-dessous* | 7,1 % |

### Ce que ça établit, et ce que ça ne établit pas

- **RÉFUTÉ : « il nous manque la dérivée de rayonnement, nous sommes trop
  sourds ».** Mesuré, **nous sommes plus BRILLANTS** qu'ES3D : +2,4 contre
  −4,6 dB/oct, sept décibels par octave d'écart, dans l'autre sens. L'hypothèse
  d'une pression rayonnée là où il faudrait d*Q*/d*t* prédisait l'inverse.
  Réserve : les régimes diffèrent et la pente en dépend.
- **Le seul écart robuste est la PÉRIODICITÉ de l'enveloppe : 0,92 contre
  0,38-0,50.** C'est une autocorrélation normalisée, donc la moins sensible au
  régime des quatre. Le détecteur verrouille proprement sur ES3D et sur un ton
  pur, et **pas** sur notre rendu : il y annonce 250 Hz là où l'allumage est à
  211. L'enveloppe de notre moteur n'a pas de structure de période d'allumage
  nette.
- **Crest élevé ET périodicité basse est une signature**, pas deux mesures
  indépendantes : 18-25 dB de crest avec 0,4 de périodicité, ce sont des pics
  **isolés et irréguliers**, pas un train d'impulsions régulier. ES3D est
  l'inverse exact — 7,4 dB de crest pour 0,92 de périodicité, donc des
  impulsions modestes mais parfaitement régulières.
- **Notre COV cycle à cycle (79 à 188 %) ne veut rien dire** et ne doit pas être
  cité : il est calculé sur une période que le détecteur n'a pas trouvée. Un
  COV n'est lisible que si la périodicité qui l'accompagne est forte.

Cet écart pointe vers le chemin **événement d'allumage → audio**, pas vers la
géométrie d'échappement.

## Un défaut trouvé dans le harness lui-même

`--rpm` en dessous d'environ 6 000 tr/min **cale le moteur et le harness
imprimait quand même des chiffres** — rendus à 900, 1 250 et 3 000 tr/min : tous
`rpm 0`, l'un à rms 0,000474, et des valeurs de crest, de pente et de COV
publiées comme si de rien n'était.

La cause est ligne 497 : `controls.throttle = t < 0.9 ? 0.2 : 0.85`. Les gaz
sont **fixes à 85 %** quelle que soit la consigne, et seul le frein régule, avec
un intégrateur à gain 1,20/s saturé à 0,92 — la famille de défaut d'absorbeur
que `CLAUDE.md` documente déjà pour les bancs WOT. Une consigne basse le sature,
il écrase le moteur et le cale.

Une garde refuse désormais tout rendu dont le régime final est sous 80 % de la
consigne, et vérifiée non vacante : sortie 1 à 1 250 tr/min, sortie 0 au régime
par défaut. **Les chiffres de la phase 0 ne sont pas touchés** : ils ont tous
été pris au régime par défaut, qui est tenu (6 160 visés, 6 219 à 6 327 atteints).

Conséquence de méthode : **ce harness ne peut pas mesurer un ralenti.** Même si
l'absorbeur était corrigé, une consigne basse tenue à 85 % de gaz serait du
plein gaz en sous-régime, jamais un ralenti — le piège exact que `CLAUDE.md`
décrit pour `--trace <bas régime>`. La comparaison des ralentis demande
`AbClipRenderer`, qui rend déjà des segments de ralenti.

### Ce qui est définitivement écarté

**« ES3D sonne mieux parce que son silencieux atténue vraiment. »** Non : 2,72 dB.
La satisfaction que l'utilisateur entend ne vient pas de là. Restent la
réponse en timbre (mesurée, réelle, 2,8×), la réverbération par convolution
avec une IR de lieu réel, le modèle de rayonnement, et le voicing.
