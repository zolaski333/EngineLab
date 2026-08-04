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

## Ce que la mesure doit trancher

Une seule question, et elle oriente deux travaux très différents :

- **Si l'écart de forme d'ES3D dépasse nettement nos 11,9-14,4 dB**, alors sa
  réponse à la géométrie est réellement supérieure et il faut chercher où.
- **S'il est comparable**, alors la satisfaction que l'utilisateur y entend ne
  vient PAS de la sensibilité géométrique, et la chercher là est une impasse.
  Restent le niveau, la réverbération par convolution avec une IR de lieu réel,
  le modèle de rayonnement, et le voicing — quatre pistes distinctes qu'aucune
  lecture de code ne peut départager.
