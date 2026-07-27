# Brief de reprise — nouvelle machine (2026-07-28)

Ce fichier est le prompt de démarrage à donner à un agent IA qui reprend le
projet sur une autre machine. Il est écrit pour être lu à froid, sans le
contexte de la session précédente.

---

Tu reprends EngineLab, un simulateur de moteur thermique temps réel dont
l'objectif principal est la **qualité du son**, et l'objectif secondaire une
physique à ±10-15 % du réel. La fonctionnalité qui compte le plus pour
l'utilisateur est de pouvoir composer des échappements arbitraires (un 4-2-1 sur
une banque d'un V8 et un 4-1 sur l'autre, par exemple) et d'entendre la
différence.

## Avant toute chose : lis ces trois fichiers en entier

1. `CLAUDE.md` — ce ne sont pas des conventions de style, c'est la liste des
   pièges qui ont coûté du temps ici. Plusieurs « bugs » évidents du code sont
   des compensations délibérées qu'il ne faut pas « corriger ».
2. `docs/physics-audit.md` — les mesures, et surtout les hypothèses réfutées.
   Beaucoup d'idées séduisantes y sont déjà mortes, avec les chiffres.
3. `docs/rework-validation-log.md` — le journal chronologique.

La règle qui prime sur tout : **mesurer avant de corriger.** Ce code se lit comme
s'il était plein de bugs ; la lecture induit en erreur, la mesure non.

## Changement de machine — c'est la première chose à faire

La session précédente tournait sur un **Ryzen 7 8840U** (8 cœurs / 16 threads,
puce mobile 15-28 W). Cette machine s'effondrait thermiquement : le même banc sur
le même binaire mesurait 467 ns/cellule à froid et **1404 après quelques heures**,
un facteur trois. Toutes les valeurs absolues de performance dans la
documentation en portent la marque.

La nouvelle machine est un **i5-11600** (6 cœurs / 12 threads, 65 W, bureau).
Deux conséquences opposées, et il faut les mesurer et non les supposer :

- fréquence soutenue bien meilleure, pas d'effondrement thermique → le
  monothread devrait nettement progresser, et surtout **les mesures redeviennent
  fiables** ;
- mais 12 threads au lieu de 16, et le pool de workers vaut
  `min(cylindres - 1, hardware_concurrency/2 - 1)`, soit un plafond de **5 au
  lieu de 7**. Le V8 et le V12 reçoivent donc *moins* de workers qu'avant.

**Première action, avant tout travail de code :**

```
cmake --build out/build/windows-vs2022 --config Release
out/build/windows-vs2022/tools/Release/EngineLabRealtimeBudgetHarness --free-run --rpm 7000 --seconds 6
```

Publie la table complète. Elle redéfinit le problème : il est possible que la
majorité du catalogue passe déjà le temps réel, auquel cas la priorité bascule
du CPU vers le son. Ne réutilise **aucun** chiffre absolu de la documentation
comme référence — ils viennent d'une machine différente et dégradée.

## Protocole de mesure — non négociable

Ces règles ont été apprises en produisant de faux résultats.

- **Jamais de comparaison à un chiffre écrit ailleurs** (doc, message de commit,
  message précédent). Seule une mesure A/B faite dans la même heure compte.
  Protocole : `git stash` → build → mesure → `git stash pop` → build → mesure ;
  ou mieux, copier les deux binaires côte à côte (`rt_avant.exe`, `rt_apres.exe`
  — ils sont liés statiquement) et les **alterner**.
- **Toujours un témoin nul** : un moteur que le changement ne peut pas atteindre
  par construction, et qui doit donc lire 0 %. Sans lui, un « +23 %, trois tours
  sur trois » a failli être publié alors que son témoin lisait +8 %.
- **`A, B, B, A` n'est pas contrebalancé.** La position 1 paie le démarrage à
  froid et revient toujours à A. Jette un passage d'échauffement et alterne la
  variante de tête.
- **Minimum sur N essais, jamais la moyenne** : la boucle est déterministe et
  liée au CPU, l'interférence ne peut qu'ajouter du temps. Le minimum sur six
  fait passer la répétabilité du banc de ~16 % à 2-5 %.
- `EngineLabIntakeDuctBench` imprime un **checksum bit-exact**. Une optimisation
  qui laisse les six checksums inchangés n'est prouvablement pas un changement de
  physique — trente secondes au lieu d'une reprise de catalogue de dix minutes.
- Attention : `EngineLabRealtimeBudgetHarness` ne fait tourner **ni callback
  audio ni interface**. Il ne peut pas voir le coût d'une sur-souscription de la
  machine réelle.

## Où en est le travail

Branche `codex/audio-physics-phases-4-5`, trois commits récents :

- `c03b6d3` — les runners d'admission 1-D avancent en concurrence. La difficulté
  n'était pas le threading mais le plénum partagé : geler, retarder ou moyenner
  son état casse l'invariant « un moteur à l'arrêt se stabilise à l'ambiant ».
  Ce qui tient est un escalier de Gauss-Seidel reconstruit depuis une prédiction
  au même instant, prédicteur Heun, deux tours.
- `2c18cb3` — l'échange thermique de paroi du conduit est sous-cadencé à 150 µs.
  Solveur de conduit −24 %. Gains entrelacés : CP2 +35 %, LS3 +21 %, Merlin +9 %.
  Physique déplacée : EGT sous 0,15 % de moyenne par moteur.
- `944e2cd` — deux réfutations (voir ci-dessous).

La suite de 19 tests était verte. `ctest --test-dir out/build/windows-vs2022 -C Release`.

## Ce qui est déjà réfuté — ne pas réessayer

- **Élargir le SIMD** : `/arch:AVX` 616 ns/cellule et `/arch:AVX2` 578 contre
  533 pour la base, checksums identiques. Le solveur est limité par la *latence*
  de chaînes dépendantes.
- **Compter les divisions** : supprimer les quatre divisions de `massFractions`
  (~31 % de toutes les divisions du solveur) vaut **1,4 %**, checksums
  identiques. Elles sont indépendantes entre elles, donc elles se recouvrent au
  débit et non à la latence. Le seul poste latence-critique est la chaîne
  `density → velocity → énergie interne → température → pression → gamma →
  vitesse du son → sqrt`, et elle ne se raccourcit pas.
- **Grossir le maillage runner** : ×1,26 sur le V8, mais +16,7 % de VE sur le
  Big Twin et −22,3 % de couple sur le Merlin.
- **Quatre formulations de « ram » d'admission** — voir « L'inertance de runner »
  dans l'audit avant d'en proposer une cinquième.
- Le bloc MUSCL est le plus gros poste restant (16-29 %) mais **indisponible** :
  annuler les pentes, c'est retomber à l'ordre 1.

## Priorités, dans cet ordre

1. **Mesurer le catalogue sur la nouvelle machine** (ci-dessus). Tout le reste
   en dépend.
2. **Le plafond de threads**, qui devient contraignant sur 12 threads (5 workers
   au lieu de 7 pour le V8 et le V12) et qui redevient *mesurable* sans
   effondrement thermique. Il n'a pas pu être tranché avant : trois protocoles,
   témoin nul à +8 % puis −19 %. À reprendre avec le protocole ci-dessus.
   Attention à ne pas sur-souscrire : l'application réelle a un thread audio et
   une interface que le banc ne simule pas.
3. **La convergence du maillage d'admission.** C'est le point qui menace
   directement l'objectif de ±10-15 % : l'erreur du schéma numérique est du même
   ordre que la cible de précision sur certains moteurs (voir la réfutation du
   maillage grossier). Tant que ce n'est pas réglé, on ne sait pas distinguer une
   erreur de physique d'une erreur de discrétisation.
4. **La bande passante du couplage échappement** (tâche #32) : l'intervalle est
   plafonné à 250 µs, ce qui limite la bande physique à ~2190 Hz. C'est le point
   qui menace directement l'objectif de son, et c'est ce que le budget CPU gagné
   sert à financer.
5. **Construire des courbes de référence** avant d'ajouter des modes.
   `CatalogPhysics` vérifie que les moteurs tournent et ne divergent pas ; il ne
   vérifie **aucun couple absolu**. La marge de ±10-15 % n'est donc mesurée nulle
   part de façon systématique.
6. **Rendre le ralenti robuste.** Les ralentis du catalogue sont des attracteurs
   marginaux : un ULP fait basculer un moteur entre se stabiliser et caler. Ce
   n'est pas un bug à chasser, c'est un régulateur qui vit trop près de son point
   de décrochage. C'est une taxe qui grossit à chaque optimisation.

## Deux observations stratégiques à garder en tête

**80 % du CPU part du côté qui influence le moins le son.** L'admission 1-D
consomme 75 à 84 % du pas de calcul, l'échappement 4 à 9 %. Or le son vient de
l'échappement et de la pression cylindre ; l'admission n'y contribue qu'à travers
un scalaire lentement variable, la masse d'air admise. Cette allocation n'a jamais
été décidée, elle est arrivée par accumulation. La question « l'admission peut-elle
avancer sur une cadence plus grossière que l'échappement ? » est **ouverte** — la
note qui la décourage dans `CLAUDE.md` est un avertissement raisonné, pas une
mesure. Si le CPU redevient contraignant, c'est le premier endroit à regarder.

**Les deux objectifs n'ont pas les mêmes exigences.** Un son crédible ne demande
pas ±15 % sur le couple : il demande que la structure d'ondes soit juste — ordre
d'allumage, longueurs de tubes, résonances, forme de la bouffée de blowdown.
C'est bien plus tolérant. Le son peut atteindre un excellent niveau bien avant la
précision dyno, et les deux peuvent avancer à des rythmes différents. Ne bloque
pas le son en attendant la physique.

Et une attente à poser honnêtement : **±10-15 % sur un moteur arbitraire créé par
un utilisateur, sans calibration spécifique, est très ambitieux.** Ce qui est
réaliste, c'est ±10-15 % sur des moteurs ressemblant à ceux contre lesquels on
calibre, avec une dégradation progressive en s'en éloignant.

## Ce qui est hors périmètre pour l'instant

Le mode « expert » de reprogrammation ECU est un projet à part entière, de la
taille du simulateur physique, et il n'apporte rien au son. Le mode simplifié et
le mode avancé partagent le même cœur et sont peu coûteux. Garder le troisième
dans la vision, pas dans le plan.

## Contraintes de build

- Les avertissements sont des erreurs. Un build vert et un `ctest` vert sont la
  barre minimale.
- Un build Release parallèle non bridé peut épuiser le tas du compilateur MSVC
  (`C1060`) sur trois unités de traduction LTO. Utiliser `--parallel 1` en cas
  d'échec — c'est une limite mémoire de build, pas une erreur de source.
- Vérifier que la cible s'est bien reliée : `--target X` avec un mauvais `X`
  échoue avec `MSB1009` sans rien construire, et `ctest` relance alors
  l'**ancien** binaire.
