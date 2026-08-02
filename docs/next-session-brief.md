# Brief de reprise — nouvelle machine (2026-07-28)

> **Session du 2 août 2026 (soir) — quatre lots, dont une réfutation.**
> Lire `docs/rework-validation-log.md` à partir de « Un retard d'allumage ».
>
> 1. **Étincelle.** Un rendez-vous d'allumage exprimé comme une phase absolue ne
>    peut pas représenter un retard au-delà du PMH, et l'ECU en commande
>    (plancher −10°, atteint par cliquetis ou surchauffe). Mesuré : **0 étincelle
>    et 0 allumage** au lieu d'un allumage retardé. Le rendez-vous est désormais
>    une distance de vilebrequin restante.
> 2. **Variabilité cycle-à-cycle : hypothèse RÉFUTÉE.** `cycle_variation_cov = 0`
>    ne veut **pas** dire que les cycles se répètent. Mesuré sans dispersion
>    autorée : COV(PMI) **1,7-7,0 %** à charge partielle et **0,9-12,9 %** à
>    pleine charge. Une fermeture pilotée par la dilution a été prototypée puis
>    **retirée** (inerte : fraction brûlée à l'allumage 0,004-0,026 partout).
>    L'instrument reste : `EngineLabCyclicVariabilityHarness`.
> 3. **Distance d'écoute.** Le catalogue mélangeait scène de mesure et point
>    d'écoute (V12 à 24 m, radial à 6, le reste à 3,5-5). Le couple de micros est
>    remis à l'échelle autour de l'origine à `listening_distance_m` (4 m).
>    Merlin : rms **0,0095 → 0,0538 (+15,1 dB)**, pic observateur 6,0 → 40,8 Pa,
>    sans AGC ni limiteur. No-op exact vérifié à 0.
> 4. **Turbine.** La détente était référencée à l'ambiante, donc la géométrie
>    aval était absente de la puissance arbre par construction. Elle se détend
>    maintenant vers sa propre sortie : 2JZ, sortie 45 → 95 mm, pression de
>    sortie turbine 131,7 → 102,8 kPa, contre-pression 260,5 → 210,3 kPa, couple
>    356,9 → 380,1 Nm. Porte : `EngineLab.TurboDownstreamAuthority`.
>
> **Trois pistes ouvertes, mesurées, non corrigées — ne pas les redécouvrir :**
>
> - **Plafond compresseur.** `1 + (PR-1)*speedRatio²` sature à 2,154 sur le 2JZ
>   (arbre plafonné 1,16×, ratio 1,12). Wastegate fermée, la suralimentation ne
>   bouge à aucun diamètre aval. C'est ce qui bloque encore le gain « gros
>   downpipe = plus de boost ». **Ne pas relâcher ce plafond pour faire passer
>   une porte.**
> - **Dispersion inversée.** Sept moteurs sont plus dispersés à pleine charge
>   qu'à charge partielle. Ce n'est pas l'absorbeur : régime tenu à 0,10-0,48 %
>   pendant que le travail par cycle varie de 5 à 13 %.
> - **Résidu piégé trop faible d'un ordre de grandeur.** 0,4-2,6 % contre 15-25 %
>   pour un moteur de série à charge partielle. Vérifier avec
>   `EngineLabPhysicsPerfHarness --trace 1 --idle`, pas avec le harnais de
>   variabilité.
>
> **Non traité de la demande utilisateur, par ordre :** banc en rampe continue
> style ES2D (le balayage reste par paliers de 250 tr/min,
> `EngineRuntime.cpp` ~845) ; garde-fous physiques ÉCHAP PRO (la validation ne
> vérifie que bornes et topologie) ; afterfire localisé **et** son chemin vers
> l'audio (le chemin temps réel n'a aujourd'hui aucune référence à l'afterfire) ;
> énergie hors mode plan rayonnée au lieu d'être jetée (`DuctModeCutoff`, cause
> du « gros diamètre = étouffé »).


> **Mise à jour Diesel du 2 août 2026.** Le diagnostic essence « mélange trop
> pauvre » ne s'applique plus au Diesel. `fuel.target_afr` y est explicitement
> une limite fumée minimale et une nouvelle carte live
> `fuel.diesel_quantity_mg_per_cycle` expose enfin la vraie commande de couple
> par quantité injectée. À 2 000 tr/min WOT, une réduction de carte
> 52,55 -> 34,15 mg/cyl/cycle fait passer le couple mesuré de 346,82 à
> 216,96 Nm ; une hausse est arrêtée proprement à AFR 17,22 pour une limite
> 16,99. Le gate constructeur Diesel reste à +9,03 % de couple et +1,54 % de
> puissance, donc dans ±15 %. Voir
> `docs/diesel-control-validation-2026-08-02.md`. Le prochain chantier ouvert
> est la contre-pression turbo.

> **Mise à jour banc du 2 août 2026.** Le banc utilisateur dispose maintenant
> d'une préparation progressive lorsqu'il est activé au-dessus de son premier
> point. Le catalogue passe 16/16 balayages complets, sans calage ni
> récupération, et les 24 références constructeur restent dans ±15 %. Voir
> `docs/user-dyno-validation-2026-08-02.md`.

> **Mise à jour physique du 2 août 2026.** Les chutes de couple environ
> 2 000 tr/min avant rupteur sont maintenant reproduites par un harnais à plein
> gaz en 2e/3e et supprimées sur les 14 moteurs routiers du catalogue. Les
> causes étaient le couplage inertiel retardé, une cible d'étincelle mobile et
> une double pénalisation de raté sur l'Audi I5. Voir
> `docs/loaded-acceleration-validation-2026-08-02.md`. Prochaines étapes : banc
> utilisateur progressif, diagnostic/autorité AFR diesel, puis
> contre-pression turbo. Ne pas réactiver la limite d'adhérence : sa
> désactivation est un choix de test volontaire.

> **État final vérifié le 2 août 2026.** Lire d'abord
> `docs/audio-roadmap-final-validation-2026-08-02.md`. Le commit de code testé
> `da33134` passe **16/16** en `--free-run`, zéro overrun, avec le Merlin pire
> cas à **1,167×** (environ **14,3 %** de budget avant l'échéance), puis
> **31/31 tests** en 642,39 s. Le Big Twin a révélé puis verrouillé une vraie
> régression de démarrage : seuls l'afterfire de décélération et le rupteur
> humide opt-in peuvent désormais contourner la porte d'injection. Les chiffres
> plus anciens ci-dessous sont historiques et ne remplacent jamais une nouvelle
> mesure locale. Prochaine action : écoute A/B aveugle des stems pression/jet et
> du voicing catalogue, pas une nouvelle optimisation admission.

> **État sonore vérifié au 1er août 2026, après mesure propre.** La nouvelle
> table `--free-run` au repos passe **16/16** sans overrun ; le pire cas est le
> Merlin à **1,187×** au meilleur passage et **1,086×** au pire des six passages.
> Le catalogue a donc basculé sur le son, sans nouvelle simplification de
> l'admission. Les 16 profils de voicing sont maintenant des données YAML et
> l'A/B conserve aussi les paramètres cachés de mixage. Voir
> `docs/audio-voicing-catalogue-validation-2026-08-01.md`.
>
> La prétendue réduction de la haute bande à un guide par chemin était une
> documentation périmée : le binaire Release vérifie le DAG acoustique complet,
> ses troncs de branche, les changements de section, les états par conduit et
> les sorties spatialisées. Voir
> `docs/exhaust-audio-topology-validation-2026-08-01.md`. Ne pas réimplémenter
> ce graphe. L'afterfire de décélération conditionné par l'ECU est également
> livré et validé : fraction bornée, armement par demande conducteur, témoins
> négatifs et réaction chimique réelle. Voir
> `docs/overrun-afterfire-validation-2026-08-01.md`. Les sources sont désormais
> séparables dans les exports HQ : onde de pression
> (blowdown/réflexions/afterfire) et jet reconstruisent exactement
> `exhaust_dry`, sans altérer le master. Voir
> `docs/exhaust-source-stem-validation-2026-08-01.md`. La prochaine action doit
> être une écoute A/B de ces stems, pas une nouvelle source procédurale.

> **Mise à jour produit du 1er août 2026.** Lire d'abord
> `docs/audio-physics-productization-validation-2026-08-01.md`. Les modèles
> poreux, de variation et d'afterfire ne sont plus seulement des API opt-in :
> AUDIO HQ expose une démo/bypass et leur télémétrie live, ÉCHAP PRO expose le
> garnissage, et le catalogue contient `Audio Physics Lab 689 Twin`. Une A/B
> offline mesure un delta RMS de 0,0640, des cycles 0,858..1,153 et 6,602 g de
> carburant brûlés dans l'échappement. Le rendu par défaut reste neutre.
>
> La première table 16 moteurs de ce lot est **contaminée par une charge
> externe** (`RobloxPlayerBeta` ~171 % d'un cœur logique, Chrome actif) : elle
> passe 16/16 sans overrun mais le Merlin ne vaut que 1,050×. Ne pas réutiliser
> ce chiffre comme référence propre ni prétendre que la marge 10–15 % est
> prouvée ; refaire `--free-run` au repos selon le protocole ci-dessous.

> **État vérifié au 1er août 2026.** Lire d'abord
> `docs/final-validation-2026-08-01.md`. La priorité P2 « sortir le voicing du
> code » ci-dessous est terminée, ainsi que les stems reconstructibles, la carte
> des ordres, l'absorption poreuse passive, la variabilité cycle-à-cycle et
> l'afterfire physique. Le catalogue courant passe 15/15 en mesure locale
> `--free-run`, sans overrun ; le pire cas Merlin vaut 1,128 fois le temps réel,
> soit 11,3 % de marge avant échéance. La suite Release passe 31/31.
>
> **Prochaine priorité : écoute humaine A/B aveugle et calibration mesurée.**
> Ne pas relancer une optimisation admission/pool tant qu'une nouvelle mesure
> locale ne contredit pas la table. Ne pas inventer de réglages de matériau,
> variabilité ou afterfire pour rendre le son spectaculaire : partir de
> références réelles et utiliser les stems pour localiser l'écart. Les alertes
> de contre-pression turbo 2JZ/EJ25/Audi restent visibles et doivent être
> expliquées par une comparaison physique, pas masquées.

Ce fichier est le prompt de démarrage à donner à un agent IA qui reprend le
projet sur une autre machine. Il est écrit pour être lu à froid, sans le
contexte de la session précédente.

> **Priorité en cours au 2026-07-30 : la boucle d'écoute, puis le voicing comme
> donnée.** Lire `docs/audio-listening-protocol-2026-07-30.md`.
>
> Le diagnostic est que le projet a un objectif (du son qui ne sonne pas
> artificiel) et une architecture qui le rend indirect : le timbre est dérivé
> d'une simulation physique, donc **chaque problème de son est un problème de
> physique**, couplé aux 14 moteurs à la fois. C'est ce qui rend chaque
> changement coûteux, pas la difficulté du son.
>
> **P1 — fait.** La boucle de jugement était biaisée : un seul gain BS.1770 sur
> une trajectoire ralenti→rupteur présentait le ralenti **23,3 dB trop bas**, ce
> que le premier auditeur a rapporté comme un défaut du moteur. Les clips sont
> désormais découpés par condition et calés séparément ; les références sont
> appariées par condition (schéma de manifeste 3) ; `--compare` donne un A/B en
> aveugle entre deux moteurs du catalogue. Le rendu n'a pas changé (trajectoire
> −21,5486 → −21,55 LUFS).
>
> **P2 — fait le 1er août 2026.** Le voicing est sorti du code vers
> un `voicing/<moteur>.yaml` (EQ de rayonnement, gains de couches, ordre de
> saturation, largeur stéréo). Deux raisons mesurées : une décision d'écoute ne
> doit pas exiger de trouver quel terme de dynamique des gaz rend un moteur aigu
> et de le faire retomber sur 14 moteurs et 20 tests ; et un changement de
> voicing dans un en-tête coûte **115 s** de rebuild contre 11 s pour un `.cpp`,
> donc faire du voicing une donnée retire le build de la boucle. La « voice
> layer » d'échappement à gain 0,20, décrite ailleurs comme un vestige à
> supprimer, est l'embryon de cette architecture.
>
> **Ne pas faire :** de la performance (six hypothèses déjà mesurées et
> réfutées, marge quasi nulle, risque élevé sur les ralentis) ni de nouvelle
> physique 1-D. Un cache de télémétrie audio a été scopé puis **abandonné après
> mesure** : il n'adressait aucune partie du coût réel.

> **Livraison suivante validée le 29 juillet 2026.** Lire en premier
> `docs/final-validation-2026-07-29.md` : les priorités encore ouvertes de ce
> brief ont été exécutées. La commande prescrite `--free-run --rpm 7000
> --seconds 6` produit désormais **14/14 points valides**, sans dépassement ; le
> pire cas est le Merlin à **1,167×**, soit environ **14,3 % de marge sur
> l'échéance**. Le lot Release passe **25/25 tests** et le banc passe **24/24
> références constructeur** dans ±15 % avec des fenêtres réellement stables.
> L'audio possède maintenant des gates transitoires/rapports, un corpus réel
> CC0 A/B, des erreurs IR visibles et aucune double source turbo. Le véhicule
> modélise le transfert de charge longitudinal ; le schéma NVH accepte des
> modes sourcés sans en inventer pour le catalogue. Les limites prioritaires
> restantes sont l'évaluation humaine aveugle du corpus, l'acquisition de modes
> NVH réellement mesurés et l'investigation des alertes de contre-pression
> turbo. La liste historique plus bas ne doit plus être reprise comme plan.
>
> **Mise à jour après reprise (2026-07-28).** Les priorités 1 à 6 ci-dessous
> ont été exécutées et validées. Lire d'abord
> `docs/validation-2026-07-28.md` pour les tables, le protocole A/B, les
> 20/20 tests et les limites restantes. La cause aval principale de la PMEP a
> ensuite été corrigée : le collecteur conserve maintenant sa quantité de
> mouvement dirigée. La production passe de 1,475 à 1,046 bar à 6 000 tr/min,
> mais l'oracle reste à 0,990 pour une cible 0,750. Lire aussi
> `docs/exhaust-pmep-investigation-2026-07-28.md`. Ne pas recommencer une
> optimisation d'admission ou de pool sans contredire les nouveaux garde-fous.
> Une protection de charge préserve en plus l'échappement/audio : elle espace
> seulement le calcul thermique des parois d'admission après six retards
> consécutifs, et restaure la cadence normale avec hystérésis.
>
> **État final de cette reprise.** La liste de priorités plus bas est conservée
> comme historique, mais elle n'est plus une liste de tâches. La production
> utilise maintenant un maillage admission ciblé à **95 mm**, **2 workers**
> jusqu'au V8 et **3 workers** à partir de 10 cylindres. Le lot de politique
> mesurait le Merlin à 1,113× ; sur l'exact binaire final, le meilleur de six
> passages vaut **1,147×**. Un passage plein à 1,082× est conservé comme preuve
> de charge externe : `CompatTelRunner` occupait un cœur et le CP2 témoin
> chutait simultanément de 2,574× à 1,894×.
> Le maillage 95 mm s'écarte au plus de **4,460 %** de l'oracle 30 mm/RK2 ;
> même la protection thermique 600 µs reste à **4,916 %**. Le candidat 120 mm a
> été refusé à 16,580 %. Le rendu audio complet passe avec le chemin
> physique/topologique/modal/ondulatoire, zéro perte et une similarité spectrale
> maximale de 0,642. Enfin, la reprise après coupure de décélération compense
> désormais la part du pulse neuf retenue dans le film de paroi : le Merlin
> récupère avec le seuil DFCO proportionnel d'origine, sans calibration spéciale.
> Les preuves et commandes sont dans `docs/validation-2026-07-28.md`.

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

La machine avait été annoncée comme un **i5-11600**, mais Windows l'identifie
comme un **i5-10600** (6 cœurs / 12 threads, bureau). Ce désaccord de nom ne
change pas le protocole : seules les mesures locales comptent.
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

La suite de 20 tests est verte après la reprise.
`ctest --test-dir out/build/windows-vs2022 -C Release`.

## Ce qui est déjà réfuté — ne pas réessayer

### Mise à jour 2026-08-02 — base simulation stabilisée

Les sursauts pré-rupteur, le démarrage du banc produit, l'autorité Diesel, le
partage turbine/wastegate et l'affichage mobile des chemises radiales sont
corrigés. Validation finale : 14/14 moteurs routiers sans sursaut sur les deux
rapports testés, banc 16/16, références 24/24, `ctest` 35/35 et catalogue audio
temps réel 16/16. Le Merlin reste le bord CPU mesuré. Lire impérativement
`docs/foundation-recovery-final-validation-2026-08-02.md` avant de reprendre.

### Mise à jour 2026-08-02 — pression motrice turbo

Le faux diagnostic de contre-pression turbo et le partage non conservatif de
débit wastegate sont corrigés. En troisième plein gaz, les quatre turbos du
catalogue mesurent un rapport pression motrice/MAP de 1,11 à 1,39 ; les huit
points constructeur turbo/TDI passent dans ±15 %. Ne pas tenter de résoudre la
pression amont turbine en agrandissant uniquement le cat-back. Voir
`docs/turbo-backpressure-validation-2026-08-02.md`.

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
4. ~~**La bande passante du couplage échappement** (tâche #32) : l'intervalle est
   plafonné à 250 µs, ce qui limite la bande physique à ~2190 Hz.~~ **FAIT le
   2026-07-28.** Le cap est passé à **125 µs**
   (`maximumLowSpeedExhaustCouplingSeconds.value_or(125.0e-6)`) : Nyquist
   physique **5 640 Hz** sur le LS3 et **3 480 Hz** sur le Merlin. Ne pas
   reprendre ce point comme ouvert — le « plafond de 2,19 kHz » ne décrit plus
   le code livré. Voir `docs/validation-2026-07-28.md` et §14 de
   `docs/thermoacoustic-architecture.md`.
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
