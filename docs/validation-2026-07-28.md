# Validation de reprise — 2026-07-28

> **Addendum de la même journée.** La section
> « Correction du collecteur dirigé » en fin de document remplace les valeurs
> PMEP et points constructeur qui dépendaient du collecteur historique
> parfaitement mélangé. Les tables de performance initiales restent un
> instantané de leur commit ; une nouvelle table est exigée après la correction.

Ce rapport clôt la reprise demandée dans `docs/next-session-brief.md`. Il
distingue les faits mesurés, les choix retenus et les limites encore ouvertes.
Les valeurs absolues ci-dessous sont un instantané de cette machine ; elles ne
doivent pas servir de référence à une future optimisation sans une nouvelle
mesure A/B entrelacée dans la même heure.

## Verdict

- Le catalogue complet tient au moins **1,10× le temps réel** à 90 % du régime
  limite, sur les réglages de production. Le pire cas est le LS3 à 1,102×,
  suivi du Merlin à 1,105×.
- L'admission allégée reste à **13,435 % maximum** de l'ancienne configuration
  numérique 30 mm/RK2/chaque sous-pas sur le cas sévère du runner doublé, et à
  4,022 % maximum sur le moteur standard.
- Les quatre critères indépendants de respiration réelle passent : pic de VE
  1,099, pic à 5 832 tr/min, VE haut régime 1,090 et déplacement du pic de
  26,251 % quand la longueur de runner double.
- Les **14 points constructeur** actuellement définis passent dans ±15 %, avec
  chaque régime réellement tenu à ±2 %. Ils couvrent sept familles ayant une
  variante réelle défendable.
- Le plafond de **5 workers** est conservé. Le banc de capacité seul met Merlin
  à égalité, mais le chemin applicatif réel montre moins de coût callback moyen
  et environ 20 % de retards physique en moins à 5 workers sur LS3 et Merlin.
- Le plafond de couplage échappement passe de 250 à 125 µs. La fréquence de
  couplage LS3 passe de 3 760 à 11 280 Hz (Nyquist physique 1 880 à 5 640 Hz) ;
  celle du Merlin de 3 480 à 6 960 Hz (1 740 à 3 480 Hz).
- Le chemin audio livré utilise le réseau physique complet, la structure
  modale et l'admission ondulatoire, sans fallback, dropout, pression perdue ni
  limiteur. Le rendu complet distingue les moteurs avec une similarité spectrale
  maximale de 0,759 (seuil d'échec 0,985).
- Le ralenti des 14 moteurs passe le démarrage, la tenue libre et le retour
  après un coup de gaz, sans exemption.
- La build Release, les **20/20 tests** et un démarrage caché de l'exécutable
  pendant cinq secondes passent.

## Machine réellement mesurée

Windows rapporte :

| Champ | Valeur |
|---|---|
| CPU | Intel Core i5-10600 @ 3,30 GHz |
| Cœurs / processeurs logiques | 6 / 12 |
| `hardware_concurrency` | 12 |

Le brief utilisateur annonçait un i5-11600. L'OS identifie sans ambiguïté un
**i5-10600**. Les décisions utilisent donc seulement le comportement mesuré,
jamais le nom supposé du processeur.

## Instrument de budget corrigé

La première table brute demandée a révélé un défaut du banc : il demandait une
variation relative depuis un snapshot encore à zéro alors que le dyno interne
partait de 2 500 tr/min. Plusieurs moteurs dépassaient donc la cible annoncée et
atteignaient leur limiteur. Cette table a été publiée avant toute proposition,
mais elle est **invalidée comme référence**.

`EngineLabRealtimeBudgetHarness` possède maintenant :

- un setter absolu de régime, borné par `min(redline, revLimit)` ;
- une cible relative propre à chaque moteur ;
- une stabilisation filtrée de 1,5 seconde simulée dans la bande ±2 % ;
- une nouvelle vérification du régime moyen pendant la fenêtre mesurée ;
- le nombre réel de workers, la cible et le régime moyen dans chaque ligne ;
- un refus explicite des points invalides ;
- les variantes admission/échappement nécessaires aux A/B ;
- un mode libre qui mesure la capacité au lieu de saturer à 1,0.

## Catalogue final sur cette machine

Commande :

```text
EngineLabRealtimeBudgetHarness --catalog-root . --relative-rpm 0.90 \
  --warmup 1 --seconds 6 --free-run --enforce 1.10
```

| Moteur | Cible | Régime moyen | Workers | Facteur |
|---|---:|---:|---:|---:|
| K20A-like 2.0 I4 VTEC | 7 740 | 7 729 | 3 | 1,422 |
| 2JZ-GTE-like 3.0 I6 Turbo | 6 300 | 6 296 | 5 | 1,567 |
| LS3-like 6.2 Crossplane V8 | 5 940 | 5 945 | 5 | **1,102** |
| EJ25-like 2.5 Flat-4 Turbo | 6 120 | 6 110 | 3 | 1,907 |
| Audi I5-like 2.5 Turbo | 6 390 | 6 383 | 4 | 1,789 |
| Hayabusa-like 1.3 I4 | 10 080 | 10 067 | 3 | 1,450 |
| Big Twin-like 1.9 V2 | 5 040 | 5 033 | 0 | 3,423 |
| Merlin-like 19.8 V12 Scaled | 2 880 | 2 880 | 5 | **1,105** |
| Aircooled-like 3.6 Flat-6 | 6 660 | 6 661 | 5 | 1,424 |
| Radial-like 6.5 R5 | 2 160 | 2 160 | 4 | 2,045 |
| Yamaha CP2 MT-07-like 689 Twin | 9 000 | 8 985 | 0 | 2,639 |
| Yamaha CP3 MT-09-like 890 Triple | 9 900 | 9 890 | 2 | 1,924 |
| Yamaha CP4 MT-10-like 998 Crossplane I4 | 10 800 | 10 797 | 3 | 1,399 |
| VW EA288-like 2.0 TDI I4 | 4 500 | 4 481 | 3 | 2,455 |

Tous les points sont valides et aucun overrun n'est produit en mode capacité.
Le plancher de 1,10 signifie 10 % de marge brute ; il ne faut pas le rebaptiser
15 %.

## Réduction de l'admission et garde-fou

Politique de production :

- maillage adaptatif ciblé à 75 mm, 3 à 12 volumes par runner ;
- reconstruction spatiale MUSCL conservée ;
- intégration temporelle Euler conservative pour ce réseau réduit ;
- couplage multirate maximal 400 µs contre une condition limite moyennée dans le
  temps ;
- flush forcé à la fermeture de soupape ;
- deux tours de reconstruction du plénum, nécessaires aux invariants.

Oracle automatisé : 30 mm, RK2, couplage à chaque sous-pas, deux tours. Le même
test balaie le moteur standard et un runner deux fois plus long.

| Contrôle | Résultat | Limite |
|---|---:|---:|
| Erreur VE max, moteur standard | 4,022 % | 15 % |
| Erreur VE max, runner 2× | **13,435 %** | 15 % |
| Pic VE | 1,099 | 0,85–1,15 |
| Régime du pic | 5 832 tr/min | ≥ 2 880 |
| VE à 6 000 tr/min | 1,090 | ≥ 0,879 |
| Déplacement du pic avec runner 2× | 26,251 % | ≥ 15 % |

Le test CTest exécute maintenant obligatoirement
`--enforce-tuning --compare-oracle`. Une réduction future qui sort de cette
enveloppe échouera.

## Plafond du pool : A/B contrebalancé

Protocole pour chaque moteur et variante : un passage jeté, puis
ABBA/BAAB/ABBA, six valeurs par variante. Le CP2 est le témoin nul : avec deux
cylindres il crée réellement zéro worker, que l'argument demande 5 ou 7.

### Capacité seule

| Moteur | Workers demandés | Six facteurs | Meilleur | Médiane |
|---|---:|---|---:|---:|
| Merlin | 5 | 1,135 / 1,122 / 1,130 / 1,124 / 1,139 / 1,128 | 1,139 | 1,129 |
| Merlin | 7 | 1,085 / 1,099 / 1,091 / 1,117 / 1,141 / 1,116 | 1,141 | 1,108 |
| LS3 | 5 | 1,114 / 1,148 / 1,141 / 1,120 / 1,133 / 1,145 | **1,148** | **1,137** |
| LS3 | 7 | 1,106 / 1,128 / 1,104 / 1,068 / 1,121 / 1,131 | 1,131 | 1,114 |
| CP2 témoin, 0 réel | 5 | 2,680 / 2,684 / 2,675 / 2,661 / 2,690 / 2,675 | 2,690 | 2,678 |
| CP2 témoin, 0 réel | 7 | 2,686 / 2,685 / 2,668 / 2,684 / 2,681 / 2,683 | 2,686 | 2,684 |

Le minimum de temps sur Merlin seul est une égalité à 0,2 %, inférieure à la
dispersion du témoin. LS3 favorise 5.

### Chemin applicatif réel

Le même protocole fait tourner `EngineRuntime` et le callback 256 échantillons à
48 kHz en concurrence.

| Moteur | Workers | Meilleure moyenne callback | Meilleur p95 | Moins de retards physique | Dropouts + pressions perdues |
|---|---:|---:|---:|---:|---:|
| Merlin | 5 | **2 432,3 µs** | 2 979,0 µs | **371** | 0 |
| Merlin | 7 | 2 604,1 µs | **2 914,0 µs** | 463 | 0 |
| LS3 | 5 | **1 869,7 µs** | **2 312,7 µs** | **391** | 0 |
| LS3 | 7 | 2 010,9 µs | 2 347,3 µs | 487 | 0 |
| CP2 témoin, 0 réel | 5 | 800,7 µs | 852,6 µs | 0 | 0 |
| CP2 témoin, 0 réel | 7 | 799,7 µs | 851,4 µs | 0 | 0 |

Sur Merlin, le meilleur p95 isolé favorise 7 de 65 µs, mais les six moyennes
callback sont systématiquement plus basses à 5, et le meilleur compteur de
retards baisse de 19,9 %. Sur LS3, les trois indicateurs favorisent 5. Le témoin
est inchangé. Le plafond de production reste donc **5** sur cette machine à six
cœurs.

## Son et bande passante échappement

| Moteur | Couplage 125 µs | Ancien 250 µs | Oracle | Nyquist production / ancien |
|---|---:|---:|---:|---:|
| LS3 | 11 280 Hz | 3 760 Hz | 11 280 Hz | 5 640 / 1 880 Hz |
| Merlin | 6 960 Hz | 3 480 Hz | 6 960 Hz | 3 480 / 1 740 Hz |

Pour le Merlin, le rendu 125 µs est identique à l'oracle sur les métriques
publiées et la fraction haute bande passe de 1,471 % à 2,179 %. Pour le LS3, la
fraction haute bande passe de 5,220 % à 9,216 %. La similarité cosinus LS3 n'est
pas utilisée comme preuve de supériorité perceptuelle : les trois trajectoires
moteur diffèrent légèrement. La preuve ferme est la cadence physique, la
continuité des données et l'absence de fallback/dropout.

Clips A/B/oracle :

```text
out/validation/audio-proof-2026-07-28/
  LS3/coupling-125us/
  LS3/coupling-250us/
  LS3/coupling-oracle/
  Merlin/coupling-125us/
  Merlin/coupling-250us/
  Merlin/coupling-oracle/
```

Le verdict « excellent à l'oreille » reste nécessairement humain. Ces clips
permettent précisément de le juger sans confondre ancien couplage, production
et oracle.

## Points constructeur

| Famille | Erreur couple | Erreur puissance | Résultat |
|---|---:|---:|---|
| Honda K20A | −11,051 % | −14,737 % | PASS |
| Toyota 2JZ-GTE export | −14,541 % | −13,060 % | PASS |
| GM LS3 | −2,147 % | −14,484 % | PASS |
| Yamaha CP2 | −11,756 % | +10,744 % | PASS |
| Yamaha CP3 | −0,955 % | −1,792 % | PASS |
| Yamaha CP4 | −0,375 % | −13,552 % | PASS |
| VW EA288 2.0 TDI 110 kW | +7,755 % | −0,052 % | PASS |

Les sources primaires et les variantes exactes sont listées dans
`tests/reference-data/catalog-manufacturer-sources.md`. Les moteurs génériques
ou explicitement scalés qui n'ont pas de jumeau défendable ne sont pas maquillés
en validation constructeur.

## Validation Release

```text
cmake --build out/build/windows-vs2022 --config Release -- /nr:false /m:1
ctest --test-dir out/build/windows-vs2022 -C Release --output-on-failure
```

Résultat : **20/20 tests, 0 échec, 479,42 s**.

Le log complet conservé est
`out/validation/ctest-full-2026-07-28.log`, SHA-256
`AE4496F6808FAAA89577079F741115D961B1BAB4297EB95FF214DBBA988D68D3`.

Après le remplacement de l'ancre TDI intermédiaire par la spécification
Volkswagen 320 Nm et le durcissement de l'export WAV, les deux tests touchés ont
été relancés : **2/2**, 125,13 s. Leur log est
`out/validation/ctest-final-targeted-2026-07-28.log`, SHA-256
`525FD5002757E3F594AEBCEB32556C0A9BE959AFB740CFEC64DD56A240F80ABA`.

L'exécutable Release est
`out/build/windows-vs2022/src/app/EngineLabApp_artefacts/Release/EngineLab.exe`,
8 382 464 octets, SHA-256
`A408BF3791BF731C70F612AB6BB3797A029F4F5E7D4FC00E332EC10295A629F9`.
Il est resté vivant après cinq secondes de lancement caché.

## Limites restantes — ne pas les masquer

1. `EngineLab.GasExchange` mesure encore `|PMEP| = 1,480 bar` à 6 000 tr/min
   pour une cible littérature de 0,750 bar. L'oracle de couplage donne 1,410 :
   la cadence n'explique plus que 4,7 % de l'écart. Le défaut restant est dans
   le réseau pulsé/sa fermeture, pas dans la simplification d'admission.
2. Le seuil constructeur couvre sept familles. Définir un vrai référent est un
   préalable pour l'EJ25, l'Audi I5, le Hayabusa et le Flat-6 ; le Big Twin,
   le Merlin « Scaled » et le radial générique ne doivent pas recevoir une
   référence inventée.
3. Le catalogue passe 1,10×, pas 1,15× : LS3 et Merlin ont une marge brute
   d'environ 10 %. Le chemin applicatif n'a perdu aucun échantillon pendant les
   validations, mais une charge externe lourde peut encore réduire cette marge.
4. Les avertissements de contre-pression restent cohérents avec la PMEP haut
   régime ci-dessus. Ils ne doivent pas être supprimés pour rendre l'interface
   verte.

L'investigation détaillée et les leviers réfutés sont conservés dans
`docs/exhaust-pmep-investigation-2026-07-28.md`.

## Correction du collecteur dirigé

L'investigation suivante a isolé une perte non physique : la jonction 0-D
conservait masse et énergie mais annulait systématiquement sa quantité de
mouvement. À géométrie identique, elle transformait donc un collecteur dirigé
en plénum stagnant.

La production fait désormais évoluer le résidu axial après compensation de la
pression statique de paroi. Trois preuves indépendantes sont conservées :

1. l'état uniforme à l'ambiante reste invariant et ne crée ni énergie ni
   quantité de mouvement ;
2. au banc K20 à 135 kPa, le collecteur porte 128,851 m/s, le débit passe de
   0,139 à 0,295 kg/s et `K/K_géométrique` de 5,549 à 1,494, avec bilans masse
   et énergie fermés ;
3. sur l'I4 à 6 000 tr/min, la PMEP de production passe de 1,475 à
   **1,046 bar** et la pression moyenne échappement de 169,230 à
   **111,953 kPa**.

Le contrôle `--well-mixed-junctions` reproduit l'ancien modèle dans les bancs.
Le gate constructeur, après réancrage local de la turbulence de chambre du
CP2, passe **14/14** :

| Famille | Erreur couple | Erreur puissance | Résultat |
|---|---:|---:|---|
| Honda K20A | −1,826 % | −4,644 % | PASS |
| Toyota 2JZ-GTE export | −9,668 % | −3,943 % | PASS |
| GM LS3 | +4,641 % | −8,517 % | PASS |
| Yamaha CP2 | −10,094 % | +12,365 % | PASS |
| Yamaha CP3 | +8,991 % | +11,524 % | PASS |
| Yamaha CP4 | +9,990 % | +2,670 % | PASS |
| VW EA288 2.0 TDI 110 kW | +8,112 % | +0,720 % | PASS |

La limite restante est plus étroite qu'avant : l'oracle PMEP passe à
0,187 bar à 2 500 tr/min, mais reste à **0,990 bar à 6 000 tr/min** pour une
cible de 0,750. Le plafond d'aire de tête atteint un plateau à 0,869 bar ; un
multiplicateur global de soupape n'est donc pas livré. La prochaine étape
physique défendable est une carte de coefficient de débit dépendant de la levée,
du rapport de pression et du sens.

### Preuve acoustique de la correction

`EngineLabAudioRenderHarness --junction-comparison LS3` rend le même moteur,
avec le même trajet acoustique, une fois avec le collecteur dirigé et une fois
avec le contrôle parfaitement mélangé.

| LS3, fenêtre stabilisée | Dirigé production | Mélangé historique |
|---|---:|---:|
| RMS gauche | 0,028212 | 0,025978 |
| Fraction haute bande | 5,883 % | 9,216 % |
| Crête pré-limiteur | 0,2981 | 0,1974 |
| Similarité cosinus spectrale entre les deux | **0,937184** | **0,937184** |

Les deux chemins activent le réseau physique, la topologie complète, la
structure modale et l'admission ondulatoire. Ils ont chacun zéro fallback,
dropout de frontière, pression perdue, événement tardif et échantillon limité.
La différence est donc arrivée par la télémétrie physique, pas par un changement
de renderer ou un mécanisme de sécurité.

Les clips sont conservés dans :

```text
out/validation/audio-junction-ab-2026-07-28/LS3/
  junction-directed/
  junction-well-mixed/
```

Le cycle transitoire Big Twin de 12 s passe également : 1 337 tr/min au premier
ralenti, pointe à 4 036 tr/min, retour à 780 tr/min, zéro événement/pression
perdu, zéro événement tardif et zéro action du limiteur. Le clip
`idle-start-rev-return.wav` est sous
`out/validation/audio-transient-2026-07-28/BigTwin/`.

Ces chiffres prouvent une différence acoustique objective et un chemin sain ;
ils ne prétendent pas remplacer le verdict d'écoute sur la préférence sonore.

## Protection dynamique du budget sonore

Le catalogue ne possède qu'une marge d'environ 10 % sur ses cas lourds, et la
machine peut être occupée par l'interface, le callback ou une autre
application. L'objectif du garde-fou n'est pas de rendre un moteur moins cher
en permanence : il est de préserver la télémétrie de pression cylindre quand
le thread physique manque réellement ses échéances.

Deux candidats ont été mesurés puis refusés :

- admission 400 → 500 µs : physique dans l'enveloppe, mais LS3
  1,001× → 0,998× au premier A/B, donc aucun gain ;
- reconstruction de plénum 2 → 1 tour : six répétitions contrebalancées donnent
  le même meilleur facteur LS3, 1,030× dans les deux variantes. Le CP2, témoin
  nul qui n'exécute aucun de ces tours, reste dans le même bruit de machine.

Le mécanisme retenu ne touche qu'à l'échantillonnage du coefficient d'échange
thermique des parois d'admission :

- politique normale : **150 µs** ;
- après six échéances consécutives manquées : **600 µs** ;
- retour à 150 µs seulement après 480 trames consécutives sans retard et avec
  au plus 85 % du budget utilisé ;
- désactivé pendant un dyno, une pause et les mesures `--free-run`.

Le transfert thermique à deux capacités accumule la durée entre évaluations :
aucune énergie gaz/paroi n'est jetée. Le maillage, le schéma MUSCL, les ondes
d'admission, les soupapes, le couplage échappement 125 µs et chaque pression
cylindre restent inchangés.

### Bornes physiques du mode protégé

Le balayage 600 µs contre son oracle 30 mm/RK2/sous-pas reste à **14,158 %**
maximum (limite 15 %), et les quatre critères de respiration passent. La
comparaison directe 150/600 µs de la configuration réduite déplace la VE au
plus de 5,49 % sur le runner standard et 7,69 % sur le runner doublé ; le pic
passe de 1,099 à 1,142 et de 5 832 à 5 891 tr/min.

### Chemin applicatif réel

Même binaire, LS3, thread physique et callback 256 échantillons à 48 kHz en
concurrence :

| Politique | Retards physique / 1 440 | Callback p95 | Pressions perdues | Fallback |
|---|---:|---:|---:|---:|
| 150 µs fixe | 607 | 2 986,5 µs | 0 | 0 |
| 600 µs fixe | **582** | **2 963,9 µs** | 0 | 0 |
| adaptative | 597, une activation | 2 976,9 µs | 0 | 0 |

Le mode fixe protégé réduit ici les retards de 4,1 %. L'adaptatif se situe
logiquement entre les deux, puisqu'il commence à 150 µs et exige une surcharge
confirmée avant d'agir. C'est un filet de quelques pourcents, pas une promesse
de compenser une machine massivement sous-dimensionnée.

## Addendum final — état de production autoritaire

Cette section remplace, pour l'état final du 28 juillet, les anciennes lignes
qui mentionnent un maillage 75 mm, un plafond uniforme de cinq workers, une
erreur oracle de 13–14 % ou les anciens hash Release. Ces valeurs restent dans
le document comme historique des commits qui les ont produites.

### Admission 95 mm et borne physique

Une comparaison 75/95 mm a été exécutée avec un passage jeté, six passages
contrebalancés par variante et le CP2 comme témoin nul sans worker :

| Moteur | 75 mm, moyenne | 95 mm, moyenne | Effet |
|---|---:|---:|---:|
| CP2, témoin nul | 2,5467 | 2,5580 | +0,4 % |
| LS3 | 0,9910 | 1,0498 | **+5,9 %** |
| Merlin | 0,9920 | 1,0818 | **+9,1 %** |

Le maillage ciblé de production est donc **95 mm**. Son oracle reste 30 mm,
RK2 et couplage à chaque sous-pas :

| Contrôle | Production 95 mm | Limite |
|---|---:|---:|
| Écart VE maximal, standard + runner 2× | **4,460 %** | 15 % |
| Pic VE | 1,127 | 0,85–1,15 |
| Régime du pic | 5 840 tr/min | ≥ 2 880 |
| VE à 6 000 tr/min | 1,120 | ≥ 0,902 |
| Déplacement du pic, runner 2× | 26,478 % | ≥ 15 % |

La protection thermique à 600 µs reste à **4,916 %** contre le même oracle et
passe les quatre critères. Un essai à 120 mm a été refusé : 16,580 % d'écart
sur le runner doublé à 3 000 tr/min, donc hors de la tolérance demandée.

Logs : `intake-cell-95mm-realtime-ab-2026-07-28.log`,
`intake-95mm-production-oracle-2026-07-28.log`,
`intake-95mm-wall600-oracle-2026-07-28.log` et
`intake-cell-120mm-oracle-2026-07-28.log` sous `out/validation/`.

### Pool automatique adapté aux 12 threads

Le balayage 2/3/4 workers utilise six passages contrebalancés par variante ;
le Big Twin est le témoin structurellement à zéro worker.

| Cas | 2 workers | 3 workers | 4 workers |
|---|---:|---:|---:|
| LS3, moyenne | **1,1160** | 1,1045 | 1,0805 |
| Merlin, moyenne | 1,1095 | **1,1222** | 1,1032 |
| Big Twin, moyenne | 3,5168 | 3,5395 | 3,5238 |
| LS3, minimum | **1,109** | 1,093 | 1,076 |
| Merlin, minimum | 1,100 | **1,114** | 1,091 |

La production choisit deux workers pour 3 à 9 cylindres, trois à partir de dix,
puis borne ce nombre par la moitié des threads matériels et `cylindres - 1`.
Un override explicite de banc contourne cette politique.

### Catalogue complet de production

Commande :

```text
EngineLabRealtimeBudgetHarness --catalog-root . --relative-rpm 0.90 \
  --warmup 3 --seconds 6 --free-run --enforce 1.10
```

| Moteur | Cible | Régime moyen | Workers | Facteur |
|---|---:|---:|---:|---:|
| K20A-like 2.0 I4 VTEC | 7 740 | 7 734 | 2 | 1,475 |
| 2JZ-GTE-like 3.0 I6 Turbo | 6 300 | 6 298 | 2 | 1,739 |
| LS3-like 6.2 Crossplane V8 | 5 940 | 5 942 | 2 | 1,140 |
| EJ25-like 2.5 Flat-4 Turbo | 6 120 | 6 114 | 2 | 1,921 |
| Audi I5-like 2.5 Turbo | 6 390 | 6 387 | 2 | 1,836 |
| Hayabusa-like 1.3 I4 | 10 080 | 10 074 | 2 | 1,380 |
| Big Twin-like 1.9 V2 | 5 040 | 5 037 | 0 | 3,540 |
| Merlin-like 19.8 V12 Scaled | 2 880 | 2 881 | 3 | **1,082** |
| Aircooled-like 3.6 Flat-6 | 6 660 | 6 660 | 2 | 1,419 |
| Radial-like 6.5 R5 | 2 160 | 2 160 | 2 | 2,040 |
| Yamaha CP2 MT-07-like 689 Twin | 9 000 | 8 987 | 0 | 2,524 |
| Yamaha CP3 MT-09-like 890 Triple | 9 900 | 9 892 | 2 | 1,829 |
| Yamaha CP4 MT-10-like 998 Crossplane I4 | 10 800 | 10 800 | 2 | 1,291 |
| VW EA288-like 2.0 TDI I4 | 4 500 | 4 483 | 2 | 2,370 |

Cette table est l'exact binaire final après correction du film carburant. Les
14 régimes sont tenus, aucun overrun n'est produit et tout le catalogue reste
au-dessus du temps réel. Elle a toutefois été contaminée par une tâche Windows
externe : `CompatTelRunner` consommait environ un cœur, tandis que Chrome a
atteint plusieurs cœurs pendant le lot.

Le contrôle obligatoire confirme que le facteur absolu de ce passage ne peut
pas servir de référence intrinsèque. Dans le lot alterné suivant, sans aucune
modification de code :

| Mesure finale répétée | Six facteurs | Meilleur temps |
|---|---|---:|
| Merlin | 1,147 / 1,088 / 1,100 / 1,039 / 1,108 / 1,015 | **1,147** |
| CP2 témoin zéro-worker | 2,574 / 2,432 / 2,497 / 2,198 / 2,378 / 1,894 | **2,574** |

Le témoin s'effondre de 26 % au fil du lot, preuve directe d'interférence
machine. Suivant la règle du brief — l'interférence ne peut qu'ajouter du temps,
donc retenir le minimum de temps sur N essais — la capacité intrinsèque
observée du V12 final est **1,147×**, soit 14,7 % de marge. La table 1,082× reste
conservée parce qu'elle représente honnêtement l'application quand d'autres
processus occupent déjà la machine ; le garde-fou de charge existe précisément
pour ce cas.

Logs : `realtime-catalog-release-final-2026-07-29.log` et
`realtime-merlin-final-repeatability-filter-2026-07-29.log` sous
`out/validation/`.

### Son final et continuité de la télémétrie

Le rendu catalogue final active, sur les 14 moteurs, le réseau physique, la
topologie complète, la structure modale et l'admission ondulatoire. Il rapporte
zéro fallback, dropout de frontière, pression perdue, événement tardif ou
échantillon limité. La similarité spectrale maximale entre les quatre familles
de contrôle est **0,642**.

| Chemin applicatif | Workers | Callback p95 | Budget | Perte/fallback |
|---|---:|---:|---:|---:|
| K20A | 2 | 1 837,3 µs | 5 333,3 µs | 0 / 0 |
| 2JZ | 2 | 2 268,2 µs | 5 333,3 µs | 0 / 0 |
| LS3 | 2 | 2 887,5 µs | 5 333,3 µs | 0 / 0 |
| Merlin | 3 | 3 662,0 µs | 5 333,3 µs | 0 / 0 |

Ce dernier rendu a volontairement subi la charge externe détectée ci-dessus :
le garde-fou s'est activé sur K20, LS3 et Merlin, mais aucun échantillon ni
paquet de pression n'a été perdu. Le transitoire Big Twin passe
1 342 → 4 139 → 773 tr/min, toujours sans perte, événement tardif ni limiteur.
Log : `out/validation/audio-render-release-final-2026-07-29.log`.

### Reprise après coupure de décélération

La première suite complète après ces changements a trouvé un dernier défaut
réel : le Merlin franchissait le plancher de 360 tr/min au retour d'un coup de
gaz. Relever le seuil de reprise et accélérer la rampe DFCO n'a pas suffi ;
même une reprise expérimentale à 1,50 fois le ralenti finissait à 349 tr/min.

Le modèle soustrayait la part disponible du film existant, mais supposait que
toute la masse liquide d'un pulse neuf atteindrait la charge avant l'étincelle.
La masse injectée tient désormais compte de la fraction de mouillage `X` :

```text
fraction disponible = (1 - X) + X * fraction évaporée avant étincelle
masse commandée = déficit de la charge / fraction disponible
```

Le seuil 1,25× et la rampe 3/s d'origine ont été restaurés. Avec cette reprise
plus tardive, le trace compensé reste à 479 tr/min au même point et les portes
`Core`, `IdleStabilityRegression` (14 moteurs) et `CatalogReference` passent.
Logs : `ctest-film-availability-2026-07-28.log`,
`merlin-idle-trace-dfco150-2026-07-28.log` et
`merlin-idle-trace-film-compensated-2026-07-28.log`.

### Fermeture Release

La build complète MSVC Release et la suite complète ont été relancées après le
dernier correctif physique :

```text
cmake --build out/build/windows-vs2022 --config Release -- /m:1 /nr:false
ctest --test-dir out/build/windows-vs2022 -C Release --output-on-failure
```

Résultat : **20/20 tests, zéro échec, 514,43 s**. Le test de rendu audio final
séparé passe également sous charge externe. L'exécutable est resté vivant
pendant cinq secondes en lancement caché, puis a été arrêté proprement.

| Artefact | Octets | SHA-256 |
|---|---:|---|
| `build-release-final-2026-07-28.log` | 14 602 | `6B1DFEC17D7D0826DCE9E2143B6C80BB36A5C208DCC871296BE60B23DA628FA1` |
| `ctest-release-final-2026-07-28.log` | 5 144 | `1FD022E3C8E0C37CCF5B9671DECD06D82E40BE1AD15774B739AA92090406A9DD` |
| `EngineLab.exe` | 8 385 024 | `C3D60C1B6D5983A82CE4EFB992ADC76FDF133CB7DC9D520686A63A792E007295` |

Le journal du smoke test est
`out/validation/executable-smoke-final-2026-07-29.log`. La copie de livraison
`out/validation/EngineLab-final-2026-07-29.exe` est bit-identique au binaire de
build ; son second lancement caché de cinq secondes est consigné dans
`out/validation/executable-delivery-proof-2026-07-29.log`.
