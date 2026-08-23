# Afterfire physique dans l’échappement

L’afterfire d’EngineLab n’est ni un sample ni un bruit ajouté. L’ECU transporte
des paquets de carburant imbrûlé et d’oxygène dans le réseau quasi-1D ; une
réaction locale libère ensuite de l’énergie dans la maille ou la jonction où les
conditions sont réunies. La hausse de pression traverse le même DAG acoustique,
les mêmes collecteurs, silencieux et sorties que le blowdown normal.

## Stratégies ECU

Le schéma moteur 6 distingue explicitement trois intentions :

- `clean_dfco` : coupure propre, aucun carburant retenu ;
- `continuous_anti_lag` : carburant et allumage échappement continus, destiné à
  un grondement/anti-lag et non à des pops isolés ;
- `discrete_afterfire` : paquets de carburant hachés, destinés à produire des
  combustions séparées après un lever de pied.

Exemple de calibration discrète :

```yaml
schema_version: 8
engine:
  exhaust_afterfire:
    strategy: discrete_afterfire
    enabled: true
    ignition_temperature_k: 800
    reaction_time_constant_s: 0.002
    reaction_efficiency: 0.95
    overrun_fuel_fraction: 0.08
    overrun_minimum_rpm: 3000
    overrun_maximum_throttle: 0.02
    overrun_pulse_hz: 2.0
    overrun_pulse_duty: 0.08
    overrun_pulse_timing_variation: 0.40
    induction_time_s: 0.004
    induction_reference_pressure_kpa: 101.325
    induction_activation_temperature_k: 13340
    induction_pressure_exponent: 0.989
    induction_equivalence_ratio_exponent: -0.577
    induction_decay_time_s: 0.008
    minimum_equivalence_ratio: 0.45
    maximum_equivalence_ratio: 1.80
    quench_temperature_k: 520
```

La fraction de carburant est une valeur moyenne. En mode pulsé, l’ECU la
normalise par le rapport cyclique. Le profil final garde fraction et duty à 8 % :
la commande instantanée atteint donc la pleine injection normale pendant une
fenêtre de 40 ms, deux fois par seconde, pour seulement 8 % de masse moyenne.
L’ancien profil 18 % / 35 % / 4 Hz ouvrait pendant 87,5 ms. Il traversait de
nombreuses opportunités d’injection et a mesuré 76,6 % de duty chimique sur le
Twin, presque 100 % sur le K20 : c’était un grondement proche de l’anti-lag, pas
une suite de pops. Une fenêtre de 25 ms à 5 % pouvait au contraire manquer les
injections d’un bicylindre et laisser jusqu’à quatre secondes entre réactions.

La cadence de `discrete_afterfire` n’est plus un carré parfaitement périodique.
`overrun_pulse_timing_variation` décale les frontières des paquets avec une
séquence déterministe à faible répétition. Une valeur de 0,40 autour de 2 Hz
borne chaque intervalle entre 300 et 700 ms au niveau ECU ; le front de
réaction mesuré peut s’en écarter légèrement à cause du transport et de
l’induction. Chaque paquet garde le même duty relatif à son intervalle, donc la
masse moyenne prescrite est conservée. Une valeur nulle restitue exactement la
cadence régulière historique.

## Conditions physiques

Un site ne brûle que si toutes les conditions suivantes sont vraies :

1. carburant et oxygène coexistent localement ;
2. l’équivalence locale est dans la fenêtre de flammabilité ;
3. le gaz ou la paroi dépasse la température d’allumage ;
4. ce mélange reste actif pendant le délai d’induction ;
5. une flamme établie n’est pas sous la température de quench.

Chaque maille et chaque jonction possède son propre état persistant d’induction
et de combustion. La réaction consomme les espèces de manière conservative et
ajoute `masse_carburant × PCI` à l’énergie. Elle publie un événement borné qui
contient le nœud exact, la position axiale, l’énergie, la durée, la densité, la
célérité et la section locale.

`induction_time_s` est le délai de référence au seuil, à la pression de
référence et à `phi=1`. Le schéma 8 intègre `dt/tau(T,p,phi)` avec une loi
Arrhenius dont tous les coefficients sont exposés ; les schémas 1 à 7 migrent
avec des exposants nuls et retrouvent exactement leur durée plate. L’ancien
calcul divisait silencieusement le temps par
`(T - T_allumage) / 450 K` : un délai écrit à 4 ms devenait ainsi 1,8 s seulement
1 K au-dessus du seuil. Ce facteur caché reste supprimé. L’origine paroi/gaz de
la flamme est mémorisée à l’allumage ; elle n’est plus réinterprétée après que la
réaction elle-même a chauffé le gaz. Équation, provenance, A/B et limites :
[afterfire-induction-implementation-2026-08-22.md](afterfire-induction-implementation-2026-08-22.md).

La chimie conserve son état dès qu’une stratégie de réaction est authorée, y
compris sous charge. Cela oxyde les traces d’hydrocarbures au fil de leur
transport au lieu d’accumuler artificiellement soixante secondes de carburant
puis d’enflammer cet ancien inventaire au lever. Cette oxydation d’entretien ne
publie toutefois ni télémétrie afterfire ni source acoustique. Ces deux sorties
restent strictement réservées au DFCO qui retient du carburant ou au rupteur
humide actif ; le contrôle chargé juste avant le lever mesure toujours zéro
événement acoustique.

La source acoustique compacte est dérivée de la chaleur libérée. Le saut total
de pression vaut :

```text
Delta p = (gamma - 1) × Qdot / (A × c)
```

Le réseau le partage ensuite entre ses deux caractéristiques voyageuses ; chaque
onde sortante reçoit donc `(gamma - 1) × Qdot / (2 × A × c)`. Le signal à la
cadence du solveur est reconstruit par le même filtre anti-imaging LR8 que les
frontières physiques. Un bloqueur continu à deux pôles et 25 Hz retire seulement
la chaleur quasi stationnaire déjà portée par l’écoulement moyen. L’ancien
passe-haut vers 3–6 kHz supprimait au contraire presque toute une réaction de
quelques millisecondes.

La source est injectée au nœud et à la position axiale de la réaction. Il
n’existe ni oscillateur « pop », ni sample, ni périodicité audio imposée, ni
source globale placée artificiellement à la sortie. Une borne de dernier recours
à 100 kPa protège le réseau linéaire ; chaque échantillon qui l’atteindrait est
compté, affiché et invalide les harness de validation.

Chaque voix conserve maintenant la puissance `énergie / durée` pendant la durée
exacte du pas de réaction. L’ancien rendu la maintenait deux fois plus longtemps
avec un minimum de 0,5 ms : il dupliquait l’énergie des événements courts et
collait les noyaux voisins en une vague continue.

## Utilisation dans AUDIO HQ

Le bouton `APPLY` envoie désormais la calibration au thread de simulation par
mailbox. Il ne reconstruit plus `EngineRuntime` et conserve donc régime, phase,
inventaires gazeux et températures de paroi. L’application est refusée pendant
un pull dyno afin de ne pas modifier une mesure en cours.

`DEMO AUDIBLE` sélectionne `discrete_afterfire`, une réaction compacte de 2 ms,
2 Hz nominaux, 8 % de duty, 40 % de variation temporelle et 8 % de carburant
moyen. Le simple toggle complète aussi une calibration restée à zéro et installe
la corrélation d’induction thermochimique du schéma 8. Une calibration anti-lag
ou afterfire déjà authorée reste inchangée.

Pour le test direct :

1. sélectionner `Audio Physics Lab 689 Twin` ;
2. activer `DEMO AUDIBLE` ;
3. tenir le moteur au-dessus de 3 000 tr/min avec plus de 20 % de gaz ;
4. relâcher complètement.

La télémétrie indique la stratégie, les bloqueurs ECU, les kW, les mg/s, le
nombre de volumes réactifs et les événements perdus. Une puissance nulle reste
un résultat physique possible : ligne froide, mélange hors fenêtre, DFCO propre
ou stratégie non armée.

## Preuve mesurée du chemin acoustique (profil produit du 23 août)

Le contrôle final emploie le Twin laboratoire tel qu’il est catalogué, 60 s de
chauffe, un intervalle chargé juste avant le lever et 8 s d’overrun. Les deux
rendus ci-dessous ont une trajectoire moteur, une chimie et une énergie
strictement identiques ; le contrôle retire seulement la copie des événements de
réaction vers le réseau audio.

| Cas | Réactions avant lever | Carburant brûlé | Événements chaleur | Puissance crête | Crête audio | P99,9 | Crest |
|---|---:|---:|---:|---:|---:|---:|---:|
| événements réaction non injectés | 0 | 89,373 mg | 10 | 15,068 kW | 0,04764 | 0,03310 | 4,75 |
| événements réaction injectés | 0 | 89,373 mg | 10 | 15,068 kW | 0,05835 | 0,03349 | 5,73 |

La soustraction des WAV donne une contribution de réaction à 0,03415 de crête.
Son RMS médian sur des fenêtres de 5 ms est exactement nul, puis atteint 0,01229
sur les pops : ce n’est plus une hausse continue du volume. Les dix fronts
audio isolés sont espacés de 460 à 1 090 ms. Leur énergie se répartit à 15,35 %
entre 20–120 Hz, 64,60 % entre 120–500 Hz, 19,65 % entre 500 Hz–2 kHz et 0,39 %
entre 2–8 kHz. Les 1 479 événements de volumes finis transportent 5 419,634 J ;
le saut compact maximal est 80,902 kPa, sans atteindre la borne de 100 kPa.

Le rendu actif consomme 13,5 % du budget moyen d’un bloc de 200 échantillons et
15,9 % au p99, contre 13,1 % et 14,2 % dans le contrôle. La simulation consomme
20,6 % du pas de 4,167 ms en moyenne et 24,4 % au p99. Aucun des 1 920 blocs ou
pas n’a dépassé sa durée. Les WAV frais sont sous
`out/audit-2026-08-23-afterfire-twin-fuel08-duty08/` et
`out/audit-2026-08-23-afterfire-twin-fuel08-duty08-no-source/`.

Ces nombres valident le chemin logiciel et la non-vacuité du modèle. La
calibration de 2 ms reste une estimation d’ingénierie du moteur laboratoire,
pas une identification issue d’un enregistrement ou d’un banc instrumenté.

La preuve schéma-8 avec induction thermochimique se trouve dans
[afterfire-induction-implementation-2026-08-22.md](afterfire-induction-implementation-2026-08-22.md).
Le diagnostic complet du niveau et du nouveau profil se trouve dans
[audio-level-afterfire-correction-2026-08-23.md](audio-level-afterfire-correction-2026-08-23.md).
