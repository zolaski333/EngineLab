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
schema_version: 6
engine:
  exhaust_afterfire:
    strategy: discrete_afterfire
    enabled: true
    ignition_temperature_k: 800
    reaction_time_constant_s: 0.002
    reaction_efficiency: 0.95
    overrun_fuel_fraction: 0.18
    overrun_minimum_rpm: 3000
    overrun_maximum_throttle: 0.02
    overrun_pulse_hz: 4.0
    overrun_pulse_duty: 0.35
    overrun_pulse_timing_variation: 0.25
    induction_time_s: 0.004
    minimum_equivalence_ratio: 0.45
    maximum_equivalence_ratio: 1.80
    quench_temperature_k: 520
```

La fraction de carburant est une valeur moyenne. En mode pulsé, l’ECU la
normalise par le rapport cyclique : passer de 100 % à 35 % de duty ne supprime
donc plus 65 % de la masse demandée. À 12 % moyen et 35 % de duty, le paquet du
laboratoire restait néanmoins trop pauvre (`phi ≈ 0,34`) pour la borne physique
`phi_min = 0,45`. La démonstration d’écoute emploie 18 % (`phi ≈ 0,51`).

La cadence de `discrete_afterfire` n’est plus un carré parfaitement périodique.
`overrun_pulse_timing_variation` décale les frontières des paquets avec une
séquence déterministe à faible répétition. Une valeur de 0,25 autour de 4 Hz
borne chaque intervalle entre 187,5 et 312,5 ms au niveau ECU ; le front de
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

Le délai d’induction est la durée écrite **au seuil**. L’ancien calcul le
divisait silencieusement par `(T - T_allumage) / 450 K` : un délai écrit à 4 ms
devenait ainsi 1,8 s seulement 1 K au-dessus du seuil. Ce facteur caché a été
supprimé. L’origine paroi/gaz de la flamme est mémorisée à l’allumage ; elle
n’est plus réinterprétée après que la réaction elle-même a chauffé le gaz.

La chimie n’est exécutée que dans un état qui peut effectivement la demander :
DFCO avec stratégie retenant du carburant, ou rupteur humide actif. Le simple
fait qu’un moteur possède une calibration afterfire ne déclenche plus
l’oxydation de traces d’hydrocarbures à pleine charge.

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

## Utilisation dans AUDIO HQ

Le bouton `APPLY` envoie désormais la calibration au thread de simulation par
mailbox. Il ne reconstruit plus `EngineRuntime` et conserve donc régime, phase,
inventaires gazeux et températures de paroi. L’application est refusée pendant
un pull dyno afin de ne pas modifier une mesure en cours.

`DEMO AUDIBLE` sélectionne `discrete_afterfire`, une réaction compacte de 2 ms,
4 Hz nominaux, 35 % de duty, 25 % de variation temporelle et 18 % de carburant
moyen. Le simple toggle
complète aussi une calibration restée à zéro, au lieu d’afficher « actif » sans
aucune matière réactive.

Pour le test direct :

1. sélectionner `Audio Physics Lab 689 Twin` ;
2. activer `DEMO AUDIBLE` ;
3. tenir le moteur au-dessus de 3 000 tr/min avec plus de 20 % de gaz ;
4. relâcher complètement.

La télémétrie indique la stratégie, les bloqueurs ECU, les kW, les mg/s, le
nombre de volumes réactifs et les événements perdus. Une puissance nulle reste
un résultat physique possible : ligne froide, mélange hors fenêtre, DFCO propre
ou stratégie non armée.

## Preuve mesurée

Le contrôle final emploie le Twin laboratoire tel qu’il est catalogué, 60 s de
chauffe, un intervalle chargé juste avant le lever et 8 s d’overrun. Les deux
rendus ci-dessous ont une trajectoire moteur, une chimie et une énergie
strictement identiques ; le contrôle retire seulement la copie des événements de
réaction vers le réseau audio.

| Cas | Réactions avant lever | Carburant brûlé | Événements chaleur | Puissance crête | Crête audio | P99,9 | Crest |
|---|---:|---:|---:|---:|---:|---:|---:|
| événements réaction non injectés | 0 | 63,532 mg | 27 | 9,244 kW | 0,00348 | 0,00262 | 4,48 |
| événements réaction injectés | 0 | 63,532 mg | 27 | 9,244 kW | 0,00518 | 0,00291 | 6,41 |

L’injection physique ajoute 3,46 dB à la crête. Sa composante directe mesurée
par soustraction des WAV vaut −12,44 dB par rapport au mix complet sur tout
l’overrun ; l’ancien chemin était à environ −57,75 dB. Les 817 événements
couplés transportent 4 579,559 J ; le saut compact maximal calculé est 54,708
kPa, sans atteindre la borne de 100 kPa.

Le rendu actif consomme 13,6 % du budget moyen d’un bloc de 200 échantillons et
14,6 % au p99, contre 13,1 % et 13,9 % dans le contrôle. Aucun des 1 920 blocs
n’a dépassé sa durée. Les WAV frais sont sous
`out/audit-2026-08-20/afterfire-final-timed-on/` et
`out/audit-2026-08-20/afterfire-final-timed-off/`.

Ces nombres valident le chemin logiciel et la non-vacuité du modèle. La
calibration de 2 ms reste une estimation d’ingénierie du moteur laboratoire,
pas une identification issue d’un enregistrement ou d’un banc instrumenté.
