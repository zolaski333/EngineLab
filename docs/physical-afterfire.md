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
    reaction_time_constant_s: 0.008
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

La source acoustique est dérivée de la chaleur libérée :

```text
p_source ≈ (gamma - 1) × Qdot / (2 × A × c)
```

Elle est injectée au nœud de réaction du guide d’onde. Il n’existe pas
d’oscillateur « pop », de périodicité audio imposée ou de source globale placée
artificiellement à la sortie.

## Utilisation dans AUDIO HQ

Le bouton `APPLY` envoie désormais la calibration au thread de simulation par
mailbox. Il ne reconstruit plus `EngineRuntime` et conserve donc régime, phase,
inventaires gazeux et températures de paroi. L’application est refusée pendant
un pull dyno afin de ne pas modifier une mesure en cours.

`DEMO AUDIBLE` sélectionne `discrete_afterfire`, 4 Hz nominaux, 35 % de duty,
25 % de variation temporelle et 18 % de carburant moyen. Le simple toggle
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

Sur le Twin laboratoire à environ 4 000 tr/min, paroi chaude :

| Cas | Carburant brûlé | Événements | Puissance crête | Crête audio | Crest factor |
|---|---:|---:|---:|---:|---:|
| DFCO propre | 0 mg | 0 | 0 kW | 0,00313 | 2,74 |
| Afterfire discret irrégulier | 37,477 mg | 10 | 8,694 kW | 0,00502 | 4,29 |

Le cas discret gagne 4,11 dB en crête, mais seulement 1,73 dB au percentile
99,9 %. Les dix réactions sont espacées de 170,8 à 325,0 ms, avec un écart-type
de 54,0 ms : elles ne forment plus la vague strictement périodique à 250 ms.
La hausse du crest factor reste le comportement attendu d’impulsions brèves,
contrairement à l’ancien swell régulier qui augmentait surtout le niveau
continu.

Les WAV de contrôle sont produits dans
`out/implementation-2026-08-11/afterfire-null-final4/` et
`out/implementation-2026-08-11/afterfire-discrete-irregular-final/`.
