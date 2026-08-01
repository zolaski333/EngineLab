# Afterfire physique dans l'echappement

L'afterfire optionnel d'EngineLab est une reaction chimique dans le reseau
quasi-1D. Il ne programme aucun « pop » et ne declenche aucun sample. Une maille
ne reagit que si elle contient simultanement du carburant imbrule, de l'oxygene
et une temperature superieure au seuil d'allumage. La masse des especes est
conservee et le pouvoir calorifique du carburant augmente l'energie totale de
la maille. La hausse de pression qui en resulte traverse ensuite le meme reseau
et la meme sortie acoustique que le blowdown normal.

La fonction est desactivee par defaut :

```yaml
exhaust_afterfire:
  enabled: true
  ignition_temperature_k: 900
  reaction_time_constant_s: 0.010
  reaction_efficiency: 0.95
  # Optionnel : petite charge de carburant sur vraie décélération DFCO.
  overrun_fuel_fraction: 0.12
  overrun_minimum_rpm: 3000
  overrun_maximum_throttle: 0.02
ignition:
  # Le rupteur coupe l'etincelle mais conserve l'injection.
  limiter_keeps_fuel: true
```

- `ignition_temperature_k` est le seuil thermique, valide de 500 a 2 000 K ;
- `reaction_time_constant_s` fixe la vitesse d'oxydation au-dessus du seuil ;
- `reaction_efficiency` borne la fraction de reactifs qui peut reagir pendant
  une etape.
- `overrun_fuel_fraction` conserve une fraction bornée de la charge normale
  pendant une vraie décélération DFCO et coupe l'étincelle ; zéro conserve
  exactement la coupure propre historique ;
- `overrun_minimum_rpm` et `overrun_maximum_throttle` bornent la zone ECU.

La stoechiometrie, la masse molaire et le pouvoir calorifique viennent de
`fuel_properties`. `EngineState::exhaustAfterfireHeatReleaseKw` et
`exhaustAfterfireFuelBurnMgPerSecond` rendent le phenomene mesurable.

Activer le modele ne garantit volontairement aucun bruit : une coupure
d'injection propre ne fournit pas de carburant, un melange riche sans oxygene ne
peut pas bruler, et une ligne froide reste silencieuse. Pour obtenir un
afterfire, la calibration moteur doit produire physiquement les trois conditions
necessaires. Ce comportement empeche de confondre un effet sonore avec une
combustion d'echappement plausible.

Le rupteur historique coupe carburant et etincelle et reste le comportement par
defaut. `limiter_keeps_fuel: true` fournit un chemin physique volontairement
humide : au hard cut, l'injection continue mais l'etincelle est supprimee. La
coupure de carburant en deceleration reste prioritaire tant que
`overrun_fuel_fraction` vaut zéro. Avec une fraction positive, l'ECU n'arme le
mode qu'après une demande conducteur supérieure à 20 % au-dessus du seuil RPM.
Au lever de pied, il conserve exactement la fraction auteur sans la multiplier
par l'enrichissement transitoire, coupe l'étincelle puis laisse le carburant
traverser le cylindre. Il n'y a donc toujours pas de pop programmé : il faut
encore de l'oxygène et une ligne assez chaude.

Dans **AUDIO HQ**, le bloc **PHYSIQUE AUDIO** expose ces reglages. **DEMO
AUDIBLE** applique une calibration d'ecoute (COV 6 %, correlation 0,55,
afterfire actif, rupteur humide, carburant de décélération 12 %, seuil 800 K,
reaction 8 ms), puis redemarre le moteur. **BYPASS** remet variation, afterfire,
carburant de décélération et rupteur humide a zero/off. La
ligne `LIVE` affiche les kW et mg/s effectivement produits : zero signifie que
les conditions chimiques ne sont pas reunies, pas que l'interface est en panne.

Pour l'essai le plus direct, sélectionner `Audio Physics Lab 689 Twin`, cliquer
**DEMO AUDIBLE**, dépasser 3 000 tr/min avec plus de 20 % d'accélérateur puis
relâcher. `decel ACTIVE` prouve la stratégie ECU ; seuls des kW/mg/s non nuls
prouvent ensuite que la chimie d'échappement a réellement réagi.
