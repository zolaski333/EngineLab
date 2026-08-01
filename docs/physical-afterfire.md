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
ignition:
  # Le rupteur coupe l'etincelle mais conserve l'injection.
  limiter_keeps_fuel: true
```

- `ignition_temperature_k` est le seuil thermique, valide de 500 a 2 000 K ;
- `reaction_time_constant_s` fixe la vitesse d'oxydation au-dessus du seuil ;
- `reaction_efficiency` borne la fraction de reactifs qui peut reagir pendant
  une etape.

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
coupure de carburant en deceleration reste prioritaire. Il n'y a donc toujours
pas de pop programme ; le carburant doit traverser le cylindre, rencontrer de
l'oxygene et une ligne assez chaude.

Dans **AUDIO HQ**, le bloc **PHYSIQUE AUDIO** expose ces reglages. **DEMO
AUDIBLE** applique une calibration d'ecoute (COV 6 %, correlation 0,55,
afterfire actif, rupteur humide, seuil 800 K, reaction 8 ms), puis redemarre le
moteur. **BYPASS** remet variation, afterfire et rupteur humide a zero/off. La
ligne `LIVE` affiche les kW et mg/s effectivement produits : zero signifie que
les conditions chimiques ne sont pas reunies, pas que l'interface est en panne.
