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
