# Variabilite cycle-a-cycle de combustion

EngineLab peut appliquer une dispersion cycle-a-cycle au chemin physique de
combustion. Elle agit sur la turbulence/vitesse de flamme d'un moteur a
allumage commande et sur le temps de melange d'un diesel. La pression cylindre,
le travail indique, le debit d'echappement et enfin le son voient donc tous le
meme evenement. Ce n'est pas un bruit ajoute dans le renderer.

Deux champs de `combustion_calibration` la pilotent :

```yaml
combustion_calibration:
  cycle_variation_cov: 0.04
  cycle_variation_correlation: 0.55
```

- `cycle_variation_cov` est l'ecart-type du multiplicateur de vitesse de
  combustion. La plage valide est 0 a 0,20. Zero, valeur par defaut, est un
  contournement exact qui n'avance meme pas le generateur pseudo-aleatoire.
- `cycle_variation_correlation` est la correlation AR(1) entre deux cycles
  consecutifs du meme cylindre, de 0 a 0,98.

Le tirage est deterministe, borne entre 0,55 et 1,45 et utilise un flux aleatoire
independant par cylindre. Il ne modifie pas la sequence des vrais rates
d'allumage. `CylinderState::combustionCycleMultiplier` publie la valeur active
pour les mesures et les exports.

La fonction est volontairement non calibree dans le catalogue. Une valeur
doit etre choisie depuis une serie de cycles mesures (IMEP ou pression cylindre),
pas pour fabriquer artificiellement un ralenti irregulier. Comme ordre de
grandeur de depart pour une ecoute A/B, 0,02 a 0,05 convient a un moteur chaud
stable ; les valeurs plus fortes doivent correspondre a un regime pauvre,
dilue ou instable que la simulation explique aussi physiquement.
