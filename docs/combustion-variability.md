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

Les moteurs de production restent volontairement non calibres dans le
catalogue. La variante explicitement nommee `Audio Physics Lab 689 Twin` est
une exception pedagogique : son COV de 0,06 est un reglage d'ecoute estime et
annonce comme tel, pas une valeur constructeur. Pour un moteur calibre, une
valeur doit etre choisie depuis une serie de cycles mesures (IMEP ou pression
cylindre), pas pour fabriquer artificiellement un ralenti irregulier. Comme ordre de
grandeur de depart pour une ecoute A/B, 0,02 a 0,05 convient a un moteur chaud
stable ; les valeurs plus fortes doivent correspondre a un regime pauvre,
dilue ou instable que la simulation explique aussi physiquement.

**AUDIO HQ** publie en direct le minimum et le maximum des multiplicateurs vus
sur les cylindres. Le bouton **BYPASS** remet le COV a zero, ce qui conserve le
contournement bit-exact et permet une comparaison sans tirage aleatoire cache.

## Ce que le catalogue delivre reellement (mesure du 2 aout 2026)

`cycle_variation_cov` vaut 0 sur quinze des seize moteurs, et il serait naturel
d'en conclure que leurs cycles se repetent. **C'est faux, et cela a ete mesure.**

`EngineLabCyclicVariabilityHarness` mesure le COV du travail indique par cycle
et par cylindre -- soit COV(PMI) -- chaque cylindre autour de sa propre moyenne.
Sur le catalogue livre, sans aucune dispersion autoree :

- charge partielle, regime tenu : **1,7 a 7,0 %** ;
- pleine charge, regime tenu : **0,9 a 12,9 %**.

La dispersion vient du couplage dynamique des gaz, du film de paroi, des ondes
de conduit et de l'ECU. Le simulateur est deterministe -- deux executions sont
identiques -- mais il n'est pas periodique. Une proposition d'ajouter de la
variabilite doit donc partir de cette table, pas du champ a zero.

Trois consequences pratiques :

1. Une fermeture physique pilotee par la dilution a ete prototypee puis
   **retiree** : la fraction de gaz brules a l'allumage vaut 0,004 a 0,026 sur
   tout le catalogue, donc son terme etait identiquement nul. La faire agir
   aurait exige d'abaisser son seuil sur la sortie du simulateur.
2. L'ordre est **inverse** sur sept moteurs, plus disperses a pleine charge qu'a
   charge partielle. Ce n'est pas l'absorbeur : la colonne `dN%` montre le
   regime tenu a 0,10-0,48 % pendant que le travail varie de 5 a 13 %.
3. Un **ralenti libre n'est pas un instrument valide** pour cette question : son
   COV(PMI) est domine par la chasse du regulateur (90 % mesures sur le radial).
   Tenir le regime, ou utiliser `EngineLab.IdleStabilityRegression`, qui repond a
   une autre question.

Le harnais n'est volontairement **pas** enregistre comme test : ses bandes de
reference viennent de la litterature et neuf moteurs en sortent aujourd'hui. En
faire une porte maintenant obligerait a elargir les bandes jusqu'au comportement
courant, ce qui detruirait leur valeur. Voir `docs/physics-audit.md`.

```
out/build/windows-vs2022/tools/Release/EngineLabCyclicVariabilityHarness.exe
```

`--cov X` force la dispersion autoree pour un A/B dans une seule session et un
seul binaire ; `--filter NOM` restreint le catalogue ; `--enforce` transforme les
bandes en porte, une fois qu'elles seront tenables.
