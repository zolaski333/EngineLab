# Induction d'afterfire thermochimique — implémentation du 22 août 2026

## Résultat

L'induction locale de l'afterfire n'est plus obligatoirement un chronomètre
plat. Le schéma moteur 8 peut maintenant authorer une corrélation
température/pression/richesse ; chaque volume de contrôle intègre ensuite un
état de Livengood–Wu sans nouvelle maille, voix audio ni allocation temps réel.

Le chemin précédent reste disponible de manière exacte pour les documents de
schéma 1 à 7 : les exposants migrent à zéro et la durée de décroissance devient
deux fois le délai de référence. La correction ne change donc pas en silence un
ancien moteur.

## Défaut mesuré avant correction

Un sweep contrôlé a été ajouté au test du réseau gaz. Même température minimale
dépassée, même quantité de carburant et même géométrie ; un seul état est changé
par ligne :

| Cas | T | pression | phi | délai mesuré avant |
|---|---:|---:|---:|---:|
| référence | 901 K | 101,325 kPa | 1,00 | 4,00 ms |
| chaud | 1 150 K | 101,325 kPa | 1,00 | 4,00 ms |
| pressurisé | 901 K | 180 kPa | 1,00 | 4,00 ms |
| pauvre | 901 K | 101,325 kPa | 0,50 | 4,00 ms |

Le fichier brut est
`out/audit-2026-08-20/afterfire-induction-flat-baseline.log`.

Ce résultat n'était pas un défaut d'arrondi : le code ajoutait directement des
secondes à `inductionSeconds`. Une fois `ignitionTemperatureK` franchie, aucune
grandeur thermodynamique ne pouvait encore influer sur l'instant d'allumage.

## Modèle livré

Le délai local est la forme de Modified Arrhenius normalisée sur le point
authoré :

```text
tau(T,p,phi) = tau_ref
             * exp[theta_a * (1/T - 1/T_ref)]
             * (p/p_ref)^(-n)
             * phi^m
```

avec :

- `tau_ref = induction_time_s` ;
- `T_ref = ignition_temperature_k` ;
- `p_ref = induction_reference_pressure_kpa` ;
- `theta_a = induction_activation_temperature_k = Ea/R` ;
- `n = induction_pressure_exponent` ;
- `m = induction_equivalence_ratio_exponent`.

Chaque site persistant avance ensuite selon :

```text
I[n+1] = min(1, I[n] + dt / tau(T,p,phi))
```

Le noyau est établi à `I = 1`. Si la source chaude repasse sous le seuil avant
l'allumage mais que le mélange demeure flammable, l'intégrale revient vers zéro
sur `induction_decay_time_s` : cette mémoire chimique n'est plus cachée derrière
un facteur codé en dur. Un inventaire devenu non flammable remet immédiatement
l'intégrale à zéro, comme avant le schéma 8.

La masse, l'oxygène et l'énergie ne changent qu'après l'établissement du noyau.
La même réaction conservative, le même temps de mélange et la même source
thermoacoustique restent autoritaires.

## Provenance des exposants du moteur laboratoire

Le Twin `Audio Physics Lab 689 Twin` emploie comme estimation :

```yaml
induction_time_s: 0.004
induction_reference_pressure_kpa: 101.325
induction_activation_temperature_k: 13340
induction_pressure_exponent: 0.989
induction_equivalence_ratio_exponent: -0.577
induction_decay_time_s: 0.008
```

La forme et les trois exposants viennent de la corrélation globale publiée par
Shaqiri et al. pour des données d'allumage de l'essence et de substituts :
`tau ~ phi^-0.577 p^-0.989 exp(26.51 kcal mol^-1 / RT)`. La conversion
`26,51 kcal/mol / R` donne environ 13 340 K.

Sources primaires :

- A. Shaqiri et al., *High-pressure ignition delay time measurements of a
  four-component gasoline surrogate and its high-level blends with ethanol and
  methyl acetate*, Fuel 290 (2021),
  <https://doi.org/10.1016/j.fuel.2020.118016> ;
- manuscrit auteur archivé par l'OSTI :
  <https://www.osti.gov/biblio/1799409>.

Limite importante : les points du papier sont des essais de tube à choc, autour
de 4 à 60 atm et 968 à 1 361 K, pas une ignition de paroi dans une ligne à
environ 1 atm. EngineLab réutilise donc la **forme relative** et les exposants,
mais conserve son délai de référence de 4 ms comme estimation d'ingénierie. Ce
n'est ni une calibration OEM, ni une validation d'une cinétique détaillée à
800 K.

## Oracle après correction

Le même sweep, avec ces paramètres explicites, donne :

| Cas | délai mesuré après | relation exigée par le test |
|---|---:|---|
| référence | 3,95 ms | 3,8–4,1 ms |
| chaud | 0,20 ms | < 10 % de la référence |
| pressurisé | 2,25 ms | < 70 % de la référence |
| pauvre | 5,90 ms | > 135 % de la référence |

Le pas d'observation est 0,05 ms, ce qui explique l'arrondi. Le test vérifie
aussi que chaque cas reste flammable, fini et effectivement allumé. La masse et
l'énergie du test conservatif historique restent inchangées : 44,2482 mg de
carburant, 155,257 mg d'oxygène et 1 946,92 J sur son cas chaud.

Le fichier brut est
`out/audit-2026-08-20/afterfire-induction-arrhenius-final.log`.

## A/B dans le geste produit

Le harness possède maintenant `--flat-induction`, contrôle même binaire qui met
seulement les trois exposants à zéro. Les deux passages utilisent le moteur
catalogué, 60 s de chauffe, un vrai lever à 4 291 tr/min et 8 s d'overrun.

| Mesure | timer plat | intégrale thermochimique |
|---|---:|---:|
| délai local observé | 4,000–4,000 ms | 0,001–6,543 ms |
| carburant livré / brûlé | 177,850 / 63,532 mg | 178,070 / 72,092 mg |
| excursions chaleur | 27 | 25 |
| puissance chaleur crête | 9,244 kW | 10,547 kW |
| énergie source publiée | 4 579,559 J | 5 175,180 J |
| pression port crête | 138,8 kPa | 142,5 kPa |
| saut compact maximal | 54,708 kPa | 54,018 kPa |
| crête audio | 0,00518 | 0,00457 |
| P99,9 audio | 0,00291 | 0,00292 |
| crest audio | 6,41 | 5,67 |

La nouvelle loi ne cherche donc pas à fabriquer un pop plus fort. Elle change
le lieu et l'instant où les poches atteignent leur induction, brûle davantage de
l'inventaire livré et répartit un peu plus l'énergie ; le maximum audio baisse
de 1,1 dB tandis que le P99,9 reste pratiquement identique. C'est accepté comme
conséquence physique mesurée, pas compensé par un gain.

Avec la source de réaction active contre le même trajet où seule sa copie audio
est coupée, la composante afterfire vaut 5,103 LSB RMS contre 25,011 LSB pour le
contrôle, soit -13,81 dB. La crête passe de 0,00348 à 0,00457 et le crest de
4,45 à 5,67 : l'afterfire reste identifiable sans saturation.

Fichiers :

- `out/audit-2026-08-20/afterfire-induction-ab-authored.log` ;
- `out/audit-2026-08-20/afterfire-induction-ab-flat.log` ;
- `out/audit-2026-08-20/afterfire-arrhenius-off.log` ;
- WAV correspondants dans les dossiers de même nom.

## Observabilité corrigée

Le résultat chimique publie maintenant :

- la plus grande intégrale locale ;
- le plus petit et le plus grand délai local fini du passage.

`EngineState` transporte ces deux grandeurs et le panneau `PHYSICS DEBUG`
affiche `I / tau`. L'agrégation se fait sur toute la frame mécanique. Avant
cette correction, `wallIgnitedFraction` était remplacé par zéro à chaque
sous-pas sans flush ; le même scénario affichait ainsi 0,1 % en moyenne alors
que l'agrégation correcte donne 2,2 %. Aucun état physique n'a été changé par
cette réparation de télémétrie.

## Coût CPU

Le calcul Arrhenius est exécuté uniquement si :

1. la chimie d'overrun ou de rupteur humide est active ;
2. le site contient un mélange flammable ;
3. le noyau n'est pas déjà établi ;
4. au moins un exposant est non nul.

Les schémas historiques et les moteurs sans afterfire prennent une branche qui
retourne directement `induction_time_s`, sans logarithme ni exponentielle. Il
n'y a aucune nouvelle boucle, maille, file ou voix.

Le harness chronomètre désormais séparément le pas simulation de 4,167 ms et le
rendu audio de 200 échantillons. Cette séparation est importante : le compteur
audio historique ne pouvait pas mesurer le coût de la corrélation, qui s'exécute
sur le thread physique.

Sur le Twin en réaction active, les 1 920 pas simulation consomment 20,3 % du
budget moyen, 23,4 % au p99 et 25,0 % au maximum. Le contrôle plat exécuté juste
après mesure 20,2 %, 23,5 % et 25,9 %. Le coût de la corrélation est donc
indiscernable du bruit de ce passage, jusque dans les queues mesurées. Aucun pas
ne dépasse son budget.

Sur le thread audio séparé, la loi thermochimique mesure 13,7 % en moyenne,
15,0 % au p99 et 15,9 % au maximum, contre 13,6 %, 14,7 % et 17,3 % pour le
timer plat. Aucun bloc ne dépasse 4,167 ms et tous les compteurs de pertes,
limite de pression et leveler restent à zéro. Il ne faut pas additionner ces
deux pourcentages : ils décrivent deux échéances et deux threads distincts.

Le harness invalide désormais aussi le passage si une stratégie qui retient du
carburant n'arme pas, ne livre ou ne brûle rien, n'atteint pas `I=1`, ne produit
aucune excursion de chaleur ou, quand l'audio est mesuré, ne publie aucune
source de réaction. Les deux branches de l'A/B satisfont ce contrat.

## Validation Release intégrale

La reconstruction complète de `out/build/windows-vs2022` en Release passe,
application comprise. La suite autoritaire termine à **42/42 CTest**, zéro
échec, en **719,78 s**. Elle couvre notamment `GasDynamics`, `Core`,
`OverrunThermalRegression`, les rendus/transitoires audio, le catalogue complet,
l'autorité turbo aval et les rampes dyno produit CP2/LS3.

Journaux bruts :

- `out/audit-2026-08-20/afterfire-induction-core.log` ;
- `out/audit-2026-08-20/afterfire-induction-full-ctest.log` ;
- les quatre journaux et trois dossiers WAV listés dans les sections
  précédentes.

## Limites conservées

1. Une température de source égale au maximum gaz/paroi est une fermeture
   d'ordre réduit ; elle ne résout pas la couche limite thermique au contact du
   métal.
2. L'intégrale est locale au volume de contrôle. Un noyau/radical distinct n'est
   toujours pas advecté entre cellules.
3. La réaction après allumage reste une relaxation à une étape, pas une chimie
   multi-espèces.
4. Le domaine basse pression de l'échappement n'est pas couvert par la source
   expérimentale des exposants. Toute calibration moteur réelle doit remplacer
   les six paramètres avec une provenance adaptée.
5. Le minimum de 0,001 ms observé arrive dans du gaz déjà très chaud ; le temps
   de réaction de 2 ms borne encore la libération de chaleur et la source reste
   sous la limite acoustique linéaire. Il ne faut pas interpréter ce minimum
   comme une mesure physique résolue à la microseconde.

La prochaine extension ne doit transporter un état de noyau entre cellules que
si un oracle de slug montre une erreur de localisation ou une dépendance au
maillage. L'ajouter par principe augmenterait l'état et le coût sans preuve.
