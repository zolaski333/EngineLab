# Livraison EngineLab — validation finale du 29 juillet 2026

Ce document fixe les preuves de la livraison exécutée sur la machine actuelle.
Les anciens chiffres absolus de performance ne servent pas de référence : les
mesures ci-dessous proviennent toutes du même build MSVC Release et de la même
session locale.

## Verdict

- Le catalogue complet tient le temps réel avec la commande prescrite :
  **14 moteurs valides sur 14**, aucun dépassement de budget.
- Le pire cas est le Merlin V12 à **1,167× temps réel**, soit environ
  **14,3 % de marge sur l'échéance** (`1 - 1 / 1,167`).
- Les **25 tests CTest sur 25** passent.
- Les **24 points constructeur sur 24** sont stabilisés et restent dans
  l'enveloppe de ±15 %.
- Le ZIP final contient l'application, les 14 moteurs, les 6 réponses
  impulsionnelles, les 6 catalogues de pièces, les exemples et la
  documentation.

## Catalogue complet — protocole prescrit

Commande :

```powershell
out/build/windows-vs2022/tools/Release/EngineLabRealtimeBudgetHarness.exe `
  --free-run --rpm 7000 --seconds 6
```

Le harnais borne une consigne absolue à 95 % du rupteur propre au moteur. Cela
évite de demander exactement le régime où l'ECU coupe volontairement le couple,
ce qui rendrait le point invalide sans mesurer le coût du solveur. La colonne
`cible` montre la consigne réellement testée.

| Moteur | Cyl. | Cible (tr/min) | Moyenne (tr/min) | Workers | Facteur temps réel |
|---|---:|---:|---:|---:|---:|
| Honda K20 | 4 | 7 000 | 7 002 | 2 | 1,568× |
| Toyota 2JZ-GTE | 6 | 6 650 | 6 651 | 2 | 1,596× |
| GM LS3 | 8 | 6 270 | 6 271 | 2 | 1,201× |
| Subaru EJ257 | 4 | 6 460 | 6 461 | 2 | 2,005× |
| Audi EA855 I5 | 5 | 6 745 | 6 746 | 2 | 1,782× |
| Suzuki Hayabusa 1999 | 4 | 7 000 | 7 003 | 2 | 1,885× |
| Harley-Davidson Big Twin | 2 | 5 320 | 5 320 | 0 | 3,409× |
| Rolls-Royce Merlin V12 | 12 | 3 040 | 3 043 | 3 | **1,167×** |
| Porsche 964 flat-six | 6 | 7 000 | 7 001 | 2 | 1,390× |
| Radial cinq cylindres | 5 | 2 280 | 2 282 | 2 | 2,569× |
| Yamaha CP2 | 2 | 7 000 | 7 004 | 0 | 3,071× |
| Yamaha CP3 | 3 | 7 000 | 7 001 | 2 | 2,341× |
| Yamaha CP4 | 4 | 7 000 | 7 002 | 2 | 1,835× |
| VW EA288 diesel | 4 | 4 750 | 4 750 | 2 | 2,500× |

Le résultat important n'est pas de comparer ces valeurs à une ancienne
machine : c'est que chaque simulation termine sa charge avant son échéance au
cours du même passage. Le plafond effectif observé est de trois workers pour le
V12 dans la politique actuelle ; il ne provoque plus de rupture temps réel.

Un témoin complémentaire normalisé à 80 % du régime maximal
(`--relative-rpm 0.8 --seconds 6`) tient lui aussi les 14 points, avec le Merlin
encore pire cas à 1,269×.

## Validation fonctionnelle et physique

Le build Release complet puis :

```powershell
ctest --test-dir out/build/windows-vs2022 -C Release --output-on-failure -j 1
```

ont produit **25/25 PASS** en 549,19 s. La durée du lot n'est pas une référence
de performance ; elle prouve seulement que les gardes suivantes passent
ensemble :

- physique du catalogue, régressions, échange gazeux, combustion et ralenti ;
- échappement, topologie, propagation et rendu audio ;
- transitoires audio et passages de rapport atmosphérique/turbo ;
- dynamique véhicule et transfert de charge ;
- configuration NVH structurelle ;
- formats, scripts, ECU, banc et instantanés de rendu.

### Banc et références

Le banc emploie maintenant un absorbeur partagé exprimé en couple et ne publie
un point que lorsque sa fenêtre est stable : moyenne ±2 %, excursion ≤4 %,
dérive ≤1 % et écart-type ≤1,5 %. Les **24 fenêtres et 24 valeurs sur 24**
passent. La table, les sources et les critères sont dans
[`dyno-hold-validation-2026-07-29.md`](dyno-hold-validation-2026-07-29.md).

### Audio

- Les sources turbo annulées ou dupliquées ont été retirées ; les cinq témoins
  atmosphériques restent bit-identiques.
- Les transitoires de production et les passages de rapport atmosphérique et
  turbo possèdent des tests dédiés.
- Les erreurs de réponse impulsionnelle sont visibles et testées au lieu de
  produire silencieusement un rendu trompeur.
- Les commandes exposées par l'interface correspondent aux couches physiques ;
  les anciens réglages sans effet sont explicitement désactivés.
- Un corpus CC0 de cinq familles de moteurs et dix fichiers A/B de même durée
  et même sonie est reproductible. Voir
  [`audio-ab-listening.md`](audio-ab-listening.md).

### Véhicule et structure

- Le transfert longitudinal `m*a*h/L` distingue traction, propulsion et
  transmission intégrale ; la persistance JSON/YAML et les 14 appariements du
  catalogue sont testés. Voir
  [`vehicle-load-transfer-validation-2026-07-29.md`](vehicle-load-transfer-validation-2026-07-29.md).
- Le schéma accepte des modes structurels mesurés ou calculés, sourcés, avec
  fréquence, amortissement, masse modale, rayonnement, excitation et
  participation par cylindre. Les fausses provenances sont refusées. Voir
  [`structural-nvh-configuration.md`](structural-nvh-configuration.md).

## Artefact de livraison

L'application non empaquetée et l'application extraite du ZIP ont chacune été
lancées dans une fenêtre cachée, ont atteint l'état d'entrée Windows et sont
restées vivantes après cinq secondes. Le contrôle du contenu exige :

- un `EngineLab.exe` ;
- 14 définitions moteur ;
- 6 réponses impulsionnelles WAV ;
- 6 catalogues de pièces ;
- la documentation et les exemples.

Le SHA-256 du binaire Release est :

```text
7452702E34812965DE8DC0D79B5EE5CA60715BEDD380291F92AEEC5BB692C4A4
```

Le hash du ZIP est relevé après sa reconstruction finale, car inclure son
propre hash dans le paquet créerait une dépendance circulaire.

## Limites restantes, explicitement ouvertes

1. Le corpus A/B est prêt et vérifié techniquement, mais aucune préférence
   humaine aveugle n'est encore enregistrée. La qualité perceptuelle finale ne
   peut donc pas être déclarée gagnante.
2. Le catalogue livré n'invente aucun mode NVH constructeur. Il reste sur une
   estimation de famille tant qu'un jeu de mesures structurelles sourcé n'est
   pas disponible.
3. Des avertissements de contre-pression turbo apparaissent encore sur certains
   régimes du 2JZ et de l'Audi I5, et selon le point sur l'EJ257. Ils doivent
   guider une investigation/calibration, pas être masqués.
4. EngineLab ne revendique pas encore une supériorité globale sur ES2D sans
   comparaison perceptuelle exécutable et contrôlée.

