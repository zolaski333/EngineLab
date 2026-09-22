# Bilan d'énergie de l'arbre turbo — 20 août 2026

## Verdict

Le modèle turbo possédait un puits d'énergie caché. Après avoir intégré

```text
E[n+1] = E[n] + (P_turbine - P_compresseur - P_paliers) dt
```

il tronquait la vitesse à `1,16 * N_design`, puis tronquait séparément la
vitesse employée par le compresseur à `1,12 * N_design`. Toute puissance
excédentaire disparaissait sans travail, chaleur, dommage ni diagnostic.

Sur le 2JZ catalogue, le scénario de stress mesurait exactement 150 800 tr/min
pour les quatre sorties alors que la turbine fournissait 32,71 à 41,60 kW et
que le compresseur n'en absorbait que 17,45 à 17,49 kW. En ajoutant environ
0,38 kW de pertes de palier, **14,9 à 23,7 kW étaient effacés en continu**.

La correction conserve maintenant chaque joule dans l'une des trois voies
explicites : travail du compresseur, pertes de palier, ou énergie cinétique de
l'arbre. La wastegate reste l'actionneur qui réduit la puissance turbine. Le
chemin n'ajoute ni cellule, ni table, ni état audio-rate.

## Causes exactes

### Deux plafonds incompatibles

L'ancien chemin appliquait :

```text
omega_next = min(sqrt(2 E_next / J), 1.16 omega_design)
speed_ratio = clamp(omega_next / omega_design, 0, 1.12)
PR = 1 + (PR_design - 1) speed_ratio^2
```

Le premier `min` détruisait l'énergie après son intégration. Le second clamp
rendait même les 4 % supérieurs de la vitesse retenue inaudibles et sans effet
thermodynamique. Sur le 2JZ, cela produisait le nombre invariant :

```text
1 + (1.92 - 1) * 1.12^2 = 2.154048
```

### L'accélérateur commandait directement la tête du compresseur

La pression cible était aussi multipliée par `throttle^0.38`. Une fermeture du
papillon faisait donc disparaître directement la pression située **en amont**
du papillon. Cela confondait deux organes : le papillon doit modifier le débit
et la pression du collecteur ; la roue compresseur répond à sa vitesse et au
travail qu'elle échange avec le débit. Une pression amont peut ainsi subsister
pendant un lever de pied et ouvrir physiquement la soupape de décharge.

### Le contrôle de stress était mal nommé

`TurboDownstreamAuthorityHarness` appelait son second scénario « open loop » et
affirmait que la wastegate restait fermée. Il élevait en réalité sa consigne à
3,42 ; une fois cette consigne atteinte, le contrôleur pouvait naturellement
rouvrir la wastegate. Le scénario est désormais nommé
`raised-target energy stress` et annonce explicitement qu'il ne constitue pas
une calibration de pression.

## Modèle réduit livré

Les fonctions unitaires résident dans
`foundation/ForcedInductionFlow.hpp`. Elles séparent la puissance réellement
prélevée pendant le pas de la tête disponible à la fin du pas.

### Puissance du compresseur

À partir du débit massique mesuré, de la température d'entrée, du rapport de
pression actuellement produit et du rendement authoré :

```text
P_c = m_dot * cp * T_in * (PR^((gamma-1)/gamma) - 1) / eta_c
```

avec `cp = 1005 J/(kg K)` et l'exposant air `2/7` déjà employé par le modèle.
Le débit de la soupape de décharge est inclus : l'air recirculé ou rejeté n'est
pas comprimé gratuitement.

### Pertes de palier

La loi authorée existante reste explicite :

```text
P_b = P_b,design * (omega / omega_design)^2
```

Elle n'est pas transformée en limiteur et sa valeur est publiée séparément.

### Intégration conservative

Pour des puissances tenues pendant le sous-pas :

```text
P_net = P_t - P_c - P_b
E_next = max(0, 0.5 J omega^2 + P_net dt)
omega_next = sqrt(2 E_next / J)
```

Si le travail compresseur/palier demandé pendant un pas dépasse la puissance
turbine plus l'énergie cinétique disponible, les deux pertes tenues sont
réduites dans la même proportion. Un arbre arrêté ne peut ainsi pas fournir un
travail fictif. Le plancher numérique à zéro ne corrige plus que l'arrondi et
n'efface jamais une énergie positive. L'oracle unitaire couvre accélération,
équilibre et arrêt puis reconstruit `E_next` à la précision flottante.

### Tête centrifuge

La loi d'affinité déjà sous-jacente au projet est rendue continue :

```text
PR_next = 1 + (PR_design - 1) (omega_next / omega_design)^2
```

Le point `omega = omega_design` traverse donc exactement le rapport de pression
authoré. Aucun accélérateur et aucun second clamp ne modifient cette identité.

Cette équation est un modèle réduit de similitude, pas une carte compresseur.
Le rapport `N/N_design` reste visible précisément pour ne pas cacher une
extrapolation.

## Observabilité produit

`EngineState` publie maintenant :

- `turboBearingPowerKw` ;
- `turboShaftNetPowerKw` ;
- `forcedInductionShaftSpeedRatio`.

Le panneau `PHYSICS DEBUG` montre vitesse et rapport de conception, les trois
puissances turbine/compresseur/paliers, puis la puissance nette. Il marque
`>DESIGN` dès que la vitesse dépasse le point authoré. Ces lectures sont des
copies d'état à 30 Hz et n'ajoutent rien au hot path audio.

Le harness imprime les mêmes grandeurs et invalide désormais :

- un moteur stock hors de 0,85–1,15 fois sa vitesse de conception ;
- un stress qui ne traverse pas réellement l'ancien clamp ;
- une réponse vitesse/pression qui redevient plate avec le downstream ;
- un bilan établi dont `|P_net|` dépasse 0,25 kW.

## A/B mesuré

Même binaire Release, même 2JZ, 4 197 tr/min, quatre diamètres de sortie.

### Avant

| sortie | boost stress | arbre | compresseur | turbine | puissance effacée estimée |
|---:|---:|---:|---:|---:|---:|
| 45 mm | 2,154 | 150 800 rpm | 17,45 kW | 32,71 kW | 14,9 kW |
| 60 mm | 2,154 | 150 800 rpm | 17,46 kW | 39,52 kW | 21,7 kW |
| 76 mm | 2,154 | 150 800 rpm | 17,46 kW | 41,46 kW | 23,6 kW |
| 95 mm | 2,154 | 150 800 rpm | 17,49 kW | 41,60 kW | 23,7 kW |

### Après — configuration stock

| sortie | boost | N/Ndesign | compresseur | turbine | palier | net |
|---:|---:|---:|---:|---:|---:|---:|
| 45 mm | 1,921 | 1,000 | 13,26 kW | 13,56 kW | 0,28 kW | 0,02 kW |
| 60 mm | 1,930 | 1,005 | 13,63 kW | 13,93 kW | 0,28 kW | 0,02 kW |
| 76 mm | 1,932 | 1,006 | 13,69 kW | 14,05 kW | 0,28 kW | 0,07 kW |
| 95 mm | 1,931 | 1,006 | 13,71 kW | 14,01 kW | 0,28 kW | 0,02 kW |

Le régulateur stock continue donc de tenir la pression, tandis que le couple
monte de 380,36 à 400,50 Nm lorsque la restriction downstream baisse.

### Après — stress énergétique hors point de conception

| sortie | boost | arbre | N/Ndesign | compresseur | turbine | palier | net |
|---:|---:|---:|---:|---:|---:|---:|---:|
| 45 mm | 3,410 | 210 399 rpm | 1,618 | 37,41 kW | 38,18 kW | 0,73 kW | 0,03 kW |
| 60 mm | 3,427 | 211 130 rpm | 1,624 | 38,57 kW | 39,32 kW | 0,74 kW | 0,01 kW |
| 76 mm | 3,432 | 211 347 rpm | 1,626 | 38,92 kW | 39,75 kW | 0,74 kW | 0,09 kW |
| 95 mm | 3,432 | 211 372 rpm | 1,626 | 39,06 kW | 39,88 kW | 0,74 kW | 0,08 kW |

La variation de boost de 0,022 est modeste mais non nulle et monotone. Le point
important est le bilan : les dizaines de kilowatts supplémentaires deviennent
du travail compresseur et des pertes explicites, plus un nombre effacé.

## Audio et budget CPU

Le transitoire 2JZ après correction mesure :

- boost avant lever / maximum : 1,907 / 1,931 ;
- débit maximum de soupape de décharge : 0,11002 kg/s ;
- pas de transition commandée isolée (`ratio = 0,06`) ;
- aucun drop, retard, vol de voix, fallback, leveler ou limiteur ;
- résultat `PASS`.

Le budget produit 2JZ à 6 300 tr/min, Release, 48 kHz / 256 samples, 20 s :

- capacité : **1,426x temps réel** ;
- DSP moyen 30,6 %, p99 45 % ;
- pic pré-limiteur 0,1183 ;
- tous les compteurs du contrat temps réel à zéro.

La correction retire un `pow(throttle, 0.38)` du sous-pas et réorganise des
opérations scalaires déjà présentes. Elle n'ajoute aucune boucle dépendante du
nombre de cellules ou de cylindres.

## Validation

- `EngineLab.Core` : point de conception, équilibre exact, conservation au-delà
  de l'ancien clamp ;
- `EngineLab.GasExchange` : passé ;
- `EngineLab.TurboDownstreamAuthority` : tous les nouveaux gates passés ;
- `EngineLab.CatalogReference` : passé sans retune du 2JZ ni du diesel ;
- `EngineLab.AudioRender` : 16 moteurs passés ;
- `EngineLab.AudioTransients` : passé ;
- `EngineLab.AudioShiftTransientBoosted` : passé ;
- application Release reliée ;
- suite Release autoritaire : **42/42 CTest**, zéro échec, en **765,38 s**,
  rampes dyno produit CP2 et LS3 incluses.

Les mesures brutes sont conservées dans :

- `out/audit-2026-08-20/turbo-compressor-baseline-telemetry.log` ;
- `out/audit-2026-08-20/turbo-compressor-energy-final.log` ;
- `out/audit-2026-08-20/turbo-compressor-realtime-2jz.log` ;
- `out/audit-2026-08-20/turbo-compressor-audio-transients/`.

## Limites honnêtes

1. Le projet ne possède toujours pas de carte compresseur débit corrigé / ligne
   de vitesse / rendement. La loi de similitude est défendable autour du point
   authoré, mais pas une preuve de précision à `N/Ndesign = 1,62`.
2. Le stress à environ 3,4 de rapport de pression sert uniquement à traverser
   l'ancien clamp et fermer le bilan d'énergie. Il ne représente pas une
   pression recommandée ni validée pour le matériel 2JZ.
3. Surge, choke, température dépendante du rendement et rupture d'arbre ne sont
   pas modélisés. Les inventer sans carte ou paramètres de famille remplacerait
   un clamp caché par des nombres tout aussi cachés.
4. La pression de sortie turbine reste une fermeture quasi-stationnaire du
   downstream ; elle n'est pas encore un volume dynamique séparé.

La prochaine extension compresseur ne doit être engagée que lorsqu'un schéma
peut porter au minimum des lignes de vitesse et de rendement avec provenance.
Jusque-là, l'application montre explicitement quand elle extrapole au-delà du
point de conception.
