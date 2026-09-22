# Lot audio 4B — conservation de puissance large bande FI

Date : 29 juillet 2026  
État : défaut physique corrigé ; effet catalogue volontairement subtil

## Défaut

`ForcedInductionAcoustics` convertit une puissance de compresseur, turbine,
wastegate ou dump valve en pression, puis filtre un bruit blanc entre deux
pôles. Le code compensait uniquement le RMS `1/sqrt(3)` du bruit uniforme, mais
pas la perte du filtre différentiel.

Pour deux pôles `a` et `b` alimentés par le même bruit blanc de variance unité :

```
var(a) = a / (2 - a)
cov(a,b) = a*b / (1 - (1-a)*(1-b))
var(bande) = var(a) + var(b) - 2*cov(a,b)
```

À une fréquence centrale de 300 Hz et 48 kHz, le filtre historique ne conservait
qu’environ `0,151` RMS. En tenant également compte du fait que
`pressurePeakFromPower()` retourne le pic d’un sinus alors qu’un bruit se
normalise en RMS, la composante large bande sortait environ `4,70 ×` trop faible
par rapport à **sa propre puissance configurée**.

## Correction

Le filtre calcule désormais sa variance analytique, normalise la bande à un RMS
unitaire, puis emploie la pression RMS :

```
p_rms = sqrt(W * rho * c / (4*pi*r^2))
```

Les coefficients `tonal_acoustic_efficiency` et
`turbulent_jet_noise_coefficient`, les débits, les aires, les fréquences de
Strouhal et les niveaux des tons n’ont pas changé.

Le contrôle nul `setBroadbandPowerNormalisationEnabled(false)` reproduit
l’ancien niveau pour les A/B du même binaire.

## A/B Release

Commande :

```powershell
out/build/windows-vs2022/tools/Release/EngineLabAudioRenderHarness.exe `
  --forced-induction-power-comparison 2JZ `
  --output out/validation/audio-lot4-fi-power-2026-07-29/final/2jz
```

| Moteur | Différence RMS master | Pic différence | Pic FI avant | Pic FI après | Variation pic FI |
|---|---:|---:|---:|---:|---:|
| 2JZ | 0,206 % | 0,000590 | 0,4632 Pa | 0,5215 Pa | +12,6 % |
| EJ25 | 0,095 % | 0,000436 | 0,6764 Pa | 0,7183 Pa | +6,2 % |
| Audi I5 | 0,187 % | 0,000456 | 0,8923 Pa | 0,9567 Pa | +7,2 % |
| VW TDI | 0,084 % | 0,000198 | 0,2596 Pa | 0,2728 Pa | +5,1 % |

Le cosinus spectral du master reste `1,000000` à six décimales sur les quatre
cas. Cela ne signifie pas que la correction est nulle : le test analytique
mesure la puissance de la source isolée, et le pic FI progresse bien. Cela
signifie que le bruit large bande FI reste minoritaire devant les tons, le
réseau moteur, l’admission et la structure.

Tous les A/B ont zéro perte de pression/événement, zéro frontière invalide,
fallback, leveler ou sortie non finie. Les pré-limiteurs restent sous `0,64`.

Preuves :
`out/validation/audio-lot4-fi-power-2026-07-29/final`.

## Coût temps réel

Quatre passages alternés de 10 s sur l’EJ25 :

| Passage | Normalisation | Facteur | Callback moyen | Callback p99 |
|---|---|---:|---:|---:|
| 1 | OFF | 2,242× | 26,5 % | 41 % |
| 2 | ON | 2,249× | 26,6 % | 41 % |
| 3 | ON | 2,264× | 26,4 % | 41 % |
| 4 | OFF | 2,230× | 26,4 % | 41 % |
| **moyenne** | **OFF** | **2,236×** | **26,45 %** | **41 %** |
| **moyenne** | **ON** | **2,257×** | **26,50 %** | **41 %** |

Le facteur global se croise encore dans le sens opposé au coût attendu. La
différence directe moyenne du callback est de `+0,05` point et le p99 est
inchangé ; le coût n’est donc pas résolu au-delà du bruit de mesure. Les quatre
passages ont zéro overrun, miss, perte, retard, fallback ou leveler.

Preuves :
`out/validation/audio-lot4-fi-power-2026-07-29/realtime-ab`.

## Tests

La régression rend pendant deux secondes une source compresseur seule, ignore
la première seconde de chauffe, calcule indépendamment `W` puis `p_rms`, et
exige que le RMS mesuré reste à moins de 12 % de cette valeur. Elle exige aussi
que le contrôle nul expose l’ancienne perte d’au moins un facteur deux.

La suite transitoire complète passe avec :

- dump valve 2JZ à `0,09834 kg/s` ;
- zéro perte, retard, fallback ou leveler ;
- pré-limiteur K20 à `0,5685` ;
- résultat global `PASS`.

## Décision et limite

La correction est gardée parce qu’elle restaure une grandeur déjà promise par
le modèle, sans inventer de gain et sans coût temps réel mesurable. Elle ne
constitue pas une calibration perceptuelle du turbo. Le coefficient large bande
reste semi-empirique et ne sera pas augmenté moteur par moteur sans écoute
aveugle ou référence comparable.
