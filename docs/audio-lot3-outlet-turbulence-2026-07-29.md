# Lot audio 3 — turbulence causale à la sortie d’échappement

Date : 29 juillet 2026  
Machine : i5-11600 fixe, 12 threads logiques  
État : implémenté et validé techniquement ; préférence humaine encore à établir

## Pourquoi cette couche existe

La frontière thermoacoustique de pression cylindre et le réseau d’échappement
restent les sources principales. Leur cadence de couplage limite cependant la
partie directement reconstruite du signal ; le pire cas du catalogue publie une
bande utile de 2 190 Hz. Il manquait donc une excitation large bande crédible
au débouché, surtout perceptible en charge, sans inventer une seconde combustion
ni réinjecter du bruit dans le solveur.

`ExhaustJetNoise` convertit le débit **déjà résolu** à chaque sortie en bruit de
mélange turbulent. La source est strictement unidirectionnelle :

```
solveur gaz -> débit/état du débouché -> source de jet -> observateur -> IR
```

Elle n’envoie aucune pression fabriquée vers le réseau de conduits.

## Modèle et provenance

Au début de chaque bloc, le débit moyen du chemin est réparti entre ses sorties
proportionnellement à leur aire. La vitesse moyenne vient donc de :

```
U = débit massique / (rho * aire)
```

La puissance suit la loi compacte de jet subsonique :

```
W = K * rho * aire * U^8 / c^5
```

La fréquence centrale suit `St = fD/U = 0,2`. Les deux ancrages sont cohérents
avec la littérature NASA sur la [loi en U^8 des jets subsoniques à basse
vitesse](https://ntrs.nasa.gov/citations/19730003289) et le [pic de Strouhal
voisin de 0,2](https://ntrs.nasa.gov/citations/19790050524).

Le signal de débit volumique audio au débouché module ensuite causalement la
puissance en `U^8`. Les pointes sont bornées à `4,5 ×` la vitesse moyenne et à
`0,95 c` : la première limite empêche d’interpréter une perturbation acoustique
linéaire comme un second solveur compressible, la seconde maintient le modèle
dans son domaine subsonique.

Le coefficient propre de jet est `K = 1e-4`. Un multiplicateur de puissance
**explicitement authored** de `100` (+20 dB) représente le fait qu’un
échappement moteur n’est pas une buse de laboratoire propre. La littérature
signale que, à faible vitesse, les unités réelles s’écartent de la loi propre et
que les fluctuations de débit dues à la turbulence intense ou aux ondes
produites en amont peuvent fixer le plancher de bruit ([NASA CR-3281,
p. 20–21](https://ntrs.nasa.gov/api/citations/19800006956/downloads/19800006956.pdf)).
Cette référence justifie la présence d’un terme moteur, **pas sa valeur de
100** : cette valeur reste une calibration d’écoute à remplacer lorsqu’un
corpus moteur comparable le permettra.

Le bruit est déterministe, indépendant par sortie, normalisé en RMS et filtré
autour de la fréquence de Strouhal. Le callback ne fait ni allocation, ni
verrou, ni appel de service aléatoire, ni fonction transcendante. Le rayonnement
emploie le même `FreeFieldObserver`, la même position, le même axe, la même
directivité, les deux mêmes microphones et la même IR que le débouché physique.

## Calibrations refusées

Les essais intermédiaires sont conservés sous
`out/validation/audio-lot3-2026-07-29`, mais ne sont pas des références :

- jet propre, débit moyen seul : pic K20 de `0,0248 Pa`, différence inaudible
  dans les métriques ;
- multiplicateur `100`, débit moyen seul : pic K20 de `0,2476 Pa`, encore trop
  faible devant le blowdown ;
- débit instantané non borné : K20 et LS3 plausibles, mais Big Twin à `158 Pa`,
  différence RMS de `136 %` et pré-limiteur à `1,06` — rejet immédiat ;
- limite instantanée `2,25 ×` : Big Twin à seulement `0,19 %` de différence —
  effet presque nul ;
- limite finale `4,5 ×` : effet mesurable sur les cinq familles, sans recours
  au leveler ni au limiteur.

## A/B sonore, même binaire

Commande :

```powershell
out/build/windows-vs2022/tools/Release/EngineLabAudioRenderHarness.exe `
  --exhaust-jet-comparison "K20" `
  --output out/validation/audio-lot3-2026-07-29/final-k20
```

Chaque comparaison rend deux instances déterministes du même binaire, source
OFF puis ON. Le signal OFF exige un pic jet exactement nul. Les WAV `jet-off`
et `jet-on` sont livrés dans chaque dossier.

| Moteur | Différence RMS relative | Pic de la différence | Pic jet observé | Cosinus spectral |
|---|---:|---:|---:|---:|
| K20 I4 | 3,57 % | 0,0572 | 8,4268 Pa | 0,999999 |
| LS3 V8 cross-plane | 6,81 % | 0,0392 | 5,9624 Pa | 0,997918 |
| Big Twin V2 | 2,40 % | 0,0217 | 3,3099 Pa | 0,999992 |
| 2JZ I6 turbo | 1,37 % | 0,0215 | 3,2777 Pa | 0,999997 |
| Merlin V12 | 4,53 % | 0,0488 | 7,5060 Pa | 0,999627 |

Ces chiffres prouvent que la couche existe, reste bornée et ne remplace pas la
signature du réseau. Ils ne prouvent pas qu’elle est préférée à l’aveugle. Les
fichiers devront être notés avec le protocole de
[`audio-ab-listening.md`](audio-ab-listening.md) avant tout nouveau gain.

## Transitoires

`EngineLabAudioRenderHarness --transient-only` passe les trois scénarios de
production :

| Cas | Mesure principale | Résultat |
|---|---|---:|
| Big Twin démarrage/ralenti/rev | ratio pas transition / fond | 3,168 |
| 2JZ boost/lever/reprise | débit dump maximal | 0,09834 kg/s |
| 2JZ boost/lever/reprise | ratio pas transition / fond | 1,40 |
| K20 rupteur | régime maximal | 8 575 tr/min |
| K20 rupteur | pré-limiteur maximal | 0,5685 |

Les trois cas ont zéro perte de pression/événement, zéro retard, zéro fallback,
zéro leveler et restent finis.

## Coût temps réel A/B

Le contrôle `--disable-exhaust-jet-noise` retire réellement la synthèse et ses
observateurs du callback. Quatre mesures de 10 s ont été alternées dans la même
fenêtre temporelle sur le Merlin, cas le plus serré :

| Passage | Jet | Facteur global | Callback moyen | Callback p99 |
|---|---|---:|---:|---:|
| 1 | OFF | 1,129× | 43,7 % | 66 % |
| 2 | ON | 1,131× | 44,1 % | 67 % |
| 3 | ON | 1,144× | 44,4 % | 67 % |
| 4 | OFF | 1,126× | 44,3 % | 67 % |
| **moyenne** | **OFF** | **1,1275×** | **44,0 %** | **66,5 %** |
| **moyenne** | **ON** | **1,1375×** | **44,25 %** | **67,0 %** |

Le facteur global se croise dans le sens opposé au coût attendu : le bruit du
solveur et de l’ordonnanceur est donc supérieur à l’effet de la couche, et aucun
pourcentage de coût global ne doit en être déduit. Le chronométrage direct du
callback donne `+0,25` point moyen, soit environ `+0,6 %` du temps audio, et
`+0,5` point au p99. Les quatre passages restent entre `1,126×` et `1,144×`,
sans overrun, miss, perte, retard, fallback, leveler ni échantillon non fini.

Preuves : `out/validation/audio-lot3-2026-07-29/realtime-ab-final-v2`.

## Tests de non-régression

`exhaustJetNoiseRegression` vérifie :

- calcul de vitesse depuis débit, densité et aire ;
- `St = 0,2` ;
- rapport de puissance exactement `256` lorsque la vitesse subsonique double ;
- déterminisme échantillon par échantillon ;
- conservation du niveau RMS du filtre ;
- silence exact à débit nul ;
- rayonnement réel avec débit moyen ;
- réseau source-free exactement silencieux lorsque la couche est coupée.

## Limite restante

Le lot est **techniquement validé**, pas perceptuellement validé. Aucune
préférence humaine en aveugle n’a encore accepté le multiplicateur `100` ou la
borne `4,5 ×`. Jusqu’à ce test, ils doivent rester nommés, isolables et faciles
à remplacer ; aucune retouche moteur par moteur n’est autorisée.
