# Silencieux passif à noyau perforé — implémentation du 20 août 2026

Ce document fixe le contrat, les mesures et les limites du lot « silencieux
passif ». Il ne prétend pas qu'un silencieux du catalogue reproduit déjà un
modèle industriel précis : aucun jeu de mesures fabricant n'est fourni. Le but
de ce lot est plus fondamental : faire en sorte que les dimensions écrites
représentent le bon matériel, qu'elles modifient réellement le transfert
acoustique et que le coût reste borné.

Implémentation : commit `f865a7b` sur `codex/engine-corrections-roadmap`.

## Défaut racine

Un silencieux droit garni possède au moins deux volumes différents :

1. le noyau perforé, qui porte le débit moyen et le temps de propagation ;
2. le volume annulaire entre ce noyau et le corps extérieur, rempli de matériau
   et fermé vis-à-vis du débit axial principal.

Le graphe connaissait déjà `diameter_mm`, `length_mm`, `volume_l` et les trois
propriétés du garnissage. Cependant, `ExhaustNetworkLayout` interprétait tout
`volume_l` supérieur au volume balayé comme une chambre de débit. Sur le LS3,
le noyau écrit à 76 mm et le volume brut d'un corps proche de 142 mm devenaient
donc un conduit gaz proche de 142 mm. Les champs existaient, mais leur
sémantique physique était fausse.

Conséquences :

- la section, la vitesse, le frottement, le CFL et la pression moyenne ne
  correspondaient pas au noyau réel ;
- le volume annulaire ne possédait aucun état acoustique ;
- faire varier le corps modifiait implicitement le conduit au lieu de modifier
  un volume latéral ;
- ouvrir puis appliquer l'éditeur legacy pouvait changer la représentation du
  silencieux ;
- le résumé de volume du chemin comptait le volume scellé comme volume de gaz.

## Contrat de géométrie retenu

Pour un composant `muffler` dont les trois champs de garnissage sont positifs :

- `diameter_mm` et `outlet_diameter_mm` décrivent le noyau perforé ;
- `length_mm` est la longueur physique commune du noyau et du corps ;
- `volume_l` est le volume brut du corps extérieur ;
- `restriction` reste un coefficient de perte de charge du débit moyen ;
- la résistivité, l'épaisseur et la fraction ouverte restent des données de
  matériau/perforation, jamais des gains en dB.

Pour un noyau éventuellement conique, le volume balayé exact est :

```text
Vcore = L / 3 * (Ain + sqrt(Ain * Aout) + Aout)
Vann  = Vgross - Vcore
```

La validation exige `Vgross > Vcore`. Le solveur gaz ne voit que `Vcore`. Le
graphe de débit et ses résumés excluent `Vann`. Sans garnissage, un `muffler`
garde sa sémantique de chambre d'expansion sèche et `volume_l` peut donc encore
définir sa section interne.

La conversion du format scalaire conserve cette distinction : diamètre de
collecteur pour le noyau d'un silencieux garni, diamètre de chambre pour un
silencieux sec, et volume brut calculé depuis le corps scalaire. Un test compile
les chemins legacy et éditable puis exige diamètre, longueur, volume et perte
locale identiques.

## Modèle acoustique passif et borné

Le modèle comporte deux effets séparés.

### Perte dans le matériau

`PorousLinerLoss` conserve le modèle Delany-Bazley à couche rigide. La
résistivité et l'épaisseur déterminent l'impédance de surface ; la fraction
ouverte pondère la surface réellement exposée. Le filtre ajusté reste causal et
passif. Aucune absorption n'est déduite de `restriction`, du nom de la pièce ou
d'une consigne de niveau.

### Stockage dans le volume annulaire

Le mode de compression dominant du volume autour du noyau est représenté par
une compliance compacte :

```text
Vcouplé = Vann * fraction_ouverte
C       = Vcouplé / (rho * c²)
Yc      = 2 * C / T
```

`Yc` est l'admittance trapézoïdale/WDF déjà utilisée par les jonctions
compactes. La compliance est partagée entre les deux extrémités du noyau ; leur
distance reste portée par la ligne de délai du noyau. Si les deux extrémités
aboutissent au même nœud compact, le volume n'est ajouté qu'une fois.

Cette construction ajoute :

- zéro cellule gaz ;
- zéro sous-pas ;
- zéro nouvelle ligne de délai ;
- au plus le volume d'une compliance déjà présente à chacun des deux nœuds ;
- aucun branchement dépendant d'un nombre de perforations dans le callback.

Elle est donc une approximation distribuée d'ordre réduit, pas un réseau de
milliers de trous.

## Oracles déterministes

`EngineLab.ExhaustTransfer` excite des graphes frais avec la même source, le
même milieu et le même observateur. Chaque réponse doit être finie, non vide,
bornée par le plafond passif et décroître en fin de fenêtre. Deux rendus du
bypass doivent rester bit-identiques.

Résultats Release du fixture complet :

| Variation à un facteur | Écart de forme |
|---|---:|
| chambre contre tube droit | 5,823 dB |
| taper fini contre tube droit | 1,123 dB |
| résonateur 250/500 Hz | 11,015 dB |
| cavité terminale absente/présente | 11,825 dB |
| catalyseur bypass/monolithe | 7,062 dB |
| corps perforé 4 L/8 L, noyau identique | 5,138 dB |
| split asymétrique contre tube | 6,568 dB |
| volume compact de jonction 0,25/2,0 L | 6,434 dB |

Le test 4/8 L exige aussi le même nombre de conduits acoustiques : le volume du
corps change le transfert sans augmenter le travail structurel par échantillon.

## Mesure produit LS3

Le harness de sensibilité a rendu chaque variante à 4 000 tr/min pendant 3 s.
La « forme » est l'écart par tiers d'octave après retrait de l'offset large
bande ; le niveau physique provient de l'observateur avant le mix final.

| Variante | Pression observateur | RMS échappement | Forme échappement seul |
|---|---:|---:|---:|
| référence | 30,1 Pa | 0,002857 | référence |
| sans silencieux | 89,1 Pa | 0,003878 | 16,17 dB |
| avec garnissage A/B | 19,7 Pa | 0,002448 | 1,89 dB |
| corps/chambre ×2,25 | 11,4 Pa | 0,001917 | 6,83 dB |
| corps/chambre ×5,3 | 5,4 Pa | 0,000975 | 13,49 dB |

Retirer uniquement le corps donne `+9,41 dB` au niveau physique. Le mix final
ne monte que de `+0,07 dB`, car l'admission et la mécanique masquent
l'échappement dans 16 bandes sur 28. Le silencieux influence donc bien la
physique ; son audibilité dans le mix est un problème de balance de couches
distinct, qui ne doit pas être corrigé par un faux gain dans le silencieux.

Le pire changement de forme parmi toutes les variantes d'échappement mesurées
atteint 16,27 dB sur la couche échappement. Aucun échantillon n'a atteint le
limiteur et le garde-niveau est resté à 1,000.

## Budget CPU et optimisation sans perte de physique

Corriger le noyau 76 mm a légitimement raccourci le pas CFL par rapport au faux
conduit de 142 mm : le LS3 est passé d'environ 20 160 à 26 880 sous-pas
d'échappement par seconde. La première mesure sans audio est alors tombée vers
0,92× de temps réel. Le coût ne venait pas de la compliance audio mais du
solveur gaz désormais exécuté à la cadence correcte.

Un profil CPU utilisateur avec symboles a attribué, dans
`ExhaustGasNetwork::advance` :

| Zone | Part inclusive |
|---|---:|
| `evaluateStage` | 43,62 % |
| `prepareStageStates` | 31,05 % |
| `prepareStateCache` | 24,11 % |
| `recoverPrimitive` | 23,60 % |
| prédicat `finite` | 18,74 % |
| flux de Riemann | 16,50 % |
| `computeResidual` | 13,48 % |

Le prédicat a été remplacé par le test exact de l'exposant IEEE-754 binary64 :
un exposant à tous les bits à un signifie NaN ou infini. Un `static_assert`
interdit la compilation si `double` n'est pas ce format. NaN, +infini et
-infini restent rejetés dans chaque variable conservative ; zéro signé et les
subnormaux restent finis. Aucun seuil, plancher, maillage ou critère CFL n'a
changé.

A/B immédiat, chaque version reconstruite et exécutée deux fois après chauffe :

| Version | Facteurs free-run | Sous-pas échappement |
|---|---:|---:|
| `std::isfinite` | 0,932 / 0,935 | ~26 880 Hz |
| masque binary64 exact | 1,102 / 1,108 | ~26 880 Hz |

Le gain utile est d'environ 18 %. Couple, puissance et VE restent identiques à
la précision publiée par le harness. Deux flux transmissifs de bout de conduit,
immédiatement remplacés par les frontières du graphe, sont aussi évités ; cette
micro-optimisation est mathématiquement couverte par le gate qui exige que
chaque extrémité soit réellement assignée, mais aucun gain macro ne lui est
attribué séparément.

Mesures finales avec audio produit, 48 kHz / blocs de 256, six secondes et gate
`--enforce 0.97` :

| Moteur/régime | Facteur free-run | DSP moyen | DSP p99 | Contrat audio |
|---|---:|---:|---:|---:|
| LS3, 6 270 tr/min | 1,018 | 36,3 % | 55 % | aucune violation |
| Merlin, 3 040 tr/min | 1,367 | 56,0 % | 82 % | aucune violation |

Les pertes firing, pression, acoustique et réaction, les événements tardifs ou
pending, voix volées, délais tronqués, frontières invalides, chemins legacy,
garde-niveau, non-finitude et rendus hors budget sont tous restés à zéro.
Le LS3 franchit le gate de 0,97 mais sa marge n'est que de 4,8 % sur ce passage :
elle doit rester un contrôle obligatoire des lots suivants, pas être interprétée
comme un budget disponible pour multiplier les voix ou les mailles.

### Compatibilité afterfire et découverte transmise au lot suivant

Le smoke sur `Audio Physics Lab 689 Twin` conserve `physical=yes`, `compiled=yes`
et tous les compteurs de livraison à zéro. Rectification importante découverte
au lot suivant : le harness remplaçait silencieusement la calibration du moteur
par 900 K. Ce n'était donc pas le « seuil produit » ; le catalogue écrit 800 K.
Sous ce seuil forcé, 30 s de chauffe ne portent la paroi qu'à 327,8 °C et aucun
volume ne s'allume. Cette mesure reste une preuve de compatibilité du silencieux,
mais pas une mesure fidèle de l'afterfire catalogué.

Une mesure corrigée avec les paramètres réellement écrits, après 60 s de charge,
atteint environ 517 °C de paroi et produit 5,593 mg brûlés en sept excursions sur
le chemin sans audio. Avec le chemin audio, six excursions et 5,025 mg atteignent
l'observateur sans aucune perte, mais le niveau ne gagne qu'environ 0,8 dB au pic
et 0,2 dB au percentile 99,9 par rapport au contrôle sans carburant : le défaut
audible reste donc confirmé.

Un contrôle instrumental à 520 K, qui n'est **pas** une calibration proposée,
fait brûler 22,523 mg en dix événements, avec un pic de 7,144 kW et toujours
zéro perte audio/réaction. Le chemin réaction → graphe → observateur survit donc
au silencieux passif. Le prochain lot doit corriger l'état thermique et le
modèle d'allumage avec des paramètres physiques cohérents, pas simplement
abaisser ce seuil dans le catalogue.

## Validation de livraison

La reconstruction Release autoritaire a relié `EngineLab.exe`, les tests Core
et RealtimeRegression, ainsi que les harness de budget et d'afterfire. Les
tests ciblés gaz, graphe et transfert ont passé avant la suite complète.

```text
42/42 CTest passés
0 échec
temps réel CTest : 860,90 s
git diff --check : propre (hors avertissements LF/CRLF Windows)
```

La suite inclut notamment physique catalogue, échange gaz, overrun thermique,
AudioRender, transitoires audio, référence catalogue et rampes dyno produit CP2
et LS3. Les fichiers de mesure/profil sont conservés sous
`out/audit-2026-08-20/` et restent hors Git.

## Limites assumées

- Delany-Bazley est un modèle empirique de matériau poreux, pas une mesure du
  silencieux complet.
- Le volume annulaire n'a qu'un mode compact d'ordre réduit ; ses modes axiaux
  et circonférentiels supérieurs ne sont pas résolus.
- L'inertance des perforations demanderait diamètre de trou, pas/entraxe et
  épaisseur de tôle. Ces champs n'existent pas encore, donc rien n'est inventé.
- Cloisons, tubes de dérivation et chambres multiples doivent être écrits avec
  les composants composables du graphe ; ils ne sont pas cachés dans un type
  `muffler` monolithique.
- Les valeurs du catalogue sont `estimated-family`. Sans relevé dimensionnel et
  mesure de matériau, il serait malhonnête de parler d'équivalence OEM.
- L'écart de 5,138 dB prouve la sensibilité et la non-vacuité du modèle, pas sa
  précision face à un silencieux réel déterminé.

## Suite

Ce lot débloque l'afterfire : les sources de réaction distribuées disposent
maintenant d'un réseau d'échappement dont les jonctions, tapers, branches,
catalyseurs et silencieux ont une influence passive mesurable. La prochaine
étape peut donc porter sur l'inventaire combustible/oxygène, le délai
d'auto-allumage et la variabilité événementielle, sans ajouter des « samples de
pop » artificiels et sans refaire une maille gaz par phénomène acoustique.
