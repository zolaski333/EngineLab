# Livraison des phases 3, 4 et 5

État vérifié le 13 juillet 2026. Ce document décrit ce qui est réellement
implémenté dans EngineLab et les limites qui restent ouvertes. Il complète
[`phase-0-1-2.md`](phase-0-1-2.md).

## Phase 3 — Couple issu du travail cylindre

Le couple de fonctionnement reste calculé cylindre par cylindre depuis la
pression résolue et la géométrie bielle-manivelle. `IndicatedWorkModel` intègre
en plus la boucle signée `∮(P cylindre - P ambiante) dV` de chaque cylindre par
trapèzes. À la fin de chaque cycle, EngineLab publie :

- le travail indiqué par cycle et par cylindre ;
- le travail total du moteur, l'IMEP, la puissance indiquée et le couple moyen
  équivalent P·dV ;
- l'écart avec le couple instantané de pression, utile pour diagnostiquer les
  pertes mécaniques et le remplissage.

L'ancien modèle de couple moyen n'est jamais réinjecté dans le vilebrequin. Il
reste une référence de diagnostic uniquement.

L'allumage est maintenant réellement individuel : l'ECU programme une
étincelle par cylindre, puis un délai d'inflammation calibré dépendant de la
température, de la pression, de la richesse et des gaz résiduels précède la
naissance du noyau de flamme. Le front Metghalchi-Keck existant conserve une
propagation explicite et prend en compte mélange, turbulence, dilution,
pertes aux parois, avance et raté. Les coefficients de délai, de dilution et
de transfert thermique sont configurables, sérialisés et validés.

Conséquence importante : arbre à cames, écoulement de culasse, longueur de
runner, richesse, résiduels et avance agissent par la charge piégée, la pression
et la combustion. Aucun multiplicateur de couple spécifique à un preset ne
simule artificiellement ces effets.

## Phase 4 — Chaîne mécanique et transmission énergétique

`DrivelineModel` possède l'état mécanique de l'embrayage, de la boîte, du
différentiel, des roues motrices et du véhicule. Il résout une chaîne d'énergie
signée entre le vilebrequin et la route :

- inerties de vilebrequin, bielles, volant, arbre d'entrée de boîte,
  différentiel et roues ;
- marche arrière, point mort et rapports avant dans la même machine d'état ;
- passage temporisé avec débrayage, changement de rapport, réembrayage et
  réduction de couple pendant le passage ;
- embrayage limité par sa capacité physique, échauffement par glissement,
  refroidissement, fading puis perte de capacité à haute température ;
- roue motrice comme degré de liberté, glissement longitudinal du pneu, limite
  d'adhérence, traînée, roulement et masse du véhicule ;
- frein de roue commandé par la flèche gauche ;
- télémétrie de puissance dissipée, énergie cumulée, énergie stockée et résidu
  du bilan énergétique.

Le couplage raide pneu–roue–véhicule est intégré par sous-pas mécaniques de
1 ms. Cela permet aux vitesses de simulation élevées de rester stables sans
affaiblir la raideur du pneu ni masquer une divergence avec un clamp arbitraire.
Les limites de couple d'embrayage et d'effort du pneu sont des capacités
physiques configurées, pas des correctifs numériques.

## Phase 5 — Culasses, distribution et admission résonante

Chaque arbre à cames, global ou propre à une banque, peut définir :

- des profils de levée admission et échappement échantillonnés arbitrairement ;
- des courbes coefficient de décharge/levée distinctes pour admission et
  échappement ;
- une calibration continue RPM/charge d'avance admission, d'avance
  échappement et de multiplicateur de levée ;
- la fréquence de réponse de l'actionneur VVT/VVL.

`ValveTrainModel` interpole ces données, fait évoluer les actionneurs de façon
continue et fournit au réseau gazeux la levée, la phase et le coefficient de
décharge réellement atteints. Le profil haut discret reste compatible avec
les anciens presets, mais n'est plus la seule manière de faire varier la
distribution.

Chaque runner d'admission possède déjà son volume géométrique. Il est désormais
couplé au plénum par `HelmholtzRunnerModel`, un oscillateur acoustique amorti
dont la fréquence dépend de la section, de la longueur, du volume et de la
vitesse locale du son. Sa pression résonante modifie l'admittance du transfert
conservatif ; elle n'ajoute ni masse ni couple. Amortissement, couplage et
amplitude maximale sont configurables.

Le catalogue fournit des courbes de débit cohérentes pour les cames livrées et
une calibration VVT/VVL continue pour la came sportive atmosphérique. Tous les
nouveaux champs ont un round-trip JSON/YAML et une validation de plage et
d'ordre des échantillons.

## Vérifications exécutées

- boucle P·dV rectangulaire connue : aire signée retrouvée exactement ;
- interpolation d'une courbe de débit et convergence de l'actionneur VVT/VVL ;
- fréquence Helmholtz plus élevée pour un runner court que pour un runner long ;
- marche arrière signée, dissipation d'embrayage et freinage ;
- simulation complète : travail indiqué, IMEP, puissance et résonance finis et
  positifs quand le moteur fonctionne ;
- round-trips JSON/YAML des nouveaux contrats ;
- chargement et simulation de tout le catalogue ;
- harnais déterministe I4, V8, V-twin et radial avec comparaison de référence.

## Ce qui n'est pas livré par les phases 3 à 5

- pas de solveur d'ondes 1D maillé dans les runners ou l'échappement : le
  modèle Helmholtz est un premier ordre 0D physiquement paramétré ;
- pas de CFD de culasse, de chambre ou de spray, ni de données flowbench
  certifiées pour tous les presets ;
- pas d'import CSV de profils de came depuis l'interface ; les profils
  arbitraires sont décrits dans la configuration YAML/JSON ;
- pas de torsion multi-vilebrequin indépendante, de jeu d'engrenage, de
  synchroniseurs détaillés, d'ABS, de suspension ou de transfert de charge ;
- le pneu est un modèle longitudinal agrégé, pas un modèle Pacejka complet ;
- pas de validation A/B exécutée contre ES2D ni contre un banc moteur réel ;
- pas de refonte complète de l'éditeur de configuration et de l'UI au niveau
  d'ES2D, ni de scripting Piranha/Lua ;
- pas d'export WAV hors ligne du renderer JUCE complet.

Ces limites sont déclarées. Elles ne sont pas compensées par des coefficients
cachés, des valeurs forcées par moteur ou des branches propres à un preset.
