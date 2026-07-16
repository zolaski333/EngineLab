# Modèle de simulation

EngineLab est un simulateur temps réel à volumes de contrôle. Ses équations
sont conçues pour rester cohérentes et observables dans un budget interactif ;
elles ne constituent pas un modèle CFD ni une homologation de performances.

## Intégration et résolution angulaire

L'appel public `EngineSimulator::step(dt, controls)` est découpé en sous-pas.
La fréquence demandée est le maximum entre la cadence mécanique minimale,
incluant `gasSubsteps`, et la cadence requise pour respecter
`maximumCrankDegreesPerStep` au régime courant. Elle reste bornée par
`maximumMechanicalFrequencyHz`.

La validation refuse une configuration dont la fréquence maximale ne pourrait
pas tenir la résolution angulaire promise au rupteur. Le solveur publie le pas
angulaire réellement atteint et le dépassement éventuel ; il ne masque pas une
sous-résolution derrière une fréquence nominale.

## Cinématique, pression et couple

`MechanicalKinematics` résout la géométrie bielle-manivelle à partir des
crankshafts et journaux normalisés. Les bielles conventionnelles, maîtresses et
articulées partagent la même API de position, vitesse, accélération et bras de
levier. Les dimensions de deck, hauteur de compression, offset d'axe et rayon
du journal articulé participent à la position du PMH lorsque renseignés.

À chaque sous-pas, la pression de chaque cylindre agit sur l'aire du piston et
le bras de levier exact. Les efforts gazeux, alternatifs, de friction, de
démarreur, de charge et de transmission forment le couple net, puis :

```text
accélération angulaire = couple net / inertie équivalente
```

Le couple en fonctionnement vient de cette pression résolue. Aucun couple moyen
empirique n'est mélangé à la dynamique du vilebrequin.

En parallèle, `IndicatedWorkModel` intègre par trapèzes la boucle signée
`∮(P_cylindre - P_ambiante)dV`. Au changement de cycle, il publie travail
indiqué, IMEP, puissance indiquée et couple moyen équivalent. Cette lecture sert
au diagnostic et au bilan, sans devenir une seconde source de couple.

Les frottements piston/chemise utilisent une loi de Stribeck par cylindre :
force de décollage, Coulomb, transition à basse vitesse et terme visqueux. Le
sens est défini même au voisinage de la vitesse nulle, ce qui évite une friction
qui accélérerait artificiellement le piston.

## Réseau gazeux conservatif

L'atmosphère, les plénums, runners, cylindres, primaires et collecteurs sont des
`GasCell`. Une cellule stocke les quantités de matière par espèce, l'énergie
interne, le volume, une orientation, une section caractéristique et une
quantité de mouvement 2D.

Les restrictions utilisent leur aire réelle et un coefficient de décharge.
Le débit devient critique lorsque le rapport de pression atteint la condition
sonique ; sinon il suit la relation isentropique subsonique. La pression
dynamique directionnelle est signée : un momentum dirigé vers la restriction
augmente sa pression totale effective, un momentum opposé la diminue.

Un transfert transporte simultanément :

- oxygène, gaz inerte, vapeur de carburant et produits brûlés ;
- masse et quantité de mouvement sur les deux axes ;
- enthalpie de stagnation et énergie cinétique macroscopique.

Une recherche d'équilibre borne le transfert avant l'inversion non physique du
gradient. La dissipation d'un momentum excédant la vitesse du son reconvertit
l'énergie cinétique en chaleur. Les propriétés effectives `Cv`, `gamma`, masse
molaire et vitesse du son dépendent de la composition.

La température et la pression restent dérivées de toute l'énergie interne
conservée, y compris au-delà de la plage thermique habituelle du moteur : aucun
plafond d'affichage ne peut masquer une réserve d'énergie dans une cellule.

Pendant le croisement des soupapes, admission et échappement sont évalués à
partir du même état de départ puis appliqués ensemble. Cette transaction évite
qu'un ordre d'appel arbitraire modifie le gradient vu par la seconde soupape.

Le modèle reste toutefois 0D par volume. La quantité de mouvement apporte une
inertie directionnelle ; elle ne transforme pas un runner en tube maillé où une
onde se propage spatialement.

## Distribution et admission

`ValveTrainModel` évalue le profil de la banque du cylindre. Les profils de
levée et courbes levée/coefficient de débit sont échantillonnés et interpolés.
Une calibration RPM/charge peut commander en continu l'avance admission,
l'avance échappement et le multiplicateur de levée ; les actionneurs suivent
leur cible à une fréquence de réponse configurable. Le profil haut commuté est
conservé pour les anciennes configurations.

`HelmholtzRunnerModel` associe un mode amorti à chaque runner. Sa fréquence
dépend de la section, de la longueur, du volume et de la vitesse locale du son.
La pression de ce mode modifie l'admittance de la restriction conservative :
elle ne crée ni masse ni bonus de couple indépendant du remplissage.

## Injection et mélange

Le débit injecteur dépend de sa capacité nominale et de la racine du différentiel
de pression. Pour une injection indirecte, `rail_pressure_bar` est la pression
différentielle régulée par rapport au collecteur : le débit ne s'effondre donc
pas sous boost. Pour une injection directe, c'est une pression de rail absolue
et la contre-pression instantanée du cylindre est soustraite.

- En injection directe, le carburant rejoint le cylindre et sa chaleur latente
  refroidit la charge selon le rendement configuré.
- En injection indirecte, une fraction rejoint un film liquide persistant sur
  le port. Son évaporation dépend de la température et d'une constante de temps.

La masse commandée, le film, la vapeur disponible à l'étincelle et le carburant
réellement consommé restent distincts. Une fenêtre trop courte, un injecteur
sous-dimensionné ou un film lent réduit donc le carburant effectivement brûlé.

L'AFR et lambda télémétrés proviennent des espèces piégées. Un correcteur par
cylindre apprend les pertes de transport du cycle précédent. Sa bande passante
dépend de la durée du cycle et, en injection indirecte, de la constante de
vaporisation du film ; cela évite la chasse lambda des gros moteurs lents. Les tables ECU
fixent la cible AFR et l'avance en fonction du régime et d'une charge
normalisée ; enrichissement d'accélération, démarrage à froid, température,
knock et limiteur s'appliquent ensuite.

## Allumage, flamme et knock

Une étincelle est planifiée par cylindre. Avant la naissance du noyau, un délai
d'inflammation dépend de la pression, de la température, de l'équivalence et
des résiduels. La vitesse laminaire suit une corrélation de type
Metghalchi-Keck ; une fermeture par vitesse moyenne du piston ajoute la
turbulence, tandis que la dilution réduit vitesse et rendement.

La géométrie de progression actuelle est un volume effectif cylindrique :

```text
V_brûlé = π × trajet_radial² × trajet_axial
```

Les deux trajets sont bornés par le rayon d'alésage et la hauteur instantanée
équivalente de chambre. Il ne s'agit ni d'un front ellipsoïdal, ni d'une surface
3D résolue. La fraction géométrique commande un nombre absolu de moles à faire
réagir ; l'énergie libérée emploie le PCI et la stœchiométrie du carburant.

Le knock utilise une intégrale de Livengood-Wu sur le gaz de fin de combustion.
Lorsque son seuil est atteint, une part du reliquat s'auto-enflamme réellement
dans la cellule, augmente la pression et alimente la télémétrie. L'ECU retire
ensuite de l'avance. Cette corrélation globale n'est pas une cinétique chimique
multi-espèces.

## Suralimentation

Le turbocompresseur suit un bilan de puissance : turbine moins compresseur et
pertes de palier, intégré avec l'inertie d'arbre. Les sections de turbine et de
wastegate influencent le débit du collecteur et donc la contre-pression, le
spool et le rapport de pression. Le compresseur volumétrique utilise une
fermeture distincte, sans prétendre modéliser une carte compresseur complète.

## Échappement

Chaque `ExhaustPathConfig` peut contenir un DAG personnalisé. Pipes, jonctions,
résonateurs, silencieux, catalyseurs et sorties sont compilés en routes ; les
pertes des segments communs sont en série et les branches aval en parallèle.
Les routes fournissent longueur, délai, restriction, résonance et gain aux
événements audio. Pour le gaz, le compilateur réduit le DAG en une gorge d'entrée
par cylindre, un volume et une longueur moyenne par chemin, puis une conductance
de sorties parallèles atténuée par `1 / √(1 + K_équivalent)`. Ces propriétés
alimentent réellement les transferts runner/collecteur/atmosphère. Aucune
`GasCell` n'est cependant créée par nœud : les branches restent une fermeture
0D agrégée. Sans graphe, la géométrie historique pilote le débit et est aussi
convertie en primaires, collecteur, silencieux et sortie pour les métriques de
route. Voir [custom-exhaust.md](custom-exhaust.md).

Le réseau thermodynamique et le renderer audio n'ont pas la même discrétisation :
le premier utilise des volumes agrégés, le second des délais/guides d'onde et
une convolution. Cette séparation évite de présenter l'acoustique DSP comme un
solveur thermodynamique d'ondes.

## Transmission et véhicule

`DrivelineModel` possède embrayage, arbre de boîte, différentiel, roue motrice
et véhicule. Marche arrière, point mort et rapports avant partagent une machine
d'état avec débrayage, changement, réembrayage et réduction de couple.

La capacité de l'embrayage borne son couple. Le travail de glissement le
chauffe ; refroidissement et fading font évoluer cette capacité. La roue reste
un degré de liberté distinct de la vitesse véhicule. Son glissement génère une
force longitudinale bornée par l'adhérence, puis traînée, roulement et frein
dissipent l'énergie. Le runtime sous-échantillonne ce couplage à 1 ms.

En mode véhicule, la charge manuelle est une force résistante longitudinale ;
elle revient au vilebrequin uniquement par la roue, la boîte et l'embrayage. En
mode dyno, le frein agit directement au vilebrequin et le véhicule est découplé.
Les deux chemins ne sont jamais appliqués simultanément.

Les bilans publient énergie stockée, dissipée et résidu. Le modèle n'inclut ni
suspension, ni transfert de charge, ni ABS, ni synchroniseurs détaillés, ni
Pacejka complet.

## Limites et interprétation

- essence quatre temps seulement malgré la présence de types réservés à des
  extensions futures ;
- volumes gazeux 0D et mode Helmholtz agrégé, sans CFD ou acoustique 1D maillée ;
- réaction globale, turbulence, parois, blow-by, film et knock semi-empiriques ;
- pas de spray, champ de température ou front de flamme 3D ;
- vilebrequins multiples liés par des rapports rigides, sans torsion propre ;
- paramètres de catalogue non certifiés par flowbench ou banc moteur ;
- puissance et couple utiles à la comparaison interne, pas à une décision
  d'ingénierie ou de tuning sur un véhicule réel.

Les tests vérifient invariants, finitude, tendances et régressions. Ils ne
remplacent pas un étalonnage expérimental.
