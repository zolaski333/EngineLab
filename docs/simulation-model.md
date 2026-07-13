# Modèle de simulation

EngineLab est un simulateur moteur temps réel à volumes de contrôle. Il ne
prétend pas être un solveur CFD, mais les échanges de masse, d'énergie et de
quantité de mouvement suivent désormais des contrats conservatifs vérifiables.

## Rotation et couple

À chaque sous-pas, le simulateur intègre `alpha = couple_net / inertie`, puis la
vitesse et l'angle du vilebrequin. Le couple de pression est calculé cylindre par
cylindre à partir de la pression résolue, de l'aire du piston et du bras de
levier bielle-manivelle exact. C'est la source de couple en fonctionnement
normal. L'ancien modèle de travail moyen reste publié dans la télémétrie pour
comparer et diagnostiquer, mais il n'est plus mélangé au couple physique.

`IndicatedWorkModel` intègre séparément la boucle signée
`∮(P cylindre - P ambiante) dV` par trapèzes. Un franchissement de cycle publie
le travail indiqué par cylindre ; leur somme donne l'IMEP, la puissance
indiquée et le couple moyen équivalent P·dV. Cette seconde lecture permet de
contrôler l'énergie par cycle sans lisser le couple instantané du vilebrequin.

Le calcul des efforts alternatifs utilise l'état de `MechanicalKinematics`,
commun à la simulation et au rendu. La masse alternative comprend le piston et
un tiers de la bielle.
Les frottements piston/chemise suivent une loi de Stribeck configurable par
cylindre : Coulomb, force de décollage, vitesse de transition et terme visqueux.
La force latérale vient de l'angle réel de la bielle et des efforts gaz/inertie.

## Réseau gazeux conservatif

Chaque chemin possède son propre plénum ; les runners utilisent leur volume
géométrique `πr²L`. Les plénums, runners, cylindres, primaires et collecteurs stockent
les espèces, l'énergie interne et un vecteur de quantité de mouvement 2D. Les
écoulements utilisent les relations isentropiques subsoniques ou bloquées, la
pression totale directionnelle, la section réelle et le coefficient de débit.

Un transfert transporte les espèces, l'enthalpie de stagnation et la quantité
de mouvement. L'énergie totale inclut l'énergie cinétique macroscopique ; la
dissipation de vitesse la reconvertit en chaleur. Une recherche monotone de
l'équilibre de pression empêche le dépassement sans l'ancienne limite
arbitraire de 35 % de la masse par appel. La vitesse du gaz est bornée par la
vitesse du son locale en conservant l'énergie.

`Cv` et `gamma` dépendent de la composition. L'air froid reste proche de 1,40,
les produits brûlés proches de 1,26, et la vapeur de carburant apporte sa forte
capacité thermique. Ces propriétés pilotent compression/détente adiabatique,
vitesse du son et débit critique.

Pendant le croisement des soupapes, les deux restrictions sont évaluées depuis
le même état initial du cylindre. Leurs deltas de masse, espèces, enthalpie et
momentum sont ensuite validés et appliqués en une transaction conservative.
L'admission ne peut donc plus modifier artificiellement le gradient vu par
l'échappement simplement parce qu'elle est calculée en premier.

## Combustion par propagation de flamme

`FlamePhysicsModel` est indépendant du simulateur. Une commande d'étincelle
propre à chaque cylindre démarre d'abord un délai d'inflammation calibré par la
température, la pression, le ratio d'équivalence et les gaz résiduels. À son
terme, le modèle crée un noyau puis fait progresser un front ellipsoïdal dans la chambre mobile. La
vitesse laminaire suit la corrélation Metghalchi-Keck en fonction du ratio
d'équivalence, de la température et de la pression. La turbulence liée à la
vitesse moyenne du piston accélère le front ; la dilution par les gaz résiduels
réduit vitesse et rendement.

Le volume brûlé donne directement l'avancement de réaction. La réaction reçoit
un nombre absolu de moles à brûler, évitant l'erreur qui consistait à appliquer
chaque incrément à la quantité restante. L'énergie libérée utilise le PCI et la
chimie du carburant configuré. Pression cylindre, vitesse de flamme, fraction
brûlée et rendement restent observables par cylindre.

Le knock utilise une intégrale de Livengood-Wu par cylindre. Le délai
d'auto-inflammation dépend de la pression, de la température, de l'octane et du
ratio d'équivalence du gaz non brûlé. Quand l'intégrale atteint l'unité, une
partie du reliquat s'auto-enflamme dans la `GasCell`, ce qui produit à la fois
la hausse de pression, le signal de knock, le retrait d'avance ECU et l'audio.

Le front turbulent utilise une contribution proportionnelle à l'intensité de
turbulence entraînée par le piston. Cette fermeture place la vitesse intégrale
dans la plage moteur de 18–40 m/s selon le régime et évite une combustion qui
se poursuivrait artificiellement après l'ouverture échappement. La flamme est
arrêtée à l'EVO ; le carburant restant rejoint alors l'inventaire imbrûlé de
l'échappement.

L'AFR affiché provient de l'oxygène et du carburant réellement enfermés, sans
compter l'azote résiduel comme air frais. Un correcteur lambda par cylindre
apprend les pertes de transport et de film du cycle précédent. Le réglage UI
d'allumage est un trim autour de la carte propre au moteur.

Pour un turbocompresseur, l'énergie cinétique d'arbre évolue avec la puissance
turbine moins la puissance compresseur et les pertes de palier. La section de
turbine et la wastegate ferment réellement le collecteur d'échappement : la
contre-pression, le spool et le boost résultent donc du réseau de gaz et de
l'inertie, et non d'une interpolation directe du régime moteur.

Les pertes de chaleur aux parois et la sensibilité à la dilution résiduelle ne
sont plus des constantes internes : leurs calibrations sont décrites dans le
fichier moteur, sérialisées et validées.

## Distribution et résonance d'admission

`ValveTrainModel` évalue la came de la banque du cylindre. Les profils de levée
admission/échappement sont échantillonnés librement ; une seconde courbe
échantillonnée transforme la levée effective en coefficient de décharge. Une
table continue RPM/charge commande l'avance admission, l'avance échappement et
le multiplicateur de levée. Les actionneurs suivent leur consigne avec une
réponse continue configurable, ce qui évite les changements instantanés de
remplissage.

`HelmholtzRunnerModel` associe à chaque runner un oscillateur amorti. Sa
fréquence propre est calculée depuis la section, la longueur, le volume du
runner/plénum et la vitesse du son. La différence de pression excite le mode ;
sa pression résonante modifie l'admittance de la restriction conservative. Le
modèle ne crée donc ni masse ni énergie de combustion artificielle.

## Injection

La fenêtre angulaire, le mode, le débit nominal et la température restent
configurables. Le débit instantané varie maintenant comme la racine de la
pression différentielle entre le rail et le volume récepteur.

- En injection directe, le carburant entre dans le cylindre et sa chaleur
  latente refroidit la charge selon un rendement configurable.
- En injection indirecte, une fraction configurable forme un film liquide sur
  le port. Ce film persiste entre les sous-pas et s'évapore avec une constante
  de temps dépendante de la température avant d'atteindre la chambre.

La quantité mesurée, la vapeur réellement présente à l'étincelle et le
carburant consommé sont distincts. Une capacité d'injecteur insuffisante ou une
vaporisation trop lente réduit donc réellement la combustion.

La consigne n'est plus la masse d'air estimée par le modèle mean-value. Elle
vient de l'oxygène réel et de la dernière charge piégée ; le carburant vaporisé
ou stocké en film est déduit avant de commander l'injecteur. Le couple pression
est utilisé dès le premier sous-pas, sans bascule arbitraire après 10 ms.

`injector_flow_mg_s` est le débit massique d'un injecteur. Avec une densité
d'essence de 0,745 kg/L, 400 cc/min vaut environ 4 970 mg/s. Les valeurs par
défaut DI et port diffèrent volontairement à cause de la pression et de la
fenêtre disponible.

## Transmission

`DrivelineModel` résout une roue motrice comme degré de liberté, distincte de la
vitesse du véhicule. L'embrayage compare la vitesse moteur à celle de l'arbre
de boîte, applique une capacité de couple et renvoie la réaction opposée au
vilebrequin. Le travail de glissement chauffe l'embrayage ; son refroidissement,
son fading et sa température de défaillance sont configurables.

Marche arrière, point mort et rapports avant utilisent la même machine d'état.
Un passage suit une enveloppe débrayage/changement/réembrayage accompagnée d'une
réduction de couple. `automatic_shifting` réutilise cette machine avec des
seuils de montée et de descente.

L'inertie d'entrée de boîte est réfléchie par le rapport total ; le
différentiel et les roues conservent leurs inerties propres. La différence
entre vitesse de surface du pneu et vitesse véhicule génère l'effort
longitudinal jusqu'à la limite `μN`. Traînée, roulement et frein de roue
dissipent l'énergie. Ce couplage raide est intégré à 1 ms dans le runtime et un
résidu énergétique est publié avec l'énergie stockée et dissipée.

## Solveur

La cadence s'adapte au régime et au pas angulaire maximum. Les fichiers moteur
sont refusés si leur fréquence maximale ne peut pas tenir la résolution promise
au rupteur. La recherche d'équilibre gazeux effectue d'abord un essai au débit
demandé et n'emploie la bissection que si ce débit inverserait le gradient de
pression, afin de préserver le temps réel.

Le banc utilise un régime filtré pour son asservissement : l'ondulation de
couple cylindre par cylindre ne remet plus à zéro une stabilisation pourtant
physiquement correcte.

## Limites assumées

- volumes 0D avec momentum vectoriel, et non CFD ou acoustique 1D maillée ;
- résonance d'admission limitée à un mode Helmholtz agrégé par runner ;
- réaction globale essence/oxygène, sans cinétique chimique multi-espèces ;
- front de flamme ellipsoïdal et turbulence agrégée, sans champ spatial 3D ;
- film d'injection indirecte agrégé, sans suivi de gouttelettes ni spray 3D ;
- délai end-gas corrélé globalement, sans cinétique chimique multi-espèces ;
- transferts thermiques et blow-by encore semi-empiriques ;
- essence quatre temps uniquement dans la composition runtime actuelle ;
- plusieurs vilebrequins contraints par un rapport cinématique rigide, sans
  dynamique torsionnelle ou jeu d'engrenage indépendant.
- pneu longitudinal agrégé, sans suspension, transfert de charge, ABS ou modèle
  Pacejka complet.

Ces limites doivent être étendues par de nouveaux modèles et interfaces, pas
par des exceptions propres à un preset.
