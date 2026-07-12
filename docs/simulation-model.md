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

Le calcul des efforts alternatifs utilise la vitesse angulaire prédite au milieu
du sous-pas. La masse alternative comprend le piston et un tiers de la bielle.
Les frottements piston/chemise suivent une loi de Stribeck configurable par
cylindre : Coulomb, force de décollage, vitesse de transition et terme visqueux.
La force latérale vient de l'angle réel de la bielle et des efforts gaz/inertie.

## Réseau gazeux conservatif

Le plénum, les runners, les cylindres, les primaires et les collecteurs stockent
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

`FlamePhysicsModel` est indépendant du simulateur. À l'étincelle, il crée un
noyau puis fait progresser un front ellipsoïdal dans la chambre mobile. La
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

Le runtime résout le glissement entre vilebrequin et arbre réfléchi par le
rapport total. Près de la synchronisation, le couple requis pour verrouiller
les deux inerties est calculé puis borné par la capacité de l'embrayage ; en
glissement, une loi continue bornée transmet le couple et sa réaction opposée
au vilebrequin. L'inertie des roues augmente la masse longitudinale équivalente.

Un passage de rapport suit une enveloppe débrayage/changement/réembrayage.
`automatic_shifting` active les seuils de montée et descente ; sans lui, les
commandes manuelles utilisent la même machine d'état. Le moteur peut caler si
le couple réfléchi dépasse le couple disponible à faible régime.

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
- réaction globale essence/oxygène, sans cinétique chimique multi-espèces ;
- front de flamme ellipsoïdal et turbulence agrégée, sans champ spatial 3D ;
- film d'injection indirecte agrégé, sans suivi de gouttelettes ni spray 3D ;
- délai end-gas corrélé globalement, sans cinétique chimique multi-espèces ;
- transferts thermiques et blow-by encore semi-empiriques ;
- essence quatre temps uniquement dans la composition runtime actuelle ;
- moteur radial sans cinématique de bielle maîtresse/articulée.

Ces limites doivent être étendues par de nouveaux modèles et interfaces, pas
par des exceptions propres à un preset.
