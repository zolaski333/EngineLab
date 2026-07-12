# Modèle de simulation

EngineLab reste un simulateur perceptuel, mais ses grandeurs principales ont
une cohérence dimensionnelle vérifiable.

## Rotation

À chaque pas fixe, le runtime intègre `alpha = couple_net / inertie`, puis
`omega += alpha * dt`. Le régime et l'angle proviennent de cette vitesse. Le
couple net réunit explicitement couple au vilebrequin, démarreur, frein et
efforts alternatifs des pistons.

Le couple au vilebrequin est le couple indiqué diminué des frottements et pertes
de pompage. Chaque terme reste exposé dans `EngineState`, afin que banc et UI ne
confondent jamais couple moteur et couple d'accélération.

## Air, carburant et combustion

L'admission, le plénum, chaque runner, chaque cylindre et chaque runner
d'échappement sont des volumes gazeux conservatifs. Ils stockent les moles de
carburant, d'oxygène, de gaz brûlés et d'inerte, ainsi que l'énergie interne et
la quantité de mouvement 1D. Pression et température sont dérivées de cet état.
Les transferts utilisent un écoulement compressible isentropique, subsonique ou
bloqué, et transportent les espèces, l'énergie et la quantité de mouvement sans
les recréer.

Le volume cylindre suit exactement la géométrie bielle-manivelle. Compression
et détente sont adiabatiques, les soupapes relient le cylindre aux runners selon
leur levée réelle, la réaction essence/oxygène libère sa chaleur dans le gaz et
les pertes aux parois et le blow-by sont explicites. Le couple de pression issu
du travail `p dV` est mélangé au modèle énergétique calibré : ce compromis garde
les presets stables tout en rendant la pression et les échanges gazeux causaux.

Le solveur est multi-taux. Sa fréquence mécanique vaut 2 kHz par défaut et
augmente avec le régime jusqu'à 20 kHz afin de ne pas dépasser 2 degrés de
vilebrequin par pas. Ces trois valeurs et le nombre de sous-pas gazeux sont
configurables. Le calcul des accélérations de piston et des couples alternatifs
utilise la formule géométrique exacte du système bielle-manivelle.

La masse de carburant découle de la masse d'air et de l'AFR ECU. Le travail
cyclique utilise le pouvoir calorifique de l'essence et un rendement corrigé
par compression, richesse, avance, usure et dégâts. Avance et AFR proviennent
de cartes régime/charge interpolées, avec corrections en direct.

L'AFR publié est l'AFR réellement obtenu après enrichissement de chauffe. La
puissance thermique est calculée depuis le débit massique de carburant par
seconde. Les ratés sont tirés par cylindre avant la fenêtre d'allumage et
retirent réellement leur impulsion de couple.

L'avance est interpolée dans la courbe d'allumage de la configuration puis
corrigée par la commande live. Le rupteur possède son propre régime, sa durée de
coupure et un latch, indépendamment du simple affichage de la zone rouge. Un
profil d'arbre à cames haut peut être sélectionné par banque selon régime et
papillon ; la levée active est partagée par la physique et l'animation.

## Transmission et véhicule

Le couple d'embrayage est signé. Il accélère ou freine à la fois le moteur et la
transmission selon leur différence de vitesse, puis traverse le rapport engagé
et le pont. Masse du véhicule, rayon du pneu, traînée et résistance au roulement
ferment la boucle. La roue peut donc entraîner le moteur en décélération : le
frein moteur n'est ni écrêté ni simulé comme une charge positive séparée.

## Banc de puissance

Le banc stabilise le régime par charge freinée, moyenne les échantillons signés
de couple et publie valeurs brutes et corrigées atmosphériquement. Les valeurs
négatives instantanées ne sont jamais écrêtées avant la moyenne.

## Limites assumées

- réseau gazeux 0D par volumes avec quantité de mouvement 1D, et non CFD/ondes
  acoustiques 1D à maillage spatial ;
- réaction chimique globale essence/oxygène, sans cinétique multi-espèces ;
- couple de pression volontairement mélangé à un modèle énergétique calibré ;
- températures ramenées à des réseaux thermiques de premier ordre ;
- échappement acoustique réduit à retards, restriction et résonances ;
- essence quatre temps uniquement pour le moment.

Une future stratégie 2T ou diesel doit implémenter les interfaces existantes,
sans ajouter de branches dans le modèle essence.
