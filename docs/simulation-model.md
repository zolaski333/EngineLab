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

La pression collecteur est intégrée par conservation de masse dans un plénum
paramétrable. Le débit entrant utilise le diamètre et l'ouverture effective du
papillon via un modèle d'écoulement compressible isentropique (prenant en compte le débit bloqué/choked flow) ; le débit consommé utilise régime, cylindrée, densité et rendement
volumétrique dépendant du régime, des deux arbres à cames et de la
contre-pression.

Pour assurer la stabilité numérique sous fortes charges et à basse fréquence d'affichage, la simulation utilise un sous-découpage dynamique du pas de temps (sub-stepping) à une fréquence minimale de 240 Hz. De plus, le calcul des accélérations de piston et des couples alternatifs de rappel utilise la formule géométrique exacte du système bielle-manivelle.

La masse de carburant découle de la masse d'air et de l'AFR ECU. Le travail
cyclique utilise le pouvoir calorifique de l'essence et un rendement corrigé
par compression, richesse, avance, usure et dégâts. Avance et AFR proviennent
de cartes régime/charge interpolées, avec corrections en direct.

L'AFR publié est l'AFR réellement obtenu après enrichissement de chauffe. La
puissance thermique est calculée depuis le débit massique de carburant par
seconde. Les ratés sont tirés par cylindre avant la fenêtre d'allumage et
retirent réellement leur impulsion de couple.

## Banc de puissance

Le banc stabilise le régime par charge freinée, moyenne les échantillons signés
de couple et publie valeurs brutes et corrigées atmosphériquement. Les valeurs
négatives instantanées ne sont jamais écrêtées avant la moyenne.

## Limites assumées

- pas de résolution 1D des gaz ni de pression cylindre à chaque degré ;
- combustion représentée par un travail moyen avec ondulation angulaire ;
- températures ramenées à des réseaux thermiques de premier ordre ;
- échappement acoustique réduit à retards, restriction et résonances ;
- essence quatre temps uniquement pour le moment.

Une future stratégie 2T ou diesel doit implémenter les interfaces existantes,
sans ajouter de branches dans le modèle essence.
