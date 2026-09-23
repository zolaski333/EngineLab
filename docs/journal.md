# Journal de mesures

Entrées courtes (10 lignes maximum), la plus récente en haut. Une entrée dit ce
qui a été mesuré, comment, et ce qui ne l'a pas été. Ce qui devient une règle
durable va dans `CLAUDE.md`. L'historique antérieur est dans `docs/archive/`.

## 2026-09-23 — Corrections après l'audit de l'afterfire

- Calage au démarrage en côte : réserve ECU ×2,12 gardée ~14 s après démarrage
  en plus du X-tau ; limitée au démarreur. À 4 s : 8 → 13/14 routiers ; à 25 s :
  13/14 inchangé. Calages déclarés : 2JZ ≤ 8 s (pauvre, écrasé par l'embrayage),
  Aircooled à 25 s. Accepté par le propriétaire : 2JZ 5 600 tr/min +14,81 →
  +15,11 % (tolérance 16 %), passage de rapport lancé après 20 s de ralenti.
- Couche crack (bruit > 4 kHz piloté par la réaction), rapport 1,0 non calibré,
  borné entre 0,8 et 2,5 : +3,9 dB en 4-8 kHz pendant les pops du 2JZ.
- **CP2 (standard et Full System) : le profil discret ne brûle rien** (0 % de
  200 mg, paroi 390-424 °C) ; à 650 K au lieu de 800 K : 18 %. Non corrigé.

## 2026-09-23 — Audit de l'afterfire

- **Régression de `d70e8b8`** : les deux CP2 et le LS3 calent au démarrage
  (1re, plein gaz, embrayage 0,9 s) dans `EngineLabAfterfireHarness` ; à
  `75f64fa` le même Twin labo démarre et atteint 4 284 tr/min. Aucun test ne
  l'a vu. `--trace` affiche désormais la trajectoire si l'armement échoue.
- Front audio d'une pétarade : 0,2-0,8 ms (médiane Twin 0,46, 2JZ 0,61),
  durée 2-4 ms. Énergie : centroïde 450-790 Hz, 0,2-0,5 % entre 4 et 8 kHz,
  0 au-dessus (coupure LR8 à 0,47 × couplage). Crête ON/OFF : -3 à +10 dB.
- Carburant continu à 18 % : 0,0 % brûlé (richesse 0,18 < 0,45). Même masse en
  paquets : 89 % brûlé (2JZ). Seul le moteur 16 authore l'afterfire.

## 2026-09-22 — Remise en ordre du projet

- Écoute à l'aveugle faite par le propriétaire contre de vrais moteurs : **aucun
  moteur n'est reconnaissable au-delà du nombre de cylindres**. Le rythme
  (ordre d'allumage) est juste ; le timbre ne ressemble à aucun moteur précis.
- L'afterfire ne sonne pas comme une pétarade, malgré une douzaine de commits.
- Le corpus `references/real-engine-audio/` ne permet pas de mesurer l'écart :
  10 enregistrements sur 10 ont un régime inconnu, 7 sont des proxys, et aucun
  n'est un CP2. Il n'y a jamais eu de cible sonore mesurable.
- Nouvelle direction : voir `VISION.md`. Moteur pilote CP2 (MT-07, Arrow),
  enregistré par le propriétaire.

## 2026-09-22 — Commit du travail de l'audit du 27 août

Travail en cours (34 fichiers : AFR, banc, lissage admission, reprise DFCO)
commité après un build Release sans avertissement et 44/44 tests verts. Deux
points à connaître :

- La reprise DFCO passe de 1,25× à **1,50× le ralenti**, ce qu'une note
  antérieure déconseillait (1,50× seul calait encore sans la correction du
  film carburant). Les deux corrections coexistent. À réexaminer seulement si
  un problème de ralenti ou de reprise réapparaît.
- Un terme de pertes mécaniques commun au-dessus de 4 000 tr/min a été ajouté
  pour recaler le haut régime : c'est une compensation, pas un modèle.
- Audit du 27 août : LS3 à 1,151× le temps réel en free-run, Merlin à 88 % d'un
  bloc audio en P99, sur la machine de référence, sans l'interface ouverte.
