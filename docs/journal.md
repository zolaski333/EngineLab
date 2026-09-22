# Journal de mesures

Entrées courtes (10 lignes maximum), la plus récente en haut. Une entrée dit ce
qui a été mesuré, comment, et ce qui ne l'a pas été. Ce qui devient une règle
durable va dans `CLAUDE.md`. L'historique antérieur est dans `docs/archive/`.

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
