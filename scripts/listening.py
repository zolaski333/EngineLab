#!/usr/bin/env python3
"""Feuilles de reponse et depouillement pour l'ecoute A/B en aveugle.

Deux modes :

  python scripts/listening.py sheets  --pack <dossier> --listeners 5
  python scripts/listening.py report  --pack <dossier>

`sheets` ecrit des CSV vierges dans <dossier>/responses/. Il ne lit jamais
`listening-key.json`, donc le dossier remis aux auditeurs reste aveugle.

`report` relit la cle et les CSV remplis, puis produit un rapport markdown et
un JSON. Il refuse une feuille incomplete ou hors domaine plutot que de
deviner : une reponse manquante est une reponse manquante.

Stdlib uniquement.
"""

from __future__ import annotations

import argparse
import csv
import json
import math
import sys
from pathlib import Path

COLUMNS = [
    "pair",
    "realism_A",
    "realism_B",
    "preference_A",
    "preference_B",
    "most_realistic",
    "preferred",
    "comment",
]

SCORE_COLUMNS = ["realism_A", "realism_B", "preference_A", "preference_B"]
CHOICE_COLUMNS = ["most_realistic", "preferred"]


# --------------------------------------------------------------------------- sheets


def make_sheets(pack: Path, listeners: int) -> int:
    key_path = pack / "listening-key.json"
    if not key_path.is_file():
        print(f"error: {key_path} not found", file=sys.stderr)
        return 1
    with key_path.open(encoding="utf-8") as handle:
        key = json.load(handle)
    pair_numbers = sorted(entry["pair"] for entry in key["pairs"])

    out_dir = pack / "responses"
    out_dir.mkdir(parents=True, exist_ok=True)
    written = []
    for index in range(1, listeners + 1):
        path = out_dir / f"listener_{index:02d}.csv"
        if path.exists():
            print(f"skip (exists): {path}")
            continue
        with path.open("w", encoding="utf-8", newline="") as handle:
            writer = csv.writer(handle)
            writer.writerow(COLUMNS)
            for pair in pair_numbers:
                writer.writerow([pair, "", "", "", "", "", "", ""])
        written.append(path)
    for path in written:
        print(f"wrote {path}")
    print(
        f"\n{len(pair_numbers)} pairs per sheet. "
        "Scores are 1-5 integers; most_realistic and preferred are A or B."
    )
    return 0


# --------------------------------------------------------------------------- stats


def wilson(successes: int, total: int, z: float = 1.959963985) -> tuple[float, float]:
    """95% Wilson score interval. Correct at small n, unlike the normal
    approximation, which is exactly the regime a five-listener pilot lives in."""
    if total == 0:
        return (0.0, 0.0)
    phat = successes / total
    denominator = 1.0 + z * z / total
    centre = phat + z * z / (2.0 * total)
    margin = z * math.sqrt(phat * (1.0 - phat) / total + z * z / (4.0 * total * total))
    return ((centre - margin) / denominator, (centre + margin) / denominator)


def two_sided_sign_test(successes: int, total: int) -> float:
    """Exact binomial p-value against p=0.5."""
    if total == 0:
        return 1.0

    def binomial_tail(k: int) -> float:
        return sum(math.comb(total, i) for i in range(0, k + 1)) / (2.0**total)

    extreme = min(successes, total - successes)
    return min(1.0, 2.0 * binomial_tail(extreme))


# --------------------------------------------------------------------------- report


def load_responses(pack: Path) -> tuple[list[dict], list[str]]:
    responses_dir = pack / "responses"
    if not responses_dir.is_dir():
        raise SystemExit(f"error: {responses_dir} not found; run 'sheets' first")
    rows: list[dict] = []
    problems: list[str] = []
    sheets = sorted(responses_dir.glob("*.csv"))
    if not sheets:
        raise SystemExit(f"error: no CSV in {responses_dir}")
    for sheet in sheets:
        listener = sheet.stem
        with sheet.open(encoding="utf-8-sig", newline="") as handle:
            for line_number, raw in enumerate(csv.DictReader(handle), start=2):
                if raw.get("pair") in (None, ""):
                    continue
                where = f"{sheet.name}:{line_number}"
                try:
                    pair = int(str(raw["pair"]).strip())
                except (TypeError, ValueError):
                    problems.append(f"{where}: pair '{raw.get('pair')}' is not an integer")
                    continue
                record: dict = {"listener": listener, "pair": pair, "where": where}
                incomplete = False
                for column in SCORE_COLUMNS:
                    value = (raw.get(column) or "").strip()
                    if value == "":
                        problems.append(f"{where}: {column} is empty")
                        incomplete = True
                        continue
                    try:
                        score = int(value)
                    except ValueError:
                        problems.append(f"{where}: {column}='{value}' is not an integer")
                        incomplete = True
                        continue
                    if not 1 <= score <= 5:
                        problems.append(f"{where}: {column}={score} outside 1-5")
                        incomplete = True
                        continue
                    record[column] = score
                for column in CHOICE_COLUMNS:
                    value = (raw.get(column) or "").strip().upper()
                    if value not in {"A", "B"}:
                        problems.append(f"{where}: {column}='{value}' must be A or B")
                        incomplete = True
                        continue
                    record[column] = value
                record["comment"] = (raw.get("comment") or "").strip()
                if not incomplete:
                    rows.append(record)
    return rows, problems


def report(pack: Path, allow_incomplete: bool) -> int:
    key_path = pack / "listening-key.json"
    if not key_path.is_file():
        print(f"error: {key_path} not found", file=sys.stderr)
        return 1
    with key_path.open(encoding="utf-8") as handle:
        key = json.load(handle)
    by_pair = {entry["pair"]: entry for entry in key["pairs"]}

    rows, problems = load_responses(pack)
    if problems and not allow_incomplete:
        print("Refusing to score an incomplete or out-of-domain set:\n", file=sys.stderr)
        for problem in problems[:40]:
            print(f"  {problem}", file=sys.stderr)
        if len(problems) > 40:
            print(f"  ... and {len(problems) - 40} more", file=sys.stderr)
        print(
            "\nFix the sheets, or pass --allow-incomplete to score only the "
            "complete rows (the report then states how many were dropped).",
            file=sys.stderr,
        )
        return 2

    listeners = sorted({row["listener"] for row in rows})
    per_pair: dict[int, dict] = {}
    unknown_pairs = set()

    for row in rows:
        pair = row["pair"]
        entry = by_pair.get(pair)
        if entry is None:
            unknown_pairs.add(pair)
            continue
        el_side = entry["enginelab_side"]
        ref_side = "A" if el_side == "B" else "B"
        bucket = per_pair.setdefault(
            pair,
            {
                "engine": entry["engine"],
                "match_quality": entry["reference"].get("match_quality", "unknown"),
                "n": 0,
                "realism_el": [],
                "realism_ref": [],
                "pref_el": [],
                "pref_ref": [],
                "chose_el_realistic": 0,
                "chose_el_preferred": 0,
                "comments": [],
            },
        )
        bucket["n"] += 1
        bucket["realism_el"].append(row[f"realism_{el_side}"])
        bucket["realism_ref"].append(row[f"realism_{ref_side}"])
        bucket["pref_el"].append(row[f"preference_{el_side}"])
        bucket["pref_ref"].append(row[f"preference_{ref_side}"])
        if row["most_realistic"] == el_side:
            bucket["chose_el_realistic"] += 1
        if row["preferred"] == el_side:
            bucket["chose_el_preferred"] += 1
        if row["comment"]:
            bucket["comments"].append(f"{row['listener']}: {row['comment']}")

    def mean(values: list[int]) -> float:
        return sum(values) / len(values) if values else float("nan")

    total_n = sum(bucket["n"] for bucket in per_pair.values())
    total_realistic = sum(bucket["chose_el_realistic"] for bucket in per_pair.values())
    total_preferred = sum(bucket["chose_el_preferred"] for bucket in per_pair.values())

    lines: list[str] = []
    lines.append("# Resultat d'ecoute A/B en aveugle - EngineLab contre enregistrements reels")
    lines.append("")
    lines.append(f"- dossier : `{pack.as_posix()}`")
    lines.append(f"- graine de tirage : `{key.get('seed')}`")
    lines.append(f"- sonie cible : {key.get('target_lufs')} LUFS (ITU-R BS.1770)")
    lines.append(f"- auditeurs : {len(listeners)} ({', '.join(listeners)})")
    lines.append(f"- jugements retenus : {total_n}")
    if problems:
        lines.append(f"- **lignes ecartees comme incompletes : {len(problems)}**")
    if unknown_pairs:
        lines.append(f"- **paires inconnues ignorees : {sorted(unknown_pairs)}**")
    lines.append("")
    lines.append(
        "Ce test mesure l'ecart a un enregistrement reel. Ce n'est pas un score "
        "absolu d'EngineLab : 7 references sur 10 sont des proxys, et aucune n'a "
        "de trajectoire de regime ni de position micro appariees. Sa valeur est "
        "le CLASSEMENT des familles, qui dit ou porter l'effort."
    )
    lines.append("")

    lines.append("## Global")
    lines.append("")
    lines.append("| Question | EngineLab choisi | Taux | IC 95 % (Wilson) | p (test des signes) |")
    lines.append("|---|---:|---:|---|---:|")
    for label, successes in (
        ("Le plus realiste", total_realistic),
        ("Prefere", total_preferred),
    ):
        low, high = wilson(successes, total_n)
        pvalue = two_sided_sign_test(successes, total_n)
        rate = successes / total_n if total_n else float("nan")
        lines.append(
            f"| {label} | {successes}/{total_n} | {rate:.1%} | "
            f"[{low:.1%}, {high:.1%}] | {pvalue:.3f} |"
        )
    lines.append("")

    lines.append("## Par famille, classe par ecart de realisme (le plus deficitaire d'abord)")
    lines.append("")
    lines.append(
        "| Paire | Moteur | Correspondance | n | Realisme EL | Realisme ref | Ecart | "
        "EL juge + realiste | IC 95 % |"
    )
    lines.append("|---:|---|---|---:|---:|---:|---:|---:|---|")
    ordered = sorted(
        per_pair.items(),
        key=lambda item: mean(item[1]["realism_el"]) - mean(item[1]["realism_ref"]),
    )
    for pair, bucket in ordered:
        el_mean = mean(bucket["realism_el"])
        ref_mean = mean(bucket["realism_ref"])
        low, high = wilson(bucket["chose_el_realistic"], bucket["n"])
        lines.append(
            f"| {pair} | {bucket['engine']} | {bucket['match_quality']} | {bucket['n']} | "
            f"{el_mean:.2f} | {ref_mean:.2f} | {el_mean - ref_mean:+.2f} | "
            f"{bucket['chose_el_realistic']}/{bucket['n']} | [{low:.0%}, {high:.0%}] |"
        )
    lines.append("")

    lines.append("## Commentaires libres")
    lines.append("")
    for pair, bucket in sorted(per_pair.items()):
        if not bucket["comments"]:
            continue
        lines.append(f"**Paire {pair} - {bucket['engine']}**")
        lines.append("")
        for comment in bucket["comments"]:
            lines.append(f"- {comment}")
        lines.append("")

    if problems:
        lines.append("## Lignes ecartees")
        lines.append("")
        for problem in problems:
            lines.append(f"- {problem}")
        lines.append("")

    report_md = "\n".join(lines) + "\n"
    md_path = pack / "listening-report.md"
    md_path.write_text(report_md, encoding="utf-8")

    payload = {
        "pack": pack.as_posix(),
        "seed": key.get("seed"),
        "target_lufs": key.get("target_lufs"),
        "listeners": listeners,
        "judgements": total_n,
        "dropped_rows": len(problems),
        "global": {
            "most_realistic": {
                "enginelab": total_realistic,
                "total": total_n,
                "wilson95": wilson(total_realistic, total_n),
                "sign_test_p": two_sided_sign_test(total_realistic, total_n),
            },
            "preferred": {
                "enginelab": total_preferred,
                "total": total_n,
                "wilson95": wilson(total_preferred, total_n),
                "sign_test_p": two_sided_sign_test(total_preferred, total_n),
            },
        },
        "per_pair": {
            str(pair): {
                "engine": bucket["engine"],
                "match_quality": bucket["match_quality"],
                "n": bucket["n"],
                "realism_enginelab_mean": mean(bucket["realism_el"]),
                "realism_reference_mean": mean(bucket["realism_ref"]),
                "preference_enginelab_mean": mean(bucket["pref_el"]),
                "preference_reference_mean": mean(bucket["pref_ref"]),
                "chose_enginelab_more_realistic": bucket["chose_el_realistic"],
                "chose_enginelab_preferred": bucket["chose_el_preferred"],
                "comments": bucket["comments"],
            }
            for pair, bucket in per_pair.items()
        },
    }
    json_path = pack / "listening-report.json"
    json_path.write_text(
        json.dumps(payload, indent=2, ensure_ascii=False), encoding="utf-8"
    )

    print(report_md)
    print(f"wrote {md_path}")
    print(f"wrote {json_path}")
    return 0


# --------------------------------------------------------------------------- cli


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    subparsers = parser.add_subparsers(dest="command", required=True)

    sheets = subparsers.add_parser("sheets", help="write blank response sheets")
    sheets.add_argument("--pack", required=True, type=Path)
    sheets.add_argument("--listeners", type=int, default=5)

    scoring = subparsers.add_parser("report", help="score the filled sheets")
    scoring.add_argument("--pack", required=True, type=Path)
    scoring.add_argument(
        "--allow-incomplete",
        action="store_true",
        help="score the complete rows and list what was dropped",
    )

    args = parser.parse_args()
    if args.command == "sheets":
        return make_sheets(args.pack, args.listeners)
    return report(args.pack, args.allow_incomplete)


if __name__ == "__main__":
    raise SystemExit(main())
