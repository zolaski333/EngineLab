#!/usr/bin/env python3
"""Response sheets and scoring for blind A/B listening tests.

Two modes:

  python scripts/listening.py sheets  --pack <folder> --listeners 5
  python scripts/listening.py report  --pack <folder>

`sheets` writes blank CSVs into <folder>/responses/. It never reads
`listening-key.json`, so the folder handed to listeners stays blind.

`report` reads the key and the filled CSVs back, then writes a markdown report
and a JSON file. It refuses an incomplete or out-of-range sheet rather than
guessing: a missing answer is a missing answer.

Standard library only.
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
    # The condition is pre-filled so a listener knows whether they are scoring an
    # idle or a rev-up. It reveals nothing: the engine name and the A/B assignment
    # both stay in the key. `report` never reads this column.
    conditions = {entry["pair"]: entry.get("segment", "") for entry in key["pairs"]}
    # A pair with no control side has one file on disk and cannot be scored A vs B,
    # so it gets no row at all. Giving it one would force the listener to either
    # invent a grade for silence or leave the row blank, and a blank row makes
    # `report` refuse the whole sheet.
    scoreable = [
        entry for entry in key["pairs"] if entry.get("control_present", True)
    ]
    skipped = len(key["pairs"]) - len(scoreable)
    pair_numbers = sorted(entry["pair"] for entry in scoreable)
    if not pair_numbers:
        print(
            "error: no pair in this pack has two sides, so there is nothing to "
            "score blind. Render with a reference manifest, or with --compare.",
            file=sys.stderr,
        )
        return 1

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
            writer.writerow(["pair", "condition", *COLUMNS[1:]])
            for pair in pair_numbers:
                writer.writerow([pair, conditions.get(pair, ""), "", "", "", "", "", "", ""])
        written.append(path)
    for path in written:
        print(f"wrote {path}")
    print(
        f"\n{len(pair_numbers)} pairs per sheet. "
        "Scores are 1-5 integers; most_realistic and preferred are A or B."
    )
    if skipped:
        print(
            f"{skipped} clip(s) have no second side and are NOT on the sheet; "
            "they can be auditioned (solo_*.wav) but not scored A/B."
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
    # Two comparisons share this machinery because the statistics are identical --
    # a blind binary choice, Wilson, sign test. Only the LABELS differ, and getting
    # them wrong would report "EngineLab beat a real recording" about a run where
    # no recording was involved.
    variant_mode = key.get("mode") == "variant"
    baseline = key.get("baseline_engine", "-")
    candidate = key.get("candidate_engine", "-")
    subject_label = "Candidate" if variant_mode else "EngineLab"
    control_label = "Baseline" if variant_mode else "Reference"

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
    # A pair whose reference slot is empty has only ONE side on disk, so a listener
    # scoring "B" scored nothing. Those judgements are not comparisons and are
    # excluded from every statistic rather than averaged in -- the report lists
    # them separately so the gap in the corpus stays visible.
    solo_pairs: dict[int, dict] = {}

    for row in rows:
        pair = row["pair"]
        entry = by_pair.get(pair)
        if entry is None:
            unknown_pairs.add(pair)
            continue
        if not entry.get("control_present", True):
            solo = solo_pairs.setdefault(
                pair,
                {
                    "engine": entry["engine"],
                    "segment": entry.get("segment", "-"),
                    "reason": entry.get("control_error", "no control side"),
                    "n": 0,
                },
            )
            solo["n"] += 1
            continue
        el_side = entry["subject_side"]
        ref_side = "A" if el_side == "B" else "B"
        # `control` is absent whenever the pair has no usable other side, which is
        # the normal state for a segment the corpus does not cover. Reading it
        # unconditionally raised KeyError and took the whole report down.
        control = entry.get("control") or {}
        bucket = per_pair.setdefault(
            pair,
            {
                "engine": entry["engine"],
                "segment": entry.get("segment", "-"),
                "match_quality": control.get("match_quality", "no-control"),
                "window_matched": control.get("window_condition_matched", None),
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
    if variant_mode:
        lines.append(f"# Blind A/B listening - {candidate} versus {baseline}")
    else:
        lines.append(
            "# Blind A/B listening result - EngineLab versus real recordings"
        )
    lines.append("")
    lines.append(f"- pack: `{pack.as_posix()}`")
    lines.append(f"- shuffle seed: `{key.get('seed')}`")
    lines.append(f"- target loudness: {key.get('target_lufs')} LUFS (ITU-R BS.1770)")
    lines.append(f"- listeners: {len(listeners)} ({', '.join(listeners)})")
    lines.append(f"- judgements kept: {total_n}")
    if solo_pairs:
        dropped = sum(bucket["n"] for bucket in solo_pairs.values())
        lines.append(
            f"- **judgements discarded for lack of a reference: {dropped} "
            f"over {len(solo_pairs)} pair(s)**"
        )
    if problems:
        lines.append(f"- **rows discarded as incomplete: {len(problems)}**")
    if unknown_pairs:
        lines.append(f"- **unknown pairs ignored: {sorted(unknown_pairs)}**")
    lines.append("")

    # Counted from the key rather than asserted: a hardcoded "7 references out of
    # 10 are proxies" goes stale the moment the corpus changes, and a stale
    # caveat is worse than none because it is read as current.
    qualities: dict[str, int] = {}
    for entry in key["pairs"]:
        control = entry.get("control") or {}
        if entry.get("control_present", False):
            label = control.get("match_quality", "unknown")
            qualities[label] = qualities.get(label, 0) + 1
    paired = sum(qualities.values())
    proxies = sum(count for label, count in qualities.items() if "proxy" in label)
    if variant_mode:
        lines.append(
            f"This test compares two catalogue engines: **{candidate}** (the"
            f" candidate) versus **{baseline}** (the internal reference). It says"
            " nothing about absolute realism -- no real recording takes part."
            " It only says which of the two a listener prefers, blind and"
            " at equal loudness."
        )
    else:
        lines.append(
            "This test measures the distance to a real recording. It is not an absolute "
            f"EngineLab score: of {paired} matched pair(s), {proxies} are matched "
            "against a proxy, and no reference has a matched engine-speed trajectory "
            "or microphone position. Its value is the RANKING of the "
            "families, which says where to put the effort."
        )
    if qualities:
        detail = ", ".join(
            f"{label} x{count}" for label, count in sorted(qualities.items())
        )
        lines.append("")
        lines.append(f"Match qualities present: {detail}.")
    lines.append("")

    lines.append("## Global")
    lines.append("")
    lines.append(
        f"| Question | {subject_label} chosen | Rate | 95 % CI (Wilson) | "
        "p (sign test) |"
    )
    lines.append("|---|---:|---:|---|---:|")
    for label, successes in (
        ("Most realistic", total_realistic),
        ("Preferred", total_preferred),
    ):
        low, high = wilson(successes, total_n)
        pvalue = two_sided_sign_test(successes, total_n)
        rate = successes / total_n if total_n else float("nan")
        lines.append(
            f"| {label} | {successes}/{total_n} | {rate:.1%} | "
            f"[{low:.1%}, {high:.1%}] | {pvalue:.3f} |"
        )
    lines.append("")

    lines.append("## Per family, ranked by realism gap (largest deficit first)")
    lines.append("")
    lines.append(
        f"| Pair | Engine | Condition | Match | n | Realism "
        f"{subject_label} | Realism {control_label} | Gap | {subject_label} judged "
        "more realistic | 95 % CI |"
    )
    lines.append("|---:|---|---|---|---:|---:|---:|---:|---:|---|")
    ordered = sorted(
        per_pair.items(),
        key=lambda item: mean(item[1]["realism_el"]) - mean(item[1]["realism_ref"]),
    )
    for pair, bucket in ordered:
        el_mean = mean(bucket["realism_el"])
        ref_mean = mean(bucket["realism_ref"])
        low, high = wilson(bucket["chose_el_realistic"], bucket["n"])
        match = bucket["match_quality"]
        if bucket["window_matched"] is False:
            match += " (window not matched)"
        lines.append(
            f"| {pair} | {bucket['engine']} | {bucket['segment']} | {match} | "
            f"{bucket['n']} | {el_mean:.2f} | {ref_mean:.2f} | {el_mean - ref_mean:+.2f} | "
            f"{bucket['chose_el_realistic']}/{bucket['n']} | [{low:.0%}, {high:.0%}] |"
        )
    lines.append("")

    unmatched = sorted(
        pair for pair, bucket in per_pair.items() if bucket["window_matched"] is False
    )
    if unmatched:
        lines.append(
            "> **Warning.** Pairs "
            + ", ".join(str(pair) for pair in unmatched)
            + " compare a segment with a reference window whose condition is not"
            " matched: the simulated idle may be set against a recording"
            " under load. Their verdict is worthless and must not"
            " be published."
        )
        lines.append("")

    if solo_pairs:
        lines.append("## Pairs without a reference - not comparable, excluded from scores")
        lines.append("")
        lines.append(
            "These pairs have only one side on disk. A listener who scored"
            " the other side scored silence, so their judgements are discarded."
            " They stay listed because it is the corpus that is missing, not"
            " the listener."
        )
        lines.append("")
        lines.append("| Pair | Engine | Condition | Discarded judgements | Reason |")
        lines.append("|---:|---|---|---:|---|")
        for pair, bucket in sorted(solo_pairs.items()):
            lines.append(
                f"| {pair} | {bucket['engine']} | {bucket['segment']} | "
                f"{bucket['n']} | {bucket['reason']} |"
            )
        lines.append("")

    lines.append("## Free comments")
    lines.append("")
    for pair, bucket in sorted(per_pair.items()):
        if not bucket["comments"]:
            continue
        lines.append(f"**Pair {pair} - {bucket['engine']}**")
        lines.append("")
        for comment in bucket["comments"]:
            lines.append(f"- {comment}")
        lines.append("")

    if problems:
        lines.append("## Discarded rows")
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
        "match_quality_counts": qualities,
        "unpaired_segments": {
            str(pair): {
                "engine": bucket["engine"],
                "segment": bucket["segment"],
                "discarded_judgements": bucket["n"],
                "reason": bucket["reason"],
            }
            for pair, bucket in solo_pairs.items()
        },
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
                "segment": bucket["segment"],
                "match_quality": bucket["match_quality"],
                "window_condition_matched": bucket["window_matched"],
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
