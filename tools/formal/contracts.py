#!/usr/bin/env python3
# SPDX-License-Identifier: Apache-2.0
"""Assume/guarantee contract checker (task F0.3, docs/formal.md).

Scans the Rust sources of rtl/src for the formal statements `assume!`,
`check!` and `cover!` and checks the contract rules of the section
"Méthode : garantie/hypothèse":

  a. every `assume!` message is either the message of a `check!` in ANOTHER
     unit (a guarantee proved upstream), or listed in
     tools/formal/environment_assumptions.txt (rule a(ii)). Otherwise: error.
  b. an `assume!` whose message is also a `check!` in the SAME unit is a
     warning (a self-guarantee proves nothing upstream).
  c. the same message used for different conditions in one unit is a warning.
  d. a `check!` that no unit assumes is not an error.

Usage: python3 tools/formal/contracts.py [--root DIR] [--allow FILE]
Exit status: 0 when there is no error (warnings do not fail), 1 on error,
2 when the allow-list or the sources cannot be read.
Standard library only.
"""
import argparse
import bisect
import glob
import os
import sys
from dataclasses import dataclass, field
from typing import Dict, List, Optional, Tuple

HERE = os.path.dirname(os.path.abspath(__file__))
REPO_ROOT = os.path.abspath(os.path.join(HERE, "..", ".."))
ALLOW_FILE = os.path.join(HERE, "environment_assumptions.txt")
MACROS = ("assume", "check", "cover")
UNIT_MODULE = "<module>"
MULTI_PUNCT = ("::", "->", "=>", "!=", "==", "&&", "||")
SIMPLE_ESCAPES = {"n": "\n", "t": "\t", "r": "\r", "0": "\0",
                  "\\": "\\", '"': '"', "'": "'"}


class ParseError(Exception):
    pass


@dataclass
class Tok:
    kind: str   # ident, num, punct, str, char, life
    text: str   # for str: the decoded value
    off: int    # offset in the source


@dataclass
class Statement:
    file: str
    unit: str
    line: int
    kind: str                 # assume, check or cover
    condition: str            # argument tokens before the message
    message: Optional[str]    # trimmed message, None when missing

    @property
    def unit_id(self) -> Tuple[str, str]:
        return (self.file, self.unit)


@dataclass
class Report:
    units: Dict[Tuple[str, str], Dict[str, int]] = field(default_factory=dict)
    guaranteed: List[str] = field(default_factory=list)
    allowed: List[str] = field(default_factory=list)
    errors: List[str] = field(default_factory=list)
    warnings: List[str] = field(default_factory=list)
    unused_checks: int = 0


# --- tokenizer ---------------------------------------------------------------

def _lex_string(src: str, i: int) -> Tuple[str, int]:
    """src[i] is the opening quote. Return (decoded text, index after it)."""
    out = []
    j = i + 1
    while True:
        c = src[j]
        if c == '"':
            return "".join(out), j + 1
        if c != "\\":
            out.append(c)
            j += 1
            continue
        nxt = src[j + 1]
        if nxt in "\r\n":                       # string continuation
            j += 1
            while src[j] in " \t\r\n":
                j += 1
            continue
        if nxt == "x":
            out.append(chr(int(src[j + 2:j + 4], 16)))
            j += 4
            continue
        if nxt == "u":                          # \u{XXXX}
            end = src.index("}", j)
            out.append(chr(int(src[j + 3:end].replace("_", ""), 16)))
            j = end + 1
            continue
        out.append(SIMPLE_ESCAPES.get(nxt, nxt))
        j += 2


def tokenize(src: str) -> List[Tok]:
    """Rust tokens, without comments and whitespace. Strings are decoded."""
    toks: List[Tok] = []
    n = len(src)
    i = 0
    while i < n:
        c = src[i]
        if c.isspace():
            i += 1
            continue
        if src.startswith("//", i):
            end = src.find("\n", i)
            i = n if end < 0 else end
            continue
        if src.startswith("/*", i):
            depth = 0
            while i < n:
                if src.startswith("/*", i):
                    depth += 1
                    i += 2
                elif src.startswith("*/", i):
                    depth -= 1
                    i += 2
                    if depth == 0:
                        break
                else:
                    i += 1
            continue
        start = i
        if c.isalpha() or c == "_":
            j = i
            while j < n and (src[j].isalnum() or src[j] == "_"):
                j += 1
            word = src[i:j]
            if word in ("r", "br") and j < n and src[j] in '#"':
                k = j
                while k < n and src[k] == "#":
                    k += 1
                if k < n and src[k] == '"':            # raw string r#"..."#
                    closing = '"' + "#" * (k - j)
                    end = src.index(closing, k + 1)
                    toks.append(Tok("str", src[k + 1:end], start))
                    i = end + len(closing)
                    continue
            if word == "b" and j < n and src[j] == '"':
                text, i = _lex_string(src, j)
                toks.append(Tok("str", text, start))
                continue
            toks.append(Tok("ident", word, start))
            i = j
            continue
        if c.isdigit():
            j = i
            while j < n and (src[j].isalnum() or src[j] == "_"):
                j += 1
            toks.append(Tok("num", src[i:j], start))
            i = j
            continue
        if c == '"':
            text, i = _lex_string(src, i)
            toks.append(Tok("str", text, start))
            continue
        if c == "'":
            if i + 1 < n and src[i + 1] == "\\":       # '\n', '\'', '\u{..}'
                end = src.index("'", i + 3)
                toks.append(Tok("char", src[i:end + 1], start))
                i = end + 1
                continue
            if i + 2 < n and src[i + 2] == "'":        # 'x'
                toks.append(Tok("char", src[i:i + 3], start))
                i += 3
                continue
            j = i + 1                                   # lifetime 'a
            while j < n and (src[j].isalnum() or src[j] == "_"):
                j += 1
            toks.append(Tok("life", src[i:j], start))
            i = j
            continue
        if src[i:i + 2] in MULTI_PUNCT:
            toks.append(Tok("punct", src[i:i + 2], start))
            i += 2
            continue
        toks.append(Tok("punct", c, start))
        i += 1
    return toks


# --- structure ---------------------------------------------------------------

def match_brackets(toks: List[Tok]) -> Dict[int, int]:
    """Index of the matching bracket for every (, [, { and its closer."""
    match: Dict[int, int] = {}
    stack: List[int] = []
    for idx, t in enumerate(toks):
        if t.kind != "punct":
            continue
        if t.text in ("(", "[", "{"):
            stack.append(idx)
        elif t.text in (")", "]", "}") and stack:
            opener = stack.pop()
            match[opener] = idx
            match[idx] = opener
    return match


def _starts_item(toks: List[Tok], k: int) -> bool:
    if k == 0:
        return True
    p = toks[k - 1]
    if p.kind == "punct":
        return p.text in ("}", ";", "{", "]")   # ']' ends an attribute
    return p.kind == "ident" and p.text in ("pub", "unsafe")


def test_ranges(toks: List[Tok], match: Dict[int, int]) -> List[Tuple[int, int]]:
    """Token ranges of items marked #[test] or #[cfg(test)]."""
    ranges: List[Tuple[int, int]] = []
    pending: Optional[int] = None
    i = 0
    while i < len(toks):
        t = toks[i]
        if (t.kind == "punct" and t.text == "#" and i + 1 < len(toks)
                and toks[i + 1].text == "[" and i + 1 in match):
            close = match[i + 1]
            words = [x.text for x in toks[i + 2:close]]
            if words in (["test"], ["cfg", "(", "test", ")"]) and pending is None:
                pending = i
            i = close + 1
            continue
        if pending is not None and t.kind == "punct" and t.text in ("{", ";"):
            end = match.get(i, i) if t.text == "{" else i
            ranges.append((pending, end))
            pending = None
            i = end + 1
            continue
        i += 1
    return ranges


def _unit_name(header: List[Tok]) -> str:
    """Type name of `impl [<..>] [Trait<..> for] Type<..>`."""
    for_at = [h for h, t in enumerate(header) if t.kind == "ident" and t.text == "for"]
    if for_at:
        names: List[str] = []
        for t in header[for_at[-1] + 1:]:
            if t.kind == "ident":
                names.append(t.text)
            elif t.text == "::" or (t.kind == "punct" and t.text == "&") or t.kind == "life":
                continue
            else:
                break
        return names[-1] if names else "<unknown>"
    j = 0
    if j < len(header) and header[j].text == "<":
        depth = 0
        while j < len(header):
            if header[j].text == "<":
                depth += 1
            elif header[j].text == ">":
                depth -= 1
                if depth == 0:
                    j += 1
                    break
            j += 1
    for t in header[j:]:
        if t.kind == "ident":
            return t.text
    return "<unknown>"


def impl_ranges(toks: List[Tok], match: Dict[int, int]) -> List[Tuple[int, int, str]]:
    """(index of '{', index of '}', unit name) for every impl block."""
    res = []
    for k, t in enumerate(toks):
        if not (t.kind == "ident" and t.text == "impl" and _starts_item(toks, k)):
            continue
        b = k + 1
        while b < len(toks) and not (toks[b].kind == "punct" and toks[b].text in ("{", ";")):
            b += 1
        if b >= len(toks) or toks[b].text == ";":
            continue
        res.append((b, match.get(b, len(toks) - 1), _unit_name(toks[k + 1:b])))
    return res


def split_args(toks: List[Tok]) -> List[List[Tok]]:
    """Split a macro argument list at its top-level commas."""
    args: List[List[Tok]] = [[]]
    depth = 0
    for t in toks:
        if t.kind == "punct":
            if t.text in ("(", "[", "{"):
                depth += 1
            elif t.text in (")", "]", "}"):
                depth -= 1
            elif t.text == "," and depth == 0:
                args.append([])
                continue
        args[-1].append(t)
    if args and not args[-1]:
        args.pop()
    return args


def scan_source(rel: str, src: str) -> List[Statement]:
    """Every assume!/check!/cover! outside test items, with its unit."""
    try:
        toks = tokenize(src)
    except (IndexError, ValueError) as exc:
        raise ParseError(f"{rel}: cannot tokenize ({exc})")
    line_starts = [0] + [i + 1 for i, c in enumerate(src) if c == "\n"]
    match = match_brackets(toks)
    skip = test_ranges(toks, match)
    impls = impl_ranges(toks, match)

    def unit_at(k: int) -> str:
        best = None
        for b, e, name in impls:
            if b < k < e and (best is None or b > best[0]):
                best = (b, name)
        return best[1] if best else UNIT_MODULE

    stmts: List[Statement] = []
    for k in range(len(toks) - 2):
        t = toks[k]
        if t.kind != "ident" or t.text not in MACROS:
            continue
        bang, op = toks[k + 1], toks[k + 2]
        if bang.kind != "punct" or bang.text != "!":
            continue
        if op.kind != "punct" or op.text not in ("(", "[", "{"):
            continue
        if any(a <= k <= b for a, b in skip):
            continue
        line = bisect.bisect_right(line_starts, t.off)
        close = match.get(k + 2)
        if close is None:
            raise ParseError(f"{rel}:{line}: unbalanced {t.text}!")
        args = split_args(toks[k + 3:close])
        condition = ", ".join(" ".join(x.text for x in a) for a in args[:-1])
        message = None
        if args and len(args[-1]) == 1 and args[-1][0].kind == "str":
            message = args[-1][0].text.strip()
        stmts.append(Statement(rel, unit_at(k), line, t.text, condition, message))
    return stmts


# --- allow-list ----------------------------------------------------------------

def parse_allowlist(text: str, name: str = "allow-list") -> Dict[str, str]:
    """Lines `<message> # <reason / guarantor>`; '#' alone starts a comment."""
    allow: Dict[str, str] = {}
    for lineno, raw in enumerate(text.splitlines(), 1):
        line = raw.strip()
        if not line or line.startswith("#"):
            continue
        msg, sep, reason = line.partition(" # ")
        msg, reason = msg.strip(), reason.strip()
        if not sep or not reason:
            raise ValueError(f"{name}:{lineno}: entry needs a '# reason / guarantor' comment")
        if msg in allow:
            raise ValueError(f"{name}:{lineno}: duplicate entry \"{msg}\"")
        allow[msg] = reason
    return allow


def load_allowlist(path: str) -> Dict[str, str]:
    with open(path, encoding="utf-8") as f:
        return parse_allowlist(f.read(), path)


# --- rules ---------------------------------------------------------------------

def analyze(sources: List[Tuple[str, str]], allow: Dict[str, str]) -> Report:
    rep = Report()
    stmts: List[Statement] = []
    for rel, src in sources:
        try:
            stmts.extend(scan_source(rel, src))
        except ParseError as exc:
            rep.errors.append(str(exc))

    for s in stmts:
        counts = rep.units.setdefault(s.unit_id, {"check": 0, "assume": 0, "cover": 0})
        counts[s.kind] += 1

    checks: Dict[str, List[Statement]] = {}
    for s in stmts:
        if s.kind == "check" and s.message is not None:
            checks.setdefault(s.message, []).append(s)

    assumed = set()
    for s in stmts:
        where = f"{s.file}:{s.line}"
        if s.message is None:
            rep.errors.append(f"{where}: {s.kind}! without a string message")
            continue
        if s.kind != "assume":
            continue
        m = s.message
        assumed.add(m)
        same = [c for c in checks.get(m, []) if c.unit_id == s.unit_id]
        other = [c for c in checks.get(m, []) if c.unit_id != s.unit_id]
        if same:
            rep.warnings.append(f"{where}: assume! \"{m}\" is also a check! in the same "
                                f"unit {s.unit} (self-guarantee, not a proof upstream)")
        if other:
            c = other[0]
            rep.guaranteed.append(f"{where} ({s.unit}): \"{m}\" guaranteed by "
                                  f"{c.unit} at {c.file}:{c.line}")
            if m in allow:
                rep.warnings.append(f"{where}: allow-list entry not needed, guaranteed "
                                    f"by {c.unit}: \"{m}\"")
        elif m in allow:
            rep.allowed.append(f"{where} ({s.unit}): \"{m}\" -- {allow[m]}")
            if "UNREVIEWED" in allow[m]:
                rep.warnings.append(f"{where}: environment assumption not reviewed "
                                    f"by a human: \"{m}\"")
        else:
            rep.errors.append(f"{where} ({s.unit}): orphan assume! \"{m}\": no check! of "
                              f"this text in another unit and not in the allow-list")

    for m in allow:
        if m not in assumed:
            rep.warnings.append(f"allow-list entry matches no assume!: \"{m}\"")

    by_unit_msg: Dict[Tuple[Tuple[str, str], str], List[Statement]] = {}
    for s in stmts:
        if s.message is not None:
            by_unit_msg.setdefault((s.unit_id, s.message), []).append(s)
    for (unit_id, m), group in by_unit_msg.items():
        conds = {s.condition for s in group}
        if len(conds) > 1:
            lines = ", ".join(str(s.line) for s in group)
            rep.warnings.append(f"{unit_id[0]}: unit {unit_id[1]} uses the message \"{m}\" "
                                f"for {len(conds)} different conditions (lines {lines})")

    rep.unused_checks = sum(len(lst) for m, lst in checks.items() if m not in assumed)
    return rep


# --- command line --------------------------------------------------------------

def is_generated(text: str) -> bool:
    return any("@generated" in line for line in text.splitlines()[:5])


def collect_sources(root: str) -> List[Tuple[str, str]]:
    out = []
    for path in sorted(glob.glob(os.path.join(root, "rtl", "src", "*.rs"))):
        base = os.path.basename(path)
        if base.endswith("_test.rs") or base == "tests.rs":
            continue
        with open(path, encoding="utf-8") as f:
            text = f.read()
        if is_generated(text):
            continue
        out.append((os.path.relpath(path, root).replace(os.sep, "/"), text))
    return out


def print_report(rep: Report) -> None:
    print("Units (statements per unit)")
    print(f"  {'file':<24} {'unit':<16} {'check':>5} {'assume':>6} {'cover':>5}")
    tot = {"check": 0, "assume": 0, "cover": 0}
    for (file, unit), c in rep.units.items():
        print(f"  {file:<24} {unit:<16} {c['check']:>5} {c['assume']:>6} {c['cover']:>5}")
        for k in tot:
            tot[k] += c[k]
    print(f"  {'total':<24} {'':<16} {tot['check']:>5} {tot['assume']:>6} {tot['cover']:>5}")
    print()
    print("Assumptions")
    for line in rep.guaranteed:
        print(f"  OK     {line}")
    for line in rep.allowed:
        print(f"  OK     {line}")
    for line in rep.warnings:
        print(f"  WARN   {line}")
    for line in rep.errors:
        print(f"  ERROR  {line}")
    print()
    print(f"checks not assumed by any unit (not an error): {rep.unused_checks}")
    verdict = "FAIL" if rep.errors else "PASS"
    print(f"Result: {verdict} ({len(rep.errors)} error(s), {len(rep.warnings)} warning(s))")


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description="assume/guarantee contract checker (F0.3)")
    ap.add_argument("--root", default=REPO_ROOT, help="repository root (contains rtl/src)")
    ap.add_argument("--allow", default=ALLOW_FILE, help="environment assumptions file")
    args = ap.parse_args(argv)
    try:
        allow = load_allowlist(args.allow)
        sources = collect_sources(args.root)
    except (OSError, ValueError) as exc:
        print(f"contracts: {exc}", file=sys.stderr)
        return 2
    rep = analyze(sources, allow)
    print_report(rep)
    return 1 if rep.errors else 0


if __name__ == "__main__":
    sys.exit(main())
