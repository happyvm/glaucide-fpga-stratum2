# SPDX-License-Identifier: Apache-2.0
"""Tests of tools/formal/contracts.py. Run: python3 -m unittest discover tools/formal"""
import contextlib
import io
import os
import shutil
import sys
import tempfile
import unittest

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)

import contracts  # noqa: E402


def run(sources, allow=None):
    """sources: {file name: Rust text}. Returns the Report."""
    items = [(name, text) for name, text in sources.items()]
    return contracts.analyze(items, allow or {})


UNIT_A = """
impl Unit for A {
    async fn run(&mut self) {
        loop {
            check!(x, "shared guarantee");
        }
    }
}
"""

UNIT_B_ASSUMES = """
#[lower]
impl Unit<Rx<T>, Tx<U>> for B {
    async fn run(&mut self) {
        loop {
            assume!(y, "shared guarantee");
        }
    }
}
"""


class Tokenizer(unittest.TestCase):
    def test_multiline_macro_with_nested_parens(self):
        src = """
impl Unit for A {
    async fn run(&mut self) {
        loop {
            check!(
                !(a & b) | Bit::from(x == U::<32>::from(3)),
                "multi-line, nested (parens) ok"
            );
        }
    }
}
"""
        (s,) = contracts.scan_source("t.rs", src)
        self.assertEqual(s.kind, "check")
        self.assertEqual(s.unit, "A")
        self.assertEqual(s.message, "multi-line, nested (parens) ok")
        self.assertIn("Bit :: from", s.condition)
        self.assertNotIn("multi-line", s.condition)
        self.assertEqual(s.line, 5)

    def test_strings_with_bang_parens_and_escapes(self):
        src = ('impl Unit for A { fn run() { '
               'assume!(x, "bang! (paren) \\"quoted\\" msg"); '
               'cover!(y, "a check!(z, \\"no\\") text"); } }')
        assume, cover = contracts.scan_source("t.rs", src)
        self.assertEqual(assume.message, 'bang! (paren) "quoted" msg')
        self.assertEqual(cover.message, 'a check!(z, "no") text')

    def test_comments_test_items_and_lifetimes_are_ignored_or_handled(self):
        src = """
// check!(x, "in a line comment");
/* assume!(x, "in a block /* nested */ comment"); */
impl<'a> Unit for B<'a> {
    async fn run(&mut self) {
        check!(c == '\\'', "char literal and lifetime ok");
        check!(r#"raw "x" ok"#.len() > 0, "raw string ok");
    }
}
#[cfg(test)]
mod tests {
    impl Unit for T {
        fn f() { check!(x, "only in tests"); }
    }
}
#[test]
fn t() { assume!(x, "only in a test fn"); }
"""
        stmts = contracts.scan_source("t.rs", src)
        self.assertEqual([(s.kind, s.unit, s.message) for s in stmts], [
            ("check", "B", "char literal and lifetime ok"),
            ("check", "B", "raw string ok"),
        ])

    def test_unit_name_of_impl_headers(self):
        src = """
impl<const N: usize, S: Clock> Default
    for TxPathG<N, S>
{
    fn default() -> Self { todo!() }
}
impl<const A: usize> Unit<(Rx<X>, In<U<48>>), Tx<Y>>
    for EthTx
{
    async fn run(&mut self) { check!(a, "in the multi-line impl"); }
}
"""
        (s,) = contracts.scan_source("t.rs", src)
        self.assertEqual(s.unit, "EthTx")

    def test_missing_message_is_reported(self):
        rep = run({"t.rs": "impl Unit for A { fn r() { check!(x); } }"})
        self.assertTrue(any("without a string message" in e for e in rep.errors))


class Rules(unittest.TestCase):
    def test_assume_guaranteed_by_other_unit_is_ok(self):
        rep = run({"a.rs": UNIT_A, "b.rs": UNIT_B_ASSUMES})
        self.assertEqual(rep.errors, [])
        self.assertEqual(len(rep.guaranteed), 1)
        self.assertIn("guaranteed by A at a.rs:5", rep.guaranteed[0])

    def test_orphan_assume_is_an_error(self):
        rep = run({"b.rs": UNIT_B_ASSUMES})
        self.assertEqual(len(rep.errors), 1)
        self.assertIn("orphan assume!", rep.errors[0])
        self.assertIn("shared guarantee", rep.errors[0])

    def test_same_unit_match_is_a_warning_and_not_a_guarantee(self):
        src = """
impl Unit for A {
    async fn run(&mut self) {
        check!(x, "self proved");
        assume!(x, "self proved");
    }
}
"""
        rep = run({"a.rs": src})
        self.assertTrue(any("self-guarantee" in w for w in rep.warnings))
        self.assertEqual(rep.guaranteed, [])
        self.assertEqual(len(rep.errors), 1)

    def test_allow_list_accepts_an_environment_assumption(self):
        rep = run({"b.rs": UNIT_B_ASSUMES}, {"shared guarantee": "config, guarantor T9"})
        self.assertEqual(rep.errors, [])
        self.assertEqual(len(rep.allowed), 1)
        self.assertIn("guarantor T9", rep.allowed[0])

    def test_unreviewed_entry_is_a_warning(self):
        rep = run({"b.rs": UNIT_B_ASSUMES}, {"shared guarantee": "UNREVIEWED"})
        self.assertEqual(rep.errors, [])
        self.assertTrue(any("not reviewed" in w for w in rep.warnings))

    def test_stale_and_redundant_allow_entries_are_warnings(self):
        rep = run({"a.rs": UNIT_A, "b.rs": UNIT_B_ASSUMES},
                  {"shared guarantee": "redundant", "gone": "stale"})
        self.assertEqual(rep.errors, [])
        self.assertTrue(any("not needed" in w for w in rep.warnings))
        self.assertTrue(any('matches no assume!: "gone"' in w for w in rep.warnings))

    def test_same_message_for_different_conditions_in_one_unit_warns(self):
        src = """
impl Unit for A {
    async fn run(&mut self) {
        check!(x, "same text");
        check!(y, "same text");
    }
}
"""
        rep = run({"a.rs": src})
        self.assertTrue(any("2 different conditions" in w for w in rep.warnings))
        self.assertEqual(rep.errors, [])

    def test_unused_checks_are_not_errors(self):
        rep = run({"a.rs": UNIT_A})
        self.assertEqual(rep.errors, [])
        self.assertEqual(rep.unused_checks, 1)

    def test_counts_per_unit(self):
        rep = run({"a.rs": UNIT_A, "b.rs": UNIT_B_ASSUMES})
        self.assertEqual(rep.units[("a.rs", "A")], {"check": 1, "assume": 0, "cover": 0})
        self.assertEqual(rep.units[("b.rs", "B")], {"check": 0, "assume": 1, "cover": 0})


class AllowList(unittest.TestCase):
    def test_parse(self):
        text = "# header\n\nmsg one (x) # reason one\n"
        self.assertEqual(contracts.parse_allowlist(text), {"msg one (x)": "reason one"})

    def test_entry_without_reason_is_refused(self):
        with self.assertRaises(ValueError):
            contracts.parse_allowlist("msg without reason\n")

    def test_duplicate_entry_is_refused(self):
        with self.assertRaises(ValueError):
            contracts.parse_allowlist("m # a\nm # b\n")


class CommandLine(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.mkdtemp()
        self.addCleanup(shutil.rmtree, self.tmp)
        os.makedirs(os.path.join(self.tmp, "rtl", "src"))
        with open(os.path.join(self.tmp, "rtl", "src", "b.rs"), "w", encoding="utf-8") as f:
            f.write(UNIT_B_ASSUMES)
        self.allow = os.path.join(self.tmp, "allow.txt")
        with open(self.allow, "w", encoding="utf-8") as f:
            f.write("# none\n")

    def test_orphan_gives_exit_1(self):
        out = io.StringIO()
        with contextlib.redirect_stdout(out):
            code = contracts.main(["--root", self.tmp, "--allow", self.allow])
        self.assertEqual(code, 1)
        self.assertIn("Result: FAIL", out.getvalue())

    def test_missing_allow_file_gives_exit_2(self):
        with contextlib.redirect_stderr(io.StringIO()):
            code = contracts.main(["--root", self.tmp, "--allow", self.allow + ".missing"])
        self.assertEqual(code, 2)


class RealTree(unittest.TestCase):
    def test_repository_contracts_hold(self):
        sources = contracts.collect_sources(contracts.REPO_ROOT)
        self.assertTrue(sources, "no rtl/src/*.rs found")
        allow = contracts.load_allowlist(contracts.ALLOW_FILE)
        rep = contracts.analyze(sources, allow)
        self.assertEqual(rep.errors, [])


if __name__ == "__main__":
    unittest.main()
