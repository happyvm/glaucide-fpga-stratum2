// SPDX-License-Identifier: Apache-2.0
//! Writes the Verilog of the board build (task T1.11) to `out/top/`:
//! `arty_top.v` (the generated module tree of `ArtyBoard`, one module per
//! unit plus the registered channel module `arty_top_txhdl_chan`), a copy
//! of the hand-written board wrapper `arty_top_board.v` (from `rtl/hdl/`),
//! and `files.f`, the list of sources in compile order for Vivado
//! (`tools/vivado/build.tcl`). Run from anywhere in the workspace:
//! `cargo run -p ntp_fpga --bin top_gen` (an optional argument overrides
//! the output directory).
//!
//! The module tree goes through [`ntp_fpga::top::fix_rst_clash`] (a
//! local `rst` wire of two units; see its documentation).
//!
//! `--with-client` writes the Phase B build (`ArtyBoardWithClient`, the
//! client-side discipline wired, `docs/top-client-hooks.md`) instead.
use ntp_fpga::top::{fix_rst_clash, ArtyBoard, ArtyBoardWithClient};
use std::path::PathBuf;

fn main() {
    let root = PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("..");
    let args: Vec<String> = std::env::args().skip(1).collect();
    let with_client = args.iter().any(|a| a == "--with-client");
    let out = args
        .iter()
        .find(|a| !a.starts_with("--"))
        .map(PathBuf::from)
        .unwrap_or_else(|| root.join("out").join("top"));
    std::fs::create_dir_all(&out).expect("create the output directory");
    let v = if with_client {
        fix_rst_clash(&ArtyBoardWithClient::verilog("arty_top"))
    } else {
        fix_rst_clash(&ArtyBoard::verilog("arty_top"))
    };
    std::fs::write(out.join("arty_top.v"), &v).expect("write arty_top.v");
    let wrapper = std::fs::read_to_string(root.join("rtl/hdl/arty_top_board.v"))
        .expect("read rtl/hdl/arty_top_board.v");
    std::fs::write(out.join("arty_top_board.v"), wrapper).expect("write arty_top_board.v");
    let files = "arty_top.v\narty_top_board.v\n";
    std::fs::write(out.join("files.f"), files).expect("write files.f");
    let modules = v.lines().filter(|l| l.starts_with("module ")).count();
    println!(
        "{}: arty_top.v ({} modules, {} lines), arty_top_board.v, files.f",
        out.display(),
        modules,
        v.lines().count()
    );
}
