// SPDX-License-Identifier: Apache-2.0
//! Formal smoke test: proves the toolchain end to end. A decimal counter
//! (adapted from TxHDL's `ex_formal`) whose `check!` statements are proved
//! for every input by `tools/formal/prove.sh`.
use txhdl::comp::{mux, Clock, DefaultClock, In, Out, Reg, Unit};
use txhdl::types::{Bit, U};
use txhdl::{assume, check, cover, lower, with, Trace};

#[derive(Trace, Default)]
pub struct Digit {
    pub count: Reg<U<4>>,
}

#[lower]
impl Unit for Digit {
    async fn run(&mut self, step: In<U<2>>, (value, wrap): (Out<U<4>>, Out<Bit>)) {
        loop {
            DefaultClock::rising().await;
            let count = self.count.get();
            let step = step.get();
            assume!(step != 3, "the step is nought, one or two");
            check!(count <= 9, "the count is a digit");
            cover!(count == 9, "the count reaches nine");
            let sum = count + step.zext::<4>();
            let over = Bit::from(sum > 9);
            let next = mux(over, sum - 10, sum);
            if over.to_bool() {
                check!(next <= 1, "a wrap lands on nought or one");
            }
            with!(self <= {
                Bit::One ? count: next,
            });
            value.set(count);
            wrap.set(over);
        }
    }
}
