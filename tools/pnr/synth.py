import sys,re
from pyosys import libyosys as ys
src=open(sys.argv[1]).read()
src=re.sub(r"^\s*initial for \((?:mem_i|rl_i).*$","",src,flags=re.M)
open("/tmp/pnr_t.v","w").write(src)
d=ys.Design()
ys.run_pass("read_verilog /tmp/pnr_t.v",d)
ys.run_pass("synth_xilinx -family xc7 -top arty_top -flatten -nodsp",d)
ys.run_pass("techmap -map tools/pnr/r256.v",d)
ys.run_pass("opt_clean",d)
ys.run_pass("write_json "+sys.argv[2],d)
