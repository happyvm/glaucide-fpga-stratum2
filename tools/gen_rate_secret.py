#!/usr/bin/env python3
"""Pick or check a RATE_SECRET for the NTP server's rate limiter (docs/security.md F1, ntpperf N1).

The limiter hashes the source address with a multiply-shift hash whose multiplier derives from the
secret. A multiply-shift hash is universal only ON AVERAGE over the multiplier: for some secrets,
consecutive client addresses fall into very few buckets and share rate limits. `config::Checks`
rejects such secrets at compile time (`ntp_server::rl_secret_ok`); this script is the same test in
Python so you can choose a value:

    python3 tools/gen_rate_secret.py            # print a random secret that passes
    python3 tools/gen_rate_secret.py 0x1234...  # check a given one
"""
import secrets, sys

M = (1 << 64) - 1
W = 0x9E3779B97F4A7C15
BUCKETS = 4096
BASE = 0xC0000200  # 192.0.2.0: the test only needs a representative block
# (stride in addresses, number of keys, minimum distinct buckets = ~90 % of the random-function expectation)
CASES = [(1, 4096, 2330), (2, 4096, 2330), (256, 2048, 1450), (65536, 2048, 1450)]


def splitmix64(x):
    z = (x + W) & M
    z = ((z ^ (z >> 30)) * 0xBF58476D1CE4E5B9) & M
    z = ((z ^ (z >> 27)) * 0x94D049BB133111EB) & M
    return z ^ (z >> 31)


def index(key, secret):
    m = splitmix64(secret) | 1
    add = (((m << 32) | (m >> 32)) & M) ^ W
    return ((m * key + add) & M) >> 52


def spreads(secret):
    return [len({index(BASE + i * s, secret) for i in range(n)}) for s, n, _ in CASES]


def ok(secret):
    return all(v >= lo for v, (_, _, lo) in zip(spreads(secret), CASES))


if __name__ == "__main__":
    if len(sys.argv) > 1:
        s = int(sys.argv[1], 0)
        print("spreads (stride 1, 2, 256, 65536):", spreads(s), "->", "OK" if ok(s) else "REJECTED")
        sys.exit(0 if ok(s) else 1)
    while True:
        s = secrets.randbits(64)
        if ok(s):
            print(hex(s))
            break
