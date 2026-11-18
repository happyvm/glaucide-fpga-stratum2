"""Deterministic hex vector generator.  Usage (from tools/):  python3 -m golden.vectors [outdir]

Writes vectors/*.hex and vectors/README.md. Same code + same seed = same bytes.
"""
import os
import random
import sys
from typing import Dict, List, Tuple

from . import arp, eth, icmp, icmpv6, ipv4, ipv6, ntp, udp
from .responder import Config, respond

SEED = 20261117
HERE = os.path.dirname(os.path.abspath(__file__))
DEFAULT_OUT = os.path.join(HERE, "vectors")

SRV_MAC = eth.mac("02:00:00:00:00:01")
CLI_MAC = eth.mac("02:00:00:00:00:02")
FOREIGN_MAC = eth.mac("02:00:00:00:00:99")
SRV_IP4 = bytes([192, 0, 2, 1])
CLI_IP4 = bytes([192, 0, 2, 2])
SRV_LL = ipv6.addr("fe80::ff:fe00:1")      # EUI-64 of SRV_MAC
SRV_GUA = ipv6.addr("2001:db8::1")
CLI_LL = ipv6.addr("fe80::ff:fe00:2")
CLI_GUA = ipv6.addr("2001:db8::2")

# 2026-11-17 12:00:00 UTC (reference), T2 = +0.25 s, T3 = T2 + 20 us
T_REF = ntp.unix_to_ts(1763380800)
T_RECV = ntp.unix_to_ts(ntp.Fraction(1763380800) + ntp.Fraction(1, 4))
T_XMIT = ntp.seconds_to_ts(ntp.ts_to_fraction(T_RECV) + ntp.Fraction(20, 1000000))

CONFIG = Config(
    mac=SRV_MAC, ipv4=SRV_IP4, ipv6_ll=SRV_LL, ipv6_global=SRV_GUA,
    stratum=2, ref_id=ntp.refid_ipv4("192.0.2.99"), ref_id_v6=ntp.refid_ipv6("2001:db8::99"),
    precision=-24, root_delay=ntp.seconds_to_short(0.0125), root_dispersion=ntp.seconds_to_short(0.025),
    ref_ts=T_REF, recv_ts=T_RECV, xmit_ts=T_XMIT, synchronised=True)

Step = Tuple[bytes, List[bytes]]  # one received frame and the frames expected out


def _cfg(**kw) -> Config:
    import dataclasses
    return dataclasses.replace(CONFIG, **kw)


# ---------------------------------------------------------------- frame builders (rx side)

def _wire(frame: bytes) -> bytes:
    """What the MAC delivers for a received frame: padded to 60 bytes, FCS already removed."""
    return eth.pad_to_min(frame)


def rx_arp_request(tpa=SRV_IP4, src_mac=CLI_MAC):
    p = arp.ArpPacket(arp.OP_REQUEST, src_mac, CLI_IP4, bytes(6), tpa).build()
    return _wire(eth.EthFrame(eth.BROADCAST, src_mac, eth.ETHERTYPE_ARP, p).build())


def rx_ping4(rng, ident, seq, n, dst_mac=SRV_MAC, **ipkw):
    data = bytes(rng.randrange(256) for _ in range(n))
    icm = icmp.IcmpEcho(icmp.TYPE_ECHO_REQUEST, ident, seq, data).build()
    ip = ipv4.IPv4Packet(CLI_IP4, SRV_IP4, ipv4.PROTO_ICMP, icm, ident=rng.randrange(65536), **ipkw).build()
    return _wire(eth.EthFrame(dst_mac, CLI_MAC, eth.ETHERTYPE_IPV4, ip).build())


def ntp_request(rng, vn=4, mode=3, poll=6):
    return ntp.NtpPacket(vn=vn, mode=mode, poll=poll, xmit_ts=rng.getrandbits(64)).build()


def rx_ntp4(payload, dport=123, dst_mac=SRV_MAC, **ipkw):
    u = udp.UdpDatagram(40000, dport, payload).build(CLI_IP4, SRV_IP4)
    ip = ipv4.IPv4Packet(CLI_IP4, SRV_IP4, ipv4.PROTO_UDP, u, **ipkw).build()
    return _wire(eth.EthFrame(dst_mac, CLI_MAC, eth.ETHERTYPE_IPV4, ip).build())


def rx_ntp6(payload, dst=SRV_GUA, src=CLI_GUA):
    u = udp.UdpDatagram(40000, 123, payload).build(src, dst)
    ip = ipv6.IPv6Packet(src, dst, ipv6.NH_UDP, u).build()
    return _wire(eth.EthFrame(SRV_MAC, CLI_MAC, eth.ETHERTYPE_IPV6, ip).build())


def rx_ping6(rng, ident, seq, n, dst=SRV_GUA):
    data = bytes(rng.randrange(256) for _ in range(n))
    body = icmpv6.Icmpv6Echo(icmpv6.T_ECHO_REQUEST, ident, seq, data).build(CLI_GUA, dst)
    ip = ipv6.IPv6Packet(CLI_GUA, dst, ipv6.NH_ICMPV6, body).build()
    return _wire(eth.EthFrame(SRV_MAC, CLI_MAC, eth.ETHERTYPE_IPV6, ip).build())


def rx_ns(target=SRV_LL, hop_limit=255, via_multicast=True):
    dst = ipv6.solicited_node(target) if via_multicast else target
    body = icmpv6.NeighborSolicitation(target, [(icmpv6.OPT_SLLA, CLI_MAC)]).build(CLI_LL, dst)
    ip = ipv6.IPv6Packet(CLI_LL, dst, ipv6.NH_ICMPV6, body, hop_limit=hop_limit).build()
    dmac = ipv6.multicast_mac(dst) if via_multicast else SRV_MAC
    return _wire(eth.EthFrame(dmac, CLI_MAC, eth.ETHERTYPE_IPV6, ip).build())


def _flip_byte(frame: bytes, offset: int) -> bytes:
    b = bytearray(frame)
    b[offset] ^= 0xFF
    return bytes(b)


# ---------------------------------------------------------------- scenarios

def scenarios() -> Dict[str, Tuple[str, Config, List[bytes]]]:
    """name -> (description, config, list of rx frames). tx frames come from the reference."""
    rng = random.Random(SEED)
    s: Dict[str, Tuple[str, Config, List[bytes]]] = {}
    ok4 = ntp_request(rng)
    ok6 = ntp_request(rng)

    s["arp_request_reply"] = ("ARP who-has 192.0.2.1 -> reply (RFC 826); a request for another address gets nothing",
                              CONFIG, [rx_arp_request(), rx_arp_request(tpa=bytes([192, 0, 2, 77]))])
    s["ping4"] = ("ICMP echo request to 192.0.2.1 -> echo reply (RFC 792), two sizes",
                  CONFIG, [rx_ping4(rng, 0x1234, 1, 32), rx_ping4(rng, 0x1234, 2, 100)])
    s["ping6"] = ("ICMPv6 echo request to 2001:db8::1 -> echo reply (RFC 4443)",
                  CONFIG, [rx_ping6(rng, 0x4321, 1, 32), rx_ping6(rng, 0x4321, 2, 56)])
    s["ndp_ns_na"] = ("NS for fe80::ff:fe00:1 via solicited-node multicast, and unicast NS for the global address -> NA S=1 O=1 (RFC 4861)",
                      CONFIG, [rx_ns(), rx_ns(target=SRV_GUA, via_multicast=False)])
    s["ntp4_basic"] = ("NTPv4 client request over IPv4 -> mode 4 reply, synchronised stratum 2",
                       CONFIG, [rx_ntp4(ok4)])
    s["ntp6_basic"] = ("NTPv4 client request over IPv6 (global address) -> mode 4 reply",
                       CONFIG, [rx_ntp6(ok6), rx_ntp6(ntp_request(rng), dst=SRV_LL, src=CLI_LL)])
    s["ntp_unsynchronised"] = ("Not synchronised: LI=3, stratum 16", _cfg(synchronised=False),
                               [rx_ntp4(ntp_request(rng)), rx_ntp6(ntp_request(rng))])
    s["ntp_version3"] = ("NTPv3 client request -> reply with version 3", CONFIG,
                         [rx_ntp4(ntp_request(rng, vn=3)), rx_ntp6(ntp_request(rng, vn=3))])

    # Negative scenarios: a valid twin first (proves the base frame is answered), then the
    # mutated frame, which must yield nothing.
    def neg(name, desc, good, bad):
        s[name] = (desc + " (first frame valid and answered, second frame dropped)", CONFIG, [good, bad])

    g4 = rx_ntp4(ok4)
    neg("neg_ipv4_checksum", "IPv4 header checksum corrupted", g4, _flip_byte(g4, 14 + 10))
    g6 = rx_ntp6(ok6)
    neg("neg_udp6_checksum", "UDP checksum corrupted over IPv6", g6, _flip_byte(g6, 14 + 40 + 6))
    neg("neg_udp_port", "UDP destination port 9999 instead of 123", g4, rx_ntp4(ok4, dport=9999))
    neg("neg_ipv4_fragment", "IPv4 first fragment (MF=1)", g4, rx_ntp4(ok4, df=False, mf=True))
    neg("neg_ipv4_ttl0", "IPv4 TTL 0", g4, rx_ntp4(ok4, ttl=0))
    gn = rx_ns()
    neg("neg_ndp_hoplimit", "Neighbor Solicitation with hop limit 64 instead of 255", gn, rx_ns(hop_limit=64))
    neg("neg_ntp_mode4", "NTP packet in mode 4 (server) sent to us", g4, rx_ntp4(ntp_request(rng, mode=4)))
    neg("neg_foreign_mac", "Unicast frame addressed to another MAC", g4, rx_ntp4(ok4, dst_mac=FOREIGN_MAC))
    neg("neg_runt", "Runt frame of 13 bytes (shorter than an Ethernet header)", g4, g4[:13])
    return s


def render(name: str, desc: str, cfg: Config, rx_frames: List[bytes]) -> str:
    lines = ["# " + name + ": " + desc]
    for fr in rx_frames:
        lines.append("rx " + fr.hex())
        for out in respond(fr, cfg):
            lines.append("tx " + out.hex())
    return "\n".join(lines) + "\n"


README = """# Vecteurs de test (format)

Fichiers `*.hex` générés par `python3 -m golden.vectors` (depuis `tools/`), ne pas éditer à la main.
Régénération identique octet par octet (graine fixe %(seed)d), vérifiée par les tests.

## Format

Une ligne par trame : `<direction> <octets en hexadécimal>` (minuscules, sans espaces ni `0x`).

- `rx` : trame reçue par l'équipement ;
- `tx` : trame que l'équipement doit émettre en réponse à la dernière trame `rx` qui précède.

Les lignes vides et les lignes commençant par `#` sont ignorées. Aucune ligne `tx` après un `rx` :
l'équipement ne doit rien émettre. Les trames se jouent dans l'ordre du fichier.

Convention de trame (`rtl/src/stream.rs`) : trames Ethernet **sans FCS**, de l'adresse MAC
destination à la fin de la charge utile.

- `rx` : complétées par des zéros jusqu'à 60 octets (ce que le MAC livre après retrait du FCS),
  sauf le scénario `neg_runt` (13 octets) ;
- `tx` : sortie du constructeur Ethernet, **sans bourrage** (`eth_tx` complète à 60 et ajoute le FCS).

## Configuration de l'équipement pour tous les scénarios (sauf mention)

| Champ | Valeur |
|---|---|
| MAC locale | `02:00:00:00:00:01` |
| IPv4 | `192.0.2.1` |
| IPv6 link-local | `fe80::ff:fe00:1` |
| IPv6 globale | `2001:db8::1` |
| Stratum / synchronisé | 2 / oui (`ntp_unsynchronised` : non, donc LI=3 et stratum 16) |
| Reference ID IPv4 | `c0000263` (192.0.2.99) |
| Reference ID IPv6 | `%(rid6)s` (MD5 de 2001:db8::99, 4 premiers octets) |
| Precision | -24 |
| Root delay / dispersion (16.16) | `%(rd)08x` / `%(rdisp)08x` |
| Reference timestamp | `%(rts)016x` |
| Receive timestamp (T2) | `%(t2)016x` |
| Transmit timestamp (T3) | `%(t3)016x` |

Client : MAC `02:00:00:00:00:02`, `192.0.2.2`, `2001:db8::2`, `fe80::ff:fe00:2`, port source UDP 40000.

## Scénarios

%(table)s
Les scénarios `neg_*` commencent par la trame valide correspondante (avec sa réponse `tx`) puis la
trame altérée, qui ne doit provoquer aucune émission.
"""


def write_all(outdir: str = DEFAULT_OUT) -> List[str]:
    os.makedirs(outdir, exist_ok=True)
    sc = scenarios()
    for stale in os.listdir(outdir):
        if stale.endswith(".hex") and stale[:-4] not in sc:
            os.remove(os.path.join(outdir, stale))
    for name, (desc, cfg, rx) in sc.items():
        with open(os.path.join(outdir, name + ".hex"), "w", newline="\n") as fh:
            fh.write(render(name, desc, cfg, rx))
    table = "| Fichier | Contenu |\n|---|---|\n" + "".join(
        "| `%s.hex` | %s |\n" % (n, d) for n, (d, _c, _r) in sc.items())
    with open(os.path.join(outdir, "README.md"), "w", newline="\n") as fh:
        fh.write(README % dict(seed=SEED, rid6=CONFIG.ref_id_v6.hex(), rd=CONFIG.root_delay,
                               rdisp=CONFIG.root_dispersion, rts=T_REF, t2=T_RECV, t3=T_XMIT, table=table))
    return sorted(sc)


if __name__ == "__main__":
    names = write_all(sys.argv[1] if len(sys.argv) > 1 else DEFAULT_OUT)
    print("wrote %d vector files" % len(names))
