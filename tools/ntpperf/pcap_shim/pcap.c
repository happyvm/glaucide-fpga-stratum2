// SPDX-License-Identifier: Apache-2.0
//
// pcap.c: minimal replacement libpcap for ntpperf (task NTPPERF).
//
// The capture "device" is the simulation bridge: the descriptor named by the
// environment variable NTPPERF_SIM_FD (one end of an AF_UNIX SOCK_SEQPACKET
// socketpair created by rtl/src/bin/ntpperf_bridge.rs, inherited across
// exec). Frames written to it (pcap_inject, or sendmmsg on pcap_fileno() as
// ntpperf's sender does) are put on the simulated MII receive pins; frames
// the simulated design transmits come back as messages carrying a VIRTUAL
// timestamp chosen by the bridge (simlink.h). The interface name given to
// pcap_create() is not checked (any name works: the time shim answers the
// SIOCGIFHWADDR query for it).
//
// Only what ntpperf needs is implemented; the option setters only record
// their value (snaplen, the time stamp type and precision are honoured).
#define _GNU_SOURCE
#include <errno.h>
#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/socket.h>
#include <unistd.h>

#include "pcap/pcap.h"
#include "simlink.h"

_Static_assert(sizeof(struct sim_hello) == 12, "hello must be shorter than a frame");

#define MAX_FRAME 2048

struct pcap {
    int fd;
    int activated;
    int snaplen;
    int tstamp_type;
    int precision;
    int nonblock;
    char errbuf[PCAP_ERRBUF_SIZE];
    struct pcap_pkthdr hdr;
    unsigned char buf[sizeof(struct sim_pkt_hdr) + MAX_FRAME];
};

pcap_t *pcap_create(const char *source, char *errbuf) {
    (void)source;
    pcap_t *p = calloc(1, sizeof *p);
    if (!p) {
        snprintf(errbuf, PCAP_ERRBUF_SIZE, "out of memory");
        return NULL;
    }
    p->fd = -1;
    p->snaplen = 262144;
    p->tstamp_type = PCAP_TSTAMP_HOST;
    p->precision = PCAP_TSTAMP_PRECISION_MICRO;
    return p;
}

#define SETTER(name, field)                         \
    int name(pcap_t *p, int v) {                    \
        if (p->activated)                           \
            return PCAP_ERROR_ACTIVATED;            \
        p->field = v;                               \
        return 0;                                   \
    }
SETTER(pcap_set_snaplen, snaplen)
SETTER(pcap_set_tstamp_type, tstamp_type)
SETTER(pcap_set_tstamp_precision, precision)

int pcap_set_promisc(pcap_t *p, int v) {
    (void)v;
    return p->activated ? PCAP_ERROR_ACTIVATED : 0;
}
int pcap_set_timeout(pcap_t *p, int v) {
    (void)v;
    return p->activated ? PCAP_ERROR_ACTIVATED : 0;
}
int pcap_set_immediate_mode(pcap_t *p, int v) {
    (void)v;
    return p->activated ? PCAP_ERROR_ACTIVATED : 0;
}
int pcap_set_buffer_size(pcap_t *p, int v) {
    (void)v;
    return p->activated ? PCAP_ERROR_ACTIVATED : 0;
}

int pcap_activate(pcap_t *p) {
    if (p->activated)
        return PCAP_ERROR_ACTIVATED;
    const char *s = getenv("NTPPERF_SIM_FD");
    if (!s || !*s) {
        snprintf(p->errbuf, sizeof p->errbuf, "NTPPERF_SIM_FD not set (run under ntpperf_bridge)");
        return PCAP_ERROR_NO_SUCH_DEVICE;
    }
    int fd = atoi(s);
    if (fcntl(fd, F_GETFD) < 0) {
        snprintf(p->errbuf, sizeof p->errbuf, "NTPPERF_SIM_FD %d: %s", fd, strerror(errno));
        return PCAP_ERROR_NO_SUCH_DEVICE;
    }
    struct sim_hello h;
    memcpy(h.tag, "HELO", 4);
    h.tstamp_type = (uint16_t)p->tstamp_type;
    h.precision = (uint16_t)p->precision;
    h.snaplen = (uint32_t)p->snaplen;
    if (send(fd, &h, sizeof h, 0) != (ssize_t)sizeof h) {
        snprintf(p->errbuf, sizeof p->errbuf, "hello to the bridge: %s", strerror(errno));
        return PCAP_ERROR;
    }
    p->fd = fd;
    p->activated = 1;
    return 0;
}

int pcap_set_datalink(pcap_t *p, int dlt) {
    (void)p;
    return dlt == DLT_EN10MB ? 0 : PCAP_ERROR;
}

int pcap_datalink(pcap_t *p) {
    return p->activated ? DLT_EN10MB : PCAP_ERROR_NOT_ACTIVATED;
}

// The bridge only ever sends what the design transmits: direction "in".
int pcap_setdirection(pcap_t *p, pcap_direction_t d) {
    (void)p;
    return d == PCAP_D_OUT ? PCAP_ERROR : 0;
}

int pcap_setnonblock(pcap_t *p, int nonblock, char *errbuf) {
    (void)errbuf;
    p->nonblock = nonblock;
    return 0;
}

int pcap_fileno(pcap_t *p) { return p->fd; }

// Returns 1 with a frame, 0 when none is pending (non-blocking mode, or a
// closed bridge: ntpperf loops while the result is non-zero, so errors are
// never reported as negative values here).
int pcap_next_ex(pcap_t *p, struct pcap_pkthdr **h, const u_char **data) {
    if (!p->activated)
        return PCAP_ERROR_NOT_ACTIVATED;
    for (;;) {
        ssize_t r = recv(p->fd, p->buf, sizeof p->buf, p->nonblock ? MSG_DONTWAIT : 0);
        if (r < 0 && errno == EINTR)
            continue;
        if (r < (ssize_t)sizeof(struct sim_pkt_hdr))
            return 0;
        struct sim_pkt_hdr ph;
        memcpy(&ph, p->buf, sizeof ph);
        uint32_t len = (uint32_t)r - (uint32_t)sizeof ph;
        if (ph.len < len)
            len = ph.len;
        p->hdr.len = ph.len;
        p->hdr.caplen = len < (uint32_t)p->snaplen ? len : (uint32_t)p->snaplen;
        p->hdr.ts.tv_sec = (time_t)(ph.ts_ns / 1000000000ull);
        uint64_t frac = ph.ts_ns % 1000000000ull;
        p->hdr.ts.tv_usec =
            (suseconds_t)(p->precision == PCAP_TSTAMP_PRECISION_NANO ? frac : frac / 1000);
        *h = &p->hdr;
        *data = p->buf + sizeof ph;
        return 1;
    }
}

int pcap_inject(pcap_t *p, const void *buf, size_t len) {
    if (!p->activated)
        return PCAP_ERROR_NOT_ACTIVATED;
    if (len < 14 || len > MAX_FRAME) {
        snprintf(p->errbuf, sizeof p->errbuf, "bad frame length %zu", len);
        return PCAP_ERROR;
    }
    ssize_t r = send(p->fd, buf, len, 0);
    if (r < 0) {
        snprintf(p->errbuf, sizeof p->errbuf, "send: %s", strerror(errno));
        return PCAP_ERROR;
    }
    return (int)r;
}

int pcap_sendpacket(pcap_t *p, const u_char *buf, int len) {
    return pcap_inject(p, buf, (size_t)len) == len ? 0 : PCAP_ERROR;
}

const char *pcap_statustostr(int status) {
    static char b[64];
    switch (status) {
    case 0:
        return "Success";
    case PCAP_ERROR:
        return "Generic error";
    case PCAP_ERROR_ACTIVATED:
        return "The setting can't be changed after the pcap_t is activated";
    case PCAP_ERROR_NOT_ACTIVATED:
        return "The pcap_t has not been activated";
    case PCAP_ERROR_NO_SUCH_DEVICE:
        return "No simulation bridge (NTPPERF_SIM_FD)";
    default:
        snprintf(b, sizeof b, "status %d", status);
        return b;
    }
}

char *pcap_geterr(pcap_t *p) { return p->errbuf; }

void pcap_close(pcap_t *p) {
    if (!p)
        return;
    // The descriptor belongs to the bridge connection; ntpperf's sender
    // child holds a copy. Closing it here is what libpcap would do.
    if (p->fd >= 0)
        close(p->fd);
    free(p);
}
