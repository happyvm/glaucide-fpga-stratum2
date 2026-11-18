// SPDX-License-Identifier: Apache-2.0
//
// Minimal replacement of <pcap/pcap.h> for running ntpperf against the
// cycle-accurate simulation of the design (tools/ntpperf/, task NTPPERF).
// This is NOT libpcap: it declares only the API ntpperf uses (plus
// pcap_inject/pcap_sendpacket/pcap_geterr/pcap_datalink), with the constant
// values of the real libpcap so that a program built against it behaves the
// same. The "interface" is a socket to the simulation bridge (simlink.h).
#ifndef SIM_PCAP_PCAP_H
#define SIM_PCAP_PCAP_H

#include <stddef.h>
#include <sys/time.h>
#include <sys/types.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef unsigned int bpf_u_int32;
typedef struct pcap pcap_t;

#define PCAP_ERRBUF_SIZE 256

// Status codes (values of libpcap).
#define PCAP_ERROR -1
#define PCAP_ERROR_ACTIVATED -4
#define PCAP_ERROR_NO_SUCH_DEVICE -5
#define PCAP_ERROR_NOT_ACTIVATED -3
#define PCAP_WARNING 1

// Time stamp types and precisions (values of libpcap).
#define PCAP_TSTAMP_HOST 0
#define PCAP_TSTAMP_HOST_LOWPREC 1
#define PCAP_TSTAMP_HOST_HIPREC 2
#define PCAP_TSTAMP_ADAPTER 3
#define PCAP_TSTAMP_ADAPTER_UNSYNCED 4
#define PCAP_TSTAMP_PRECISION_MICRO 0
#define PCAP_TSTAMP_PRECISION_NANO 1

// Link type.
#define DLT_EN10MB 1

typedef enum { PCAP_D_INOUT = 0, PCAP_D_IN, PCAP_D_OUT } pcap_direction_t;

// As in libpcap: with nanosecond precision, ts.tv_usec holds nanoseconds.
struct pcap_pkthdr {
    struct timeval ts;
    bpf_u_int32 caplen;
    bpf_u_int32 len;
};

pcap_t *pcap_create(const char *source, char *errbuf);
int pcap_set_snaplen(pcap_t *p, int snaplen);
int pcap_set_promisc(pcap_t *p, int promisc);
int pcap_set_timeout(pcap_t *p, int to_ms);
int pcap_set_immediate_mode(pcap_t *p, int immediate);
int pcap_set_buffer_size(pcap_t *p, int size);
int pcap_set_tstamp_type(pcap_t *p, int type);
int pcap_set_tstamp_precision(pcap_t *p, int precision);
int pcap_activate(pcap_t *p);
int pcap_set_datalink(pcap_t *p, int dlt);
int pcap_datalink(pcap_t *p);
int pcap_setdirection(pcap_t *p, pcap_direction_t d);
int pcap_setnonblock(pcap_t *p, int nonblock, char *errbuf);
int pcap_fileno(pcap_t *p);
int pcap_next_ex(pcap_t *p, struct pcap_pkthdr **h, const u_char **data);
int pcap_inject(pcap_t *p, const void *buf, size_t len);
int pcap_sendpacket(pcap_t *p, const u_char *buf, int len);
const char *pcap_statustostr(int status);
char *pcap_geterr(pcap_t *p);
void pcap_close(pcap_t *p);

#ifdef __cplusplus
}
#endif

#endif
