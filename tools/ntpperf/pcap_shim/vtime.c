// SPDX-License-Identifier: Apache-2.0
//
// vtime.c: LD_PRELOAD library that runs ntpperf (both of its processes: the
// measuring parent and the forked sender) in the VIRTUAL time of the
// cycle-accurate simulation driven by rtl/src/bin/ntpperf_bridge.rs.
// Same idea as tools/chrony/vtime_shim.c (lockstep: the bridge owns time,
// the client computes in zero virtual time), adapted to a program that never
// blocks: ntpperf busy-spins on clock_gettime(CLOCK_MONOTONIC).
//
// Model (docs/ntpperf-results.md, section 2):
//  * Every process of ntpperf has a slot in the shared page (simlink.h). The
//    bridge advances virtual time only when EVERY slot is ready:
//      - ST_TIME_WAIT with a deadline later than the current virtual time,
//      - ST_BLOCKED_WRITE (writing a full pipe: only the reader can unblock
//        it, and the reader needs time to advance first),
//      - ST_BLOCKED_READ on a pipe with no byte in flight,
//      - ST_EXITED;
//    and it advances to the EARLIEST deadline, simulating the design up to
//    that instant (the frames ntpperf sent before are on the wire by then).
//  * A spin is a clock read that cannot change anything: the shim turns it
//    into a wait for a virtual deadline, then returns the new time:
//      - the sender process: its deadline is known EXACTLY: it is the `when`
//        of the next request it has not sent yet. The shim reads the
//        requests (struct sender_request of ntpperf's sender.h, 40 bytes:
//        a struct timespec of CLOCK_MONOTONIC first) as the sender reads
//        them from its pipe, and drops them as sendmmsg() reports them
//        sent. A clock read with a request already due returns at once.
//      - any other process (the measuring parent): a clock read is a spin
//        when the process did nothing productive since its previous clock
//        read (no frame received, no request written to the pipe). It then
//        waits for vt + quantum (SW_QUANTUM_NS, 1 ms by default): the
//        parent schedules requests 25 ms ahead and reads the received frames
//        with their own (virtual) timestamps, so a 1 ms reaction time
//        changes no measured value.
//  * Pipes created by the process (pipe/pipe2, inherited by fork) are
//    tracked: bytes written count as "in flight" until read, so the bridge
//    never advances between a write and the wake-up of a blocked reader.
//  * fork() allocates the child's slot in the parent (state RUNNING) before
//    forking, so time cannot run while the child starts.
//  * The interface MAC: ioctl(SIOCGIFHWADDR) on the interface named by
//    NTPPERF_IFACE (default "sim0") returns NTPPERF_MAC (default
//    02:00:00:00:00:02): ntpperf uses it as the source MAC of its requests.
//  * Clocks: CLOCK_REALTIME/_COARSE/TAI = virtual Unix time; every other
//    clock (MONOTONIC, BOOTTIME, CPU time) = virtual time - T0 + 1 s;
//    gettimeofday() and time() follow. Without NTPPERF_VTIME_SHM in the
//    environment every call is passed through unchanged.
//
// Every state change is followed by a bump of SW_SEQ (a futex the bridge
// sleeps on); the bridge validates a snapshot of all slots by reading SW_SEQ
// before and after it (docs/ntpperf-results.md explains why this is enough).
//
// Build: gcc -O2 -shared -fPIC -o ntpperf_vtime.so vtime.c -ldl
#define _GNU_SOURCE
#include <dlfcn.h>
#include <errno.h>
#include <fcntl.h>
#include <limits.h>
#include <linux/futex.h>
#include <net/if.h>
#include <poll.h>
#include <signal.h>
#include <stdarg.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <sys/socket.h>
#include <sys/syscall.h>
#include <sys/time.h>
#include <time.h>
#include <unistd.h>

#include "simlink.h"

static volatile uint64_t *shm;
static int my_slot = -1;
static int productive = 1;

static ssize_t (*real_read)(int, void *, size_t);
static ssize_t (*real_write)(int, const void *, size_t);
static int (*real_pipe)(int[2]);
static int (*real_pipe2)(int[2], int);
static pid_t (*real_fork)(void);
static int (*real_sendmmsg)(int, struct mmsghdr *, unsigned int, int);
static ssize_t (*real_recv)(int, void *, size_t, int);
static int (*real_ioctl)(int, unsigned long, ...);

// Tracked pipe descriptors.
#define MAX_FD 1024
static unsigned char tracked[MAX_FD];

// Pending request deadlines of the sender (virtual Unix ns), FIFO.
#define QCAP 65536
static uint64_t q[QCAP];
static unsigned qhead, qlen;

// ntpperf's struct sender_request (sender.h), 40 bytes.
struct sender_request {
    struct timespec when;
    uint32_t src_address;
    uint32_t _pad;
    uint64_t remote_id;
    uint64_t local_id;
};
_Static_assert(sizeof(struct sender_request) == 40, "sender_request layout");

static uint64_t ld(int w) { return __atomic_load_n(&shm[w], __ATOMIC_SEQ_CST); }
static void st(int w, uint64_t v) { __atomic_store_n(&shm[w], v, __ATOMIC_SEQ_CST); }
static uint64_t *wp(int w) { return (uint64_t *)&shm[w]; }
static int sw(int k) { return SW_SLOTS + my_slot * SW_SLOT_WORDS + k; }

static long futex(uint64_t *word, int op, uint32_t val, const struct timespec *to) {
    // The futex is the low 32 bits of a 64-bit word (x86_64: little endian).
    return syscall(SYS_futex, (uint32_t *)word, op, val, to, NULL, 0);
}

static void bump_seq(void) {
    __atomic_fetch_add(wp(SW_SEQ), 1, __ATOMIC_SEQ_CST);
    futex(wp(SW_SEQ), FUTEX_WAKE, INT_MAX, NULL);
}

static void set_state(uint64_t s) {
    st(sw(SL_STATE), s);
    bump_seq();
}

__attribute__((constructor)) static void vt_init(void) {
    real_read = dlsym(RTLD_NEXT, "read");
    real_write = dlsym(RTLD_NEXT, "write");
    real_pipe = dlsym(RTLD_NEXT, "pipe");
    real_pipe2 = dlsym(RTLD_NEXT, "pipe2");
    real_fork = dlsym(RTLD_NEXT, "fork");
    real_sendmmsg = dlsym(RTLD_NEXT, "sendmmsg");
    real_recv = dlsym(RTLD_NEXT, "recv");
    real_ioctl = dlsym(RTLD_NEXT, "ioctl");
    const char *p = getenv("NTPPERF_VTIME_SHM");
    if (!p || !*p)
        return;
    int fd = open(p, O_RDWR | O_CLOEXEC);
    if (fd < 0)
        return;
    void *m = mmap(NULL, 4096, PROT_READ | PROT_WRITE, MAP_SHARED, fd, 0);
    close(fd);
    if (m == MAP_FAILED)
        return;
    volatile uint64_t *s = m;
    if (__atomic_load_n(&s[SW_MAGIC], __ATOMIC_ACQUIRE) != SIM_MAGIC)
        return;
    shm = s;
}

static int alloc_slot(void) {
    uint64_t i = __atomic_fetch_add(wp(SW_NSLOTS), 1, __ATOMIC_SEQ_CST);
    if (i >= SW_MAX_SLOTS) {
        fprintf(stderr, "ntpperf_vtime: too many processes\n");
        _exit(70);
    }
    return (int)i;
}

// Registers the calling process lazily (first intercepted call), so that a
// program that only passes through (a shell, env) leaves no slot behind.
static void ensure_slot(void) {
    if (my_slot >= 0)
        return;
    int s = alloc_slot();
    my_slot = s;
    st(sw(SL_PID), (uint64_t)getpid());
    set_state(ST_RUNNING);
}

__attribute__((destructor)) static void vt_fini(void) {
    if (shm && my_slot >= 0)
        set_state(ST_EXITED);
}

static void check_bridge(void) {
    if (ld(SW_STOP)) {
        fprintf(stderr, "ntpperf_vtime: the bridge stopped; exiting\n");
        _exit(75);
    }
    pid_t b = (pid_t)ld(SW_BRIDGE_PID);
    if (b > 0 && kill(b, 0) != 0 && errno == ESRCH) {
        fprintf(stderr, "ntpperf_vtime: the bridge is gone; exiting\n");
        _exit(75);
    }
}

// Waits until the virtual time reaches `deadline`.
static void wait_until(uint64_t deadline) {
    if (ld(SW_VT_NS) >= deadline)
        return;
    st(sw(SL_DEADLINE), deadline);
    st(sw(SL_WAITS), ld(sw(SL_WAITS)) + 1);
    set_state(ST_TIME_WAIT);
    for (;;) {
        uint32_t e = (uint32_t)ld(SW_EPOCH);
        if (ld(SW_VT_NS) >= deadline)
            break;
        struct timespec to = {0, 200000000};
        if (futex(wp(SW_EPOCH), FUTEX_WAIT, e, &to) < 0 && errno == ETIMEDOUT)
            check_bridge();
    }
    set_state(ST_RUNNING);
}

static uint64_t mono_to_vt(const struct timespec *ts) {
    uint64_t m = (uint64_t)ts->tv_sec * 1000000000ull + (uint64_t)ts->tv_nsec;
    return m - 1000000000ull + ld(SW_T0_NS);
}

// Called on every clock read of a registered process: turns spins into waits.
static void maybe_wait(void) {
    ensure_slot();
    if (qlen > 0) {
        // The sender: wait for its next request (returns at once if due).
        wait_until(q[qhead]);
    } else if (!productive) {
        uint64_t qn = ld(SW_QUANTUM_NS);
        wait_until(ld(SW_VT_NS) + (qn ? qn : 1000000ull));
    }
    productive = 0;
}

static void ns_to_ts(uint64_t ns, struct timespec *ts) {
    ts->tv_sec = (time_t)(ns / 1000000000ull);
    ts->tv_nsec = (long)(ns % 1000000000ull);
}

static void virt_clock(clockid_t id, struct timespec *ts) {
    uint64_t v = ld(SW_VT_NS);
    switch (id) {
    case CLOCK_REALTIME:
    case CLOCK_REALTIME_COARSE:
#ifdef CLOCK_TAI
    case CLOCK_TAI:
#endif
        ns_to_ts(v, ts);
        break;
    default:
        ns_to_ts(v - ld(SW_T0_NS) + 1000000000ull, ts);
        break;
    }
}

// ---------------------------------------------------------------- clocks

int clock_gettime(clockid_t id, struct timespec *ts) {
    if (!shm)
        return (int)syscall(SYS_clock_gettime, id, ts);
    maybe_wait();
    virt_clock(id, ts);
    return 0;
}

int gettimeofday(struct timeval *tv, void *tz) {
    (void)tz;
    if (!shm)
        return (int)syscall(SYS_gettimeofday, tv, tz);
    struct timespec ts;
    virt_clock(CLOCK_REALTIME, &ts);
    tv->tv_sec = ts.tv_sec;
    tv->tv_usec = ts.tv_nsec / 1000;
    return 0;
}

time_t time(time_t *t) {
    struct timespec ts;
    if (!shm)
        syscall(SYS_clock_gettime, CLOCK_REALTIME, &ts);
    else
        virt_clock(CLOCK_REALTIME, &ts);
    if (t)
        *t = ts.tv_sec;
    return ts.tv_sec;
}

// ---------------------------------------------------------------- pipes

static void track(int fds[2]) {
    for (int i = 0; i < 2; i++)
        if (fds[i] >= 0 && fds[i] < MAX_FD)
            tracked[fds[i]] = 1;
}

int pipe(int fds[2]) {
    int r = real_pipe(fds);
    if (r == 0 && shm)
        track(fds);
    return r;
}

int pipe2(int fds[2], int flags) {
    int r = real_pipe2(fds, flags);
    if (r == 0 && shm)
        track(fds);
    return r;
}

int close(int fd) {
    if (fd >= 0 && fd < MAX_FD)
        tracked[fd] = 0;
    return (int)syscall(SYS_close, fd);
}

static int is_tracked(int fd) { return shm && fd >= 0 && fd < MAX_FD && tracked[fd]; }

ssize_t write(int fd, const void *buf, size_t n) {
    if (!is_tracked(fd))
        return real_write(fd, buf, n);
    ensure_slot();
    // Bytes in flight from now on: a blocked reader is not "ready" any more.
    __atomic_fetch_add(wp(SW_INFLIGHT), (uint64_t)n, __ATOMIC_SEQ_CST);
    bump_seq();
    struct pollfd pf = {fd, POLLOUT, 0};
    int blocked = poll(&pf, 1, 0) != 1;
    if (blocked) {
        st(sw(SL_IOWAITS), ld(sw(SL_IOWAITS)) + 1);
        set_state(ST_BLOCKED_WRITE);
    }
    ssize_t r = real_write(fd, buf, n);
    if (blocked)
        set_state(ST_RUNNING);
    size_t done = r > 0 ? (size_t)r : 0;
    if (done < n) {
        __atomic_fetch_sub(wp(SW_INFLIGHT), (uint64_t)(n - done), __ATOMIC_SEQ_CST);
        bump_seq();
    }
    productive = 1;
    return r;
}

ssize_t read(int fd, void *buf, size_t n) {
    if (!is_tracked(fd))
        return real_read(fd, buf, n);
    ensure_slot();
    struct pollfd pf = {fd, POLLIN, 0};
    int blocked = poll(&pf, 1, 0) != 1;
    if (blocked) {
        st(sw(SL_IOWAITS), ld(sw(SL_IOWAITS)) + 1);
        set_state(ST_BLOCKED_READ);
    }
    ssize_t r = real_read(fd, buf, n);
    // RUNNING strictly before the in-flight count drops (the bridge reads
    // the count first, then the states).
    if (blocked)
        set_state(ST_RUNNING);
    if (r > 0) {
        if (r % (ssize_t)sizeof(struct sender_request) == 0) {
            const struct sender_request *rq = buf;
            for (ssize_t i = 0; i < r / (ssize_t)sizeof *rq; i++) {
                if (qlen == QCAP) {
                    fprintf(stderr, "ntpperf_vtime: request queue full\n");
                    _exit(70);
                }
                q[(qhead + qlen++) % QCAP] = mono_to_vt(&rq[i].when);
            }
        }
        __atomic_fetch_sub(wp(SW_INFLIGHT), (uint64_t)r, __ATOMIC_SEQ_CST);
        bump_seq();
    }
    productive = 1;
    return r;
}

// ---------------------------------------------------------------- processes

pid_t fork(void) {
    if (!shm)
        return real_fork();
    ensure_slot();
    int child = alloc_slot();
    int base = SW_SLOTS + child * SW_SLOT_WORDS;
    st(base + SL_PID, 0);
    st(base + SL_STATE, ST_RUNNING);
    bump_seq();
    pid_t pid = real_fork();
    if (pid == 0) {
        my_slot = child;
        st(sw(SL_PID), (uint64_t)getpid());
        productive = 1;
        return 0;
    }
    if (pid < 0) {
        st(base + SL_STATE, ST_EXITED);
        bump_seq();
    } else {
        st(base + SL_PID, (uint64_t)pid);
    }
    return pid;
}

// ---------------------------------------------------------------- sockets

int sendmmsg(int fd, struct mmsghdr *v, unsigned int n, int flags) {
    int r = real_sendmmsg(fd, v, n, flags);
    if (shm && r > 0) {
        unsigned k = (unsigned)r < qlen ? (unsigned)r : qlen;
        qhead = (qhead + k) % QCAP;
        qlen -= k;
        productive = 1;
    }
    return r;
}

ssize_t recv(int fd, void *buf, size_t n, int flags) {
    ssize_t r = real_recv(fd, buf, n, flags);
    if (shm && r > 0)
        productive = 1;
    return r;
}

int ioctl(int fd, unsigned long req, ...) {
    va_list ap;
    va_start(ap, req);
    void *arg = va_arg(ap, void *);
    va_end(ap);
    if (shm && req == SIOCGIFHWADDR && arg) {
        struct ifreq *ifr = arg;
        const char *name = getenv("NTPPERF_IFACE");
        if (!name)
            name = "sim0";
        if (strncmp(ifr->ifr_name, name, IFNAMSIZ) == 0) {
            const char *m = getenv("NTPPERF_MAC");
            unsigned int b[6] = {2, 0, 0, 0, 0, 2};
            if (m && sscanf(m, "%x:%x:%x:%x:%x:%x", &b[0], &b[1], &b[2], &b[3], &b[4], &b[5]) != 6)
                return errno = EINVAL, -1;
            ifr->ifr_hwaddr.sa_family = 1;  // ARPHRD_ETHER
            for (int i = 0; i < 6; i++)
                ifr->ifr_hwaddr.sa_data[i] = (char)b[i];
            return 0;
        }
    }
    return real_ioctl(fd, req, arg);
}
