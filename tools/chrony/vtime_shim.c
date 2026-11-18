// SPDX-License-Identifier: Apache-2.0
//
// vtime_shim.c: LD_PRELOAD library that runs chronyd in the VIRTUAL time of
// the cycle-accurate simulation driven by rtl/src/bin/chrony_bridge.rs.
//
// Model (lockstep co-simulation, see docs/chrony-interop.md):
//  * The bridge owns time. It publishes the simulated true time (Unix epoch,
//    nanoseconds) in a shared page (`vt_ns`). Every clock chronyd reads
//    (clock_gettime of any clock id, gettimeofday, time) is derived from it.
//    While chronyd computes, the bridge does not advance: chronyd's work
//    takes zero virtual time, like an infinitely fast host.
//  * When chronyd blocks (select/pselect/poll/ppoll/nanosleep/...), the shim
//    publishes the virtual deadline and a wait sequence number, then polls
//    the real file descriptors with a short WALL timeout until either a
//    descriptor is ready (the bridge delivered a reply, chronyc asked
//    something) or the virtual time reached the deadline. Only then does
//    the bridge advance the simulation (up to the deadline or up to the
//    next datagram it delivers, whichever is first).
//  * Kernel socket timestamps (SO_TIMESTAMP, SO_TIMESTAMPNS,
//    SO_TIMESTAMPING) are wall-clock values: enabling them is refused
//    (ENOPROTOOPT), so chronyd falls back to its own (virtual) reading of
//    the clock right after select() returns, which is exactly the instant
//    the bridge delivered the datagram.
//
// Without CHRONY_VTIME_SHM in the environment every call is passed through.
// Build: gcc -O2 -shared -fPIC -o vtime_shim.so vtime_shim.c -ldl
#define _GNU_SOURCE
#include <dlfcn.h>
#include <errno.h>
#include <fcntl.h>
#include <poll.h>
#include <signal.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <sys/mman.h>
#include <sys/select.h>
#include <sys/socket.h>
#include <sys/syscall.h>
#include <sys/time.h>
#include <time.h>
#include <unistd.h>

// Layout of the shared page (u64 words), shared with chrony_bridge.rs.
enum {
    W_MAGIC = 0,     // VT_MAGIC once the bridge has initialised the page
    W_VT_NS = 1,     // virtual Unix time, ns (bridge)
    W_WAIT_SEQ = 2,  // incremented by the shim on every wait entry
    W_WAITING = 3,   // 1 while chronyd is blocked in a wait (shim)
    W_DEADLINE = 4,  // virtual deadline of the current wait, ns; ~0 = none
    W_T0_NS = 5,     // virtual time at start (base of the monotonic clocks)
    W_STOP = 6,      // the bridge has finished: time is frozen for good
    W_WORDS = 8
};
#define VT_MAGIC 0x56544d4348524e59ull  // "VTMCHRNY"
#define NONE_DEADLINE (~0ull)
// Wall-clock slice of one real poll of the descriptors while waiting.
#define SLICE_US 100

static volatile uint64_t *shm;
static int (*real_select)(int, fd_set *, fd_set *, fd_set *, struct timeval *);
static int (*real_poll)(struct pollfd *, nfds_t, int);
static int (*real_setsockopt)(int, int, int, const void *, socklen_t);

__attribute__((constructor)) static void vt_init(void) {
    real_select = dlsym(RTLD_NEXT, "select");
    real_poll = dlsym(RTLD_NEXT, "poll");
    real_setsockopt = dlsym(RTLD_NEXT, "setsockopt");
    const char *p = getenv("CHRONY_VTIME_SHM");
    if (!p || !*p)
        return;
    int fd = open(p, O_RDWR);
    if (fd < 0)
        return;
    void *m = mmap(NULL, 4096, PROT_READ | PROT_WRITE, MAP_SHARED, fd, 0);
    close(fd);
    if (m == MAP_FAILED)
        return;
    volatile uint64_t *s = m;
    if (__atomic_load_n(&s[W_MAGIC], __ATOMIC_ACQUIRE) != VT_MAGIC)
        return;
    shm = s;
}

static uint64_t ld(int w) { return __atomic_load_n(&shm[w], __ATOMIC_ACQUIRE); }
static void st(int w, uint64_t v) { __atomic_store_n(&shm[w], v, __ATOMIC_SEQ_CST); }

static uint64_t vt_now(void) { return ld(W_VT_NS); }

static void ns_to_ts(uint64_t ns, struct timespec *ts) {
    ts->tv_sec = (time_t)(ns / 1000000000ull);
    ts->tv_nsec = (long)(ns % 1000000000ull);
}

// ---------------------------------------------------------------- clocks

int clock_gettime(clockid_t id, struct timespec *ts) {
    if (!shm) {
        return (int)syscall(SYS_clock_gettime, id, ts);
    }
    uint64_t v = vt_now();
    switch (id) {
    case CLOCK_REALTIME:
    case CLOCK_REALTIME_COARSE:
#ifdef CLOCK_TAI
    case CLOCK_TAI:
#endif
        ns_to_ts(v, ts);
        return 0;
    default:
        // Monotonic, boottime, CPU-time clocks: virtual time since start,
        // plus one second so that it is never zero.
        ns_to_ts(v - ld(W_T0_NS) + 1000000000ull, ts);
        return 0;
    }
}

int gettimeofday(struct timeval *tv, void *tz) {
    (void)tz;
    struct timespec ts;
    clock_gettime(CLOCK_REALTIME, &ts);
    if (tv) {
        tv->tv_sec = ts.tv_sec;
        tv->tv_usec = ts.tv_nsec / 1000;
    }
    return 0;
}

time_t time(time_t *t) {
    struct timespec ts;
    clock_gettime(CLOCK_REALTIME, &ts);
    if (t)
        *t = ts.tv_sec;
    return ts.tv_sec;
}

// chronyd must never touch the host clock (it runs with -x): refuse.
int clock_settime(clockid_t id, const struct timespec *ts) {
    (void)id;
    (void)ts;
    errno = EPERM;
    return -1;
}

int settimeofday(const struct timeval *tv, const struct timezone *tz) {
    (void)tv;
    (void)tz;
    errno = EPERM;
    return -1;
}

// ---------------------------------------------------------------- waits

// Enters a virtual wait ending at `deadline` (ns, NONE_DEADLINE = none).
static void wait_enter(uint64_t deadline) {
    st(W_DEADLINE, deadline);
    st(W_WAIT_SEQ, ld(W_WAIT_SEQ) + 1);
    st(W_WAITING, 1);
}

static void wait_leave(void) { st(W_WAITING, 0); }

static int deadline_passed(uint64_t deadline) {
    return deadline != NONE_DEADLINE && vt_now() >= deadline;
}

// A wait lasts at least MIN_WAIT_NS of virtual time, even with a zero
// timeout: chronyd computes select()'s timeout in whole microseconds from a
// timer that may be less than 1 us ahead; with a frozen clock a zero-length
// wait would never let the timer expire (it would spin). On a real host the
// call itself takes about that long.
#define MIN_WAIT_NS 1000ull

static uint64_t add_ns(uint64_t base, uint64_t d) {
    if (d < MIN_WAIT_NS)
        d = MIN_WAIT_NS;
    return d >= NONE_DEADLINE - base ? NONE_DEADLINE - 1 : base + d;
}

int select(int n, fd_set *r, fd_set *w, fd_set *e, struct timeval *tv) {
    if (!shm)
        return real_select(n, r, w, e, tv);
    uint64_t start = vt_now();
    uint64_t deadline =
        tv ? add_ns(start, (uint64_t)tv->tv_sec * 1000000000ull + (uint64_t)tv->tv_usec * 1000ull)
           : NONE_DEADLINE;
    fd_set r0, w0, e0;
    if (r)
        r0 = *r;
    if (w)
        w0 = *w;
    if (e)
        e0 = *e;
    wait_enter(deadline);
    int rc;
    for (;;) {
        if (r)
            *r = r0;
        if (w)
            *w = w0;
        if (e)
            *e = e0;
        struct timeval z = {0, SLICE_US};
        rc = real_select(n, r, w, e, &z);
        if (rc != 0)
            break;
        if (deadline_passed(deadline)) {
            rc = 0;
            break;
        }
    }
    wait_leave();
    if (rc == 0) {
        if (r)
            FD_ZERO(r);
        if (w)
            FD_ZERO(w);
        if (e)
            FD_ZERO(e);
    }
    if (tv) {
        // Linux semantics: the remaining (virtual) time.
        uint64_t now = vt_now();
        uint64_t rem = now >= deadline ? 0 : deadline - now;
        tv->tv_sec = (time_t)(rem / 1000000000ull);
        tv->tv_usec = (suseconds_t)((rem % 1000000000ull) / 1000ull);
    }
    return rc;
}

int pselect(int n, fd_set *r, fd_set *w, fd_set *e, const struct timespec *ts,
            const sigset_t *mask) {
    (void)mask;  // chronyd does not use pselect; signal masks not emulated
    struct timeval tv, *ptv = NULL;
    if (ts) {
        tv.tv_sec = ts->tv_sec;
        tv.tv_usec = ts->tv_nsec / 1000;
        ptv = &tv;
    }
    return select(n, r, w, e, ptv);
}

int poll(struct pollfd *fds, nfds_t n, int timeout_ms) {
    if (!shm)
        return real_poll(fds, n, timeout_ms);
    uint64_t deadline =
        timeout_ms < 0 ? NONE_DEADLINE : add_ns(vt_now(), (uint64_t)timeout_ms * 1000000ull);
    wait_enter(deadline);
    int rc;
    for (;;) {
        rc = real_poll(fds, n, 0);
        if (rc != 0)
            break;
        if (deadline_passed(deadline))
            break;
        struct timespec s = {0, SLICE_US * 1000};
        syscall(SYS_nanosleep, &s, NULL);  // the real one: nanosleep is interposed
    }
    wait_leave();
    return rc;
}

int ppoll(struct pollfd *fds, nfds_t n, const struct timespec *ts, const sigset_t *mask) {
    (void)mask;
    int ms = ts ? (int)(ts->tv_sec * 1000 + ts->tv_nsec / 1000000) : -1;
    return poll(fds, n, ms);
}

static int vsleep_ns(uint64_t d) {
    uint64_t deadline = add_ns(vt_now(), d);
    wait_enter(deadline);
    while (!deadline_passed(deadline)) {
        struct timespec s = {0, SLICE_US * 1000};
        syscall(SYS_nanosleep, &s, NULL);
    }
    wait_leave();
    return 0;
}

int nanosleep(const struct timespec *req, struct timespec *rem) {
    if (!shm)
        return (int)syscall(SYS_nanosleep, req, rem);
    if (rem)
        rem->tv_sec = rem->tv_nsec = 0;
    return vsleep_ns((uint64_t)req->tv_sec * 1000000000ull + (uint64_t)req->tv_nsec);
}

int clock_nanosleep(clockid_t id, int flags, const struct timespec *req, struct timespec *rem) {
    if (!shm)
        return (int)syscall(SYS_clock_nanosleep, id, flags, req, rem);
    uint64_t d = (uint64_t)req->tv_sec * 1000000000ull + (uint64_t)req->tv_nsec;
    if (flags & TIMER_ABSTIME) {
        struct timespec now;
        clock_gettime(id, &now);
        uint64_t n = (uint64_t)now.tv_sec * 1000000000ull + (uint64_t)now.tv_nsec;
        d = d > n ? d - n : 0;
    }
    vsleep_ns(d);
    return 0;
}

unsigned int sleep(unsigned int s) {
    if (!shm) {
        struct timespec t = {s, 0};
        syscall(SYS_nanosleep, &t, NULL);
        return 0;
    }
    vsleep_ns((uint64_t)s * 1000000000ull);
    return 0;
}

// ---------------------------------------------------------------- sockets

int setsockopt(int fd, int level, int name, const void *val, socklen_t len) {
    if (shm && level == SOL_SOCKET &&
        (name == SO_TIMESTAMP || name == SO_TIMESTAMPNS || name == SO_TIMESTAMPING)) {
        // Wall-clock kernel timestamps would contradict the virtual clock.
        errno = ENOPROTOOPT;
        return -1;
    }
    return real_setsockopt(fd, level, name, val, len);
}
