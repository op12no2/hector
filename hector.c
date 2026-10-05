/*
 * hector.c - Hector the hexapod: 12 Waveshare SC09 bus servos via a
 * Waveshare Bus Servo Adapter (A) / USB serial. Started as a copy of
 * ../servo/servo.c (the generic servo tester) and keeps all its commands;
 * the hexapod commands (stand, legtest, ident, offset, calibrate, walk, set) are at the end.
 *
 * Build:  make
 * Run:    ./hector [/dev/ttyACM0] [baud]
 * Then type commands on stdin ("help" lists them).
 *
 * The same file builds for the ATOM S3R (ESP-IDF, atom/): the bus is then
 * UART1 on G38/G39 into the adapter's UART header and the console is the
 * ATOM's USB. Only the I/O section below differs (ESP_PLATFORM).
 *
 * Protocol:
 *   TX: FF FF ID LEN INSTR PARAM... CHK     LEN = nparams + 2
 *   RX: FF FF ID LEN ERR   PARAM... CHK
 *   CHK = ~(ID + LEN + INSTR/ERR + sum(PARAM)) & 0xFF
 * 16-bit registers are high byte first.
 */
#include <errno.h>
#include <math.h>
#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <unistd.h>
#ifdef ESP_PLATFORM
#include "driver/gpio.h"
#include "driver/uart.h"
#include "driver/usb_serial_jtag.h"
#include "driver/usb_serial_jtag_vfs.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#else
#include <fcntl.h>
#include <signal.h>
#include <termios.h>
#include <sys/select.h>
#include <sys/time.h>
#endif

#define INST_PING       0x01
#define INST_READ       0x02
#define INST_WRITE      0x03
#define INST_SYNC_WRITE 0x83
#define BROADCAST       0xFE

/* SC09 register map */
#define REG_ID            0x05
#define REG_MIN_ANGLE     0x09  /* u16 */
#define REG_MAX_ANGLE     0x0B  /* u16 */
#define REG_DEADZONE_CW   0x1A
#define REG_DEADZONE_CCW  0x1B
#define REG_TORQUE_ENABLE 0x28
#define REG_GOAL_POS      0x2A  /* u16 */
#define REG_GOAL_TIME     0x2C  /* u16 */
#define REG_GOAL_SPEED    0x2E  /* u16 */
#define REG_LOCK          0x30  /* EPROM lock: 1=locked, 0=unlocked */
#define REG_PRESENT_POS   0x38  /* u16 */
#define REG_PRESENT_SPEED 0x3A  /* u16 */
#define REG_PRESENT_LOAD  0x3C  /* u16 */
#define REG_VOLTAGE       0x3E
#define REG_TEMP          0x3F
#define REG_MOVING        0x42

static int verbose = 0;
static int timeout_ms = 50;

/* 16-bit register values are high byte first */
static int get16(const unsigned char *b) { return b[0] << 8 | b[1]; }
static void put16(unsigned char *b, int v) { b[0] = v >> 8; b[1] = v; }

static void die(const char *fmt, ...)
{
    va_list ap; va_start(ap, fmt); vfprintf(stderr, fmt, ap); va_end(ap);
    fputc('\n', stderr); exit(1);
}

/*
 * ---- I/O ----
 * The servo bus (open_port, bus_flush, bus_write, read_bytes), the console
 * (con_*) and timing (msleep, tick_*). Everything else is the same on the Pi
 * and the ATOM.
 */
#ifdef ESP_PLATFORM

/* ATOM S3R: UART1, TX G38 to the adapter's H2 TXD, RX G39 from its RXD (jumper in A) */
#define BUS_UART UART_NUM_1
#define BUS_TX   38
#define BUS_RX   39

static void open_port(const char *dev, int baud)
{
    (void)dev;
    uart_config_t c = {
        .baud_rate = baud, .data_bits = UART_DATA_8_BITS, .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1, .flow_ctrl = UART_HW_FLOWCTRL_DISABLE, .source_clk = UART_SCLK_DEFAULT,
    };
    if (uart_driver_install(BUS_UART, 1024, 0, 0, NULL, 0) != ESP_OK || uart_param_config(BUS_UART, &c) != ESP_OK ||
        uart_set_pin(BUS_UART, BUS_TX, BUS_RX, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE) != ESP_OK)
        die("uart setup failed");
    gpio_set_pull_mode(BUS_RX, GPIO_PULLUP_ONLY);
}

static void bus_flush(void) { uart_flush_input(BUS_UART); }

static void bus_write(const unsigned char *b, int n)
{
    uart_write_bytes(BUS_UART, b, n);
    uart_wait_tx_done(BUS_UART, portMAX_DELAY);
}

/* read n bytes or time out; returns bytes read */
static int read_bytes(unsigned char *buf, int n, int ms)
{
    int r = uart_read_bytes(BUS_UART, buf, n, pdMS_TO_TICKS(ms));
    return r < 0 ? 0 : r;
}

/*
 * The console is the ATOM's USB (USB-Serial/JTAG). It has no termios and no
 * SIGINT, so it is always "raw", and ctrl-c arrives as a byte (walk checks).
 */
static int con_block = 1;

static void con_init(void)
{
    usb_serial_jtag_driver_config_t c = USB_SERIAL_JTAG_DRIVER_CONFIG_DEFAULT();
    usb_serial_jtag_driver_install(&c);
    usb_serial_jtag_vfs_use_driver();
}

static int con_tty(void) { return 1; }
static void con_raw(int block) { con_block = block; }
static void con_restore(void) { con_block = 1; }

/* next byte; with con_raw(0), -1 if there isn't one */
static int con_getc(void)
{
    unsigned char c;
    return usb_serial_jtag_read_bytes(&c, 1, con_block ? portMAX_DELAY : 0) == 1 ? c : -1;
}

/* drop typed-ahead input */
static void con_drop(void)
{
    unsigned char c;
    while (usb_serial_jtag_read_bytes(&c, 1, 0) == 1) ;
}

static volatile int halted;
static void sigint_catch(void) {}
static void sigint_restore(void) {}

static void msleep(int ms) { vTaskDelay(pdMS_TO_TICKS(ms)); }

/* fixed-rate ticks for walk: tick_start, then tick_wait(ms) each tick; tick_start again to resync */
static TickType_t tick_last;
static void tick_start(void) { tick_last = xTaskGetTickCount(); }
static void tick_wait(int ms) { xTaskDelayUntil(&tick_last, pdMS_TO_TICKS(ms)); }

#else

static int fd = -1;

static speed_t baud_const(int baud)
{
    switch (baud) {
    case 9600: return B9600;     case 19200: return B19200;
    case 38400: return B38400;   case 57600: return B57600;
    case 115200: return B115200; case 230400: return B230400;
    case 500000: return B500000; case 1000000: return B1000000;
    default: return 0;
    }
}

static void open_port(const char *dev, int baud)
{
    speed_t sp = baud_const(baud);
    if (!sp) die("unsupported baud %d", baud);
    fd = open(dev, O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (fd < 0) die("open %s: %s", dev, strerror(errno));
    struct termios t;
    if (tcgetattr(fd, &t) < 0) die("tcgetattr: %s", strerror(errno));
    cfmakeraw(&t);
    cfsetispeed(&t, sp); cfsetospeed(&t, sp);
    t.c_cflag |= CLOCAL | CREAD;
    t.c_cflag &= ~(PARENB | CSTOPB | CRTSCTS);
    t.c_cflag = (t.c_cflag & ~CSIZE) | CS8;
    t.c_cc[VMIN] = 0; t.c_cc[VTIME] = 0;
    if (tcsetattr(fd, TCSANOW, &t) < 0) die("tcsetattr: %s", strerror(errno));
    tcflush(fd, TCIOFLUSH);
}

static void bus_flush(void) { tcflush(fd, TCIFLUSH); }

static void bus_write(const unsigned char *b, int n)
{
    if (write(fd, b, n) != n) die("write: %s", strerror(errno));
    tcdrain(fd);
}

/* read exactly n bytes or time out; returns bytes read */
static int read_bytes(unsigned char *buf, int n, int ms)
{
    int got = 0;
    while (got < n) {
        fd_set rf; FD_ZERO(&rf); FD_SET(fd, &rf);
        struct timeval tv = { ms / 1000, (ms % 1000) * 1000 };
        int r = select(fd + 1, &rf, NULL, NULL, &tv);
        if (r <= 0) break;
        r = read(fd, buf + got, n - got);
        if (r <= 0) break;
        got += r;
    }
    return got;
}

/* the console is stdin/stdout; raw = no line buffering or echo, ctrl-c still a signal */
static struct termios con_old;

static void con_init(void) {}
static int con_tty(void) { return isatty(0); }

/* block: con_getc waits for a key (else returns -1 if there isn't one) */
static void con_raw(int block)
{
    struct termios raw;
    tcgetattr(0, &con_old);
    raw = con_old;
    raw.c_lflag &= ~(ICANON | ECHO);
    raw.c_cc[VMIN] = block; raw.c_cc[VTIME] = 0;
    tcsetattr(0, TCSANOW, &raw);
}

static void con_restore(void) { tcsetattr(0, TCSANOW, &con_old); }

static int con_getc(void)
{
    unsigned char c;
    return read(0, &c, 1) == 1 ? c : -1;
}

static void con_drop(void) { tcflush(0, TCIFLUSH); }

static volatile sig_atomic_t halted;
static void on_sigint(int sig) { (void)sig; halted = 1; }
static struct sigaction oldsa;

static void sigint_catch(void)
{
    struct sigaction sa = { .sa_handler = on_sigint, .sa_flags = SA_RESTART };
    sigemptyset(&sa.sa_mask);
    sigaction(SIGINT, &sa, &oldsa);
}

static void sigint_restore(void) { sigaction(SIGINT, &oldsa, NULL); }

static void msleep(int ms)
{
    struct timespec ts = { ms / 1000, (ms % 1000) * 1000000L };
    nanosleep(&ts, NULL);
}

static struct timespec tick_next;
static void tick_start(void) { clock_gettime(CLOCK_MONOTONIC, &tick_next); }

static void tick_wait(int ms)
{
    tick_next.tv_nsec += ms * 1000000L;
    if (tick_next.tv_nsec >= 1000000000L) { tick_next.tv_nsec -= 1000000000L; tick_next.tv_sec++; }
    clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &tick_next, NULL);
}

#endif

static void hexdump(const char *tag, const unsigned char *b, int n)
{
    printf("%s", tag);
    for (int i = 0; i < n; i++) printf(" %02X", b[i]);
    printf("\n");
}

/*
 * Send instruction packet, receive status packet.
 * Returns number of params in reply (>=0), -1 on timeout/no reply,
 * -2 on bad checksum. *err gets the error byte. The adapter is half
 * duplex so we read back (and discard) our own TX echo first.
 */
static int txrx(int id, int instr, const unsigned char *p, int np,
                unsigned char *err, unsigned char *out, int outmax)
{
    unsigned char tx[260];
    int n = 0;
    tx[n++] = 0xFF; tx[n++] = 0xFF; tx[n++] = id; tx[n++] = np + 2; tx[n++] = instr;
    unsigned sum = id + (np + 2) + instr;
    for (int i = 0; i < np; i++) { tx[n++] = p[i]; sum += p[i]; }
    tx[n++] = ~sum & 0xFF;

    bus_flush();
    bus_write(tx, n);
    if (verbose) hexdump("  tx:", tx, n);
    if (id == BROADCAST && instr != INST_PING) return 0;   /* no reply, don't wait for one */

    /* swallow echo if the adapter loops TX back to RX */
    unsigned char rx[64];
    int got = read_bytes(rx, n, timeout_ms);
    if (got == n && memcmp(rx, tx, n) == 0) {
        if (verbose) printf("  (echo)\n");
        got = 0;
    }

    /* find header */
    int hdr = 0;
    while (1) {
        if (got < 4) {
            int r = read_bytes(rx + got, 4 - got, timeout_ms);
            if (r <= 0) { if (verbose && got) hexdump("  rx(partial):", rx, got); return -1; }
            got += r;
        }
        for (hdr = 0; hdr + 1 < got; hdr++)
            if (rx[hdr] == 0xFF && rx[hdr + 1] == 0xFF && (hdr + 2 >= got || rx[hdr + 2] != 0xFF)) break;
        if (hdr + 3 < got) break;
        memmove(rx, rx + hdr, got - hdr); got -= hdr;
        if (got >= 4) { hdr = 0; break; }
    }
    if (hdr) { memmove(rx, rx + hdr, got - hdr); got -= hdr; }
    int len = rx[3];
    int total = 4 + len;
    if (total > (int)sizeof rx) return -2;
    if (got < total) got += read_bytes(rx + got, total - got, timeout_ms);
    if (got < total) { if (verbose) hexdump("  rx(short):", rx, got); return -1; }
    if (verbose) hexdump("  rx:", rx, total);
    unsigned s = 0;
    for (int i = 2; i < total - 1; i++) s += rx[i];
    if ((~s & 0xFF) != rx[total - 1]) { printf("  bad checksum\n"); return -2; }
    if (err) *err = rx[4];
    int npar = len - 2;
    for (int i = 0; i < npar && i < outmax; i++) out[i] = rx[5 + i];
    return npar;
}

static const char *errstr(unsigned char e)
{
    static char buf[128];
    if (!e) return "ok";
    buf[0] = 0;
    if (e & 0x01) strcat(buf, " voltage");
    if (e & 0x02) strcat(buf, " angle");
    if (e & 0x04) strcat(buf, " overheat");
    if (e & 0x08) strcat(buf, " overcurrent");
    if (e & 0x20) strcat(buf, " overload");
    return buf + 1;
}

/*
 * Error bits from status replies, per id, gathered by read_regs/write_regs
 * rather than printed there, so each id's status is reported once per command:
 * on its own output line via status_tail(), else by flush_status() afterwards.
 */
static unsigned char status_err[254];

static void note_status(int id, unsigned char e)
{
    if (id >= 0 && id < 254) status_err[id] |= e;
}

/* " (status ...)" for id's pending errors, clearing them; "" if none */
static const char *status_tail(int id)
{
    static char buf[160];
    buf[0] = 0;
    if (id >= 0 && id < 254 && status_err[id]) {
        snprintf(buf, sizeof buf, " (status %s)", errstr(status_err[id]));
        status_err[id] = 0;
    }
    return buf;
}

static void flush_status(void)
{
    for (int id = 0; id < 254; id++)
        if (status_err[id]) {
            printf("id %d: status %s\n", id, errstr(status_err[id]));
            status_err[id] = 0;
        }
}

static int ping(int id)
{
    unsigned char e = 0, out[8];
    int r = txrx(id, INST_PING, NULL, 0, &e, out, sizeof out);
    if (r < 0) return r;
    unsigned char m[2]; int model = -1;
    if (txrx(id, INST_READ, (unsigned char[]){0x03, 2}, 2, &e, m, 2) == 2)
        model = get16(m);
    printf("id %d: alive, model %d (status %s)\n", id, model, errstr(e));
    return 0;
}

/* Ping every id 0..253 with a short timeout; returns number found. */
static int ping_all(void)
{
    int save = timeout_ms, found = 0;
    timeout_ms = 15;
    for (int id = 0; id <= 253; id++)
        if (ping(id) >= 0) found++;
    timeout_ms = save;
    return found;
}

static int read_regs(int id, int addr, int n, unsigned char *out)
{
    unsigned char p[2] = { addr, n }, e = 0;
    int r = txrx(id, INST_READ, p, 2, &e, out, n);
    if (r < 0) { printf("id %d: no reply\n", id); return -1; }
    note_status(id, e);
    return r;
}

static int write_regs(int id, int addr, const unsigned char *v, int n)
{
    unsigned char p[32], e = 0, out[8];
    p[0] = addr; memcpy(p + 1, v, n);
    int r = txrx(id, INST_WRITE, p, n + 1, &e, out, sizeof out);
    if (r < 0) { printf("id %d: no reply\n", id); return -1; }
    note_status(id, e);
    return 0;
}

static int read_u16(int id, int addr, int *val)
{
    unsigned char b[2];
    if (read_regs(id, addr, 2, b) < 2) return -1;
    *val = get16(b);
    return 0;
}

static int write_u8(int id, int addr, int v)
{
    unsigned char b = v; return write_regs(id, addr, &b, 1);
}

static int write_u16(int id, int addr, int v)
{
    unsigned char b[2];
    put16(b, v);
    return write_regs(id, addr, b, 2);
}

static void help(void)
{
    puts(
    "<id> can be a list, no spaces: 3, 1-12, 2,6, 1-6,9 (every command that takes one, except setid)\n"
    "several commands on one line, separated by ';', run in order: move 1 200; move 1 800\n"
    "up/down arrows recall earlier lines (up + enter repeats the last)\n"
    "\n"
    "ping [id]                 ping servo(s); no id pings all 0..253\n"
    "pos <id>                  report position\n"
    "stat <id>                 pos/speed/load/volt/temp/moving, mode and angle limits\n"
    "move <id> <pos> [time] [speed]   goal pos (0..1023); time = ms to reach it (0 = asap);\n"
    "                          speed = max speed in steps/s, 0..1023 (0 = full speed, reg 0x2E); both default 0;\n"
    "                          a list of ids is sent as one sync write so they all start together;\n"
    "                          warns if the goal is outside an id's angle limits (the servo clamps it);\n"
    "                          waits for the move to finish, reports ids off goal by more than their dead zone;\n"
    "                          each id's offset (offset[] in hector.c, set by calibrate) is added to pos\n"
    "torque <id> 0|1           torque enable\n"
    "rb <id> <addr>            read byte\n"
    "rw <id> <addr>            read 16-bit\n"
    "wb <id> <addr> <val>      write byte\n"
    "ww <id> <addr> <val>      write 16-bit\n"
    "dump <id>                 dump registers 0x00..0x45\n"
    "setid <old> <new>         change ID (unlock EPROM, write, lock)\n"
    "lock <id> 0|1             EPROM lock register (0x30)\n"
    "limits <id> [min max]     read/set angle limits (0x09/0x0B)\n"
    "motor <id>                enter motor/wheel mode (angle limits -> 0/0, EPROM)\n"
    "spin <id> <speed>         motor mode speed -1000..1000 (0 = stop)\n"
    "servomode <id> [min max]  back to position mode (limits default 20..1003)\n"
    "raw <hexbytes...>         send raw bytes, print reply\n"
    "verbose 0|1               hex dump packets\n"
    "timeout [ms]              show/set reply timeout\n"
    "help | ?                  list commands\n"
    "quit | q | exit           exit\n"
    "\n"
    "hexapod (leg n: hip id n, lift id n+6; layout and directions in legs[] in hector.c):\n"
    "stand [ms]                all feet down, hips centred, taking ms (default 1000)\n"
    "legtest [1-6]             each leg in turn (or one, by hip id): up to 700, forward 80, back, down; the others stay put\n"
    "ident [id]                twitch each id (default 1-12) in turn, 1 s apart: +30 from where it is and back\n"
    "offset [id val]           show the per-id offsets added to goal positions, or set one (until exit)\n"
    "calibrate                 stand, then nudge the lift offsets until all six feet carry the same load;\n"
    "                          prints the offset table to paste into hector.c\n"
    "walk [cycles] [stride] [turn]   stand, then walk with the current gait; stride < 0 walks backwards,\n"
    "                          turn > 0 turns left (stride 0 turns on the spot); no cycles = until a key;\n"
    "                          a key stops (it finishes the step and brings the legs to centre), ctrl-c freezes\n"
    "set [name value]          list or set walk parameters: gait step stride lift height");
}

static int arg(char **tok, int i, int ntok, int dflt, int *ok)
{
    if (i >= ntok) { if (ok) *ok = 0; return dflt; }
    return (int)strtol(tok[i], NULL, 0);
}

/*
 * Parse an id list like "3", "1-12", "2,6" or "1-6,9" into ids[] (in order,
 * duplicates dropped). Returns count, or -1 if malformed or outside 0..253.
 */
static int parse_ids(const char *s, int *ids)
{
    unsigned char seen[254] = {0};
    int n = 0;
    for (;;) {
        char *e;
        long lo = strtol(s, &e, 0), hi = lo;
        if (e == s) return -1;
        if (*e == '-') { s = e + 1; hi = strtol(s, &e, 0); if (e == s) return -1; }
        if (lo < 0 || hi > 253 || lo > hi) return -1;
        for (long id = lo; id <= hi; id++) if (!seen[id]) { seen[id] = 1; ids[n++] = id; }
        if (!*e) return n;
        if (*e != ',') return -1;
        s = e + 1;
    }
}

/* goal pos[i] to ids[i], same time/speed for all, in one SYNC_WRITE (35 per packet: LEN = 2 + 7n + 2 <= 255) */
static void sync_move(const int *ids, int nid, const int *pos, int tm, int sp)
{
    for (int k = 0; k < nid; k += 35) {
        int m = nid - k < 35 ? nid - k : 35, n = 0;
        unsigned char p[2 + 35 * 7];
        p[n++] = REG_GOAL_POS; p[n++] = 6;
        for (int i = k; i < k + m; i++) {
            p[n++] = ids[i];
            put16(p + n, pos[i]); put16(p + n + 2, tm); put16(p + n + 4, sp); n += 6;
        }
        txrx(BROADCAST, INST_SYNC_WRITE, p, n, NULL, NULL, 0);
    }
}

/*
 * After a move: warn about ids whose angle limits clamp the goal, poll each id
 * until it stops, then report any that settled outside the (clamped) goal
 * +/- its dead zone (larger of the CW/CCW regs). An id counts as stopped once
 * "moving" has read 0 and pos hasn't changed for 100 ms (300 ms if off goal,
 * as it can creep on after a pause): a slow start reads moving 0, and after an
 * overshoot the flag drops while the servo is still easing back. Gives up
 * after time + 3 s.
 */
static void verify_move(const int *ids, int nid, const int *goal, int tm)
{
    int tol[254], gl[254], still[254], last[254], done[254], left = nid;
    for (int i = 0; i < nid; i++) {
        unsigned char b[REG_DEADZONE_CCW - REG_MIN_ANGLE + 1];
        still[i] = 0;
        last[i] = -1;
        gl[i] = goal[i];
        done[i] = read_regs(ids[i], REG_MIN_ANGLE, sizeof b, b) != (int)sizeof b;   /* no reply: already reported */
        left -= done[i];
        if (done[i]) continue;
#define R(a) (b + (a) - REG_MIN_ANGLE)
        int cw = *R(REG_DEADZONE_CW), ccw = *R(REG_DEADZONE_CCW);
        int mn = get16(R(REG_MIN_ANGLE)), mx = get16(R(REG_MAX_ANGLE));
#undef R
        tol[i] = cw > ccw ? cw : ccw;
        if (mn || mx) gl[i] = goal[i] < mn ? mn : goal[i] > mx ? mx : goal[i];   /* 0/0 = motor mode */
        if (gl[i] != goal[i])
            printf("id %d: goal %d outside limits %d..%d, clamped to %d\n", ids[i], goal[i], mn, mx, gl[i]);
    }
    for (int ms = 0; left; ms += 20) {
        usleep(20000);
        for (int i = 0; i < nid; i++) {
            if (done[i]) continue;
            unsigned char b[11];
            if (read_regs(ids[i], REG_PRESENT_POS, 11, b) != 11) { done[i] = 1; left--; continue; }
            int p = get16(b), d = p - gl[i], moved = p != last[i];
            last[i] = p;
            if (b[REG_MOVING - REG_PRESENT_POS]) {
                still[i] = 0;
                if (ms < tm + 3000) continue;
                printf("id %d: still moving at %d after %d ms, goal %d\n", ids[i], p, ms, gl[i]);
            }
            else {
                if (moved) still[i] = 0;
                if (++still[i] < (abs(d) > tol[i] ? 15 : 5) && ms < tm + 3000) continue;
                if (abs(d) > tol[i])
                    printf("id %d: stopped at %d, goal %d (off by %+d, dead zone %d)\n", ids[i], p, gl[i], d, tol[i]);
            }
            done[i] = 1; left--;
        }
    }
}

/* ---- hexapod ---- */

#define CENTRE   511
#define HIP_MAX  150    /* hip clamp either side of centre, steps: neighbouring legs can meet */
#define UP_MAX   300    /* lift clamp above centre */
#define DOWN_MAX 50     /* lift clamp below centre: the legs reach the chassis at about 50 down */
#define SPLAY_MAX 150   /* most negative height: stance legs this far above centre */
#define TICK_MS  20     /* pose update period while walking */

/*
 * Per-id offsets, added to every goal position sent: move's and the legs'.
 * With [7] = 20, move 7 511 sends 531. Reads (pos, stat, move's check) are the servo's own
 * positions. The lifts' are from calibrate (2026-10-01, height 0).
 * The offset and calibrate commands change it until exit; paste what they
 * print here to keep it.
 */
static int offset[254] = { [7] = 4, [8] = -7, [9] = 3, [10] = -23, [11] = 5, [12] = -25 };

/* the offset table as a line of C, to paste above */
static void print_offsets(void)
{
    printf("static int offset[254] = {");
    int n = 0;
    for (int id = 0; id < 254; id++)
        if (offset[id]) printf("%s [%d] = %d", n++ ? "," : "", id, offset[id]);
    puts(n ? " };" : " 0 };");
}

/*
 * The legs. Hips (yaw) are ids 1-6 and lifts 7-12; side and row say where each
 * leg is, which is all the gaits need. hip_dir is +1 if a higher position swings
 * the foot forward (towards the head), lift_dir +1 if a higher position raises
 * the foot; legtest shows both.
 */
static struct leg {
    int side, row;              /* side 0 left, 1 right; row 0 front, 1 middle, 2 rear */
    int hip, lift;              /* servo ids */
    int hip_dir, lift_dir;
    double x, z;                /* hip swing (+ forward) and foot lift (+ up) from the stance pose, steps */
} legs[6] = {
    /* side row hip lift hip_dir lift_dir x z */
    { 0, 0, 1,  7, -1, 1, 0, 0 },     /* LF */
    { 0, 1, 2,  8, -1, 1, 0, 0 },     /* LM */
    { 0, 2, 3,  9, -1, 1, 0, 0 },     /* LR */
    { 1, 0, 4, 10,  1, 1, 0, 0 },     /* RF */
    { 1, 1, 5, 11,  1, 1, 0, 0 },     /* RM */
    { 1, 2, 6, 12,  1, 1, 0, 0 },     /* RR */
};

/*
 * Gaits: each leg is in the air for `swing` of the cycle, starting at its
 * offset in the cycle (0..1), and on the ground for the rest.
 */
static const struct gait {
    const char *name;
    double swing;
    double off[2][3];           /* [left, right][front, middle, rear] */
} gaits[] = {
    { "wave",   1 / 6.0, { { 5 / 6.0, 4 / 6.0, 3 / 6.0 }, { 2 / 6.0, 1 / 6.0, 0 } } },       /* 1 leg up at a time, back to front, right then left */
    { "ripple", 1 / 3.0, { { 2 / 3.0, 1 / 3.0, 0 }, { 1 / 6.0, 5 / 6.0, 1 / 2.0 } } },       /* 2 up: each side back to front, sides half a cycle apart */
    { "tripod", 1 / 2.0, { { 0, 1 / 2.0, 0 }, { 1 / 2.0, 0, 1 / 2.0 } } },                   /* 3 up: LF+RM+LR, then RF+LM+RR */
};

/* walk parameters (the set command) */
static int gait = 0, step_ms = 800, stride = 50, lift = 70, height = 0;       /* gentle first-walk settings, for a new body */
static const struct param { const char *name; int *v, min, max; const char *what; } params[] = {
    { "gait",   &gait,    0,   2,        "wave, ripple or tripod (1, 2 or 3 legs up at a time)" },
    { "step",   &step_ms, 200, 5000,     "ms each leg spends in the air" },
    { "stride", &stride,  0,   HIP_MAX,  "hip swing either side of centre, steps" },
    { "lift",   &lift,    0,   UP_MAX,   "foot lift during a step, steps" },
    { "height", &height,  -SPLAY_MAX, DOWN_MAX, "legs this far below centre when standing, steps: > 0 raises the body, < 0 lowers it, legs splayed" },
};

static double now(void)
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return ts.tv_sec + ts.tv_nsec / 1e9;
}

static double clampd(double v, double lo, double hi) { return v < lo ? lo : v > hi ? hi : v; }

static const char *leg_name(const struct leg *L)
{
    static char s[3];
    s[0] = "LR"[L->side]; s[1] = "FMR"[L->row]; s[2] = 0;
    return s;
}

/* a leg's x/z as its hip and lift goal positions */
static void leg_goals(const struct leg *L, int *hip, int *lift)
{
    *hip = CENTRE + offset[L->hip] + L->hip_dir * lround(clampd(L->x, -HIP_MAX, HIP_MAX));
    *lift = CENTRE + offset[L->lift] + L->lift_dir * lround(clampd(L->z - height, -DOWN_MAX, UP_MAX));
}

/* every leg's x/z as goal positions, in one sync write with goal time tm ms */
static void send_pose(int tm)
{
    int ids[12], pos[12];
    for (int i = 0; i < 6; i++) {
        ids[i] = legs[i].hip;
        ids[i + 6] = legs[i].lift;
        leg_goals(&legs[i], &pos[i], &pos[i + 6]);
    }
    sync_move(ids, 12, pos, tm, 0);
}

/* one leg's x/z, leaving the others where they are */
static void send_leg(const struct leg *L, int tm)
{
    int ids[2] = { L->hip, L->lift }, pos[2];
    leg_goals(L, &pos[0], &pos[1]);
    sync_move(ids, 2, pos, tm, 0);
}

/* read voltage and temperature from all 12 servos; -1 if any doesn't reply */
static int check_servos(void)
{
    int vmin = 255, vmax = 0, tmin = 255, tmax = 0, bad = 0;
    for (int i = 0; i < 12; i++) {
        int id = i < 6 ? legs[i].hip : legs[i - 6].lift;
        unsigned char b[2];
        if (read_regs(id, REG_VOLTAGE, 2, b) != 2) { bad = 1; continue; }
        if (b[0] < vmin) vmin = b[0];
        if (b[0] > vmax) vmax = b[0];
        if (b[1] < tmin) tmin = b[1];
        if (b[1] > tmax) tmax = b[1];
    }
    if (bad) puts("not all 12 servos replied");
    else printf("servos %.1f..%.1f V, %d..%d C\n", vmin / 10.0, vmax / 10.0, tmin, tmax);
    flush_status();         /* now, not after the command: an overload matters before moving */
    return bad ? -1 : 0;
}

/* all feet down, hips centred, taking ms */
static int stand(int ms)
{
    if (check_servos()) return -1;
    for (int i = 0; i < 6; i++) legs[i].x = legs[i].z = 0;
    send_pose(ms);
    msleep(ms + 200);
    return 0;
}

/*
 * Each leg in turn (or just the one with hip id `only`): up, forward, back,
 * down (to its stance pose). Only the leg being tested moves.
 */
static void legtest(int only)
{
    double up = 700 - CENTRE + height;      /* lift at 700 (189 above centre, for lift_dir +1) */
    const double seq[4][2] = { { 0, up }, { 80, up }, { 0, up }, { 0, 0 } };   /* x, z */
    if (check_servos()) return;
    for (int i = 0; i < 6; i++) {
        struct leg *L = &legs[i];
        if (only && L->hip != only) continue;
        printf("%s (hip %d, lift %d): up, forward, back, down\n", leg_name(L), L->hip, L->lift);
        fflush(stdout);
        for (int k = 0; k < 4; k++) {
            L->x = seq[k][0]; L->z = seq[k][1];
            send_leg(L, 400);
            msleep(700);
        }
    }
}

/*
 * Even out the weight on the six feet, starting from the current offsets:
 * stand, then repeatedly read each lift's load (averaged, signed so + is the
 * foot pushing down), raise the feet of legs above the mean load and lower
 * those below, by 1-2 steps a round, until all are within CAL_TOL of the mean
 * or CAL_ROUNDS rounds have run. Keeps the round with the smallest spread.
 * Only the lift offsets change, each by at most CAL_MAX.
 */
#define CAL_TOL    15   /* load units (0.1%) either side of the mean */
#define CAL_ROUNDS 20
#define CAL_MAX    30   /* steps any offset may move from where it started */

static void calibrate(void)
{
    int start[6], best[6], bad = 0;
    double bestspread = 1e9;
    if (stand(1000)) return;
    printf("calibrating at height %d; lift loads in %%, + = pushing down\n", height);
    for (int i = 0; i < 6; i++) start[i] = best[i] = offset[legs[i].lift];
    for (int round = 1; round <= CAL_ROUNDS; round++) {
        msleep(500);
        double ld[6] = {0}, mean = 0;
        for (int s = 0; s < 5 && !bad; s++) {
            for (int i = 0; i < 6 && !bad; i++) {
                int v = 0;
                bad = read_u16(legs[i].lift, REG_PRESENT_LOAD, &v);     /* prints no reply */
                ld[i] += legs[i].lift_dir * (v & 0x400 ? -(v & 0x3FF) : v & 0x3FF) / 5.0;
            }
            msleep(40);
        }
        if (bad) break;
        for (int i = 0; i < 6; i++) mean += ld[i] / 6;
        double spread = 0;
        for (int i = 0; i < 6; i++) if (fabs(ld[i] - mean) > spread) spread = fabs(ld[i] - mean);
        printf("%2d:", round);
        for (int i = 0; i < 6; i++) printf(" %d:%+d %.1f", legs[i].lift, offset[legs[i].lift], ld[i] / 10);
        printf("  (mean %.1f, worst %.1f off)\n", mean / 10, spread / 10);
        fflush(stdout);
        if (mean < 20) { puts("hardly any load on the feet: is it standing on them?"); break; }
        if (spread < bestspread) {
            bestspread = spread;
            for (int i = 0; i < 6; i++) best[i] = offset[legs[i].lift];
        }
        if (spread <= CAL_TOL) { puts("balanced"); break; }
        if (round == CAL_ROUNDS) { puts("not balanced: keeping the best round"); break; }
        for (int i = 0; i < 6; i++) {
            double dev = ld[i] - mean;
            int d = lround(clampd(dev / 30, -2, 2));
            if (!d && fabs(dev) > CAL_TOL) d = dev > 0 ? 1 : -1;
            int *o = &offset[legs[i].lift];
            *o = lround(clampd(*o + legs[i].lift_dir * d, start[i] - CAL_MAX, start[i] + CAL_MAX));
        }
        send_pose(200);
    }
    for (int i = 0; i < 6; i++) offset[legs[i].lift] = best[i];
    send_pose(200);
    printf("offsets in use until exit; to keep them, put this in hector.c:\n");
    print_offsets();
}

/*
 * Twitch each id in turn, a second apart, to see which servo has which id:
 * +30 from wherever it is (up, for a lift with lift_dir +1) and back. Prints
 * what legs[] says the id is first.
 */
static void ident(const int *ids, int nid)
{
    for (int i = 0; i < nid; i++) {
        int id = ids[i], p;
        const char *what = "not in legs[]";
        char buf[16];
        for (int k = 0; k < 6; k++)
            if (legs[k].hip == id || legs[k].lift == id) {
                snprintf(buf, sizeof buf, "%s %s", leg_name(&legs[k]), legs[k].hip == id ? "hip" : "lift");
                what = buf;
            }
        if (read_u16(id, REG_PRESENT_POS, &p)) continue;    /* prints no reply */
        printf("id %d (%s) at %d%s\n", id, what, p, status_tail(id));
        fflush(stdout);
        unsigned char b[4];
        put16(b, p + 30); put16(b + 2, 150);
        write_regs(id, REG_GOAL_POS, b, 4);
        msleep(300);
        put16(b, p);
        write_regs(id, REG_GOAL_POS, b, 4);
        msleep(700);
    }
}

/*
 * Walk with the current gait. On the ground a leg's hip sweeps back at 2 amp per
 * stance time, the same rate for every foot down, so none slip. In its swing the
 * leg lifts (z = lift sin) and its hip follows a cubic from where it lifted off
 * to amp that starts and ends at that ground rate, so the foot doesn't scuff or
 * jolt as it lifts and lands (at speed; a slow walk barely needs it).
 * amp = stride -/+ turn (left/right), times a ramp r that rises over the first
 * cycle (so the legs that step last aren't dragged too far back) and falls over
 * one step at the end, after which each leg not already within 3 steps of
 * centre takes one more step, to centre.
 * A key (tty) or `cycles` cycles starts the stop; ctrl-c freezes where it is
 * (SIGINT on the Pi, a byte on the ATOM).
 * Each tick also reads one servo's load, round robin, for a peak load report;
 * the reads note any status errors (overload), which print after the command.
 */
static void walk(int cycles, int str, int turn)
{
    const struct gait *g = &gaits[gait];
    double period = step_ms / 1000.0 / g->swing, stance = period - step_ms / 1000.0;
    if (stand(1000)) return;

    /* a leg is on the ground (0), in the air (1), or on the ground but starting inside its swing window (2) */
    int state[6], settled[6] = {0}, nsettled = 0, tick = 0, peak[12] = {0};
    double x0[6];
    for (int i = 0; i < 6; i++) {
        double u = -g->off[legs[i].side][legs[i].row];
        u -= floor(u);
        state[i] = u > 0 && u < g->swing ? 2 : 0;
    }

    int tty = con_tty();
    if (tty) { con_raw(0); con_drop(); }
    sigint_catch();
    halted = 0;

    printf("walking: %s, %.1f s cycle, stride %d, turn %d, lift %d; %sctrl-c freezes\n",
           g->name, period, str, turn, lift, tty ? "any key stops, " : "");
    fflush(stdout);
    double phase = 0, r = 0, t = now();
    int stopping = 0;
    tick_start();
    while (nsettled < 6) {
        tick_wait(TICK_MS);
        int c = tty ? con_getc() : -1;
        if (c == 3) halted = 1;                 /* ctrl-c as a byte (the ATOM's console has no SIGINT) */
        if (halted) break;
        double t1 = now(), dt = t1 - t;
        t = t1;
        if (dt > 0.1) dt = 0.1;                 /* a stall: slow the gait, don't jump it */
        if (dt > 2 * TICK_MS / 1000.0) tick_start();    /* fell behind: don't burst to catch up */

        if (!stopping && (c >= 0 || (cycles && phase >= cycles))) {
            stopping = 1;
            puts("stopping");
            fflush(stdout);
        }
        r = stopping ? fmax(r - dt * 1000 / step_ms, 0) : fmin(r + dt / period, 1);
        phase += dt / period;

        for (int i = 0; i < 6; i++) {
            struct leg *L = &legs[i];
            double amp = clampd(r * (str + (L->side ? turn : -turn)), -HIP_MAX, HIP_MAX);
            double u = phase - g->off[L->side][L->row];
            u -= floor(u);
            if (r == 0 && state[i] != 1 && fabs(L->x) <= 3 && !settled[i]) { settled[i] = 1; nsettled++; }   /* down, near centre */
            if (u < g->swing) {
                if (state[i] == 0 && !settled[i]) { state[i] = 1; x0[i] = L->x; }
            } else state[i] = 0;
            double v = -2 * amp / stance;       /* hip rate on the ground, steps/s */
            if (state[i] == 1) {
                /* cubic from x0 to amp, leaving and arriving at the ground rate, so the foot is still over the ground as it lifts and lands */
                double w = u / g->swing, m = v * step_ms / 1000.0;
                L->x = x0[i] + (amp - x0[i]) * w * w * (3 - 2 * w) + m * w * (1 - w) * (1 - 2 * w);
                L->z = lift * sin(M_PI * w);
            } else {
                L->x += v * dt;
                L->z = 0;
            }
        }
        send_pose(TICK_MS);

        /* one servo's load per tick, for the peak load report */
        int id = tick % 12 < 6 ? legs[tick % 12].hip : legs[tick % 12 - 6].lift, ld;
        if (read_u16(id, REG_PRESENT_LOAD, &ld) == 0 && (ld & 0x3FF) > peak[tick % 12]) peak[tick % 12] = ld & 0x3FF;
        tick++;
    }

    sigint_restore();
    if (tty) { con_drop(); con_restore(); }    /* drop extra keys, not into the next line */
    if (halted) puts("\nhalted, holding this pose (stand puts all feet down)");
    else {
        for (int i = 0; i < 6; i++) legs[i].x = legs[i].z = 0;    /* tidy the last fraction of a step */
        send_pose(200);
        msleep(300);
        printf("stopped after %.1f cycles; ", phase);
        check_servos();
    }
    printf("peak load %% (a servo over 80%% for 4 s drops to 20%%):");
    for (int k = 0; k < 12; k++) printf(" %d:%d", k < 6 ? legs[k].hip : legs[k - 6].lift, (peak[k] + 5) / 10);
    putchar('\n');
}

/* run one tokenised command; returns 1 on quit */
static int run(char **tok, int nt)
{
    const char *c = tok[0];
    int ok = 1;

    if (!strcmp(c, "help") || !strcmp(c, "?")) help();
    else if (!strcmp(c, "quit") || !strcmp(c, "q") || !strcmp(c, "exit")) return 1;
    else if (!strcmp(c, "verbose")) verbose = arg(tok, 1, nt, 1, NULL);
    else if (!strcmp(c, "timeout")) {
        if (nt < 2) printf("timeout %d ms\n", timeout_ms);
        else timeout_ms = arg(tok, 1, nt, 0, NULL);
    }
    else if (!strcmp(c, "ping")) {
        if (nt < 2) printf("%d servo(s) found\n", ping_all());
        else {
            int id = arg(tok, 1, nt, 1, NULL);
            if (ping(id) < 0) printf("id %d: no reply\n", id);
        }
    }
    else if (!strcmp(c, "pos")) {
        int id = arg(tok, 1, nt, 1, NULL), v;
        if (read_u16(id, REG_PRESENT_POS, &v) == 0) printf("id %d pos %d%s\n", id, v, status_tail(id));
    }
    else if (!strcmp(c, "stat")) {
        int id = arg(tok, 1, nt, 1, NULL);
        unsigned char b[11], l[4];
        if (read_regs(id, REG_PRESENT_POS, 11, b) == 11) {
#define P(reg) (b + (reg) - REG_PRESENT_POS)
            printf("id %d pos %d speed %d load %d volt %.1fV temp %dC moving %d",
                   id, get16(P(REG_PRESENT_POS)), get16(P(REG_PRESENT_SPEED)), get16(P(REG_PRESENT_LOAD)),
                   *P(REG_VOLTAGE) / 10.0, *P(REG_TEMP), *P(REG_MOVING));
#undef P
            if (read_regs(id, REG_MIN_ANGLE, 4, l) != 4) printf("%s\n", status_tail(id));
            else if (!get16(l) && !get16(l + 2)) printf(" motor%s\n", status_tail(id));
            else printf(" servo %d..%d%s\n", get16(l), get16(l + 2), status_tail(id));
        }
    }
    else if (!strcmp(c, "move")) {             /* takes its own id list: one sync write */
        int ids[254], nid = nt > 1 ? parse_ids(tok[1], ids) : 0, pos = arg(tok, 2, nt, 512, &ok);
        int tm = arg(tok, 3, nt, 0, NULL), sp = arg(tok, 4, nt, 0, NULL);
        if (!ok || nid < 1) { puts("usage: move <id> <pos> [time] [speed]"); return 0; }
        if (pos < 0 || pos > 1023) printf("warning: pos %d outside 0..1023, servo will clamp to its limits\n", pos);
        int goals[254];
        for (int i = 0; i < nid; i++) goals[i] = pos + offset[ids[i]];
        if (nid == 1) {
            unsigned char b[6];
            put16(b, goals[0]); put16(b + REG_GOAL_TIME - REG_GOAL_POS, tm); put16(b + REG_GOAL_SPEED - REG_GOAL_POS, sp);
            if (write_regs(ids[0], REG_GOAL_POS, b, 6)) return 0;
        }
        else sync_move(ids, nid, goals, tm, sp);
        verify_move(ids, nid, goals, tm);
    }
    else if (!strcmp(c, "torque")) write_u8(arg(tok, 1, nt, 1, NULL), REG_TORQUE_ENABLE, arg(tok, 2, nt, 1, NULL));
    else if (!strcmp(c, "rb")) {
        int id = arg(tok, 1, nt, 1, &ok), a = arg(tok, 2, nt, 0, &ok); unsigned char b;
        if (!ok) { puts("usage: rb <id> <addr>"); return 0; }
        if (read_regs(id, a, 1, &b) == 1) printf("id %d [0x%02X] = %d (0x%02X)%s\n", id, a, b, b, status_tail(id));
    }
    else if (!strcmp(c, "rw")) {
        int id = arg(tok, 1, nt, 1, &ok), a = arg(tok, 2, nt, 0, &ok), v;
        if (!ok) { puts("usage: rw <id> <addr>"); return 0; }
        if (read_u16(id, a, &v) == 0) printf("id %d [0x%02X] = %d (0x%04X)%s\n", id, a, v, v, status_tail(id));
    }
    else if (!strcmp(c, "wb")) {
        int id = arg(tok, 1, nt, 1, &ok), a = arg(tok, 2, nt, 0, &ok), v = arg(tok, 3, nt, 0, &ok);
        if (!ok) { puts("usage: wb <id> <addr> <val>"); return 0; }
        write_u8(id, a, v);
    }
    else if (!strcmp(c, "ww")) {
        int id = arg(tok, 1, nt, 1, &ok), a = arg(tok, 2, nt, 0, &ok), v = arg(tok, 3, nt, 0, &ok);
        if (!ok) { puts("usage: ww <id> <addr> <val>"); return 0; }
        write_u16(id, a, v);
    }
    else if (!strcmp(c, "dump")) {
        int id = arg(tok, 1, nt, 1, NULL);
        unsigned char b[0x46];
        for (int a = 0; a < 0x46; a += 8) {
            int k = a + 8 > 0x46 ? 0x46 - a : 8;
            if (read_regs(id, a, k, b + a) != k) return 0;
        }
        printf("id %d:%s\n", id, status_tail(id));
        for (int a = 0; a < 0x46; a++) {
            if (a % 8 == 0) printf("%02X:", a);
            printf(" %02X", b[a]);
            if (a % 8 == 7 || a == 0x45) putchar('\n');
        }
    }
    else if (!strcmp(c, "setid")) {
        int o = arg(tok, 1, nt, 1, &ok), n = arg(tok, 2, nt, 1, &ok);
        if (!ok || n < 0 || n > 253) { puts("usage: setid <old> <new>   (new 0..253)"); return 0; }
        if (write_u8(o, REG_LOCK, 0)) return 0;
        if (write_u8(o, REG_ID, n)) return 0;
        write_u8(n, REG_LOCK, 1);
        if (ping(n) == 0) printf("id changed %d -> %d\n", o, n);
    }
    else if (!strcmp(c, "lock")) write_u8(arg(tok, 1, nt, 1, NULL), REG_LOCK, arg(tok, 2, nt, 1, NULL));
    else if (!strcmp(c, "limits")) {
        int id = arg(tok, 1, nt, 1, NULL), mn, mx;
        if (nt >= 4) {
            write_u8(id, REG_LOCK, 0);
            write_u16(id, REG_MIN_ANGLE, arg(tok, 2, nt, 0, NULL));
            write_u16(id, REG_MAX_ANGLE, arg(tok, 3, nt, 1023, NULL));
            write_u8(id, REG_LOCK, 1);
        }
        if (read_u16(id, REG_MIN_ANGLE, &mn) == 0 && read_u16(id, REG_MAX_ANGLE, &mx) == 0)
            printf("id %d limits %d..%d%s\n", id, mn, mx, status_tail(id));
    }
    else if (!strcmp(c, "motor")) {            /* SC "PWM/wheel" mode: both angle limits = 0 */
        int id = arg(tok, 1, nt, 1, NULL);
        unsigned char z[4] = {0, 0, 0, 0};
        write_u8(id, REG_LOCK, 0);
        write_regs(id, REG_MIN_ANGLE, z, 4);
        write_u8(id, REG_LOCK, 1);
        int m = 0; unsigned char mb;
        if (read_regs(id, 0x21, 1, &mb) == 1) m = mb;
        printf("id %d: motor mode (limits 0/0, mode reg 0x21 = %d). 'spin %d <-1000..1000>'\n", id, m, id);
    }
    else if (!strcmp(c, "spin")) {             /* speed via goal-time reg, bit 10 = reverse */
        int id = arg(tok, 1, nt, 1, &ok), sp = arg(tok, 2, nt, 0, &ok);
        if (!ok) { puts("usage: spin <id> <speed -1000..1000>"); return 0; }
        int v = sp < 0 ? (-sp & 0x3FF) | 0x400 : (sp & 0x3FF);
        write_u16(id, REG_GOAL_TIME, v);
    }
    else if (!strcmp(c, "servomode")) {        /* back to position mode: limits 20..1003 */
        int id = arg(tok, 1, nt, 1, NULL);
        write_u16(id, REG_GOAL_TIME, 0);
        write_u8(id, REG_LOCK, 0);
        write_u8(id, 0x21, 0);
        write_u16(id, REG_MIN_ANGLE, arg(tok, 2, nt, 20, NULL));
        write_u16(id, REG_MAX_ANGLE, arg(tok, 3, nt, 1003, NULL));
        write_u8(id, REG_LOCK, 1);
        int mn, mx;
        if (read_u16(id, REG_MIN_ANGLE, &mn) == 0 && read_u16(id, REG_MAX_ANGLE, &mx) == 0)
            printf("id %d: servo mode, limits %d..%d%s\n", id, mn, mx, status_tail(id));
    }
    else if (!strcmp(c, "stand")) stand(arg(tok, 1, nt, 1000, NULL));
    else if (!strcmp(c, "legtest")) {
        int only = arg(tok, 1, nt, 0, NULL);
        if (only < 0 || only > 6) { puts("usage: legtest [hip id 1-6]"); return 0; }
        legtest(only);
    }
    else if (!strcmp(c, "ident")) {
        int ids[254], nid = parse_ids(nt > 1 ? tok[1] : "1-12", ids);
        if (nid < 1) { puts("usage: ident [id]"); return 0; }
        ident(ids, nid);
    }
    else if (!strcmp(c, "calibrate")) calibrate();
    else if (!strcmp(c, "offset")) {
        int ids[254], nid = nt > 1 ? parse_ids(tok[1], ids) : 0, v = arg(tok, 2, nt, 0, &ok);
        if (nt > 1 && (nid < 1 || !ok)) { puts("usage: offset [id val]"); return 0; }
        for (int i = 0; i < nid; i++) offset[ids[i]] = v;
        print_offsets();
    }
    else if (!strcmp(c, "walk")) {
        int cycles = arg(tok, 1, nt, 0, NULL), str = arg(tok, 2, nt, stride, NULL), turn = arg(tok, 3, nt, 0, NULL);
        if (cycles < 0 || abs(str) > HIP_MAX || abs(turn) > HIP_MAX) {
            printf("usage: walk [cycles] [stride -%d..%d] [turn -%d..%d]\n", HIP_MAX, HIP_MAX, HIP_MAX, HIP_MAX);
            return 0;
        }
        walk(cycles, str, turn);
    }
    else if (!strcmp(c, "set")) {
        const struct param *p = NULL;
        for (int i = 0; nt > 1 && i < (int)(sizeof params / sizeof *params); i++)
            if (!strcmp(tok[1], params[i].name)) p = &params[i];
        if (nt == 1) {
            for (int i = 0; i < (int)(sizeof params / sizeof *params); i++) {
                const struct param *q = &params[i];
                char v[16];
                if (q->v == &gait) snprintf(v, sizeof v, "%s", gaits[gait].name);
                else snprintf(v, sizeof v, "%d", *q->v);
                printf("%-7s %-7s %s\n", q->name, v, q->what);
            }
            printf("cycle %.1f s\n", step_ms / 1000.0 / gaits[gait].swing);
        } else if (!p || nt != 3) puts("usage: set [name value] (set alone lists them)");
        else {
            int v = -1;
            if (p->v == &gait) {
                for (int i = 0; i < 3; i++) if (!strcmp(tok[2], gaits[i].name)) v = i;
            } else {
                char *e;
                v = strtol(tok[2], &e, 0);
                if (*e) v = p->min - 1;
            }
            if (v < p->min || v > p->max) {
                if (p->v == &gait) puts("gait: wave, ripple or tripod");
                else printf("%s: %d..%d\n", p->name, p->min, p->max);
            } else *p->v = v;
        }
    }
    else if (!strcmp(c, "raw")) {
        unsigned char b[64]; int n = 0;
        for (int i = 1; i < nt && n < 64; i++) b[n++] = strtol(tok[i], NULL, 16);
        bus_flush();
        bus_write(b, n);
        hexdump("tx:", b, n);
        unsigned char r[64]; int got = read_bytes(r, sizeof r, timeout_ms);
        hexdump("rx:", r, got);
    }
    else printf("unknown command '%s' (help)\n", c);
    return 0;
}

/* commands whose first argument is an <id> that may be a list; run once per id */
static const char *per_id_cmds[] = {
    "ping", "pos", "stat", "torque", "rb", "rw", "wb", "ww", "dump",
    "lock", "limits", "motor", "spin", "servomode", NULL
};

static int dispatch(char **tok, int nt)
{
    int per_id = 0;
    for (int i = 0; per_id_cmds[i]; i++) if (!strcmp(tok[0], per_id_cmds[i])) per_id = 1;
    if (!per_id || (nt < 2 && !strcmp(tok[0], "ping"))) {
        int q = run(tok, nt);
        flush_status();
        return q;
    }
    if (nt < 2) { printf("%s: missing <id> (help)\n", tok[0]); return 0; }

    int ids[254], nid = parse_ids(tok[1], ids);
    if (nid < 0) { printf("bad id list '%s' (e.g. 3, 1-12, 2,6, 1-6,9)\n", tok[1]); return 0; }
    char idbuf[8], *t[16];
    memcpy(t, tok, nt * sizeof *t);
    t[1] = idbuf;
    for (int i = 0; i < nid; i++) {
        snprintf(idbuf, sizeof idbuf, "%d", ids[i]);
        run(t, nt);
        flush_status();
    }
    return 0;
}

/* minimal line editor for a tty: arrows, history, home/end, backspace */
#define NHIST 100
static char *hist[NHIST];
static int nhist;

static void redraw(const char *buf, int len, int cur)
{
    printf("\r> %.*s\x1b[K", len, buf);
    if (len > cur) printf("\x1b[%dD", len - cur);
    fflush(stdout);
}

static void load(char *buf, int size, const char *s, int *len, int *cur)
{
    snprintf(buf, size, "%s", s);
    *len = *cur = strlen(buf);
}

/* returns 0 on EOF (ctrl-d on an empty line) */
static int edit_line(char *buf, int size)
{
    static int prev;        /* a terminal may send CR LF for enter (idf.py monitor does): one line, not two */
    con_raw(1);

    char draft[1024] = "";  /* the line being typed, kept while browsing history */
    int len = 0, cur = 0, h = nhist, ok = 1;
    buf[0] = 0;
    redraw(buf, len, cur);
    for (;;) {
        int c = con_getc(), lf = c == '\n' && prev == '\r';
        prev = c;
        if (c < 0) { ok = 0; break; }
        if (lf) continue;
        if (c == '\r' || c == '\n') break;
        if (c == 4 && !len) { ok = 0; break; }                     /* ctrl-d */
        if (c == 127 || c == 8) {                                  /* backspace */
            if (cur) { memmove(buf + cur - 1, buf + cur, len - cur); len--; cur--; }
        } else if (c == 1) cur = 0;                                /* ctrl-a */
        else if (c == 5) cur = len;                                /* ctrl-e */
        else if (c == 21) len = cur = 0;                           /* ctrl-u */
        else if (c == 27) {                                        /* escape sequence */
            int e = con_getc();
            if (e != '[' && e != 'O') continue;
            int k = con_getc();
            if (k >= '0' && k <= '9') {                            /* ESC [ n ~ */
                if (con_getc() != '~') continue;
                k = (k == '1' || k == '7') ? 'H' : (k == '4' || k == '8') ? 'F' : (k == '3') ? 'X' : 0;
            }
            if (k == 'A' || k == 'B') {                            /* up / down: history */
                if (h == nhist) { buf[len] = 0; snprintf(draft, sizeof draft, "%s", buf); }
                if (k == 'A' && h > 0) h--;
                else if (k == 'B' && h < nhist) h++;
                else continue;
                load(buf, size, h < nhist ? hist[h] : draft, &len, &cur);
            }
            else if (k == 'C' && cur < len) cur++;
            else if (k == 'D' && cur > 0) cur--;
            else if (k == 'H') cur = 0;
            else if (k == 'F') cur = len;
            else if (k == 'X' && cur < len) { memmove(buf + cur, buf + cur + 1, len - cur - 1); len--; }
        } else if (c >= 32 && len < size - 1) {                    /* insert */
            memmove(buf + cur + 1, buf + cur, len - cur);
            buf[cur++] = c; len++;
        }
        redraw(buf, len, cur);
    }
    buf[len] = 0;
    putchar('\n');
    con_restore();

    if (ok && len && (!nhist || strcmp(hist[nhist - 1], buf))) {
        if (nhist == NHIST) { free(hist[0]); memmove(hist, hist + 1, (NHIST - 1) * sizeof *hist); nhist--; }
        hist[nhist++] = strdup(buf);
    }
    return ok;
}

/* read and run lines until quit or EOF */
static void repl(void)
{
    char line[1024];
    int interactive = con_tty(), done = 0;
    while (!done) {
        if (interactive ? !edit_line(line, sizeof line) : !fgets(line, sizeof line, stdin)) break;
        /* ';' separates commands on one line; run them in order */
        for (char *cmd = line, *next; cmd && !done; cmd = next) {
            if ((next = strchr(cmd, ';'))) *next++ = 0;
            char *tok[16]; int nt = 0;
            for (char *s = strtok(cmd, " \t\r\n"); s && nt < 16; s = strtok(NULL, " \t\r\n")) tok[nt++] = s;
            if (nt) done = dispatch(tok, nt);
        }
    }
}

#ifdef ESP_PLATFORM
void app_main(void)
{
    con_init();
    open_port(NULL, 1000000);
    for (;;) {
        printf("hector on the ATOM: bus on UART1 (TX G38, RX G39) @ 1000000 (type help)\n");
        repl();
        puts("nothing to quit to on the ATOM");
    }
}
#else
int main(int argc, char **argv)
{
    const char *dev = argc > 1 ? argv[1] : "/dev/ttyACM0";
    int baud = argc > 2 ? atoi(argv[2]) : 1000000;
    con_init();
    open_port(dev, baud);
    printf("opened %s @ %d (type help)\n", dev, baud);
    repl();
    close(fd);
    return 0;
}
#endif
