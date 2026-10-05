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
#include "driver/i2c_master.h"
#include "driver/spi_master.h"
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
 * (con_*), timing (msleep, tick_*), the screen (scr_*) and the IMU (imu_*):
 * the last two are the ATOM's, stubs on the Pi. Everything else is the same on the Pi and the ATOM.
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

/*
 * The screen: 0.85" 128x128 on SPI (MOSI G21, SCLK G15, CS G14, DC G42, RST G48).
 * The panel is a GC9107, or an ST7735S on some batches, told apart by RDDID as
 * M5GFX does (init sequences and offsets are from M5GFX too). The backlight is
 * an LP5562 LED driver on the internal I2C (SDA G45, SCL G0), which the IMU shares.
 * Drawing goes into fb[], and scr_show() sends all of it (32 KB, about 7 ms).
 */
#define LCD_HOST SPI3_HOST
#define LCD_MOSI 21
#define LCD_SCLK 15
#define LCD_CS   14
#define LCD_DC   42
#define LCD_RST  48
#define SYS_SDA  45     /* the internal I2C: backlight and IMU */
#define SYS_SCL  0
#define BL_ADDR  0x30   /* LP5562 */
#define BL_LEVEL 160    /* 0..255 */

static spi_device_handle_t lcd;
static int lcd_x0, lcd_y0;              /* where the 128x128 window starts in the controller's memory */
static uint16_t fb[128 * 128];          /* RGB565, byte-swapped: sent high byte first */

/*
 * Init lists: command, number of parameters (| 0x80 if a delay follows),
 * parameters, then the delay in ms (255 = 500); 0xFF ends the list.
 */
static const unsigned char gc9107_init[] = {
    0xFE, 0x80, 5,
    0xEF, 0x80, 5,
    0xB0, 1, 0xC0,  0xB2, 1, 0x2F,  0xB3, 1, 0x03,  0xB6, 1, 0x19,  0xB7, 1, 0x01,
    0xAC, 1, 0xCB,  0xAB, 1, 0x0E,  0xB4, 1, 0x04,  0xA8, 1, 0x19,  0xB8, 1, 0x08,
    0xE8, 1, 0x24,  0xE9, 1, 0x48,  0xEA, 1, 0x22,  0xC6, 1, 0x30,  0xC7, 1, 0x18,
    0xF0, 14, 0x01, 0x2B, 0x23, 0x3C, 0xB7, 0x12, 0x17, 0x60, 0x00, 0x06, 0x0C, 0x17, 0x12, 0x1F,
    0xF1, 14, 0x05, 0x2E, 0x2D, 0x44, 0xD6, 0x15, 0x17, 0xA0, 0x02, 0x0D, 0x0D, 0x1A, 0x18, 0x1F,
    0x11, 0x80, 120,                    /* sleep out */
    0x29, 0,                            /* display on */
    0x3A, 1, 0x55,                      /* 16-bit colour */
    0x36, 1, 0x08,                      /* MADCTL: BGR */
    0x20, 0,                            /* inversion off */
    0xFF,
};
static const unsigned char st7735s_init[] = {
    0x01, 0x80, 150,                    /* software reset */
    0x11, 0x80, 255,                    /* sleep out */
    0xB1, 3, 0x01, 0x2C, 0x2D,
    0xB2, 3, 0x01, 0x2C, 0x2D,
    0xB3, 6, 0x01, 0x2C, 0x2D, 0x01, 0x2C, 0x2D,
    0xB4, 1, 0x07,
    0xC0, 3, 0xA2, 0x02, 0x84,  0xC1, 1, 0xC5,  0xC2, 2, 0x0A, 0x00,  0xC3, 2, 0x8A, 0x2A,
    0xC4, 2, 0x8A, 0xEE,  0xC5, 1, 0x0E,
    0xE0, 16, 0x02, 0x1C, 0x07, 0x12, 0x37, 0x32, 0x29, 0x2D, 0x29, 0x25, 0x2B, 0x39, 0x00, 0x01, 0x03, 0x10,
    0xE1, 16, 0x03, 0x1D, 0x07, 0x06, 0x2E, 0x2C, 0x29, 0x2D, 0x2E, 0x2E, 0x37, 0x3F, 0x00, 0x00, 0x02, 0x10,
    0x13, 0x80, 10,                     /* normal display on */
    0x29, 0x80, 100,                    /* display on */
    0x3A, 1, 0x55,                      /* 16-bit colour */
    0x36, 1, 0xDC,                      /* MADCTL: rotated 180, BGR */
    0x21, 0,                            /* inversion on */
    0xFF,
};

/* the panel's id (RDDID, 3-wire: one dummy bit, then 32 bits back on MOSI) */
static uint32_t lcd_id(int hz)
{
    spi_device_interface_config_t d = {
        .command_bits = 8, .dummy_bits = 1, .clock_speed_hz = hz, .spics_io_num = LCD_CS,
        .queue_size = 1, .flags = SPI_DEVICE_3WIRE | SPI_DEVICE_HALFDUPLEX,
    };
    spi_device_handle_t h;
    if (spi_bus_add_device(LCD_HOST, &d, &h) != ESP_OK) return 0;
    spi_transaction_t t = { .cmd = 0x04, .rxlength = 32, .flags = SPI_TRANS_USE_RXDATA };
    uint32_t id = 0;
    gpio_set_level(LCD_DC, 0);
    if (spi_device_polling_transmit(h, &t) == ESP_OK)
        id = t.rx_data[0] | t.rx_data[1] << 8 | t.rx_data[2] << 16 | (uint32_t)t.rx_data[3] << 24;
    spi_bus_remove_device(h);
    return id;
}

/* dc 0 = command, 1 = data */
static void lcd_send(int dc, const void *p, int n)
{
    spi_transaction_t t = { .length = n * 8, .tx_buffer = p };
    gpio_set_level(LCD_DC, dc);
    spi_device_polling_transmit(lcd, &t);
}

static void lcd_cmds(const unsigned char *c)
{
    while (*c != 0xFF) {
        unsigned char cmd = *c++, n = *c++, par[16];
        lcd_send(0, &cmd, 1);
        memcpy(par, c, n & 0x7F);           /* via RAM: DMA can't read flash */
        if (n & 0x7F) lcd_send(1, par, n & 0x7F);
        c += n & 0x7F;
        if (n & 0x80) { int ms = *c++; msleep(ms == 255 ? 500 : ms); }
    }
}

/*
 * 5x7 font, ' ' to '~', a byte per column, bit 0 at the top. From Adafruit GFX's
 * glcdfont.c: Copyright (c) 2012 Adafruit Industries. All rights reserved.
 * BSD licence: redistributions must keep this notice. THIS SOFTWARE IS PROVIDED
 * BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED
 * WARRANTIES ARE DISCLAIMED (full text in Adafruit-GFX-Library's glcdfont.c).
 */
static const unsigned char font5x7[95][5] = {
    { 0x00, 0x00, 0x00, 0x00, 0x00 },  /* ' ' */
    { 0x00, 0x00, 0x5F, 0x00, 0x00 },  /* '!' */
    { 0x00, 0x07, 0x00, 0x07, 0x00 },  /* '"' */
    { 0x14, 0x7F, 0x14, 0x7F, 0x14 },  /* '#' */
    { 0x24, 0x2A, 0x7F, 0x2A, 0x12 },  /* '$' */
    { 0x23, 0x13, 0x08, 0x64, 0x62 },  /* '%' */
    { 0x36, 0x49, 0x56, 0x20, 0x50 },  /* '&' */
    { 0x00, 0x08, 0x07, 0x03, 0x00 },  /* "'" */
    { 0x00, 0x1C, 0x22, 0x41, 0x00 },  /* '(' */
    { 0x00, 0x41, 0x22, 0x1C, 0x00 },  /* ')' */
    { 0x2A, 0x1C, 0x7F, 0x1C, 0x2A },  /* '*' */
    { 0x08, 0x08, 0x3E, 0x08, 0x08 },  /* '+' */
    { 0x00, 0x80, 0x70, 0x30, 0x00 },  /* ',' */
    { 0x08, 0x08, 0x08, 0x08, 0x08 },  /* '-' */
    { 0x00, 0x00, 0x60, 0x60, 0x00 },  /* '.' */
    { 0x20, 0x10, 0x08, 0x04, 0x02 },  /* '/' */
    { 0x3E, 0x51, 0x49, 0x45, 0x3E },  /* '0' */
    { 0x00, 0x42, 0x7F, 0x40, 0x00 },  /* '1' */
    { 0x72, 0x49, 0x49, 0x49, 0x46 },  /* '2' */
    { 0x21, 0x41, 0x49, 0x4D, 0x33 },  /* '3' */
    { 0x18, 0x14, 0x12, 0x7F, 0x10 },  /* '4' */
    { 0x27, 0x45, 0x45, 0x45, 0x39 },  /* '5' */
    { 0x3C, 0x4A, 0x49, 0x49, 0x31 },  /* '6' */
    { 0x41, 0x21, 0x11, 0x09, 0x07 },  /* '7' */
    { 0x36, 0x49, 0x49, 0x49, 0x36 },  /* '8' */
    { 0x46, 0x49, 0x49, 0x29, 0x1E },  /* '9' */
    { 0x00, 0x00, 0x14, 0x00, 0x00 },  /* ':' */
    { 0x00, 0x40, 0x34, 0x00, 0x00 },  /* ';' */
    { 0x00, 0x08, 0x14, 0x22, 0x41 },  /* '<' */
    { 0x14, 0x14, 0x14, 0x14, 0x14 },  /* '=' */
    { 0x00, 0x41, 0x22, 0x14, 0x08 },  /* '>' */
    { 0x02, 0x01, 0x59, 0x09, 0x06 },  /* '?' */
    { 0x3E, 0x41, 0x5D, 0x59, 0x4E },  /* '@' */
    { 0x7C, 0x12, 0x11, 0x12, 0x7C },  /* 'A' */
    { 0x7F, 0x49, 0x49, 0x49, 0x36 },  /* 'B' */
    { 0x3E, 0x41, 0x41, 0x41, 0x22 },  /* 'C' */
    { 0x7F, 0x41, 0x41, 0x41, 0x3E },  /* 'D' */
    { 0x7F, 0x49, 0x49, 0x49, 0x41 },  /* 'E' */
    { 0x7F, 0x09, 0x09, 0x09, 0x01 },  /* 'F' */
    { 0x3E, 0x41, 0x41, 0x51, 0x73 },  /* 'G' */
    { 0x7F, 0x08, 0x08, 0x08, 0x7F },  /* 'H' */
    { 0x00, 0x41, 0x7F, 0x41, 0x00 },  /* 'I' */
    { 0x20, 0x40, 0x41, 0x3F, 0x01 },  /* 'J' */
    { 0x7F, 0x08, 0x14, 0x22, 0x41 },  /* 'K' */
    { 0x7F, 0x40, 0x40, 0x40, 0x40 },  /* 'L' */
    { 0x7F, 0x02, 0x1C, 0x02, 0x7F },  /* 'M' */
    { 0x7F, 0x04, 0x08, 0x10, 0x7F },  /* 'N' */
    { 0x3E, 0x41, 0x41, 0x41, 0x3E },  /* 'O' */
    { 0x7F, 0x09, 0x09, 0x09, 0x06 },  /* 'P' */
    { 0x3E, 0x41, 0x51, 0x21, 0x5E },  /* 'Q' */
    { 0x7F, 0x09, 0x19, 0x29, 0x46 },  /* 'R' */
    { 0x26, 0x49, 0x49, 0x49, 0x32 },  /* 'S' */
    { 0x03, 0x01, 0x7F, 0x01, 0x03 },  /* 'T' */
    { 0x3F, 0x40, 0x40, 0x40, 0x3F },  /* 'U' */
    { 0x1F, 0x20, 0x40, 0x20, 0x1F },  /* 'V' */
    { 0x3F, 0x40, 0x38, 0x40, 0x3F },  /* 'W' */
    { 0x63, 0x14, 0x08, 0x14, 0x63 },  /* 'X' */
    { 0x03, 0x04, 0x78, 0x04, 0x03 },  /* 'Y' */
    { 0x61, 0x59, 0x49, 0x4D, 0x43 },  /* 'Z' */
    { 0x00, 0x7F, 0x41, 0x41, 0x41 },  /* '[' */
    { 0x02, 0x04, 0x08, 0x10, 0x20 },  /* '\' */
    { 0x00, 0x41, 0x41, 0x41, 0x7F },  /* ']' */
    { 0x04, 0x02, 0x01, 0x02, 0x04 },  /* '^' */
    { 0x40, 0x40, 0x40, 0x40, 0x40 },  /* '_' */
    { 0x00, 0x03, 0x07, 0x08, 0x00 },  /* '`' */
    { 0x20, 0x54, 0x54, 0x78, 0x40 },  /* 'a' */
    { 0x7F, 0x28, 0x44, 0x44, 0x38 },  /* 'b' */
    { 0x38, 0x44, 0x44, 0x44, 0x28 },  /* 'c' */
    { 0x38, 0x44, 0x44, 0x28, 0x7F },  /* 'd' */
    { 0x38, 0x54, 0x54, 0x54, 0x18 },  /* 'e' */
    { 0x00, 0x08, 0x7E, 0x09, 0x02 },  /* 'f' */
    { 0x18, 0xA4, 0xA4, 0x9C, 0x78 },  /* 'g' */
    { 0x7F, 0x08, 0x04, 0x04, 0x78 },  /* 'h' */
    { 0x00, 0x44, 0x7D, 0x40, 0x00 },  /* 'i' */
    { 0x20, 0x40, 0x40, 0x3D, 0x00 },  /* 'j' */
    { 0x7F, 0x10, 0x28, 0x44, 0x00 },  /* 'k' */
    { 0x00, 0x41, 0x7F, 0x40, 0x00 },  /* 'l' */
    { 0x7C, 0x04, 0x78, 0x04, 0x78 },  /* 'm' */
    { 0x7C, 0x08, 0x04, 0x04, 0x78 },  /* 'n' */
    { 0x38, 0x44, 0x44, 0x44, 0x38 },  /* 'o' */
    { 0xFC, 0x18, 0x24, 0x24, 0x18 },  /* 'p' */
    { 0x18, 0x24, 0x24, 0x18, 0xFC },  /* 'q' */
    { 0x7C, 0x08, 0x04, 0x04, 0x08 },  /* 'r' */
    { 0x48, 0x54, 0x54, 0x54, 0x24 },  /* 's' */
    { 0x04, 0x04, 0x3F, 0x44, 0x24 },  /* 't' */
    { 0x3C, 0x40, 0x40, 0x20, 0x7C },  /* 'u' */
    { 0x1C, 0x20, 0x40, 0x20, 0x1C },  /* 'v' */
    { 0x3C, 0x40, 0x30, 0x40, 0x3C },  /* 'w' */
    { 0x44, 0x28, 0x10, 0x28, 0x44 },  /* 'x' */
    { 0x4C, 0x90, 0x90, 0x90, 0x7C },  /* 'y' */
    { 0x44, 0x64, 0x54, 0x4C, 0x44 },  /* 'z' */
    { 0x00, 0x08, 0x36, 0x41, 0x00 },  /* '{' */
    { 0x00, 0x00, 0x77, 0x00, 0x00 },  /* '|' */
    { 0x00, 0x41, 0x36, 0x08, 0x00 },  /* '}' */
    { 0x02, 0x01, 0x02, 0x04, 0x02 },  /* '~' */
};

static void scr_clear(void) { memset(fb, 0, sizeof fb); }

/* s at pixel (x, y), each font pixel scale x scale; characters are 6 x 8 font pixels */
static void scr_text(int x, int y, int scale, int colour, const char *s)
{
    uint16_t c = (colour >> 8 & 0xFF) | (colour & 0xFF) << 8;
    for (; *s; s++, x += 6 * scale) {
        int ch = (unsigned char)*s;
        const unsigned char *g = font5x7[ch < 0x20 || ch > 0x7E ? '?' - 0x20 : ch - 0x20];
        for (int col = 0; col < 5; col++)
            for (int row = 0; row < 8; row++)
                if (g[col] >> row & 1)
                    for (int dy = 0; dy < scale; dy++)
                        for (int dx = 0; dx < scale; dx++) {
                            int px = x + col * scale + dx, py = y + row * scale + dy;
                            if (px >= 0 && px < 128 && py >= 0 && py < 128) fb[py * 128 + px] = c;
                        }
    }
}

/* send fb to the panel */
static void scr_show(void)
{
    unsigned char cmd, w[4] = { 0, lcd_x0, 0, lcd_x0 + 127 };
    cmd = 0x2A; lcd_send(0, &cmd, 1); lcd_send(1, w, 4);           /* CASET */
    w[1] = lcd_y0; w[3] = lcd_y0 + 127;
    cmd = 0x2B; lcd_send(0, &cmd, 1); lcd_send(1, w, 4);           /* RASET */
    cmd = 0x2C; lcd_send(0, &cmd, 1); lcd_send(1, fb, sizeof fb);  /* RAMWR */
}

/* a device on the internal I2C (SDA G45, SCL G0: the backlight and the IMU), setting the bus up first time */
static int sys_i2c_dev(int addr, i2c_master_dev_handle_t *d)
{
    static i2c_master_bus_handle_t bus;
    i2c_master_bus_config_t b = {
        .i2c_port = -1, .sda_io_num = SYS_SDA, .scl_io_num = SYS_SCL, .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7, .flags.enable_internal_pullup = 1,
    };
    i2c_device_config_t c = { .dev_addr_length = I2C_ADDR_BIT_LEN_7, .device_address = addr, .scl_speed_hz = 400000 };
    if (!bus && i2c_new_master_bus(&b, &bus) != ESP_OK) { bus = NULL; return -1; }
    return i2c_master_bus_add_device(bus, &c, d) == ESP_OK ? 0 : -1;
}

/* LP5562: enable, internal clock, all channels from their PWM registers, W (the backlight) to level */
static int bl_init(int level)
{
    i2c_master_dev_handle_t d;
    if (sys_i2c_dev(BL_ADDR, &d)) return -1;
    unsigned char w[][2] = { { 0x00, 0x40 }, { 0x08, 0x01 }, { 0x70, 0x00 }, { 0x0E, level } };
    for (int i = 0; i < 4; i++) {
        if (i2c_master_transmit(d, w[i], 2, 50) != ESP_OK) return -1;
        if (i == 0) msleep(1);
    }
    return 0;
}

/* set up the screen and clear it; what it is, or NULL if there isn't one. *ok = 0 if the backlight didn't answer */
static const char *scr_init(int *ok)
{
    static char what[48];
    spi_bus_config_t b = {
        .mosi_io_num = LCD_MOSI, .miso_io_num = -1, .sclk_io_num = LCD_SCLK,
        .quadwp_io_num = -1, .quadhd_io_num = -1, .max_transfer_sz = sizeof fb,
    };
    if (spi_bus_initialize(LCD_HOST, &b, SPI_DMA_CH_AUTO) != ESP_OK) return NULL;
    gpio_reset_pin(LCD_DC);
    gpio_set_direction(LCD_DC, GPIO_MODE_OUTPUT);
    gpio_reset_pin(LCD_RST);
    gpio_set_direction(LCD_RST, GPIO_MODE_OUTPUT);
    gpio_set_level(LCD_RST, 0);
    msleep(10);
    gpio_set_level(LCD_RST, 1);
    msleep(120);

    /* some GC9107s only answer slowly; an ST7735S answers at the first speed */
    uint32_t id = lcd_id(8000000);
    int st = (id & 0xFFFF) == 0x7683 || (id & 0xFFFF) == 0x897C, gc = (id & 0xFFFFFF) == 0x079100;
    if (!st && !gc) {
        id = lcd_id(100000);
        gc = (id & 0xFFFFFF) == 0x079100;
    }
    spi_device_interface_config_t d = { .clock_speed_hz = 40000000, .spics_io_num = LCD_CS, .queue_size = 1 };
    if (spi_bus_add_device(LCD_HOST, &d, &lcd) != ESP_OK) return NULL;
    lcd_cmds(st ? st7735s_init : gc9107_init);
    lcd_x0 = st ? 2 : 0;
    lcd_y0 = st ? 3 : 32;
    snprintf(what, sizeof what, "%s (id %06lx%s)", st ? "ST7735S" : "GC9107",
             (unsigned long)(id & 0xFFFFFF), st || gc ? "" : ", not recognised");
    scr_clear();
    scr_show();
    *ok = bl_init(BL_LEVEL) == 0 && (st || gc);
    return what;
}

/*
 * The IMU: a Bosch BMI270 (accelerometer and gyro) at 0x68 on the internal
 * I2C. It needs Bosch's 8 KB configuration file uploaded at every start-up.
 * Accelerometer +-4 g at 100 Hz, gyro +-500 deg/s at 200 Hz. As the ATOM is
 * mounted, the chip's axes are the robot's: +x forward, +y left, +z up
 * (found by tipping it up at each edge).
 */
#define IMU_ADDR 0x68
#include "atom/bmi270_config.h"

static i2c_master_dev_handle_t imu;

static int imu_wr(int reg, int v)
{
    unsigned char w[2] = { reg, v };
    return i2c_master_transmit(imu, w, 2, 50) == ESP_OK ? 0 : -1;
}

static int imu_rd(int reg, unsigned char *b, int n)
{
    unsigned char r = reg;
    return i2c_master_transmit_receive(imu, &r, 1, b, n, 50) == ESP_OK ? 0 : -1;
}

/* 0 if it's there and its config loaded, -1 if not (1 = no IMU on this host, the Pi's stub) */
static int imu_init(void)
{
    unsigned char id = 0, st = 0, buf[1 + 256];
    if (sys_i2c_dev(IMU_ADDR, &imu) || imu_rd(0x00, &id, 1) || id != 0x24) return -1;     /* CHIP_ID */
    imu_wr(0x7E, 0xB6);                         /* CMD: soft reset (needs 2 ms) */
    msleep(5);
    imu_wr(0x7C, 0x00);                         /* PWR_CONF: advanced power save off, for the upload (450 us) */
    msleep(5);
    imu_wr(0x59, 0x00);                         /* INIT_CTRL: start of config load */
    for (int i = 0; i < (int)sizeof bmi270_config_file; i += 256) {
        unsigned char a[3] = { 0x5B, (i / 2) & 0x0F, (i / 2) >> 4 };       /* INIT_ADDR_0/1, in words */
        buf[0] = 0x5E;                                                      /* INIT_DATA */
        memcpy(buf + 1, bmi270_config_file + i, 256);
        if (i2c_master_transmit(imu, a, 3, 50) != ESP_OK || i2c_master_transmit(imu, buf, sizeof buf, 100) != ESP_OK) return -1;
    }
    imu_wr(0x59, 0x01);                         /* INIT_CTRL: config load done */
    for (int i = 0; i < 20 && (st & 0x0F) != 1; i++) { msleep(5); imu_rd(0x21, &st, 1); }  /* INTERNAL_STATUS: 1 = init ok */
    if ((st & 0x0F) != 1) return -1;
    imu_wr(0x7D, 0x0E);                         /* PWR_CTRL: accelerometer, gyro, temperature on */
    imu_wr(0x40, 0xA8);                         /* ACC_CONF: 100 Hz, normal filter, performance mode */
    imu_wr(0x41, 0x01);                         /* ACC_RANGE: +-4 g */
    imu_wr(0x42, 0xA9);                         /* GYR_CONF: 200 Hz, normal filter, performance mode */
    imu_wr(0x43, 0x02);                         /* GYR_RANGE: +-500 deg/s */
    imu_wr(0x7C, 0x02);                         /* PWR_CONF: FIFO self wake-up, no power save */
    msleep(50);
    return 0;
}

/* acceleration in g (+1 on z standing level) and rotation in deg/s: x forward, y left, z up */
static int imu_read(double acc[3], double gyr[3])
{
    unsigned char b[12];
    if (!imu || imu_rd(0x0C, b, 12)) return -1;  /* ACC x y z, GYR x y z, little-endian */
    for (int i = 0; i < 3; i++) {
        acc[i] = (int16_t)(b[2 * i] | b[2 * i + 1] << 8) * 4.0 / 32768;
        gyr[i] = (int16_t)(b[6 + 2 * i] | b[7 + 2 * i] << 8) * 500.0 / 32768;
    }
    return 0;
}

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

/* no screen on the Pi */
static const char *scr_init(int *ok) { *ok = 0; return NULL; }
static void scr_clear(void) {}
static void scr_text(int x, int y, int scale, int colour, const char *s) { (void)x; (void)y; (void)scale; (void)colour; (void)s; }
static void scr_show(void) {}

/* nor an IMU */
static int imu_init(void) { return 1; }
static int imu_read(double acc[3], double gyr[3]) { (void)acc; (void)gyr; return -1; }

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
    "set [name value]          list or set walk parameters: gait step stride lift height\n"
    "loads [secs]              stream every servo's load and position error (default 10 s, or until a key);\n"
    "                          reads only: stand first to see what pushing on it does\n"
    "imu [secs]                stream the ATOM's IMU: acceleration, rotation and tilt (default 10 s, or until a key)\n"
    "bow [secs]                stand; tip it up at an edge and put it down, and it bows towards that edge\n"
    "                          and back up (needs the ATOM's IMU); until a key, or secs seconds\n"
    "selftest                  the startup checks again (reads only, nothing moves): servos, battery,\n"
    "                          temperature, errors, overload settings, positions, load; on the ATOM's screen too");
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

/*
 * The self test, at startup and by the selftest command. It only reads, so
 * nothing moves. Each check logs a line on the console and a short one (10
 * characters) on the ATOM's screen as it goes; then the screen says Hi!, in
 * green, yellow if there were warnings, red if anything failed, with the
 * problems under it.
 */
#define REG_UNLOAD        0x13  /* unloading conditions: bit 5 = overload protection */
#define REG_PROT_TORQUE   0x25  /* then 0x26 protection time (40 ms units), 0x27 overload torque (%) */
#define BATT_LOW  70            /* 2S, 0.1 V */
#define BATT_FLAT 66
#define BATT_HIGH 85

enum { T_OK, T_WARN, T_FAIL };
static const int t_colour[] = { 0x07E0, 0xFFE0, 0xF800 };     /* RGB565 green, yellow, red */
#define T_ROWS 8                                              /* 16-pixel lines on the screen */
struct t_line { char s[11]; int level; };
static struct t_line t_log[T_ROWS], t_bad[T_ROWS];
static int t_nlog, t_nbad, t_worst;
static const char *scr_what;                                  /* from scr_init(); NULL with no screen */
static int scr_ok;
static int imu_st;                                            /* from imu_init() */

static void t_draw(const struct t_line *l, int n, int y)
{
    for (int i = 0; i < n; i++) scr_text(4, y + 16 * i, 2, t_colour[l[i].level], l[i].s);
}

static void t_line(int level, const char *scr, const char *fmt, ...)
{
    va_list ap;
    printf("%s  ", (const char *[]){ "ok  ", "warn", "FAIL" }[level]);
    va_start(ap, fmt); vprintf(fmt, ap); va_end(ap);
    putchar('\n');
    fflush(stdout);
    if (level > t_worst) t_worst = level;
    if (t_nlog == T_ROWS) memmove(t_log, t_log + 1, --t_nlog * sizeof *t_log);
    struct t_line *l = &t_log[t_nlog++];
    snprintf(l->s, sizeof l->s, "%s", scr);
    l->level = level;
    if (level != T_OK && t_nbad < T_ROWS) t_bad[t_nbad++] = *l;
    if (!scr_what) return;
    scr_clear();
    t_draw(t_log, t_nlog, 0);
    scr_show();
    msleep(250);                        /* to be seen going by */
}

/* "1,7,12" from the ids flagged in bad[12] (indexed like check_servos) */
static const char *t_ids(const int *bad)
{
    static char s[48];
    int n = 0;
    s[0] = 0;
    for (int i = 0; i < 12; i++)
        if (bad[i]) n += snprintf(s + n, sizeof s - n, "%s%d", n ? "," : "", i < 6 ? legs[i].hip : legs[i - 6].lift);
    return s;
}

static int selftest(void)
{
    int volt[12], temp[12], pos[12], miss[12] = {0}, nmiss = 0;
    unsigned char err[12] = {0};
    t_nlog = t_nbad = t_worst = 0;

    if (scr_what) t_line(scr_ok ? T_OK : T_WARN, scr_ok ? "screen" : "screen?", "screen: %s%s", scr_what,
                         scr_ok ? "" : "; the backlight (LP5562) didn't answer or the panel wasn't recognised");

    if (imu_st <= 0) {
        double a[3], g[3];
        if (imu_st < 0 || imu_read(a, g)) t_line(T_FAIL, "no imu", "IMU (BMI270) didn't answer or start");
        else {
            double pitch = atan2(a[0], a[2]) * 180 / M_PI, roll = atan2(a[1], a[2]) * 180 / M_PI;
            char s[16];
            snprintf(s, sizeof s, "tilt %.1f", hypot(pitch, roll));
            t_line(T_OK, s, "IMU: pitch %.1f deg (+ = nose up), roll %.1f deg (+ = left side up), rotation %.1f %.1f %.1f deg/s",
                   pitch, roll, g[0], g[1], g[2]);
        }
    }

    /* every servo replies (quietly: a missing battery would print 12 "no reply"s) */
    for (int i = 0; i < 12; i++) {
        int id = i < 6 ? legs[i].hip : legs[i - 6].lift;
        unsigned char b[2], e = 0;
        if (txrx(id, INST_READ, (unsigned char[]){ REG_VOLTAGE, 2 }, 2, &e, b, 2) != 2) { miss[i] = 1; nmiss++; continue; }
        volt[i] = b[0];
        temp[i] = b[1];
        err[i] |= e;
    }
    if (nmiss == 12) {
        t_line(T_FAIL, "no servos", "no servo replied: battery off, or the bus not connected?");
        goto done;
    }
    if (nmiss) {
        char s[64];
        snprintf(s, sizeof s, "no %s", t_ids(miss));
        t_line(T_FAIL, s, "no reply from servo(s) %s", t_ids(miss));
    }
    else t_line(T_OK, "servos 12", "all 12 servos reply");

    /* the battery, as the servos see it */
    int vmin = 255, vmax = 0, tmin = 255, tmax = 0;
    for (int i = 0; i < 12; i++) {
        if (miss[i]) continue;
        if (volt[i] < vmin) vmin = volt[i];
        if (volt[i] > vmax) vmax = volt[i];
        if (temp[i] < tmin) tmin = temp[i];
        if (temp[i] > tmax) tmax = temp[i];
    }
    char s[64];
    snprintf(s, sizeof s, "batt %.1fV", vmin / 10.0);
    t_line(vmin < BATT_FLAT ? T_FAIL : vmin < BATT_LOW || vmax > BATT_HIGH ? T_WARN : T_OK, s,
           "battery %.1f V (servos read %.1f..%.1f)%s", vmin / 10.0, vmin / 10.0, vmax / 10.0,
           vmin < BATT_FLAT ? ": flat, charge it" : vmin < BATT_LOW ? ": low" : vmax > BATT_HIGH ? ": high for a 2S" : "");

    snprintf(s, sizeof s, "temp %dC", tmax);
    t_line(tmax >= 60 ? T_FAIL : tmax >= 50 ? T_WARN : T_OK, s, "servo temperatures %d..%d C", tmin, tmax);

    /* error bits in the replies (overload, overheat, ...) */
    int nerr = 0;
    for (int i = 0; i < 12; i++) {
        if (!err[i]) continue;
        int id = i < 6 ? legs[i].hip : legs[i - 6].lift;
        static const char *const bitname[8] = { "volt", "angle", "heat", "amps", "?", "load", "?", "?" };
        int bit = 0;
        while (!(err[i] >> bit & 1)) bit++;
        snprintf(s, sizeof s, "%d %s", id, bitname[bit]);
        t_line(T_FAIL, s, "servo %d reports %s", id, errstr(err[i]));
        nerr++;
    }
    if (!nerr) t_line(T_OK, "no errors", "no servo reports an error");

    /* overload protection at the defaults: over 80% for 4 s drops to 20% */
    int prot[12] = {0}, nprot = 0;
    for (int i = 0; i < 12; i++) {
        int id = i < 6 ? legs[i].hip : legs[i - 6].lift;
        unsigned char p[3], u, e;
        if (miss[i]) continue;
        if (txrx(id, INST_READ, (unsigned char[]){ REG_PROT_TORQUE, 3 }, 2, &e, p, 3) != 3 ||
            txrx(id, INST_READ, (unsigned char[]){ REG_UNLOAD, 1 }, 2, &e, &u, 1) != 1) { prot[i] = 1; nprot++; continue; }
        if (p[0] != 20 || p[1] != 100 || p[2] != 80 || u != 0x20) {
            printf("      servo %d: overload torque %d%%, time %d ms, protection torque %d%%, unloading 0x%02X\n",
                   id, p[2], p[1] * 40, p[0], u);
            prot[i] = 1;
            nprot++;
        }
    }
    if (nprot) {
        snprintf(s, sizeof s, "prot %s", t_ids(prot));
        t_line(T_WARN, s, "overload protection not at the defaults (or unreadable) on servo(s) %s", t_ids(prot));
    }
    else t_line(T_OK, "prot ok", "overload protection at the defaults (over 80%% for 4 s drops to 20%%)");

    /* positions: inside the range the legs are driven over */
    int far[12] = {0}, nfar = 0, lmax = 0;
    for (int i = 0; i < 12; i++) {
        const struct leg *L = &legs[i % 6];
        int id = i < 6 ? L->hip : L->lift;
        unsigned char b[2], e;
        if (miss[i]) continue;
        if (txrx(id, INST_READ, (unsigned char[]){ REG_PRESENT_POS, 2 }, 2, &e, b, 2) != 2) { far[i] = 1; nfar++; continue; }
        pos[i] = get16(b) - CENTRE - offset[id];
        if (i < 6 ? abs(pos[i]) > HIP_MAX + 10 : L->lift_dir * pos[i] < -DOWN_MAX - 10) { far[i] = 1; nfar++; }
        if (txrx(id, INST_READ, (unsigned char[]){ REG_PRESENT_LOAD, 2 }, 2, &e, b, 2) == 2 && (get16(b) & 0x3FF) > lmax)
            lmax = get16(b) & 0x3FF;
    }
    if (nfar) {
        snprintf(s, sizeof s, "pos %s", t_ids(far));
        t_line(T_WARN, s, "servo(s) %s outside the range the legs are driven over (hips +-%d, lifts down to %d); stand will move them a long way",
               t_ids(far), HIP_MAX, DOWN_MAX);
    }
    else t_line(T_OK, "pos ok", "every servo inside the range the legs are driven over");

    /* standing still is when overload protection bites */
    snprintf(s, sizeof s, "load %d%%", lmax / 10);
    t_line(lmax >= 800 ? T_FAIL : lmax >= 500 ? T_WARN : T_OK, s, "highest load %.1f%% (overload protection starts at 80%% held for 4 s)", lmax / 10.0);

done:
    printf("self test: %s\n", t_worst == T_OK ? "all ok" : t_worst == T_WARN ? "warnings" : "problems");
    if (scr_what) {
        msleep(1000);
        scr_clear();
        int y = t_nbad ? 8 : 46;
        scr_text(21, y, 5, t_colour[t_worst], "Hi!");
        t_draw(t_bad, t_nbad < 4 ? t_nbad : 4, 56);
        scr_show();
    }
    return t_worst;
}

/*
 * Stream every servo's load and position error for secs seconds (or until a
 * key), to see what pushing on the robot looks like. Reads only. A line per
 * sweep of the 12: ms since the start, then per leg (LF LM LR RF RM RR) hip
 * load/error and lift load/error. The error is present minus goal position,
 * both read from the servo. Signed by hip_dir/lift_dir: a lift's + load is the
 * foot pushing down and + error the foot above its goal; a hip's + is forward.
 * The screen shows the seconds, to time presses by.
 */
/* title and the whole seconds since t0 on the screen, when they change: to time presses or lifts by */
static void scr_secs(const char *title, double t0, int *shown)
{
    char buf[16];
    int s = now() - t0;
    if (!scr_what || s == *shown) return;
    *shown = s;
    snprintf(buf, sizeof buf, "%d", s);
    scr_clear();
    scr_text(4, 4, 2, 0xFFFF, title);
    scr_text(64 - (int)strlen(buf) * 18, 40, 6, 0xFFFF, buf);
    scr_show();
}

static void loads(int secs)
{
    printf("ms      LF hip  lift    LM hip  lift    LR hip  lift    RF hip  lift    RM hip  lift    RR hip  lift   (load/error)\n");
    con_raw(0);
    con_drop();
    double t0 = now();
    int shown = -1;
    while (now() - t0 < secs && con_getc() < 0 && !halted) {
        scr_secs("loads", t0, &shown);
        printf("%6d", (int)((now() - t0) * 1000));
        for (int i = 0; i < 6; i++)
            for (int j = 0; j < 2; j++) {
                const struct leg *L = &legs[i];
                int id = j ? L->lift : L->hip, dir = j ? L->lift_dir : L->hip_dir;
                unsigned char b[20];
                if (read_regs(id, REG_GOAL_POS, 20, b) != 20) { printf("      -/-"); continue; }
                int goal = get16(b), pos = get16(b + REG_PRESENT_POS - REG_GOAL_POS);
                int ld = get16(b + REG_PRESENT_LOAD - REG_GOAL_POS);
                ld = ld & 0x400 ? -(ld & 0x3FF) : ld & 0x3FF;
                printf("%s %4d/%-3d", j ? "" : "  ", dir * ld, dir * (pos - goal));
            }
        putchar('\n');
    }
    con_restore();
    if (scr_what) { scr_clear(); scr_show(); }
}

/*
 * Stream the IMU for secs seconds (or until a key), 50 lines a second: ms,
 * acceleration (g) and rotation (deg/s), x forward, y left, z up, and the
 * tilt from the first reading (deg). The screen shows the seconds.
 */
static void imu_stream(int secs)
{
    double a[3], g[3], a0[3];
    if (imu_read(a0, g)) { puts("no IMU"); return; }
    printf("ms         ax     ay     az  (g)     gx     gy     gz  (deg/s)   tilt (deg)\n");
    con_raw(0);
    con_drop();
    double t0 = now();
    int shown = -1;
    tick_start();
    while (now() - t0 < secs && con_getc() < 0 && !halted) {
        scr_secs("imu", t0, &shown);
        if (imu_read(a, g)) { puts("IMU read failed"); break; }
        double dot = a[0] * a0[0] + a[1] * a0[1] + a[2] * a0[2];
        double na = sqrt(a[0] * a[0] + a[1] * a[1] + a[2] * a[2]), n0 = sqrt(a0[0] * a0[0] + a0[1] * a0[1] + a0[2] * a0[2]);
        printf("%6d  %6.3f %6.3f %6.3f   %6.1f %6.1f %6.1f   %5.1f\n", (int)((now() - t0) * 1000),
               a[0], a[1], a[2], g[0], g[1], g[2], acos(clampd(dot / (na * n0), -1, 1)) * 180 / M_PI);
        tick_wait(20);
    }
    con_restore();
    if (scr_what) { scr_clear(); scr_show(); }
}

/*
 * Tip it up at an edge and put it down: it bows towards that edge and comes
 * back up. Stands, takes the IMU's reading as level, then at 50 Hz: a tilt of
 * over BOW_TIP_ON degrees from level is a tip, and its direction at the
 * biggest tilt is kept; once it has been back under BOW_TIP_OFF for
 * BOW_SETTLE_MS (put down and still), it bows: the lifts of the two legs
 * nearest that way (from where the hips are: 60 degrees apart, the middle
 * legs straight out) go up to BOW_POS over BOW_MS, the same as
 * "move 7-8 700 1000" for the left front pair, so that edge of the body
 * comes down onto the ground (it's designed to land safely), then it
 * stands, and re-takes level. Until a key, or secs seconds.
 */
#define BOW_TIP_ON    3.0     /* deg */
#define BOW_TIP_OFF   1.5
#define BOW_SETTLE_MS 300
#define BOW_POS       700     /* the near lifts' position, as move's (the offset is added) */
#define BOW_MS        1000    /* to get there, and to stand again */

/* the IMU's acceleration averaged over ms, as the level reference */
static int bow_level(double a0[3], int ms)
{
    double a[3], g[3];
    int n = 0;
    a0[0] = a0[1] = a0[2] = 0;
    for (int t = 0; t < ms; t += 10, n++) {
        if (imu_read(a, g)) return -1;
        for (int i = 0; i < 3; i++) a0[i] += a[i];
        msleep(10);
    }
    for (int i = 0; i < 3; i++) a0[i] /= n;
    return 0;
}

static const char *bow_dir_name(double ang)
{
    static const char *const nm[8] = { "front", "front L", "left", "rear L", "rear", "rear R", "right", "front R" };
    int k = (int)lround(ang / (M_PI / 4));
    return nm[((k % 8) + 8) % 8];
}

static void bow_screen(const char *s, int colour)
{
    if (!scr_what) return;
    scr_clear();
    scr_text(4, 4, 2, 0xFFFF, "bow");
    scr_text(4, 56, 2, colour, s);
    scr_show();
}

static int stand(int ms);

static void bow(int secs)
{
    double a0[3], a[3], g[3];
    if (imu_read(a, g)) { puts("no IMU"); return; }
    if (stand(1000)) return;
    if (bow_level(a0, 500)) { puts("IMU read failed"); return; }
    printf("tip me up at an edge and put me down; a key stops\n");
    bow_screen("tip me", 0x07E0);
    con_raw(0);
    con_drop();
    double t0 = now(), peak = 0, pang = 0;
    int state = 0, settle = 0;          /* 0 waiting for a tip, 1 tipped */
    tick_start();
    while ((!secs || now() - t0 < secs) && con_getc() < 0 && !halted) {
        tick_wait(TICK_MS);
        if (imu_read(a, g)) { puts("IMU read failed"); break; }
        /* tilt from level, as the change in the horizontal part of gravity: + x = front up, + y = left up */
        double dx = a[0] - a0[0], dy = a[1] - a0[1];
        double tilt = asin(clampd(hypot(dx, dy), 0, 1)) * 180 / M_PI;
        if (state == 0) {
            if (tilt > BOW_TIP_ON) { state = 1; peak = 0; settle = 0; bow_screen("...", 0xFFE0); }
            continue;
        }
        if (tilt > peak) { peak = tilt; pang = atan2(dy, dx); }
        settle = tilt < BOW_TIP_OFF ? settle + TICK_MS : 0;
        if (settle < BOW_SETTLE_MS) continue;

        bow_screen(bow_dir_name(pang), 0x07E0);
        /* the two legs pointing most nearly that way */
        double c[6];
        int n1 = 0, n2 = 1;
        for (int i = 0; i < 6; i++) {
            const double lx[3] = { 0.866, 0, -0.866 }, ly[3] = { 0.5, 1, 0.5 };
            c[i] = lx[legs[i].row] * cos(pang) + (legs[i].side ? -1 : 1) * ly[legs[i].row] * sin(pang);
        }
        for (int i = 0; i < 6; i++) if (c[i] > c[n1]) n1 = i;
        if (n2 == n1) n2 = 0;
        for (int i = 0; i < 6; i++) if (i != n1 && c[i] > c[n2]) n2 = i;
        printf("tipped %s, %.1f deg: bowing on %s", bow_dir_name(pang), peak, leg_name(&legs[n1]));
        printf(" and %s\n", leg_name(&legs[n2]));         /* leg_name's buffer is static */
        fflush(stdout);
        /* z for a lift position of BOW_POS + offset, as leg_goals() works it out */
        legs[n1].z = height + legs[n1].lift_dir * (BOW_POS - CENTRE);
        legs[n2].z = height + legs[n2].lift_dir * (BOW_POS - CENTRE);
        send_pose(BOW_MS);
        msleep(BOW_MS + 200);
        if (halted || stand(BOW_MS)) break;
        if (bow_level(a0, 300)) { puts("IMU read failed"); break; }
        bow_screen("tip me", 0x07E0);
        state = 0;
        tick_start();
    }
    con_restore();
    for (int i = 0; i < 6; i++) legs[i].z = 0;
    send_pose(BOW_MS);
    bow_screen("", 0);
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
    else if (!strcmp(c, "selftest")) selftest();
    else if (!strcmp(c, "loads")) loads(arg(tok, 1, nt, 10, NULL));
    else if (!strcmp(c, "imu")) imu_stream(arg(tok, 1, nt, 10, NULL));
    else if (!strcmp(c, "bow")) bow(arg(tok, 1, nt, 0, NULL));
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
    scr_what = scr_init(&scr_ok);
    imu_st = imu_init();
    printf("hector on the ATOM: bus on UART1 (TX G38, RX G39) @ 1000000\n");
    if (selftest() != T_FAIL) stand(1000);      /* power on = stand up, unless something failed */
    for (;;) {
        printf("type help\n");
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
    printf("opened %s @ %d\n", dev, baud);
    scr_what = scr_init(&scr_ok);
    imu_st = imu_init();
    selftest();
    printf("type help\n");
    repl();
    close(fd);
    return 0;
}
#endif
