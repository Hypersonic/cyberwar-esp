#include "driver/gpio.h"
#include "driver/spi_master.h"
#include "esp_system.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "soc/gpio_struct.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define PIN_NUM_MISO 25
#define PIN_NUM_MOSI 23
#define PIN_NUM_CLK 19
#define PIN_NUM_CS 22

#define PIN_NUM_DC 21
#define PIN_NUM_RST 18
#define PIN_NUM_BCKL 5

/*
 The LCD needs a bunch of command/argument values to be initialized. They are
 stored in this struct.
*/
typedef struct {
  uint8_t cmd;
  uint8_t data[16];
  uint8_t databytes; // No of data in data; bit 7 = delay after set; 0xFF = end
                     // of cmds.
} lcd_init_cmd_t;

typedef enum {
  LCD_TYPE_ILI = 1,
  LCD_TYPE_ST,
  LCD_TYPE_MAX,
} type_lcd_t;

// Place data into DRAM. Constant data gets placed into DROM by default, which
// is not accessible by DMA.
DRAM_ATTR static const lcd_init_cmd_t st_init_cmds[] = {
    {0x36, {(1 << 5) | (1 << 6)}, 1},
    {0x3A, {0x55}, 1},
    {0xB2, {0x0c, 0x0c, 0x00, 0x33, 0x33}, 5},
    {0xB7, {0x45}, 1},
    {0xBB, {0x2B}, 1},
    {0xC0, {0x2C}, 1},
    {0xC2, {0x01, 0xff}, 2},
    {0xC3, {0x11}, 1},
    {0xC4, {0x20}, 1},
    {0xC6, {0x0f}, 1},
    {0xD0, {0xA4, 0xA1}, 1},
    {0xE0,
     {0xD0, 0x00, 0x05, 0x0E, 0x15, 0x0D, 0x37, 0x43, 0x47, 0x09, 0x15, 0x12,
      0x16, 0x19},
     14},
    {0xE1,
     {0xD0, 0x00, 0x05, 0x0D, 0x0C, 0x06, 0x2D, 0x44, 0x40, 0x0E, 0x1C, 0x18,
      0x16, 0x19},
     14},
    {0x11, {0}, 0x80},
    {0x29, {0}, 0x80},
    {0, {0}, 0xff}};

DRAM_ATTR static const lcd_init_cmd_t ili_init_cmds[] = {
    {0xCF, {0x00, 0x83, 0X30}, 3},
    {0xED, {0x64, 0x03, 0X12, 0X81}, 4},
    {0xE8, {0x85, 0x01, 0x79}, 3},
    {0xCB, {0x39, 0x2C, 0x00, 0x34, 0x02}, 5},
    {0xF7, {0x20}, 1},
    {0xEA, {0x00, 0x00}, 2},
    {0xC0, {0x26}, 1},
    {0xC1, {0x11}, 1},
    {0xC5, {0x35, 0x3E}, 2},
    {0xC7, {0xBE}, 1},
    {0x36, {0x28}, 1},
    {0x3A, {0x55}, 1},
    {0xB1, {0x00, 0x1B}, 2},
    {0xF2, {0x08}, 1},
    {0x26, {0x01}, 1},
    {0xE0,
     {0x1F, 0x1A, 0x18, 0x0A, 0x0F, 0x06, 0x45, 0X87, 0x32, 0x0A, 0x07, 0x02,
      0x07, 0x05, 0x00},
     15},
    {0XE1,
     {0x00, 0x25, 0x27, 0x05, 0x10, 0x09, 0x3A, 0x78, 0x4D, 0x05, 0x18, 0x0D,
      0x38, 0x3A, 0x1F},
     15},
    {0x2A, {0x00, 0x00, 0x00, 0xEF}, 4},
    {0x2B, {0x00, 0x00, 0x01, 0x3f}, 4},
    {0x2C, {0}, 0},
    {0xB7, {0x07}, 1},
    {0xB6, {0x0A, 0x82, 0x27, 0x00}, 4},
    {0x11, {0}, 0x80},
    {0x29, {0}, 0x80},
    {0, {0}, 0xff},
};

// Send a command to the LCD. Uses spi_device_transmit, which waits until the
// transfer is complete.
void lcd_cmd(spi_device_handle_t spi, const uint8_t cmd) {
  esp_err_t ret;
  spi_transaction_t t;
  memset(&t, 0, sizeof(t));           // Zero out the transaction
  t.length = 8;                       // Command is 8 bits
  t.tx_buffer = &cmd;                 // The data is the cmd itself
  t.user = (void *)0;                 // D/C needs to be set to 0
  ret = spi_device_transmit(spi, &t); // Transmit!
  assert(ret == ESP_OK);              // Should have had no issues.
}

// Send data to the LCD. Uses spi_device_transmit, which waits until the
// transfer is complete.
void lcd_data(spi_device_handle_t spi, const uint8_t *data, int len) {
  esp_err_t ret;
  spi_transaction_t t;
  if (len == 0)
    return;                 // no need to send anything
  memset(&t, 0, sizeof(t)); // Zero out the transaction
  t.length = len * 8;       // Len is in bytes, transaction length is in bits.
  t.tx_buffer = data;       // Data
  t.user = (void *)1;       // D/C needs to be set to 1
  ret = spi_device_transmit(spi, &t); // Transmit!
  assert(ret == ESP_OK);              // Should have had no issues.
}

// This function is called (in irq context!) just before a transmission starts.
// It will set the D/C line to the value indicated in the user field.
void lcd_spi_pre_transfer_callback(spi_transaction_t *t) {
  int dc = (int)t->user;
  gpio_set_level(PIN_NUM_DC, dc);
}

uint32_t lcd_get_id(spi_device_handle_t spi) {
  // get_id cmd
  lcd_cmd(spi, 0x04);

  spi_transaction_t t;
  memset(&t, 0, sizeof(t));
  t.length = 8 * 3;
  t.flags = SPI_TRANS_USE_RXDATA;
  t.user = (void *)1;

  esp_err_t ret = spi_device_transmit(spi, &t);
  assert(ret == ESP_OK);

  return *(uint32_t *)t.rx_data;
}

// Initialize the display
void lcd_init(spi_device_handle_t spi) {
  int cmd = 0;
  const lcd_init_cmd_t *lcd_init_cmds;

  // Initialize non-SPI GPIOs
  gpio_set_direction(PIN_NUM_DC, GPIO_MODE_OUTPUT);
  gpio_set_direction(PIN_NUM_RST, GPIO_MODE_OUTPUT);
  gpio_set_direction(PIN_NUM_BCKL, GPIO_MODE_OUTPUT);

  // Reset the display
  gpio_set_level(PIN_NUM_RST, 0);
  vTaskDelay(100 / portTICK_RATE_MS);
  gpio_set_level(PIN_NUM_RST, 1);
  vTaskDelay(100 / portTICK_RATE_MS);

  // detect LCD type
  uint32_t lcd_id = lcd_get_id(spi);
  int lcd_detected_type = 0;
  int lcd_type;

  printf("LCD ID: %08X\n", lcd_id);
  if (lcd_id == 0) {
    // zero, ili
    lcd_detected_type = LCD_TYPE_ILI;
    printf("ILI9341 detected...\n");
  } else {
    // none-zero, ST
    lcd_detected_type = LCD_TYPE_ST;
    printf("ST7789V detected...\n");
  }

#ifdef CONFIG_LCD_TYPE_AUTO
  lcd_type = lcd_detected_type;
#elif defined(CONFIG_LCD_TYPE_ST7789V)
  printf("kconfig: force CONFIG_LCD_TYPE_ST7789V.\n");
  lcd_type = LCD_TYPE_ST;
#elif defined(CONFIG_LCD_TYPE_ILI9341)
  printf("kconfig: force CONFIG_LCD_TYPE_ILI9341.\n");
  lcd_type = LCD_TYPE_ILI;
#endif
  if (lcd_type == LCD_TYPE_ST) {
    printf("LCD ST7789V initialization.\n");
    lcd_init_cmds = st_init_cmds;
  } else {
    printf("LCD ILI9341 initialization.\n");
    lcd_init_cmds = ili_init_cmds;
  }

  // Send all the commands
  while (lcd_init_cmds[cmd].databytes != 0xff) {
    lcd_cmd(spi, lcd_init_cmds[cmd].cmd);
    lcd_data(spi, lcd_init_cmds[cmd].data, lcd_init_cmds[cmd].databytes & 0x1F);
    if (lcd_init_cmds[cmd].databytes & 0x80) {
      vTaskDelay(100 / portTICK_RATE_MS);
    }
    cmd++;
  }

  /// Enable backlight
  gpio_set_level(PIN_NUM_BCKL, 0);
}

// To send a line we have to send a command, 2 data bytes, another command, 2
// more data bytes and another command before sending the line data itself; a
// total of 6 transactions. (We can't put all of this in just one transaction
// because the D/C line needs to be toggled in the middle.)
// This routine queues these commands up so they get sent as quickly as
// possible.
static void send_line(spi_device_handle_t spi, int ypos, uint16_t *line) {
  esp_err_t ret;
  int x;
  // Transaction descriptors. Declared static so they're not allocated on the
  // stack; we need this memory even when this function is finished because the
  // SPI driver needs access to it even while we're already calculating the next
  // line.
  static spi_transaction_t trans[6];

  // In theory, it's better to initialize trans and data only once and hang on
  // to the initialized variables. We allocate them on the stack, so we need to
  // re-init them each call.
  for (x = 0; x < 6; x++) {
    memset(&trans[x], 0, sizeof(spi_transaction_t));
    if ((x & 1) == 0) {
      // Even transfers are commands
      trans[x].length = 8;
      trans[x].user = (void *)0;
    } else {
      // Odd transfers are data
      trans[x].length = 8 * 4;
      trans[x].user = (void *)1;
    }
    trans[x].flags = SPI_TRANS_USE_TXDATA;
  }
  trans[0].tx_data[0] = 0x2A;              // Column Address Set
  trans[1].tx_data[0] = 0;                 // Start Col High
  trans[1].tx_data[1] = 0;                 // Start Col Low
  trans[1].tx_data[2] = (320) >> 8;        // End Col High
  trans[1].tx_data[3] = (320) & 0xff;      // End Col Low
  trans[2].tx_data[0] = 0x2B;              // Page address set
  trans[3].tx_data[0] = ypos >> 8;         // Start page high
  trans[3].tx_data[1] = ypos & 0xff;       // start page low
  trans[3].tx_data[2] = (ypos + 1) >> 8;   // end page high
  trans[3].tx_data[3] = (ypos + 1) & 0xff; // end page low
  trans[4].tx_data[0] = 0x2C;              // memory write
  trans[5].tx_buffer = line;               // finally send the line data
  trans[5].length = 320 * 2 * 8;           // Data length, in bits
  trans[5].flags = 0;                      // undo SPI_TRANS_USE_TXDATA flag

  // Queue all transactions.
  for (x = 0; x < 6; x++) {
    ret = spi_device_queue_trans(spi, &trans[x], portMAX_DELAY);
    assert(ret == ESP_OK);
  }

  // When we are here, the SPI driver is busy (in the background) getting the
  // transactions sent. That happens mostly using DMA, so the CPU doesn't have
  // much to do here. We're not going to wait for the transaction to finish
  // because we may as well spend the time calculating the next line. When that
  // is done, we can call send_line_finish, which will wait for the transfers to
  // be done and check their status.
}

static void send_line_finish(spi_device_handle_t spi) {
  spi_transaction_t *rtrans;
  esp_err_t ret;
  // Wait for all 6 transactions to be done and get back the results.
  for (int x = 0; x < 6; x++) {
    ret = spi_device_get_trans_result(spi, &rtrans, portMAX_DELAY);
    assert(ret == ESP_OK);
    // We could inspect rtrans now if we received any info back. The LCD is
    // treated as write-only, though.
  }
}

// it's like... 333 encoded as 555... wtf
#define RGB(r, g, b) ((b) << 10) | ((r) << 5) | (g)

uint16_t frame_buffer[240][320];
// VGA mode 13h palette
const uint16_t palette[256] = {
    RGB(0, 0, 0), RGB(0, 0, 5), RGB(0, 5, 0), RGB(0, 5, 5), RGB(5, 0, 0),
    RGB(5, 0, 5), RGB(5, 2, 0), RGB(5, 5, 5), RGB(2, 2, 2), RGB(2, 2, 7),
    RGB(2, 7, 2), RGB(2, 7, 7), RGB(7, 2, 2), RGB(7, 2, 7), RGB(7, 7, 2),
    RGB(7, 7, 7), RGB(0, 0, 0), RGB(1, 1, 1), RGB(1, 1, 1), RGB(1, 1, 1),
    RGB(2, 2, 2), RGB(2, 2, 2), RGB(2, 2, 2), RGB(3, 3, 3), RGB(3, 3, 3),
    RGB(4, 4, 4), RGB(4, 4, 4), RGB(4, 4, 4), RGB(5, 5, 5), RGB(6, 6, 6),
    RGB(6, 6, 6), RGB(7, 7, 7), RGB(0, 0, 7), RGB(2, 0, 7), RGB(3, 0, 7),
    RGB(5, 0, 7), RGB(7, 0, 7), RGB(7, 0, 5), RGB(7, 0, 3), RGB(7, 0, 2),
    RGB(7, 0, 0), RGB(7, 2, 0), RGB(7, 3, 0), RGB(7, 5, 0), RGB(7, 7, 0),
    RGB(5, 7, 0), RGB(3, 7, 0), RGB(2, 7, 0), RGB(0, 7, 0), RGB(0, 7, 2),
    RGB(0, 7, 3), RGB(0, 7, 5), RGB(0, 7, 7), RGB(0, 5, 7), RGB(0, 3, 7),
    RGB(0, 2, 7), RGB(3, 3, 7), RGB(4, 3, 7), RGB(5, 3, 7), RGB(6, 3, 7),
    RGB(7, 3, 7), RGB(7, 3, 6), RGB(7, 3, 5), RGB(7, 3, 4), RGB(7, 3, 3),
    RGB(7, 4, 3), RGB(7, 5, 3), RGB(7, 6, 3), RGB(7, 7, 3), RGB(6, 7, 3),
    RGB(5, 7, 3), RGB(4, 7, 3), RGB(3, 7, 3), RGB(3, 7, 4), RGB(3, 7, 5),
    RGB(3, 7, 6), RGB(3, 7, 7), RGB(3, 6, 7), RGB(3, 5, 7), RGB(3, 4, 7),
    RGB(5, 5, 7), RGB(5, 5, 7), RGB(6, 5, 7), RGB(6, 5, 7), RGB(7, 5, 7),
    RGB(7, 5, 6), RGB(7, 5, 6), RGB(7, 5, 5), RGB(7, 5, 5), RGB(7, 5, 5),
    RGB(7, 6, 5), RGB(7, 6, 5), RGB(7, 7, 5), RGB(6, 7, 5), RGB(6, 7, 5),
    RGB(5, 7, 5), RGB(5, 7, 5), RGB(5, 7, 5), RGB(5, 7, 6), RGB(5, 7, 6),
    RGB(5, 7, 7), RGB(5, 6, 7), RGB(5, 6, 7), RGB(5, 5, 7), RGB(0, 0, 3),
    RGB(1, 0, 3), RGB(2, 0, 3), RGB(2, 0, 3), RGB(3, 0, 3), RGB(3, 0, 2),
    RGB(3, 0, 2), RGB(3, 0, 1), RGB(3, 0, 0), RGB(3, 1, 0), RGB(3, 2, 0),
    RGB(3, 2, 0), RGB(3, 3, 0), RGB(2, 3, 0), RGB(2, 3, 0), RGB(1, 3, 0),
    RGB(0, 3, 0), RGB(0, 3, 1), RGB(0, 3, 2), RGB(0, 3, 2), RGB(0, 3, 3),
    RGB(0, 2, 3), RGB(0, 2, 3), RGB(0, 1, 3), RGB(2, 2, 3), RGB(2, 2, 3),
    RGB(2, 2, 3), RGB(3, 2, 3), RGB(3, 2, 3), RGB(3, 2, 3), RGB(3, 2, 2),
    RGB(3, 2, 2), RGB(3, 2, 2), RGB(3, 2, 2), RGB(3, 2, 2), RGB(3, 3, 2),
    RGB(3, 3, 2), RGB(3, 3, 2), RGB(2, 3, 2), RGB(2, 3, 2), RGB(2, 3, 2),
    RGB(2, 3, 2), RGB(2, 3, 2), RGB(2, 3, 3), RGB(2, 3, 3), RGB(2, 3, 3),
    RGB(2, 2, 3), RGB(2, 2, 3), RGB(2, 2, 3), RGB(2, 2, 3), RGB(3, 2, 3),
    RGB(3, 2, 3), RGB(3, 2, 3), RGB(3, 2, 3), RGB(3, 2, 3), RGB(3, 2, 2),
    RGB(3, 2, 2), RGB(3, 2, 2), RGB(3, 3, 2), RGB(3, 3, 2), RGB(3, 3, 2),
    RGB(3, 3, 2), RGB(3, 3, 2), RGB(2, 3, 2), RGB(2, 3, 2), RGB(2, 3, 2),
    RGB(2, 3, 3), RGB(2, 3, 3), RGB(2, 3, 3), RGB(2, 3, 3), RGB(2, 3, 3),
    RGB(2, 2, 3), RGB(0, 0, 2), RGB(0, 0, 2), RGB(1, 0, 2), RGB(1, 0, 2),
    RGB(2, 0, 2), RGB(2, 0, 1), RGB(2, 0, 1), RGB(2, 0, 0), RGB(2, 0, 0),
    RGB(2, 0, 0), RGB(2, 1, 0), RGB(2, 1, 0), RGB(2, 2, 0), RGB(1, 2, 0),
    RGB(1, 2, 0), RGB(0, 2, 0), RGB(0, 2, 0), RGB(0, 2, 0), RGB(0, 2, 1),
    RGB(0, 2, 1), RGB(0, 2, 2), RGB(0, 1, 2), RGB(0, 1, 2), RGB(0, 0, 2),
    RGB(1, 1, 2), RGB(1, 1, 2), RGB(1, 1, 2), RGB(2, 1, 2), RGB(2, 1, 2),
    RGB(2, 1, 2), RGB(2, 1, 1), RGB(2, 1, 1), RGB(2, 1, 1), RGB(2, 1, 1),
    RGB(2, 1, 1), RGB(2, 2, 1), RGB(2, 2, 1), RGB(2, 2, 1), RGB(1, 2, 1),
    RGB(1, 2, 1), RGB(1, 2, 1), RGB(1, 2, 1), RGB(1, 2, 1), RGB(1, 2, 2),
    RGB(1, 2, 2), RGB(1, 2, 2), RGB(1, 1, 2), RGB(1, 1, 2), RGB(1, 1, 2),
    RGB(1, 1, 2), RGB(1, 1, 2), RGB(2, 1, 2), RGB(2, 1, 2), RGB(2, 1, 2),
    RGB(2, 1, 1), RGB(2, 1, 1), RGB(2, 1, 1), RGB(2, 1, 1), RGB(2, 1, 1),
    RGB(2, 2, 1), RGB(2, 2, 1), RGB(2, 2, 1), RGB(1, 2, 1), RGB(1, 2, 1),
    RGB(1, 2, 1), RGB(1, 2, 1), RGB(1, 2, 1), RGB(1, 2, 2), RGB(1, 2, 2),
    RGB(1, 2, 2), RGB(1, 1, 2), RGB(1, 1, 2), RGB(0, 0, 0), RGB(0, 0, 0),
    RGB(0, 0, 0), RGB(0, 0, 0), RGB(0, 0, 0), RGB(0, 0, 0), RGB(0, 0, 0),
    RGB(0, 0, 0),

};

#define WRITE_PIXEL(x, y, color) frame_buffer[(y)][(x)] = palette[(color)]

void flush_frame_buffer(spi_device_handle_t spi) {
  // TODO: it should be possible to write the whole frame in one spi command,
  // right?
  int y;
  for (y = 0; y < 240; y++) {
    send_line(spi, y, frame_buffer[y]);
    send_line_finish(spi);
  }
}

void clear_frame_buffer(void) { memset(frame_buffer, 0, sizeof(frame_buffer)); }

uint16_t get_octant(int16_t x, int16_t y) {
  short lut[8] = {0, 1, 3, 2, 7, 6, 4, 5};
  short flags = 0;
  short a, b, c;
  a = y < 0;
  b = x < 0;
  c = abs(x) < abs(y);

  if (a)
    flags |= (1 << 2);
  if (b)
    flags |= (1 << 1);
  if (c)
    flags |= (1 << 0);

  return lut[flags];
}

void normalize_to_octant(uint16_t octant, int16_t *x, int16_t *y) {
  uint16_t rx, ry;
  switch (octant) {
  case 0: // (x, y)
    break;
  case 1: // (y, x)
    rx = *x;
    ry = *y;
    *x = ry;
    *y = rx;
    break;
  case 2: // (y, -x)
    rx = -*x;
    ry = *y;
    *x = ry;
    *y = rx;
    break;
  case 3: // (-x, y)
    *x = -*x;
    break;
  case 4: // (-x, -y)
    *x = -*x;
    *y = -*y;
    break;
  case 5: // (-y, -x)
    rx = -*x;
    ry = -*y;
    *x = ry;
    *y = rx;
    break;
  case 6: // (-y, x)
    rx = *x;
    ry = -*y;
    *x = ry;
    *y = rx;
    break;
  case 7: // (x, -y)
    *y = -*y;
    break;
  }
}

void denormalize_from_octant(uint16_t octant, int16_t *x, int16_t *y) {
  int16_t rx, ry;
  switch (octant) {
  case 0: // (x, y)
    break;
  case 1: // (y, x)
    rx = *x;
    ry = *y;
    *x = ry;
    *y = rx;
    break;
  case 2: // (-y, x)
    rx = *x;
    ry = -*y;
    *x = ry;
    *y = rx;
    break;
  case 3: // (-x, y)
    *x = -*x;
    *y = *y;
    break;
  case 4: // (-x, -y)
    *x = -*x;
    *y = -*y;
    break;
  case 5: // (-y, -x)
    rx = -*x;
    ry = -*y;
    *x = ry;
    *y = rx;
    break;
  case 6: // (y, -x)
    rx = -*x;
    ry = *y;
    *x = ry;
    *y = rx;
    break;
  case 7: // (x, -y)
    *x = *x;
    *y = -*y;
    break;
  }
}

void draw_hline(int16_t x0, int16_t x1, int16_t y, int16_t color) {
  for (; x0 < x1; x0++) {
    WRITE_PIXEL(x0, y, color);
  }
}

void draw_line(int16_t x0, int16_t y0, int16_t x1, int16_t y1, uint16_t color) {
  uint16_t octant = get_octant(x1 - x0, y1 - y0);
  normalize_to_octant(octant, &x0, &y0);
  normalize_to_octant(octant, &x1, &y1);
  int16_t dx = x1 - x0;
  int16_t dy = y1 - y0;
  int16_t D = 2 * dy - dx;
  int16_t y = y0;
  int16_t x;
  int16_t tx, ty;
  for (x = x0; x < x1; x++) {
    tx = x, ty = y;
    denormalize_from_octant(octant, &tx, &ty);
    WRITE_PIXEL(tx, ty, color);
    if (D > 0) {
      y++;
      D -= 2 * dx;
    }
    D += 2 * dy;
  }
}

#define LERP_POWER_OF_2 4
int16_t interpolate(int16_t lo, int16_t hi, uint16_t t) {
  int16_t ax = (((1 << LERP_POWER_OF_2) - t) * lo) >> LERP_POWER_OF_2;
  int16_t bx = (t * hi) >> LERP_POWER_OF_2;
  return ax + bx;
}

void draw_bezier(int16_t x0, int16_t y0, int16_t x1, int16_t y1, int16_t x2,
                 int16_t y2, uint16_t color, uint16_t start_sweep,
                 uint16_t end_sweep) {
  uint16_t t;
  int16_t ix0, iy0, ix1, iy1, ox, oy, px, py;
  px = x0;
  py = y0;
  for (t = start_sweep; t <= end_sweep; t++) {
    ix0 = interpolate(x0, x1, t);
    iy0 = interpolate(y0, y1, t);
    ix1 = interpolate(x1, x2, t);
    iy1 = interpolate(y1, y2, t);
    ox = interpolate(ix0, ix1, t);
    oy = interpolate(iy0, iy1, t);
    draw_line(px, py, ox, oy, color);
    px = ox;
    py = oy;
  }
}

void draw_filled_circle(int16_t x0, int16_t y0, int16_t radius,
                        uint16_t color) {
  int16_t x = radius - 1;
  int16_t y = 0;
  int16_t dx = 1;
  int16_t dy = 1;
  int16_t err = dx - (radius << 1);

  while (x >= y) {
    draw_hline(x0 - x, x0 + x, y0 + y, color);
    draw_hline(x0 - y, x0 + y, y0 + x, color);
    draw_hline(x0 - x, x0 + x, y0 - y, color);
    draw_hline(x0 - y, x0 + y, y0 - x, color);

    if (err <= 0) {
      y++;
      err += dy;
      dy += 2;
    }

    if (err > 0) {
      x--;
      dx += 2;
      err += dx - (radius << 1);
    }
  }
}

struct map_pixel {
  int16_t x;
  int16_t y;
};
const struct map_pixel worldmap[] = {
#include "map.inc"
};
const uint16_t n_worldmap_px = sizeof(worldmap) / sizeof(struct map_pixel);

struct launchsite {
  int16_t x;
  int16_t y;
};

const struct launchsite launchsites[2][4] = {
    { // usa:
        {70, 77},
        {73, 84},
        {65, 75},
        {60, 79},
    },
    { // ussr:
        {186, 74},
        {202, 68},
        {158, 76},
        {215, 77},
    },
};

struct missile {
  int16_t launch_x;
  int16_t launch_y;
  int16_t target_x;
  int16_t target_y;
  int16_t end_sweep;
  int16_t ticks_until_move;
  int8_t in_use;
  int8_t country;
  int16_t color;
};

struct explosion {
  int16_t explosion_x;
  int16_t explosion_y;
  int16_t ticks_until_death;
  int8_t in_use;
};

#define MAX_MISSLES 0x40
#define MAX_EXPLOSIONS MAX_MISSLES
#define TICKS_BETWEEN_MOVES 1
#define EXPLOSION_TICKS 10

struct game_state {
  int frame_num;
  uint16_t rng_state;
  struct missile missiles[MAX_MISSLES];
  struct explosion explosions[MAX_EXPLOSIONS];
  uint8_t shooting_country;
};

void draw_worldmap(void) {
  int i;
  for (i = 0; i < n_worldmap_px; i++) {
    WRITE_PIXEL(worldmap[i].x, worldmap[i].y, 0x1f);
  }
}

int16_t min(int16_t a, int16_t b) {
  if (a < b)
    return a;
  return b;
}

uint16_t rng_next(struct game_state *state) {
  state->rng_state *= 0x2331;
  state->rng_state += 12345;
  return state->rng_state;
}

void draw_missile(struct missile *missile) {
  int16_t mid_x = interpolate(missile->launch_x, missile->target_x, 0xb);
  int16_t mid_y = min(missile->launch_y, missile->target_y) - 100;
  draw_bezier(missile->launch_x, missile->launch_y, mid_x, mid_y,
              missile->target_x, missile->target_y, missile->color, 0,
              missile->end_sweep);
}

struct missile *find_free_missile_slot(struct game_state *state) {
  uint16_t i;
  for (i = 0; i < MAX_MISSLES; i++) {
    if (state->missiles[i].in_use)
      continue;
    return &state->missiles[i];
  }
  return NULL;
}

struct explosion *find_free_explosion_slot(struct game_state *state) {
  uint16_t i;
  for (i = 0; i < MAX_EXPLOSIONS; i++) {
    if (state->explosions[i].in_use)
      continue;
    return &state->explosions[i];
  }
  return NULL;
}

static void tick_gameplay(struct game_state *state) {
  uint16_t i, j;
  state->frame_num++;
  // create new missiles if we need to
  struct missile *free_slot = find_free_missile_slot(state);
  if (free_slot) {
    const struct launchsite *ls =
        &launchsites[state->shooting_country][rng_next(state) & 0b11];
    uint16_t min_x, max_x;
    int16_t target_x = -1, target_y = -1;
    if (state->shooting_country == 0) { // america
      min_x = 160;
      max_x = 290;
    } else { // ussr
      min_x = 30;
      max_x = 90;
    }
    while (!(min_x <= target_x && target_x <= max_x)) {
      target_x = rng_next(state);
    }
    while (!(60 <= target_y && target_y <= 100)) {
      target_y = rng_next(state);
    }
    free_slot->in_use = true;
    free_slot->launch_x = ls->x;
    free_slot->launch_y = ls->y;
    free_slot->target_x = target_x;
    free_slot->target_y = target_y;
    free_slot->end_sweep = 0;
    free_slot->color = (state->shooting_country + target_x + target_y) & 0xff;
    free_slot->ticks_until_move = TICKS_BETWEEN_MOVES;

    state->shooting_country = !state->shooting_country;
  }

  // simulate
  // missiles
  for (i = 0; i < MAX_MISSLES; i++) {
    if (!state->missiles[i].in_use)
      continue;

    if (--state->missiles[i].ticks_until_move == 0) {
      state->missiles[i].ticks_until_move = TICKS_BETWEEN_MOVES;
      if (state->missiles[i].end_sweep++ == 16) {
        state->missiles[i].in_use = false;
        struct explosion *explosion = find_free_explosion_slot(state);
        if (explosion) {
          explosion->in_use = true;
          explosion->explosion_x = state->missiles[i].target_x;
          explosion->explosion_y = state->missiles[i].target_y;
          explosion->ticks_until_death = EXPLOSION_TICKS;
        }
      }
    }
  }
  // explosions
  for (i = 0; i < MAX_EXPLOSIONS; i++) {
    if (state->explosions[i].in_use &&
        state->explosions[i].ticks_until_death-- == 0) {
      state->explosions[i].in_use = false;
    }
  }

  // render
  draw_worldmap();
  // launchsites
  for (i = 0; i < 2; i++) {
    for (j = 0; j < 4; j++) {
      const struct launchsite *ls = &launchsites[i][j];
      WRITE_PIXEL(ls->x, ls->y - 1, i + 3);
      WRITE_PIXEL(ls->x - 1, ls->y + 1, i + 3);
      WRITE_PIXEL(ls->x + 1, ls->y + 1, i + 3);
    }
  }
  // explosions
  for (i = 0; i < MAX_EXPLOSIONS; i++) {
    if (state->explosions[i].in_use) {
      int16_t radius =
          -(state->explosions[i].ticks_until_death - EXPLOSION_TICKS);
      draw_filled_circle(state->explosions[i].explosion_x,
                         state->explosions[i].explosion_y, radius, 0xf);
    }
  }
  // missiles
  for (i = 0; i < MAX_MISSLES; i++) {
    if (state->missiles[i].in_use)
      draw_missile(&state->missiles[i]);
  }
}

void app_main() {
  esp_err_t ret;
  spi_device_handle_t spi;
  spi_bus_config_t buscfg = {.miso_io_num = PIN_NUM_MISO,
                             .mosi_io_num = PIN_NUM_MOSI,
                             .sclk_io_num = PIN_NUM_CLK,
                             .quadwp_io_num = -1,
                             .quadhd_io_num = -1};
  spi_device_interface_config_t devcfg = {
      .clock_speed_hz = 30 * 1000 * 1000, // Clock out at 30 MHz
      .mode = 0,                          // SPI mode 0
      .spics_io_num = PIN_NUM_CS,         // CS pin
      .queue_size = 7, // We want to be able to queue 7 transactions at a time
      .pre_cb = lcd_spi_pre_transfer_callback, // Specify pre-transfer callback
                                               // to handle D/C line
  };
  // Initialize the SPI bus
  ret = spi_bus_initialize(HSPI_HOST, &buscfg, 1);
  assert(ret == ESP_OK);
  // Attach the LCD to the SPI bus
  ret = spi_bus_add_device(HSPI_HOST, &devcfg, &spi);
  assert(ret == ESP_OK);
  // Initialize the LCD
  lcd_init(spi);

  struct game_state state = {0};
  while (1) {
    tick_gameplay(&state);
    flush_frame_buffer(spi);
    clear_frame_buffer();
  }
}
