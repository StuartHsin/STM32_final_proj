#ifndef __SSD1306_CONF_H__
#define __SSD1306_CONF_H__

// ==== DISPLAY SETTINGS ================================
#define SSD1306_WIDTH           128
#define SSD1306_HEIGHT          32  // ← 若用的是 128x64 請改成 64

#define SSD1306_INCLUDE_FONT_7x10
#define SSD1306_INCLUDE_Font_12x15
#define SSD1306_INCLUDE_FONT_11x18

// ==== I2C SETTINGS =====================================
#define SSD1306_USE_I2C         1
#define SSD1306_I2C_PORT        hi2c1
#define SSD1306_I2C_ADDR        0x78  // 0x3C << 1

// ==== BUFFER SETTINGS ==================================
#define SSD1306_BUFFER_SIZE     (SSD1306_WIDTH * SSD1306_HEIGHT / 8)

#endif /* __SSD1306_CONF_H__ */
