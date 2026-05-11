// User setup for TFT_eSPI library - Raspberry Pi Pico with SPI display

#ifndef USER_SETUP_H_INCLUDED
#define USER_SETUP_H_INCLUDED

// ===========================
// Display Model Configuration
// ===========================
// Uncomment your display model:
//#define ILI9341_DRIVER       // 2.4" ILI9341 SPI display
 #define ILI9488_DRIVER    // 3.5" ILI9488 SPI display  
// #define ST7735_DRIVER     // 1.8" ST7735 SPI display
// #define ST7789_DRIVER     // 1.3" ST7789 SPI display

// ===========================
// SPI Pin Configuration for Raspberry Pi Pico
// ===========================
#define TFT_MOSI 19    // GPIO19
#define TFT_MISO 16    // GPIO16
#define TFT_SCLK 18    // GPIO18
#define TFT_CS   17    // GPIO17 (Chip Select)
#define TFT_DC   20    // GPIO20 (Data/Command)
#define TFT_RST  -1    // Set to -1 if RST is connected to 3.3V

// Optional: backlight pin (set to -1 if not using)
#define TFT_BL   -1    // Backlight control pin

// ===========================
// SPI Speed
// ===========================
#define SPI_FREQUENCY  75000000    // 75 MHz - RP2350 max (sys_clk 150 MHz / 2)

// ===========================
// Display Orientation and Size
// ===========================
#define TFT_WIDTH  240   // confirmed by hardware
#define TFT_HEIGHT 320   // confirmed by hardware
// ===========================
// Font Smoothing
// ===========================
#define SMOOTH_FONT

// ===========================
// Color Order
// ===========================
// ILI9488 panels sometimes ship with BGR order instead of RGB.
// If blue appears as red (or vice versa), swap this define.
#define TFT_RGB_ORDER TFT_RGB
#define TFT_INVERSION_ON   // try TFT_RGB if colors are still wrong


// ===========================
// Library Options
// ===========================
#define SUPPORT_TRANSACTIONS

#endif // USER_SETUP_H_INCLUDED
