#include "JPEGDecoder.h"
#include "TFT_eSPI.h"

void drawArrayJpeg(TFT_eSPI *tft, const uint8_t array[], uint32_t  array_size, int xpos, int ypos);
void drawSdJpeg(TFT_eSPI *tft, const char *filename, int xpos, int ypos);
void jpegRender(TFT_eSPI *tft, int xpos, int ypos);
void jpegInfo();



