#ifndef WS2812_H_
#define WS2812_H_
//--------------------------------------------------
#include "stm32f1xx_hal.h"
#include <string.h>
#include <stdlib.h>
#include <stdint.h>
//--------------------------------------------------
#define DELAY_LEN 48
#define LED_COUNT 64
#define ARRAY_LEN DELAY_LEN + LED_COUNT*24
#define HIGH 43	//65
#define LOW	21 //26
//--------------------------------------------------
#define TIM_NUM	   2  ///< Timer number
#define TIM_CH	   TIM_CHANNEL_2  ///< Timer's PWM channel
#define DMA_HANDLE hdma_tim2_ch2_ch4  ///< DMA Channel
#define DMA_SIZE_BYTE     ///< DMA Memory Data Width: {.._BYTE, .._HWORD, .._WORD}

//--------------------------------------------------



//--------------------------------------------------
#define BitIsSet(reg, bit) ((reg & (1<<bit)) != 0)
//--------------------------------------------------
void ws2812_init(void);
void ws2812_pixel_rgb_to_buf_dma(uint8_t Rpixel , uint8_t Gpixel, uint8_t Bpixel, uint16_t posX);
void ws2812_prepareValue (uint8_t r00, uint8_t g00, uint8_t b00,
    uint8_t r01, uint8_t g01, uint8_t b01,
    uint8_t r02, uint8_t g02, uint8_t b02,
    uint8_t r03, uint8_t g03, uint8_t b03,
    uint8_t r04, uint8_t g04, uint8_t b04,
    uint8_t r05, uint8_t g05, uint8_t b05,
    uint8_t r06, uint8_t g06, uint8_t b06,
    uint8_t r07, uint8_t g07, uint8_t b07,
    uint8_t r08, uint8_t g08, uint8_t b08,
    uint8_t r09, uint8_t g09, uint8_t b09,
    uint8_t r10, uint8_t g10, uint8_t b10,
    uint8_t r11, uint8_t g11, uint8_t b11);
void ws2812_set(uint8_t G, uint8_t B, uint8_t R, uint16_t len);
void ws2812_clear(uint16_t len);
void ws2812_setValue(void);
void ws2812_light(void);
void ws2812_test01(void);
void ws2812_test02(void);
void ws2812_test03(void);
void ws2812_test04(uint8_t col);
void ws2812_test05(void);
//--------------------------------------------------
#endif /* WS2812_H_ */
