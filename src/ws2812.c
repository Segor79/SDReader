#include "ws2812.h"
//----------------------------------------------------------------------------
extern TIM_HandleTypeDef htim2;

static void TIM_DMAPulseCplt(DMA_HandleTypeDef *hdma);
static void TIM_DMAPulseHalfCplt(DMA_HandleTypeDef *hdma);
//----------------------------------------------------------------------------
/// Timer handler
#if TIM_NUM == 1
#define TIM_HANDLE  htim1
#elif TIM_NUM == 2
#define TIM_HANDLE  htim2
#elif TIM_NUM == 3
#define TIM_HANDLE  htim3
#elif TIM_NUM == 4
#define TIM_HANDLE  htim4
#elif TIM_NUM == 5
#define TIM_HANDLE  htim5
#elif TIM_NUM == 8
#define TIM_HANDLE  htim8
#else
#warning If you shure, set TIM_HANDLE
#endif

//volatile ARGB_STATE ARGB_LOC_ST; ///< Buffer send status

//----------------------------------------------------------------------------
uint16_t BUF_DMA [ARRAY_LEN] = {0};
uint8_t rgb_temp[12][3];
//uint16_t DMA_BUF_TEMP[24];
//------------------------------------------------------------------
void ws2812_init(void)
{
  int i;
  for(i=DELAY_LEN;i<ARRAY_LEN;i++) BUF_DMA[i] = LOW;
	
    /* Auto-calculation! */
    uint32_t APBfq; // Clock freq
#ifdef APB1
    APBfq = HAL_RCC_GetPCLK1Freq();
    APBfq *= (RCC->CFGR & RCC_CFGR_PPRE1) == 0 ? 1 : 2;
#endif
#ifdef APB2
    APBfq = HAL_RCC_GetPCLK2Freq();
    APBfq *= (RCC->CFGR & RCC_CFGR_PPRE2) == 0 ? 1 : 2;
#endif
#ifdef WS2811S
    APBfq /= (uint32_t) (400 * 1000);  // 400 KHz - 2.5us
#else
    APBfq /= (uint32_t) (800 * 1000);  // 800 KHz - 1.25us
#endif
    TIM_HANDLE.Instance->PSC = 0;                        // dummy hardcode now
    TIM_HANDLE.Instance->ARR = (uint16_t) (APBfq - 1);   // set timer prescaler
    TIM_HANDLE.Instance->EGR = 1;                        // update timer registers
#if defined(WS2811F) || defined(WS2811S)
    PWM_HI = (u8_t) (APBfq * 0.48) - 1;     // Log.1 - 48% - 0.60us/1.2us
    PWM_LO = (u8_t) (APBfq * 0.20) - 1;     // Log.0 - 20% - 0.25us/0.5us
#endif
#ifdef WS2812
    PWM_HI = (u8_t) (APBfq * 0.56) - 1;     // Log.1 - 56% - 0.70us
    PWM_LO = (u8_t) (APBfq * 0.28) - 1;     // Log.0 - 28% - 0.35us
#endif
#ifdef SK6812
    PWM_HI = (u8_t) (APBfq * 0.48) - 1;     // Log.1 - 48% - 0.60us
    PWM_LO = (u8_t) (APBfq * 0.24) - 1;     // Log.0 - 24% - 0.30us
#endif

    TIM_CCxChannelCmd(TIM_HANDLE.Instance, TIM_CH, TIM_CCx_ENABLE); // Enable GPIO to IDLE state
//		ARGB_LOC_ST = ARGB_READY; // Set Ready Flag
    HAL_Delay(1); // Make some delay
	
}
//------------------------------------------------------------------
void ws2812_pixel_rgb_to_buf_dma(uint8_t Rpixel , uint8_t Gpixel, uint8_t Bpixel, uint16_t posX)
{
  volatile uint16_t i;
  for(i=0;i<8;i++)
  {
    if (BitIsSet(Rpixel,(7-i)) == 1)
    {
      BUF_DMA[DELAY_LEN+posX*24+i+8] = HIGH;
    }else
    {
      BUF_DMA[DELAY_LEN+posX*24+i+8] = LOW;
    }
    if (BitIsSet(Gpixel,(7-i)) == 1)
    {
      BUF_DMA[DELAY_LEN+posX*24+i+0] = HIGH;
    }else
    {
      BUF_DMA[DELAY_LEN+posX*24+i+0] = LOW;
    }
    if (BitIsSet(Bpixel,(7-i)) == 1)
    {
      BUF_DMA[DELAY_LEN+posX*24+i+16] = HIGH;
    }else
    {
      BUF_DMA[DELAY_LEN+posX*24+i+16] = LOW;
    }
  }
}
//------------------------------------------------------------------
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
    uint8_t r11, uint8_t g11, uint8_t b11)
  {
  rgb_temp[0][0]=r00; rgb_temp[0][1]=g00; rgb_temp[0][2]=b00;
  rgb_temp[1][0]=r01; rgb_temp[1][1]=g01; rgb_temp[1][2]=b01;
  rgb_temp[2][0]=r02; rgb_temp[2][1]=g02; rgb_temp[2][2]=b02;
  rgb_temp[3][0]=r03; rgb_temp[3][1]=g03; rgb_temp[3][2]=b03;
  rgb_temp[4][0]=r04; rgb_temp[4][1]=g04; rgb_temp[4][2]=b04;
  rgb_temp[5][0]=r05; rgb_temp[5][1]=g05; rgb_temp[5][2]=b05;
  rgb_temp[6][0]=r06; rgb_temp[6][1]=g06; rgb_temp[6][2]=b06;
  rgb_temp[7][0]=r07; rgb_temp[7][1]=g07; rgb_temp[7][2]=b07;
  rgb_temp[8][0]=r08; rgb_temp[8][1]=g08; rgb_temp[8][2]=b08;
  rgb_temp[9][0]=r09; rgb_temp[9][1]=g09; rgb_temp[9][2]=b09;
  rgb_temp[10][0]=r10;rgb_temp[10][1]=g10;rgb_temp[10][2]=b10;
  rgb_temp[11][0]=r11;rgb_temp[11][1]=g11;rgb_temp[11][2]=b11;
}
void ws2812_set(uint8_t G, uint8_t B, uint8_t R, uint16_t len){
	 uint8_t n=0;
  for(n=0;n<len;n++)
  {
		ws2812_pixel_rgb_to_buf_dma( G, B, R, n);
	}
	
}
void ws2812_clear(uint16_t len){
	 uint8_t n=0;
  for(n=0;n<len;n++)
  {
		ws2812_pixel_rgb_to_buf_dma( 0, 0, 0, n);
	}
	
}
//------------------------------------------------------------------
void ws2812_setValue(void)
{
  uint8_t n=0;
  for(n=0;n<12;n++)
  {
    ws2812_pixel_rgb_to_buf_dma( rgb_temp[0][0], rgb_temp[0][1], rgb_temp[0][2], n*12);
    ws2812_pixel_rgb_to_buf_dma( rgb_temp[1][0], rgb_temp[1][1], rgb_temp[1][2], n*12+1);
    ws2812_pixel_rgb_to_buf_dma( rgb_temp[2][0], rgb_temp[2][1], rgb_temp[2][2], n*12+2);
    ws2812_pixel_rgb_to_buf_dma( rgb_temp[3][0], rgb_temp[3][1], rgb_temp[3][2], n*12+3);
    ws2812_pixel_rgb_to_buf_dma( rgb_temp[4][0], rgb_temp[4][1], rgb_temp[4][2], n*12+4);
    ws2812_pixel_rgb_to_buf_dma( rgb_temp[5][0], rgb_temp[5][1], rgb_temp[5][2], n*12+5);
    ws2812_pixel_rgb_to_buf_dma( rgb_temp[6][0], rgb_temp[6][1], rgb_temp[6][2], n*12+6);
    ws2812_pixel_rgb_to_buf_dma( rgb_temp[7][0], rgb_temp[7][1], rgb_temp[7][2], n*12+7);
    ws2812_pixel_rgb_to_buf_dma( rgb_temp[8][0], rgb_temp[8][1], rgb_temp[8][2], n*12+8);
    ws2812_pixel_rgb_to_buf_dma( rgb_temp[9][0], rgb_temp[9][1], rgb_temp[9][2], n*12+9);
    ws2812_pixel_rgb_to_buf_dma(rgb_temp[10][0],rgb_temp[10][1],rgb_temp[10][2],n*12+10);
    ws2812_pixel_rgb_to_buf_dma(rgb_temp[11][0],rgb_temp[11][1],rgb_temp[11][2],n*12+11);
  }
}


//-----------------------------------------------  прерывание в конце пакета
static void TIM_DMAPulseCplt(DMA_HandleTypeDef *hdma) {
	
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_14, GPIO_PIN_RESET);			// для отладки
	
	TIM_HandleTypeDef *htim = (TIM_HandleTypeDef *) ((DMA_HandleTypeDef *) hdma)->Parent;
	

//	TIM_CHANNEL_STATE_SET(htim, TIM_CHANNEL_2, HAL_TIM_CHANNEL_STATE_READY);
	
	HAL_TIM_PWM_Stop_DMA(&htim2,TIM_CHANNEL_2);
	

        // STOP DMA:
#if TIM_CH == TIM_CHANNEL_1
        __HAL_TIM_DISABLE_DMA(htim, TIM_DMA_CC1);
        (void) HAL_DMA_Abort_IT(htim->hdma[TIM_DMA_ID_CC1]);
#endif
#if TIM_CH == TIM_CHANNEL_2
        __HAL_TIM_DISABLE_DMA(htim, TIM_DMA_CC2);
        (void) HAL_DMA_Abort_IT(htim->hdma[TIM_DMA_ID_CC2]);
#endif
#if TIM_CH == TIM_CHANNEL_3
        __HAL_TIM_DISABLE_DMA(htim, TIM_DMA_CC3);
        (void) HAL_DMA_Abort_IT(htim->hdma[TIM_DMA_ID_CC3]);
#endif
#if TIM_CH == TIM_CHANNEL_4
        __HAL_TIM_DISABLE_DMA(htim, TIM_DMA_CC4);
        (void) HAL_DMA_Abort_IT(htim->hdma[TIM_DMA_ID_CC4]);
#endif
        if (IS_TIM_BREAK_INSTANCE(htim->Instance) != RESET) {
            /* Disable the Main Output */
            __HAL_TIM_MOE_DISABLE(htim);
        }
        /* Disable the Peripheral */
        __HAL_TIM_DISABLE(htim);
        /* Set the TIM channel state */
        TIM_CHANNEL_STATE_SET(htim, TIM_CH, HAL_TIM_CHANNEL_STATE_READY);
//        ARGB_LOC_ST = ARGB_READY;
 
    htim->Channel = HAL_TIM_ACTIVE_CHANNEL_CLEARED;

	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_14, GPIO_PIN_SET);			// для отладки

}
//------------------------------------------------ прерывание в середине пакета
static void TIM_DMAPulseHalfCplt(DMA_HandleTypeDef *hdma) {
	

				HAL_GPIO_WritePin(GPIOC, GPIO_PIN_15, GPIO_PIN_RESET);			// для отладки


	
	
	


				HAL_GPIO_WritePin(GPIOC, GPIO_PIN_15, GPIO_PIN_SET);			// для отладки

}
//------------------------------------------------------------------
void ws2812_light(void)
{
//	HAL_StatusTypeDef DMA_Send_Stat = HAL_ERROR;
	HAL_StatusTypeDef DMA_Send_Stat;
	while (DMA_Send_Stat != HAL_OK) {
			if (TIM_CHANNEL_STATE_GET(&TIM_HANDLE, TIM_CH) == HAL_TIM_CHANNEL_STATE_BUSY) {
					DMA_Send_Stat = HAL_BUSY;
					continue;
			} else if (TIM_CHANNEL_STATE_GET(&TIM_HANDLE, TIM_CH) == HAL_TIM_CHANNEL_STATE_READY) {
					TIM_CHANNEL_STATE_SET(&TIM_HANDLE, TIM_CH, HAL_TIM_CHANNEL_STATE_BUSY);
			} else {
					DMA_Send_Stat = HAL_ERROR;
					continue;
			}
	
#if TIM_CH == TIM_CHANNEL_1
#define RGB_TIM_DMA_ID TIM_DMA_ID_CC1
#define RGB_TIM_DMA_CC TIM_DMA_CC1
#define RGB_TIM_CCR CCR1
#elif TIM_CH == TIM_CHANNEL_2
#define RGB_TIM_DMA_ID TIM_DMA_ID_CC2
#define RGB_TIM_DMA_CC TIM_DMA_CC2
#define RGB_TIM_CCR CCR2
#elif TIM_CH == TIM_CHANNEL_3
#define RGB_TIM_DMA_ID TIM_DMA_ID_CC3
#define RGB_TIM_DMA_CC TIM_DMA_CC3
#define RGB_TIM_CCR CCR3
#elif TIM_CH == TIM_CHANNEL_4
#define RGB_TIM_DMA_ID TIM_DMA_ID_CC4
#define RGB_TIM_DMA_CC TIM_DMA_CC4
#define RGB_TIM_CCR CCR4
#endif
	
	TIM_HANDLE.hdma[RGB_TIM_DMA_ID]->XferCpltCallback = TIM_DMAPulseCplt;
  TIM_HANDLE.hdma[RGB_TIM_DMA_ID]->XferHalfCpltCallback = TIM_DMAPulseHalfCplt;
  TIM_HANDLE.hdma[RGB_TIM_DMA_ID]->XferErrorCallback = TIM_DMAError;
	
	
  HAL_TIM_PWM_Start_DMA(&TIM_HANDLE,TIM_CHANNEL_2,(uint32_t*)&BUF_DMA,ARRAY_LEN);
	
	
			__HAL_TIM_ENABLE_DMA(&TIM_HANDLE, RGB_TIM_DMA_CC);
		if (IS_TIM_BREAK_INSTANCE(TIM_HANDLE.Instance) != RESET)
				__HAL_TIM_MOE_ENABLE(&TIM_HANDLE);
		if (IS_TIM_SLAVE_INSTANCE(TIM_HANDLE.Instance)) {
				uint32_t tmpsmcr = TIM_HANDLE.Instance->SMCR & TIM_SMCR_SMS;
				if (!IS_TIM_SLAVEMODE_TRIGGER_ENABLED(tmpsmcr))
						__HAL_TIM_ENABLE(&TIM_HANDLE);
		} else
				__HAL_TIM_ENABLE(&TIM_HANDLE);
		DMA_Send_Stat = HAL_OK;
	}
}
//----------------------------------------------------------------------------
