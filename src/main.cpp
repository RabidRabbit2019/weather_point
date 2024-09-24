#include "trace.h"
#include "radiomodem.h"
#include "bme280.h"

#include "stm32f103x6.h"
#include <stdio.h>
#include <string.h>


extern volatile uint32_t g_milliseconds;

// us delay
void delay_ms( uint32_t a_ms ) {
  uint32_t a_from = g_milliseconds;
  while ( ((uint32_t)(g_milliseconds - a_from)) < a_ms ) {}
}

extern "C" {
extern uint32_t _sidata;
extern uint32_t _sdata;
extern uint32_t _edata;
extern uint32_t _sbss;
extern uint32_t _ebss;
}

const uint8_t g_key[8] = {0x01, 0x66, 0xAF, 0x2D, 0x08, 0x00, 0x00, 0xD8};

void run() {
  // copy initialized data
  uint32_t * v_ptr = (uint32_t *)&_sidata;
  for ( uint32_t * i = (uint32_t *)&_sdata; i < (uint32_t *)&_edata; ++i ) {
    *i = *v_ptr++;
  }
  // zero initialized data
  for ( uint32_t * i = (uint32_t *)&_sbss; i < (uint32_t *)&_ebss; ++i ) {
    *i = 0;
  }
  // at start clock source = HSI
  // configure RCC: 8 MHz HSE + PLL x8 = 64 MHz
  // enable HSE
  RCC->CR |= RCC_CR_HSEON;
  // wait for HSE starts
  while ( 0 == (RCC->CR & RCC_CR_HSERDY) ) {}
  // FLASH latency 2
  FLASH->ACR = FLASH_ACR_PRFTBE
             | FLASH_ACR_LATENCY_2
             ;
  // clock params: PLL = (HSE/1)*8, AHB = PLL/1, APB1 = PLL/2, APB2 = PLL/1
  RCC->CFGR = RCC_CFGR_SW_HSI
            | RCC_CFGR_HPRE_DIV1
            | RCC_CFGR_PPRE1_DIV2
            | RCC_CFGR_PPRE2_DIV1
            | RCC_CFGR_ADCPRE_DIV4
            | RCC_CFGR_PLLSRC
            | RCC_CFGR_PLLXTPRE_HSE
            | RCC_CFGR_PLLMULL8
            | RCC_CFGR_USBPRE
            | RCC_CFGR_MCO_NOCLOCK
            ;
  // enable PLL
  RCC->CR |= RCC_CR_PLLON;
  // wait for PLL starts
  while ( 0 == (RCC->CR & RCC_CR_PLLRDY) ) {}
  // switch clock source from HSI to PLL, it works because SW_HSI = 0
  RCC->CFGR |= RCC_CFGR_SW_PLL;
  // now clock at 64 MHz, AHB 64 MHz, APB1 32 MHz, APB2 32 MHz
  RCC->APB2ENR |= ( RCC_APB2ENR_AFIOEN );
  // enable clock for PIOA, PIOB, PIOC
  RCC->APB2ENR |= (RCC_APB2ENR_IOPAEN | RCC_APB2ENR_IOPBEN | RCC_APB2ENR_IOPCEN);
  // PIOC13 - output open-drain, low speed
  GPIOC->CRH = (GPIOC->CRH & ~(GPIO_CRH_MODE13_Msk | GPIO_CRH_CNF13_Msk))
               | GPIO_CRH_MODE13_1
               | GPIO_CRH_CNF13_0
               ;
  GPIOC->BSRR = GPIO_BSRR_BS13;
  // PIOA9 - output push-pull alternate fn USART1 TX
  GPIOA->CRH = (GPIOA->CRH & ~(GPIO_CRH_MODE9_Msk | GPIO_CRH_CNF9_Msk))
               | GPIO_CRH_MODE9_1
               | GPIO_CRH_CNF9_1
               ;
  // USART1 115200 8N1
  // 64E6 / 16 / (34+12/16) ~= 115108
  RCC->APB2ENR |= RCC_APB2ENR_USART1EN;
  USART1->SR = 0;
  USART1->BRR = (34 << USART_BRR_DIV_Mantissa_Pos) | (12 << USART_BRR_DIV_Fraction_Pos);
  USART1->CR3 = 0;
  USART1->CR2 = 0;
  USART1->CR1 = USART_CR1_TE
              | USART_CR1_UE
              ;
  // SysTick interrupt for each 1 millisecond (SysTick clocks by AHB/8 = 8MHz)
  SysTick_Config2( 8000 );
  //
  printf( "Hello from WeatherStation CKS32F103 based\n" );
  uint8_t v_buf[64];
  int v_rm_init = rm_init();
  printf( "radiomodem init %s\n", 0 == v_rm_init ? "OK" : "ERROR" );
  //
  bool v_bmp_init = false;
  for ( uint32_t v_from = g_milliseconds; !v_bmp_init && ((uint32_t)(g_milliseconds - v_from)) < 1000u; ) {
    v_bmp_init = BMP280_init();
    if ( !v_bmp_init ) {
      RCC->APB1RSTR = RCC_APB1RSTR_I2C1RST;
      delay_ms( 2 );
      RCC->APB1RSTR = 0;
      delay_ms( 5 );
    }
  }
  printf( "BMP{P|E}280 init %s\n", v_bmp_init ? "OK" : "ERROR" );
  if ( v_bmp_init ) {
    printf( "BM%c detected\n", BMP280_is_BME() ? 'E' : 'P' );
  }
  //
  for (;;) {
    if ( 0 == v_rm_init ) {
      uint8_t * v_ptr = v_buf;
      memset( v_buf, 0, sizeof(v_buf) );
      if ( 0 == rm_receive( v_buf, 1000 ) ) {
        for ( int i = 0; i < 8; ++i ) {
          for ( int j = 0; j < 8; ++j ) {
            printf( "%02X", *v_ptr++ );
          }
          printf( "\n" );
        }
        if ( rm_decode_request( v_buf, g_packet_length ) ) {
          printf( "decode OK\n" );
        } else {
          printf( "decode error\n" );
        }
        printf( "%u bytes, rssi = %u ----\n", g_packet_length, g_rssi );
      }
    }
    if ( v_bmp_init ) {
      int v_temp, v_press, v_hum;
      if ( BMP280_readMesure( &v_temp, &v_press, &v_hum ) ) {
        printf( "t=%d, p=%d, h=%d\n", v_temp, v_press, v_hum );
      }
    }
    //
    GPIOC->ODR ^= GPIO_ODR_ODR13;
  }
}
