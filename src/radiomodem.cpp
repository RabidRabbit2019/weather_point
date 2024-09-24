#include "radiomodem.h"
#include "stm32f103x6.h"
#include "SI443X.h"
#include "datadesc.h"
#include "crc16.h"

#include <stdio.h>
#include <string.h>


extern volatile uint32_t g_milliseconds;

void delay_ms( uint32_t a_ms );


#define SETTINGS_REG_PACK(a,b)  ((a<<8)|b)
#define SETTINGS_REG_NUM(a)     ((a>>8)&0xFF)
#define SETTINGS_REG_VAL(a)     (a&0xFF)


#define g_req_pass 2130402695u
#define g_ans_pass 480466474u

// значения регистров для настройки радиомодема

const uint16_t g_registers_setup[] =
{
  SETTINGS_REG_PACK(0x1C,0x05)
, SETTINGS_REG_PACK(0x1D,0x44)
, SETTINGS_REG_PACK(0x1E,0x0A)
, SETTINGS_REG_PACK(0x1F,0x00)
, SETTINGS_REG_PACK(0x20,0xA1)
, SETTINGS_REG_PACK(0x21,0x20)
, SETTINGS_REG_PACK(0x22,0x4E)
, SETTINGS_REG_PACK(0x23,0xA5)
, SETTINGS_REG_PACK(0x24,0x10)
, SETTINGS_REG_PACK(0x25,0x24)
//, SETTINGS_REG_PACK(0x27,0x00)
, SETTINGS_REG_PACK(0x2A,0x20)
, SETTINGS_REG_PACK(0x2C,0x28)
, SETTINGS_REG_PACK(0x2D,0x82)
, SETTINGS_REG_PACK(0x2E,0x2A)
, SETTINGS_REG_PACK(0x30,0x8C)
, SETTINGS_REG_PACK(0x32,0x00)
, SETTINGS_REG_PACK(0x33,0x02)
, SETTINGS_REG_PACK(0x34,0x08)
, SETTINGS_REG_PACK(0x35,0x22)
, SETTINGS_REG_PACK(0x36,0x2D)
, SETTINGS_REG_PACK(0x37,0xD4)
, SETTINGS_REG_PACK(0x38,0x00)
, SETTINGS_REG_PACK(0x39,0x00)
, SETTINGS_REG_PACK(0x3A,0x00)
, SETTINGS_REG_PACK(0x3B,0x00)
, SETTINGS_REG_PACK(0x3C,0x00)
, SETTINGS_REG_PACK(0x3D,0x00)
, SETTINGS_REG_PACK(0x3E,0x30)
, SETTINGS_REG_PACK(0x3F,0x00)
, SETTINGS_REG_PACK(0x40,0x00)
, SETTINGS_REG_PACK(0x41,0x00)
, SETTINGS_REG_PACK(0x42,0x00)
, SETTINGS_REG_PACK(0x43,0xFF)
, SETTINGS_REG_PACK(0x44,0xFF)
, SETTINGS_REG_PACK(0x45,0xFF)
, SETTINGS_REG_PACK(0x46,0xFF)
, SETTINGS_REG_PACK(0x58,0x80)
, SETTINGS_REG_PACK(0x69,0x60)
, SETTINGS_REG_PACK(0x6E,0x4E)
, SETTINGS_REG_PACK(0x6F,0xA5)
, SETTINGS_REG_PACK(0x70,0x2C)
, SETTINGS_REG_PACK(0x71,0x23)
, SETTINGS_REG_PACK(0x72,0x48)
, SETTINGS_REG_PACK(0x75,0x53)
, SETTINGS_REG_PACK(0x76,0x5F)
, SETTINGS_REG_PACK(0x77,0x00)
, SETTINGS_REG_PACK(0,0) // маркер
};


const uint16_t g_registers_after_send[] =
{
  SETTINGS_REG_PACK(SI443X_REG_INTERRUPT_ENABLE_2,0)
, SETTINGS_REG_PACK(0x71,0x23)
, SETTINGS_REG_PACK(0x75,0x53)
, SETTINGS_REG_PACK(0x76,0x5F)
, SETTINGS_REG_PACK(0x77,0x00)
, SETTINGS_REG_PACK(0,0) // маркер
};


uint8_t g_rssi = 0;
uint8_t g_packet_length = 0;


int rm_spi_xfer( uint8_t a_dr, uint8_t * a_dst ) {
  //
  SPI1->DR; // read DR for reset RDRE
  SPI1->DR = a_dr; // start transfer
  uint32_t tcv = g_milliseconds;
  while ( ((uint32_t)(g_milliseconds - tcv)) < 2u ) {
    if ( 0 != (SPI1->SR & SPI_SR_RXNE) ) {
      // принято
      *a_dst = SPI1->DR;
      return ERR_NONE;
    }
  }
  return ERR_SPI;
}


int rm_read_register( uint8_t a_reg_num, uint8_t * a_dst ) {
  int v_rc = ERR_NONE;
  
  GPIOA->BSRR = GPIO_BSRR_BR4; // nSEL = 0
  
  if ( ERR_NONE != rm_spi_xfer( a_reg_num, a_dst )
    || ERR_NONE != rm_spi_xfer( 0, a_dst ) ) {
    v_rc = ERR_SPI;
  }

  GPIOA->BSRR = GPIO_BSRR_BS4; // nSEL = 1
  
  return v_rc;
}


int rm_write_register( uint8_t a_reg_num, uint8_t a_src ) {
  uint8_t v_dummy;
  
  int v_rc = ERR_NONE;
  
  GPIOA->BSRR = GPIO_BSRR_BR4; // nSEL = 0
  
  if ( ERR_NONE != rm_spi_xfer( a_reg_num | RM_TDR_WRITE_FLAG, &v_dummy )
    || ERR_NONE != rm_spi_xfer( a_src, &v_dummy ) ) {
    v_rc = ERR_SPI;
  }

  GPIOA->BSRR = GPIO_BSRR_BS4; // nSEL = 1
  
  return v_rc;
}


int rm_write_FIFO( uint8_t * a_src, uint32_t a_len )
{
    for ( size_t i = 0; i < a_len; ++i ) {
      if ( ERR_NONE != rm_write_register( SI443X_REG_FIFO_ACCESS, a_src[i] ) ) {
        printf( "FIFO write error\n" );
        return ERR_SPI;
      }
    }
    //
    return ERR_NONE;
}


int rm_read_FIFO( uint8_t * a_dst, uint32_t a_len )
{
    for ( size_t i = 0; i < a_len; ++i ) {
      if ( ERR_NONE != rm_read_register( SI443X_REG_FIFO_ACCESS, &a_dst[i] ) ) {
        printf( "FIFO read error\n" );
        return ERR_SPI;
      }
    }
    //
    return ERR_NONE;
}


int rm_set_reg_value(uint8_t reg, uint8_t value, uint8_t msb, uint8_t lsb)
{
  uint8_t currentValue = 0;
  if ( ERR_NONE != rm_read_register(reg, &currentValue) )
  {
      printf( "error set reg value\n" );
      return ERR_SPI;
  }
  uint8_t mask = ~((0xFF << (msb + 1)) | (0xFF >> (8 - lsb)));
  uint8_t newValue = (currentValue & ~mask) | (value & mask);
  return rm_write_register(reg, newValue);
}


int rm_get_reg_value(uint8_t reg, uint8_t * a_dst, uint8_t msb, uint8_t lsb)
{
  uint8_t rawValue = 0;
  if ( ERR_NONE != rm_read_register(reg, &rawValue) )
  {
      printf( "error get reg value\n" );
      return ERR_SPI;
  }
  *a_dst = rawValue & ((0xFF << lsb) & (0xFF >> (7 - msb)));
  return ERR_NONE;
}


int rm_find() {
  uint8_t v_device_type;
  uint8_t v_device_version;
  
  //
  for ( int i = 0; i < 10; i++ ) {
    // reset Si4432
    GPIOA->BSRR = GPIO_BSRR_BS2;
    delay_ms(5);
    GPIOA->BSRR = GPIO_BSRR_BR2;
    delay_ms(100);
    rm_write_register(SI443X_REG_OP_FUNC_CONTROL_1, SI443X_SOFTWARE_RESET);
    delay_ms(200);
    //
    if ( ERR_NONE == rm_read_register(SI443X_REG_DEVICE_TYPE, &v_device_type)
      && ERR_NONE == rm_read_register(SI443X_REG_DEVICE_VERSION, &v_device_version)
      && SI443X_DEVICE_TYPE == v_device_type
      && SI443X_DEVICE_VERSION == v_device_version ) {
      return ERR_NONE;
    } else {
      printf("v_device_type: %u, v_device_version: %u\n", v_device_type, v_device_version);
    }
  }

  return ERR_CHIP_NOT_FOUND;
}


int rm_clear_irq_flags()
{
  uint8_t st;
  if ( ERR_NONE == rm_read_register( SI443X_REG_INTERRUPT_STATUS_1, &st )
    && ERR_NONE == rm_read_register( SI443X_REG_INTERRUPT_STATUS_2, &st ) ) {
    return ERR_NONE;
  } else {
    return ERR_SPI;
  }
}


int rm_apply_register_settings(const uint16_t * a_from)
{
    //
    for ( int i = 0; 0 != a_from[i]; ++i )
    {
        uint16_t v = a_from[i];
        
        //
        if ( ERR_NONE != rm_write_register( SETTINGS_REG_NUM(v), SETTINGS_REG_VAL(v) ) )
        {
            //
            printf(
                "%s - error write value 0x%02X into register 0x%02X"
                , __func__
                , SETTINGS_REG_VAL(v)
                , SETTINGS_REG_NUM(v)
                );
            //
            return ERR_SPI;
        }
    }
    //
    return rm_set_reg_value(SI443X_REG_TX_POWER, 6, 2, 0);
}


int rm_init() {
  // init GPIO
  // PIOA2 - output push-pull 2 MHz RadioModem RESET
  // PIOA3 - input pull-up RadioModem INT
  // PIOA4 - output push-pull 2 MHz RadioModem CS
  // PIOA5 - output push-pull 10 MHz alternate fn SPI1 SCK
  // PIOA6 - input pull-up SPI1 MISO
  // PIOA7 - output push-pull 10 MHz alternate fn SPI1 MOSI
  GPIOA->CRL = (GPIOA->CRL & ~( GPIO_CRL_MODE2 | GPIO_CRL_CNF2
                              | GPIO_CRL_MODE3 | GPIO_CRL_CNF3
                              | GPIO_CRL_MODE4 | GPIO_CRL_CNF4
                              | GPIO_CRL_MODE5 | GPIO_CRL_CNF5
                              | GPIO_CRL_MODE6 | GPIO_CRL_CNF6
                              | GPIO_CRL_MODE7 | GPIO_CRL_CNF7
                              ))
               | GPIO_CRL_MODE2_1
               | GPIO_CRL_CNF3_1
               | GPIO_CRL_MODE4_1
               | GPIO_CRL_MODE5_0 | GPIO_CRL_CNF5_1
               | GPIO_CRL_CNF6_1
               | GPIO_CRL_MODE7_0 | GPIO_CRL_CNF7_1
               ;
  // 
  GPIOA->BSRR = GPIO_BSRR_BR2 | GPIO_BSRR_BS3 | GPIO_BSRR_BS4 | GPIO_BSRR_BS6;
  GPIOA->BSRR = GPIO_BSRR_BS4;
  // enable SPI1 clock
  RCC->APB2ENR |= RCC_APB2ENR_SPI1EN;
  // init SPI1
  SPI1->CR1 = SPI_CR1_BR_1  // 64M / 8 = SCK 8 MHz
            | SPI_CR1_SSM
            | SPI_CR1_SSI
            | SPI_CR1_MSTR
            | SPI_CR1_SPE
            ;
  //
  int v_rc = rm_find();
  if ( ERR_NONE != v_rc ) {
    return v_rc;
  }
  //
  v_rc = rm_clear_irq_flags();
  if ( ERR_NONE != v_rc ) {
    return v_rc;
  }
  //
  return rm_apply_register_settings(g_registers_setup);
}


int rm_start_receive()
{
  if (
      // clear Rx FIFO
      ERR_NONE == rm_set_reg_value(SI443X_REG_OP_FUNC_CONTROL_2, SI443X_RX_FIFO_RESET, 1, 1)
   && ERR_NONE == rm_set_reg_value(SI443X_REG_OP_FUNC_CONTROL_2, SI443X_RX_FIFO_CLEAR, 1, 1)

      // set interrupt mapping
   && ERR_NONE == rm_write_register(SI443X_REG_INTERRUPT_ENABLE_1, SI443X_VALID_PACKET_RECEIVED_ENABLED | SI443X_CRC_ERROR_ENABLED)
   && ERR_NONE == rm_write_register(SI443X_REG_INTERRUPT_ENABLE_2, SI443X_SYNC_WORD_DETECTED_ENABLED)

      // set mode to receive
   && ERR_NONE == rm_write_register(SI443X_REG_OP_FUNC_CONTROL_1, SI443X_RX_ON | SI443X_XTAL_ON)

      // clear interrupt flags
   && ERR_NONE == rm_clear_irq_flags()
   ) {
     return ERR_NONE;
   } else {
     return ERR_SPI;
   }
}


int rm_get_packet_length(uint8_t * a_dst)
{
  return rm_read_register(SI443X_REG_RECEIVED_PACKET_LENGTH, a_dst);
}


int rm_read_data(uint8_t * a_dst)
{
  g_packet_length = 0;
  
  // clear interrupt flags
  if ( ERR_NONE != rm_clear_irq_flags() ) {
    return ERR_SPI;
  }

  // get packet length
  if ( ERR_NONE != rm_get_packet_length(&g_packet_length) )
  {
    return ERR_SPI;
  }

  // read packet data
  return rm_read_FIFO(a_dst, g_packet_length);
}


int rm_receive(uint8_t * a_dst, uint32_t timeout)
{
  if ( timeout < 3 ) {
    timeout = 3;
  }

  // start reception
  if ( ERR_NONE != rm_start_receive() ) {
    return ERR_SPI;
  }

  // wait for transmission end or timeout
  uint32_t tcv = g_milliseconds;
  for (;;) {
    while( 0 != (GPIOA->IDR & GPIO_IDR_IDR3) ) {
      if( ((uint32_t)(g_milliseconds - tcv)) >= timeout ) {
        rm_clear_irq_flags();
        return ERR_RX_TIMEOUT;
      } else {
        delay_ms( 1 );
      }
    }
    uint8_t v_flags2 = 0;
    if ( ERR_NONE != rm_read_register(SI443X_REG_INTERRUPT_STATUS_2, &v_flags2) ) {
      return ERR_SPI;
    }
    if ( 0 != (SI443X_SYNC_WORD_DETECTED_INTERRUPT & v_flags2) ) {
      if ( ERR_NONE != rm_read_register(SI443X_REG_RSSI, &g_rssi) ) {
        return ERR_SPI;
      }
    }
    uint8_t v_flags1 = 0;
    if ( ERR_NONE != rm_read_register(SI443X_REG_INTERRUPT_STATUS_1, &v_flags1) ) {
      return ERR_SPI;
    }
    if ( 0 != (SI443X_CRC_ERROR_INTERRUPT & v_flags1) ) {
      return ERR_RX_CRC;
    }
    if ( 0 != (SI443X_VALID_PACKET_RECEIVED_INTERRUPT & v_flags1) ) {
      break;
    }
  }
  
  // read packet data
  if ( ERR_NONE != rm_read_data( a_dst ) ) {
    return ERR_SPI;
  }

  // clear interrupt flags
  rm_clear_irq_flags();

  return ERR_NONE;
}


int rm_start_transmit(uint8_t* data, uint32_t len) {
  if (
    // clear Tx FIFO
       ERR_NONE == rm_set_reg_value(SI443X_REG_OP_FUNC_CONTROL_2, SI443X_TX_FIFO_RESET, 0, 0)
    && ERR_NONE == rm_set_reg_value(SI443X_REG_OP_FUNC_CONTROL_2, SI443X_TX_FIFO_CLEAR, 0, 0)

    // set packet length
    && ERR_NONE == rm_write_register(SI443X_REG_TRANSMIT_PACKET_LENGTH, len)

    // write packet to FIFO
    && ERR_NONE == rm_write_FIFO(data, len)

    // set interrupt mapping
    && ERR_NONE == rm_write_register(SI443X_REG_INTERRUPT_ENABLE_1, SI443X_PACKET_SENT_ENABLED)
    && ERR_NONE == rm_write_register(SI443X_REG_INTERRUPT_ENABLE_2, 0)

    // clear interrupt flags
    && ERR_NONE == rm_clear_irq_flags()

    // set mode to transmit
    && ERR_NONE == rm_write_register(SI443X_REG_OP_FUNC_CONTROL_1, SI443X_TX_ON | SI443X_XTAL_ON)
    ) {
    return ERR_NONE;
  } else {
    return ERR_SPI;
  }
}


int rm_transmit(uint8_t* data, uint32_t len) {
  // calculate timeout in milliseconds (500 % of expected time-on-air)
  unsigned int timeout = (len * 8 * 1000 / RM_BIT_RATE) * 5;
  if ( timeout < 5 ) {
    timeout = 5;
  }
  if ( timeout > 10000 ) {
    timeout = 10000;
  }

  // start transmission
  if ( ERR_NONE != rm_start_transmit(data, len) ) {
    return ERR_SPI;
  }

  // wait for transmission end or timeout
  uint32_t tcv = g_milliseconds;
  while( 0 != (GPIOA->IDR & GPIO_IDR_IDR3) ) {
    if( ((uint32_t)(g_milliseconds - tcv)) >= timeout ) {
      rm_clear_irq_flags();
      return ERR_TX_TIMEOUT;
    } else {
      delay_ms( 1 );
    }
  }

  // clear interrupt flags
  rm_clear_irq_flags();

  // the next transmission will timeout without the following
  rm_apply_register_settings(g_registers_after_send);

  return ERR_NONE;
}


//
uint32_t rm_gen_ans_key() {
  uint32_t v_result = 0;
  return v_result;
}


#define le16toh(x) (uint16_t)(x)
#define le32toh(x) (uint32_t)(x)


// буфер (без учёта header.msgOnce) расшифровывается
// далее сверяется crc16
bool rm_decode_request( uint8_t * a_req, size_t a_req_len) {
    // проверка размера пакета
    if ( a_req_len < sizeof(cmdSimple) ) {
      printf( "[%d] %s invalid packet length '%u', too short\n", __LINE__, __func__, (unsigned int)a_req_len );
      return false;
    }
    // доступ к одноразовому ключу
    rs485Header * v_header = (rs485Header *)a_req;
    // размер пакета без учёта одноразового ключа
    size_t v_req_size = a_req_len - sizeof(rs485Header::msgOnce);
    // дешифруем буфер
    cypher( 
        a_req + sizeof(rs485Header::msgOnce)
      , v_req_size
      , g_req_pass
      , v_header->msgOnce
      );
    // проверяем crc16 (последние 2 байта открытого текста
    uint16_t v_expected_crc = crc16(a_req, a_req_len - sizeof(rs485Footer));
    rs485Footer * v_footer = (rs485Footer *)(a_req + (a_req_len - sizeof(rs485Footer)));
    if ( v_expected_crc != le16toh(v_footer->crc) ) {
      printf(
          "[%d] %s invalid crc '%04X', expected '%04X'\n"
        , __LINE__
        , __func__
        , v_footer->crc
        , v_expected_crc
        );
      return false;
    }
    //
    return true;
}
