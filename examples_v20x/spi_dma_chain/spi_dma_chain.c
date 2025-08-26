#include "ch32fun.h"
#include <stdio.h>
#include <string.h>

#define NODE_ID 0xaa
#define USE_EXTI 1
#define SPI_TRANSMISSION_TIMEOUT 1000

#define DMA_BASE_ADDR(n)  (DMA1_Channel1_BASE+(0x14*(n-1)))
#define DMA1_CH(n) ((DMA_Channel_TypeDef *)DMA_BASE_ADDR(n))
#define DMA1_IRQ(x) DMA1_Channel1_IRQn + x - 1

#define _EXTI_N(x) EXTI##x##_IRQHandler(void)
#define EXTI_N(x) _EXTI_N(x)

#define SPI1_CS1 PA4
#define SPI2_CS1 PB12

#define GET_PIN(x) ((x) - ((x/16) * 16))
#define SPI_SLAVE_CS_PIN GET_PIN(SPI2_CS1)

#define PIN_TO_EXTI(pin) (pin % 16)

#define SPI_Master 1
#define SPI_Slave 0

#define DMA_In 0
#define DMA_Out 1

enum SPITransfer {
  SPI_IDLE,
  SPI_POLLING,
  SPI_IN_PROGRESS,
  SPI_DONE,
};

#define PACKET_SIZE 10
#define BUFFER_SIZE PACKET_SIZE*32

typedef struct {
	volatile uint8_t in[BUFFER_SIZE];
	volatile uint8_t out[BUFFER_SIZE];
	uint32_t pos_out; // Position in the array for the next packet to be placed at
	uint32_t pos_in;
	uint32_t sending_pos; // Position we are actually reading data from during send
	uint32_t receiving_pos; // or writing to during receive
  uint32_t to_send;
} buffer_t;

typedef struct {
  volatile enum SPITransfer transmission;
  SPI_TypeDef * addr;
  DMA_Channel_TypeDef * dma_in;
  DMA_Channel_TypeDef * dma_out;
  uint32_t cs_pin;
  buffer_t * buffer;
  uint8_t next_node_ID;
  uint16_t message_number;
  uint8_t incoming_message[PACKET_SIZE];
} spi_t;

volatile uint8_t message_received = 0;
volatile enum SPITransfer spi1_transmission = SPI_IDLE;
volatile enum SPITransfer spi2_transmission = SPI_IDLE;

buffer_t buffer;
buffer_t buffer_spi1;
buffer_t buffer_spi2;

spi_t spi1 = {
  SPI_IDLE,
  SPI1,
  DMA1_Channel2,
  DMA1_Channel3,
  SPI1_CS1,
  &buffer_spi1,
};

spi_t spi2 = {
  SPI_IDLE,
  SPI2,
  DMA1_Channel4,
  DMA1_Channel5,
  SPI2_CS1,
  &buffer_spi2,
};

spi_t* spi_master = &spi1;
spi_t* spi_slave = &spi2;

uint32_t timer = 0;
volatile uint8_t incomming_packet = 0;

void process_incoming(spi_t * spi) {
  printf("Processing incoming: %02x %02x %02x %02x %02x %02x %02x %02x\n", spi->buffer->in[spi->buffer->pos_in], spi->buffer->in[spi->buffer->pos_in + 1], spi->buffer->in[spi->buffer->pos_in + 2], spi->buffer->in[spi->buffer->pos_in + 3], spi->buffer->in[spi->buffer->pos_in + 4], spi->buffer->in[spi->buffer->pos_in + 5], spi->buffer->in[spi->buffer->pos_in + 6], spi->buffer->in[spi->buffer->pos_in + 7]);
  // printf("%d - %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x\n", spi->buffer->pos_in, spi->buffer->in[spi->buffer->pos_in], spi->buffer->in[spi->buffer->pos_in + 1], spi->buffer->in[spi->buffer->pos_in + 2], spi->buffer->in[spi->buffer->pos_in + 3], spi->buffer->in[spi->buffer->pos_in + 4], spi->buffer->in[spi->buffer->pos_in + 5], spi->buffer->in[spi->buffer->pos_in + 6], spi->buffer->in[spi->buffer->pos_in + 7], spi->buffer->in[spi->buffer->pos_in + 8], spi->buffer->in[spi->buffer->pos_in + 9]);
  uint8_t inc_id = spi->buffer->in[spi->buffer->pos_in] & 0xff;
	if (inc_id == NODE_ID || inc_id == 0 || inc_id == 0xff) {
		if (message_received == 0) memcpy(spi->incoming_message, &spi->buffer->in[spi->buffer->pos_in], PACKET_SIZE);
    if (inc_id == 0 && spi->next_node_ID == 0) spi->next_node_ID = spi->incoming_message[1];
	} else {
		memcpy(&spi->buffer->out[spi->buffer->pos_out], &spi->buffer->in[spi->buffer->pos_in], PACKET_SIZE);
		if ((spi->buffer->pos_out += PACKET_SIZE) >= BUFFER_SIZE) spi->buffer->pos_out = 0;
	}
	if ((spi->buffer->pos_in += PACKET_SIZE) >= BUFFER_SIZE) spi->buffer->pos_in = 0;
  memset(spi->buffer->in, 0, PACKET_SIZE);
}

void DMA1_Channel2_IRQHandler( void ) __attribute__((interrupt));
void DMA1_Channel2_IRQHandler( void ) 
{
  printf("\nCH2 INTERRUPT\n");
	// Backup flags.
	volatile int intfr = DMA1->INTFR;
	do
	{
		// Clear all possible flags.
		DMA1->INTFCR = DMA1_IT_GL2;
		if( intfr & DMA1_IT_TC2 )
		{
			process_incoming(&spi1);
			DMA1_CH(2)->MADDR = (uint32_t)&spi1.buffer->in[spi1.buffer->pos_in];
		}
		// Can abort send with  DMA1_Channel3->CFGR &= ~DMA_Mode_Circular;
		intfr = DMA1->INTFR;
	} while( intfr );
}

void DMA1_Channel3_IRQHandler( void ) __attribute__((interrupt));
void DMA1_Channel3_IRQHandler( void ) 
{
  printf("\nCH3 INTERRUPT\n");
	volatile int intfr = DMA1->INTFR;
	do
	{
		DMA1->INTFCR = DMA1_IT_GL3;
		if( intfr & DMA1_IT_TC3 )
		{
			if (spi1.buffer->sending_pos != spi1.buffer->pos_out) {
        printf("There is something to send %ld - %ld\n", spi1.buffer->sending_pos, spi1.buffer->pos_out);
        DMA1_CH(3)->CFGR &= ~DMA_CFGR1_EN;
				if (spi1.buffer->sending_pos > spi1.buffer->pos_out) {
					DMA1_CH(3)->CNTR = (BUFFER_SIZE - spi1.buffer->sending_pos) + spi1.buffer->pos_out;
				} else {
					DMA1_CH(3)->CNTR = spi1.buffer->pos_out - spi1.buffer->sending_pos;
				}
				DMA1_CH(3)->CFGR |= DMA_CFGR1_EN;
				spi1.buffer->sending_pos = spi1.buffer->pos_out;
			}
		}
		intfr = DMA1->INTFR;
	} while( intfr );
}

void DMA1_Channel4_IRQHandler( void ) __attribute__((interrupt));
void DMA1_Channel4_IRQHandler( void ) 
{
  printf("\nCH4 INTERRUPT\n");
	volatile int intfr = DMA1->INTFR;
	do
	{
		DMA1->INTFCR = DMA1_IT_GL4;
		if( intfr & DMA1_IT_TC4 )
		{
			process_incoming(&spi2);
			DMA1_CH(4)->MADDR = (uint32_t)&spi2.buffer->in[spi2.buffer->pos_in];
		}
		intfr = DMA1->INTFR;
	} while( intfr );
}

void DMA1_Channel5_IRQHandler( void ) __attribute__((interrupt));
void DMA1_Channel5_IRQHandler( void ) 
{
	printf("\nCH5 INTERRUPT\n");
	volatile int intfr = DMA1->INTFR;
	do
	{
		DMA1->INTFCR = DMA1_IT_GL5;
		if( intfr & DMA1_IT_TC5 )
		{
			if (spi2.buffer->sending_pos != spi2.buffer->pos_out) {
        printf("There is something to send %ld - %ld\n", spi2.buffer->sending_pos, spi2.buffer->pos_out);
        DMA1_CH(5)->CFGR &= ~DMA_CFGR1_EN;
				if (spi2.buffer->sending_pos > spi2.buffer->pos_out) {
					DMA1_CH(5)->CNTR = (BUFFER_SIZE - spi2.buffer->sending_pos) + spi2.buffer->pos_out;
				} else {
					DMA1_CH(5)->CNTR = spi2.buffer->pos_out - spi2.buffer->sending_pos;
				}
				DMA1_CH(5)->CFGR |= DMA_CFGR1_EN;
				spi2.buffer->sending_pos = spi2.buffer->pos_out;
			} else {
        spi2.transmission = SPI_DONE;
      }
		}
		intfr = DMA1->INTFR;
	} while( intfr );
}

static inline void spi_nodma_transmission(spi_t * spi) {
  volatile uint32_t timer1 = SysTick->CNT;
  // printf("nodma\n");
  // volatile uint8_t local_buffer[20];
  uint8_t master = ((spi->addr->CTLR1 & 0x4) == 0x4);
  uint32_t timeout = SPI_TRANSMISSION_TIMEOUT;
  
  uint8_t to_receive = PACKET_SIZE;
  while (timeout--) {
    // If SPI slave, we only run this function in interrupt
    
    // If SPI master, we only run while there is something to send, or until timeout if POLLING
    if (master && spi->buffer->to_send && spi->transmission != SPI_POLLING) break;

    if ((spi->addr->STATR & SPI_STATR_TXE) && spi->buffer->to_send) {
      // Write byte from buffer
      spi->addr->DATAR = spi->buffer->out[spi->buffer->sending_pos++];
      spi->buffer->to_send--;
      timeout = SPI_TRANSMISSION_TIMEOUT; // Update timeout
    }
    if ((spi->addr->STATR & SPI_STATR_RXNE) && spi->buffer->receiving_pos < BUFFER_SIZE) {
      
      // local_buffer[spi->buffer->receiving_pos++] = spi->addr->DATAR;
      spi->buffer->in[spi->buffer->receiving_pos++] = spi->addr->DATAR;
      if (to_receive) to_receive--;
      timeout = SPI_TRANSMISSION_TIMEOUT;
    }
    if (!master && funDigitalRead(spi->cs_pin)) {
      // printf("CS pin HIGH, breaking\n");
      break;
    }
  }

  // printf("to_receive = %d, timeout = %d\n", to_receive, timeout);
  EXTI->INTFR = 1 << (spi->cs_pin % 16);
  
  // If we actually received anything, process incoming buffer
  if (to_receive != PACKET_SIZE) {
    // printf("%02x %02x %02x %02x %02x %02x %02x %02x %02x %02x\n", local_buffer[0], local_buffer[1], local_buffer[2], local_buffer[3], local_buffer[4], local_buffer[5], local_buffer[6], local_buffer[7], local_buffer[8], local_buffer[9]);
    process_incoming(&spi2);
    spi->buffer->receiving_pos = 0; // In nodma mode we don't need circular buffer for receive;
    spi->buffer->pos_in = 0; // In nodma mode we don't need circular buffer for receive;
    // spi->addr->DATAR; // Clear buffer?
  }
  uint32_t timer2 = SysTick->CNT;
  printf("interrupt took %ld ticks, %ldus\n", (timer2 - timer1), (timer2 - timer1)/DELAY_US_TIME);
}

__attribute__((interrupt)) void EXTI0_IRQHandler( void ) { spi_nodma_transmission(spi_slave); }
__attribute__((interrupt)) void EXTI1_IRQHandler( void ) { spi_nodma_transmission(spi_slave); }
__attribute__((interrupt)) void EXTI2_IRQHandler( void ) { spi_nodma_transmission(spi_slave); }
__attribute__((interrupt)) void EXTI3_IRQHandler( void ) { spi_nodma_transmission(spi_slave); }
__attribute__((interrupt)) void EXTI4_IRQHandler( void ) { spi_nodma_transmission(spi_slave); }
__attribute__((interrupt)) void EXTI9_5_IRQHandler( void ) { spi_nodma_transmission(spi_slave); }
__attribute__((interrupt)) void EXTI15_10_IRQHandler( void ) { spi_nodma_transmission(spi_slave); }

void SPI1_IRQHandler(void) __attribute__((interrupt));
void SPI1_IRQHandler(void) {
  printf("SPI1 IRQ %04x\n", SPI1->STATR);
  if (SPI1->STATR & SPI_STATR_RXNE) {
    // printf("%02x ", SPI1->DATAR);
    SPI1->CTLR2 &= ~SPI_CTLR2_RXNEIE;
    incomming_packet = 1;
    // Starting receiving new packet
    // if (spi1.buffer->receiving_pos >= spi1.buffer->pos_in) {
    //   spi1.buffer->pos_in += PACKET_SIZE;
    // }
    // spi1.buffer->in[spi1.buffer->receiving_pos] = SPI1->DATAR;
    // for( uint8_t i = 0; i < 8; i++) printf("%02x\n", SPI1->DATAR);
    // if (spi1.buffer->receiving_pos++ >= spi1.buffer->pos_in) {
    //   process_incoming(&spi1);
    // } 
  }
  if ((SPI1->STATR & SPI_STATR_TXE) && (SPI1->CTLR2 & SPI_CTLR2_TXEIE)) {
    // Write byte from buffer
    SPI1->DATAR = spi1.buffer->out[spi1.buffer->sending_pos];
    // If we've sent everything there was, disable TX interrupt
    if (spi1.buffer->sending_pos++ >= spi1.buffer->pos_out) SPI1->CTLR2 &= ~SPI_CTLR2_TXEIE;
  }
}

void SPI2_IRQHandler(void) __attribute__((interrupt));
void SPI2_IRQHandler(void) {
  // printf("SPI2 IRQ\n");
  if (SPI2->STATR & SPI_STATR_RXNE || ((SPI2->STATR & SPI_STATR_TXE) && (SPI2->CTLR2 & SPI_CTLR2_TXEIE))) {
    // Starting receiving new packet
    uint8_t i = 0;
    uint8_t j = 0;
    while (i < PACKET_SIZE || (j < PACKET_SIZE && (SPI2->CTLR2 & SPI_CTLR2_TXEIE))) {
      if (SPI2->STATR & SPI_STATR_TXE && j < PACKET_SIZE + 1 && (SPI2->CTLR2 & SPI_CTLR2_TXEIE)) {
        // Write byte from buffer
        SPI2->DATAR = spi2.buffer->out[spi2.buffer->pos_out + j];
        // If we've sent everything there was, disable TX interrupt
        if (spi2.buffer->sending_pos++ >= spi2.buffer->pos_out) SPI2->CTLR2 &= ~SPI_CTLR2_TXEIE;
        j++;
      }
      if (SPI2->STATR & SPI_STATR_RXNE && i < PACKET_SIZE + 1) {
        spi2.buffer->in[spi2.buffer->pos_in + i] = SPI2->DATAR;
        i++;
      }
    }
    if (spi2.buffer->receiving_pos >= spi2.buffer->pos_in) {
      
      process_incoming(&spi2);
      spi2.buffer->pos_in += PACKET_SIZE;
      spi2.buffer->receiving_pos = spi2.buffer->pos_in;
      SPI2->DATAR; // Clear buffer?
    } 
    // if (spi2.buffer->receiving_pos >= spi2.buffer->pos_in) {
    //   spi2.buffer->pos_in += PACKET_SIZE;
    // }
    // spi2.buffer->in[spi2.buffer->receiving_pos] = SPI2->DATAR;
    // if (spi2.buffer->receiving_pos++ >= spi2.buffer->pos_in) {
    //   process_incoming(&spi2);
    // } 
  }
}

static inline void setupEXTI(uint8_t pin, uint8_t rising) {
  uint8_t port = (uint8_t)(pin / 16);
  uint8_t exti = (uint8_t)((pin%16) / 4);

	AFIO->EXTICR[exti] |= port;

  if (rising) {
    EXTI->RTENR |= 1 << (pin%16); // enable rising edge trigger
    EXTI->FTENR &= ~(1 << (pin%16)); // disable falling edge trigger
  } else {
    EXTI->RTENR &= ~(1 << (pin%16)); // disable rising edge trigger
    EXTI->FTENR |= 1 << (pin%16); // enable falling edge trigger
  }

  if ((pin%16) < 5) NVIC_EnableIRQ((IRQn_Type)((uint8_t)EXTI0_IRQn + (pin%16)));
  else if ((pin%16) < 10) NVIC_EnableIRQ(EXTI9_5_IRQn);
  else NVIC_EnableIRQ(EXTI15_10_IRQn);

  // printf("exti priority %02x\n", NVIC);
  // NVIC_SetPriority();
  EXTI->INTENR |= 1 << (pin%16); // Enable EXTIx interrupt
}

void setupDMA(uint8_t ch, SPI_TypeDef * spi, uint8_t mode, uint8_t sixteen_bits, uint8_t* buffer)
{
  printf("DMA_ch%d, buffer = %08x\n", ch, buffer);
	RCC->AHBPCENR |= RCC_AHBPeriph_DMA1;

	DMA1_CH(ch)->PADDR = (uint32_t)&spi->DATAR;
  
	DMA1_CH(ch)->MADDR = (uint32_t)buffer;
	DMA1_CH(ch)->CNTR  = 0;
	DMA1_CH(ch)->CFGR  =
		DMA_M2M_Disable |		 
		DMA_Priority_High |
		(sixteen_bits?DMA_MemoryDataSize_Word:DMA_MemoryDataSize_Byte) |
		(sixteen_bits?DMA_PeripheralDataSize_Word:DMA_PeripheralDataSize_Byte) |
		DMA_MemoryInc_Enable |
		(mode?DMA_Mode_Normal:DMA_Mode_Circular) |
		(mode?DMA_DIR_PeripheralDST:DMA_DIR_PeripheralSRC) |
		// DMA_IT_HT | 
    DMA_IT_TC; 

  // printf("DATAR = %08x\n", (uint32_t)&spi->DATAR);
  // printf("PADDR%d = %08x\n", ch, DMA1_CH(ch)->PADDR);
  // printf("CFGR%d = %08x\n", ch, DMA1_CH(ch)->CFGR);

	NVIC_EnableIRQ( DMA1_IRQ(ch) );
	DMA1_CH(ch)->CFGR |= DMA_CFGR1_EN;
}

void setup_SPI(SPI_TypeDef * spi, uint8_t mode, uint8_t sixteen_bits)
{
  printf("SPI = %08lx\n", (uint32_t)spi);
	if (spi == SPI1) {
		funPinMode( SPI1_CS1, mode?GPIO_CFGLR_OUT_50Mhz_PP:GPIO_CNF_IN_PUPD );
		funPinMode( PA5, mode?GPIO_CFGLR_OUT_50Mhz_AF_PP:GPIO_CNF_IN_FLOATING );
		funPinMode( PA6, mode?GPIO_CNF_IN_FLOATING:GPIO_CFGLR_OUT_50Mhz_AF_PP );
		funPinMode( PA7, mode?GPIO_CFGLR_OUT_50Mhz_AF_PP:GPIO_CNF_IN_FLOATING );
		RCC->APB2PRSTR = RCC_SPI1RST;
		RCC->APB2PRSTR = 0;
		RCC->APB2PCENR |= RCC_APB2Periph_SPI1;
	}
	else {
		funPinMode( SPI2_CS1, mode?GPIO_CFGLR_OUT_50Mhz_PP:GPIO_CNF_IN_PUPD );
		funPinMode( PB13, mode?GPIO_CFGLR_OUT_50Mhz_AF_PP:GPIO_CNF_IN_FLOATING );
		funPinMode( PB14, mode?GPIO_CNF_IN_FLOATING:GPIO_CFGLR_OUT_50Mhz_AF_PP );
		funPinMode( PB15, mode?GPIO_CFGLR_OUT_50Mhz_AF_PP:GPIO_CNF_IN_FLOATING );
		RCC->APB1PRSTR = RCC_SPI2RST;
		RCC->APB1PRSTR = 0;
		RCC->APB1PCENR |= RCC_APB1Periph_SPI2;
	}

  // DMA only works in Master mode for some reason. Need to use interrupts in Slave mode.
  if (mode) {
    spi->CTLR2 = SPI_CTLR2_RXDMAEN | SPI_CTLR2_TXDMAEN;
  } else {
#if !defined(USE_EXTI) || !USE_EXTI
    spi->CTLR2 = SPI_CTLR2_RXNEIE;
    if (spi == SPI1) NVIC_EnableIRQ(SPI1_IRQn);
    else NVIC_EnableIRQ(SPI2_IRQn);
#endif
  }

	// Configure SPI 
#if USE_EXTI
	spi->CTLR1 = SPI_NSS_Soft | SPI_CPHA_1Edge | SPI_CPOL_Low | (sixteen_bits?SPI_DataSize_16b:0) | SPI_BaudRatePrescaler_256 | (mode?SPI_Mode_Master:0);
#else
	spi->CTLR1 = (mode?SPI_NSS_Soft:0) | SPI_CPHA_1Edge | SPI_CPOL_Low | (sixteen_bits?SPI_DataSize_16b:0) | SPI_BaudRatePrescaler_256 | (mode?SPI_Mode_Master:0);
#endif
	// spi->CRCR = 7;

  if (mode) {
    if (spi == SPI1) {
      setupDMA(2, spi, DMA_In, sixteen_bits, spi1.buffer->in);
      setupDMA(3, spi, DMA_Out, sixteen_bits, spi1.buffer->out);
    } else {
      setupDMA(4, spi, DMA_In, sixteen_bits, spi2.buffer->in);
      setupDMA(5, spi, DMA_Out, sixteen_bits, spi2.buffer->out);
    }
  }
	spi->CTLR1 |= CTLR1_SPE_Set; // Enable

}

void spi_send(spi_t * spi, uint8_t receiver_id, uint8_t* buf) {
  uint8_t master = ((spi->addr->CTLR1 & 0x4) == 0x4);
  
  printf("sending message via SPI-%s\n", (master?"master":"slave"));
  
  spi->buffer->out[spi->buffer->pos_out] = (uint8_t)NODE_ID;
  spi->buffer->out[spi->buffer->pos_out+1] = receiver_id;
  spi->buffer->out[spi->buffer->pos_out+2] = (uint8_t)(spi->message_number<<9);
  spi->buffer->out[spi->buffer->pos_out+3] = (uint8_t)((spi->message_number<<1) | 1);

  memcpy(&spi->buffer->out[spi->buffer->pos_out+4], buf, PACKET_SIZE);
  
  printf("%02x %02x %02x %02x %02x %02x %02x %02x\n", spi->buffer->out[0], spi->buffer->out[1], spi->buffer->out[2], spi->buffer->out[3], spi->buffer->out[4], spi->buffer->out[5], spi->buffer->out[6], spi->buffer->out[7]);
  
  if (master) {  
    if (spi->transmission == SPI_IDLE && master) if (master) funDigitalWrite(spi->cs_pin, 0);
    if (spi->transmission == SPI_IDLE || spi->transmission == SPI_DONE) {
      spi->dma_out->CFGR &= ~DMA_CFGR1_EN;
      spi->dma_out->MADDR = &spi->buffer->out[spi->buffer->pos_out];
      spi->dma_out->CNTR = PACKET_SIZE;
      // printf("CNTR = %d\n", DMA1_CH(3)->CNTR);
      spi->dma_out->CFGR |= DMA_CFGR1_EN;
      spi->buffer->sending_pos = spi->buffer->pos_out;
    }

    printf("MADDR = %08lx\n", spi->dma_out->MADDR);
    printf("buffer.out = %08lx\n", (uint32_t)&spi->buffer->out);
    printf("buffer.pos_out = %ld\n", spi->buffer->pos_out);

  } else {
    if (spi->transmission == SPI_IDLE || spi->transmission == SPI_DONE) {
      spi->addr->CTLR2 |= SPI_CTLR2_TXEIE;
    }
  }

  if ((spi->buffer->pos_out += PACKET_SIZE) >= BUFFER_SIZE) spi->buffer->pos_out = 0;
}

void spi_poll(spi_t * spi) {
  uint8_t buf[6] = {0, 0, 0, 0, 0, 0,};

  spi_send(spi, 0, buf);
}

int main()
{
	SystemInit();

	funGpioInitAll();

  printf("setup\n");

  memset(spi_master->buffer->in, 0, BUFFER_SIZE);
  memset(spi_master->buffer->out, 0, BUFFER_SIZE);

  memset(spi_slave->buffer->in, 0, BUFFER_SIZE);
  memset(spi_slave->buffer->out, 0, BUFFER_SIZE);

	// setup_SPI(SPI1, SPI_Master, 0);
  // funDigitalWrite(spi_master->cs_pin, 1);

	setup_SPI(SPI2, SPI_Slave, 0);
  funDigitalWrite(spi_slave->cs_pin, 0);
#if USE_EXTI
  setupEXTI(spi_slave->cs_pin, 0);
#endif

	// RCC->CFGR0 &= ~RCC_PPRE1_DIV16;

  uint8_t message[PACKET_SIZE-2] = {0x11, 0x22, 0x33, 0x44, 0x55, 0x77};
  // spi_send(spi_master, 0xaa, message);
  printf("sent\n");

	while(1)
	{
    if(SysTick->CNT - timer > Ticks_from_Ms(1000))
		{
			timer = SysTick->CNT;
      printf("timer tick\n");
			// spi_send(spi_master, 0xaa, message);
      // printf("%02x\n", incomming_packet);
		}

    if(!(spi_master->addr->STATR & SPI_I2S_FLAG_BSY)) {
			funDigitalWrite(spi_master->cs_pin, 1);
      spi_master->transmission = SPI_IDLE;
		}
    
    // if (incomming_packet) {
    //   printf("%02x %02x %02x %02x %02x %02x %02x %02x %02x %02x\n", spi2.buffer->in[0], spi2.buffer->in[1], spi2.buffer->in[2], spi2.buffer->in[3], spi2.buffer->in[4], spi2.buffer->in[5], spi2.buffer->in[6], spi2.buffer->in[7], spi2.buffer->in[8], spi2.buffer->in[9]);
    //   // uint8_t i = 0;
    //   // uint8_t j = 0;

    //   // while (i < PACKET_SIZE || j < PACKET_SIZE) {
    //   //   if (SPI2->STATR & SPI_STATR_TXE && j < PACKET_SIZE) {
    //   //     spi_slave->addr->DATAR = 0xff;
    //   //     j++;
    //   //   }
    //   //   if (SPI2->STATR & SPI_STATR_RXNE && i < PACKET_SIZE) {
    //   //     // uint16_t datar = spi_slave->addr->DATAR;
    //   //     printf("%04x", spi_slave->addr->DATAR);
    //   //     spi_slave->addr->DATAR = 0;
    //   //     i++;
    //   //   }
        
    //   // }
    //   incomming_packet = 0;
    // }

	}
}

