#include "ch32fun.h"
#include <stdio.h>
#include <string.h>

#define NODE_ID 0xaa
#define SPI_TRANSMISSION_TIMEOUT 10000

#define MAX_NODES_N 64 // Maximum nodes number in the chain
#if (MAX_NODES_N > 255)
#error "You can have only 255 nodes max"
#endif

#define DMA_BASE_ADDR(n)  (DMA1_Channel1_BASE+(0x14*(n-1)))
#define DMA1_CH(n) ((DMA_Channel_TypeDef *)DMA_BASE_ADDR(n))
#define DMA1_IRQ(x) DMA1_Channel1_IRQn + x - 1

#define SPI1_CS1 PA4
#define SPI2_CS1 PB12

#define SPI_Master 1
#define SPI_Slave 0

#define DMA_In 0
#define DMA_Out 1

#define PACKET_SIZE 10
#define BUFFER_SIZE PACKET_SIZE*10

enum SPITransfer {
  SPI_IDLE,
  SPI_POLLING,
  SPI_IN_PROGRESS,
  SPI_DONE,
};

typedef enum {
  commandIdle = 0,
  commandAck,
  commandNak,
  commandEnumerate,
  commandReadSensor,
  commandLed,
} command_t;

typedef struct {
	__attribute__ ((aligned(4))) uint8_t in[BUFFER_SIZE];
	__attribute__ ((aligned(4))) uint8_t out[BUFFER_SIZE];
	volatile uint32_t pos_out; // Position in the array for the next packet to be placed at
	volatile uint32_t pos_in;
	volatile uint32_t sending_pos; // Position we are actually reading data from during send
	volatile uint32_t receiving_pos; // or writing to during receive
} buffer_t;

typedef struct {
  SPI_TypeDef * addr;
  DMA_Channel_TypeDef * dma_in;
  DMA_Channel_TypeDef * dma_out;
  uint32_t cs_pin;
  buffer_t * buffer;
  volatile enum SPITransfer transmission; // Currect SPI transmission status
  uint8_t next_node_ID;
  uint16_t message_number;
  uint16_t in_message_number;
  command_t in_pending_command;
  command_t out_pending_command;
  volatile uint8_t incoming_message[PACKET_SIZE];
  uint8_t previous_out_message[PACKET_SIZE]; // Save previous message to be able to repeat it if receiver NAKs
  volatile uint8_t transmission_packets_n; // Number of packets received in last transmission and to be processed
  int16_t incomming_message_pos;
  uint8_t master;
} spi_t;

volatile uint8_t message_received = 0;
volatile enum SPITransfer spi1_transmission = SPI_IDLE;
volatile enum SPITransfer spi2_transmission = SPI_IDLE;
// volatile uint8_t test_buffer[10] = {0xaa, 0xaa, 0x1, 0x1, 0x77, 0x66, 0x55, 0x44, 0x33, 0x22};
buffer_t buffer;
buffer_t buffer_spi1;
buffer_t buffer_spi2;

spi_t spi1 = {
  SPI1,
  DMA1_Channel2,
  DMA1_Channel3,
  SPI1_CS1,
  &buffer_spi1,
  SPI_IDLE,
};

spi_t spi2 = {
  SPI2,
  DMA1_Channel4,
  DMA1_Channel5,
  SPI2_CS1,
  &buffer_spi2,
  SPI_IDLE,
};

spi_t* spi_master = &spi1;
spi_t* spi_slave = &spi2;

uint64_t timer = 0;
volatile uint8_t incomming_packet = 0;

uint8_t nodes_map_up[MAX_NODES_N - 1];
uint8_t nodes_map_down[MAX_NODES_N - 1];
uint8_t n_nodes_up = 0;
uint8_t n_nodes_down = 0;

static inline void process_incoming(spi_t * spi) {
  // printf("Processing incoming: [%ld] %02x %02x %02x %02x %02x %02x %02x %02x\n", spi->buffer->pos_in, spi->buffer->in[spi->buffer->pos_in], spi->buffer->in[spi->buffer->pos_in + 1], spi->buffer->in[spi->buffer->pos_in + 2], spi->buffer->in[spi->buffer->pos_in + 3], spi->buffer->in[spi->buffer->pos_in + 4], spi->buffer->in[spi->buffer->pos_in + 5], spi->buffer->in[spi->buffer->pos_in + 6], spi->buffer->in[spi->buffer->pos_in + 7]);
  uint32_t position = spi->buffer->pos_in;
  for (int i = 0; i < spi->transmission_packets_n; i++) {
    position = spi->buffer->pos_in + (i*PACKET_SIZE);
    if (position >= BUFFER_SIZE) position -= BUFFER_SIZE; // Loop back in buffer
    if (spi->buffer->in[position + 1] == 0) continue; // Skip message if sender field is blank

    uint8_t inc_id = spi->buffer->in[position];
    // If this message is intended for tis node, or is a broadcast
    if (inc_id == NODE_ID || inc_id == 0 || inc_id == 0xff) {
      if (spi->incomming_message_pos < 0) {
        spi->incomming_message_pos = position;
      }
      // This should be done during enumeration.
      // TODO: maybe move to enumeration function?
      if (inc_id == 0 && spi->next_node_ID == 0) spi->next_node_ID = spi->buffer->in[position + 1];
    // Else pass message further
    } else {
      // Select opposite spi struct to copy message to
      spi_t * target = (spi == &spi1)?&spi2:&spi1;
      // if (spi == &spi1) target = &spi2;

      memcpy(&target->buffer->out[target->buffer->pos_out], &spi->buffer->in[position], PACKET_SIZE);
      if ((target->buffer->pos_out += PACKET_SIZE) >= BUFFER_SIZE) target->buffer->pos_out = 0;
    }
  }
  
  // Now we need to find a new position in buffer for next transmission
  if (!spi->master) {
    // In nodma mode we don't need circular buffer for receive;
    // We will use half of the buffer, just in case. To let processor process incoming messages
    if (spi->buffer->pos_in) spi->buffer->pos_in = 0; 
    else spi->buffer->pos_in = BUFFER_SIZE / 2;
  } else {
    // If we use DMA better rely on it's counter
    if (spi->dma_in->CFGR & DMA_CFGR1_EN) spi->buffer->pos_in = BUFFER_SIZE - spi->dma_in->CNTR;
    // Otherwise use calculated position
    else spi->buffer->pos_in = position;
  }
  spi->buffer->receiving_pos = spi->buffer->pos_in; // Not sure if we need this counter globally
}

__attribute__((interrupt)) void DMA1_Channel3_IRQHandler( void ) 
{
  if (DMA1->INTFR & DMA1_IT_TC3) {
    DMA1_CH(3)->CFGR &= ~DMA_CFGR1_EN; // Disable OUT DMA
    if (spi1.transmission != SPI_IDLE) spi1.transmission = SPI_DONE; // Notify main
  }
  DMA1->INTFCR = DMA1_IT_GL3 | DMA1_IT_GL2; // Clear all interrupt flags for both DMA channels for SPI1
}

__attribute__((interrupt)) void DMA1_Channel5_IRQHandler(void) {
  if (DMA1->INTFR & DMA1_IT_TC5) {
    DMA1_CH(5)->CFGR &= ~DMA_CFGR1_EN;
    if (spi2.transmission != SPI_IDLE) spi2.transmission = SPI_DONE;
  }
  DMA1->INTFCR = DMA1_IT_GL4 | DMA1_IT_GL5; // Clear all interrupt flags for both DMA channels for SPI2
}

// GCC (with -Os) ignores inline keyword here, if you really want you have to use "__attribute__((always_inline))"
// __attribute__((always_inline)) static inline void spi_nodma_transmission(spi_t * spi, uint8_t * buf, uint32_t len) {
void spi_nodma_transmission(spi_t * spi, uint8_t * buf, uint32_t len) {
  
  uint32_t timeout = SPI_TRANSMISSION_TIMEOUT;
  uint32_t sent = 0;
  uint32_t received = 0;
  uint8_t to_receive = PACKET_SIZE; // Minimum amount we expect to receive

  if (buf && len == 0) buf = NULL;
  if (buf == NULL) {
    len = spi->buffer->pos_out + (BUFFER_SIZE - spi->buffer->sending_pos);
    if (len >= BUFFER_SIZE) len -= BUFFER_SIZE;
  }
  
  while (timeout--) {
    // If SPI slave, we only run this function in interrupt
    if (len && ((spi->addr->STATR & SPI_STATR_TXE) || !sent)) {
      // Write byte from buffer
      if (buf == NULL) {
        spi->addr->DATAR = spi->buffer->out[spi->buffer->sending_pos++];
        if (spi->buffer->sending_pos >= BUFFER_SIZE) spi->buffer->sending_pos = 0;
      } else {
        spi->addr->DATAR = buf[sent];
      }
      timeout = SPI_TRANSMISSION_TIMEOUT; // Reload timeout
      len--;
      sent++;
    }

    if ((spi->addr->STATR & SPI_STATR_RXNE)) {
      // Read from SPI to buffer
      spi->buffer->in[spi->buffer->receiving_pos++] = spi->addr->DATAR;
      if (to_receive) to_receive--;
      timeout = SPI_TRANSMISSION_TIMEOUT;
      received++;
    }

    if (!spi->master && funDigitalRead(spi->cs_pin)) {
      break;
    }
    // If SPI master, we only run while there is something to send, or until timeout if POLLING
    if (spi->master && (!len) && spi->transmission != SPI_POLLING) break;
  }

  spi->buffer->sending_pos = PACKET_SIZE * (spi->buffer->sending_pos / PACKET_SIZE); // Align position to the packet start for next transmission
  
  spi->transmission_packets_n = received / PACKET_SIZE;

  // If we actually received anything, process incoming buffer
  if (to_receive != PACKET_SIZE) {
    process_incoming(spi);
  }

  EXTI->INTFR = 1 << (spi->cs_pin % 16);
}

// All EXTI handlers are declared to be able to easily change trigger pin with a function.
// On practice it is recommended leave only the one handler that is actually being use. It will save ~400B of flash.
// Also without "__attribute__((always_inline))" or "-fno-ipa-icf-functions" in gcc options,
// this way of declaring interrupt handlers produced unexpected and unwanted results.
__attribute__((interrupt)) __attribute__((always_inline)) void EXTI0_IRQHandler( void ) { spi_nodma_transmission(spi_slave, NULL, 0); }
__attribute__((interrupt)) __attribute__((always_inline)) void EXTI1_IRQHandler( void ) { spi_nodma_transmission(spi_slave, NULL, 0); }
__attribute__((interrupt)) __attribute__((always_inline)) void EXTI2_IRQHandler( void ) { spi_nodma_transmission(spi_slave, NULL, 0); }
__attribute__((interrupt)) __attribute__((always_inline)) void EXTI3_IRQHandler( void ) { spi_nodma_transmission(spi_slave, NULL, 0); }
__attribute__((interrupt)) __attribute__((always_inline)) void EXTI4_IRQHandler( void ) { spi_nodma_transmission(spi_slave, NULL, 0); }
__attribute__((interrupt)) __attribute__((always_inline)) void EXTI9_5_IRQHandler( void ) { spi_nodma_transmission(spi_slave, NULL, 0); }
__attribute__((interrupt)) __attribute__((always_inline)) void EXTI15_10_IRQHandler( void ) { spi_nodma_transmission(spi_slave, NULL, 0); }

static inline void setupEXTI(uint8_t pin, uint8_t rising) {
  uint8_t port = (uint8_t)(pin / 16);
  pin = pin%16;
  uint8_t exti = (uint8_t)(pin / 4);

	AFIO->EXTICR[exti] |= port;

  if (rising) {
    EXTI->RTENR |= 1 << pin; // enable rising edge trigger
    EXTI->FTENR &= ~(1 << pin); // disable falling edge trigger
  } else {
    EXTI->RTENR &= ~(1 << pin); // disable rising edge trigger
    EXTI->FTENR |= 1 << pin; // enable falling edge trigger
  }

  IRQn_Type irqn;
  if (pin < 5) irqn = (IRQn_Type)((uint8_t)EXTI0_IRQn + pin);
  else if (pin < 10) irqn = EXTI9_5_IRQn;
  else irqn = EXTI15_10_IRQn;

  // printf("exti priority %02x\n", NVIC);
  NVIC_EnableIRQ(irqn);
#if defined(FUNCONF_ENABLE_HPE) && FUNCONF_ENABLE_HPE
  NVIC_SetPriority(irqn, 0<<7); // Set HIGH preemptive priority for the IRQ
#endif
  EXTI->INTENR |= 1 << pin; // Enable EXTIx interrupt
}

void setupDMA(uint8_t ch, SPI_TypeDef * spi, uint8_t mode, uint8_t sixteen_bits, uint8_t* buffer)
{
	RCC->AHBPCENR |= RCC_AHBPeriph_DMA1;

	DMA1_CH(ch)->PADDR = (uint32_t)&spi->DATAR;
	DMA1_CH(ch)->MADDR = (uint32_t)buffer;
	DMA1_CH(ch)->CNTR  = (mode?0:BUFFER_SIZE);
	DMA1_CH(ch)->CFGR  =
		DMA_M2M_Disable |		 
		(mode?DMA_Priority_VeryHigh:DMA_Priority_High) |
		(sixteen_bits?DMA_MemoryDataSize_Word:DMA_MemoryDataSize_Byte) |
		(sixteen_bits?DMA_PeripheralDataSize_Word:DMA_PeripheralDataSize_Byte) |
		DMA_MemoryInc_Enable |
		(mode?DMA_Mode_Normal:DMA_Mode_Circular) |
		(mode?DMA_DIR_PeripheralDST:DMA_DIR_PeripheralSRC) |
    (mode?DMA_IT_TC:0); 

  if (mode) {
    NVIC_EnableIRQ( DMA1_IRQ(ch) );
#if defined(FUNCONF_ENABLE_HPE) && FUNCONF_ENABLE_HPE && (TARGET_MCU!=CH32V103)
    NVIC_SetPriority(DMA1_IRQ(ch), 1<<7);
#endif
  }
	DMA1_CH(ch)->CFGR |= DMA_CFGR1_EN;
}

void setup_SPI(spi_t * spi, uint8_t mode, uint8_t sixteen_bits)
{
  printf("SPI%d = %08lx\n", (spi->addr == SPI1)?1:2, (uint32_t)spi->addr);

  // I've found that GPIO modes other than 2MHz, at least in case of slave SPI MISO doesn't really work that great (if at all).
  // 2MHz is more stable and I didn't find any downsides.
	if (spi->addr == SPI1) {
		funPinMode( SPI1_CS1, mode?GPIO_CFGLR_OUT_2Mhz_PP:GPIO_CNF_IN_PUPD );
		funPinMode( PA5, mode?GPIO_CFGLR_OUT_2Mhz_AF_PP:GPIO_CNF_IN_FLOATING );
		funPinMode( PA6, mode?GPIO_CNF_IN_FLOATING:GPIO_CFGLR_OUT_2Mhz_AF_PP );
		funPinMode( PA7, mode?GPIO_CFGLR_OUT_2Mhz_AF_PP:GPIO_CNF_IN_FLOATING );
		RCC->APB2PRSTR = RCC_SPI1RST;
		RCC->APB2PRSTR = 0;
		RCC->APB2PCENR |= RCC_APB2Periph_SPI1;
	} else {
		funPinMode( SPI2_CS1, mode?GPIO_CFGLR_OUT_2Mhz_PP:GPIO_CNF_IN_PUPD );
		funPinMode( PB13, mode?GPIO_CFGLR_OUT_2Mhz_AF_PP:GPIO_CNF_IN_FLOATING );
		funPinMode( PB14, mode?GPIO_CNF_IN_FLOATING:GPIO_CFGLR_OUT_2Mhz_AF_PP );
		funPinMode( PB15, mode?GPIO_CFGLR_OUT_2Mhz_AF_PP:GPIO_CNF_IN_FLOATING );
		RCC->APB1PRSTR = RCC_SPI2RST;
		RCC->APB1PRSTR = 0;
		RCC->APB1PCENR |= RCC_APB1Periph_SPI2;
	}

	// Configure SPI 
	spi->addr->CTLR1 = (mode?SPI_NSS_Soft:0) | SPI_CPHA_1Edge | SPI_CPOL_Low | (sixteen_bits?SPI_DataSize_16b:0) | SPI_BaudRatePrescaler_16 | (mode?SPI_Mode_Master:0);
	// spi->CRCR = 7;

  // DMA only works in Master mode for some reason. Need to use interrupts in Slave mode.
  if (mode) {
    spi->master = 1;
    spi->addr->CTLR2 = SPI_CTLR2_RXDMAEN | SPI_CTLR2_TXDMAEN;
    
    if (spi->addr == SPI1) {
      setupDMA(2, spi->addr, DMA_In, sixteen_bits, spi->buffer->in);
      setupDMA(3, spi->addr, DMA_Out, sixteen_bits, spi->buffer->out);
    } else {
      setupDMA(4, spi->addr, DMA_In, sixteen_bits, spi->buffer->in);
      setupDMA(5, spi->addr, DMA_Out, sixteen_bits, spi->buffer->out);
    }
  }
  
	spi->addr->CTLR1 |= CTLR1_SPE_Set; // Enable SPI

}

void spi_send(spi_t * spi, uint32_t pos, uint8_t n_packets) {
  printf("sending message via SPI-%s\n", (spi->master?"master":"slave"));
  
  // Update position in buffer for next data
  // Loop the buffer if needed
  if ((spi->buffer->pos_out += PACKET_SIZE * n_packets) >= BUFFER_SIZE) spi->buffer->pos_out -= BUFFER_SIZE;
  // printf("spi->buffer->pos_out = %d\n", spi->buffer->pos_out);
  
  // Master needs to initiate a transmission if it's not in progress already
  // Slave will transmit in interrupt when master will initiate, so nothing to do here
  if (spi->master) {
    if (spi->transmission == SPI_IDLE) {
      spi->dma_out->CFGR &= ~DMA_CFGR1_EN; // Disable DMA to be able to make changes
      spi->dma_out->MADDR = (uint32_t)&spi->buffer->out[spi->buffer->sending_pos]; // Update buffer postion address

      if (spi->buffer->sending_pos >= spi->buffer->pos_out) { // This means we are looping over buffer
        spi->dma_out->CNTR = BUFFER_SIZE - spi->buffer->sending_pos;
        spi->buffer->sending_pos = 0; // We'll send the rest in another transmission
      } else {
        spi->dma_out->CNTR = spi->buffer->pos_out - spi->buffer->sending_pos;
        spi->buffer->sending_pos = spi->buffer->pos_out;
      }

      spi->transmission_packets_n =+ spi->dma_out->CNTR / PACKET_SIZE;
      funDigitalWrite(spi->cs_pin, 0); // CS low to start transmission
      spi->transmission = SPI_IN_PROGRESS;
      Delay_Ms(1);  // Let slave to react
      spi->dma_out->CFGR |= DMA_CFGR1_EN;
    }
  }
}

void spi_send_message(spi_t * spi, uint8_t receiver_id, uint8_t* buf, uint8_t len) {
  
  if (spi->message_number++ > 16383) spi->message_number = 1;
  spi->buffer->out[spi->buffer->pos_out] = receiver_id;
  spi->buffer->out[spi->buffer->pos_out+1] = (uint8_t)NODE_ID;
  spi->buffer->out[spi->buffer->pos_out+2] = (uint8_t)(spi->message_number>>6);
  spi->buffer->out[spi->buffer->pos_out+3] = (uint8_t)((spi->message_number<<2) | 3);

  // TODO: make a loop to send long messages in multiple packets
  if (len > PACKET_SIZE - 4) len = PACKET_SIZE - 4;
  memcpy(&spi->buffer->out[spi->buffer->pos_out+4], buf, len);
  
  // printf("buffer.sending_pos = %ld = %02x\n", spi->buffer->sending_pos, spi->buffer->out[spi->buffer->sending_pos]);
  // printf("buffer.pos_out = %ld = %02x\n", spi->buffer->pos_out, spi->buffer->out[spi->buffer->pos_out]);
  // printf("[%d] %02x %02x %02x %02x %02x %02x %02x %02x\n", spi->buffer->pos_out, spi->buffer->out[spi->buffer->pos_out], spi->buffer->out[spi->buffer->pos_out+1], spi->buffer->out[spi->buffer->pos_out+2], spi->buffer->out[spi->buffer->pos_out+3], spi->buffer->out[spi->buffer->pos_out+4], spi->buffer->out[spi->buffer->pos_out+5], spi->buffer->out[spi->buffer->pos_out+6], spi->buffer->out[spi->buffer->pos_out+7]);
  spi_send(spi, spi->buffer->pos_out, 1);
}

void spi_send_control(spi_t * spi, uint8_t receiver_id, uint16_t message_number, uint8_t ack) {
  
  spi->buffer->out[spi->buffer->pos_out] = receiver_id;
  spi->buffer->out[spi->buffer->pos_out+1] = (uint8_t)NODE_ID;
  spi->buffer->out[spi->buffer->pos_out+2] = (uint8_t)(message_number>>6);
  spi->buffer->out[spi->buffer->pos_out+3] = (uint8_t)(message_number);

  // second bit 1 - ACK, 0 - NAK
  if (ack) spi->buffer->out[spi->buffer->pos_out+3] |= 2;
  for (int i = 4; i < PACKET_SIZE; i++) {
    spi->buffer->out[spi->buffer->pos_out+i] = 0;
  }
  
  spi_send(spi, spi->buffer->pos_out, 1);

}

void spi_poll() {
  // Slave will ignore empty message, but will send any data it has to send
  for (int i = 0; i < PACKET_SIZE; i++) {
    spi_master->buffer->out[spi_master->buffer->pos_out + i] = 0;  
  }
  spi_send(spi_master, spi_master->buffer->pos_out, 1);
  // spi_send_control(spi_master, 0, 0, 0);
}

void process_message(spi_t * spi) {
  if (spi->incomming_message_pos < 0 || spi->transmission_packets_n == 0) return;

  uint8_t local_message[PACKET_SIZE];
  uint32_t old_number = spi->message_number;

  while (spi->transmission_packets_n--) {
    memcpy(local_message, &spi->buffer->in[spi->incomming_message_pos], PACKET_SIZE);
    printf("Incomming message: ");
    for (int n = 0; n < PACKET_SIZE; n++) {
      printf("%02x ", local_message[n]);
    }
    // printf("\nspi->transmission_packets_n = %d\n", spi->transmission_packets_n);
    printf("\n");
    // Copying a message to process, so it won't be overwritten by the IRQ in the process
    memcpy(local_message, &spi->buffer->in[spi->incomming_message_pos], PACKET_SIZE);
    if (spi->incomming_message_pos += PACKET_SIZE >= BUFFER_SIZE) spi->incomming_message_pos = 0;
    // Check the message number that is located in 3rd and 4th bits
    uint16_t message_number = (local_message[2]<<8) | local_message[3];
    // Valid message always has to have a number
    if (!message_number) continue;
    // LSB of the message number is to mark new message vs a reply
    uint8_t message_is_new = message_number & 1;
    uint8_t ack = message_number & 2;

    // Resend a command/message if NAK received while we waited a proper answer
    if (!message_is_new && !ack && spi->out_pending_command != commandIdle && message_number == old_number) {
      spi_send_message(spi, spi->previous_out_message[0], spi->previous_out_message, PACKET_SIZE);
    // If this is a new message and we have no ongoing commands to finish
    } else if (message_is_new && spi->in_pending_command == commandIdle) {
      
      switch (local_message[4]) {
        case commandEnumerate:
          // Send enumberate command downstream
          spi->in_pending_command = local_message[4];
          break;
        case commandLed:
          // Turn LED on/off
          // Ack
          spi_send_control(spi, local_message[1], message_number, 1);
          break;
        case commandReadSensor:
          // read_sensor_local();
          spi->in_pending_command = local_message[4];
          // Or send cashed data if we do it frequently
          break;
        default:
          // printf("Incomming message: ");
          // for (int n = 0; n < PACKET_SIZE; n++) {
          //   printf("%02x ", local_message[n]);
          // }
          // printf("\n");
          break;
      }
    // If this is a new message, but we are still finishing replying to previous
    } else if (spi->in_pending_command != commandIdle) {
      // Maybe NAK here so the sender can repeat request later
      spi_send_control(spi, local_message[1], message_number, 0);
      printf("Unanswer message is pending. Incomming message skipped.\n");
      return; // Don't synchronize message number
    // If this is a reply and we are waiting for one
    } else if (!message_is_new && spi->out_pending_command != commandIdle) {
      // And if this is a reply to a correct message
      if ((message_number >> 2) == old_number) {
        switch (spi->out_pending_command) {
          case commandEnumerate:
            for (int i = 4; i < PACKET_SIZE; i++) {
              if (local_message[i] == 0xff || local_message[i] == 0) {
                spi->out_pending_command = commandIdle;
                break;
              }
              // If this is a master SPI
              if ((spi->addr->CTLR1 & 0x4) == 0x4) nodes_map_down[n_nodes_down++] = local_message[i];
              else nodes_map_up[n_nodes_up++] = local_message[i];
            }
            break;
          case commandLed:
            spi->out_pending_command = commandIdle;
            break;
          case commandReadSensor:
            // Process incomming sensor data, just printing for now
            printf("Sensor data from [0x%02x]:%02x-%02x-%02x-%02x-%02x-%02x\n", local_message[1], local_message[4], local_message[5], local_message[6], local_message[7], local_message[8], local_message[9]);
            spi->out_pending_command = commandIdle;
            break;
        }
      } else {
        // This shouldn't happen? A reply with a different number, where would it come from?
        printf("Unexpected message number: %d", message_number);
        return;
      }
    // If this is a reply but we didn't request anythig - act confused but do nothing
    } else if (!message_is_new) {
      printf("Message number %04x, %d, type: %s Incomming message:\n", message_number, message_number >> 2, (message_number&1)?"new":"reply");
      for (uint8_t i = 0; i < PACKET_SIZE; i++) {
        printf("%02x ", local_message[i]);
      }
      printf("\n");
      // Probably still synchronize message number
    }
    // Syncronize message number?
    // Can potentially cause problem if incomming message with a different number from not neighboring node
    if (spi->message_number == old_number) {
      spi->message_number = message_number >> 2;
    }
  }
  // spi->transmission_packets_n = 0;
  spi->incomming_message_pos = -1;
}

void read_sensor_local() {
  SPI_TypeDef * master = spi_master->addr;
  
  while ((master->STATR & SPI_I2S_FLAG_BSY)) {
    funDigitalWrite(spi_master->cs_pin, 1);
    spi_master->transmission = SPI_IDLE;
  }

  uint16_t save_ctlr2 = master->CTLR2;
  master->CTLR2 = 0; // Disable SPI DMA, and interrupts if enabled

  // do sensor stuff

  // Wait until done
  while ((master->STATR & SPI_I2S_FLAG_BSY)) {
    funDigitalWrite(spi_master->cs_pin, 1);
    spi_master->transmission = SPI_IDLE;
  }

  master->CTLR2 = save_ctlr2; // Restore SPI DMA and interrupt settings
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

	setup_SPI(spi_master, SPI_Master, 0);
  funDigitalWrite(spi_master->cs_pin, 1);

	setup_SPI(spi_slave, SPI_Slave, 0);
  funDigitalWrite(spi_slave->cs_pin, 0);

  setupEXTI(spi_slave->cs_pin, 0);

#if (TARGET_MCU==CH32V103)
  NVIC->CFGR = NVIC_KEY1|1;
#endif

  uint8_t message_slave[PACKET_SIZE-4] = {0x77, 0x55, 0x44, 0x33, 0x22, 0x11};
  uint8_t message[PACKET_SIZE-4] = {0x11, 0x22, 0x33, 0x44, 0x55, 0x77};
  // spi_poll();
  // spi_send_message(spi_master, 0xaa, message);
  // spi_send_message(spi_slave, 0xaa, message);
  // printf("sent\n");

	while(1)
	{
    if (SysTick->CNT - timer > Ticks_from_Ms(1000)) {
			// spi_send_message(spi_master, 0xaa, message, PACKET_SIZE - 4);
			spi_send_message(spi_slave, 0xaa, message_slave, PACKET_SIZE - 4);
      // message_slave[2]++;
      timer = SysTick->CNT;
		}

    if (!(spi_master->addr->STATR & SPI_I2S_FLAG_BSY) && spi_master->transmission != SPI_IDLE) {
      process_incoming(spi_master);
      
			if (spi_master->buffer->sending_pos != spi_master->buffer->pos_out) {
        printf("There is something to send %ld - %ld\n", spi_master->buffer->sending_pos, spi_master->buffer->pos_out);

        spi_master->dma_out->CFGR &= ~DMA_CFGR1_EN; // Disable DMA to be able to make changes
        spi_master->dma_out->MADDR = (uint32_t)&spi_master->buffer->out[spi_master->buffer->sending_pos]; // Update buffer postion address

        if (spi_master->buffer->sending_pos >= spi_master->buffer->pos_out) { // This means we are looping over buffer
          spi_master->dma_out->CNTR = BUFFER_SIZE - spi_master->buffer->sending_pos;
          spi_master->buffer->sending_pos = 0; // We'll send the rest in another transmission
        } else {
          spi_master->dma_out->CNTR = spi_master->buffer->pos_out - spi_master->buffer->sending_pos;
          spi_master->buffer->sending_pos = spi_master->buffer->pos_out;
        }

        spi_master->transmission_packets_n =+ spi_master->dma_out->CNTR / PACKET_SIZE;
        funDigitalWrite(spi_master->cs_pin, 0); // CS low to start transmission
        spi_master->transmission = SPI_IN_PROGRESS;
        Delay_Ms(1);  // Let slave react
        spi_master->dma_out->CFGR |= DMA_CFGR1_EN;
			} else {
        funDigitalWrite(spi_master->cs_pin, 1);
        spi_master->transmission = SPI_IDLE;
      }
		}
    process_message(spi_master);
    process_message(spi_slave);
    // spi_nodma_transmission(spi_slave, message_slave, 6);
	}
}
