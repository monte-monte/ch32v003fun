#ifndef _CH32V208_ETH_H
#define _CH32V208_ETH_H

#include <stdbool.h>
#include <stdint.h>

#ifndef ETH_RX_BUF_COUNT
#define ETH_RX_BUF_COUNT 2
#endif

#ifndef ETH_TX_BUF_COUNT
#define ETH_TX_BUF_COUNT 1
#endif

#ifndef ETH_RX_BUF_SIZE
#define ETH_RX_BUF_SIZE 1536
#endif

#ifndef ETH_TX_BUF_SIZE
#define ETH_TX_BUF_SIZE 1536
#endif

#ifndef ETH_MAX_PACKET_SIZE
#define ETH_MAX_PACKET_SIZE 1536
#endif

#define ETH_MAC_ADDR_LEN 6
#define ETH_HEADER_LEN 14

#define ROM_CFG_USERADR_ID 0x1FFFF7E8
#define ETH_DMARxDesc_FrameLengthShift 16

// Ethernet frame structure
typedef struct
{
	uint8_t dest_mac[ETH_MAC_ADDR_LEN];
	uint8_t src_mac[ETH_MAC_ADDR_LEN];
	uint16_t ethertype; // Big endian
} __attribute__( ( packed ) ) eth_header_t;

// Callback types
typedef void ( *eth_rx_callback_t )( const uint8_t *packet, uint16_t length );
typedef void ( *eth_link_callback_t )( bool link_up );

// Ethernet configuration
typedef struct
{
	uint8_t *mac_addr; // MAC address (can be NULL to use chip default)
	eth_rx_callback_t rx_callback; // Called when packet received (in interrupt context)
	eth_link_callback_t link_callback; // Called when link status changes
	bool promiscuous_mode; // Enable promiscuous mode
	bool broadcast_filter; // Accept broadcast packets
	bool multicast_filter; // Accept multicast packets
} eth_config_t;

// Ethernet statistics
typedef struct
{
	uint32_t rx_packets;
	uint32_t tx_packets;
	uint32_t rx_errors;
	uint32_t tx_errors;
	uint32_t rx_dropped;
	uint32_t tx_dropped;
} eth_stats_t;

#ifdef __cplusplus
extern "C"
{
#endif

	/**
	 * Init the Ethernet peripheral
	 * @param config Ptr to config struct
	 * @return 0 on success, negative on error
	 */
	int eth_init( const eth_config_t *config );

	/**
	 * Send an Ethernet packet
	 * @param packet Ptr to packet data (incl. Ethernet header)
	 * @param length Length of pkt in bytes
	 * @return 0: success, -1: queue full, -2: invalid length)
	 */
	int eth_send_packet( const uint8_t *packet, uint16_t length );

	/**
	 * Process received packets (call from main loop)
	 * This will invoke the rx_callback for each received pkt
	 */
	void eth_process_rx( void );

	/**
	 * Poll link status (call periodically from main loop)
	 */
	void eth_poll_link( void );

	/**
	 * Get current MAC
	 * @param mac_addr Buffer to store MAC address (6 bytes)
	 */
	void eth_get_mac_address( uint8_t *mac_addr );

	/**
	 * Check if link is up
	 * @return true if link is up
	 */
	bool eth_is_link_up( void );

	/**
	 * Get stats
	 * @param stats Pointer to statistics structure to fill
	 */
	void eth_get_stats( eth_stats_t *stats );

	/**
	 * Reset stat counters
	 */
	void eth_reset_stats( void );

#ifdef __cplusplus
}
#endif

/**
 * Build Ethernet header
 * @param buffer Buffer to write header to
 * @param dest_mac Destination MAC address
 * @param src_mac Source MAC address
 * @param ethertype EtherType (e.g., 0x0800 for IPv4, 0x0806 for ARP)
 * @return Number of bytes written (14)
 */
static inline uint16_t eth_build_header(
	uint8_t *buffer, const uint8_t *dest_mac, const uint8_t *src_mac, uint16_t ethertype )
{
	eth_header_t *hdr = (eth_header_t *)buffer;
	for ( int i = 0; i < ETH_MAC_ADDR_LEN; i++ )
	{
		hdr->dest_mac[i] = dest_mac[i];
		hdr->src_mac[i] = src_mac[i];
	}
	hdr->ethertype = __builtin_bswap16( ethertype ); // swap byte order
	return ETH_HEADER_LEN;
}

/**
 * Parse Ethernet header
 * @param packet Pointer to packet data
 * @param header Pointer to header structure to fill
 * @return Pointer to payload (after header)
 */
static inline const uint8_t *eth_parse_header( const uint8_t *packet, eth_header_t *header )
{
	const eth_header_t *hdr = (const eth_header_t *)packet;
	*header = *hdr;
	header->ethertype = __builtin_bswap16( header->ethertype ); // swap byte order
	return packet + ETH_HEADER_LEN;
}

// Broadcast MAC address
#ifdef ETHERNET_IMPLEMENTATION
const uint8_t ETH_BROADCAST_MAC[ETH_MAC_ADDR_LEN] = { 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF };
#else
extern const uint8_t ETH_BROADCAST_MAC[ETH_MAC_ADDR_LEN];
#endif

#endif // _CH32V208_ETH_H


#ifdef CH32V208_ETH_IMPLEMENTATION

#include "ch32fun.h"
#include "ch32v20xhw.h"
#include <stdio.h>
#include <string.h>

// DMA descriptor
typedef struct
{
	volatile uint32_t Status;
	volatile uint32_t Buffer1Addr;
	volatile uint32_t Buffer2NextDescAddr;
	volatile uint32_t Reserved;
} ETH_DMADESCTypeDef;

// TX queue management
typedef struct
{
	volatile uint32_t head;
	volatile uint32_t tail;
	volatile bool is_full;
} tx_queue_t;

// driver state
typedef struct
{
	ETH_DMADESCTypeDef *rx_desc_head;
	ETH_DMADESCTypeDef *rx_desc_tail;
	tx_queue_t tx_q;
	uint8_t mac_addr[ETH_MAC_ADDR_LEN];
	eth_rx_callback_t rx_callback;
	eth_link_callback_t link_callback;
	volatile bool link_up;
	volatile bool link_irq_flag;
	eth_stats_t stats;
} eth_driver_state_t;

__attribute__( ( aligned( 4 ) ) ) static ETH_DMADESCTypeDef g_dma_rx_descs[ETH_RX_BUF_COUNT];
__attribute__( ( aligned( 4 ) ) ) static ETH_DMADESCTypeDef g_dma_tx_descs[ETH_TX_BUF_COUNT];
__attribute__( ( aligned( 4 ) ) ) static uint8_t g_mac_rx_bufs[ETH_RX_BUF_COUNT * ETH_RX_BUF_SIZE];
__attribute__( ( aligned( 4 ) ) ) static uint8_t g_mac_tx_bufs[ETH_TX_BUF_COUNT * ETH_TX_BUF_SIZE];

static eth_driver_state_t g_eth_state = { 0 };

const uint8_t ETH_BROADCAST_MAC[ETH_MAC_ADDR_LEN] = { 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF };

static void phy_write_reg( uint8_t reg_add, uint16_t reg_val )
{
	R32_ETH_MIWR = ( reg_add & RB_ETH_MIREGADR_MASK ) | RB_ETH_MIWR_MIIWR | ( reg_val << RB_ETH_MIWR_DATA_SHIFT );
}

static uint16_t phy_read_reg( uint8_t reg_add )
{
	ETH10M->MIERGADR = reg_add;
	return ETH10M->MIRD;
}

static inline void tx_queue_init( tx_queue_t *q )
{
	q->head = 0;
	q->tail = 0;
	q->is_full = false;
}

static inline bool tx_queue_is_empty( const tx_queue_t *q )
{
	return !q->is_full && ( q->head == q->tail );
}

static inline bool tx_queue_is_full( const tx_queue_t *q )
{
	return q->is_full;
}

static inline void tx_queue_produce( tx_queue_t *q )
{
	q->head = ( q->head + 1 ) % ETH_TX_BUF_COUNT;
	if ( q->head == q->tail )
	{
		q->is_full = true; // head caught up to tail = full
	}
}

static inline void tx_queue_consume( tx_queue_t *q )
{
	q->tail = ( q->tail + 1 ) % ETH_TX_BUF_COUNT;
	q->is_full = false; // after consuming, definitely not full
}

static void eth_get_chip_mac_addr( uint8_t *mac )
{
	const uint8_t *macaddr_src = (const uint8_t *)( ROM_CFG_USERADR_ID + 5 );
	for ( int i = 0; i < 6; i++ )
	{
		mac[i] = *( macaddr_src-- );
	}
}

static void tx_start_if_possible( void )
{
	if ( ETH10M->ECON1 & RB_ETH_ECON1_TXRTS )
	{
		return;
	}

	if ( tx_queue_is_empty( &g_eth_state.tx_q ) )
	{
		return;
	}

	uint32_t idx = g_eth_state.tx_q.tail;
	ETH_DMADESCTypeDef *desc = &g_dma_tx_descs[idx];
	uint16_t len = desc->Status;

	ETH10M->ETXLN = len; // set tx packet len
	ETH10M->ETXST = desc->Buffer1Addr; // set tx buf start address (DMA source)
	ETH10M->ECON1 |= RB_ETH_ECON1_TXRTS; // set tx req flag to start DMA transmission
}

int eth_init( const eth_config_t *config )
{
	if ( !config )
	{
		return -1;
	}

	memset( &g_eth_state, 0, sizeof( g_eth_state ) );

	g_eth_state.rx_callback = config->rx_callback;
	g_eth_state.link_callback = config->link_callback;

	if ( config->mac_addr )
	{
		memcpy( g_eth_state.mac_addr, config->mac_addr, ETH_MAC_ADDR_LEN );
	}
	else
	{
		eth_get_chip_mac_addr( g_eth_state.mac_addr );
	}

	// printf( "Ethernet MAC: %02X:%02X:%02X:%02X:%02X:%02X\n", g_eth_state.mac_addr[0], g_eth_state.mac_addr[1],
	// 	g_eth_state.mac_addr[2], g_eth_state.mac_addr[3], g_eth_state.mac_addr[4], g_eth_state.mac_addr[5] );

	RCC->APB2PCENR |= RCC_APB2Periph_AFIO;
	RCC->CFGR0 |= RCC_ETHPRE; // Ethernet clock prescaler
	EXTEN->EXTEN_CTR |= EXTEN_ETH_10M_EN; // Extended Control Register, 10M Ethernet enable and clock enable

	// Transmit/Receive module reset
	ETH10M->ECON1 = RB_ETH_ECON1_TXRST | RB_ETH_ECON1_RXRST;
	ETH10M->ECON1 = 0;

	uint32_t rx_filter;

	if ( config->promiscuous_mode )
	{
		// Promiscuous: disable filtering (ERXFCON=0 *probably* receives everything)
		rx_filter = 0;
	}
	else
	{
		// Normal mode: enable filtering and ALWAYS receive unicast to our MAC
		rx_filter = RB_ETH_ERXFCON_EN | RB_ETH_ERXFCON_UCEN;

		// broadcast packet reception
		if ( config->broadcast_filter )
		{
			rx_filter |= RB_ETH_ERXFCON_BCEN;
		}
		// multicast packet reception
		if ( config->multicast_filter )
		{
			rx_filter |= RB_ETH_ERXFCON_MCEN;
		}
	}

	ETH10M->ERXFCON = rx_filter;
	// MAC layer receive enable
	ETH10M->MACON1 = RB_ETH_MACON1_MARXEN;

	// Pad short packets with 0x00 to 60 bytes, then append 4-byte CRC
	// Hardware pads CRC
	ETH10M->MACON2 = PADCFG_AUTO_3 | RB_ETH_MACON2_TXCRCEN;
	// Set maximum frame length (MTU)
	ETH10M->MAMXFL = ETH_MAX_PACKET_SIZE;

	// MAC is reversed
	R8_ETH_MAADRL1 = g_eth_state.mac_addr[5];
	R8_ETH_MAADRL2 = g_eth_state.mac_addr[4];
	R8_ETH_MAADRL3 = g_eth_state.mac_addr[3];
	R8_ETH_MAADRL4 = g_eth_state.mac_addr[2];
	R8_ETH_MAADRL5 = g_eth_state.mac_addr[1];
	R8_ETH_MAADRL6 = g_eth_state.mac_addr[0];

	// PHY Analog Parameter Setting, default value and "Rated driver"
	ETH10M->ECON2 = RB_ETH_ECON2_DEFAULT;

	tx_queue_init( &g_eth_state.tx_q );
	for ( int i = 0; i < ETH_TX_BUF_COUNT; i++ )
	{
		g_dma_tx_descs[i].Status = 0;
		g_dma_tx_descs[i].Buffer1Addr = (uint32_t)&g_mac_tx_bufs[i * ETH_TX_BUF_SIZE];
		g_dma_tx_descs[i].Buffer2NextDescAddr = (uint32_t)&g_dma_tx_descs[( i + 1 ) % ETH_TX_BUF_COUNT];
	}

	// init RX descriptor ring (DMA reads from these)
	g_eth_state.rx_desc_head = g_dma_rx_descs; // DMA writes to head
	g_eth_state.rx_desc_tail = g_dma_rx_descs; // CPU reads from tail
	for ( int i = 0; i < ETH_RX_BUF_COUNT; i++ )
	{
		g_dma_rx_descs[i].Status = ETH_DMARxDesc_OWN; // DMA owns all initially
		g_dma_rx_descs[i].Buffer1Addr = (uint32_t)&g_mac_rx_bufs[i * ETH_RX_BUF_SIZE];
		// create circular linked list of descriptors
		g_dma_rx_descs[i].Buffer2NextDescAddr = (uint32_t)&g_dma_rx_descs[( i + 1 ) % ETH_RX_BUF_COUNT];
	}

	// start RX
	ETH10M->ERXST = g_eth_state.rx_desc_head->Buffer1Addr;
	ETH10M->ECON1 = RB_ETH_ECON1_RXEN;

	// reset PHY
	phy_write_reg( PHY_BMCR, PHY_BMCR_RESET );
	Delay_Ms( 200 );
	// configure PHY for full-duplex mode (TODO: implement auto-negotiation)
	phy_write_reg( PHY_BMCR, PHY_BMCR_FULL_DUPLEX );

	// clear all pending interrupt flags
	ETH10M->EIR = 0xFF;

	ETH10M->EIE = RB_ETH_EIE_INTIE | // Ethernet interrupt enable (master enable)
	              RB_ETH_EIE_RXIE | // RX complete interrupt
	              RB_ETH_EIE_TXIE | // TX complete interrupt
	              RB_ETH_EIE_LINKIE | // Link status change interrupt
	              RB_ETH_EIE_TXERIE | // TX error interrupt (collision, underrun, etc.)
	              RB_ETH_EIE_RXERIE | // RX error interrupt (CRC error, overrun, etc.)
	              RB_ETH_EIE_R_EN50; // Built-in 50ohm impedance matching resistor enable

	NVIC_EnableIRQ( ETH_IRQn );

	return 0;
}

int eth_send_packet( const uint8_t *packet, uint16_t length )
{
	if ( !packet || length == 0 || length > ETH_TX_BUF_SIZE )
	{
		return -2;
	}

	__disable_irq();
	if ( tx_queue_is_full( &g_eth_state.tx_q ) )
	{
		__enable_irq();
		g_eth_state.stats.tx_dropped++;
		return -1;
	}
	uint32_t idx = g_eth_state.tx_q.head;
	tx_queue_produce( &g_eth_state.tx_q ); // reserve the slot
	__enable_irq();

	uint8_t *tx_buf = (uint8_t *)g_dma_tx_descs[idx].Buffer1Addr;
	memcpy( tx_buf, packet, length );

	// mark descriptor ready
	g_dma_tx_descs[idx].Status = length;

	// try to kick TX
	__disable_irq();
	tx_start_if_possible();
	__enable_irq();

	return 0;
}

void eth_process_rx( void )
{
	// process all packets that DMA has released (OWN bit cleared by interrupt)
	while ( !( g_eth_state.rx_desc_tail->Status & ETH_DMARxDesc_OWN ) )
	{
		uint32_t len = ( g_eth_state.rx_desc_tail->Status & ETH_DMARxDesc_FL ) >> ETH_DMARxDesc_FrameLengthShift;
		uint8_t *buffer = (uint8_t *)g_eth_state.rx_desc_tail->Buffer1Addr;

		// deliver to user callback
		if ( g_eth_state.rx_callback )
		{
			g_eth_state.rx_callback( buffer, len );
		}

		// return descriptor to DMA
		g_eth_state.stats.rx_packets++;

		// advance to next descriptor in ring
		g_eth_state.rx_desc_tail->Status = ETH_DMARxDesc_OWN;
		g_eth_state.rx_desc_tail = (ETH_DMADESCTypeDef *)g_eth_state.rx_desc_tail->Buffer2NextDescAddr;
	}
}

/**
 * TODO: this should ideally do proper auto negotiation and polarity reversal
 */
void eth_poll_link( void )
{
	if ( !g_eth_state.link_irq_flag )
	{
		return;
	}
	g_eth_state.link_irq_flag = false;

	// i'm not sure if PHY link is latched-low, maytbe we don't need 2 reads
	(void)phy_read_reg( PHY_BMSR );
	uint16_t bmsr = phy_read_reg( PHY_BMSR );

	bool link_up = ( bmsr & PHY_BMSR_LINK_STATUS ) != 0;

	if ( link_up != g_eth_state.link_up )
	{
		g_eth_state.link_up = link_up;

		if ( link_up )
		{
			ETH10M->MACON2 |= RB_ETH_MACON2_FULDPX; // enable full duplex
			// printf( "Ethernet: Link UP\n" );
		}
		else
		{
			// printf( "Ethernet: Link DOWN\n" );
		}

		if ( g_eth_state.link_callback )
		{
			g_eth_state.link_callback( link_up );
		}
	}
}

void eth_get_mac_address( uint8_t *mac_addr )
{
	if ( mac_addr )
	{
		memcpy( mac_addr, g_eth_state.mac_addr, ETH_MAC_ADDR_LEN );
	}
}

bool eth_is_link_up( void )
{
	return g_eth_state.link_up;
}

void eth_get_stats( eth_stats_t *stats )
{
	if ( stats )
	{
		*stats = g_eth_state.stats;
	}
}

void eth_reset_stats( void )
{
	memset( &g_eth_state.stats, 0, sizeof( eth_stats_t ) );
}

void ETH_IRQHandler( void ) __attribute__( ( interrupt ) ) __attribute__( ( used ) );
void ETH_IRQHandler( void )
{
	uint32_t flags = ETH10M->EIR;

	if ( flags & RB_ETH_EIR_RXIF )
	{
		ETH10M->EIR = RB_ETH_EIR_RXIF; // clear interrupt flag

		// check if DMA still owns the current head descriptor
		if ( g_eth_state.rx_desc_head->Status & ETH_DMARxDesc_OWN )
		{
			// DMA still writing, check if next descriptor is available
			ETH_DMADESCTypeDef *next_desc = (ETH_DMADESCTypeDef *)g_eth_state.rx_desc_head->Buffer2NextDescAddr;

			if ( !( next_desc->Status & ETH_DMARxDesc_OWN ) )
			{
				// ring full
				g_eth_state.stats.rx_dropped++;
			}
			else
			{
				// mark current descriptor as ready for CPU processing
				g_eth_state.rx_desc_head->Status &= ~ETH_DMARxDesc_OWN;

				// add frame metadata
				g_eth_state.rx_desc_head->Status |= ( ETH_DMARxDesc_FS | ETH_DMARxDesc_LS | // Single segment frame
													  ( ETH10M->ERXLN << ETH_DMARxDesc_FrameLengthShift ) );

				// advance head to next descriptor for DMA
				g_eth_state.rx_desc_head = next_desc;

				// tell MAC where to write next packet
				ETH10M->ERXST = (uint32_t)g_eth_state.rx_desc_head->Buffer1Addr;
			}
		}
	}

	if ( flags & RB_ETH_EIR_TXIF )
	{
		ETH10M->EIR = RB_ETH_EIR_TXIF;

		if ( !tx_queue_is_empty( &g_eth_state.tx_q ) )
		{
			g_eth_state.stats.tx_packets++;
			tx_queue_consume( &g_eth_state.tx_q );
		}

		tx_start_if_possible();
	}

	if ( flags & RB_ETH_EIR_TXERIF )
	{
		ETH10M->EIR = RB_ETH_EIR_TXERIF;
		g_eth_state.stats.tx_errors++;

		if ( !tx_queue_is_empty( &g_eth_state.tx_q ) )
		{
			tx_queue_consume( &g_eth_state.tx_q );
		}
		tx_start_if_possible();
	}

	if ( flags & RB_ETH_EIR_RXERIF )
	{
		ETH10M->EIR = RB_ETH_EIR_RXERIF;
		ETH10M->ECON1 |= RB_ETH_ECON1_RXEN;
		g_eth_state.stats.rx_errors++;
	}

	if ( flags & RB_ETH_EIR_LINKIF )
	{
		g_eth_state.link_irq_flag = true;
		ETH10M->EIR = RB_ETH_EIR_LINKIF;
	}
}

#endif // CH32V208_ETH_IMPLEMENTATION
