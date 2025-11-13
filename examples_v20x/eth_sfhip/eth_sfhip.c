#include "ch32fun.h"
#include <stdio.h>

#define SFHIP_WARN( x... ) printf( x )
#define SFHIP_IMPLEMENTATION
#define HIP_PHY_HEADER_LENGTH_BYTES 0
#define SFHIP_TCP_SOCKETS 0

#include "sfhip.h"

// SFHIP_MTU: 1536 bytes
// MAC header: 14 bytes
// PHY header: 0 bytes
// 1550 bytes (round up to 1552 for alignment)
#define ETH_RX_BUF_SIZE 1552
#define ETH_TX_BUF_SIZE 1552
#define CH32V208_ETH_IMPLEMENTATION
#include "../../extralibs/ch32v208_eth.h"

// sfhip stack
sfhip hip = {
	.ip = 0,
	.mask = 0,
	.gateway = 0,
	.self_mac = { { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 } },
	.hostname = "ch32v208",
	.need_to_discover = 1, // start w/ DHCP DISCOVER
};

static sfhip_phy_packet_mtu scratch __attribute__( ( aligned( 4 ) ) );

int sfhip_send_packet( sfhip *hip, sfhip_phy_packet *data, int length )
{
	return eth_send_packet( (uint8_t *)data, length );
}

static inline void process_rx_packets( void )
{
	uint16_t pkt_len;
	const uint8_t *pkt = eth_get_rx_packet( &pkt_len );

	if ( pkt && pkt_len > 0 && pkt_len <= sizeof( scratch ) )
	{
		// copy dma buf to scratch before processing
		// otherwise we seem to hardfault on misaligned 32-bit mem access
		// TODO: figure out how to zero-copy
		memcpy( &scratch, pkt, pkt_len );
		eth_release_rx_packet();
		sfhip_accept_packet( &hip, &scratch, pkt_len );
	}
	else if ( pkt )
	{
		// too large or zero length
		eth_release_rx_packet();
	}
}


void sfhip_got_dhcp_lease( sfhip *hip, sfhip_address addr )
{
	uint32_t ip = HIPNTOHL( addr );
	printf( "DHCP IP: %u.%u.%u.%u\n", (unsigned int)( ( ip >> 24 ) & 0xFF ), (unsigned int)( ( ip >> 16 ) & 0xFF ),
		(unsigned int)( ( ip >> 8 ) & 0xFF ), (unsigned int)( ip & 0xFF ) );
}


static void link_status_callback( bool link_up )
{
	printf( "ETH: Link %s\n", link_up ? "UP" : "DOWN" );
}

int main( void )
{
	SystemInit();
	printf( "CH32V208 ETH10M test with sfhip (DHCP)\n" );

	// init Ethernet driver
	eth_config_t cfg = { .mac_addr = NULL,
		.rx_callback = NULL,
		.link_callback = link_status_callback,
		.promiscuous_mode = false,
		.broadcast_filter = true,
		.multicast_filter = false };

	if ( eth_init( &cfg ) != 0 )
	{
		printf( "ERROR: eth_init failed\n" );
		while ( 1 );
	}

	// get MAC address and set it in sfhip
	eth_get_mac_address( (uint8_t *)&hip.self_mac );

	printf( "MAC: %02X:%02X:%02X:%02X:%02X:%02X\n", hip.self_mac.mac[0], hip.self_mac.mac[1], hip.self_mac.mac[2],
		hip.self_mac.mac[3], hip.self_mac.mac[4], hip.self_mac.mac[5] );
	printf( "Waiting for DHCP lease...\n" );

	const uint32_t ticks_per_ms = ( FUNCONF_SYSTEM_CORE_CLOCK / 1000 );
	const uint32_t poll_interval_ms = 100;
	uint64_t last_tick_ms = SysTick->CNT / ticks_per_ms;
	uint64_t last_poll_ms = last_tick_ms;

	while ( 1 )
	{
		process_rx_packets();

		uint64_t now_ms = SysTick->CNT / ticks_per_ms;

		if ( now_ms > last_tick_ms )
		{
			sfhip_tick( &hip, &scratch, now_ms - last_tick_ms );
			last_tick_ms = now_ms;
		}

		// poll link status periodically
		if ( ( now_ms - last_poll_ms ) >= poll_interval_ms )
		{
			eth_poll_link();
			last_poll_ms = now_ms;
		}
	}
}
