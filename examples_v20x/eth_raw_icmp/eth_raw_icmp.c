#include "ch32fun.h"
#include <stdio.h>

#define MY_IP_ADDR { 192, 168, 1, 100 }

#define CH32V208_ETH_IMPLEMENTATION
#include "../../extralibs/ch32v208_eth.h"

static uint8_t g_my_ip[4] = MY_IP_ADDR;
static uint8_t g_my_mac[6];

static uint16_t checksum( const uint8_t *data, uint16_t len )
{
	uint32_t sum = 0;
	const uint16_t *ptr = (const uint16_t *)data;
	while ( len > 1 )
	{
		sum += *ptr++;
		len -= 2;
	}
	if ( len ) sum += *(uint8_t *)ptr;
	while ( sum >> 16 ) sum = ( sum & 0xFFFF ) + ( sum >> 16 );
	return ~sum;
}

static void eth_rx_callback( const uint8_t *pkt, uint16_t len )
{
	if ( len < 42 ) return;

	// ARP request?
	if ( pkt[12] == 0x08 && pkt[13] == 0x06 && // EtherType ARP
		 pkt[20] == 0x00 && pkt[21] == 0x01 && // Opcode = request
		 memcmp( pkt + 38, g_my_ip, 4 ) == 0 )
	{ // Target IP = us

		uint8_t reply[42];
		memcpy( reply, pkt + 6, 6 ); // Dst MAC
		memcpy( reply + 6, g_my_mac, 6 ); // Src MAC
		reply[12] = 0x08;
		reply[13] = 0x06; // ARP
		reply[14] = 0x00;
		reply[15] = 0x01; // Ethernet
		reply[16] = 0x08;
		reply[17] = 0x00; // IPv4
		reply[18] = 6;
		reply[19] = 4; // Hardware/Protocol size
		reply[20] = 0x00;
		reply[21] = 0x02; // Opcode = reply
		memcpy( reply + 22, g_my_mac, 6 );
		memcpy( reply + 28, g_my_ip, 4 );
		memcpy( reply + 32, pkt + 22, 10 ); // Requester's MAC+IP
		eth_send_packet( reply, 42 );
		return;
	}

	// ICMP ping request?
	if ( pkt[12] == 0x08 && pkt[13] == 0x00 && // IPv4
		 pkt[23] == 0x01 && // Protocol = ICMP
		 pkt[34] == 0x08 && // Type = echo request
		 memcmp( pkt + 30, g_my_ip, 4 ) == 0 )
	{ // Dst IP = us

		uint16_t ip_len = ( pkt[16] << 8 ) | pkt[17];
		uint8_t reply[1514];

		memcpy( reply, pkt, 14 + ip_len ); // Copy entire packet
		memcpy( reply, pkt + 6, 6 ); // Swap MACs
		memcpy( reply + 6, g_my_mac, 6 );
		memcpy( reply + 26, pkt + 30, 4 ); // Swap IPs
		memcpy( reply + 30, pkt + 26, 4 );

		reply[24] = 0;
		reply[25] = 0; // Recalc IP checksum
		*(uint16_t *)( reply + 24 ) = checksum( reply + 14, 20 );

		reply[34] = 0; // Type = echo reply
		reply[36] = 0;
		reply[37] = 0; // Recalc ICMP checksum
		*(uint16_t *)( reply + 36 ) = checksum( reply + 34, ip_len - 20 );

		eth_send_packet( reply, 14 + ip_len );
	}
}

static void link_status_callback( bool link_up )
{
	printf( "ETH: Link %s\n", link_up ? "UP" : "DOWN" );
}

int main( void )
{
	SystemInit();
	printf( "CH32V208 ETH10M test\n" );

	eth_config_t cfg = { .mac_addr = NULL,
		.rx_callback = eth_rx_callback,
		.link_callback = link_status_callback,
		.promiscuous_mode = false,
		.broadcast_filter = true,
		.multicast_filter = false };

	if ( eth_init( &cfg ) != 0 )
	{
		printf( "ERROR: eth_init failed\n" );
		while ( 1 );
	}

	eth_get_mac_address( g_my_mac );
	printf( "IP:  %d.%d.%d.%d\n", g_my_ip[0], g_my_ip[1], g_my_ip[2], g_my_ip[3] );
	printf( "MAC: %02X:%02X:%02X:%02X:%02X:%02X\n", g_my_mac[0], g_my_mac[1], g_my_mac[2], g_my_mac[3], g_my_mac[4],
		g_my_mac[5] );
	printf( "\nWaiting for link...\n" );

	const uint32_t ticks_per_ms = ( FUNCONF_SYSTEM_CORE_CLOCK / 1000 );
	const uint32_t poll_interval_ticks = 100 * ticks_per_ms; // 100ms poll interval
	uint64_t last_poll_tick = SysTick->CNT;

	while ( 1 )
	{
		eth_process_rx();

		uint64_t now = SysTick->CNT;
		if ( ( now - last_poll_tick ) >= poll_interval_ticks )
		{
			eth_poll_link();
			last_poll_tick = now;
		}
	}
}
