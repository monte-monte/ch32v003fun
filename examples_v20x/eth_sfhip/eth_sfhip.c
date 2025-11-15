#include "ch32fun.h"
#include <stdio.h>

#define SFHIP_WARN( x... ) printf( x )
#define SFHIP_IMPLEMENTATION
#define HIP_PHY_HEADER_LENGTH_BYTES 0
#define SFHIP_TCP_SOCKETS 4

#include "sfhip.h"

#define ETH_RX_BUF_SIZE 1536
#define CH32V208_ETH_IMPLEMENTATION
#include "../../extralibs/ch32v208_eth.h"

#define HTTP_PORT 80

sfhip hip = {
	.ip = 0,
	.mask = 0,
	.gateway = 0,
	.self_mac = { { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 } },
	.hostname = "ch32v208",
	.need_to_discover = 1, // start w/ DHCP DISCOVER
};

static sfhip_phy_packet_mtu scratch __attribute__( ( aligned( 4 ) ) );

const char http_response[] = "HTTP/1.1 200 OK\r\n"
							 "Content-Type: text/plain\r\n"
							 "Content-Length: 22\r\n"
							 "Connection: close\r\n"
							 "\r\n"
							 "Hello from CH32V208!\r\n";

static bool response_sent[SFHIP_TCP_SOCKETS] = { false };

int sfhip_send_packet( sfhip *hip, sfhip_phy_packet *data, int length )
{
	return eth_send_packet( (const uint8_t *)data, length );
}

void sfhip_got_dhcp_lease( sfhip *hip, sfhip_address addr )
{
	uint32_t ip = HIPNTOHL( addr );
	printf( "\nGot IP: %lu.%lu.%lu.%lu\n", ( ip >> 24 ) & 0xFF, ( ip >> 16 ) & 0xFF, ( ip >> 8 ) & 0xFF, ip & 0xFF );
	printf( "HTTP server ready at http://%lu.%lu.%lu.%lu/\n\n", ( ip >> 24 ) & 0xFF, ( ip >> 16 ) & 0xFF,
		( ip >> 8 ) & 0xFF, ip & 0xFF );
}

static void link_status_callback( bool link_up )
{
	printf( "Link %s\n", link_up ? "UP" : "DOWN" );
}

int sfhip_tcp_accept_connection( sfhip *hip, int sockno, int localport, hipbe32 remote_host )
{
	if ( localport == HIPHTONS( HTTP_PORT ) ) return 0;
	return -1;
}


sfhip_length_or_tcp_code sfhip_tcp_event(
	sfhip *hip, int sockno, uint8_t *ip_payload, int ip_payload_length, int max_out_payload, int acked )
{
	if ( ip_payload_length > 0 && !response_sent[sockno] )
	{
		response_sent[sockno] = true;
		int response_len = sizeof( http_response ) - 1;
		if ( response_len > max_out_payload ) response_len = max_out_payload;
		memcpy( ip_payload, http_response, response_len );
		// printf( "." );
		return response_len;
	}

	// was ACKed, close conn
	if ( acked > 0 && response_sent[sockno] ) return SFHIP_TCP_OUTPUT_FIN;

	return 0;
}


void sfhip_tcp_socket_closed( sfhip *hip, int sockno )
{
	response_sent[sockno] = false;
}

int main( void )
{
	SystemInit();
	printf( "CH32V208 ETH10M test with sfhip (DHCP)\n" );

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

	eth_get_mac_address( (uint8_t *)&hip.self_mac );
	printf( "MAC: %02X:%02X:%02X:%02X:%02X:%02X\n", hip.self_mac.mac[0], hip.self_mac.mac[1], hip.self_mac.mac[2],
		hip.self_mac.mac[3], hip.self_mac.mac[4], hip.self_mac.mac[5] );

	const uint32_t ticks_per_ms = ( FUNCONF_SYSTEM_CORE_CLOCK / 1000 );
	const uint32_t poll_interval_ms = 100;
	uint64_t last_tick_ms = SysTick->CNT / ticks_per_ms;
	uint64_t last_poll_ms = last_tick_ms;

	while ( 1 )
	{
		uint16_t pkt_len;
		const uint8_t *pkt = eth_get_rx_packet( &pkt_len );

		if ( pkt && pkt_len > 0 && pkt_len <= SFHIP_MTU )
		{
			sfhip_accept_packet( &hip, (sfhip_phy_packet_mtu *)pkt, pkt_len );
			eth_release_rx_packet();
		}
		else if ( pkt )
		{
			eth_release_rx_packet();
		}

		uint64_t now_ms = SysTick->CNT / ticks_per_ms;

		if ( now_ms > last_tick_ms )
		{
			sfhip_tick( &hip, &scratch, now_ms - last_tick_ms );
			last_tick_ms = now_ms;
		}

		if ( ( now_ms - last_poll_ms ) >= poll_interval_ms )
		{
			eth_poll_link();
			last_poll_ms = now_ms;
		}
	}
}
