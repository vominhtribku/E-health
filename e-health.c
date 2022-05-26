/*
 * Copyright (c) 2016, Zolertia - http://www.zolertia.com
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. Neither the name of the Institute nor the names of its contributors
 *    may be used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE INSTITUTE AND CONTRIBUTORS ``AS IS'' AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE INSTITUTE OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 *
 * This file is part of the Contiki operating system.
 *
 */
/*---------------------------------------------------------------------------*/
#include "contiki.h"
#include "contiki-lib.h"
#include "contiki-net.h"
#include "net/ip/uip-debug.h"
#include "dev/leds.h"
#include "net/rpl/rpl.h"
#include "dev/watchdog.h"
#include "lib/random.h"
#include "sys/etimer.h"
#include "sys/ctimer.h"
#include "e-health.h"	
#include "net/ip/uip.h"
#include "net/ipv6/uip-ds6.h"
#include "net/ip/uip-udp-packet.h"
#include <stdio.h>
#include <string.h>

#ifdef USING_CC2650
#include "board-peripherals.h"
#include "rf-core/rf-ble.h"
#include "ti-lib.h"
#include "dev/cc26xx-uart.h"
#endif

#ifdef USING_CC2538DK
#include "dev/uart.h"
#include "dev/gpio.h"
#endif

#include "dev/button-sensor.h"
/*---------------------------------------------------------------------------*/
/* Enables printing debug output from the IP/IPv6 libraries */
#define DEBUG DEBUG_PRINT
#include "net/ip/uip-debug.h"
/*---------------------------------------------------------------------------*/
/* Default is to send a packet every 60 seconds */
#define MAX_PAYLOAD_LEN 	8
#define SEND_INTERVAL		(15 * CLOCK_SECOND)

static  char rxbuf[MAX_PAYLOAD_LEN];		/* used for UART0 interface */
static 	int cmd_cnt;

/*---------------------------------------------------------------------------*/
/* The structure used in the Simple UDP library to create an UDP connection */
static struct uip_udp_conn *client_conn;

/* This is the server IPv6 address */
static uip_ipaddr_t server_ipaddr;

/* Keeps account of the number of messages sent */
static uint16_t counter = 0;

static 	int uart0_input_byte(unsigned char c);

/*---------------------------------------------------------------------------*/
/* Create a structure and pointer to store the data to be sent as payload */
static struct my_msg_t msg;
static struct my_msg_t *msgPtr = &msg;
/*---------------------------------------------------------------------------*/
PROCESS(udp_client_process, "UDP client example process");
AUTOSTART_PROCESSES(&udp_client_process);
/*---------------------------------------------------------------------------*/
/* Whenever we receive a packet from another node (or the server), this callback
 * is invoked.  We use the "uip_newdata()" to check if there is actually data for
 * us
 */
/*---------------------------------------------------------------------------*/
static void init_default_parameters(void) {
	// init UART0-1
#ifdef USING_CC2538DK
	uart_init(0); 		
 	uart_set_input(0,uart0_input_byte);
#endif

#ifdef USING_CC2650
	cc26xx_uart_init();		
 	cc26xx_uart_set_input(uart0_input_byte);
#endif

}

/*---------------------------------------------------------------------------*/
static void
tcpip_handler(void)
{
  char *str;

  if(uip_newdata()) {
    str = uip_appdata;
    str[uip_datalen()] = '\0';
    printf("Received from the server: '%s'\n", str);
  }
}
/*---------------------------------------------------------------------------*/
static int uart0_input_byte(unsigned char c) {
	if (c==SFD) {
		cmd_cnt=1;
		rxbuf[cmd_cnt-1]=c;
	}
	else {
		cmd_cnt++;
		rxbuf[cmd_cnt-1] = c;	
	}
	return 1;
}
/*---------------------------------------------------------------------------*/
static void
send_packet(void)
{
  counter++;
  msg.sfd      	= rxbuf[0];
  msg.counter 	= counter;
  msg.type 	= rxbuf[1];
  msg.temper1 	= rxbuf[2];
  msg.temper2  	= rxbuf[3];
  msg.spo2bpm  	= rxbuf[4];
  msg.systolic  = rxbuf[5];
  msg.diastolic = rxbuf[6];
  msg.spo2 	= rxbuf[7];
  /* Print the sensor data */
  printf("sfd: %d, type: %d, phan nguyen nhiet do: %d, phan thuc nhiet do: %d, nhip tim: %d, huyet ap tam thu: %d, huyet ap tam truong: %d, spo2: %d, counter: %u\n",
          msg.sfd, msg.type, msg.temper1, msg.temper2, msg.spo2bpm,
          msg.systolic, msg.diastolic, msg.spo2, msg.counter);

  PRINTF("Send readings to %u'\n",
                                server_ipaddr.u8[sizeof(server_ipaddr.u8) - 1]);

  uip_udp_packet_sendto(client_conn, msgPtr, sizeof(msg),
                        &server_ipaddr, UIP_HTONS(UDP_SERVER_PORT));
}
/*---------------------------------------------------------------------------*/
static void
print_local_addresses(void)
{
  int i;
  uint8_t state;

  PRINTF("Client IPv6 addresses:\n");
  for(i = 0; i < UIP_DS6_ADDR_NB; i++) {
    state = uip_ds6_if.addr_list[i].state;
    if(uip_ds6_if.addr_list[i].isused &&
       (state == ADDR_TENTATIVE || state == ADDR_PREFERRED)) {
      PRINT6ADDR(&uip_ds6_if.addr_list[i].ipaddr);
      PRINTF("\n");
      /* hack to make address "final" */
      if (state == ADDR_TENTATIVE) {
        uip_ds6_if.addr_list[i].state = ADDR_PREFERRED;
      }
    }
  }
}
/*---------------------------------------------------------------------------*/
/* This is a hack to set ourselves the global address, use for testing */
//static void
//set_global_address(void)
//{
  //uip_ipaddr_t ipaddr;

/* The choice of server address determines its 6LoWPAN header compression.
 * (Our address will be compressed Mode 3 since it is derived from our link-local address)
 * Obviously the choice made here must also be selected in udp-server.c.
 *
 * For correct Wireshark decoding using a sniffer, add the /64 prefix to the 6LowPAN protocol preferences,
 * e.g. set Context 0 to fd00::.  At present Wireshark copies Context/128 and then overwrites it.
 * (Setting Context 0 to fd00::1111:2222:3333:4444 will report a 16 bit compressed address of fd00::1111:22ff:fe33:xxxx)
 *
 * Note the IPCMV6 checksum verification depends on the correct uncompressed addresses.
 */

  //uip_ip6addr(&ipaddr, 0xaaaa, 0, 0, 0, 0, 0, 0, 0);
  //uip_ds6_set_addr_iid(&ipaddr, &uip_lladdr);
  //uip_ds6_addr_add(&ipaddr, 0, ADDR_AUTOCONF);
//}
/*---------------------------------------------------------------------------*/
PROCESS_THREAD(udp_client_process, ev, data)
{
  static struct etimer periodic;

  PROCESS_BEGIN();

  PROCESS_PAUSE();

  /* Remove the comment to set the global address ourselves, as it is it will
   * obtain the IPv6 prefix from the DODAG root and create its IPv6 global
   * address
   */
  /* set_global_address(); */

  printf("UDP client process started\n");

  init_default_parameters();

  /* Set the server address here */ 
  uip_ip6addr(&server_ipaddr, 0xaaaa, 0, 0, 0, 0, 0, 0, 1);

  printf("Server address: ");
  PRINT6ADDR(&server_ipaddr);
  printf("\n");

  /* Print the node's addresses */
  print_local_addresses();

  SENSORS_ACTIVATE(button_sensor);

  /* Create a new connection with remote host.  When a connection is created
   * with udp_new(), it gets a local port number assigned automatically.
   * The "UIP_HTONS()" macro converts to network byte order.
   * The IP address of the remote host and the pointer to the data are not used
   * so those are set to NULL
   */
  client_conn = udp_new(NULL, UIP_HTONS(UDP_SERVER_PORT), NULL); 

  if(client_conn == NULL) {
    PRINTF("No UDP connection available, exiting the process!\n");
    PROCESS_EXIT();
  }

  /* This function binds a UDP connection to a specified local por */
  udp_bind(client_conn, UIP_HTONS(UDP_CLIENT_PORT)); 

  PRINTF("Created a connection with the server ");
  PRINT6ADDR(&client_conn->ripaddr);
  PRINTF(" local/remote port %u/%u\n", UIP_HTONS(client_conn->lport),
                                       UIP_HTONS(client_conn->rport));

  etimer_set(&periodic, SEND_INTERVAL);

  while(1) {
    PROCESS_YIELD();

    /* Incoming events from the TCP/IP module */
    if(ev == tcpip_event) {
      tcpip_handler();
    }

    /* Send data to the server */
    if((ev == sensors_event && data == &button_sensor) ||
       (ev == PROCESS_EVENT_TIMER)) {

      send_packet();

      if(etimer_expired(&periodic)) {
        etimer_reset(&periodic);
      }
    }
  }

  PROCESS_END();
}
/*---------------------------------------------------------------------------*/

