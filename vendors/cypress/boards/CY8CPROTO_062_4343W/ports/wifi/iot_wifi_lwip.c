 /* Amazon FreeRTOS Wi-Fi V1.0.0
 * Copyright (C) 2018 Amazon.com, Inc. or its affiliates.  All Rights Reserved.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy of
 * this software and associated documentation files (the "Software"), to deal in
 * the Software without restriction, including without limitation the rights to
 * use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of
 * the Software, and to permit persons to whom the Software is furnished to do so,
 * subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS
 * FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR
 * COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER
 * IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
 * CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 *
 * http://aws.amazon.com/freertos
 * http://www.FreeRTOS.org
 */

/**
 * @file iot_wifi_lwip.c
 * @brief Wi-Fi Interface functions specifi to lwIP.
 */

/* Socket and Wi-Fi interface includes. */
#include <FreeRTOS.h>
#include <task.h>
#include <iot_wifi.h>
#include <iot_wifi_common.h>

/* Wi-Fi configuration includes. */
#include "aws_wifi_config.h"

/* Board and Abstraction layer includes */
#include <cy_result.h>
#include <cybsp_wifi.h>
#include <cyabs_rtos.h>
#include <cy_utils.h>
#include <cy_network_buffer.h>

/* Wi-Fi Host driver includes. */
#include "whd.h"
#include "whd_wifi_api.h"
#include "whd_network_types.h"

#define CY_USE_LWIP

#if defined(CY_USE_LWIP)

/* lwIP stack includes */
#include <lwipopts.h>
#include <lwip/dhcp.h>
#include <lwip/dns.h>
#include <lwip/inet.h>
#include <lwip/netdb.h>
#include <lwip/netif.h>
#include <lwip/netifapi.h>
#include <lwip/init.h>
#include <lwip/dhcp.h>
#include <lwip/etharp.h>
#include <lwip/tcpip.h>
#include <lwip/ethip6.h>
#include <lwip/igmp.h>
#include <lwip/icmp.h>
#include <lwip/inet_chksum.h>
#include <netif/ethernet.h>

#define MULTICAST_IP_TO_MAC(ip)       { (uint8_t) 0x01,             \
                                        (uint8_t) 0x00,             \
                                        (uint8_t) 0x5e,             \
                                        (uint8_t) ((ip)[1] & 0x7F), \
                                        (uint8_t) (ip)[2],          \
                                        (uint8_t) (ip)[3]           \
                                      }

#define CY_MAX_DHCP_ITERATION_COUNT        (1000)
#define CY_DHCP_NOT_COMPLETE               (3)
#define PING_ID              (0xAFAF)
#define PING_DATA_SIZE       (32)      /** ping additional data size to include in the packet */

/** Static IP address structure */
typedef struct
{
    ip_addr_t addr ;
    ip_addr_t netmask ;
    ip_addr_t gateway ;
} ip_static_addr_t ;

static ip_static_addr_t staticAddr = 
{
    IPADDR4_INIT_BYTES(configIP_ADDR0, configIP_ADDR1, configIP_ADDR2, configIP_ADDR3),
    IPADDR4_INIT_BYTES(configNET_MASK0, configNET_MASK1, configNET_MASK2, configNET_MASK3),
    IPADDR4_INIT_BYTES(configGATEWAY_ADDR0, configGATEWAY_ADDR1, configGATEWAY_ADDR2, configGATEWAY_ADDR3)
};
static struct netif     sta_ip_handle;
#define STA_IP_HANDLE	&sta_ip_handle
struct netif *ip_handle[ 3 ] =
{
    [WHD_STA_ROLE] = STA_IP_HANDLE,
    [WHD_AP_ROLE]  = NULL,
};
#define IP_HANDLE(interface)   (ip_handle[(interface) & 3])




static struct netif *netInterface = NULL;
static bool isNetworkUp = false;
static bool isNetifAdded = false;
static bool isApStarted = false;
static bool isDhcpEnabled = false;
//struct icmp_packet
//{
//    struct icmp_echo_hdr hdr;
//    uint8_t data[PING_DATA_SIZE];
//
//};
//static struct icmp_packet ping_packet;
static uint16_t     ping_seq_num;

/*----------------------- lwIP Helpers -------------------------------*/

void cy_network_process_ethernet_data(whd_interface_t ifp, whd_buffer_t buf)
{
    if (netInterface != NULL)
    {
        if (netInterface->input(buf, netInterface) != ERR_OK)
        {
            cy_buffer_release(buf, WHD_NETWORK_RX);
        }
    }
    else
    {
        cy_buffer_release(buf, WHD_NETWORK_RX);
    }
}

static struct pbuf *pbuf_dup(const struct pbuf *orig)
{
    struct pbuf *p = pbuf_alloc(PBUF_LINK, orig->tot_len, PBUF_RAM);
    if (p != NULL)
    {
        pbuf_copy(p, orig);
        p->flags = orig->flags;
    }
    return p;
}

static err_t low_level_output(struct netif *iface, struct pbuf *p)
{
    if (whd_wifi_is_ready_to_transceive((whd_interface_t)iface->state) != WHD_SUCCESS)
    {
        printf("wifi is not ready, packet not sent\n");
        return ERR_INPROGRESS;
    }

    struct pbuf *whd_buf = pbuf_dup(p);
    if (whd_buf == NULL)
    {
        printf("failed to allocate buffer for outgoing packet\n");
        return ERR_MEM;
    }
    whd_network_send_ethernet_data((whd_interface_t)iface->state, whd_buf);
    return ERR_OK;
}

static err_t igmp_filter(struct netif *iface, const ip4_addr_t *group, enum netif_mac_filter_action action)
{
    whd_mac_t mac = { MULTICAST_IP_TO_MAC((uint8_t*)group) };

    switch ( action )
    {
        case NETIF_ADD_MAC_FILTER:
            if ( whd_wifi_register_multicast_address( (whd_interface_t)iface->state, &mac ) != CY_RSLT_SUCCESS )
            {
                return ERR_VAL;
            }
            break;

        case NETIF_DEL_MAC_FILTER:
            if ( whd_wifi_unregister_multicast_address( (whd_interface_t)iface->state, &mac ) != CY_RSLT_SUCCESS )
            {
                return ERR_VAL;
            }
            break;

        default:
            return ERR_VAL;
    }
    return ERR_OK;
}

static err_t ethernetif_init(struct netif *iface)
{
    cy_rslt_t res;
    whd_mac_t macaddr;    

    iface->output = etharp_output;
    iface->linkoutput = low_level_output;
    iface->name[0] = 'w';
    iface->name[1] = 'l';

    res = whd_wifi_get_mac_address((whd_interface_t)iface->state, &macaddr);
    if (res != CY_RSLT_SUCCESS)
    {
        printf("initLWIP: whd_wifi_get_mac_address call failed, err = %lx", res);
        return res;
    }  

    memcpy(&iface->hwaddr, &macaddr, sizeof(macaddr));
    iface->hwaddr_len = sizeof(macaddr);

    iface->mtu = WHD_LINK_MTU;
    iface->flags |= (NETIF_FLAG_BROADCAST | NETIF_FLAG_ETHARP | NETIF_FLAG_IGMP);

#if LWIP_IPV6 == 1
    iface->output_ip6 = ethip6_output;

    iface->ip6_autoconfig_enabled = 1;
    netif_create_ip6_linklocal_address(iface, 1);
    netif_set_igmp_mac_filter(iface, igmp_filter);
    netif_set_mld_mac_filter(iface, NULL);

    macaddr.octet[0] = 0x33;
    macaddr.octet[1] = 0x33;
    macaddr.octet[2] = 0xff;  
    whd_wifi_register_multicast_address((whd_interface_t)iface->state, &macaddr);

    memset(&macaddr, 0, sizeof(macaddr));
    macaddr.octet[0] = 0x33;
    macaddr.octet[1] = 0x33;
    macaddr.octet[5] = 0x01;
    whd_wifi_register_multicast_address((whd_interface_t)iface->state, &macaddr);

    memset(&macaddr, 0, sizeof(macaddr));
    macaddr.octet[0] = 0x33;
    macaddr.octet[1] = 0x33;
    macaddr.octet[5] = 0x02;
    whd_wifi_register_multicast_address((whd_interface_t)iface->state, &macaddr);
#endif

    return ERR_OK;
}

void checkDhcpStatus(void * arg)
{
    cy_rslt_t* result = (cy_rslt_t *)arg;
    *result = dhcp_supplied_address(netInterface) ? CY_RSLT_SUCCESS : CY_DHCP_NOT_COMPLETE;
}

static cy_rslt_t cywifi_lwip_bringup(void)
{
    cy_rslt_t result = CY_RSLT_SUCCESS;
    int itr_count = 0;
    netifapi_netif_set_link_up(netInterface);
    netifapi_netif_set_up(netInterface);
    if (isDhcpEnabled)
    {
        netifapi_dhcp_start(netInterface);
        result = CY_DHCP_NOT_COMPLETE;
        while(result != CY_RSLT_SUCCESS && itr_count < CY_MAX_DHCP_ITERATION_COUNT)
        {
            tcpip_callback(checkDhcpStatus, &result);
            vTaskDelay(pdMS_TO_TICKS(10));
        }
    }
    return result;
}

static cy_rslt_t cywifi_lwip_bringdown(void)
{
    /* Bring down DHCP */
    if (isDhcpEnabled) {
        netifapi_dhcp_release(netInterface);
        netifapi_dhcp_stop(netInterface);
    }
    netifapi_netif_set_down(netInterface);
    netifapi_netif_set_link_down(netInterface);

    return CY_RSLT_SUCCESS;
}

static WIFIReturnCode_t network_stack_bringup(whd_interface_t iface, const ip_static_addr_t* ip_addr)
{
    if (!isNetifAdded)
    {
        configASSERT(netInterface == NULL);
        netInterface = (struct netif *)calloc(sizeof(struct netif), 1);
        netifapi_netif_add(
            netInterface,
            0,
            0,
            0,
            iface,
            ethernetif_init,
            tcpip_input);
        netifapi_netif_set_default(netInterface);
        isNetifAdded = true;
    }

    if (ip_addr != NULL)
    {
        netifapi_netif_set_addr(
            netInterface, 
            &ip_addr->addr.u_addr.ip4,
            &ip_addr->netmask.u_addr.ip4,
            &ip_addr->gateway.u_addr.ip4);
        isDhcpEnabled = false;
    }
    else
    {
        isDhcpEnabled = true;
    }

    if (cywifi_lwip_bringup() != CY_RSLT_SUCCESS)
    {
        return eWiFiFailure;
    }

    return eWiFiSuccess;
}

/*-----------------------------------------------------------*/

WIFIReturnCode_t WIFI_Off( void )
{
    if (isPoweredUp)
    {
        if ((devMode == eWiFiModeStation && WIFI_Disconnect() != eWiFiSuccess) ||
            (devMode == eWiFiModeAP && WIFI_StopAP() != eWiFiSuccess))
        {
            return eWiFiFailure;
        }

        if (wifiMutex != NULL && cy_rtos_deinit_mutex(&wifiMutex) != CY_RSLT_SUCCESS)
        {
            return eWiFiFailure;
        }

        if(isNetifAdded)
        {
            netifapi_netif_remove(netInterface);
            free(netInterface);
            netInterface = NULL;
            isNetifAdded = false;
        }

        if (cybsp_wifi_deinit(primaryInterface) != CY_RSLT_SUCCESS)
        {
            return eWiFiFailure;
        }

        isPoweredUp = false;
        devMode = eWiFiNotSupported;
    }
    return eWiFiSuccess;
}

/*-----------------------------------------------------------*/

WIFIReturnCode_t WIFI_ConnectAP( const WIFINetworkParams_t * const pxNetworkParams )
{
    cy_rslt_t res = CY_RSLT_SUCCESS;
    whd_ssid_t ssid;
    uint8_t *key;
    uint8_t keylen;
    whd_security_t security;
    uint8_t channel;

    cy_check_network_params(pxNetworkParams);
    
    if (isConnected && WIFI_Disconnect() != eWiFiSuccess)
    {
        return eWiFiFailure;
    }

    if (devMode != eWiFiModeStation &&
        devMode != eWiFiModeNotSupported &&
         WIFI_SetMode(eWiFiModeStation) != eWiFiSuccess)
    {
        return eWiFiFailure;
    }

    if (cy_rtos_get_mutex(&wifiMutex, wificonfigMAX_SEMAPHORE_WAIT_TIME_MS) == CY_RSLT_SUCCESS)
    {
        cy_convert_network_params(pxNetworkParams, &ssid, &key, &keylen, &security, &channel);

        res = whd_wifi_join(primaryInterface, &ssid, security, key, keylen);
        if (res != CY_RSLT_SUCCESS)
        {
            cy_rtos_set_mutex(&wifiMutex);
            return eWiFiFailure;
        }

        isConnected = true;
        devMode = eWiFiModeStation;

        if (!isNetworkUp)
        {
            if (network_stack_bringup(primaryInterface, (LWIP_DHCP == 1) ? NULL : &staticAddr) != eWiFiSuccess)
            {
                configPRINTF(("Failed to bring up network stack\n"));
                cy_rtos_set_mutex(&wifiMutex);
                return eWiFiFailure;
            }
            isNetworkUp = true;
        }
        if (cy_rtos_set_mutex(&wifiMutex) != CY_RSLT_SUCCESS)
        {
            return eWiFiFailure;
        }
    }
    else
    {
        return eWiFiTimeout;
    }
    return eWiFiSuccess;
}
/*-----------------------------------------------------------*/

WIFIReturnCode_t WIFI_Disconnect( void )
{
    if (isConnected)
    {
        if (cy_rtos_get_mutex(&wifiMutex, wificonfigMAX_SEMAPHORE_WAIT_TIME_MS) == CY_RSLT_SUCCESS)
        {
            if (cywifi_lwip_bringdown() != CY_RSLT_SUCCESS)
            {
                cy_rtos_set_mutex(&wifiMutex);
                return eWiFiFailure;
            }
            isNetworkUp = false;
            if (whd_wifi_leave(primaryInterface) != CY_RSLT_SUCCESS)
            {
                cy_rtos_set_mutex(&wifiMutex);
                return eWiFiFailure;
            }
            isConnected = false;
            if (cy_rtos_set_mutex(&wifiMutex) != CY_RSLT_SUCCESS)
            {
                return eWiFiFailure;
            }
        }
        else
        {
            return eWiFiTimeout;
        }
    }
    return eWiFiSuccess;
}

/*-----------------------------------------------------------*/

WIFIReturnCode_t WIFI_GetIP( uint8_t * pucIPAddr )
{
    configASSERT(pucIPAddr != NULL);
    if (cy_rtos_get_mutex(&wifiMutex, wificonfigMAX_SEMAPHORE_WAIT_TIME_MS) == CY_RSLT_SUCCESS)
    {
        if (pucIPAddr == NULL && netInterface == NULL)
        {
            cy_rtos_set_mutex(&wifiMutex);
            return eWiFiFailure;
        }
        memcpy(pucIPAddr, &netInterface->ip_addr.u_addr.ip4, sizeof(netInterface->ip_addr.u_addr.ip4));
        if (cy_rtos_set_mutex(&wifiMutex) != CY_RSLT_SUCCESS)
        {
            return eWiFiFailure;
        }
    }
    else
    {
        return eWiFiTimeout;
    }
    return eWiFiSuccess;
}


/*-----------------------------------------------------------*/
WIFIReturnCode_t WIFI_GetGW(uint8_t *pucIPAddr)
{
	configASSERT(pucIPAddr != NULL);
	if (cy_rtos_get_mutex(&wifiMutex, wificonfigMAX_SEMAPHORE_WAIT_TIME_MS) == CY_RSLT_SUCCESS)
	{
		if (pucIPAddr == NULL && netInterface == NULL)
		{
			cy_rtos_set_mutex(&wifiMutex);
			return eWiFiFailure;
		}
		memcpy(pucIPAddr, &netInterface ->gw.u_addr.ip4, sizeof(netInterface->gw.u_addr.ip4));
		if (cy_rtos_set_mutex(&wifiMutex) != CY_RSLT_SUCCESS)
		{
			return eWiFiFailure;
		}
	}
	else
	    {
	        return eWiFiTimeout;
	    }
	    return eWiFiSuccess;
}
static void ping_prepare_echo( struct icmp_echo_hdr *iecho, uint16_t len )
{
    int i, payload_size;

    payload_size = len - sizeof(struct icmp_echo_hdr);

    ICMPH_TYPE_SET(iecho, ICMP_ECHO);
    ICMPH_CODE_SET(iecho, 0);
    iecho->chksum = 0;
    iecho->id = PING_ID;
    iecho->seqno = htons( ++ping_seq_num );

    /* fill the additional data buffer with some data */
    for ( i = 0; i < payload_size; i++ )
    {
    	 ( (char *)iecho )[sizeof(struct icmp_echo_hdr) + i] = i;
    }

    iecho->chksum = inet_chksum(iecho, len);
}

#if 0
WIFIReturnCode_t ping_send(int socket_handle, ip_addr_t *IPAddr )
{
	int err;
	struct sockaddr_in to;
	struct icmp_packet *iecho = &ping_packet;


	/* Construct ping request */
	ping_prepare_echo( iecho, sizeof(ping_packet) );

	/* Send the ping request */
	to.sin_len         = sizeof( to );
	to.sin_family      = AF_INET;
	to.sin_addr.s_addr = (htonl(IPAddr ->u_addr.ip4.addr));

	err = lwip_sendto( socket_handle, iecho, sizeof(ping_packet), 0, (struct sockaddr*) &to, sizeof( to ) );

	return ( err ? ERR_OK : ERR_VAL );

}
#endif
static err_t ping_recv( int socket_handle )
{
    char                  buf[64];
    int                   fromlen;
    int                   len;
    struct sockaddr_in    from;
    struct ip_hdr*        iphdr;
    struct icmp_echo_hdr* iecho;

    do
    {
        len = lwip_recvfrom( socket_handle, buf, sizeof( buf ), 0, (struct sockaddr*) &from, (socklen_t*) &fromlen );

        if ( len >= (int) ( sizeof(struct ip_hdr) + sizeof(struct icmp_echo_hdr) ) )
        {
            iphdr = (struct ip_hdr *) buf;
            iecho = (struct icmp_echo_hdr *) ( buf + ( IPH_HL( iphdr ) * 4 ) );

            if ( ( iecho->id == PING_ID ) &&
                 ( iecho->seqno == lwip_htons( ping_seq_num ) ) &&
                 ( ICMPH_TYPE( iecho ) == ICMP_ER ) )
            {
                return ERR_OK; /* Echo reply received - return success */
            }
        }
    } while (len > 0);

    return ERR_TIMEOUT; /* No valid echo reply received before timeout */
}

WIFIReturnCode_t WIFI_Ping( uint8_t * pucIPAddr,
                            uint16_t usCount,
                            uint32_t ulIntervalMS )
{
	int result = 0;
	int i = 0, len = 300;
	configASSERT(pucIPAddr != NULL || usCount != 0);
	ip_addr_t target;
	whd_time_t send_time, curr_time;
	struct sockaddr_in host_addr;

	if (cy_rtos_get_mutex(&wifiMutex, wificonfigMAX_SEMAPHORE_WAIT_TIME_MS) == CY_RSLT_SUCCESS)
	{
		target.u_addr.ip4.addr= netif_ip4_gw(netInterface)->addr;
		configPRINTF(("Pinging: %s\n", ipaddr_ntoa(&target)));

		host_addr.sin_addr.s_addr = target.u_addr.ip4.addr;
		host_addr.sin_len = sizeof(host_addr);
		host_addr.sin_family = AF_INET;

		/*Open a socket for pinging */
		int socket_for_ping = lwip_socket(AF_INET, SOCK_RAW, IP_PROTO_ICMP);
		if(socket_for_ping < 0)
		{
			configPRINTF(("unable to create socket for ping\r\n"));
			return eWiFiFailure;
		}

		/* Set the receive timeout on local socket so pings will time out. */
		lwip_setsockopt( socket_for_ping, SOL_SOCKET, SO_RCVTIMEO, &ulIntervalMS, sizeof( ulIntervalMS ) );




		/* Set to default the requested interface */

//		netifapi_netif_set_default(IP_HANDLE(WHD_STA_ROLE));
//		netifapi_netif_add(
//		            netInterface,
//		            0,
//		            0,
//		            0,
//		            primaryInterface,
//		            ethernetif_init,
//		            tcpip_input);
#if 0
		netifapi_netif_set_up(netInterface);

		for (i = 0; i <= usCount; i++)
		{

			result = ping_send(socket_for_ping, &target);

			if (result != eWiFiSuccess)
			{
				/* close a socket */
				lwip_close( socket_for_ping );
				cy_rtos_set_mutex(&wifiMutex);
				configPRINTF(("Unable to send Ping\r\n"));
				return eWiFiFailure;
			}
			/* Record time ping was sent */
			cy_rtos_get_time(&send_time);

			/* Wait for ping reply */
			result = ping_recv( socket_for_ping );
			if (ERR_OK == result)
			{
				cy_rtos_get_time(&curr_time);
				configPRINTF( ("Ping Reply %dms\n", (int)(curr_time - send_time) ) );
			}
			else
			{
				configPRINTF( ("Ping timeout\n") );
			}
#endif
			struct icmp_echo_hdr *iecho;
			uint16_t ping_size = sizeof(struct icmp_echo_hdr) + len;

			/*Allocate memory for packet */
			if (!(iecho = mem_malloc(ping_size)))
			{
				return ERR_MEM;

			}

			/*Construct ping request */
			ping_prepare_echo(iecho, ping_size);

			while (i <= usCount)
			{
				cy_rtos_get_time(&send_time);

				netif_set_default(netInterface);

				if (lwip_sendto(socket_for_ping, iecho, ping_size, 0, (struct sockaddr *)&host_addr, host_addr.sin_len) > 0)
				{
					/* Wait for ping reply */
					err_t result = ping_recv(socket_for_ping);
					cy_rtos_get_time(&curr_time);
					if (ERR_OK == result)
					{
						whd_time_t ping_time = curr_time - send_time;
						configPRINTF(("Ping reply %d ms\r\n", (int)ping_time));

					}

					/* Sleep until time for next ping */
					sys_msleep(1000);
				}

				else
				{
					configPRINTF(("Ping timeout \r\n"));
				}

				i++;
			}

			mem_free(iecho);




		/* close a socket */
		lwip_close( socket_for_ping );

		return result;
	}

	if (cy_rtos_set_mutex(&wifiMutex) != CY_RSLT_SUCCESS)
	{
		return eWiFiFailure;
	}
	return eWiFiSuccess;
}
/*-----------------------------------------------------------*/

WIFIReturnCode_t WIFI_GetHostIP( char * pcHost,
                                 uint8_t * pucIPAddr )
{
    configASSERT(pcHost != NULL && pucIPAddr != NULL);

    if (cy_rtos_get_mutex(&wifiMutex, wificonfigMAX_SEMAPHORE_WAIT_TIME_MS) == CY_RSLT_SUCCESS)
    {
        int                 err;
        struct addrinfo     hints;
        struct addrinfo     *result;
        struct sockaddr_in  *addr_in;

        memset(&hints, 0, sizeof(hints));
        hints.ai_family     = AF_INET;
        hints.ai_socktype   = SOCK_DGRAM;
        hints.ai_flags      = 0;
        hints.ai_protocol   = 0;

        err = getaddrinfo(pcHost, NULL, &hints, &result);
        if (err != 0)
        {
            cy_rtos_set_mutex(&wifiMutex);
            return eWiFiFailure;
        }

        if (result)
        {
            /*
            * ai_addr is of struct sockaddr type.
            *
                struct sockaddr {
                u8_t        sa_len;
                sa_family_t sa_family;
                char        sa_data[14];
                };
            */
            addr_in = (struct sockaddr_in *)result->ai_addr;

            memcpy(pucIPAddr, &addr_in->sin_addr.s_addr,
                    sizeof(addr_in->sin_addr.s_addr));
        }
        else
        {
            cy_rtos_set_mutex(&wifiMutex);
            return eWiFiFailure;
        }

        freeaddrinfo(result);
        if (cy_rtos_set_mutex(&wifiMutex) != CY_RSLT_SUCCESS)
        {
            return eWiFiFailure;
        }
    }
    else
    {
        return eWiFiTimeout;
    }
    return eWiFiSuccess;
}
/*-----------------------------------------------------------*/

WIFIReturnCode_t WIFI_StartAP( void )
{
//	whd_interface_t secondaryInterface;

    if (!isApStarted)
    {
        if (cy_rtos_get_mutex(&wifiMutex, wificonfigMAX_SEMAPHORE_WAIT_TIME_MS) == CY_RSLT_SUCCESS)
        {

            cy_rslt_t res = whd_wifi_start_ap(secondaryInterface);
            if (res != CY_RSLT_SUCCESS)
            {
                configPRINTF(("Failed to start AP.\n"));
                cy_rtos_set_mutex(&wifiMutex);
                return eWiFiFailure;
            }
            isApStarted = true;

            if (!isNetworkUp)
            {
                if (network_stack_bringup(secondaryInterface, &staticAddr) !=  eWiFiSuccess)
                {
                    configPRINTF(("Failed to bring up network stack\n"));
                    cy_rtos_set_mutex(&wifiMutex);
                    return eWiFiFailure;
                }
                isNetworkUp = true;
            }
            if (cy_rtos_set_mutex(&wifiMutex) != CY_RSLT_SUCCESS)
            {
                return eWiFiFailure;
            }
        }
        else
        {
            return eWiFiTimeout;
        }
    }
    return eWiFiSuccess;
}
/*-----------------------------------------------------------*/

WIFIReturnCode_t WIFI_StopAP( void )
{
    if (isApStarted)
    {
        if (cy_rtos_get_mutex(&wifiMutex, wificonfigMAX_SEMAPHORE_WAIT_TIME_MS) == CY_RSLT_SUCCESS)
        {
            if (cywifi_lwip_bringdown() != CY_RSLT_SUCCESS)
            {
                cy_rtos_set_mutex(&wifiMutex);
                return eWiFiFailure;
            }
            isNetworkUp = false;
            if (whd_wifi_stop_ap(primaryInterface) != CY_RSLT_SUCCESS)
            {
                cy_rtos_set_mutex(&wifiMutex);
                return eWiFiFailure;
            }
            isApStarted = false;
            if (cy_rtos_set_mutex(&wifiMutex) != CY_RSLT_SUCCESS)
            {
                return eWiFiFailure;
            }
        }
        else
        {
            return eWiFiTimeout;
        }
    }
    return eWiFiSuccess;
}


/*-----------------------------------------------------------*/
#endif /* CY_USE_LWIP */
