// Standard includes
#include <stdlib.h>
#include <string.h>

// Simplelink includes
#include "simplelink.h"
#include "netcfg.h"

// Driverlib includes
#include "hw_ints.h"
#include "hw_types.h"
#include "hw_memmap.h"
#include "hw_common_reg.h"
#include "interrupt.h"
#include "utils.h"
#include "rom.h"
#include "rom_map.h"
#include "prcm.h"
#include "pin.h"

// OS includes
#include "osi.h"
#include "freertos.h"
#include "task.h"

// Common interface includes
#include "gpio_if.h"
#include "uart_if.h"
#include "i2c_if.h"
#include "common.h"

// App Includes
#include "hooks.h"
#include "tmp006drv.h"
#include "bma222drv.h"
//#include "handlers.h"
#include "device_status.h"
#include "smartconfig.h"
#include "pinmux.h"

#define APPLICATION_VERSION              "1.1.0"
#define APP_NAME                         "AirU-Firmware"

#define OOB_TASK_PRIORITY                1
#define DATAGATHER_TASK_PRIORITY		 3
#define DATAUPLOAD_TASK_PRIORITY         2
#define SPAWN_TASK_PRIORITY              9

#define OSI_STACK_SIZE                   2048

#define AP_SSID_LEN_MAX                 32
#define SH_GPIO_3                       3       /* P58 - Device Mode */
#define AUTO_CONNECTION_TIMEOUT_COUNT   50      /* 5 Sec */
#define SL_STOP_TIMEOUT                 200

typedef enum
{
  LED_OFF = 0,
  LED_ON,
  LED_BLINK
} eLEDStatus;

//*****************************************************************************
//                 GLOBAL VARIABLES -- Start
//*****************************************************************************
static const char pcDigits[] = "0123456789";
static unsigned char POST_token[] = "__SL_P_ULD";
static unsigned char GET_token_TEMP[]  = "__SL_G_UTP";
static unsigned char GET_token_ACC[]  = "__SL_G_UAC";
static unsigned char GET_token_UIC[]  = "__SL_G_UIC";
static int g_iInternetAccess = -1;
static unsigned char g_ucDryerRunning = 0;
static unsigned int g_uiDeviceModeConfig = ROLE_STA; //default is STA mode
static unsigned char g_ucLEDStatus = LED_OFF;
static unsigned long  g_ulStatus = 0;//SimpleLink Status
static unsigned char  g_ucConnectionSSID[SSID_LEN_MAX+1]; //Connection SSID
static unsigned char  g_ucConnectionBSSID[BSSID_LEN_MAX]; //Connection BSSID


#if defined(ccs)
extern void (* const g_pfnVectors[])(void);
#endif
#if defined(ewarm)
extern uVectorEntry __vector_table;
#endif
//*****************************************************************************
//                 GLOBAL VARIABLES -- End
//*****************************************************************************

//*****************************************************************************
//
//! itoa
//!
//!    @brief  Convert integer to ASCII in decimal base
//!
//!     @param  cNum is input integer number to convert
//!     @param  cString is output string
//!
//!     @return number of ASCII parameters
//!
//!
//
//*****************************************************************************
static unsigned short itoa(char cNum, char *cString)
{
    char* ptr;
    char uTemp = cNum;
    unsigned short length;

    // value 0 is a special case
    if (cNum == 0)
    {
        length = 1;
        *cString = '0';

        return length;
    }

    // Find out the length of the number, in decimal base
    length = 0;
    while (uTemp > 0)
    {
        uTemp /= 10;
        length++;
    }

    // Do the actual formatting, right to left
    uTemp = cNum;
    ptr = cString + length;
    while (uTemp > 0)
    {
        --ptr;
        *ptr = pcDigits[uTemp % 10];
        uTemp /= 10;
    }

    return length;
}

//*****************************************************************************
//
//! ReadAccSensor
//!
//!    @brief  Read Accelerometer Data from Sensor
//!
//!
//!     @return none
//!
//!
//
//*****************************************************************************
void ReadAccSensor()
{
    //Define Accelerometer Threshold to Detect Movement
    const short csAccThreshold    = 5;

    signed char cAccXT1,cAccYT1,cAccZT1;
    signed char cAccXT2,cAccYT2,cAccZT2;
    signed short sDelAccX, sDelAccY, sDelAccZ;
    int iRet = -1;
    int iCount = 0;

    iRet = BMA222ReadNew(&cAccXT1, &cAccYT1, &cAccZT1);
    if(iRet)
    {
        //In case of error/ No New Data return
        return;
    }
    for(iCount=0;iCount<2;iCount++)
    {
        MAP_UtilsDelay((90*80*1000)); //30msec
        iRet = BMA222ReadNew(&cAccXT2, &cAccYT2, &cAccZT2);
        if(iRet)
        {
            //In case of error/ No New Data continue
            iRet = 0;
            continue;
        }

        else
        {
            sDelAccX = abs((signed short)cAccXT2 - (signed short)cAccXT1);
            sDelAccY = abs((signed short)cAccYT2 - (signed short)cAccYT1);
            sDelAccZ = abs((signed short)cAccZT2 - (signed short)cAccZT1);

            //Compare with Pre defined Threshold
            if(sDelAccX > csAccThreshold || sDelAccY > csAccThreshold ||
               sDelAccZ > csAccThreshold)
            {
                //Device Movement Detected, Break and Return
                g_ucDryerRunning = 1;
                break;
            }
            else
            {
                //Device Movement Static
                g_ucDryerRunning = 0;
            }
        }
    }

}

//*****************************************************************************
// SimpleLink Asynchronous Event Handlers -- Start
//*****************************************************************************


//*****************************************************************************
//
//! \brief The Function Handles WLAN Events
//!
//! \param[in]  pWlanEvent - Pointer to WLAN Event Info
//!
//! \return None
//!
//*****************************************************************************
void SimpleLinkWlanEventHandler(SlWlanEvent_t *pWlanEvent)
{
    if(pWlanEvent == NULL)
    {
        UART_PRINT("Null pointer\n\r");
        LOOP_FOREVER();
    }
    switch(pWlanEvent->Event)
    {
        case SL_WLAN_CONNECT_EVENT:
        {
            SET_STATUS_BIT(g_ulStatus, STATUS_BIT_CONNECTION);

            //
            // Information about the connected AP (like name, MAC etc) will be
            // available in 'slWlanConnectAsyncResponse_t'
            // Applications can use it if required
            //
            //  slWlanConnectAsyncResponse_t *pEventData = NULL;
            // pEventData = &pWlanEvent->EventData.STAandP2PModeWlanConnected;
            //

            // Copy new connection SSID and BSSID to global parameters
            memcpy(g_ucConnectionSSID,pWlanEvent->EventData.
                   STAandP2PModeWlanConnected.ssid_name,
                   pWlanEvent->EventData.STAandP2PModeWlanConnected.ssid_len);
            memcpy(g_ucConnectionBSSID,
                   pWlanEvent->EventData.STAandP2PModeWlanConnected.bssid,
                   SL_BSSID_LENGTH);

            UART_PRINT("[WLAN EVENT] Device Connected to the AP: %s , "
                       "BSSID: %x:%x:%x:%x:%x:%x\n\r",
                      g_ucConnectionSSID,g_ucConnectionBSSID[0],
                      g_ucConnectionBSSID[1],g_ucConnectionBSSID[2],
                      g_ucConnectionBSSID[3],g_ucConnectionBSSID[4],
                      g_ucConnectionBSSID[5]);
        }
        break;

        case SL_WLAN_DISCONNECT_EVENT:
        {
            slWlanConnectAsyncResponse_t*  pEventData = NULL;

            CLR_STATUS_BIT(g_ulStatus, STATUS_BIT_CONNECTION);
            CLR_STATUS_BIT(g_ulStatus, STATUS_BIT_IP_AQUIRED);

            pEventData = &pWlanEvent->EventData.STAandP2PModeDisconnected;

            // If the user has initiated 'Disconnect' request,
            //'reason_code' is SL_WLAN_DISCONNECT_USER_INITIATED_DISCONNECTION
            if(SL_WLAN_DISCONNECT_USER_INITIATED_DISCONNECTION == pEventData->reason_code)
            {
                UART_PRINT("[WLAN EVENT] Device disconnected from the AP: %s, "
                           "BSSID: %x:%x:%x:%x:%x:%x on application's "
                           "request \n\r",
                           g_ucConnectionSSID,g_ucConnectionBSSID[0],
                           g_ucConnectionBSSID[1],g_ucConnectionBSSID[2],
                           g_ucConnectionBSSID[3],g_ucConnectionBSSID[4],
                           g_ucConnectionBSSID[5]);
            }
            else
            {
                UART_PRINT("[WLAN ERROR] Device disconnected from the AP AP: %s, "
                           "BSSID: %x:%x:%x:%x:%x:%x on an ERROR..!! \n\r",
                           g_ucConnectionSSID,g_ucConnectionBSSID[0],
                           g_ucConnectionBSSID[1],g_ucConnectionBSSID[2],
                           g_ucConnectionBSSID[3],g_ucConnectionBSSID[4],
                           g_ucConnectionBSSID[5]);
            }
            memset(g_ucConnectionSSID,0,sizeof(g_ucConnectionSSID));
            memset(g_ucConnectionBSSID,0,sizeof(g_ucConnectionBSSID));
        }
        break;

        case SL_WLAN_STA_CONNECTED_EVENT:
        {
            // when device is in AP mode and any client connects to device cc3xxx
            //SET_STATUS_BIT(g_ulStatus, STATUS_BIT_CONNECTION);
            //CLR_STATUS_BIT(g_ulStatus, STATUS_BIT_CONNECTION_FAILED);

            //
            // Information about the connected client (like SSID, MAC etc) will
            // be available in 'slPeerInfoAsyncResponse_t' - Applications
            // can use it if required
            //
            // slPeerInfoAsyncResponse_t *pEventData = NULL;
            // pEventData = &pSlWlanEvent->EventData.APModeStaConnected;
            //

            UART_PRINT("[WLAN EVENT] Station connected to device\n\r");
        }
        break;

        case SL_WLAN_STA_DISCONNECTED_EVENT:
        {
            // when client disconnects from device (AP)
            //CLR_STATUS_BIT(g_ulStatus, STATUS_BIT_CONNECTION);
            //CLR_STATUS_BIT(g_ulStatus, STATUS_BIT_IP_LEASED);

            //
            // Information about the connected client (like SSID, MAC etc) will
            // be available in 'slPeerInfoAsyncResponse_t' - Applications
            // can use it if required
            //
            // slPeerInfoAsyncResponse_t *pEventData = NULL;
            // pEventData = &pSlWlanEvent->EventData.APModestaDisconnected;
            //
            UART_PRINT("[WLAN EVENT] Station disconnected from device\n\r");
        }
        break;

        case SL_WLAN_SMART_CONFIG_COMPLETE_EVENT:
        {
            //SET_STATUS_BIT(g_ulStatus, STATUS_BIT_SMARTCONFIG_START);

            //
            // Information about the SmartConfig details (like Status, SSID,
            // Token etc) will be available in 'slSmartConfigStartAsyncResponse_t'
            // - Applications can use it if required
            //
            //  slSmartConfigStartAsyncResponse_t *pEventData = NULL;
            //  pEventData = &pSlWlanEvent->EventData.smartConfigStartResponse;
            //

        }
        break;

        case SL_WLAN_SMART_CONFIG_STOP_EVENT:
        {
            // SmartConfig operation finished
            //CLR_STATUS_BIT(g_ulStatus, STATUS_BIT_SMARTCONFIG_START);

            //
            // Information about the SmartConfig details (like Status, padding
            // etc) will be available in 'slSmartConfigStopAsyncResponse_t' -
            // Applications can use it if required
            //
            // slSmartConfigStopAsyncResponse_t *pEventData = NULL;
            // pEventData = &pSlWlanEvent->EventData.smartConfigStopResponse;
            //
        }
        break;

        default:
        {
            UART_PRINT("[WLAN EVENT] Unexpected event [0x%x]\n\r",
                       pWlanEvent->Event);
        }
        break;
    }
}

//*****************************************************************************
//
//! \brief This function handles network events such as IP acquisition, IP
//!           leased, IP released etc.
//!
//! \param[in]  pNetAppEvent - Pointer to NetApp Event Info
//!
//! \return None
//!
//*****************************************************************************
void SimpleLinkNetAppEventHandler(SlNetAppEvent_t *pNetAppEvent)
{
    if(pNetAppEvent == NULL)
    {
        UART_PRINT("Null pointer\n\r");
        LOOP_FOREVER();
    }

    switch(pNetAppEvent->Event)
    {
        case SL_NETAPP_IPV4_IPACQUIRED_EVENT:
        {
            SlIpV4AcquiredAsync_t *pEventData = NULL;

            SET_STATUS_BIT(g_ulStatus, STATUS_BIT_IP_AQUIRED);

            //Ip Acquired Event Data
            pEventData = &pNetAppEvent->EventData.ipAcquiredV4;

            UART_PRINT("[NETAPP EVENT] IP Acquired: IP=%d.%d.%d.%d , "
                       "Gateway=%d.%d.%d.%d\n\r",
            SL_IPV4_BYTE(pNetAppEvent->EventData.ipAcquiredV4.ip,3),
            SL_IPV4_BYTE(pNetAppEvent->EventData.ipAcquiredV4.ip,2),
            SL_IPV4_BYTE(pNetAppEvent->EventData.ipAcquiredV4.ip,1),
            SL_IPV4_BYTE(pNetAppEvent->EventData.ipAcquiredV4.ip,0),
            SL_IPV4_BYTE(pNetAppEvent->EventData.ipAcquiredV4.gateway,3),
            SL_IPV4_BYTE(pNetAppEvent->EventData.ipAcquiredV4.gateway,2),
            SL_IPV4_BYTE(pNetAppEvent->EventData.ipAcquiredV4.gateway,1),
            SL_IPV4_BYTE(pNetAppEvent->EventData.ipAcquiredV4.gateway,0));

            UNUSED(pEventData);
        }
        break;

        case SL_NETAPP_IP_LEASED_EVENT:
        {
            SET_STATUS_BIT(g_ulStatus, STATUS_BIT_IP_LEASED);

            //
            // Information about the IP-Leased details(like IP-Leased,lease-time,
            // mac etc) will be available in 'SlIpLeasedAsync_t' - Applications
            // can use it if required
            //
            // SlIpLeasedAsync_t *pEventData = NULL;
            // pEventData = &pNetAppEvent->EventData.ipLeased;
            //

        }
        break;

        case SL_NETAPP_IP_RELEASED_EVENT:
        {
            CLR_STATUS_BIT(g_ulStatus, STATUS_BIT_IP_LEASED);

            //
            // Information about the IP-Released details (like IP-address, mac
            // etc) will be available in 'SlIpReleasedAsync_t' - Applications
            // can use it if required
            //
            // SlIpReleasedAsync_t *pEventData = NULL;
            // pEventData = &pNetAppEvent->EventData.ipReleased;
            //
        }
		break;

        default:
        {
            UART_PRINT("[NETAPP EVENT] Unexpected event [0x%x] \n\r",
                       pNetAppEvent->Event);
        }
        break;
    }
}


//*****************************************************************************
//
//! \brief This function handles HTTP server events
//!
//! \param[in]  pServerEvent - Contains the relevant event information
//! \param[in]    pServerResponse - Should be filled by the user with the
//!                                      relevant response information
//!
//! \return None
//!
//****************************************************************************
void SimpleLinkHttpServerCallback(SlHttpServerEvent_t *pSlHttpServerEvent,
                                SlHttpServerResponse_t *pSlHttpServerResponse)
{
    switch (pSlHttpServerEvent->Event)
    {
        case SL_NETAPP_HTTPGETTOKENVALUE_EVENT:
        {
            unsigned char *ptr;

            ptr = pSlHttpServerResponse->ResponseData.token_value.data;
            pSlHttpServerResponse->ResponseData.token_value.len = 0;
            if(memcmp(pSlHttpServerEvent->EventData.httpTokenName.data,
                    GET_token_TEMP, strlen((const char *)GET_token_TEMP)) == 0)
            {
                float fCurrentTemp;
                TMP006DrvGetTemp(&fCurrentTemp);
                char cTemp = (char)fCurrentTemp;
                short sTempLen = itoa(cTemp,(char*)ptr);
                ptr[sTempLen++] = ' ';
                ptr[sTempLen] = 'F';
                pSlHttpServerResponse->ResponseData.token_value.len += sTempLen;

            }

            if(memcmp(pSlHttpServerEvent->EventData.httpTokenName.data,
                      GET_token_UIC, strlen((const char *)GET_token_UIC)) == 0)
            {
                if(g_iInternetAccess==0)
                    strcpy((char*)pSlHttpServerResponse->ResponseData.token_value.data,"1");
                else
                    strcpy((char*)pSlHttpServerResponse->ResponseData.token_value.data,"0");
                pSlHttpServerResponse->ResponseData.token_value.len =  1;
            }

            if(memcmp(pSlHttpServerEvent->EventData.httpTokenName.data,
                       GET_token_ACC, strlen((const char *)GET_token_ACC)) == 0)
            {

                ReadAccSensor();
                if(g_ucDryerRunning)
                {
                    strcpy((char*)pSlHttpServerResponse->ResponseData.token_value.data,"Running");
                    pSlHttpServerResponse->ResponseData.token_value.len += strlen("Running");
                }
                else
                {
                    strcpy((char*)pSlHttpServerResponse->ResponseData.token_value.data,"Stopped");
                    pSlHttpServerResponse->ResponseData.token_value.len += strlen("Stopped");
                }
            }



        }
            break;

        case SL_NETAPP_HTTPPOSTTOKENVALUE_EVENT:
        {
            unsigned char led;
            unsigned char *ptr = pSlHttpServerEvent->EventData.httpPostData.token_name.data;

            //g_ucLEDStatus = 0;
            if(memcmp(ptr, POST_token, strlen((const char *)POST_token)) == 0)
            {
                ptr = pSlHttpServerEvent->EventData.httpPostData.token_value.data;
                if(memcmp(ptr, "LED", 3) != 0)
                    break;
                ptr += 3;
                led = *ptr;
                ptr += 2;
                if(led == '1')
                {
                    if(memcmp(ptr, "ON", 2) == 0)
                    {
                        GPIO_IF_LedOn(MCU_RED_LED_GPIO);
                                                g_ucLEDStatus = LED_ON;

                    }
                    else if(memcmp(ptr, "Blink", 5) == 0)
                    {
                        GPIO_IF_LedOn(MCU_RED_LED_GPIO);
                        g_ucLEDStatus = LED_BLINK;
                    }
                    else
                    {
                        GPIO_IF_LedOff(MCU_RED_LED_GPIO);
                                                g_ucLEDStatus = LED_OFF;
                    }
                }
                else if(led == '2')
                {
                    if(memcmp(ptr, "ON", 2) == 0)
                    {
                        GPIO_IF_LedOn(MCU_ORANGE_LED_GPIO);
                    }
                    else if(memcmp(ptr, "Blink", 5) == 0)
                    {
                        GPIO_IF_LedOn(MCU_ORANGE_LED_GPIO);
                        g_ucLEDStatus = 1;
                    }
                    else
                    {
                        GPIO_IF_LedOff(MCU_ORANGE_LED_GPIO);
                    }
                }

            }
        }
            break;
        default:
            break;
    }
}

//*****************************************************************************
//
//! \brief This function handles General Events
//!
//! \param[in]     pDevEvent - Pointer to General Event Info
//!
//! \return None
//!
//*****************************************************************************
void SimpleLinkGeneralEventHandler(SlDeviceEvent_t *pDevEvent)
{
    if(pDevEvent == NULL)
    {
        UART_PRINT("Null pointer\n\r");
        LOOP_FOREVER();
    }

    //
    // Most of the general errors are not FATAL are are to be handled
    // appropriately by the application
    //
    UART_PRINT("[GENERAL EVENT] - ID=[%d] Sender=[%d]\n\n",
               pDevEvent->EventData.deviceEvent.status,
               pDevEvent->EventData.deviceEvent.sender);
}


//*****************************************************************************
//
//! This function handles socket events indication
//!
//! \param[in]      pSock - Pointer to Socket Event Info
//!
//! \return None
//!
//*****************************************************************************
void SimpleLinkSockEventHandler(SlSockEvent_t *pSock)
{
    if(pSock == NULL)
    {
        UART_PRINT("Null pointer\n\r");
        LOOP_FOREVER();
    }
    //
    // This application doesn't work w/ socket - Events are not expected
    //
    switch( pSock->Event )
    {
        case SL_SOCKET_TX_FAILED_EVENT:
            switch( pSock->socketAsyncEvent.SockTxFailData.status)
            {
                case SL_ECLOSE:
                    UART_PRINT("[SOCK ERROR] - close socket (%d) operation "
                                "failed to transmit all queued packets\n\n",
                                    pSock->socketAsyncEvent.SockTxFailData.sd);
                    break;
                default:
                    UART_PRINT("[SOCK ERROR] - TX FAILED  :  socket %d , reason "
                                "(%d) \n\n",
                                pSock->socketAsyncEvent.SockTxFailData.sd, pSock->socketAsyncEvent.SockTxFailData.status);
                  break;
            }
            break;

        default:
        	UART_PRINT("[SOCK EVENT] - Unexpected Event [%x0x]\n\n",pSock->Event);
          break;
    }
}


//*****************************************************************************
// SimpleLink Asynchronous Event Handlers -- End
//*****************************************************************************

//*****************************************************************************
//
//! \brief This function initializes the application variables
//!
//! \param  None
//!
//! \return None
//!
//*****************************************************************************
static void InitializeAppVariables()
{
    g_ulStatus = 0;
    memset(g_ucConnectionSSID,0,sizeof(g_ucConnectionSSID));
    memset(g_ucConnectionBSSID,0,sizeof(g_ucConnectionBSSID));
    g_iInternetAccess = -1;
    g_ucDryerRunning = 0;
    g_uiDeviceModeConfig = ROLE_STA; //default is STA mode
    g_ucLEDStatus = LED_OFF;
}


//****************************************************************************
//
//! Confgiures the mode in which the device will work
//!
//! \param iMode is the current mode of the device
//!
//!
//! \return   SlWlanMode_t
//!                        
//
//****************************************************************************
static int ConfigureMode(int iMode)
{
    long   lRetVal = -1;

    lRetVal = sl_WlanSetMode(iMode);
    ASSERT_ON_ERROR(lRetVal);

    // Restart Network processor
    lRetVal = sl_Stop(SL_STOP_TIMEOUT);

    // Reset status bits
    CLR_STATUS_BIT_ALL(g_ulStatus);

    return sl_Start(NULL,NULL,NULL);
}


//****************************************************************************
//
//!    \brief Sets up the device in AP or STA Mode, depending on the presence
//!           and connectivity of stored WLAN profiles and the AP jumper pin.
//!
//! On startup, if the AP jumper pin is present or if no Wireless LAN (WLAN) profiles are present,
//! the station is put into Access Point (AP) mode, allowing the user to set WLAN profiles for
//! the device.
//!
//! Otherwise one or more WLAN profiles are present on the device. In this later case, the
//! station is put into station (STA) mode and connects to the highest signal strength
//! WLAN profile present, falling back to the other WLAN profiles if LAN connectivity to the
//! higher priority profile(s) is not possible. If LAN connectivity cannot be established for
//! any of the saved profiles, then the device is put into AP mode.
//!
//! Once successfully connected to an access point in station mode, if at any point LAN
//! connectivity to the access point is lost, the device trys to connect to the other present
//! WLAN profiles based on their priority.
//!
//! \return  0 - Success
//!         -1 - Failure
//
//****************************************************************************
long ConnectToNetwork()
{
    long lRetVal = -1;
    unsigned int uiConnectTimeoutCnt = 0;

    // Start Simplelink
    lRetVal =  sl_Start(NULL,NULL,NULL);
    ASSERT_ON_ERROR( lRetVal);

    if ( g_uiDeviceModeConfig == ROLE_AP )
    {
    	UART_PRINT("Force AP Jumper is Connected.\n\r");

    	if ( lRetVal != ROLE_AP )
    	{
    		// Put the device into AP mode.
    		lRetVal = ConfigureMode(ROLE_AP);
    		ASSERT_ON_ERROR(lRetVal);
    	}

		// Now the device is in AP mode, we need to wait for this event
		// before doing anything
		while(!IS_IP_ACQUIRED(g_ulStatus))
		{
		#ifndef SL_PLATFORM_MULTI_THREADED
		  _SlNonOsMainLoopTask();
		#endif
		}

    	// Stop Internal HTTP Server
		lRetVal = sl_NetAppStop(SL_NET_APP_HTTP_SERVER_ID);
		ASSERT_ON_ERROR( lRetVal);

		// Start Internal HTTP Server
		lRetVal = sl_NetAppStart(SL_NET_APP_HTTP_SERVER_ID);
		ASSERT_ON_ERROR( lRetVal);

	    char ssid[32];
	    unsigned short len = 32;
	    unsigned short config_opt = WLAN_AP_OPT_SSID;
	    sl_WlanGet(SL_WLAN_CFG_AP_ID, &config_opt , &len, (unsigned char* )ssid);
	    UART_PRINT("\n\r Connect to : \'%s\'\n\r\n\r",ssid);
    }
    else
    {
    	if ( lRetVal == ROLE_AP )
    	{
    		UART_PRINT("Device is in AP Mode and Force AP Jumper is not Connected.\n\r");

            // If the device is in AP mode, we need to wait for this event
            // before doing anything
            while(!IS_IP_ACQUIRED(g_ulStatus))
            {
            #ifndef SL_PLATFORM_MULTI_THREADED
              _SlNonOsMainLoopTask();
            #endif
            }
    	}

        // Switch to STA Mode
        lRetVal = ConfigureMode(ROLE_STA);
        ASSERT_ON_ERROR( lRetVal);

        UART_PRINT("Device has been put into STA Mode.\n\r");

        // Stop Internal HTTP Server
		lRetVal = sl_NetAppStop(SL_NET_APP_HTTP_SERVER_ID);
		ASSERT_ON_ERROR( lRetVal);

		// Start Internal HTTP Server
		lRetVal = sl_NetAppStart(SL_NET_APP_HTTP_SERVER_ID);
		ASSERT_ON_ERROR( lRetVal);

		// Waiting for the device to Auto Connect
		UART_PRINT("Trying to Auto Connect to Existing Profiles.\n\r");

		while(uiConnectTimeoutCnt < AUTO_CONNECTION_TIMEOUT_COUNT &&
			((!IS_CONNECTED(g_ulStatus)) || (!IS_IP_ACQUIRED(g_ulStatus))))
		{
			// Turn RED LED On
			GPIO_IF_LedOn(MCU_RED_LED_GPIO);
			osi_Sleep(50);

			// Turn RED LED Off
			GPIO_IF_LedOff(MCU_RED_LED_GPIO);
			osi_Sleep(50);

			uiConnectTimeoutCnt++;
		}

		// Couldn't connect Using Auto Profile
		if(uiConnectTimeoutCnt == AUTO_CONNECTION_TIMEOUT_COUNT)
		{
			UART_PRINT("Couldn't connect Using Auto Profile.\n\r");

			// Blink Red LED to Indicate Connection Error
			GPIO_IF_LedOn(MCU_RED_LED_GPIO);

			CLR_STATUS_BIT_ALL(g_ulStatus);

			// Put the station into AP mode
			lRetVal = ConfigureMode(ROLE_AP);
			ASSERT_ON_ERROR( lRetVal);
			UART_PRINT("Device has been put into AP Mode.\n\r");


			// Waiting for the AP to acquire IP address from Internal DHCP Server
			// If the device is in AP mode, we need to wait for this event
			// before doing anything
			while(!IS_IP_ACQUIRED(g_ulStatus))
			{
			#ifndef SL_PLATFORM_MULTI_THREADED
				_SlNonOsMainLoopTask();
			#endif
			}

			char cCount=0;

			// Blink LED 3 times to Indicate AP Mode
			for(cCount=0; cCount<3; cCount++)
			{
				// Turn RED LED On
				GPIO_IF_LedOn(MCU_RED_LED_GPIO);
				osi_Sleep(400);

				// Turn RED LED Off
				GPIO_IF_LedOff(MCU_RED_LED_GPIO);
				osi_Sleep(400);
			}

		    char ssid[32];
		    unsigned short len = 32;
		    unsigned short config_opt = WLAN_AP_OPT_SSID;
		    sl_WlanGet(SL_WLAN_CFG_AP_ID, &config_opt , &len, (unsigned char* )ssid);
		    UART_PRINT("\n\r Connect to : \'%s\'\n\r\n\r",ssid);
		}

		// Turn RED LED Off
		GPIO_IF_LedOff(MCU_RED_LED_GPIO);

		g_iInternetAccess = ConnectionTest();

    }

    return SUCCESS;
}


//****************************************************************************
//
//!    \brief Read Force AP GPIO and Configure Mode - 1(Access Point Mode)
//!                                                  - 0 (Station Mode)
//!
//! \return                        None
//
//****************************************************************************
static void ReadDeviceConfiguration()
{
    unsigned int uiGPIOPort;
    unsigned char pucGPIOPin;
    unsigned char ucPinValue;
        
    // Read GPIO
    GPIO_IF_GetPortNPin(SH_GPIO_3,&uiGPIOPort,&pucGPIOPin);
    ucPinValue = GPIO_IF_Get(SH_GPIO_3,uiGPIOPort,pucGPIOPin);
        
    // If Connected to VCC, Mode is AP
    if(ucPinValue == 1)
    {
        // AP Mode
        g_uiDeviceModeConfig = ROLE_AP;
    }
    else
    {
        // STA Mode
        g_uiDeviceModeConfig = ROLE_STA;
    }

}

//****************************************************************************
//
//!    \brief OOB Application Main Task - Initializes SimpleLink Driver and
//!                                              Handles HTTP Requests
//! \param[in]                  pvParameters is the data passed to the Task
//!
//! \return                        None
//
//****************************************************************************
static void OOBTask(void *pvParameters)
{
    long   lRetVal = -1;

    // Read Device Mode Configuration
    ReadDeviceConfiguration();

    // Connect to Network
    lRetVal = ConnectToNetwork();
    if(lRetVal < 0)
    {
        ERR_PRINT(lRetVal);
        LOOP_FOREVER();
    }

    // Handle Async Events
    while(1)
    {
        // LED Actions
        if(g_ucLEDStatus == LED_ON)
        {
            GPIO_IF_LedOn(MCU_RED_LED_GPIO);
            osi_Sleep(500);
        }
        if(g_ucLEDStatus == LED_OFF)
        {
            GPIO_IF_LedOff(MCU_RED_LED_GPIO);
            osi_Sleep(500);
        }
        if(g_ucLEDStatus==LED_BLINK)
        {
            GPIO_IF_LedOn(MCU_RED_LED_GPIO);
            osi_Sleep(500);
            GPIO_IF_LedOff(MCU_RED_LED_GPIO);
            osi_Sleep(500);
        }
    }
}

//****************************************************************************
//
//!    \brief DataGather Application Task - Samples the sensors every 1 minute.
//! \param[in]                  pvParameters is the data passed to the Task
//!
//! \return                        None
//
//****************************************************************************
static void DataGatherTask(void *pvParameters)
{
	TickType_t xLastWakeTime;
	const TickType_t xFreq = 30000; // 30 seconds

	xLastWakeTime = xTaskGetTickCount();

	int i = 0;
	while (1)
	{
		vTaskDelayUntil(&xLastWakeTime, xFreq);

		// Write over UART
		UART_PRINT("%d\r\n", i++);
	}
}

//****************************************************************************
//
//!    \brief DataUpload Application Task - Uploads collected data every 90 minutes.
//! \param[in]                  pvParameters is the data passed to the Task
//!
//! \return                        None
//
//****************************************************************************
//static void DataUploadTask(void *pvParameters)
//{
//	TickType_t xLastWakeTime;
//	const TickType_t xFreq = 60000; // 1 minute
//
//	xLastWakeTime = xTaskGetTickCount();
//
//	int i = 0;
//	while (1)
//	{
//		vTaskDelayUntil(&xLastWakeTime, xFreq);
//
//		// Write over UART
//		UART_PRINT("%d\r\n", i++);
//	}
//}

//*****************************************************************************
//
//! Application startup display on UART
//!
//! \param  none
//!
//! \return none
//!
//*****************************************************************************
static void DisplayBanner(char * AppName)
{
    UART_PRINT("\n\n\n\r");
    UART_PRINT("\t\t *************************************************\n\r");
    UART_PRINT("\t\t     CC3200 %s Application       \n\r", AppName);
    UART_PRINT("\t\t *************************************************\n\r");
    UART_PRINT("\n\n\n\r");
}

//*****************************************************************************
//
//! Board Initialization & Configuration
//!
//! \param  None
//!
//! \return None
//
//*****************************************************************************
static void BoardInit(void)
{

    // Set vector table base
#if defined(ccs) || defined(gcc)
    MAP_IntVTableBaseSet((unsigned long)&g_pfnVectors[0]);
#endif
#if defined(ewarm)
    MAP_IntVTableBaseSet((unsigned long)&__vector_table);
#endif
    
    // Enable Processor
    MAP_IntMasterEnable();
    MAP_IntEnable(FAULT_SYSTICK);

    PRCMCC3200MCUInit();
}

void main()
{
    // Board Initialization
    BoardInit();
    
    // Configure the pinmux settings for the peripherals exercised
    PinMuxConfig();

    PinConfigSet(PIN_58,PIN_STRENGTH_2MA|PIN_STRENGTH_4MA,PIN_TYPE_STD_PD);

    // Initialize Global Variables
    InitializeAppVariables();
    
    // UART Init
    InitTerm();
    
    DisplayBanner(APP_NAME);

    // LED Init
    GPIO_IF_LedConfigure(LED1);
      
    // Turn Off the LEDs
    GPIO_IF_LedOff(MCU_RED_LED_GPIO);

    // Simplelink Spawn Task
    VStartSimpleLinkSpawnTask(SPAWN_TASK_PRIORITY);
    
    // Create OOB Task
    osi_TaskCreate(OOBTask, (signed char*)"OOBTask", \
                                OSI_STACK_SIZE, NULL, \
                                OOB_TASK_PRIORITY, NULL );

    // Create the DataGather Task
    osi_TaskCreate(DataGatherTask, (signed char*)"DataGatherTask",
    						 1024, NULL, DATAGATHER_TASK_PRIORITY, NULL);

//    // Create the DataUpload Task
//    osi_TaskCreate(DataUploadTask, (signed char*)"DataUploadTask",
//        						 OSI_STACK_SIZE, NULL, DATAUPLOAD_TASK_PRIORITY, NULL);

    // Start OS Scheduler
    osi_start();

    while (1)
    {

    }

}
