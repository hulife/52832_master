#include "string.h"
#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include "nordic_common.h"
#include "app_error.h"
#include "app_uart.h"
#include "ble_db_discovery.h"
#include "app_timer.h"
#include "app_util.h"
#include "bsp_btn_ble.h"
#include "ble.h"
#include "ble_gap.h"
#include "ble_hci.h"
#include "nrf_sdh.h"
#include "nrf_sdh_ble.h"
#include "nrf_sdh_soc.h"
#include "ble_advdata.h"
#include "ble_nus_c.h"
#include "nrf_ble_gatt.h"
#include "nrf_pwr_mgmt.h"

#include "nrf_delay.h"
#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"

#include "nrf_ble_scan.h"
#include "nrf_drv_wdt.h"
#include "gy_common.h"
#include "sen_pack.h"
#include "nrf_queue.h"
#include "ble_data.h"
#define HARDWARE_NUMBER			"HW_4.3"
#define SOFTWARE_NUMBER			"SW_6.0.0"
#define FIRMWARE_NUMBER			"FW_15.2.0"

#define USER_FILTER_MAX_INDEX	20
#define USER_FILTER_LEN			3
uint16_t                    conn_handle;
//user filter
typedef struct
{
	uint8_t cmd_enable;
	uint8_t filter_keywords[USER_FILTER_LEN];
	uint8_t filter_length;
} user_filter_t;

user_filter_t user[USER_FILTER_MAX_INDEX];

int FLAG;

static uint8_t filter_enable = 0;	// user device name filter enable


#define APP_BLE_CONN_CFG_TAG    1                                       /**< A tag that refers to the BLE stack configuration we set with @ref sd_ble_cfg_set. Default tag is @ref BLE_CONN_CFG_TAG_DEFAULT. */
#define APP_BLE_OBSERVER_PRIO   3                                       /**< Application's BLE observer priority. You shoulnd't need to modify this value. */

#define UART_TX_BUF_SIZE        256                                     /**< UART TX buffer size. */
#define UART_RX_BUF_SIZE        256                                     /**< UART RX buffer size. */

#define NUS_SERVICE_UUID_TYPE   BLE_UUID_TYPE_VENDOR_BEGIN              /**< UUID type for the Nordic UART Service (vendor specific). */

#define SCAN_INTERVAL           0x00A0                                  /**< Determines scan interval in units of 0.625 millisecond. */
#define SCAN_WINDOW             0x0050                                  /**< Determines scan window in units of 0.625 millisecond. */
#define SCAN_DURATION           0x0000                                  /**< Timout when scanning. 0x0000 disables timeout. */

#define MIN_CONNECTION_INTERVAL MSEC_TO_UNITS(20, UNIT_1_25_MS)         /**< Determines minimum connection interval in millisecond. */
#define MAX_CONNECTION_INTERVAL MSEC_TO_UNITS(75, UNIT_1_25_MS)         /**< Determines maximum connection interval in millisecond. */
#define SLAVE_LATENCY           0                                       /**< Determines slave latency in counts of connection events. */
#define SUPERVISION_TIMEOUT     MSEC_TO_UNITS(4000, UNIT_10_MS)         /**< Determines supervision time-out in units of 10 millisecond. */

#define ECHOBACK_BLE_UART_DATA  0//1                                       /**< Echo the UART data that is received over the Nordic UART Service back to the sender. */


NRF_BLE_GATT_DEF(m_gatt);                                               /**< GATT module instance. */
BLE_DB_DISCOVERY_DEF(m_db_disc);                                        /**< DB discovery module instance. */
NRF_BLE_SCAN_DEF(m_scan);           // 定义扫描实例的名称


static uint16_t m_ble_nus_max_data_len = BLE_GATT_ATT_MTU_DEFAULT - OPCODE_LENGTH - HANDLE_LENGTH; /**< Maximum length of data (in bytes) that can be transmitted to the peer by the Nordic UART service module. */

/**@brief Connection parameters requested for connection. */
static ble_gap_conn_params_t const m_connection_param =
{
    (uint16_t)MIN_CONNECTION_INTERVAL,  // Minimum connection
    (uint16_t)MAX_CONNECTION_INTERVAL,  // Maximum connection
    (uint16_t)SLAVE_LATENCY,            // Slave latency
    (uint16_t)SUPERVISION_TIMEOUT       // Supervision time-out
};

//#define APP_QUEUE		
#define APP_DATA	
void ble_data_send_with_queue(void);
typedef struct {
    uint8_t * p_data;
    uint16_t length;
} buffer_t;

NRF_QUEUE_DEF(buffer_t, m_buf_queue, 30, NRF_QUEUE_MODE_NO_OVERFLOW);

APP_TIMER_DEF(m_timer_speed);
uint8_t m_data_array[6300];
uint32_t m_len_sent;
uint32_t m_cnt_7ms;

static uint8_t data_array[BLE_NUS_MAX_DATA_LEN];




static uint8_t m_scan_buffer_data[BLE_GAP_SCAN_BUFFER_MIN]; /**< buffer where advertising reports will be stored by the SoftDevice. */
static uint8_t m_buffer_data[BLE_GAP_ADDR_LEN]; /**< buffer where advertising reports will be stored by the SoftDevice. */

/**@brief Pointer to the buffer where advertising reports will be stored by the SoftDevice. */
static ble_data_t m_scan_buffer =
{
    m_scan_buffer_data,
    BLE_GAP_SCAN_BUFFER_MIN
};

/** @brief Parameters used when scanning. */
static ble_gap_scan_params_t const m_scan_params =
{
    .active   = 1,
    .interval = SCAN_INTERVAL,
    .window   = SCAN_WINDOW,
    .timeout          = SCAN_DURATION,
    .scan_phys        = BLE_GAP_PHY_1MBPS,
    .filter_policy    = BLE_GAP_SCAN_FP_ACCEPT_ALL,
};

/**@brief NUS uuid. */
static ble_uuid_t const m_nus_uuid =
{
    .uuid = BLE_UUID_NUS_SERVICE,
    .type = BLE_UUID_TYPE_BLE//NUS_SERVICE_UUID_TYPE
};
uint8_t myaddr[BLE_GAP_ADDR_LEN];//={0xac,0xde,0x35,0x33,0xa9,0xe7}; 
//uint8_t myaddr[BLE_GAP_ADDR_LEN]={0xe7,0xa9,0x33,0x35,0xde,0xac}; 
uint8_t *p_addr=myaddr;
//@brief NUS addr. 
#define fir  0xac
#define sec  0xde
#define thr  0x35
#define fou  0x33
#define fiv  0xa9
#define six  0xe7
char*m_nus_name="TEST_52832";

	static ble_gap_addr_t  m_nus_addr =
{
  .addr_id_peer=1,
	.addr_type=BLE_GAP_ADDR_TYPE_PUBLIC,
//	.addr=p_addr
	.addr[0]=fir,
	.addr[1]=sec,
	.addr[2]=thr,
	.addr[3]=fou,
	.addr[4]=fiv,
	.addr[5]=six
};

/**@brief Function for asserts in the SoftDevice.
 *
 * @details This function will be called in case of an assert in the SoftDevice.
 *
 * @warning This handler is an example only and does not fit a final product. You need to analyze
 *          how your product is supposed to react in case of Assert.
 * @warning On assert from the SoftDevice, the system can only recover on reset.
 *
 * @param[in] line_num     Line number of the failing ASSERT call.
 * @param[in] p_file_name  File name of the failing ASSERT call.
 */
void assert_nrf_callback(uint16_t line_num, const uint8_t * p_file_name)
{
    app_error_handler(0xDEADBEEF, line_num, p_file_name);
	
}


//@brief Function to start scanning. 
static void scan_start(void)
{
    ret_code_t ret;

    ret = sd_ble_gap_scan_start(&m_scan_params, &m_scan_buffer);
    APP_ERROR_CHECK(ret);

    ret = bsp_indication_set(BSP_INDICATE_SCANNING);
    APP_ERROR_CHECK(ret);
}


//******************************************************************
// fn : scan_start
//
// brief : 开始扫描
//
// param : none
//
// return : none

//static void scan_start(void)
//{
//    ret_code_t ret;

//    ret = nrf_ble_scan_start(&m_scan);
//    APP_ERROR_CHECK(ret);
//	 ret = bsp_indication_set(BSP_INDICATE_SCANNING);
//    APP_ERROR_CHECK(ret);
//}

void scan_stop(void)
{
    nrf_ble_scan_stop();
}


//******************************************************************
// fn : scan_evt_handler
//
// brief : 处理扫描回调事件
//
// param : scan_evt_t  扫描事件结构体
//
// return : none
static void scan_evt_handler(scan_evt_t const * p_scan_evt)
{
    switch(p_scan_evt->scan_evt_id)
    {
			

			  // 未过滤的扫描数据
        case NRF_BLE_SCAN_EVT_NOT_FOUND:
        {
					
						if(FLAG==3)    
	{
	      // 判断是否为扫描回调数据
          if(p_scan_evt->params.p_not_found->type.scan_response)
          {
            if(p_scan_evt->params.p_not_found->data.len)    // 存在扫描回调数据
            {
              NRF_LOG_INFO("scan data:  %s",
                            Util_convertHex2Str(
                            p_scan_evt->params.p_not_found->data.p_data,
                            p_scan_evt->params.p_not_found->data.len));
            }
            else
            {
              NRF_LOG_INFO("scan data:  %s","NONE");
            }
            NRF_LOG_INFO("rssi:  %ddBm",p_scan_evt->params.p_not_found->rssi);
          }
          else  // 否则为广播数据
          {
            // 打印扫描的设备MAC
            NRF_LOG_INFO("Device  ADV MAC:  %s",
                         Util_convertBdAddr2Str((uint8_t*)p_scan_evt->params.p_not_found->peer_addr.addr));
            
            if(p_scan_evt->params.p_not_found->data.len)    // 存在广播数据
            {
              NRF_LOG_INFO("adv ADV data:  %s",
                            Util_convertHex2Str(
                            p_scan_evt->params.p_not_found->data.p_data,
                            p_scan_evt->params.p_not_found->data.len));
            }
            else
            {
              NRF_LOG_INFO("adv data:  %s","NONE");
            }
          
          

          // 如果扫描到的设备信号强度大于-80dBm
          if(p_scan_evt->params.p_not_found->rssi > (-80))//
          {
            ret_code_t          err_code;
            
            // 配置准备连接的设备MAC
            ble_gap_addr_t m_addr;
            m_addr.addr_id_peer = 1;
            m_addr.addr_type = BLE_GAP_ADDR_TYPE_PUBLIC;
           // memcpy(m_addr.addr,p_scan_evt->params.p_not_found->peer_addr.addr,BLE_GAP_ADDR_LEN);
            char *addr_ch=Util_convertBdAddr2Str(p_addr);
						memcpy(m_addr.addr,p_addr,BLE_GAP_ADDR_LEN);
            // 停止扫描
            nrf_ble_scan_stop();
            // 发起连接
            err_code = sd_ble_gap_connect(&m_addr,&m_scan_params,&m_connection_param,APP_BLE_CONN_CFG_TAG);
            APP_ERROR_CHECK(err_code);
          }
          
        } 
					break;	
		}				
			}	

				
//       // 匹配的扫描数据（也就是过滤之后的）
//        case NRF_BLE_SCAN_EVT_FILTER_MATCH:
//        {
//          // 下面这一段我们只保留了扫描回调数据获取的部分，因为从机筛选广播的UUID在扫描回调数据
//          // 判断是否为扫描回调数据
//          if(p_scan_evt->params.p_not_found->type.scan_response)
//          {
//            NRF_LOG_INFO("Device MAC:  %s",
//                         Util_convertBdAddr2Str((uint8_t*)p_scan_evt->params.p_not_found->peer_addr.addr));
//            
//            if(p_scan_evt->params.p_not_found->data.len)    // 存在扫描回调数据
//            {
//              NRF_LOG_INFO("scan data:  %s",
//                            Util_convertHex2Str(
//                            p_scan_evt->params.p_not_found->data.p_data,
//                            p_scan_evt->params.p_not_found->data.len));
//            }
//            else
//            {
//              NRF_LOG_INFO("scan data:  %s","NONE");
//            }
//            NRF_LOG_INFO("rssi:  %ddBm",p_scan_evt->params.p_not_found->rssi);
//          }
//          else  // 否则为广播数据
//          {
//            // 打印扫描的设备MAC
//            NRF_LOG_INFO("Device MAC adv:  %s",
//                         Util_convertBdAddr2Str((uint8_t*)p_scan_evt->params.p_not_found->peer_addr.addr));
//            
//            if(p_scan_evt->params.p_not_found->data.len)    // 存在广播数据
//            {
//              NRF_LOG_INFO("adv data:  %s",
//                            Util_convertHex2Str(
//                            p_scan_evt->params.p_not_found->data.p_data,
//                            p_scan_evt->params.p_not_found->data.len));
//            }
//            else
//            {
//              NRF_LOG_INFO("adv data:  %s","NONE");
//            }
//          }
//        } break;

        case NRF_BLE_SCAN_EVT_CONNECTED:
        {
//            NRF_LOG_INFO("SCAN CONNECTED!"); 
//            NRF_LOG_INFO("Connected. conn_DevAddr: %s\nConnected. conn_handle: 0x%04x\nConnected. conn_Param: %d,%d,%d,%d",
//                         Util_convertBdAddr2Str((uint8_t*)p_scan_evt->params.connected.p_connected->peer_addr.addr),
//                         p_scan_evt->params.connected.conn_handle,
//                         p_scan_evt->params.connected.p_connected->conn_params.min_conn_interval,
//                         p_scan_evt->params.connected.p_connected->conn_params.max_conn_interval,
//                         p_scan_evt->params.connected.p_connected->conn_params.slave_latency,
//                         p_scan_evt->params.connected.p_connected->conn_params.conn_sup_timeout
//                         );
        }break;
			 
        case NRF_BLE_SCAN_EVT_CONNECTING_ERROR:
        {
            NRF_LOG_INFO("SCAN CONNECTING ERROR!");
            NRF_LOG_INFO("Disconnected. reason: 0x%04x",
                         p_scan_evt->params.connecting_err.err_code);
        }break;
        
        default:
           break;
    }
}

//******************************************************************
// fn : scan_init
//
// brief : 初始化扫描（未设置扫描数据限制）
//
// param : none
//
// return : none
static void scan_init(void)
{
    ret_code_t          err_code;
    nrf_ble_scan_init_t init_scan;
		int i=0;
    // 清空扫描结构体参数
    memset(&init_scan, 0, sizeof(init_scan));
    
    init_scan.connect_if_match = 1;
    init_scan.conn_cfg_tag = APP_BLE_CONN_CFG_TAG;
    
    // 配置扫描的参数
    init_scan.p_scan_param = &m_scan_params;

    // 配置连接的参数
    init_scan.p_conn_param = &m_connection_param;
    
    // 初始化扫描
    err_code = nrf_ble_scan_init(&m_scan, &init_scan, scan_evt_handler);
    APP_ERROR_CHECK(err_code);
   

//	char *addr_ch=Util_convertBdAddr2Str(p_addr);	
//	printf("change====%s\r\n",addr_ch); 

//		memcpy(m_nus_addr.addr,p_addr,BLE_GAP_ADDR_LEN);
     
//	  // 设置扫描的addr限制
//    err_code = nrf_ble_scan_filter_set(&m_scan, SCAN_ADDR_FILTER, &m_nus_addr);
//    APP_ERROR_CHECK(err_code);

//    // 使能扫描的addr限制
//    err_code = nrf_ble_scan_filters_enable(&m_scan, NRF_BLE_SCAN_ADDR_FILTER, false);
//    APP_ERROR_CHECK(err_code);

	
//		//设置扫描的name限制      <TEST OK>
//    err_code = nrf_ble_scan_filter_set(&m_scan, SCAN_NAME_FILTER, m_nus_name);
//    APP_ERROR_CHECK(err_code);

//    // 使能扫描的name限制
//    err_code = nrf_ble_scan_filters_enable(&m_scan, NRF_BLE_SCAN_NAME_FILTER, false);
//    APP_ERROR_CHECK(err_code);

//    // 设置扫描的UUID限制     <TEST OK>
//    err_code = nrf_ble_scan_filter_set(&m_scan, SCAN_UUID_FILTER, &m_nus_uuid);
//    APP_ERROR_CHECK(err_code);

//    // 使能扫描的UUID限制
//    err_code = nrf_ble_scan_filters_enable(&m_scan, NRF_BLE_SCAN_UUID_FILTER, false);
//    APP_ERROR_CHECK(err_code);
		
		
		
}

/**@brief Function for handling database discovery events.
 *
 * @details This function is callback function to handle events from the database discovery module.
 *          Depending on the UUIDs that are discovered, this function should forward the events
 *          to their respective services.
 *
 * @param[in] p_event  Pointer to the database discovery event.
 */
static void db_disc_handler(ble_db_discovery_evt_t * p_evt)
{
    ble_nus_c_on_db_disc_evt(&m_ble_nus_c, p_evt);
}











/**@brief Function for handling characters received by the Nordic UART Service.
 *
 * @details This function takes a list of characters of length data_len and prints the characters out on UART.
 *          If @ref ECHOBACK_BLE_UART_DATA is set, the data is sent back to sender.
 */
static void ble_nus_chars_received_uart_print(uint8_t * p_data, uint16_t data_len)
{
    ret_code_t ret_val;

    NRF_LOG_DEBUG("Receiving data.");
    NRF_LOG_HEXDUMP_DEBUG(p_data, data_len);

    for (uint32_t i = 0; i < data_len; i++)
    {
        do
        {
						printf("+");
            ret_val = app_uart_put(p_data[i]);
            if ((ret_val != NRF_SUCCESS) && (ret_val != NRF_ERROR_BUSY))
            {
                NRF_LOG_ERROR("app_uart_put failed for index 0x%04x.", i);
                APP_ERROR_CHECK(ret_val);
            }
        } while (ret_val == NRF_ERROR_BUSY);
    }
    if (p_data[data_len-1] == '\r')
    {
        while (app_uart_put('\n') == NRF_ERROR_BUSY);
    }
    if (ECHOBACK_BLE_UART_DATA)
    {
        // Send data back to peripheral.
        do
        {
            ret_val = ble_nus_c_string_send(&m_ble_nus_c, p_data, data_len);
            if ((ret_val != NRF_SUCCESS) && (ret_val != NRF_ERROR_BUSY))
            {
                NRF_LOG_ERROR("Failed sending NUS message. Error 0x%x. ", ret_val);
                APP_ERROR_CHECK(ret_val);
            }
        } while (ret_val == NRF_ERROR_BUSY);
    }
}

/**@brief Function for process AT command.
 *
 * @param[in] *pBuffer  pointer of string.
 * @param[in] length	Length of the data.
 */
#define MAX_DATA_SIZE 247
static void AT_cmd_handle(uint8_t *pBuffer, uint16_t length)
{
	ret_code_t err_code;

	// check whether is AT cmd or not	
	if(length < 2) 
		return;
	if(strncmp((char*)pBuffer, "AT", 2) != 0)
		return;
	
	// AT test: AT?\r\n
	if((length == 5) && (strncmp((char*)pBuffer, "AT?\r\n", 5) == 0))
	{
		printf("AT:OK\r\n");
	}
	
	// System soft reset: AT+RESET\r\n
	else if((length == 10) && (strncmp((char*)pBuffer, "AT+RESET\r\n", 10) == 0))
	{
		NVIC_SystemReset();	// Restart the system by default	
	}
	
	// Hardware/firmware/software version check: AT+VER?\r\n
	else if((length == 9) && (strncmp((char*)pBuffer, "AT+VER?\r\n", 9) == 0))
	{
		printf("AT+VER:%s,%s,%s\r\n", HARDWARE_NUMBER, FIRMWARE_NUMBER, SOFTWARE_NUMBER);	
	}	

	// MAC address check: AT+MAC?\r\n
	else if((length == 9) && (strncmp((char*)pBuffer, "AT+MAC?\r\n", 9) == 0))//check MAC addr
	{
		ble_gap_addr_t device_addr;
	
		// Get BLE address.
		#if (NRF_SD_BLE_API_VERSION >= 3)
			err_code = sd_ble_gap_addr_get(&device_addr);
		#else
			err_code = sd_ble_gap_address_get(&device_addr);
		#endif
		APP_ERROR_CHECK(err_code);

		printf("AT+MAC:%s\r\n", Util_convertBdAddr2Str(device_addr.addr));
	}	
	// disconnect: AT+DISCONN=\r\n
	else if((length >= 8) && (strncmp((char*)pBuffer, "AT+DISCONN", 10) == 0))//CONNECT
	{
		FLAG=0;
		printf("disconnect\r\n");
		
		sd_ble_gap_disconnect(conn_handle, BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
	}
		// SCAN: AT+DISSCAN=\r\n
	else if((length >= 8) && (strncmp((char*)pBuffer, "AT+DISSCAN", 10) == 0))//SCAN
	{
	
		 printf("disscan\r\n");
		 scan_stop();
			FLAG=1;
	}
		// connect: AT+CONN=\r\n
	else if((length >= 8) && (strncmp((char*)pBuffer, "AT+CONN=", 8) == 0))//CONNECT
	{
		if(FLAG==1)
		{
			sd_ble_gap_scan_start(NULL, &m_scan_buffer);
		}
		
		int i;
		printf("connect\r\n");
		uint8_t data_CON[247];
		uint8_t data_CON_NEW[247];
		uint8_t     *CON=data_CON;
		uint8_t     *CON_NEW=data_CON_NEW;
		memcpy( CON,pBuffer,length-2 );
		CON+=8;	
		printf("%s\r\n",CON);
		
		memcpy( data_CON_NEW,CON,length-10 );
		unsigned char out_data[MAX_DATA_SIZE] = {0};
    int out_data_len = MAX_DATA_SIZE;
		
		convert_string2hex(data_CON_NEW, length-10, out_data, &out_data_len);
		for (i = 0; i < out_data_len; i++) {
        printf("0x%02x ", out_data[i]);
			  myaddr[5-i]=out_data[i];
    }
		FLAG=3;
		memset(data_CON_NEW, 0, sizeof(data_CON_NEW));
	}
		// SCAN: AT+SCAN=\r\n
	else if((length >= 8) && (strncmp((char*)pBuffer, "AT+SCAN=", 8) == 0))//SCAN
	{
	
		 printf("scan\r\n");
			FLAG=2;
	}
	// Filter enable: AT+FILTER=N\r\n, 1:enable, 0:disable
	else if((length == 13) && (strncmp((char*)pBuffer, "AT+FILTER=", 10) == 0))
	{
		uint32_t filter_enable_tmp;
		sscanf((char*)pBuffer, "AT+FILTER=%x\r\n", &filter_enable_tmp);
		if((filter_enable_tmp == 0) || (filter_enable_tmp == 1))
		{
			if(filter_enable_tmp == 0)
			{
				// disable all filter keywords 
				for(uint8_t i=0; i < USER_FILTER_MAX_INDEX; i++)
				{
					user[i].cmd_enable = 0;
				}
			}
			
			filter_enable = filter_enable_tmp;
			printf("AT+FILTER:OK\r\n");
		}
		else
		{
			printf("AT+FILTER:ERP\r\n");
		}
	}
	
	// User filter format rule:AT+USER=M,N,XYZ\r\n
	else if((length >= 15) && (strncmp((char*)pBuffer, "AT+USER=", 8) == 0))
	{
		uint32_t user_cmd_index_tmp, user_cmd_enable_tmp;
		
		sscanf((char*)pBuffer, "AT+USER=%x,%x,", 
				&user_cmd_index_tmp, 	// user cmd index range
				&user_cmd_enable_tmp);	// user cmd enable:0:disable, 1:enable
		if((user_cmd_index_tmp < USER_FILTER_MAX_INDEX)
			&&((user_cmd_enable_tmp == 0) ||(user_cmd_enable_tmp == 1)))
		{
			user[user_cmd_index_tmp].cmd_enable = user_cmd_enable_tmp;
			
			char* ans;
			ans = strrchr((char*)pBuffer, ',');
			ans++;	// atfter ','
			
			memcpy(user[user_cmd_index_tmp].filter_keywords, ans, USER_FILTER_LEN);
			
			char *p = strstr(ans, "\r\n");
			user[user_cmd_index_tmp].filter_length = p - ans; 
						
			printf("AT+USER:OK\r\n");
		}
		else
		{
			printf("AT+USER:ERP\r\n");
		}
	}	
}
unsigned int ble_len[]={
172,229,234,174,195,239,223,238,238,221,190,231,228,240,244,227,218,86,192,223,236,
236,238,218,234,243,41,239,201,227,228,240,244,227,218,154,238,44
};

uint8_t buff_data[245];
uint8_t* message_data=buff_data;
const unsigned char *image_data=ble_data;

void send_bl(void)
{
//	NRF_LOG_INFO("%d",sizeof(ble_data));
	int i,j =0;
	for(i=0;i<38;i++)
	{
			nrf_delay_ms(300);    // 延时
		 memcpy(message_data, image_data, ble_len[i]);
		image_data+=ble_len[i];
		
		ble_nus_c_string_send(&m_ble_nus_c,message_data,ble_len[i]);
		memset(message_data,0,245);		
//	NRF_LOG_INFO(" %d",ble_len[i]);
//	for(j=0;j<ble_len[i];j++)
//	{
//		NRF_LOG_INFO(" %02x",image_data[j]);
//	}
}
	NRF_LOG_INFO(" OVER");
}
//7948=7583+38*11=8001






/**@brief   Function for handling app_uart events.
 *
 * @details This function will receive a single character from the app_uart module and append it to
 *          a string. The string will be be sent over BLE when the last character received was a
 *          'new line' '\n' (hex 0x0A) or if the string has reached the maximum data length.
 */
int flag;
uint8_t data[247];
uint8_t* BLE_NUS=data;
uint8_t data_C[247];
uint8_t* BLE_CHANGE=data_C;
unsigned char *BLE_RAW;
void uart_event_handle(app_uart_evt_t * p_event)
{
    static uint8_t data_array[BLE_NUS_MAX_DATA_LEN];
    static uint16_t index = 0;
		static uint16_t count = 0;
		static uint16_t cnt = 0;
	unsigned char *BLE_RAW=data_array;
  uint32_t ret_val;

    switch (p_event->evt_type)
    {
        /**@snippet [Handling data from UART] */
        case APP_UART_DATA_READY:
            UNUSED_VARIABLE(app_uart_get(&data_array[index]));
            index++;	
						count++;
						cnt=count/index;
          
           if ((data_array[index - 1] == '\n') || (index >= (244)))
            {
							
							
							NRF_LOG_DEBUG("Ready to send data over BLE NUS");
                NRF_LOG_HEXDUMP_DEBUG(data_array, index);

//                do
//                {
//									printf(" %d",cnt);
//                    ret_val = ble_nus_c_string_send(&m_ble_nus_c, data_array, index);
//                    if ( (ret_val != NRF_ERROR_INVALID_STATE) && (ret_val != NRF_ERROR_RESOURCES) )
//                    {
//                        APP_ERROR_CHECK(ret_val);
//                    }
//                } while (ret_val == NRF_ERROR_RESOURCES);
							
							
							
							//NRF_LOG_INFO("ESP32:%s\r\n", data_array);		
							#if NRF_LOG_ENABLED	
                NRF_LOG_DEBUG("Ready to send data over BLE NUS");
               NRF_LOG_HEXDUMP_DEBUG(data_array, index);
							#endif	
						// uart printf back uart received data
//							if((index >= 0) && (strncmp((char*)data_array, "AT", 2) != 0))					
//							{
//								printf("CALLBACK:%s\r\n", data_array);
//							}
							
								// SEND DATA: AT+SEND\r\n
							
					if((index >= 8) && (strncmp((char*)data_array, "AT+SEND=", 8) == 0))
					{
						printf("AT:OK\r\n");
						memcpy( BLE_NUS,BLE_RAW,index-2 );
						BLE_NUS+=8;				
						ble_nus_c_string_send(&m_ble_nus_c, BLE_NUS,index-10);			
						printf("%s",BLE_NUS);
						memset(BLE_NUS, 0, sizeof(data));
					}
					
					if((index >= 6) && (strncmp((char*)data_array, "AT+TEST", 7) == 0))
					{
						send_bl();
					}
								
						if((index >= 8) && (strncmp((char*)data_array, "AT+SPEED", 8) == 0))
					{
APP_ERROR_CHECK(app_timer_start(m_timer_speed, APP_TIMER_TICKS(7),NULL));	//开启定时器	
#ifndef APP_QUEUE	//测速专用		
			ret_code_t err_code;
			uint16_t length;	
			
			//sending code lines
			length = m_ble_nus_max_data_len;	
			do
			{		
					err_code=ble_nus_c_string_send(&m_ble_nus_c, m_data_array,length);	
				
				if ( (err_code != NRF_ERROR_INVALID_STATE) && (err_code != NRF_ERROR_RESOURCES) &&
						 (err_code != NRF_ERROR_NOT_FOUND) )
				{
						APP_ERROR_CHECK(err_code);
				}
				if (err_code == NRF_SUCCESS)
				{
					m_len_sent += length; 	
					m_data_array[0]++;
					m_data_array[length-1]++;	
				}
			} while (err_code == NRF_SUCCESS);
#else
	
#ifndef APP_DATA	
     ble_data_send_with_queue();
#endif
#endif
					
		}
							
								AT_cmd_handle(data_array, index);

                index = 0;
            }
            break;

        /**@snippet [Handling data from UART] */
        case APP_UART_COMMUNICATION_ERROR:
            NRF_LOG_ERROR("Communication error occurred while handling UART.");
            APP_ERROR_HANDLER(p_event->data.error_communication);
            break;

        case APP_UART_FIFO_ERROR:
            NRF_LOG_ERROR("Error occurred in FIFO module used by UART.");
            APP_ERROR_HANDLER(p_event->data.error_code);
            break;

        default:
            break;
    }
}


/**@brief Callback handling NUS Client events.
 *
 * @details This function is called to notify the application of NUS client events.
 *
 * @param[in]   p_ble_nus_c   NUS Client Handle. This identifies the NUS client
 * @param[in]   p_ble_nus_evt Pointer to the NUS Client event.
 */

/**@snippet [Handling events from the ble_nus_c module] */
static void ble_nus_c_evt_handler(ble_nus_c_t * p_ble_nus_c, ble_nus_c_evt_t const * p_ble_nus_evt)
{
    ret_code_t err_code;

    switch (p_ble_nus_evt->evt_type)
    {
        case BLE_NUS_C_EVT_DISCOVERY_COMPLETE:
            NRF_LOG_INFO("Discovery complete.");
            err_code = ble_nus_c_handles_assign(p_ble_nus_c, p_ble_nus_evt->conn_handle, &p_ble_nus_evt->handles);
            APP_ERROR_CHECK(err_code);

            err_code = ble_nus_c_tx_notif_enable(p_ble_nus_c);
            APP_ERROR_CHECK(err_code);
            NRF_LOG_INFO("Connected to device with Nordic UART Service.");
            break;

        case BLE_NUS_C_EVT_NUS_TX_EVT:
            ble_nus_chars_received_uart_print(p_ble_nus_evt->p_data, p_ble_nus_evt->data_len);
            break;

        case BLE_NUS_C_EVT_DISCONNECTED:
            NRF_LOG_INFO("Disconnected.");
            scan_start();
            break;
    }
}
/**@snippet [Handling events from the ble_nus_c module] */


/**
 * @brief Function for shutdown events.
 *
 * @param[in]   event       Shutdown type.
 */
static bool shutdown_handler(nrf_pwr_mgmt_evt_t event)
{
    ret_code_t err_code;

    err_code = bsp_indication_set(BSP_INDICATE_IDLE);
    APP_ERROR_CHECK(err_code);

    switch (event)
    {
        case NRF_PWR_MGMT_EVT_PREPARE_WAKEUP:
            // Prepare wakeup buttons.
            err_code = bsp_btn_ble_sleep_mode_prepare();
            APP_ERROR_CHECK(err_code);
            break;

        default:
            break;
    }

    return true;
}

NRF_PWR_MGMT_HANDLER_REGISTER(shutdown_handler, APP_SHUTDOWN_HANDLER_PRIORITY);

static bool find_peer_addr( const ble_gap_evt_adv_report_t *p_adv_report, const ble_gap_addr_t * p_addr)
{
	  ble_gap_addr_t m_addr;
    m_addr.addr_id_peer = 1;
    m_addr.addr_type = BLE_GAP_ADDR_TYPE_PUBLIC;
		memcpy(m_addr.addr,p_addr,BLE_GAP_ADDR_LEN);
//	if(m_addr.addr==p_adv_report->peer_addr.addr)
//	{

			 sd_ble_gap_connect(&m_addr,&m_scan_params,&m_connection_param,APP_BLE_CONN_CFG_TAG);
			return true;
// 停止扫描
            nrf_ble_scan_stop();
//	}
//	      return false;
}


/**@brief Function for handling the advertising report BLE event.
 *
 * @param[in] p_adv_report  Advertising report from the SoftDevice.
 */
static void on_adv_report(ble_gap_evt_adv_report_t const * p_adv_report)
{
    ret_code_t err_code;
		const ble_evt_t *const p_ble_evt;
		const ble_gap_evt_t *const p_gap_evt  =&p_ble_evt->evt.gap_evt;	
		const ble_gap_addr_t *const peer_addr =&p_gap_evt->params.adv_report.peer_addr;	
		int do_scan=0;
//    if (ble_advdata_uuid_find(p_adv_report->data.p_data, p_adv_report->data.len, &m_nus_uuid))
//    {
//        err_code = sd_ble_gap_connect(&p_adv_report->peer_addr,
//                                      &m_scan_params,
//                                      &m_connection_param,
//                                      APP_BLE_CONN_CFG_TAG);

//        if (err_code == NRF_SUCCESS)
//        {
//            // scan is automatically stopped by the connect
//            err_code = bsp_indication_set(BSP_INDICATE_IDLE);
//            APP_ERROR_CHECK(err_code);
//            NRF_LOG_INFO("scan MAC %02x%02x%02x%02x%02x%02x\r\n",
//                     p_adv_report->peer_addr.addr[0],
//                     p_adv_report->peer_addr.addr[1],
//                     p_adv_report->peer_addr.addr[2],
//                     p_adv_report->peer_addr.addr[3],
//                     p_adv_report->peer_addr.addr[4],
//                     p_adv_report->peer_addr.addr[5]
//                     );
//					do_scan=true;
//        }
//    }

//		 if (ble_advdata_name_find(p_adv_report->data.p_data, p_adv_report->data.len, m_nus_name))
//    {
//        err_code = sd_ble_gap_connect(&p_adv_report->peer_addr,
//                                      &m_scan_params,
//                                      &m_connection_param,
//                                      APP_BLE_CONN_CFG_TAG);

//        if (err_code == NRF_SUCCESS)
//        {
//            // scan is automatically stopped by the connect
//            err_code = bsp_indication_set(BSP_INDICATE_IDLE);
//            APP_ERROR_CHECK(err_code);
//            NRF_LOG_INFO("scan MAC %02x%02x%02x%02x%02x%02x\r\n",
//                     p_adv_report->peer_addr.addr[0],
//                     p_adv_report->peer_addr.addr[1],
//                     p_adv_report->peer_addr.addr[2],
//                     p_adv_report->peer_addr.addr[3],
//                     p_adv_report->peer_addr.addr[4],
//                     p_adv_report->peer_addr.addr[5]
//                     );
//					do_scan=true;
//        }
//    }
//		
				 if (find_peer_addr(&p_gap_evt->params.adv_report, &m_nus_addr))
    {
//       err_code = sd_ble_gap_connect(&p_adv_report->peer_addr,
//                                      &m_scan_params,
//                                      &m_connection_param,
//                                      APP_BLE_CONN_CFG_TAG);

        if (err_code == NRF_SUCCESS)
        {
            // scan is automatically stopped by the connect
            err_code = bsp_indication_set(BSP_INDICATE_IDLE);
            APP_ERROR_CHECK(err_code);
            NRF_LOG_INFO("scan MAC %02x%02x%02x%02x%02x%02x\r\n",
                     p_adv_report->peer_addr.addr[0],
                     p_adv_report->peer_addr.addr[1],
                     p_adv_report->peer_addr.addr[2],
                     p_adv_report->peer_addr.addr[3],
                     p_adv_report->peer_addr.addr[4],
                     p_adv_report->peer_addr.addr[5]
                     );
        }
				
				do_scan=true;
    }
		
		
    else
    {
        err_code = sd_ble_gap_scan_start(NULL, &m_scan_buffer);
        APP_ERROR_CHECK(err_code);
    }
}
#define BLE_GAP_ADV_MAX_SIZE            (31)
void filter(void)
{
	ble_evt_t const * p_ble_evt;
	ret_code_t            err_code;
  ble_gap_evt_t const * p_gap_evt = &p_ble_evt->evt.gap_evt;
	ble_data_t const data_T;
	
  ble_gap_evt_adv_report_t const * p_adv_report = &p_gap_evt->params.adv_report;
			
			uint8_t buff[60];
			uint8_t device_name[BLE_GAP_ADV_MAX_SIZE + 1];
			
			uint8_t filter_flag = 0;	// find device name when filter enable
			uint8_t no_filter_flag = 0;	// find device name when filter disable
			
			uint8_t ad_len;		// AD Length in a AD Structure
			uint8_t ad_type;	// AD Type   in a AD Structure
			uint8_t index = 0;
			
			// search for device name
			// AD Structure include AD Length | AD Type | AD Data
			while((index < (BLE_GAP_ADV_MAX_SIZE - 1)) && (index < p_adv_report->data.len))
			{
				ad_len = p_adv_report->data.len - 1;
				ad_type = p_adv_report->type.scan_response;
				if((ad_type == 0x08) || (ad_type == 0x09)) 
				{
					memcpy(device_name, &p_adv_report->data.len+ 2, ad_len);
					device_name[ad_len] = '\0';
					
					if(filter_enable)
					{
						// device name filter(first USER_FILTER_LEN Bytes)
						for(uint8_t cnt=0; cnt < USER_FILTER_MAX_INDEX; cnt++)
						{
							if(user[cnt].cmd_enable 
								&& !memcmp(device_name, user[cnt].filter_keywords, user[cnt].filter_length))
							{
								filter_flag = 1;
								break;
							}
						}
					}
					else
					{
						no_filter_flag = 1;
					}//if(filter_enable)
					
					if(filter_flag || no_filter_flag)
					{
						memset(buff, 0, sizeof(buff));
						sprintf((char*)buff, "%s,%d,9,%s\r\n", // MAC address, RSSI, 9, device name
								Util_convertBdAddr2Str((uint8_t*)p_adv_report->peer_addr.addr),
								p_adv_report->rssi,
								device_name);
						
						// uart send
						for (uint8_t i = 0; i < strlen((char*)buff); i++)
						{
							do
							{
								err_code = app_uart_put(buff[i]);
								if ((err_code != NRF_SUCCESS) && (err_code != NRF_ERROR_BUSY))
								{
								#if NRF_LOG_ENABLED
									NRF_LOG_INFO("Failed uart tx message. Error 0x%x. ", err_code);
								#endif	
									APP_ERROR_CHECK(err_code);
								}
							} while (err_code == NRF_ERROR_BUSY);
						}
					}//end if(filter_flag || no_filter_flag)
					
					break;
				}
				else
				{
					index += p_adv_report->data.len + 1;
				}
			}	
	
}



/**@brief Function for handling BLE events.
 *
 * @param[in]   p_ble_evt   Bluetooth stack event.
 * @param[in]   p_context   Unused.
 */
static void ble_evt_handler(ble_evt_t const * p_ble_evt, void * p_context)
{
    ret_code_t            err_code;
    ble_gap_evt_t const * p_gap_evt = &p_ble_evt->evt.gap_evt;
 ble_gap_evt_connected_t const * p_connected_evt = &p_gap_evt->params.connected;
    switch (p_ble_evt->header.evt_id)
    {
        case BLE_GAP_EVT_ADV_REPORT:
							//filter();
     if(FLAG==2)    
		 {	
				if (p_gap_evt->params.adv_report.type.scan_response)
            {
                if (p_gap_evt->params.adv_report.data.len > 0)
                {
                    NRF_LOG_INFO("Scan response received:");
                    NRF_LOG_RAW_HEXDUMP_INFO(p_gap_evt->params.adv_report.data.p_data, p_gap_evt->params.adv_report.data.len);
                }
                else
                {
                    NRF_LOG_INFO("Empty scan response received.");
                }
            }
            else
            {
                NRF_LOG_INFO("Advertising packet received:");
                NRF_LOG_RAW_HEXDUMP_INFO(p_gap_evt->params.adv_report.data.p_data, p_gap_evt->params.adv_report
							.data.len);
							NRF_LOG_INFO("MAC %02x%02x%02x%02x%02x%02x\r\n",
                     p_gap_evt->params.adv_report.peer_addr.addr[0],
                     p_gap_evt->params.adv_report.peer_addr.addr[1],
                     p_gap_evt->params.adv_report.peer_addr.addr[2],
                     p_gap_evt->params.adv_report.peer_addr.addr[3],
                     p_gap_evt->params.adv_report.peer_addr.addr[4],
                     p_gap_evt->params.adv_report.peer_addr.addr[5]
                     );
            }
			}
						 //  on_adv_report(&p_gap_evt->params.adv_report);
							
            // Continue scanning.
            //sd_ble_gap_scan_start(NULL, &m_scan_buffer);
						
            break; // BLE_GAP_EVT_ADV_REPORT

        case BLE_GAP_EVT_CONNECTED:
            NRF_LOG_INFO("Connected to target");
						   NRF_LOG_INFO("Connected. conn_DevAddr: %s\nConnected. conn_handle: 0x%04x\nConnected. conn_Param: %d,%d,%d,%d",
                         Util_convertBdAddr2Str((uint8_t*)p_connected_evt->peer_addr.addr),
                         p_gap_evt->conn_handle,
                         p_connected_evt->conn_params.min_conn_interval,
                         p_connected_evt->conn_params.max_conn_interval,
                         p_connected_evt->conn_params.slave_latency,
                         p_connected_evt->conn_params.conn_sup_timeout
                         );
            err_code = ble_nus_c_handles_assign(&m_ble_nus_c, p_ble_evt->evt.gap_evt.conn_handle, NULL);
            APP_ERROR_CHECK(err_code);

            err_code = bsp_indication_set(BSP_INDICATE_CONNECTED);
            APP_ERROR_CHECK(err_code);

            // start discovery of services. The NUS Client waits for a discovery result
            err_code = ble_db_discovery_start(&m_db_disc, p_ble_evt->evt.gap_evt.conn_handle);
            APP_ERROR_CHECK(err_code);
            break;

        case BLE_GAP_EVT_TIMEOUT:
            if (p_gap_evt->params.timeout.src == BLE_GAP_TIMEOUT_SRC_SCAN)
            {
                NRF_LOG_INFO("Scan timed out.");
                scan_start();
            }
            else if (p_gap_evt->params.timeout.src == BLE_GAP_TIMEOUT_SRC_CONN)
            {
                NRF_LOG_INFO("Connection Request timed out.");
            }
            break;

        case BLE_GAP_EVT_SEC_PARAMS_REQUEST:
            // Pairing not supported
            err_code = sd_ble_gap_sec_params_reply(p_ble_evt->evt.gap_evt.conn_handle, BLE_GAP_SEC_STATUS_PAIRING_NOT_SUPP, NULL, NULL);
            APP_ERROR_CHECK(err_code);
            break;

        case BLE_GAP_EVT_CONN_PARAM_UPDATE_REQUEST:
            // Accepting parameters requested by peer.
            err_code = sd_ble_gap_conn_param_update(p_gap_evt->conn_handle,
                                                    &p_gap_evt->params.conn_param_update_request.conn_params);
            APP_ERROR_CHECK(err_code);
            break;

        case BLE_GAP_EVT_PHY_UPDATE_REQUEST:
        {
            NRF_LOG_DEBUG("PHY update request.");
            ble_gap_phys_t const phys =
            {
                .rx_phys = BLE_GAP_PHY_AUTO,
                .tx_phys = BLE_GAP_PHY_AUTO,
            };
            err_code = sd_ble_gap_phy_update(p_ble_evt->evt.gap_evt.conn_handle, &phys);
            APP_ERROR_CHECK(err_code);
        } break;

        case BLE_GATTC_EVT_TIMEOUT:
            // Disconnect on GATT Client timeout event.
            NRF_LOG_DEBUG("GATT Client Timeout.");
            err_code = sd_ble_gap_disconnect(p_ble_evt->evt.gattc_evt.conn_handle,
                                             BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
            APP_ERROR_CHECK(err_code);
            break;

        case BLE_GATTS_EVT_TIMEOUT:
            // Disconnect on GATT Server timeout event.
            NRF_LOG_DEBUG("GATT Server Timeout.");
            err_code = sd_ble_gap_disconnect(p_ble_evt->evt.gatts_evt.conn_handle,
                                             BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
            APP_ERROR_CHECK(err_code);
            break;

        default:
            break;
    }
}


/**@brief Function for initializing the BLE stack.
 *
 * @details Initializes the SoftDevice and the BLE event interrupt.
 */
static void ble_stack_init(void)
{
    ret_code_t err_code;

    err_code = nrf_sdh_enable_request();
    APP_ERROR_CHECK(err_code);

    // Configure the BLE stack using the default settings.
    // Fetch the start address of the application RAM.
    uint32_t ram_start = 0;
    err_code = nrf_sdh_ble_default_cfg_set(APP_BLE_CONN_CFG_TAG, &ram_start);
    APP_ERROR_CHECK(err_code);

    // Enable BLE stack.
    err_code = nrf_sdh_ble_enable(&ram_start);
    APP_ERROR_CHECK(err_code);

    // Register a handler for BLE events.
    NRF_SDH_BLE_OBSERVER(m_ble_observer, APP_BLE_OBSERVER_PRIO, ble_evt_handler, NULL);
}


/**@brief Function for handling events from the GATT library. */
void gatt_evt_handler(nrf_ble_gatt_t * p_gatt, nrf_ble_gatt_evt_t const * p_evt)
{
    if (p_evt->evt_id == NRF_BLE_GATT_EVT_ATT_MTU_UPDATED)
    {
        NRF_LOG_INFO("ATT MTU exchange completed.");

        m_ble_nus_max_data_len = p_evt->params.att_mtu_effective - OPCODE_LENGTH - HANDLE_LENGTH;
        NRF_LOG_INFO("Ble NUS max data length set to 0x%X(%d)", m_ble_nus_max_data_len, m_ble_nus_max_data_len);
    }
}


/**@brief Function for initializing the GATT library. */
void gatt_init(void)
{
    ret_code_t err_code;

    err_code = nrf_ble_gatt_init(&m_gatt, gatt_evt_handler);
    APP_ERROR_CHECK(err_code);

    err_code = nrf_ble_gatt_att_mtu_central_set(&m_gatt, NRF_SDH_BLE_GATT_MAX_MTU_SIZE);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for handling events from the BSP module.
 *
 * @param[in] event  Event generated by button press.
 */
void bsp_event_handler(bsp_event_t event)
{
    ret_code_t err_code;

    switch (event)
    {
        case BSP_EVENT_SLEEP:
            nrf_pwr_mgmt_shutdown(NRF_PWR_MGMT_SHUTDOWN_GOTO_SYSOFF);
            break;

        case BSP_EVENT_DISCONNECT:
            err_code = sd_ble_gap_disconnect(m_ble_nus_c.conn_handle,
                                             BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
            if (err_code != NRF_ERROR_INVALID_STATE)
            {
                APP_ERROR_CHECK(err_code);
            }
            break;

        default:
            break;
    }
}


buffer_t m_buf;
void ble_data_send_with_queue(void)
{
	uint32_t err_code;
	uint16_t length = 0;
	static bool retry = false;
	
	if (retry)
	{
		length = m_buf.length;
			err_code=ble_nus_c_string_send(&m_ble_nus_c, m_data_array,length);	
		//NRF_LOG_INFO("Data2: %d", m_buf.p_data[0]);
		if ( (err_code != NRF_ERROR_INVALID_STATE) && (err_code != NRF_ERROR_RESOURCES) &&
				 (err_code != NRF_ERROR_NOT_FOUND) )
		{
				APP_ERROR_CHECK(err_code);
		}
		if (err_code == NRF_SUCCESS)
		{
			m_len_sent += length;
			retry = false;
		}
	}
	
	while (!nrf_queue_is_empty(&m_buf_queue) && !retry)
	{		

		err_code = nrf_queue_pop(&m_buf_queue, &m_buf);
		APP_ERROR_CHECK(err_code);		
		length = m_buf.length;
					
			err_code=ble_nus_c_string_send(&m_ble_nus_c, m_data_array,length);	
		//NRF_LOG_INFO("Data: %d", m_buf.p_data[0]);
		if ( (err_code != NRF_ERROR_INVALID_STATE) && (err_code != NRF_ERROR_RESOURCES) &&
				 (err_code != NRF_ERROR_NOT_FOUND) )
		{
				APP_ERROR_CHECK(err_code);
		}
		if (err_code == NRF_SUCCESS)
		{
			m_len_sent += length;
			retry = false;
		}
		else
		{
			retry = true;
			break;
		}
	}			
}

static void throughput_timer_handler(void * p_context)
{
#ifndef APP_QUEUE	
	//the snippet used to test data throughput only. no queue is involved
	ret_code_t err_code;
	uint16_t length;
	m_cnt_7ms++;	
	//sending code lines
	length = m_ble_nus_max_data_len;	
	do
	{					
			err_code=ble_nus_c_string_send(&m_ble_nus_c, m_data_array,length);	
		if ( (err_code != NRF_ERROR_INVALID_STATE) && (err_code != NRF_ERROR_RESOURCES) &&
				 (err_code != NRF_ERROR_NOT_FOUND) )
		{
				APP_ERROR_CHECK(err_code);
		}
		if (err_code == NRF_SUCCESS)
		{
			m_len_sent += length; 	
			m_data_array[0]++;
			m_data_array[length-1]++;	
		}
	} while (err_code == NRF_SUCCESS);

	//calculate speed every 1 second
	if (m_cnt_7ms == 143)
	{
		NRF_LOG_INFO("==**Speed: %d B/s**==", m_len_sent);
		m_cnt_7ms = 0;
		m_len_sent = 0;
		m_data_array[0] = 0;
		m_data_array[length-1] = 0;
	}	
	NRF_LOG_INFO("PacketNo.: %d == Time: %d *7ms", m_data_array[0], m_cnt_7ms);	
#else
	//the snippet simulate a real application scenairo. Queue is involved.
	ret_code_t err_code1, err_code2;	
	buffer_t buf;
	static uint8_t val = 0;
	//produce the data irregard of BLE activity
	m_data_array[(m_cnt_7ms%10)*420] = val++;
	m_data_array[(m_cnt_7ms%10)*420+210] = val++;
	
	//put the data into a queue to cache them
	buf.p_data = &m_data_array[(m_cnt_7ms%10)*420];
	buf.length = MIN(m_ble_nus_max_data_len,210);
	err_code1 = nrf_queue_push(&m_buf_queue, &buf);
	//APP_ERROR_CHECK(err_code1); //it may return NRF_ERROR_NO_MEM. we skip this error
	
	buf.p_data = &m_data_array[(m_cnt_7ms%10)*420+210];
	buf.length = MIN(m_ble_nus_max_data_len,210);
	err_code2 = nrf_queue_push(&m_buf_queue, &buf);
	//APP_ERROR_CHECK(err_code2);	//it may return NRF_ERROR_NO_MEM. we skip this error
	
	ble_data_send_with_queue();
	
	if(err_code1 == NRF_ERROR_NO_MEM || err_code2 == NRF_ERROR_NO_MEM)
	{
		NRF_LOG_INFO("Drop");	
	}
	
	m_cnt_7ms++;	
	//calculate speed every 1 second
	if (m_cnt_7ms == 143)
	{
		NRF_LOG_INFO("==**Speed: %d B/s**==", m_len_sent);
		m_cnt_7ms = 0;
		m_len_sent = 0;
	}	
	NRF_LOG_INFO("Time: %d *7ms", m_cnt_7ms);		
	
#endif	
}


void throughput_test(void)
{
	ret_code_t err_code;
	err_code = app_timer_create(&m_timer_speed, APP_TIMER_MODE_REPEATED, throughput_timer_handler);
	APP_ERROR_CHECK(err_code);

#if 0
	  ble_opt_t  opt;
    memset(&opt, 0x00, sizeof(opt));
    opt.common_opt.conn_evt_ext.enable = true;
    err_code = sd_ble_opt_set(BLE_COMMON_OPT_CONN_EVT_EXT, &opt);
    APP_ERROR_CHECK(err_code);
#endif	
	
}





/**@brief Function for initializing the UART. */
static void uart_init(void)
{
    ret_code_t err_code;

    app_uart_comm_params_t const comm_params =
    {
        .rx_pin_no    = RX_PIN_NUMBER,
        .tx_pin_no    = TX_PIN_NUMBER,
        .rts_pin_no   = RTS_PIN_NUMBER,
        .cts_pin_no   = CTS_PIN_NUMBER,
        .flow_control = APP_UART_FLOW_CONTROL_DISABLED,
        .use_parity   = false,
        .baud_rate    = UART_BAUDRATE_BAUDRATE_Baud230400
    };

    APP_UART_FIFO_INIT(&comm_params,
                       UART_RX_BUF_SIZE,
                       UART_TX_BUF_SIZE,
                       uart_event_handle,
                       APP_IRQ_PRIORITY_LOWEST,
                       err_code);

    APP_ERROR_CHECK(err_code);
}

/**@brief Function for initializing the NUS Client. */
static void nus_c_init(void)
{
    ret_code_t       err_code;
    ble_nus_c_init_t init;

    init.evt_handler = ble_nus_c_evt_handler;

    err_code = ble_nus_c_init(&m_ble_nus_c, &init);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for initializing buttons and leds. */
static void buttons_leds_init(void)
{
    ret_code_t err_code;
    bsp_event_t startup_event;

    err_code = bsp_init(BSP_INIT_LEDS, bsp_event_handler);
    APP_ERROR_CHECK(err_code);

    err_code = bsp_btn_ble_init(NULL, &startup_event);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for initializing the timer. */
static void timer_init(void)
{
    ret_code_t err_code = app_timer_init();
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for initializing the nrf log module. */
static void log_init(void)
{
    ret_code_t err_code = NRF_LOG_INIT(NULL);
    APP_ERROR_CHECK(err_code);

    NRF_LOG_DEFAULT_BACKENDS_INIT();
}


/**@brief Function for initializing power management.
 */
static void power_management_init(void)
{
    ret_code_t err_code;
    err_code = nrf_pwr_mgmt_init();
    APP_ERROR_CHECK(err_code);
}


/** @brief Function for initializing the Database Discovery Module. */
static void db_discovery_init(void)
{
    ret_code_t err_code = ble_db_discovery_init(db_disc_handler);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for handling the idle state (main loop).
 *
 * @details Handle any pending log operation(s), then sleep until the next event occurs.
 */
static void idle_state_handle(void)
{
    if (NRF_LOG_PROCESS() == false)
    {
        nrf_pwr_mgmt_run();
    }
}


int main(void)
{
    // Initialize.
    log_init();
    timer_init();
    uart_init();
    buttons_leds_init();
    db_discovery_init();
    power_management_init();
    ble_stack_init();
		scan_init();
    gatt_init();
    nus_c_init();

    // Start execution.
    printf("BLE UART central example started.\r\n");
    NRF_LOG_INFO("BLE UART central example started.");
    scan_start();
	
		throughput_test();//测速
	/*
	if(flag==1)
	{
		//test1();
		test();
	}
	*/
    // Enter main loop.
    for (;;)
    {
        idle_state_handle();
    }
}
