/* Copyright (c) 2013 Nordic Semiconductor. All Rights Reserved.
 *
 * The information contained herein is property of Nordic Semiconductor ASA.
 * Terms and conditions of usage are described in detail in NORDIC
 * SEMICONDUCTOR STANDARD SOFTWARE LICENSE AGREEMENT.
 *
 * Licensees are granted free, non-transferable use of the information. NO
 * WARRANTY of ANY KIND is provided. This heading must NOT be removed from
 * the file.
 *
 */

/** @file
 *
 * @defgroup ble_sdk_app_ant_hrs_main main.c
 * @{
 * @ingroup ble_sdk_app_ant_hrs
 * @brief HRM sample application using both BLE and ANT.
 *
 * The application uses the BLE Heart Rate Service (and also the Device Information
 * services), and the ANT HRM RX profile.
 *
 * It will open a receive channel which will connect to an ANT HRM TX profile device when the
 * application starts. The received data will be propagated to a BLE central through the
 * BLE Heart Rate Service.
 *
 * The ANT HRM TX profile device simulator SDK application
 * (Board\pca10003\ant\ant_hrm\hrm_tx_buttons) can be used as a peer ANT device. By changing
 * ANT_HRMRX_NETWORK_KEY to the ANT+ Network Key, the application will instead be able to connect to
 * an ANT heart rate belt.
 *
 * @note The ANT+ Network Key is available for ANT+ Adopters. Please refer to
 *       http://thisisant.com to become an ANT+ Adopter and access the key.
 *
 * @note This application is based on the BLE Heart Rate Service Sample Application
 *       (Board\nrf6310\ble\ble_app_hrs). Please refer to this application for additional
 *       documentation.
 */

#include <stdint.h>
#include <string.h>
#include "nordic_common.h"
#include "nrf.h"
#include "app_error.h"
#include "app_uart.h"
#include "nrf_gpio.h"
#include "nrf_delay.h"
#include "nrf.h"
#include "ble.h"
#include "ble_hci.h"
#include "ble_srv_common.h"
#include "ble_advdata.h"
#include "ble_advertising.h"
#include "ble_hrs.h"
#include "ble_nus.h"
#include "ble_conn_params.h"
#include "boards.h"
#include "sensorsim.h"
#include "softdevice_handler.h"
#include "app_timer.h"
#include "device_manager.h"
#include "ant_parameters.h"
#include "pstorage.h"
#include "app_trace.h"
#include "bsp.h"
#include "ant_error.h"
#include "ant_key_manager.h"
#include "ant_stack_config.h"
#include "ant_hrm.h"
#include "ant_bsc.h"
#include "ant_glasses.h"
#include "ant_interface.h"

#if (WDT_ENABLED==1)
#include "nrf_drv_wdt.h"
#endif

#if (NRF_LOG_USES_RTT==1)
#include "SEGGER_RTT.h"
#endif

#ifdef BLE_DFU_APP_SUPPORT
#include "ble_dfu.h"
#include "dfu_app_handler.h"
#endif // BLE_DFU_APP_SUPPORT


#define TAILLE_BUFFER 30


// 30
#define GPIO_BUTTON                    30

#define WILDCARD_TRANSMISSION_TYPE      0x00
#define HRMRX_NETWORK_KEY               {0xB9, 0xA5, 0x21, 0xFB, 0xBD, 0x72, 0xC3, 0x45}

#define BSC_RX_CHANNEL_NUMBER              0x01                                                     /**< Wildcard transmission type. */
#define BSC_RX_DEVICE_NUMBER               0xB02B                                                            /**< Wildcard device number. */
#define BSC_RX_DEVICE_TYPE                 0x79

#define HRM_RX_CHANNEL_NUMBER              0x00                                            /**< Default ANT Channel. */
#define HRM_RX_DEVICE_NUMBER         0x0D22                                            /**< Device Number. */

#define GLASSES_TX_CHANNEL_NUMBER              0x02                                            /**< Default ANT Channel. */
#define GLASSES_TX_DEVICE_NUMBER         0x0D22                                            /**< Device Number. */


#define ANTPLUS_NETWORK_NUMBER          0x00                                           /**< Network number. */


#define WAKEUP_BUTTON_ID                0                                            /**< Button used to wake up the application. */
#define BOND_DELETE_ALL_BUTTON_ID       1                                            /**< Button used for deleting all bonded centrals during startup. */

#define DEVICE_NAME                     "Nordic_HRM"                                 /**< Name of device. Will be included in the advertising data. */
#define MANUFACTURER_NAME               "NordicSemiconductor"                        /**< Manufacturer. Will be passed to Device Information Service. */
#define APP_ADV_INTERVAL                40                                           /**< The advertising interval (in units of 0.625 ms. This value corresponds to 25 ms). */
#define APP_ADV_TIMEOUT_IN_SECONDS      180                                          /**< The advertising timeout in units of seconds. */

#define APP_ADV_FAST_INTERVAL           40                                          /**< The advertising interval (in units of 0.625 ms). The default value corresponds to 25 ms. */
#define APP_ADV_SLOW_INTERVAL           3200                                        /**< Slow advertising interval (in units of 0.625 ms). The default value corresponds to 2 seconds. */
#define APP_ADV_FAST_TIMEOUT            180                                         /**< The advertising time-out in units of seconds. */
#define APP_ADV_SLOW_TIMEOUT            180                                         /**< The advertising time-out in units of seconds. */
#define ADV_INTERVAL_FAST_PERIOD        30                                          /**< The duration of the fast advertising period (in seconds). */

#define NUS_SERVICE_UUID_TYPE           BLE_UUID_TYPE_VENDOR_BEGIN

#define IS_SRVC_CHANGED_CHARACT_PRESENT 1                                            /**< Whether or not to include the service_changed characteristic. If not enabled, the server's database cannot be changed for the lifetime of the device */
#define APP_TIMER_PRESCALER             0                                            /**< Value of the RTC1 PRESCALER register. */

#ifdef USE_TUNES
#define APP_TIMER_OP_QUEUE_SIZE         3                                            /**< Size of timer operation queues. */
#else
#define APP_TIMER_OP_QUEUE_SIZE         2                                            /**< Size of timer operation queues. */
#endif

#define SECOND_1_25_MS_UNITS            800                                          /**< Definition of 1 second, when 1 unit is 1.25 ms. */
#define SECOND_10_MS_UNITS              100                                          /**< Definition of 1 second, when 1 unit is 10 ms. */
#define MIN_CONN_INTERVAL               (SECOND_1_25_MS_UNITS / 2)                   /**< Minimum acceptable connection interval (0.5 seconds), Connection interval uses 1.25 ms units. */
#define MAX_CONN_INTERVAL               (SECOND_1_25_MS_UNITS)                       /**< Maximum acceptable connection interval (1 second), Connection interval uses 1.25 ms units. */
#define SLAVE_LATENCY                   0                                            /**< Slave latency. */
#define CONN_SUP_TIMEOUT                (4 * SECOND_10_MS_UNITS)                     /**< Connection supervisory timeout (4 seconds), Supervision Timeout uses 10 ms units. */

#define FIRST_CONN_PARAMS_UPDATE_DELAY  APP_TIMER_TICKS(5000, APP_TIMER_PRESCALER)   /**< Time from initiating event (connect or start of notification) to first time sd_ble_gap_conn_param_update is called (5 seconds). */
#define NEXT_CONN_PARAMS_UPDATE_DELAY   APP_TIMER_TICKS(30000, APP_TIMER_PRESCALER)  /**< Time between each call to sd_ble_gap_conn_param_update after the first call (30 seconds). */
#define ANT_DELAY                       APP_TIMER_TICKS(10000, APP_TIMER_PRESCALER)
#define MAX_CONN_PARAMS_UPDATE_COUNT    3                                            /**< Number of attempts before giving up the connection parameter negotiation. */

#define SECURITY_REQUEST_DELAY          APP_TIMER_TICKS(1500, APP_TIMER_PRESCALER)  /**< Delay after connection until security request is sent, if necessary (ticks). */
#define PING_DELAY                      APP_TIMER_TICKS(10000, APP_TIMER_PRESCALER)
#define PLAY_DELAY                      APP_TIMER_TICKS(100, APP_TIMER_PRESCALER)

#define SEC_PARAM_TIMEOUT               30                                           /**< Timeout for Pairing Request or Security Request (in seconds). */
#define SEC_PARAM_BOND                  1                                            /**< Perform bonding. */
#define SEC_PARAM_MITM                  0                                            /**< Man In The Middle protection not required. */
#define SEC_PARAM_IO_CAPABILITIES       BLE_GAP_IO_CAPS_NONE                         /**< No I/O capabilities. */
#define SEC_PARAM_OOB                   0                                            /**< Out Of Band data not available. */
#define SEC_PARAM_MIN_KEY_SIZE          7                                            /**< Minimum encryption key size. */
#define SEC_PARAM_MAX_KEY_SIZE          16                                           /**< Maximum encryption key size. */

#define CENTRAL_LINK_COUNT              0                                            /**< Number of central links used by the application. When changing this number remember to adjust the RAM settings*/
#define PERIPHERAL_LINK_COUNT           1                                            /**< Number of peripheral links used by the application. When changing this number remember to adjust the RAM settings*/
#define VENDOR_SPECIFIC_UUID_COUNT      4                                           /**< The number of vendor specific UUIDs used by this example. */

#define DEAD_BEEF                       0xDEADBEEF                                   /**< Value used as error code on stack dump, can be used to identify stack location on stack unwind. */


#define UART_TX_BUF_SIZE 256                         /**< UART TX buffer size. */
#define UART_RX_BUF_SIZE 32                           /**< UART RX buffer size. */

#ifdef BLE_DFU_APP_SUPPORT
#define DFU_REV_MAJOR                    0x00                                       /** DFU Major revision number to be exposed. */
#define DFU_REV_MINOR                    0x01                                       /** DFU Minor revision number to be exposed. */
#define DFU_REVISION                     ((DFU_REV_MAJOR << 8) | DFU_REV_MINOR)     /** DFU Revision number to be exposed. Combined of major and minor versions. */
#define APP_SERVICE_HANDLE_START         0x000C                                     /**< Handle of first application specific service when when service changed characteristic is present. */
#define BLE_HANDLE_MAX                   0xFFFF                                     /**< Max handle value in BLE. */

STATIC_ASSERT(IS_SRVC_CHANGED_CHARACT_PRESENT);                                     /** When having DFU Service support in application the Service Changed Characteristic should always be present. */
#endif // BLE_DFU_APP_SUPPORT


#ifdef USE_TUNES
#include "tunes.h"
#include "app_pwm.h"
APP_PWM_INSTANCE(PWM1, 1);                   // Create the instance "PWM1" using TIMER1.
#endif

#ifdef TRACE_DEBUG
#define LOG printf
#else
#define LOG(...)
#endif 


static volatile bool ready_flag;            // A flag indicating PWM status.


static uint16_t                  m_conn_handle = BLE_CONN_HANDLE_INVALID;  /**< Handle of the current connection. */


static dm_application_instance_t m_app_handle;                             /**< Application identifier allocated by the Device Manager. */

static uint8_t nus_isinit = 0;
static ble_nus_t                 m_nus;                                      /**< Structure to identify the Nordic UART Service. */
static ble_hrs_t                 m_hrs;                                      /**< Structure used to identify the heart rate service. */
static ble_gap_adv_params_t             m_adv_params;                                /**< Parameters to be passed to the stack when starting advertising. */

static ble_uuid_t m_adv_uuids1[] = {{BLE_UUID_HEART_RATE_SERVICE,         BLE_UUID_TYPE_BLE},
                                    {BLE_UUID_BATTERY_SERVICE,            BLE_UUID_TYPE_BLE},
                                    {BLE_UUID_DEVICE_INFORMATION_SERVICE, BLE_UUID_TYPE_BLE}}; /**< Universally unique service identifiers. */

static ble_uuid_t m_adv_uuids2[] = {{BLE_UUID_NUS_SERVICE, NUS_SERVICE_UUID_TYPE}};  /**< Universally unique service identifier. */

                                   
                                   
APP_TIMER_DEF(m_sec_req_timer_id);                       /**< Security request timer. The timer lets us start pairing request if one does not arrive from the Central. */
APP_TIMER_DEF(m_sec_hrm);
APP_TIMER_DEF(m_sec_cad);
APP_TIMER_DEF(m_sec_play);
#ifdef USE_TUNES
APP_TIMER_DEF(m_sec_tune);
#endif

#ifdef BLE_DFU_APP_SUPPORT    
static ble_dfu_t                 m_dfus;                                    /**< Structure used to identify the DFU service. */
#endif // BLE_DFU_APP_SUPPORT  


/** @snippet [ANT BSC RX Instance] */
#define WHEEL_CIRCUMFERENCE         2070                                                            /**< Bike wheel circumference [mm] */
#define BSC_EVT_TIME_FACTOR         1024                                                            /**< Time unit factor for BSC events */
#define BSC_RPM_TIME_FACTOR         60                                                              /**< Time unit factor for RPM unit */
#define BSC_MS_TO_KPH_NUM           36                                                              /**< Numerator of [m/s] to [kph] ratio */
#define BSC_MS_TO_KPH_DEN           10                                                              /**< Denominator of [m/s] to [kph] ratio */
#define BSC_MM_TO_M_FACTOR          1000                                                            /**< Unit factor [m/s] to [mm/s] */
#define BSC_SPEED_UNIT_FACTOR       (BSC_MS_TO_KPH_DEN * BSC_MM_TO_M_FACTOR)                        /**< Speed unit factor */
#define SPEED_COEFFICIENT           (WHEEL_CIRCUMFERENCE * BSC_EVT_TIME_FACTOR * BSC_MS_TO_KPH_NUM) /**< Coefficient for speed value calculation */
#define CADENCE_COEFFICIENT         (BSC_EVT_TIME_FACTOR * BSC_RPM_TIME_FACTOR)                     /**< Coefficient for cadence value calculation */



/** @snippet [ANT BSC RX Instance] */
void ant_bsc_evt_handler(ant_bsc_profile_t * p_profile, ant_bsc_evt_t event);

BSC_DISP_CHANNEL_CONFIG_DEF(m_ant_bsc,
                            BSC_RX_CHANNEL_NUMBER,
                            WILDCARD_TRANSMISSION_TYPE,
                            BSC_RX_DEVICE_TYPE,
                            BSC_RX_DEVICE_NUMBER,
                            ANTPLUS_NETWORK_NUMBER,
                            BSC_MSG_PERIOD_4Hz);
BSC_DISP_PROFILE_CONFIG_DEF(m_ant_bsc, ant_bsc_evt_handler);
ant_bsc_profile_t m_ant_bsc;
/** @snippet [ANT BSC RX Instance] */

static int32_t accumulated_s_rev_cnt, previous_s_evt_cnt, prev_s_accumulated_rev_cnt,
               accumulated_s_evt_time, previous_s_evt_time, prev_s_accumulated_evt_time = 0;

static int32_t accumulated_c_rev_cnt, previous_c_evt_cnt, prev_c_accumulated_rev_cnt,
               accumulated_c_evt_time, previous_c_evt_time, prev_c_accumulated_evt_time = 0;


/** @snippet [ANT HRM RX Instance] */
HRM_DISP_CHANNEL_CONFIG_DEF(m_ant_hrm,
                            HRM_RX_CHANNEL_NUMBER,
                            WILDCARD_TRANSMISSION_TYPE,
                            HRM_RX_DEVICE_NUMBER,
                            ANTPLUS_NETWORK_NUMBER,
                            HRM_MSG_PERIOD_4Hz);
ant_hrm_profile_t           m_ant_hrm;

/** @snippet [ANT GLASSES TX Instance] */
ant_glasses_profile_t       m_ant_glasses;

const ant_channel_config_t  ant_tx_channel_config  = GLASSES_TX_CHANNEL_CONFIG(GLASSES_TX_CHANNEL_NUMBER, 5,
                                                    GLASSES_DEVICE_NUMBER, ANTPLUS_NETWORK_NUMBER);

//nrf_drv_wdt_channel_id wdt_channel_id;

static uint8_t is_hrm_init = 0;
static uint8_t is_cad_init = 0;

static bool  m_app_initialized   = false;                 /**< Application initialized flag. */


static uint8_t whichTune = 0;
#ifdef USE_TUNES
static uint8_t isTunePlaying = 0;
static uint32_t iTune = 0;
static uint8_t pwm_ready = 0;
void play_mario(void * p_context);

static void pwm_start(uint32_t period_us);
static void play_tune(void);
#endif

#if (LEDS_NUMBER > 0)
const uint8_t leds_list[LEDS_NUMBER] = LEDS_LIST;
#else
const uint8_t leds_list;
#endif

#define RB_SIZE 100
static uint8_t read_byte[RB_SIZE];
static uint8_t glasses_payload[8];
static uint16_t status_byte = 0;
static uint16_t marque_byte = 0;

void app_error_handler(uint32_t error_code, uint32_t line_num, const uint8_t *p_file_name) {
  
  if (error_code == NRF_SUCCESS) return;
  
#if (NRF_LOG_USES_RTT==1)
  log_rtt_printf(0, "Erreur: %u ligne%u %s!!\n", (unsigned int)error_code, (unsigned int)line_num, p_file_name); 
#endif
	
#ifdef DEBUG
	sprintf((char *)read_byte, "$ERR,%u,%u\n\r", (unsigned int)error_code, (unsigned int)line_num);
	
	read_byte[15] = '\0';
	read_byte[16] = '\0';
	if (nus_isinit) {
	uint32_t err_code = ble_nus_string_send(&m_nus, read_byte, 15);
       if (err_code != NRF_ERROR_INVALID_STATE)
       {
         APP_ERROR_CHECK(err_code);
       }
	}
#endif
  
  if (p_file_name) {
    printf("$DBG,1,%u,%u,%s\n\r", (unsigned int)error_code, (unsigned int)line_num, p_file_name);
  } else {
    printf("$DBG,0,%u,%u\n\r", (unsigned int)error_code, (unsigned int)line_num);
  }

#if LEDS_NUMBER > 0
  LEDS_OFF(LEDS_MASK);
  while (0) {
    nrf_delay_ms(300);
    LEDS_INVERT(LEDS_MASK);
  }
#endif
}

void app_error_handler_bare(uint32_t error_code) {
  
  if (error_code == NRF_SUCCESS) return;

#if (NRF_LOG_USES_RTT==1)
  log_rtt_printf(0, "Erreur: 0x%x\n", error_code); 
#endif

}

void app_error_fault_handler(uint32_t id, uint32_t pc, uint32_t info) {
  
}

void app_error_save_and_stop(uint32_t id, uint32_t pc, uint32_t info) {
  
}

/**@brief Callback function for asserts in the SoftDevice.
 *
 * @details This function will be called in case of an assert in the SoftDevice.
 *
 * @warning This handler is an example only and does not fit a final product. You need to analyze
 *          how your product is supposed to react in case of Assert.
 * @warning On assert from the SoftDevice, the system can only recover on reset.
 *
 * @param[in]   line_num   Line number of the failing ASSERT call.
 * @param[in]   file_name  File name of the failing ASSERT call.
 */
void assert_nrf_callback(uint16_t line_num, const uint8_t * p_file_name)
{
    app_error_handler(DEAD_BEEF, line_num, p_file_name);
}


/**@brief Start advertising.
 */
static void advertising_start(void)
{
    uint32_t err_code;

    err_code = sd_ble_gap_adv_start(&m_adv_params);
    if (err_code != NRF_SUCCESS && err_code != NRF_ERROR_INVALID_STATE)
    {
        APP_ERROR_HANDLER(err_code);
    }
    
}


/**@brief Start receiving the ANT HRM data.
 */
static void ant_hrm_rx_start(void * p_context)
{
    uint32_t err_code = ant_hrm_disp_open(&m_ant_hrm);
    APP_ERROR_CHECK(err_code);
}

/**@brief Start receiving the ANT HRM data.
 */
static void ant_bsc_rx_start(void * p_context)
{
    uint32_t err_code = ant_bsc_disp_open(&m_ant_bsc);
    APP_ERROR_CHECK(err_code);
}


/**@brief Attempt to both open the ant channel and start ble advertising.
*/
static void ant_and_adv_start(void)
{
    advertising_start();
    //APP_ERROR_CHECK(ble_advertising_start(BLE_ADV_MODE_FAST));

    ant_hrm_rx_start(NULL);
	  ant_bsc_rx_start(NULL);
}


#ifdef USE_TUNES
static void pwm_ready_callback(uint32_t pwm_id)    // PWM callback function
{
    pwm_ready = 1;
}



static void sec_tune(void * p_context) {
   play_tune();
}
#endif




uint8_t encode (uint8_t byte) {
  
  uint32_t err_code;
  
   switch (byte) {
     case '$':
       status_byte = 0;
       marque_byte = 1;
       memset(read_byte, 0, RB_SIZE);
     
       break;
     case '\r':
		 case '\n':
		 case '\0':
			 if (status_byte < 1 || marque_byte == 0) return 0;
       LOG("%s\n\r", read_byte);
#ifdef DEBUG
		   if (nus_isinit) {
				 err_code = ble_nus_string_send(&m_nus, read_byte, status_byte);
				 if (err_code != NRF_ERROR_INVALID_STATE)
				 {
					 APP_ERROR_CHECK(err_code);
				 }     
       }			 
#endif			 
#ifdef USE_TUNES
		   if (read_byte[0]=='T' && read_byte[1]=='U') {
				 switch(read_byte[2]) {
				   case '0':
             whichTune = 0;
						 err_code = app_timer_start(m_sec_play, PLAY_DELAY, NULL);
						 break;
					 case '1':
             whichTune = 1;
             err_code = app_timer_start(m_sec_play, PLAY_DELAY, NULL);
						 break;
				 }
         
         status_byte = 0;
         marque_byte = 0;
				 return 0;
			 }
#endif
       marque_byte = 0;
       status_byte = 0;
       return 1;
       
     default:
       if (status_byte < RB_SIZE - 10 && marque_byte == 1) {
         read_byte[status_byte] = byte;
         status_byte++;
       } else {
         status_byte = 0;
         marque_byte = 0;
       }
       break;
   }
  
   return 0;
}


void transmit_order () {
   // pass-through
   uint8_t i;
  
   LOG("Envoi aux lunettes: \"%c%c%c%c%c%c%c%c\"\n\r", 
                   read_byte[0], read_byte[1],
                   read_byte[2], read_byte[3],
                   read_byte[4], read_byte[5],
                   read_byte[6], read_byte[7]);
  

   memset(glasses_payload, 0, 8);
  
   for (i=0; i<4; i++) {
     if (read_byte[2*i] <= '9') {
       glasses_payload[i] = read_byte[2*i] - '0';
     } else if (read_byte[2*i] <= 'F') {
       // lettres majuscules
       glasses_payload[i] = 10 + read_byte[2*i] - 'A';
     } else {
       // lettres minuscules
       glasses_payload[i] = 10 + read_byte[2*i] - 'a';
     }
     
     glasses_payload[i] *= 16;
     
     if (read_byte[2*i+1] <= '9') {
       glasses_payload[i] += read_byte[2*i+1] - '0';
     } else if (read_byte[2*i+1] <= 'F') {
       // lettres majuscules
       glasses_payload[i] += 10 + read_byte[2*i+1] - 'A';
     } else {
       // lettres minuscules
       glasses_payload[i] += 10 + read_byte[2*i+1] - 'a';
     }
     
   }
}

void uart_evt_handle(app_uart_evt_t * p_event)
{
    uint8_t uart_byte = 0;
  
    if (p_event->evt_type == APP_UART_DATA_READY)
    {
      
#ifdef DEBUG
			if (nus_isinit)
			  ble_nus_string_send(&m_nus, "Rd", 2);
#endif
      
      // get data
      while(app_uart_get(&uart_byte) != NRF_SUCCESS) {;}
	  
	  
      if (encode(uart_byte)) {
        transmit_order ();
      }

    }
    
  
    if (1) {
      if (p_event->evt_type == APP_UART_COMMUNICATION_ERROR)
      {
        APP_ERROR_CHECK(p_event->data.error_communication);
      }
      else if (p_event->evt_type == APP_UART_FIFO_ERROR)
      {
        APP_ERROR_CHECK(p_event->data.error_code);
      }
      else if (p_event->evt_type == APP_UART_TX_EMPTY)
      {
        
      }
    }
}




static void hrm_connect(void * p_context)
{
  uint32_t err_code = NRF_SUCCESS;
  
  err_code = ant_hrm_disp_open(&m_ant_hrm);
  APP_ERROR_CHECK(err_code);
}

static void cad_connect(void * p_context)
{
  uint32_t err_code = NRF_SUCCESS;
  
  err_code = ant_bsc_disp_open(&m_ant_bsc);
  APP_ERROR_CHECK(err_code);
}


void ant_evt_glasses (ant_evt_t * p_ant_evt)
{
    uint32_t err_code = NRF_SUCCESS;
    
    switch (p_ant_evt->event)
					{
							case EVENT_TX:
                  ant_glasses_tx_evt_handle(&m_ant_glasses, p_ant_evt, glasses_payload);
                  LOG("Payload: \"%x %x %x %x %x %x %x %x\"\n\r", 
                   glasses_payload[0], glasses_payload[1],
                   glasses_payload[2], glasses_payload[3],
                   glasses_payload[4], glasses_payload[5],
                   glasses_payload[6], glasses_payload[7]);
									break;
              case EVENT_RX:
									break;
							case EVENT_RX_FAIL:
									break;
							case EVENT_RX_FAIL_GO_TO_SEARCH:
									break;
							case EVENT_CHANNEL_CLOSED:
							case EVENT_RX_SEARCH_TIMEOUT:
									break;
					}
          
    APP_ERROR_CHECK(err_code);
}

void ant_evt_cad (ant_evt_t * p_ant_evt)
{
    uint32_t err_code = NRF_SUCCESS;
    
    uint16_t pusDeviceNumber = 0;
    uint8_t pucDeviceType    = 0;
    uint8_t pucTransmitType  = 0;
	

    switch (p_ant_evt->event)
					{
							case EVENT_RX:
                  if (!is_cad_init) {
                    sd_ant_channel_id_get (BSC_RX_CHANNEL_NUMBER, 
                          &pusDeviceNumber, &pucDeviceType, &pucTransmitType);
                    printf("$ANCS,0,CAD 0x%x connected\n\r", pusDeviceNumber);
                    if (pusDeviceNumber) is_cad_init = 1;
                  }
                  ant_bsc_disp_evt_handler(&m_ant_bsc, p_ant_evt);
									break;
							case EVENT_RX_FAIL:
									break;
							case EVENT_RX_FAIL_GO_TO_SEARCH:
									break;
							case EVENT_CHANNEL_CLOSED:
							case EVENT_RX_SEARCH_TIMEOUT:
                  LOG("Reconnexion CAD...\n\r");
                  is_cad_init = 0;
                  err_code = app_timer_stop(m_sec_cad);
									err_code = app_timer_start(m_sec_cad, ANT_DELAY, NULL);
									break;
					}
          
    APP_ERROR_CHECK(err_code);
}

void ant_evt_hrm (ant_evt_t * p_ant_evt)
{
    uint32_t err_code = NRF_SUCCESS;
    
    uint16_t pusDeviceNumber = 0;
    uint8_t pucDeviceType    = 0;
    uint8_t pucTransmitType  = 0;

    switch (p_ant_evt->event)
					{
							case EVENT_RX:
                  if (!is_hrm_init) {
                    sd_ant_channel_id_get (HRM_RX_CHANNEL_NUMBER, 
                          &pusDeviceNumber, &pucDeviceType, &pucTransmitType);
                    printf("$ANCS,0,HRM 0x%x connected\n\r", pusDeviceNumber);
                    if (pusDeviceNumber) is_hrm_init = 1;
                  }
                  ant_hrm_disp_evt_handler(&m_ant_hrm, p_ant_evt);
									break;
							case EVENT_RX_FAIL:
									break;
							case EVENT_RX_FAIL_GO_TO_SEARCH:
									break;
							case EVENT_CHANNEL_CLOSED:
							case EVENT_RX_SEARCH_TIMEOUT:
                  LOG("Reconnexion HRM...\n\r");
                  is_hrm_init = 0;
                  err_code = app_timer_stop(m_sec_hrm);
									err_code = app_timer_start(m_sec_hrm, ANT_DELAY, NULL);
                  break;
					}
          
    APP_ERROR_CHECK(err_code);
}



/**@brief Function for dispatching a ANT stack event to all modules with a ANT stack event handler.
 *
 * @details This function is called from the ANT Stack event interrupt handler after a ANT stack
 *          event has been received.
 *
 * @param[in] p_ant_evt  ANT stack event.
 */
void ant_evt_dispatch(ant_evt_t * p_ant_evt)
{
    
	  switch(p_ant_evt->channel) {
			case HRM_RX_CHANNEL_NUMBER:
        nrf_gpio_pin_toggle(LED_2);
				ant_evt_hrm (p_ant_evt);
				break;
      
			case BSC_RX_CHANNEL_NUMBER:
        nrf_gpio_pin_toggle(LED_2);
				ant_evt_cad (p_ant_evt);
				break;
      
      case GLASSES_TX_CHANNEL_NUMBER:
        ant_evt_glasses (p_ant_evt);
        break;
      
			default:
				break;
		}

}



__STATIC_INLINE float calculate_speed(int32_t rev_cnt, int32_t evt_time)
{
    static float computed_speed   = 0;

    if (rev_cnt != previous_s_evt_cnt)
    {
        accumulated_s_rev_cnt  += rev_cnt - previous_s_evt_cnt;
        accumulated_s_evt_time += evt_time - previous_s_evt_time;

        /* Process rollover */
        if (previous_s_evt_cnt > rev_cnt)
        {
            accumulated_s_rev_cnt += UINT16_MAX + 1;
        }
        if (previous_s_evt_time > evt_time)
        {
            accumulated_s_evt_time += UINT16_MAX + 1;
        }

        previous_s_evt_cnt  = rev_cnt;
        previous_s_evt_time = evt_time;

        computed_speed   = SPEED_COEFFICIENT * (accumulated_s_rev_cnt  - prev_s_accumulated_rev_cnt);
        computed_speed   /= (accumulated_s_evt_time - prev_s_accumulated_evt_time);
        computed_speed   /= BSC_SPEED_UNIT_FACTOR;

        prev_s_accumulated_rev_cnt  = accumulated_s_rev_cnt;
        prev_s_accumulated_evt_time = accumulated_s_evt_time;
    }

    return (float)computed_speed;
}

static uint32_t calculate_cadence(int32_t rev_cnt, int32_t evt_time)
{
    static uint32_t computed_cadence = 0;

    if (rev_cnt != previous_c_evt_cnt)
    {
        accumulated_c_rev_cnt  += rev_cnt - previous_c_evt_cnt;
        accumulated_c_evt_time += evt_time - previous_c_evt_time;

        /* Process rollover */
        if (previous_c_evt_cnt > rev_cnt)
        {
            accumulated_c_rev_cnt += UINT16_MAX + 1;
        }
        if (previous_c_evt_time > evt_time)
        {
            accumulated_c_evt_time += UINT16_MAX + 1;
        }

        previous_c_evt_cnt  = rev_cnt;
        previous_c_evt_time = evt_time;

        computed_cadence = CADENCE_COEFFICIENT *
                           (accumulated_c_rev_cnt  - prev_c_accumulated_rev_cnt) /
                           (accumulated_c_evt_time - prev_c_accumulated_evt_time);

        prev_c_accumulated_rev_cnt  = accumulated_c_rev_cnt;
        prev_c_accumulated_evt_time = accumulated_c_evt_time;
    }

    return (uint32_t) computed_cadence;
}




void ant_bsc_evt_handler(ant_bsc_profile_t * p_profile, ant_bsc_evt_t event)
{
    unsigned int _cadence;
	  float _speed;
  
  
    switch (event)
    {
        case ANT_BSC_PAGE_0_UPDATED:
            /* fall through */
        case ANT_BSC_PAGE_1_UPDATED:
            /* fall through */
        case ANT_BSC_PAGE_2_UPDATED:
            /* fall through */
        case ANT_BSC_PAGE_3_UPDATED:
            /* fall through */
        case ANT_BSC_PAGE_4_UPDATED:
            /* fall through */
        case ANT_BSC_PAGE_5_UPDATED:
            /* Log computed value */
            break;

        case ANT_BSC_COMB_PAGE_0_UPDATED:
          
            
            _speed = calculate_speed(p_profile->BSC_PROFILE_speed_rev_count, p_profile->BSC_PROFILE_speed_event_time);
        
            _cadence = calculate_cadence(p_profile->BSC_PROFILE_cadence_rev_count, p_profile->BSC_PROFILE_cadence_event_time);
                         
			      printf("$CAD,%u,%u\n\r", (unsigned int)_cadence, (unsigned int)(100.*_speed));
				
#if (NRF_LOG_USES_RTT==1)
        log_rtt_printf(0, "Evenement BSC speed=%u cad=%u\n", (unsigned int)_cadence, (unsigned int)(100.*_speed));
#endif
            break;

        default:
            break;
    }
}




/**@brief Handle received ANT+ HRM data.
 * 
 * @param[in]   p_profile       Pointer to the ANT+ HRM profile instance.
 * @param[in]   event           Event related with ANT+ HRM Display profile. 
 */
static void ant_hrm_evt_handler(ant_hrm_profile_t * p_profile, ant_hrm_evt_t event)
{
	  uint32_t            err_code;
    static uint32_t     s_previous_beat_count  = 0;    // Heart beat count from previously received page
    uint16_t            beat_time              = p_profile->page_0.beat_time;
    uint32_t            beat_count             = p_profile->page_0.beat_count;
    uint32_t            computed_heart_rate    = p_profile->page_0.computed_heart_rate;
	  uint16_t rrInterval;
    uint16_t rrInterval_ms;
  
    switch (event)
    {
        case ANT_HRM_PAGE_0_UPDATED:
            /* fall through */
        case ANT_HRM_PAGE_1_UPDATED:
            /* fall through */
        case ANT_HRM_PAGE_2_UPDATED:
            /* fall through */
        case ANT_HRM_PAGE_3_UPDATED:
            break;
        case ANT_HRM_PAGE_4_UPDATED:
					
				    // Notify the received heart rate measurement
						err_code = ble_hrs_heart_rate_measurement_send(&m_hrs, computed_heart_rate);
						if (
								(err_code != NRF_SUCCESS)
								&&
								(err_code != NRF_ERROR_INVALID_STATE)
								&&
								(err_code != NRF_ERROR_NO_MEM)
								&&
								(err_code != BLE_ERROR_GATTS_SYS_ATTR_MISSING)
						)
						{
								APP_ERROR_HANDLER(err_code);
						}
          
#if (NRF_LOG_USES_RTT==1)
            log_rtt_printf(0, "Evenement HR BPM=%u\n", (unsigned int)computed_heart_rate);
#endif
        
            // Ensure that there is only one beat between time intervals.
            if ((beat_count - s_previous_beat_count) == 1)
            {
                uint16_t prev_beat = p_profile->page_4.prev_beat;
							
							  rrInterval = (beat_time - prev_beat);
                rrInterval_ms = rrInterval * 1000. / 1024.;
							
							  printf("$HRM,%u,%u\n\r",
                      (unsigned int)computed_heart_rate,
                      (unsigned int)rrInterval_ms);
                
                // Subtracting the event time gives the R-R interval
                //ble_hrs_rr_interval_add(&m_hrs, beat_time - prev_beat);
#if (NRF_LOG_USES_RTT==1)
                log_rtt_printf(0, "Evenement HR RR=%u\n", (unsigned int)rrInterval_ms);
#endif
											
								ble_hrs_rr_interval_add(&m_hrs, rrInterval_ms);
								if ((err_code != NRF_SUCCESS)
										&&
										(err_code != NRF_ERROR_INVALID_STATE)
										&&
										(err_code != NRF_ERROR_NO_MEM)
										&&
										(err_code != BLE_ERROR_GATTS_SYS_ATTR_MISSING))
								{
										APP_ERROR_HANDLER(err_code);
								}
            }
            
            s_previous_beat_count = beat_count;
            break;

        default:
            break;
    }


}




/**@brief Timer initialization.
 *
 * @details Initializes the timer module. This creates and starts application timers.
 */
static void timers_init(void)
{
	  uint32_t err_code;
	
    // Initialize timer module
    APP_TIMER_INIT(APP_TIMER_PRESCALER, APP_TIMER_OP_QUEUE_SIZE, NULL);
	
	  err_code = app_timer_create(&m_sec_hrm, APP_TIMER_MODE_SINGLE_SHOT, ant_hrm_rx_start);
    APP_ERROR_CHECK(err_code);
  
    err_code = app_timer_create(&m_sec_cad, APP_TIMER_MODE_SINGLE_SHOT, ant_bsc_rx_start);
    APP_ERROR_CHECK(err_code);
  
#ifdef USE_TUNES
    err_code = app_timer_create(&m_sec_play, APP_TIMER_MODE_SINGLE_SHOT, play_mario);
    APP_ERROR_CHECK(err_code);
  
    err_code = app_timer_create(&m_sec_tune, APP_TIMER_MODE_SINGLE_SHOT, sec_tune);
    APP_ERROR_CHECK(err_code);
#endif
}



#ifdef BLE_DFU_APP_SUPPORT
/**@brief Function for stopping advertising.
 */
static void advertising_stop(void)
{
    uint32_t err_code;

    err_code = sd_ble_gap_adv_stop();
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for loading application-specific context after establishing a secure connection.
 *
 * @details This function will load the application context and check if the ATT table is marked as
 *          changed. If the ATT table is marked as changed, a Service Changed Indication
 *          is sent to the peer if the Service Changed CCCD is set to indicate.
 *
 * @param[in] p_handle The Device Manager handle that identifies the connection for which the context
 *                     should be loaded.
 */
static void app_context_load(dm_handle_t const * p_handle)
{
    uint32_t                 err_code;
    static uint32_t          context_data;
    dm_application_context_t context;

    context.len    = sizeof(context_data);
    context.p_data = (uint8_t *)&context_data;

    err_code = dm_application_context_get(p_handle, &context);
    if (err_code == NRF_SUCCESS)
    {
        // Send Service Changed Indication if ATT table has changed.
        if ((context_data & (DFU_APP_ATT_TABLE_CHANGED << DFU_APP_ATT_TABLE_POS)) != 0)
        {
            err_code = sd_ble_gatts_service_changed(m_conn_handle, APP_SERVICE_HANDLE_START, BLE_HANDLE_MAX);
            if ((err_code != NRF_SUCCESS) &&
                (err_code != BLE_ERROR_INVALID_CONN_HANDLE) &&
                (err_code != NRF_ERROR_INVALID_STATE) &&
                (err_code != BLE_ERROR_NO_TX_PACKETS) &&
                (err_code != NRF_ERROR_BUSY) &&
                (err_code != BLE_ERROR_GATTS_SYS_ATTR_MISSING))
            {
                APP_ERROR_HANDLER(err_code);
            }
        }

        err_code = dm_application_context_delete(p_handle);
        APP_ERROR_CHECK(err_code);
    }
    else if (err_code == DM_NO_APP_CONTEXT)
    {
        // No context available. Ignore.
    }
    else
    {
        APP_ERROR_HANDLER(err_code);
    }
}


/** @snippet [DFU BLE Reset prepare] */
/**@brief Function for preparing for system reset.
 *
 * @details This function implements @ref dfu_app_reset_prepare_t. It will be called by
 *          @ref dfu_app_handler.c before entering the bootloader/DFU.
 *          This allows the current running application to shut down gracefully.
 */
static void reset_prepare(void)
{
    uint32_t err_code;

    if (m_conn_handle != BLE_CONN_HANDLE_INVALID)
    {
        // Disconnect from peer.
        err_code = sd_ble_gap_disconnect(m_conn_handle, BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
        APP_ERROR_CHECK(err_code);
    }
    else
    {
        // If not connected, the device will be advertising. Hence stop the advertising.
        advertising_stop();
    }

    err_code = ble_conn_params_stop();
    APP_ERROR_CHECK(err_code);
	
	//sd_power_gpregret_set(BOOTLOADER_DFU_START);

    nrf_delay_ms(500);
}
/** @snippet [DFU BLE Reset prepare] */
#endif // BLE_DFU_APP_SUPPORT




/**@brief GAP initialization.
 *
 * @details This function sets up all the necessary GAP (Generic Access Profile) parameters of the
 *          device including the device name, appearance, and the preferred connection parameters.
 */
static void gap_params_init(void)
{
    uint32_t                err_code;
    ble_gap_conn_params_t   gap_conn_params;
    ble_gap_conn_sec_mode_t sec_mode;

    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&sec_mode);
    
    err_code = sd_ble_gap_device_name_set(&sec_mode,
                                          (const uint8_t *)DEVICE_NAME, 
                                          strlen(DEVICE_NAME));
    APP_ERROR_CHECK(err_code);

    err_code = sd_ble_gap_appearance_set(BLE_APPEARANCE_HEART_RATE_SENSOR_HEART_RATE_BELT);
    APP_ERROR_CHECK(err_code);

    memset(&gap_conn_params, 0, sizeof(gap_conn_params));

    gap_conn_params.min_conn_interval = MIN_CONN_INTERVAL;
    gap_conn_params.max_conn_interval = MAX_CONN_INTERVAL;
    gap_conn_params.slave_latency     = SLAVE_LATENCY;
    gap_conn_params.conn_sup_timeout  = CONN_SUP_TIMEOUT;

    err_code = sd_ble_gap_ppcp_set(&gap_conn_params);
    APP_ERROR_CHECK(err_code);
}




/**@brief Connection Parameters module error handler.
 *
 * @param[in]   nrf_error   Error code containing information about what went wrong.
 */
static void conn_params_error_handler(uint32_t nrf_error)
{
    APP_ERROR_HANDLER(nrf_error);
}




/**@brief Connection Parameters Module handler.
 *
 * @details This function will be called for all events in the Connection Parameters Module which
 *          are passed to the application.
 *          @note All this function does is to disconnect. This could have been done by simply
 *                setting the disconnect_on_fail config parameter, but instead we use the event
 *                handler mechanism to demonstrate its use.
 *
 * @param[in]   p_evt   Event received from the Connection Parameters Module.
 */
static void on_conn_params_evt(ble_conn_params_evt_t * p_evt)
{
    uint32_t err_code;

    switch (p_evt->evt_type)
    {
        case BLE_CONN_PARAMS_EVT_FAILED:
            err_code = sd_ble_gap_disconnect(m_conn_handle, BLE_HCI_CONN_INTERVAL_UNACCEPTABLE);
            APP_ERROR_CHECK(err_code);
            break;

        default:
            // No implementation needed.
            break;
    }
}



/**@brief Initialize the Connection Parameters module.
 */
static void conn_params_init(void)
{
    uint32_t               err_code;
    ble_conn_params_init_t cp_init;

    memset(&cp_init, 0, sizeof(cp_init));

    cp_init.p_conn_params                  = NULL;
    cp_init.first_conn_params_update_delay = FIRST_CONN_PARAMS_UPDATE_DELAY;
    cp_init.next_conn_params_update_delay  = NEXT_CONN_PARAMS_UPDATE_DELAY;
    cp_init.max_conn_params_update_count   = MAX_CONN_PARAMS_UPDATE_COUNT;
    cp_init.start_on_notify_cccd_handle    = m_hrs.hrm_handles.cccd_handle;
    cp_init.disconnect_on_fail             = false;
    cp_init.evt_handler                    = on_conn_params_evt;
    cp_init.error_handler                  = conn_params_error_handler;
    
    err_code = ble_conn_params_init(&cp_init);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for handling the Device Manager events.
 *
 * @param[in]   p_evt   Data associated to the device manager event.
 */
static uint32_t device_manager_evt_handler(dm_handle_t const    * p_handle,
                                           dm_event_t const     * p_event,
                                           ret_code_t             event_result)
{
    //APP_ERROR_CHECK(event_result);

    switch (p_event->event_id)
    {
        case DM_EVT_CONNECTION:
            break;

        case DM_EVT_LINK_SECURED:
        
#ifdef BLE_DFU_APP_SUPPORT
            app_context_load(p_handle);
#endif
            break; 

        default:
            break;

    }
    return NRF_SUCCESS;
}




/**@brief Function for the Device Manager initialization.
 */
static void device_manager_init(void)
{
    uint32_t                err_code;
    dm_init_param_t        init_param;
    dm_application_param_t  register_param;
    
    // Initialize persistent storage module.
    err_code = pstorage_init();
    APP_ERROR_CHECK(err_code);

    err_code = dm_init(&init_param);
    APP_ERROR_CHECK(err_code);

    memset(&register_param.sec_param, 0, sizeof(ble_gap_sec_params_t));
    
    register_param.sec_param.bond         = SEC_PARAM_BOND;
    register_param.sec_param.mitm         = SEC_PARAM_MITM;
    register_param.sec_param.io_caps      = SEC_PARAM_IO_CAPABILITIES;
    register_param.sec_param.oob          = SEC_PARAM_OOB;
    register_param.sec_param.min_key_size = SEC_PARAM_MIN_KEY_SIZE;
    register_param.sec_param.max_key_size = SEC_PARAM_MAX_KEY_SIZE;
    register_param.evt_handler            = device_manager_evt_handler;
    register_param.service_type           = DM_PROTOCOL_CNTXT_GATT_SRVR_ID;

    err_code = dm_register(&m_app_handle, &register_param);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for handling advertising events.
 *
 * @details This function is called for advertising events that are passed to the application.
 *
 * @param[in] ble_adv_evt  Advertising event.
 */
static void on_adv_evt(ble_adv_evt_t ble_adv_evt)
{
    uint32_t err_code;

    switch (ble_adv_evt)
    {
        case BLE_ADV_EVT_DIRECTED:
            //err_code = bsp_indication_set(BSP_INDICATE_ADVERTISING_DIRECTED);
            //APP_ERROR_CHECK(err_code);
            break;

        case BLE_ADV_EVT_FAST:
            //err_code = bsp_indication_set(BSP_INDICATE_ADVERTISING);
            //APP_ERROR_CHECK(err_code);
            break;

        case BLE_ADV_EVT_FAST_WHITELIST:
            //err_code = bsp_indication_set(BSP_INDICATE_ADVERTISING_WHITELIST);
            //APP_ERROR_CHECK(err_code);
            break;

        case BLE_ADV_EVT_SLOW:
            //err_code = bsp_indication_set(BSP_INDICATE_ADVERTISING_SLOW);
            //APP_ERROR_CHECK(err_code);
            break;

        case BLE_ADV_EVT_IDLE:
            //sleep_mode
            break;

        case BLE_ADV_EVT_WHITELIST_REQUEST:
        {
            ble_gap_whitelist_t whitelist;
            ble_gap_addr_t    * p_whitelist_addr[BLE_GAP_WHITELIST_ADDR_MAX_COUNT];
            ble_gap_irk_t     * p_whitelist_irk[BLE_GAP_WHITELIST_IRK_MAX_COUNT];

            whitelist.addr_count = BLE_GAP_WHITELIST_ADDR_MAX_COUNT;
            whitelist.irk_count  = BLE_GAP_WHITELIST_IRK_MAX_COUNT;
            whitelist.pp_addrs   = p_whitelist_addr;
            whitelist.pp_irks    = p_whitelist_irk;

            err_code = dm_whitelist_create(&m_app_handle, &whitelist);
            APP_ERROR_CHECK(err_code);

            err_code = ble_advertising_whitelist_reply(&whitelist);
            APP_ERROR_CHECK(err_code);
            break;
        }

        default:
            break;
    }
}

/**@brief Application's Stack BLE event handler.
 *
 * @param[in]   p_ble_evt   Bluetooth stack event.
 */
static void on_ble_evt(ble_evt_t * p_ble_evt)
{
    uint32_t err_code = NRF_SUCCESS;

    switch (p_ble_evt->header.evt_id)
    {
        case BLE_GAP_EVT_CONNECTED:
            m_conn_handle = p_ble_evt->evt.gap_evt.conn_handle;
            break;
            
        case BLE_GAP_EVT_DISCONNECTED:
            
				    advertising_start();
            // Note: Bonding information will be stored, advertising will be restarted and the
            //       ANT channel will be reopened when ANT event CHANNEL_CLOSED is received.
            break;

        case BLE_GAP_EVT_SEC_PARAMS_REQUEST:
            err_code = sd_ble_gap_sec_params_reply(m_conn_handle, 
                                                   BLE_GAP_SEC_STATUS_PAIRING_NOT_SUPP, 
                                                   NULL,
                                                   NULL);
            APP_ERROR_CHECK(err_code);
            break;


        case BLE_GAP_EVT_TIMEOUT:
            
            break;

        case BLE_GATTS_EVT_SYS_ATTR_MISSING:
                err_code = sd_ble_gatts_sys_attr_set(m_conn_handle, NULL, 0,
                                                     BLE_GATTS_SYS_ATTR_FLAG_SYS_SRVCS | BLE_GATTS_SYS_ATTR_FLAG_USR_SRVCS);
                APP_ERROR_CHECK(err_code);
                break;

            
        default:
            // No implementation needed.
            break;
    }
}




/**@brief Function for handling bsp events.
 */
void bsp_evt_handler(bsp_event_t evt)
{
    uint32_t err_code = NRF_SUCCESS;
  
#if (NRF_LOG_USES_RTT==1)
  SEGGER_RTT_printf(0, "Event: %u\n", evt); 
#endif

  
    switch (evt)
    {
        case BSP_EVENT_KEY_0:
            printf("$BTN,0\n\r");
            break;
        case BSP_EVENT_KEY_1:
            printf("$BTN,1\n\r");
            break;
        case BSP_EVENT_KEY_2:
            printf("$BTN,2\n\r");
            break;
        case BSP_EVENT_KEY_3:
            whichTune = 0;
						err_code = app_timer_start(m_sec_play, PLAY_DELAY, NULL);
            break;
        case BSP_EVENT_KEY_4:
            whichTune = 0;
						err_code = app_timer_start(m_sec_play, PLAY_DELAY, NULL);
            break;
        default:
            return; // no implementation needed
    }

}



#ifdef USE_TUNES
static void pwm_start(uint32_t period_us) {
  
    /* 2-channel PWM, 200Hz, output on DK LED pins. */
    app_pwm_config_t pwm1_cfg = APP_PWM_DEFAULT_CONFIG_2CH(period_us, 22, 24);
    
    /* Switch the polarity of the second channel. */
    pwm1_cfg.pin_polarity[1] = APP_PWM_POLARITY_ACTIVE_HIGH;

    /* Initialize and enable PWM. */
    uint32_t err_code = app_pwm_init(&PWM1,&pwm1_cfg, pwm_ready_callback);
    APP_ERROR_CHECK(err_code);
    app_pwm_enable(&PWM1);
  
    pwm_ready = 0;
    while (app_pwm_channel_duty_set(&PWM1, 0, 50) == NRF_ERROR_BUSY) {
      nrf_delay_ms(1);
    }
    while (!pwm_ready) {
      nrf_delay_ms(1);
    }
    APP_ERROR_CHECK(app_pwm_channel_duty_set(&PWM1, 1, 50));
    
}


static void pwm_stop() {
    app_pwm_disable(&PWM1);
    uint32_t err_code = app_pwm_uninit(&PWM1);
    APP_ERROR_CHECK(err_code);
}

void play_mario(void * p_context) {
   
   iTune = NOTES_NB;
#ifdef DEBUG
	 ble_nus_string_send(&m_nus, "Musique !", 9);
#endif
	 if (isTunePlaying) {
		 isTunePlaying = 0;
		 pwm_stop();
		 nrf_delay_ms(10);
	 }
#ifndef DEBUG
   play_tune ();
#endif
}


static void play_tune (void) {
  
  uint32_t err_code, delay, period_us;
  
  if (iTune > 1) {
    // indice non nul
    if (isTunePlaying) {
      // on vient de jouer une note
      // on joue un blanc
      isTunePlaying = 0;
      pwm_stop();
      delay = APP_TIMER_TICKS(1000 / tempo[NOTES_NB - iTune], APP_TIMER_PRESCALER);
      err_code = app_timer_start(m_sec_tune, delay, NULL);
      APP_ERROR_CHECK(err_code);
      
      iTune--;
    } else {
      // on vient de jouer un blanc ou on commence
      if (iTune - 1 > 1) {
        // on relance
        if (melody[NOTES_NB - iTune] > 0) {
          period_us = 1000000 / melody[NOTES_NB - iTune];
          pwm_start(period_us);
        }
        delay = APP_TIMER_TICKS(1300 / tempo[NOTES_NB - iTune], APP_TIMER_PRESCALER);
        err_code = app_timer_start(m_sec_tune, delay, NULL);
        APP_ERROR_CHECK(err_code);
      }
      
      isTunePlaying = 1;
    }
    
  }
  
}
#endif

/**@brief Dispatches a stack event to all modules with a stack BLE event handler.
 *
 * @details This function is called from the Stack event interrupt handler after a stack BLE
 *          event has been received.
 *
 * @param[in]   p_ble_evt   Stack Bluetooth event.
 */
static void ble_evt_dispatch(ble_evt_t * p_ble_evt)
{

    dm_ble_evt_handler(p_ble_evt);
	
#ifdef BLE_DFU_APP_SUPPORT
    /** @snippet [Propagating BLE Stack events to DFU Service] */
    ble_dfu_on_ble_evt(&m_dfus, p_ble_evt);
    /** @snippet [Propagating BLE Stack events to DFU Service] */
#endif // BLE_DFU_APP_SUPPORT

    ble_nus_on_ble_evt(&m_nus, p_ble_evt);
    ble_hrs_on_ble_evt(&m_hrs, p_ble_evt);
    ble_conn_params_on_ble_evt(p_ble_evt);
    on_ble_evt(p_ble_evt);
}




/**@brief Function for dispatching a system event to interested modules.
 *
 * @details This function is called from the System event interrupt handler after a system
 *          event has been received.
 *
 * @param[in]   sys_evt   System stack event.
 */
static void sys_evt_dispatch(uint32_t sys_evt)
{
    uint32_t err_code;
    uint32_t count;

    pstorage_sys_event_handler(sys_evt);

    // Verify if BLE bond data storage access is in progress or not.    
    if (m_app_initialized == true)
    {
        err_code = pstorage_access_status_get(&count);
        if ((err_code == NRF_SUCCESS) && (count == 0))
        {        
            ant_and_adv_start();
        }
    }
    else
    {
        m_app_initialized = true;
    }
}


/**@brief BLE + ANT stack initialization.
 *
 * @details Initializes the SoftDevice and the stack event interrupt.
 */
static void ble_ant_stack_init(void)
{
    uint32_t                err_code;
    
    nrf_clock_lf_cfg_t clock_lf_cfg = NRF_CLOCK_LFCLKSRC;

    // Initialize SoftDevice
    SOFTDEVICE_HANDLER_INIT(&clock_lf_cfg, NULL);
    
    // Initialize BLE stack
    ble_enable_params_t ble_enable_params;
    err_code = softdevice_enable_get_default_config(CENTRAL_LINK_COUNT,
                                                    PERIPHERAL_LINK_COUNT,
                                                    &ble_enable_params);
    APP_ERROR_CHECK(err_code);

    ble_enable_params.gatts_enable_params.service_changed = IS_SRVC_CHANGED_CHARACT_PRESENT;

    //Check the ram settings against the used number of links
    CHECK_RAM_START_ADDR(CENTRAL_LINK_COUNT,PERIPHERAL_LINK_COUNT);

    err_code = softdevice_enable(&ble_enable_params);
    APP_ERROR_CHECK(err_code);

    // Subscribe for BLE events.
    err_code = softdevice_ble_evt_handler_set(ble_evt_dispatch);
    APP_ERROR_CHECK(err_code);
        
    // Subscribe for ANT events.
    err_code = softdevice_ant_evt_handler_set(ant_evt_dispatch);
    APP_ERROR_CHECK(err_code);

    err_code = ant_stack_static_config();
    APP_ERROR_CHECK(err_code);

    err_code = ant_plus_key_set(ANTPLUS_NETWORK_NUMBER);
    APP_ERROR_CHECK(err_code);
	// or
	  //err_code = sd_ant_network_address_set(ANTPLUS_NETWORK_NUMBER, m_network_key);
    //APP_ERROR_CHECK(err_code);
		
    err_code = ant_hrm_disp_init(&m_ant_hrm,
                                 HRM_DISP_CHANNEL_CONFIG(m_ant_hrm),
                                 ant_hrm_evt_handler);
    APP_ERROR_CHECK(err_code);
		
		err_code = ant_bsc_disp_init(&m_ant_bsc,
                                 BSC_DISP_CHANNEL_CONFIG(m_ant_bsc),
                                 BSC_DISP_PROFILE_CONFIG(m_ant_bsc));
    APP_ERROR_CHECK(err_code);
	
	// GLASSES
    err_code = ant_glasses_init(&m_ant_glasses, &ant_tx_channel_config);
    APP_ERROR_CHECK(err_code);

    err_code = ant_glasses_open(&m_ant_glasses);
    APP_ERROR_CHECK(err_code);

    // Register with the SoftDevice handler module for BLE events.
    err_code = softdevice_sys_evt_handler_set(sys_evt_dispatch);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for handling the data from the Nordic UART Service.
 *
 * @details This function will process the data received from the Nordic UART BLE Service and send
 *          it to the UART module.
 *
 * @param[in] p_nus    Nordic UART Service structure.
 * @param[in] p_data   Data to be send to UART module.
 * @param[in] length   Length of the data.
 */
/**@snippet [Handling the data received over BLE] */
static void nus_data_handler(ble_nus_t * p_nus, uint8_t * p_data, uint16_t length)
{
  
#if LEDS_NUMBER > 0
  LEDS_INVERT(LEDS_MASK);
#endif
  
  for (uint32_t i = 0; i < length; i++)
    {
      encode (p_data[i]);
			while(app_uart_put(p_data[i]) != NRF_SUCCESS);
    }
		encode ('\n');
    while(app_uart_put('\n') != NRF_SUCCESS);

}



/**@brief Initialize services that will be used by the application.
 *
 * @details Initialize the Heart Rate and Device Information services.
 */
static void services_init(void)
{
    uint32_t       err_code;
    ble_nus_init_t nus_init;
	  ble_hrs_init_t hrs_init;
    uint8_t        body_sensor_location;

    // Initialize Heart Rate Service.
    body_sensor_location = BLE_HRS_BODY_SENSOR_LOCATION_CHEST;

    memset(&hrs_init, 0, sizeof(hrs_init));

    hrs_init.evt_handler                 = NULL;
    hrs_init.is_sensor_contact_supported = false;
    hrs_init.p_body_sensor_location      = &body_sensor_location;

    // Here the sec level for the Heart Rate Service can be changed/increased.
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&hrs_init.hrs_hrm_attr_md.cccd_write_perm);
    BLE_GAP_CONN_SEC_MODE_SET_NO_ACCESS(&hrs_init.hrs_hrm_attr_md.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_NO_ACCESS(&hrs_init.hrs_hrm_attr_md.write_perm);

    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&hrs_init.hrs_bsl_attr_md.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_NO_ACCESS(&hrs_init.hrs_bsl_attr_md.write_perm);

    err_code = ble_hrs_init(&m_hrs, &hrs_init);
    APP_ERROR_CHECK(err_code);

    memset(&nus_init, 0, sizeof(nus_init));

    nus_init.data_handler = nus_data_handler;
    
    err_code = ble_nus_init(&m_nus, &nus_init);
    APP_ERROR_CHECK(err_code);
		
#ifdef BLE_DFU_APP_SUPPORT
    /** @snippet [DFU BLE Service initialization] */
    ble_dfu_init_t   dfus_init;

    // Initialize the Device Firmware Update Service.
    memset(&dfus_init, 0, sizeof(dfus_init));

    dfus_init.evt_handler   = dfu_app_on_dfu_evt;
    dfus_init.error_handler = NULL;
    dfus_init.evt_handler   = dfu_app_on_dfu_evt;
    dfus_init.revision      = DFU_REVISION;

    err_code = ble_dfu_init(&m_dfus, &dfus_init);
    APP_ERROR_CHECK(err_code);

    dfu_app_reset_prepare_set(reset_prepare);
    dfu_app_dm_appl_instance_set(m_app_handle);
    /** @snippet [DFU BLE Service initialization] */
#endif // BLE_DFU_APP_SUPPORT
}


/**@brief Advertising functionality initialization.
 *
 * @details Encodes the required advertising data and passes it to the stack.
 *          Also builds a structure to be passed to the stack when starting advertising.
 */
static void advertising_init(void)
{
    uint32_t      err_code;
    ble_advdata_t advdata;
    ble_advdata_t scanrsp;

     // Build and set advertising data
    memset(&advdata, 0, sizeof(advdata));

    advdata.name_type                = BLE_ADVDATA_FULL_NAME;
    advdata.include_appearance       = true;
    advdata.flags                    = BLE_GAP_ADV_FLAGS_LE_ONLY_LIMITED_DISC_MODE;
  
    advdata.uuids_complete.uuid_cnt = sizeof(m_adv_uuids1) / sizeof(m_adv_uuids1[0]);
    advdata.uuids_complete.p_uuids  = m_adv_uuids1;
  
    memset(&scanrsp, 0, sizeof(advdata));
  
    scanrsp.uuids_complete.uuid_cnt = sizeof(m_adv_uuids2) / sizeof(m_adv_uuids2[0]);
    scanrsp.uuids_complete.p_uuids  = m_adv_uuids2;

    // Initialise advertising parameters (used when starting advertising).
    memset(&m_adv_params, 0, sizeof(m_adv_params));

    m_adv_params.type        = BLE_GAP_ADV_TYPE_ADV_IND;
    m_adv_params.p_peer_addr = NULL;
    m_adv_params.fp          = BLE_GAP_ADV_FP_ANY;
    m_adv_params.interval    = APP_ADV_INTERVAL;
    m_adv_params.timeout     = APP_ADV_TIMEOUT_IN_SECONDS;
    
    ble_adv_modes_config_t options = {0};
    options.ble_adv_fast_enabled  = BLE_ADV_FAST_ENABLED;
    options.ble_adv_fast_interval = APP_ADV_INTERVAL;
    options.ble_adv_fast_timeout  = APP_ADV_TIMEOUT_IN_SECONDS;

    //err_code = ble_advdata_set(&advdata, NULL);
    err_code = ble_advertising_init(&advdata, &scanrsp, &options, on_adv_evt, NULL);
    APP_ERROR_CHECK(err_code);
}

/**@brief Function for initializing the UART.
 */
static void uart_init(void)
{
    uint32_t err_code;

    
    const app_uart_comm_params_t comm_params =  
    {
        RX_PIN_NUMBER, 
        TX_PIN_NUMBER, 
        RTS_PIN_NUMBER, 
        CTS_PIN_NUMBER, 
        APP_UART_FLOW_CONTROL_DISABLED, 
        false, 
        UART_BAUDRATE_BAUDRATE_Baud115200
    };

    APP_UART_FIFO_INIT(&comm_params,
                       UART_RX_BUF_SIZE,
                       UART_TX_BUF_SIZE,
                       uart_evt_handle,
                       APP_IRQ_PRIORITY_LOW,
                       err_code);

    APP_ERROR_CHECK(err_code);
}


/**@brief Function for initializing buttons and LEDs.
 *
 * @param[out] p_erase_bonds  True if the clear bonds button was pressed to wake the application up.
 */
void buttons_leds_init(bool * p_erase_bonds)
{
    nrf_gpio_cfg_output(GPIO_BUTTON);
    // pin to ground
    nrf_gpio_pin_clear(GPIO_BUTTON);
  
    // BSP_INIT_BUTTONS | BSP_INIT_LED
    uint32_t err_code = bsp_init(BSP_INIT_BUTTONS | BSP_INIT_LED,
                                 APP_TIMER_TICKS(100, APP_TIMER_PRESCALER),
                                 bsp_evt_handler);
    APP_ERROR_CHECK(err_code);

}


/**@brief Power manager.
 */
static void power_manage(void)
{
    uint32_t err_code;
    
    // Wait for events    
    err_code = sd_app_evt_wait();
    APP_ERROR_CHECK(err_code);
}

/**
 * @brief WDT events handler.
 */
void wdt_event_handler(void)
{
#if LEDS_NUMBER > 0
    LEDS_INVERT(LEDS_MASK);
#endif
}


/**@brief Application main function.
 */
int main(void)
{
    bool     erase_bonds = true;
  uint32_t count;
	uint32_t err_code;
	
	#if (NRF_LOG_USES_RTT == 1)
	  log_rtt_init();
    #endif
  
    nrf_gpio_cfg_output(LED_2);
    // pin to ground
    nrf_gpio_pin_set(LED_2);
  

	  //Configure WDT.
#if (WDT_ENABLED == 1)
    nrf_drv_wdt_config_t config = NRF_DRV_WDT_DEAFULT_CONFIG;
    err_code = nrf_drv_wdt_init(&config, wdt_event_handler);
    APP_ERROR_CHECK(err_code);
    err_code = nrf_drv_wdt_channel_alloc(&wdt_channel_id);
    APP_ERROR_CHECK(err_code);
    nrf_drv_wdt_enable();
#endif
	
    // Initialize peripherals
    timers_init();
	  uart_init();
	  buttons_leds_init(&erase_bonds);

    // Initialize S332 SoftDevice & ANT
    ble_ant_stack_init();

    //err_code = bsp_init(BSP_INIT_LED | BSP_INIT_BUTTONS, APP_TIMER_TICKS(100, APP_TIMER_PRESCALER), bsp_evt_handler);
    //APP_ERROR_CHECK(err_code);


    // Initialize Bluetooth stack parameters.
    device_manager_init();
    gap_params_init();
    services_init();
    advertising_init();
    conn_params_init();
	
    ant_and_adv_start();
    
#if (NRF_LOG_USES_RTT==1)
    log_rtt_printf(0, "Start !!\n"); 
#endif
    
    // Enter main loop.
    for (;;)
    {
        power_manage();
			
#if (WDT_ENABLED == 1)
			  // WDT feed
        nrf_drv_wdt_channel_feed(wdt_channel_id);
#endif
    }
}

/** 
 * @}
 */
