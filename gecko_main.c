/***************************************************************************//**
 * @file
 * @brief Silicon Labs BT Mesh Empty Example Project
 * This example demonstrates the bare minimum needed for a Blue Gecko BT Mesh C application.
 * The application starts unprovisioned Beaconing after boot
 *******************************************************************************
 * # License
 * <b>Copyright 2018 Silicon Laboratories Inc. www.silabs.com</b>
 *******************************************************************************
 *
 * The licensor of this software is Silicon Laboratories Inc. Your use of this
 * software is governed by the terms of Silicon Labs Master Software License
 * Agreement (MSLA) available at
 * www.silabs.com/about-us/legal/master-software-license-agreement. This
 * software is distributed to you in Source Code format and is governed by the
 * sections of the MSLA applicable to Source Code.
 *
 * author: daniel walkes
 * co-author: santhosh thummanapalli
 *
 ******************************************************************************/

/* Board headers */
#include "init_mcu.h"
#include "init_board.h"
#include "init_app.h"
#include "ble-configuration.h"
#include "board_features.h"

/* Bluetooth stack headers */
#include "bg_types.h"
#include "native_gecko.h"
#include "gatt_db.h"
#include <gecko_configuration.h>
#include "mesh_generic_model_capi_types.h"
#include "mesh_lighting_model_capi_types.h"
#include "mesh_lib.h"
#include <mesh_sizes.h>

/* Libraries containing default Gecko configuration values */
#include "em_emu.h"
#include "em_cmu.h"
#include <em_gpio.h>

/* Device initialization header */
#include "hal-config.h"

#if defined(HAL_CONFIG)
#include "bsphalconfig.h"
#else
#include "bspconfig.h"
#endif
#include "src/ble_mesh_device_type.h"

#include "src/log.h"
#include "src/display.h"
#include "src/gpio.h"

/***********************************************************************************************//**
 * @addtogroup Application
 * @{
 **************************************************************************************************/

/***********************************************************************************************//**
 * @addtogroup app
 * @{
 **************************************************************************************************/

// bluetooth stack heap
#define MAX_CONNECTIONS 2

static uint8 conn_handle = 0xFF;
#define TIMER_ID_RESTART          78
#define TIMER_ID_FACTORY_RESET    77

uint8_t bluetooth_stack_heap[DEFAULT_BLUETOOTH_HEAP(MAX_CONNECTIONS) + BTMESH_HEAP_SIZE + 1760];

// Bluetooth advertisement set configuration
//
// At minimum the following is required:
// * One advertisement set for Bluetooth LE stack (handle number 0)
// * One advertisement set for Mesh data (handle number 1)
// * One advertisement set for Mesh unprovisioned beacons (handle number 2)
// * One advertisement set for Mesh unprovisioned URI (handle number 3)
// * N advertisement sets for Mesh GATT service advertisements
// (one for each network key, handle numbers 4 .. N+3)
//
#define MAX_ADVERTISERS (4 + MESH_CFG_MAX_NETKEYS)

static gecko_bluetooth_ll_priorities linklayer_priorities = GECKO_BLUETOOTH_PRIORITIES_DEFAULT;

// bluetooth stack configuration
extern const struct bg_gattdb_def bg_gattdb_data;

int i;

// Flag for indicating DFU Reset must be performed
uint8_t boot_to_dfu = 0;

const gecko_configuration_t config =
{
  .bluetooth.max_connections = MAX_CONNECTIONS,
  .bluetooth.max_advertisers = MAX_ADVERTISERS,
  .bluetooth.heap = bluetooth_stack_heap,
  .bluetooth.heap_size = sizeof(bluetooth_stack_heap) - BTMESH_HEAP_SIZE,
  .bluetooth.sleep_clock_accuracy = 100,
  .bluetooth.linklayer_priorities = &linklayer_priorities,
  .gattdb = &bg_gattdb_data,
  .btmesh_heap_size = BTMESH_HEAP_SIZE,
#if (HAL_PA_ENABLE)
  .pa.config_enable = 1, // Set this to be a valid PA config
#if defined(FEATURE_PA_INPUT_FROM_VBAT)
  .pa.input = GECKO_RADIO_PA_INPUT_VBAT, // Configure PA input to VBAT
#else
  .pa.input = GECKO_RADIO_PA_INPUT_DCDC,
#endif // defined(FEATURE_PA_INPUT_FROM_VBAT)
#endif // (HAL_PA_ENABLE)
  .max_timers = 16,
};

void handle_gecko_event(uint32_t evt_id, struct gecko_cmd_packet *evt);
void mesh_native_bgapi_init(void);
bool mesh_bgapi_listener(struct gecko_cmd_packet *evt);
/********************************************
//global variables
 ********************************************/
/// For transaction ID
static uint8 trid = 0;
/// For indexing elements of the node (this example has only one element)
static uint16 _elem_index = 0xffff;


//modified the function from Bluetooth mesh light-switch example
static void initiate_factory_reset(void)
{

  displayPrintf(DISPLAY_ROW_ACTION,"FACTORY RESET");
  displayUpdate();
  /* if connection is open then close it before rebooting */
  if (conn_handle != 0xFF) {
    gecko_cmd_le_connection_close(conn_handle);
  }

  /* perform a factory reset by erasing PS storage. This removes all the keys and other settings
     that have been configured for this node */
  gecko_cmd_flash_ps_erase_all();
  // reboot after a small delay
  gecko_cmd_hardware_set_soft_timer(2 * 32768, TIMER_ID_FACTORY_RESET, 1);
}

static void set_device_name(bd_addr *pAddr)
{
  char name[20];
  uint16 res;
  LOG_INFO("entered set device address function");
  if(DEVICE_IS_ONOFF_PUBLISHER)
  {
	  displayPrintf(DISPLAY_ROW_NAME,"  PUBLISHER");
	  // create unique device name using the last two bytes of the Bluetooth address
	  sprintf(name, "ECEN5823Pub %02x:%02x", pAddr->addr[1], pAddr->addr[0]);
  }
  else
  {
	  displayPrintf(DISPLAY_ROW_NAME,"  SUBSCRIBER");
	  sprintf(name,"ECEN5823Sub %02x:%02x", pAddr->addr[1], pAddr->addr[0]);
  }

  // write device name to the GATT database
  res = gecko_cmd_gatt_server_write_attribute_value(gattdb_device_name, 0, strlen(name), (uint8 *)name)->result;
  if (res) {
    LOG_ERROR("gecko_cmd_gatt_server_write_attribute_value() failed- result %x\r\n", res);
  }
  displayPrintf(DISPLAY_ROW_BTADDR,"%s",name);

}


/**
 * See light switch app.c file definition
 */
void gecko_bgapi_classes_init_server_friend(void)
{
	gecko_bgapi_class_dfu_init();
	gecko_bgapi_class_system_init();
	gecko_bgapi_class_le_gap_init();
	gecko_bgapi_class_le_connection_init();
	//gecko_bgapi_class_gatt_init();
	gecko_bgapi_class_gatt_server_init();
	gecko_bgapi_class_hardware_init();
	gecko_bgapi_class_flash_init();
	gecko_bgapi_class_test_init();
	//gecko_bgapi_class_sm_init();
	//mesh_native_bgapi_init();
	gecko_bgapi_class_mesh_node_init();
	//gecko_bgapi_class_mesh_prov_init();
	gecko_bgapi_class_mesh_proxy_init();
	gecko_bgapi_class_mesh_proxy_server_init();
	//gecko_bgapi_class_mesh_proxy_client_init();
	//gecko_bgapi_class_mesh_generic_client_init();
	gecko_bgapi_class_mesh_generic_server_init();
	//gecko_bgapi_class_mesh_vendor_model_init();
	//gecko_bgapi_class_mesh_health_client_init();
	//gecko_bgapi_class_mesh_health_server_init();
	//gecko_bgapi_class_mesh_test_init();
	gecko_bgapi_class_mesh_lpn_init();
	//gecko_bgapi_class_mesh_friend_init();
}


/**
 * See main function list in soc-btmesh-switch project file
 */
void gecko_bgapi_classes_init_client_lpn(void)
{
	gecko_bgapi_class_dfu_init();
	gecko_bgapi_class_system_init();
	gecko_bgapi_class_le_gap_init();
	gecko_bgapi_class_le_connection_init();
	//gecko_bgapi_class_gatt_init();
	gecko_bgapi_class_gatt_server_init();
	gecko_bgapi_class_hardware_init();
	gecko_bgapi_class_flash_init();
	gecko_bgapi_class_test_init();
	//gecko_bgapi_class_sm_init();
	//mesh_native_bgapi_init();
	gecko_bgapi_class_mesh_node_init();
	//gecko_bgapi_class_mesh_prov_init();
	gecko_bgapi_class_mesh_proxy_init();
	gecko_bgapi_class_mesh_proxy_server_init();
	//gecko_bgapi_class_mesh_proxy_client_init();
	gecko_bgapi_class_mesh_generic_client_init();
	//gecko_bgapi_class_mesh_generic_server_init();
	//gecko_bgapi_class_mesh_vendor_model_init();
	//gecko_bgapi_class_mesh_health_client_init();
	//gecko_bgapi_class_mesh_health_server_init();
	//gecko_bgapi_class_mesh_test_init();
	gecko_bgapi_class_mesh_lpn_init();
	//gecko_bgapi_class_mesh_friend_init();

}
void gecko_main_init()
{
  // Initialize device
  initMcu();
  // Initialize board
  initBoard();
  // Initialize application
  initApp();

  // Minimize advertisement latency by allowing the advertiser to always
  // interrupt the scanner.
  linklayer_priorities.scan_max = linklayer_priorities.adv_min + 1;

  gecko_stack_init(&config);

  if( DeviceUsesClientModel() ) {
	  gecko_bgapi_classes_init_client_lpn();
  } else {
	  gecko_bgapi_classes_init_server_friend();
  }

  // Initialize coexistence interface. Parameters are taken from HAL config.
  gecko_initCoexHAL();

}

void handle_gecko_event(uint32_t evt_id, struct gecko_cmd_packet *evt)
{
	uint16_t result;


	switch (evt_id) {
    case gecko_evt_system_boot_id:

    	  // author for some part of this code is bluetooth mesh stack developers -- code from bluetooth mesh light switch example

    	 // check pushbutton state at startup. If either PB0 or PB1 is held down then do factory reset
    	      if (GPIO_PinInGet(BSP_BUTTON0_PORT, BSP_BUTTON0_PIN) == 0 || GPIO_PinInGet(BSP_BUTTON1_PORT, BSP_BUTTON1_PIN) == 0) {
    	        initiate_factory_reset();
    	        displayPrintf(DISPLAY_ROW_ACTION,"Factory Reset");
    	        displayUpdate();
    	        LOG_INFO("Entered factory rest state");
    	      } else {
    	        struct gecko_msg_system_get_bt_address_rsp_t *pAddr = gecko_cmd_system_get_bt_address();

    	        set_device_name(&pAddr->address);
    	        // Initialize Mesh stack in Node operation mode, it will generate initialized event
    	        result = gecko_cmd_mesh_node_init()->result;
    	        if (result) {

    	        }
    	      }

      // Initialize Mesh stack in Node operation mode, wait for initialized event
      gecko_cmd_mesh_node_init();
      break;
    case gecko_evt_mesh_node_initialized_id:
    	  LOG_INFO("Node initialized");
    	 if (DEVICE_IS_ONOFF_PUBLISHER==1)
    	 {
    		 // Initialize generic client models
    		 result = gecko_cmd_mesh_generic_client_init()->result;
    	 }
    	 else
    	 {
          // Initialize generic server models
          result = gecko_cmd_mesh_generic_server_init()->result;
    	 }
    	 if (result) {
    	        printf("mesh_generic_model_init failed, code 0x%x\r\n", result);
    	      }
        if(evt->data.evt_mesh_node_initialized.provisioned)
        {
    	  displayPrintf(DISPLAY_ROW_CONNECTION,"Provisioned");
    	  displayUpdate();
    	  if (DEVICE_IS_ONOFF_PUBLISHER==1	)
    	  {
    	   	 uint8_t result = mesh_lib_init(malloc, free, 8);
    	   	 LOG_INFO("%d", result);
    	   	 LOG_INFO("INITIALIZED");
    	  }
    	  else
    	  {
    		  mesh_lib_init(malloc, free, 10);
    	  }

        }
       else
       {
        // The Node is now initialized, start unprovisioned Beaconing using PB-ADV and PB-GATT Bearers
        gecko_cmd_mesh_node_start_unprov_beaconing(0x3);
       }
      break;



    case gecko_evt_hardware_soft_timer_id:
      switch (evt->data.evt_hardware_soft_timer.handle) {
        case TIMER_ID_FACTORY_RESET:
          // reset the device to finish factory reset
          gecko_cmd_system_reset(0);
          displayPrintf(DISPLAY_ROW_ACTION,"\n\r \n\r");
          break;

        case TIMER_ID_RESTART:
          // restart timer expires, reset the device
          gecko_cmd_system_reset(0);
          break;

//        case TIMER_ID_PROVISIONING:
//          // toggle LED to indicate the provisioning state
//          if (!init_done) {
//            LEDS_SetState(LED_STATE_PROV);
//          }
//          break;
//
        default:
//          // lightbulb related timer events are handled by separate function
//          handle_lightbulb_timer_evt(evt);
          break;
      }
      break;

      case gecko_evt_mesh_node_provisioning_failed_id:
        LOG_INFO("Failed as it entered provisioning failed: %x\r\n",evt->data.evt_mesh_node_provisioning_failed.result);
        displayPrintf(DISPLAY_ROW_CONNECTION,"Provisioning Failed");
        displayUpdate();
         gecko_cmd_hardware_set_soft_timer(2 * 32768, TIMER_ID_RESTART, 1);

         break;

      case gecko_evt_mesh_node_provisioning_started_id:
             LOG_INFO("Started provisioning\r\n");
       	     displayPrintf(DISPLAY_ROW_CONNECTION,"Provisioning");
       	     displayUpdate();
             break;

      case gecko_evt_mesh_node_provisioned_id:
    	     displayPrintf(DISPLAY_ROW_CONNECTION,"Provisioned");
    	     displayUpdate();
    	     if (DEVICE_IS_ONOFF_PUBLISHER==1)
    	     {
    	     mesh_lib_init(malloc, free, 8);
    	     }
    	     else{
    	    	 mesh_lib_init(malloc, free, 10);
    	     }
             break;
    case gecko_evt_le_connection_closed_id:
      /* Check if need to boot to dfu mode */
      if (boot_to_dfu) {
        /* Enter to DFU OTA mode */
        gecko_cmd_system_reset(2);
      }
      break;

    case  gecko_evt_system_external_signal_id:
    	if(SchedulerEventSet[EventHandle1Sec])
    	{
    		SchedulerEventSet[EventHandle1Sec] =0;
    		displayUpdate();
    	}
    	if(SchedulerEventSet[EventHandlePB0Pressed])
    	{
    		CORE_ATOMIC_IRQ_DISABLE();
    		SchedulerEventSet[EventHandlePB0Pressed] =0;
    		CORE_ATOMIC_IRQ_ENABLE();
    		uint16 resp;
    		struct mesh_generic_request req;
    		const uint32 transtime = 0; /* using zero transition time by default */
    	    req.kind = mesh_generic_request_on_off;
    		req.on_off = Pb0_status ? MESH_GENERIC_ON_OFF_STATE_OFF : MESH_GENERIC_ON_OFF_STATE_ON;
    		trid++;

    		  resp = mesh_lib_generic_client_publish( MESH_GENERIC_ON_OFF_CLIENT_MODEL_ID,0,  trid, &req,  transtime, 0, 0 );

    		  if (resp) {
    		    printf("gecko_cmd_mesh_generic_client_publish failed,code %x\r\n", resp);
    		  } else {
    		    printf("request sent, trid = %u\r\n", trid);
    		  }

    		if(Pb0_status)
    		{
    	    displayPrintf(DISPLAY_ROW_ACTION,"button0 released");
    		}
    		else
    		{
        	    displayPrintf(DISPLAY_ROW_ACTION,"button0 pressed");
    		}
    	    displayUpdate();
    	}
#if DEVICE_IS_ONOFF_PUBLISHER
    	if(SchedulerEventSet[EventHandlePB1Pressed])
    	{
    	    CORE_ATOMIC_IRQ_DISABLE();
    	    SchedulerEventSet[EventHandlePB1Pressed] =0;
    	    CORE_ATOMIC_IRQ_ENABLE();
    	  	if(Pb1_status)
    	    {
    	    	 displayPrintf(DISPLAY_ROW_TEMPVALUE,"button1 released");
    	    }
    	    else
    	    {
    	         displayPrintf(DISPLAY_ROW_TEMPVALUE,"button1 pressed");
    	   	}
    	    displayUpdate();
    	}
#endif
    	break;



    case gecko_evt_mesh_generic_server_client_request_id:
         LOG_INFO("received request");
         if(evt->data.evt_mesh_generic_server_client_request.parameters.data[0] == MESH_GENERIC_ON_OFF_STATE_OFF)
           {
             	 	    displayPrintf(DISPLAY_ROW_ACTION,"button0 released");
           }
         if(evt->data.evt_mesh_generic_server_client_request.parameters.data[0] == MESH_GENERIC_ON_OFF_STATE_ON)
           {
             	    	    displayPrintf(DISPLAY_ROW_ACTION,"button0 pressed");
           }
          displayUpdate();

 //        mesh_lib_generic_server_event_handler(evt);
         break;

       case gecko_evt_mesh_generic_server_state_changed_id:
    	 //  const uint8_t *msg_buf;
    	   LOG_INFO("received change");
    	     if(evt->data.evt_mesh_generic_server_client_request.parameters.data[0] == MESH_GENERIC_ON_OFF_STATE_OFF)
    	     {
    	 	    displayPrintf(DISPLAY_ROW_ACTION,"button0 released");
    	     }
    	     if(evt->data.evt_mesh_generic_server_client_request.parameters.data[0] == MESH_GENERIC_ON_OFF_STATE_ON)
    	     {
    	    	    displayPrintf(DISPLAY_ROW_ACTION,"button0 pressed");
    	     }
    	  displayUpdate();
         break;

    case gecko_evt_gatt_server_user_write_request_id:
      if (evt->data.evt_gatt_server_user_write_request.characteristic == gattdb_ota_control) {
        /* Set flag to enter to OTA mode */
        boot_to_dfu = 1;
        /* Send response to Write Request */
        gecko_cmd_gatt_server_send_user_write_response(
          evt->data.evt_gatt_server_user_write_request.connection,
          gattdb_ota_control,
          bg_err_success);

        /* Close connection to enter to DFU OTA mode */
        gecko_cmd_le_connection_close(evt->data.evt_gatt_server_user_write_request.connection);
      }
      break;
    default:
      break;
  }
}
