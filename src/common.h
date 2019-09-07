#ifndef __COMMON_H
#define __COMMON_H

#include "em_letimer.h"
/* Bluetooth stack headers */
#include "bg_types.h"
#include "native_gecko.h"
#include "gatt_db.h"
//#include "infrastructure.h"


#include "gecko_ble_errors.h"

/* Libraries containing default Gecko configuration values */
#include "em_emu.h"
#include "em_cmu.h"

/* Device initialization header */
#include "hal-config.h"

#if defined(HAL_CONFIG)
#include "bsphalconfig.h"
#else
#include "bspconfig.h"
#endif

#include "log.h"
#include "em_device.h"
#include "em_chip.h"
#include "em_core.h"
//#include "gpio.h"
//#include "sleep.h"
#include "letimer.h"
//#include "display.h"
#include "ble_device_type.h"
//#include "server.h"


#define TotalTicksPerCycle 65535     //FFFF is the total ticks
#define TimePerTick 1/32768          //assuming clock frequency of 32768Hz.
#define TimeTakenPerCycle 2000          //this is in ms and assuming clock frequency of 32768Hz.


//BLUETOOTH Parameters
#define MIN_ADVERTISING_INTERVAL	(250)     // this is in ms
#define MAX_ADVERTISING_INTERVAL	(250)     // this is in ms
#define MIN_CONNECTION_INTERVAL		(75)      // this is in ms
#define MAX_CONNECTION_INTERVAL		(75)      // this is in ms
#define SLAVE_LATENCY_MS			(300)

//these are bluetooth values for use in functions.
#define MIN_ADVERTISING_INTERVAL_VALUE	(400) //Value for advertising interval is obtained from formula=> advertising_time = value *0.625.
#define MAX_ADVERTISING_INTERVAL_VALUE  (400)
#define MIN_CONNECTION_INTERVAL_VALUE	(60)  //Value for connection interval is obtained from formula=> connection_time = value * 1.25
#define MAX_CONNECTION_INTERVAL_VALUE	(60)
#define SLAVE_LATENCY					(3)	  //Slave_Latency is obtained from formula=> Slave_latency = (LE_SLAVE_LATENCY_MS/LE_MAX_CONNECTION_INTERVAL_MS) - 1
#define CONNECTION_TIMEOUT_MS           (600) // as timeout should be >= (1+slave_latency)*(connection_interval_time*2)
#define BLE_TX_MAX 80           //Here tx maximum is 8 db for the radio but giving it in 0.1dbm units for function requirements
#define BLE_TX_MIN -260         //Here tx minimum is -26 db for the radio but giving it in 0.1dbm units for function requirements


//  float* Temp_value = NULL;


  enum device_state{
 	  State_Disconnected,
 	  State_Scanning_for_device,
 	  State_Discover_Services,
 	  State_Discover_Characteristics,
 	  state_enable_notifications
  };




 enum TempSensorState{
	Temp_Sensor_wait_For_PowerOn,
	Temp_Sensor_wait_For_Sensor_Enabled,
	Temp_Sensor_wait_For_Write_Complete,
	Temp_Sensor_wait_For_Read_Complete,
	Temp_Sensor_wait_For_PowerOff,
};

bool SchedulerEventSet[9];
enum SetSechdulerEvent
{
	EventHandlePowerOn,
	EventHandleI2CEnabled,
	EventHandleI2CTransferComplete,
	EventHandleI2CTransferFail,
	EventHandleI2CTransferInProgress,
	EventHandlePowerOff,
	EventHandle1Sec,
	EventHandlePB0Pressed,
	EventHandlePB1Pressed
};




#endif /* __COMMON_H_ */
