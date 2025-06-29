/* ************************************************************************** */
/** Descriptive File Name

  @Company
    HGE

  @File Name
    DS18B20.h

  @Summary
    Driver for DS18B20.

  @Description
    Version: 1.0.1.10
 */
/* ************************************************************************** */

#ifndef _DS18B20_H    /* Guard against multiple inclusion */
#define _DS18B20_H


#include "ONEWire.h"

#define SEARCHROM           0xF0
#define READROM             0x33
#define MATCHROM            0x55            
#define SKIPROM             0xCC
#define ALARMSEARCH         0xEC
#define CONVERTEMP          0x44
#define READSCRACHPAD       0xBE
#define WRITESCRACHPAD      0x4E
#define COPYSCRACHPAD       0x48
#define RECALLEEPROM        0xB8
#define READPOWERSUPPLY     0xB4
    
//#define DS18B20IDLE         0
//#define DS18B20READINGTEMP  1
//#define DS18B20TEMPRDYOK    2
//#define DS18B20TEMPRDYERROR 3
//#define DS18B20BUSY    		4

typedef enum{
	DS18B20_ST_ERRROR			= -1,
	DS18B20_ST_TEMPOK			= 0,
	DS18B20_ST_TEMPCRCERROR		= 1,
	DS18B20_ST_NOTPRESENT		= 2,
	DS18B20_ST_IDLE				= 3,
	DS18B20_ST_OK				= 4,
	DS18B20_ST_READINGTEMP		= 5,
} _eDS18B20Status;

typedef struct{
	_sOWHandle 	*OW;					/*<ONEWIRE Definition*/
	void (*OnTempReady)(int16_t temp, _eDS18B20Status stateTemp);

	struct {							/*<Internal use. NOT modify*/
		int16_t 			lastTempValue;
		uint8_t 			crcDS18B20;
		uint8_t 			iBytes;
		uint8_t 			dataDS18B20[9];
		_eDS18B20Status 	status;
		uint32_t 			currentus;
		uint16_t 			stateTask;
		uint32_t			timeOutTempReady;
		uint32_t			timeOutLastUs;
	} ds18B20Data;
} _sDS18B20Handle;

/* Provide C++ Compatibility */
#ifdef __cplusplus
extern "C" {
#endif



/**
 * @brief DS18B20_Init
 * Initialize the ONEWire bus. This function must be called first.
 * @param [in] aOW: Struct that define a ONEWire Bus
 * @return:  
 */
void DS18B20_Init(_sDS18B20Handle *hDS18B20, void (*OnTempReady)(int16_t temp, _eDS18B20Status stateTemp));

/**
 * @brief DS18B20_StartReadTemp
 * Start a new convertion.
 * 
 * @return:  
 */
_eDS18B20Status DS18B20_StartReadTemp(_sDS18B20Handle *hDS18B20);

/**
 * @brief DS18B20_ReadLastTemp
 * Get the latest temperature measurement.
 * Reset STATUS to IDLE.
 * 
 * Beside to get last temperature, use this function to reset status to IDLE every time that you need.
 *
 * @return: Latest temperature value
 */
int16_t DS18B20_ReadLastTemp(_sDS18B20Handle *hDS18B20);


/**
 * @brief DS18B20_Status
 * Get the latest status.
 * 
 * @return:  
 */
_eDS18B20Status DS18B20_Status(_sDS18B20Handle *hDS18B20);

/**
 * @brief DS18B20_Task
 * Task that measure a new temperature.
 * 
 */
void DS18B20_Task(_sDS18B20Handle *hDS18B20, uint32_t currentus);


    /* Provide C++ Compatibility */
#ifdef __cplusplus
}
#endif

#endif /* _EXAMPLE_FILE_NAME_H */

/* *****************************************************************************
 End of File
 */
