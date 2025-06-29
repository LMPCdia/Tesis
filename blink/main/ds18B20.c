/* ************************************************************************** */
/** Descriptive File Name

  @Company
    HGE

  @File Name
    DS18B20.c

  @Summary
    Driver DS18B20.

  @Description
*/

#include "DS18B20.h"
#include <string.h>
#include <stdlib.h>

//typedef enum {
//    IDLE = 0,
//    READINGTEMP0,
//    READINGTEMP1,
//    READINGTEMP2,
//    READINGTEMP3,
//    READINGTEMP4,
//    READINGTEMP5,
//    READINGTEMP6,
//}_eDB18b20TaskStatus;


//static int16_t lastTempValue = 0;
//
//static uint8_t crcDS18B20 = 0, iBytes = 0;
//static uint8_t dataDS18B20[9] = {0, 0, 0, 0, 0, 0, 0, 0, 0};
//
//static _eDB18b20TaskStatus db18b20TaskStatus;
//
//static uint8_t statusDS18B20 = DS18B20IDLE;
//
//static uint32_t timeOutReadTemp = 0;
//static uint32_t timeOutLastus = 0;

#define	IDLE				0x01
#define	READINGTEMP0		0x02
#define	READINGTEMP1		0x04
#define	READINGTEMP2		0x08
#define	READINGTEMP3		0x10
#define	READINGTEMP4		0x20
#define	READINGTEMP5		0x40
#define	READINGTEMP6		0x80

static uint8_t DS18B20_CRC(uint8_t *buf, uint8_t length){
	uint8_t crc = 0, inbyte, i, mix;

	while (length--) {
		inbyte = *buf++;
		for (i = 8; i; i--) {
			mix = (crc ^ inbyte) & 0x01;
			crc >>= 1;
			if (mix) {
				crc ^= 0x8C;
			}
			inbyte >>= 1;
		}
	}

	return crc;
}


void DS18B20_Task(_sDS18B20Handle *hDS18B20, uint32_t currentus){
    ONEWireTask(hDS18B20->OW, currentus);

    if(ONEWireGetStatusTask(hDS18B20->OW) != ONEWIRE_ST_READY)
        return;
    
    switch(hDS18B20->ds18B20Data.stateTask){
        case IDLE:
            break;
        case READINGTEMP0:
            if(ONEWireIsPresent(hDS18B20->OW)){
                ONEWireWriteByteTask(hDS18B20->OW, SKIPROM);
                hDS18B20->ds18B20Data.stateTask = READINGTEMP1;
            }
            else{
            	hDS18B20->ds18B20Data.stateTask = IDLE;
            	hDS18B20->ds18B20Data.status = DS18B20_ST_NOTPRESENT;
                hDS18B20->ds18B20Data.lastTempValue = 0xFA59;
                if(hDS18B20->OnTempReady != NULL)
                	hDS18B20->OnTempReady(0xFA59, DS18B20_ST_NOTPRESENT);
            }
            break;
        case READINGTEMP1:
            ONEWireWriteByteTask(hDS18B20->OW, CONVERTEMP);
            hDS18B20->ds18B20Data.stateTask = READINGTEMP2;
            hDS18B20->ds18B20Data.timeOutTempReady = 1000000;
            hDS18B20->ds18B20Data.timeOutLastUs = currentus;
            break;
        case READINGTEMP2:
        	hDS18B20->ds18B20Data.timeOutLastUs = currentus - hDS18B20->ds18B20Data.timeOutLastUs;
        	hDS18B20->ds18B20Data.timeOutTempReady -= hDS18B20->ds18B20Data.timeOutLastUs;
        	hDS18B20->ds18B20Data.timeOutLastUs = currentus;
            if(hDS18B20->ds18B20Data.timeOutTempReady > 1000000u){
                hDS18B20->ds18B20Data.iBytes = 0;
                hDS18B20->ds18B20Data.stateTask = READINGTEMP3;
                ONEWireResetTask(hDS18B20->OW);
            }
            break;
        case READINGTEMP3:
            if(ONEWireIsPresent(hDS18B20->OW)){
                ONEWireWriteByteTask(hDS18B20->OW, SKIPROM);
                hDS18B20->ds18B20Data.stateTask = READINGTEMP4;
            }
            else{
            	hDS18B20->ds18B20Data.stateTask = IDLE;
            	hDS18B20->ds18B20Data.status = DS18B20_ST_NOTPRESENT;
                hDS18B20->ds18B20Data.lastTempValue = 0xFA59;
                if(hDS18B20->OnTempReady != NULL)
                	hDS18B20->OnTempReady(0xFA59, DS18B20_ST_NOTPRESENT);
            }
            break;
        case READINGTEMP4:
            hDS18B20->ds18B20Data.iBytes = 255;
            ONEWireWriteByteTask(hDS18B20->OW, READSCRACHPAD);
            hDS18B20->ds18B20Data.stateTask = READINGTEMP5;
            break;
        case READINGTEMP5:
            if(hDS18B20->ds18B20Data.iBytes != 255)
            	hDS18B20->ds18B20Data.dataDS18B20[hDS18B20->ds18B20Data.iBytes]  = ONEWireGetLastByteReadTask(hDS18B20->OW);
            hDS18B20->ds18B20Data.iBytes++;
            if(hDS18B20->ds18B20Data.iBytes == 9){
                hDS18B20->ds18B20Data.stateTask = IDLE;
                hDS18B20->ds18B20Data.crcDS18B20 = DS18B20_CRC(hDS18B20->ds18B20Data.dataDS18B20, 8);

                hDS18B20->ds18B20Data.lastTempValue = 0;
                hDS18B20->ds18B20Data.lastTempValue |= (hDS18B20->ds18B20Data.dataDS18B20[1] << 8);
                hDS18B20->ds18B20Data.lastTempValue |= hDS18B20->ds18B20Data.dataDS18B20[0];

                hDS18B20->ds18B20Data.status = DS18B20_ST_TEMPOK;
                if(hDS18B20->ds18B20Data.dataDS18B20[8] != hDS18B20->ds18B20Data.crcDS18B20){
//                	hDS18B20->ds18B20Data.lastTempValue = 0xFA5A;
                	hDS18B20->ds18B20Data.status = DS18B20_ST_TEMPCRCERROR;
                }

                if(hDS18B20->OnTempReady != NULL){
                	hDS18B20->OnTempReady(hDS18B20->ds18B20Data.lastTempValue, hDS18B20->ds18B20Data.status);
                	hDS18B20->ds18B20Data.status = DS18B20_ST_IDLE;
                }
            }
            else
                ONEWireReadByteTask(hDS18B20->OW);
            break;
        default:
        	hDS18B20->ds18B20Data.stateTask = IDLE;
        	hDS18B20->ds18B20Data.status = DS18B20_ST_IDLE;
    }
}


void DS18B20_Init(_sDS18B20Handle *hDS18B20, void (*OnTempReady)(int16_t temp, _eDS18B20Status stateTemp)){
	memset((uint8_t *)&hDS18B20->ds18B20Data, 0, sizeof(hDS18B20->ds18B20Data));

	hDS18B20->ds18B20Data.lastTempValue = 0xFA5A;
    hDS18B20->ds18B20Data.stateTask = IDLE;
    hDS18B20->ds18B20Data.status = DS18B20_ST_IDLE;
    hDS18B20->OnTempReady = OnTempReady;
    ONEWire_Init(hDS18B20->OW);
    
    return;
}

_eDS18B20Status DS18B20_StartReadTemp(_sDS18B20Handle *hDS18B20){
    if(ONEWireGetStatusTask(hDS18B20->OW) != ONEWIRE_ST_READY)
        return hDS18B20->ds18B20Data.status;
    
    if(hDS18B20->ds18B20Data.stateTask != IDLE)
    	return hDS18B20->ds18B20Data.status;

    ONEWireResetTask(hDS18B20->OW);
    
    hDS18B20->ds18B20Data.stateTask = READINGTEMP0;
    
    hDS18B20->ds18B20Data.status = DS18B20_ST_READINGTEMP;

    return DS18B20_ST_OK;
}

int16_t DS18B20_ReadLastTemp(_sDS18B20Handle *hDS18B20){
	hDS18B20->ds18B20Data.status = DS18B20_ST_IDLE;
    return hDS18B20->ds18B20Data.lastTempValue;
}

_eDS18B20Status DS18B20_Status(_sDS18B20Handle *hDS18B20){
    return hDS18B20->ds18B20Data.status;
}


/* *****************************************************************************
 End of File
 */
