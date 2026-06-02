#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include "CHARGE.h"
#include "stm32f4xx_hal_conf.h"
#include "LTC681x.h"


void SendCANMessage(unsigned long id, uint8_t* data, CAN_HandleTypeDef hcan1, uint32_t canTxMailbox, CAN_TxHeaderTypeDef myTxHeader) {

	  if (HAL_CAN_AddTxMessage(&hcan1, &myTxHeader, data, &canTxMailbox) != HAL_OK)
	  	{
	  	   Error_Handler ();
	  	}
}

uint8_t ReceiveChargerCANMessage(unsigned long id, CAN_HandleTypeDef hcan1, uint8_t rx_data[8], CAN_FilterTypeDef filterConfig,CAN_RxHeaderTypeDef myRxHeader){

	  HAL_CAN_ConfigFilter(&hcan1, &filterConfig);

	  if (HAL_CAN_GetRxMessage(&hcan1, CAN_RX_FIFO0, &myRxHeader, rx_data) != HAL_OK)
	  {
	    Error_Handler();
	  }
	  return rx_data;
}

int GetChargingState(float soc, float temperature, float chargingCurrent, float chargingVoltage){
    if (soc < SOC_THRESHOLD) {

        if (temperature > MAX_TEMPERATURE) {
        return CHARGING_STATE_OFF;
        }

        if (chargingCurrent > MAX_CHARGING_CURRENT){
        return CHARGING_STATE_OFF;
        }

        if(chargingVoltage > MAX_CHARGING_VOLTAGE) {
        return CHARGING_STATE_OFF;
        }

        else {
        return CHARGING_STATE_ON;
        }

    }

    else {
        return CHARGING_STATE_OFF;
    }

}

void ChargingStateAlgorithm(CAN_HandleTypeDef hcan1, uint8_t rx_data[8], CAN_FilterTypeDef filterConfig,CAN_RxHeaderTypeDef myRxHeader, CAN_TxHeaderTypeDef myTxHeader, uint32_t canTxMailbox, float voltage, float temperature) {

    uint8_t RxData = ReceiveChargerCANMessage(CCS_CAN_ID, hcan1, rx_data, filterConfig, myRxHeader);
    DecodeChargerMessage(&RxData);

    int chargingState = GetChargingState(soc, hottestCellTemperature, chargingCurrent, chargingVoltage);

    uint8_t CAN_data[CAN_MSG_DLC];

    if(chargingState == CHARGING_STATE_ON){
        CAN_data[CHARGING_STATE_INDEX] = CHARGING_STATE_ON;
        EncodeDataForCAN(CAN_data);
        SendCANMessage(BMS_CAN_ID, CAN_data, hcan1, canTxMailbox, myTxHeader);
    }

    else{
        CAN_data[CHARGING_STATE_INDEX] = CHARGING_STATE_OFF;
        EncodeDataForCAN(CAN_data);
        SendCANMessage(BMS_CAN_ID, CAN_data, hcan1, canTxMailbox, myTxHeader);
    }

}

void EncodeDataForCAN(uint8_t *dataForCAN){

    uint16_t voltageInt = (uint16_t)floor((chargingVoltage * 10));
    uint16_t currentInt = (uint16_t)floor((chargingCurrent * 10));

    uint8_t voltageShort = (uint8_t)voltageInt;
    uint8_t currentShort = (uint8_t)currentInt;


    dataForCAN[MAX_VOLTAGE_HIGH_BYTE_INDEX] = (voltageShort >> 8) & 0xFF;
    dataForCAN[MAX_VOLTAGE_LOW_BYTE_INDEX] = voltageShort & 0xFF;
    dataForCAN[MAX_CURRENT_HIGH_BYTE_INDEX] = (currentShort >> 8) & 0xFF;
    dataForCAN[MAX_CURRENT_LOW_BYTE_INDEX] = currentShort & 0xFF;
}


void DecodeChargerMessage(uint8_t *dataFromCharger){

    uint16_t voltageInt = ((uint16_t)dataFromCharger[DATA_VOLTAGE_HIGH_BYTE_INDEX] << 8) | dataFromCharger[DATA_VOLTAGE_LOW_BYTE_INDEX];
    uint16_t currentInt = ((uint16_t)dataFromCharger[DATA_CURRENT_HIGH_BYTE_INDEX] << 8) | dataFromCharger[DATA_CURRENT_LOW_BYTE_INDEX];

    chargingVoltage = voltageInt * 0.1;
    chargingCurrent = currentInt * 0.1;

    chargerFlags = dataFromCharger[DATA_STATUS_FLAGS_INDEX];
}

void CellNumSwitch(int ltc){

	if(ltc%2==0){
				for(int cell=0;cell<6;cell++){
					voltage[ltc][cell] = bms_ic[ltc].cells.c_codes[cell] * 0.0001;
				}
				for(int cell=6;cell<10;cell++){
					voltage[ltc][cell+1] = bms_ic[ltc].cells.c_codes[cell] * 0.0001;
			}

		}
				else{
					if(ltc!=11){
					for(int cell=0;cell<6;cell++){
						voltage[ltc][cell] = bms_ic[ltc].cells.c_codes[cell] * 0.0001;
				}
					for(int cell=6;cell<9;cell++){
						voltage[ltc][cell] = bms_ic[ltc].cells.c_codes[cell] * 0.0001;
					}
				}

					else{
						for(int cell=0;cell<6;cell++){
							voltage[ltc][cell] = 0;
									}
						for(int cell=6;cell<9;cell++){
							voltage[ltc][cell] = 0;
						}
				}
}

void GetCellVoltages(){

	if(flagStartCVConversion){
		wakeup_idle(TOTAL_IC);
		LTC6811_adcv(MD_27KHZ_14KHZ, DCP_DISABLED, CELL_CH_ALL);
		flagStartCVConversion = false;
	}

	if(flagReadCVConversion){
		wakeup_idle(TOTAL_IC);

		for(int ltc=0;ltc<12;ltc++){
			LTC6811_rdcv(CELL_CH_ALL, ltc, bms_ic);

			CellNumSwitch(ltc);
	}
			flagReadCVConversion = false;
		}
	}
}


void GetCellTemperatures(){

	for(int i=0;i<12;i++){
		if(flagStartAuxConversion){
			wakeup_idle(TOTAL_IC);
			LTC6811_adax(MD_27KHZ_14KHZ, AUX_CH_ALL);
			flagStartAuxConversion = false;
			}

		if(flagReadAuxConversion){
			wakeup_idle(TOTAL_IC);
			LTC6811_rdaux(AUX_CH_ALL, TOTAL_IC, bms_ic);
				for(int j=0;j<4;j++){
					temperature[i][j] = bms_ic[i].aux.a_codes[j];
					LTC681x_stcomm();
				}

				flagReadAuxConversion = false;
		}
	}

}
