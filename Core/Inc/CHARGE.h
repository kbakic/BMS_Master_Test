#ifndef CHARGE_H
#define CHARGE_H

#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include "stm32f4xx_hal.h"
#include "LTC681x.h"

#define BMS_CAN_ID 0x1806E5F4 // ID of message we send to Charger with target Voltage, Current and charging state
#define CCS_CAN_ID 0x18FF50E5 // ID of message we should receive from Charger with it's current Temperature, Charging Current, Charging Voltage and SoC

#define SOC_THRESHOLD 90
#define CAN_MSG_DLC 8
#define CHARGING_STATE_OFF 1
#define CHARGING_STATE_ON 0

/*Data indexes for message BMS sends to Charger*/
#define MAX_VOLTAGE_HIGH_BYTE_INDEX 0
#define MAX_VOLTAGE_LOW_BYTE_INDEX 1
#define MAX_CURRENT_HIGH_BYTE_INDEX 2
#define MAX_CURRENT_LOW_BYTE_INDEX 3
#define CHARGING_STATE_INDEX 4

/*Data indexes for message BMS receives from Charger*/
#define DATA_VOLTAGE_HIGH_BYTE_INDEX 0
#define DATA_VOLTAGE_LOW_BYTE_INDEX 1
#define DATA_CURRENT_HIGH_BYTE_INDEX 2
#define DATA_CURRENT_LOW_BYTE_INDEX 3
#define DATA_STATUS_FLAGS_INDEX 4

/*Indexes of flags received from Charger*/
#define HARDWARE_FAILURE_FLAG 0
#define TEMPERATURE_OF_CHARGER_FLAG 1
#define INPUT_VOLTAGE_FLAG 2
#define STARTING_STATE_FLAG 3
#define COMMUNICATION_STATE_FLAG 4

/*Hardware Failure flags*/
#define NORMAL 0
#define HARDWARE_FAILURE 1
/*Temperature of Charger flags*/
#define NORMAL_TEMPERATURE 0
#define OVER_TEMPERATURE_PROTECTION 1
/*Input Voltage flags*/
#define NORMAL_INPUT_VOLTAGE 0
#define WRONG_INPUT_VOLTAGE 1
/*Starting State flags*/
#define BATTERY_DETECTED 0
#define BATTERY_NOT_DETECTED 1
/*Communication State flags*/
#define NORMAL_COMMUNICATION 0
#define COMMUNACTION_TIMED_OUT 1

#define MAX_TEMPERATURE 40
#define MAX_CHARGING_CURRENT 20
#define MAX_CHARGING_VOLTAGE 50

//slave ID
#define TOTAL_IC 16 //total number of ICs

//temporary values until function to read real ones is implemented
float soc = 20;
float hottestCellTemperature = 50;
float voltage[16][12];
float temperature[16][4];
float chargingVoltage = 0;
float chargingCurrent = 0;
uint8_t chargerFlags = 0;

extern cell_asic bms_ic[TOTAL_IC];
extern bool flagStartCVConversion;
extern bool flagReadCVConversion;
extern bool	flagStartAuxConversion;
extern bool flagReadAuxConversion;

void SendCANMessage(unsigned long id, uint8_t* data, CAN_HandleTypeDef hcan1, uint32_t canTxMailbox, CAN_TxHeaderTypeDef myTxHeader);
uint8_t ReceiveChargerCANMessage(unsigned long id, CAN_HandleTypeDef hcan1, uint8_t rx_data[8], CAN_FilterTypeDef filterConfig,CAN_RxHeaderTypeDef myRxHeader);
int GetChargingState(float soc, float temperature, float chargingCurrent, float chargingVoltage);
void ChargingStateAlgorithm(CAN_HandleTypeDef hcan1, uint8_t rx_data[8], CAN_FilterTypeDef filterConfig,CAN_RxHeaderTypeDef myRxHeader, CAN_TxHeaderTypeDef myTxHeader, uint32_t canTxMailbox, float voltage, float temperature);
void EncodeDataForCAN(uint8_t *dataForCAN);
void DecodeChargerMessage(uint8_t *dataFromCharger);
int calculateChargingCurrent(int temperature, int voltage);
void CellNumSwitch(int ltc);
void GetCellVoltages();
void GetCellTemperatures();
void Error_Handler(void);

#endif
