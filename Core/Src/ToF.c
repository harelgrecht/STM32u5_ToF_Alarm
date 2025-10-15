#include "ToF.h"
#include "stm32u5xx_hal.h"

extern I2C_HandleTypeDef hi2c2;
extern distanceHandler_t payload;

#define DEBUG_PRINTS

HAL_StatusTypeDef initToF() {
	printf("Initiating ToF module\n\r");
    HAL_StatusTypeDef ret;
    uint8_t deviceId;

    struct reg_val {
        uint8_t reg;
        uint8_t val;
    } init_table[] = {
        {0x10, 0x04},
        {0x11, 0x6E},
        {SAMPLE_REG, 0x71},
        {0x18, 0x22},
        {0x19, 0x22},
        {IRQ_REG, 0x01},
        {0x90, 0x0F},
        {0x91, 0xFF}
    };

    for(int i = 0; i < sizeof(init_table)/sizeof(init_table[0]); i++) {
        ret = HAL_I2C_Mem_Write(&hi2c2,
                                TOF_I2C_DEV,
                                init_table[i].reg,
                                I2C_MEMADD_SIZE_8BIT,
                                &init_table[i].val,
                                1,
                                100);
        if(ret != HAL_OK) return ret;
    }

    ret = HAL_I2C_Mem_Read(&hi2c2, TOF_I2C_DEV, DEVICE_ID_REG, I2C_MEMADD_SIZE_8BIT, &deviceId, 1, 100);
    if(ret != HAL_OK) return ret;
#ifdef DEBUG_PRINTS
    printf("DeviceID reg 0x00: 0x%02X\n\r", deviceId);
#endif
    return HAL_OK;
}

HAL_StatusTypeDef startToFSampling(uint8_t sampleMode, uint8_t irqMode) {
	HAL_StatusTypeDef returnStatus;
	uint8_t sampleRegData;
	uint8_t irqRegData;

	returnStatus = HAL_I2C_Mem_Write(&hi2c2, TOF_I2C_DEV, SAMPLE_REG, I2C_MEMADD_SIZE_8BIT, &sampleMode, 1, 100);
	if(returnStatus == HAL_OK) {
		HAL_I2C_Mem_Read(&hi2c2, TOF_I2C_DEV, SAMPLE_REG, I2C_MEMADD_SIZE_8BIT, &sampleRegData, 1, 100);
		if (sampleRegData == sampleMode) returnStatus = HAL_OK;
	} else {
		returnStatus = HAL_ERROR;
	}

	returnStatus = HAL_I2C_Mem_Write(&hi2c2, TOF_I2C_DEV, IRQ_REG, I2C_MEMADD_SIZE_8BIT, &irqMode, 1, 100);
	if(returnStatus == HAL_OK) {
		HAL_I2C_Mem_Read(&hi2c2, TOF_I2C_DEV, IRQ_REG, I2C_MEMADD_SIZE_8BIT, &irqRegData, 1, 100);
		if (irqRegData == irqMode) returnStatus = HAL_OK;
	} else {
		returnStatus =  HAL_ERROR;
	}

	if(returnStatus != HAL_OK)
		return HAL_ERROR;
	return HAL_OK;
}

double readToFDistance() {
	uint8_t distanceMSB = 0;
	uint8_t distanceLSB = 0;
	double distance = 1;

	while((HAL_GPIO_ReadPin(pmod_IRQ_GPIO_Port, pmod_IRQ_Pin)) != 0);
	if((HAL_I2C_Mem_Read(&hi2c2, TOF_I2C_DEV, DIST_MSB_REG, I2C_MEMADD_SIZE_8BIT, &distanceMSB, 1, 100)) != HAL_OK) return -1;
#ifdef DEBUG_PRINTS
	printf("Distance MSB: %d\n\r", distanceMSB);
#endif
	if((HAL_I2C_Mem_Read(&hi2c2, TOF_I2C_DEV, DIST_LSB_REG, I2C_MEMADD_SIZE_8BIT, &distanceLSB, 1, 100)) != HAL_OK) return -1;
#ifdef DEBUG_PRINTS
	printf("Distance LSB: %d\n\r", distanceLSB);
#endif
    distance =(((double)distanceMSB * 256 + (double)distanceLSB)/65536) * TOF_SCALE_METERS;
    return distance;
}

HAL_StatusTypeDef preformToFCalibration() {
	HAL_GPIO_WritePin(pmod_SS_GPIO_Port, pmod_SS_Pin, GPIO_PIN_HIGH);
	HAL_Delay(6);
	HAL_GPIO_WritePin(pmod_SS_GPIO_Port, pmod_SS_Pin, GPIO_PIN_LOW);
	HAL_Delay(14);
	return HAL_OK;
}


void performDistanceMeasurement() {
	if((startToFSampling(0x7D, 0x01)) != HAL_OK) return;
	preformToFCalibration();
	while(1) {
		payload.distanceCM = readToFDistance();
//		payload.timestampMS = ; TODO take the timestamp
		if(payload.distanceCM > 0)
#ifdef DEBUG_PRINTS
			printf("distance: %lf\n", payload.distanceCM);
#endif
		else
#ifdef DEBUG_PRINTS
			printf("no distance\n");
#endif
	}
}
