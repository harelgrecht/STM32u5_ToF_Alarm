#include "ToF.h"
#include "stm32u5xx_hal.h"

extern I2C_HandleTypeDef hi2c2;

HAL_StatusTypeDef initToF() {
    HAL_StatusTypeDef ret;
    struct reg_val {
        uint8_t reg;
        uint8_t val;
    } init_table[] = {
        {0x10, 0x04},
        {0x11, 0x6E},
        {SAMPLE_REG, 0x71},
        {0x18, 0x22},
        {0x19, 0x22},
        {0x60, 0x01},
        {0x90, 0x0F},
        {0x91, 0xFF}
    };

    // 1. Write all initialization registers
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

    // 2. Read serial number / device ID to verify
    uint8_t serial[2] = {0};
    ret = HAL_I2C_Mem_Read(&hi2c2, TOF_I2C_DEV, 0x16, I2C_MEMADD_SIZE_8BIT, &serial[0], 1, 100);
    if(ret != HAL_OK) return ret;

    ret = HAL_I2C_Mem_Read(&hi2c2, TOF_I2C_DEV, 0x17, I2C_MEMADD_SIZE_8BIT, &serial[1], 1, 100);
    if(ret != HAL_OK) return ret;

    uint16_t serial_number = ((uint16_t)serial[0] << 8) | serial[1];
    if(serial_number == 0 || serial_number == 0xFFFF) {
        // invalid serial number
        return HAL_ERROR;
    }
    return HAL_OK;
}


HAL_StatusTypeDef startToFSampling(uint8_t sampleMode, uint8_t irqMode) {
	HAL_StatusTypeDef returnStatus;

	returnStatus = HAL_I2C_Mem_Write(&hi2c2, TOF_I2C_DEV, SAMPLE_REG, I2C_MEMADD_SIZE_8BIT, &sampleMode, 1, 100);
	if(returnStatus != HAL_OK)
		return HAL_ERROR;
	returnStatus = HAL_I2C_Mem_Write(&hi2c2, TOF_I2C_DEV, IRQ_REG, I2C_MEMADD_SIZE_8BIT, &irqMode, 1, 100);
	if(returnStatus != HAL_OK)
		return HAL_ERROR;
	return HAL_OK;
}

double readToFDistance() {
	uint8_t distanceMSB;
	uint8_t distanceLSB;
	double distance = 1;

	while((HAL_GPIO_ReadPin(pmod_IRQ_GPIO_Port, pmod_IRQ_Pin)) != 0);
	HAL_I2C_Mem_Read(&hi2c2, TOF_I2C_DEV, DIST_MSB_REG, I2C_MEMADD_SIZE_8BIT, &distanceMSB, 1, 100);
	HAL_I2C_Mem_Read(&hi2c2, TOF_I2C_DEV, DIST_LSB_REG, I2C_MEMADD_SIZE_8BIT, &distanceLSB, 1, 100);
    distance =(((double)distanceMSB * 256 + (double)distanceLSB)/65536) * TOF_SCALE_METERS;
    return distance;
}

HAL_StatusTypeDef preformToFCalibration() {
	HAL_GPIO_WritePin(pmod_SS_GPIO_Port, pmod_SS_Pin, GPIO_PIN_HIGH);
	HAL_Delay(5.6);
	HAL_GPIO_WritePin(pmod_SS_GPIO_Port, pmod_SS_Pin, GPIO_PIN_LOW);
	HAL_Delay(14.4);
	return HAL_OK;
}


void performDistanceMeasurement() {
	if((startToFSampling(0x7D, 0x01)) != HAL_OK) return;
	preformToFCalibration();
	double distance = readToFDistance();
	if(distance > 0) {
		printf("distance: %d\n", distance);
	} else {
		printf("no distance\n");
	}
}
