#include "ToF.h"
#include "stm32u5xx_hal.h"

extern I2C_HandleTypeDef hi2c2;
extern distanceHandler_t payload;


HAL_StatusTypeDef initToF() {
	printf("Initiating ToF module\n\r");
    HAL_StatusTypeDef ret;
    uint8_t deviceId;
    uint8_t deviceStatus;
    uint8_t masterCtrl;

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
        ret = i2cWrite(init_table[i].reg, init_table[i].val);
        if(ret != HAL_OK) return ret;

    }

    ret = i2cRead(DEVICE_ID_REG, &deviceId);
    if(ret != HAL_OK) return ret;
    printf("DeviceID reg 0x%02X: 0x%02X\n\r", DEVICE_ID_REG, deviceId);

    ret = i2cRead(MASTER_CTRL_REG, &masterCtrl);
    printf("Device master ctrl reg 0x%02X: 0x%02X\n\r", MASTER_CTRL_REG, masterCtrl);

    ret = i2cRead(STATUS_REG, &deviceStatus);
    printf("Device status reg 0x%02X: 0x%02X\n\r", STATUS_REG, deviceStatus);
    return HAL_OK;
}

HAL_StatusTypeDef startToFSampling(uint8_t sampleMode, uint8_t irqMode) {
    uint8_t readValue;
    HAL_StatusTypeDef status;

    status = i2cWrite(SAMPLE_REG, sampleMode);
    if (status != HAL_OK)
        return HAL_ERROR;

    status = i2cRead(SAMPLE_REG, &readValue);
    if (status != HAL_OK)
        return HAL_ERROR;

    if (readValue != sampleMode)
        return HAL_ERROR;

    status = i2cWrite(IRQ_REG, irqMode);
    if (status != HAL_OK)
        return HAL_ERROR;

    status = i2cRead(IRQ_REG, &readValue);
    if (status != HAL_OK)
        return HAL_ERROR;

    if (readValue != irqMode)
        return HAL_ERROR;

    return HAL_OK;
}

void preformToFCalibration() {
	HAL_GPIO_WritePin(pmod_SS_GPIO_Port, pmod_SS_Pin, GPIO_PIN_LOW);
	delay_us(5600);
	HAL_GPIO_WritePin(pmod_SS_GPIO_Port, pmod_SS_Pin, GPIO_PIN_HIGH);
	delay_us(14400);
}

double readToFDistance() {
	uint8_t distanceMSB = 0;
	uint8_t distanceLSB = 0;
	double distance = 1;

	while((HAL_GPIO_ReadPin(pmod_IRQ_GPIO_Port, pmod_IRQ_Pin)) != 0);
	if(i2cRead(DIST_MSB_REG, &distanceMSB) != HAL_OK) return -1;
	//printf("Distance MSB: 0x%02x\n\r", distanceMSB);
	if(i2cRead(DIST_LSB_REG, &distanceLSB) != HAL_OK) return -1;
	//printf("Distance LSB: 0x%02x\n\r", distanceLSB);
    distance =(((double)distanceMSB * 256 + (double)distanceLSB)/65536) * TOF_SCALE_METERS;
    return distance; // measured distance in meters
}

void performDistanceMeasurement() {
	double distanceInMeters;
	if((startToFSampling(0x7D, 0x01)) != HAL_OK) return;
	while(1) {
		preformToFCalibration();
		distanceInMeters = readToFDistance();
//		payload.timestampMS = ; TODO take the timestamp
		payload.distanceCM = distanceInMeters * 100.0;
		printf("distance: %lf\n\r", payload.distanceCM);

	}
}
