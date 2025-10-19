#include "ToF.h"
#include "stm32u5xx_hal.h"

extern I2C_HandleTypeDef hi2c2;
extern distanceHandler_t payload;

HAL_StatusTypeDef initToF(void) {
    printf("Initializing ToF module...\n\r");

    const struct {
        uint8_t reg;
        uint8_t val;
    } initTable[] = {
        {0x10, 0x04},
        {0x11, 0x6E},
        {SAMPLE_REG, 0x71},
        {0x18, 0x22},
        {0x19, 0x22},
        {IRQ_REG, 0x01},
        {0x90, 0x0F},
        {0x91, 0xFF}
    };

    HAL_StatusTypeDef ret;
    uint8_t deviceId, masterCtrl, deviceStatus;

    for (uint32_t i = 0; i < sizeof(initTable)/sizeof(initTable[0]); i++) {
        ret = i2cWrite(initTable[i].reg, initTable[i].val);
        if (ret != HAL_OK) return ret;
    }

    if (i2cRead(DEVICE_ID_REG, &deviceId) == HAL_OK)
        printf("Device ID (0x%02X): 0x%02X\n\r", DEVICE_ID_REG, deviceId);

    if (i2cRead(MASTER_CTRL_REG, &masterCtrl) == HAL_OK)
        printf("Master Ctrl (0x%02X): 0x%02X\n\r", MASTER_CTRL_REG, masterCtrl);

    if (i2cRead(STATUS_REG, &deviceStatus) == HAL_OK)
        printf("Status Reg (0x%02X): 0x%02X\n\r", STATUS_REG, deviceStatus);

    printf("init done\n\r");
    return HAL_OK;
}

HAL_StatusTypeDef resetToF(void) {
	// Clean and reset the device before anything
	if(i2cWrite(CMD_REG, 0x49) != HAL_OK) return HAL_ERROR; // Emulates sample start pin
	if(i2cWrite(CMD_REG, 0xD7) != HAL_OK) return HAL_ERROR; // Resets all registers
	if(i2cWrite(CMD_REG, 0xD1) != HAL_OK) return HAL_ERROR;  // Resets internal state machine
}

HAL_StatusTypeDef startToFSampling(uint8_t sampleMode, uint8_t irqMode) {
    uint8_t value;

    if (i2cWrite(SAMPLE_REG, sampleMode) != HAL_OK) return HAL_ERROR;
    if (i2cRead(SAMPLE_REG, &value) != HAL_OK || value != sampleMode) return HAL_ERROR;

    if (i2cWrite(IRQ_REG, irqMode) != HAL_OK) return HAL_ERROR;
    if (i2cRead(IRQ_REG, &value) != HAL_OK || value != irqMode) return HAL_ERROR;

    return HAL_OK;
}

void performToFCalibration(void) {
    HAL_GPIO_WritePin(pmod_SS_GPIO_Port, pmod_SS_Pin, GPIO_PIN_LOW);
    delay_us(5600);
    HAL_GPIO_WritePin(pmod_SS_GPIO_Port, pmod_SS_Pin, GPIO_PIN_HIGH);
    delay_us(14400);
}

double readToFDistance(void) {
    uint8_t distanceMsb = 0;
    uint8_t distanceLsb = 0;
    double distanceMeters = 1;

    // Wait for IRQ to go LOW = data ready
    while (HAL_GPIO_ReadPin(pmod_IRQ_GPIO_Port, pmod_IRQ_Pin) != GPIO_PIN_LOW);
    if (i2cRead(DIST_MSB_REG, &distanceMsb) != HAL_OK) return -1;
    printf("distanceMSB: 0x%02x\n\r", distanceMsb);
    if (i2cRead(DIST_LSB_REG, &distanceLsb) != HAL_OK) return -1;
    printf("distanceLSB: 0x%02x\n\r", distanceLsb);
    distanceMeters = (((double)distanceMsb * 256 + (double)distanceLsb) / 65536) * TOF_SCALE_METERS;
//    distanceMeters -= (TOF_OFFSET_CM / 100.0);
//    if (distanceMeters < 0) distanceMeters = 0;

    return distanceMeters;
}

void performDistanceMeasurement(void) {
    if (startToFSampling(0x7D, 0x01) != HAL_OK) return;

    while(1){
        performToFCalibration();
        double distanceMeters = readToFDistance();
        if(distanceMeters == -1) return;
        payload.timestampMS = HAL_GetTick();
        payload.distanceCM  = distanceMeters * 100.0;

        printf("Distance: %.2f cm\n\r", payload.distanceCM);
    }
}
