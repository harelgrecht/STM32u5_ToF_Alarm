#ifndef TOF_H
#define TOF_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"


#define TOF_ADDR_7BIT 0x50
#define TOF_I2C_DEV  (TOF_ADDR_7BIT << 1)
#define CMD_REG       0xB0
#define IRQ_REG       0x60
#define SOFT_START    0x49
#define DIST_MSB_REG  0xD1
#define DIST_LSB_REG  0xC2
#define SAMPLE_REG 0x13


#define GPIO_PIN_HIGH 1
#define GPIO_PIN_LOW 0

#define TOF_SCALE_METERS 33.31



typedef struct {
	uint16_t distanceCM; // Distance in cm
	uint32_t timestampMS; //time from hal_getick in ms
}distanceHandler_t;


HAL_StatusTypeDef initToF();
void performDistanceMeasurement();
HAL_StatusTypeDef preformToFCalibration();
double readToFDistance();
HAL_StatusTypeDef startToFSampling(uint8_t sampleMode, uint8_t irqMode);

#ifdef __cplusplus
}
#endif

#endif /* TOF_H */
