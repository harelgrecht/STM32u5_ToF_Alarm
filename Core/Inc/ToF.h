#ifndef TOF_H
#define TOF_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

// ==================== I2C & Device Constants ====================
#define TOF_ADDR_7BIT        0x57
#define TOF_I2C_DEV          (TOF_ADDR_7BIT << 1)

// ISL29501 Register Map (used)
#define DEVICE_ID_REG        0x00
#define MASTER_CTRL_REG      0x01
#define STATUS_REG           0x02
#define CMD_REG              0xB0
#define IRQ_REG              0x60
#define DIST_MSB_REG         0xD1
#define DIST_LSB_REG         0xD2
#define SOFT_START_REG       0x49
#define SAMPLE_REG           0x13
#define INTEGRATION_PERIOD_REG 0x10
#define SAMPLE_PERIOD_REG 0x11
#define AGC_CONTROL_REG 0x19
#define DRIVER_RANGE_REG 0x90
#define EMITTER_DAC_REG 0x91


// General GPIO logic levels
#define GPIO_PIN_HIGH        1
#define GPIO_PIN_LOW         0

// Measurement constants
#define TOF_SCALE_METERS     33.31
#define TOF_OFFSET_CM        210.0   // empirical offset correction

// Moving Average Filter
#define MOVING_AVG_SIZE 5
typedef struct {
    double history[MOVING_AVG_SIZE];
    int index;
    double sum;
} MovingAverageFilter_t;

// ==================== Data Types ====================
typedef struct {
    double distanceCM;      // Distance in centimeters
    double upTimeStamp; //timestamp sice boot in hh:mm::ss format
} distanceHandler_t;

// ==================== Public APIs ====================
HAL_StatusTypeDef initToF(void);
HAL_StatusTypeDef startToFSampling(uint8_t sampleMode, uint8_t irqMode);
void performToFCalibration(void);
double readToFDistance(void);
void performDistanceMeasurement(void);
HAL_StatusTypeDef resetToF(void);
void initMovingAverage(MovingAverageFilter_t *filter);
double updateMovingAverage(MovingAverageFilter_t *filter, double newDistance);

#ifdef __cplusplus
}
#endif

#endif /* TOF_H */
