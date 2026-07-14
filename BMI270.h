#ifndef BMI270_STM32_H
#define BMI270_STM32_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include <stdint.h>
#include <stdbool.h>
#include <string.h>

#include "bmi270_api/bmi270.h"

/* ========== Compat macros de la librería Arduino ========== */

/* Distinción de sources data vs feature en mapInterruptToPin */
#define BMI2_FEATURE_DATA_OFFSET UINT8_C(128)
#define BMI2_SIG_MOTION_INT         (BMI2_SIG_MOTION + BMI2_FEATURE_DATA_OFFSET)
#define BMI2_WRIST_GESTURE_INT      (BMI2_WRIST_GESTURE + BMI2_FEATURE_DATA_OFFSET)
#define BMI2_ANY_MOTION_INT         (BMI2_ANY_MOTION + BMI2_FEATURE_DATA_OFFSET)
#define BMI2_NO_MOTION_INT          (BMI2_NO_MOTION + BMI2_FEATURE_DATA_OFFSET)
#define BMI2_STEP_COUNTER_INT       (BMI2_STEP_COUNTER + BMI2_FEATURE_DATA_OFFSET)
#define BMI2_STEP_DETECTOR_INT      (BMI2_STEP_DETECTOR + BMI2_FEATURE_DATA_OFFSET)
#define BMI2_STEP_ACTIVITY_INT      (BMI2_STEP_ACTIVITY + BMI2_FEATURE_DATA_OFFSET)
#define BMI2_WRIST_WEAR_WAKE_UP_INT (BMI2_WRIST_WEAR_WAKE_UP + BMI2_FEATURE_DATA_OFFSET)

/* FIFO down sampling factors */
#define BMI2_FIFO_DOWN_SAMPLE_1   UINT8_C(0)
#define BMI2_FIFO_DOWN_SAMPLE_2   UINT8_C(1)
#define BMI2_FIFO_DOWN_SAMPLE_4   UINT8_C(2)
#define BMI2_FIFO_DOWN_SAMPLE_8   UINT8_C(3)
#define BMI2_FIFO_DOWN_SAMPLE_16  UINT8_C(4)
#define BMI2_FIFO_DOWN_SAMPLE_32  UINT8_C(5)
#define BMI2_FIFO_DOWN_SAMPLE_64  UINT8_C(6)
#define BMI2_FIFO_DOWN_SAMPLE_128 UINT8_C(7)

/* Macros para accel FOC */
#define BMI2_GRAVITY_X   UINT8_C(0x01)
#define BMI2_GRAVITY_Y   UINT8_C(0x02)
#define BMI2_GRAVITY_Z   UINT8_C(0x04)
#define BMI2_GRAVITY_POS UINT8_C(0x08)
#define BMI2_GRAVITY_POS_X (BMI2_GRAVITY_X | BMI2_GRAVITY_POS)
#define BMI2_GRAVITY_POS_Y (BMI2_GRAVITY_Y | BMI2_GRAVITY_POS)
#define BMI2_GRAVITY_POS_Z (BMI2_GRAVITY_Z | BMI2_GRAVITY_POS)
#define BMI2_GRAVITY_NEG_X (BMI2_GRAVITY_X)
#define BMI2_GRAVITY_NEG_Y (BMI2_GRAVITY_Y)
#define BMI2_GRAVITY_NEG_Z (BMI2_GRAVITY_Z)

/* Alias más claros para axis remap */
#define BMI2_AXIS_POS_X (BMI2_X)
#define BMI2_AXIS_NEG_X (BMI2_NEG_X)
#define BMI2_AXIS_POS_Y (BMI2_Y)
#define BMI2_AXIS_NEG_Y (BMI2_NEG_Y)
#define BMI2_AXIS_POS_Z (BMI2_Z)
#define BMI2_AXIS_NEG_Z (BMI2_NEG_Z)

/* Step activity */
#define BMI2_STEP_ACTIVITY_STILL   UINT8_C(0)
#define BMI2_STEP_ACTIVITY_WALKING UINT8_C(1)
#define BMI2_STEP_ACTIVITY_RUNNING UINT8_C(2)
#define BMI2_STEP_ACTIVITY_UNKNOWN UINT8_C(3)

/* Wrist gestures */
#define BMI2_WRIST_GESTURE_UNKNOWN      UINT8_C(0)
#define BMI2_WRIST_GESTURE_ARM_DOWN     UINT8_C(1)
#define BMI2_WRIST_GESTURE_ARM_UP       UINT8_C(2)
#define BMI2_WRIST_GESTURE_SHAKE_JIGGLE UINT8_C(3)
#define BMI2_WRIST_GESTURE_FLICK_IN     UINT8_C(4)
#define BMI2_WRIST_GESTURE_FLICK_OUT    UINT8_C(5)

/* ========== Tipos ========== */

typedef enum
{
    BMI270_STM32_INTF_I2C = 0,
    BMI270_STM32_INTF_SPI = 1
} BMI270_STM32_InterfaceType;

/* Equivalente de BMI270_InterfaceData */
typedef struct
{
    bmi2_intf interface;

    /* I2C */
    uint8_t i2cAddress7b;
    I2C_HandleTypeDef *hi2c;

    /* SPI */
    SPI_HandleTypeDef *hspi;
    GPIO_TypeDef *spiCSPort;
    uint16_t spiCSPin;
} BMI270_InterfaceData;

/* Equivalente de BMI270_SensorData */
typedef struct
{
    float accelX;
    float accelY;
    float accelZ;

    float gyroX;
    float gyroY;
    float gyroZ;

    uint8_t auxData[BMI2_AUX_NUM_BYTES];

    uint32_t sensorTimeMillis;
} BMI270_SensorData;

/* Equivalente de BMI270_FIFOConfig */
typedef struct
{
    uint16_t flags;
    uint16_t watermark;
    uint8_t accelDownSample;
    uint8_t gyroDownSample;
    bool accelFilter;
    bool gyroFilter;
    bool selfWakeUp;
} BMI270_FIFOConfig;

/* Contexto principal (equivalente a clase BMI270) */
typedef struct
{
    struct bmi2_dev sensor;
    BMI270_InterfaceData interfaceData;
    BMI270_SensorData data;

    float rawToGs;
    float rawToDegSec;

    uint16_t fifoConfigFlags;
    uint8_t bytesPerFIFOData;

    /* Timeouts HAL */
    uint32_t halTimeoutMs;
} BMI270_Handle_t;

/* ========== API pública (equivalente 1:1) ========== */

/* Constructor-style */
void BMI270_InitHandle(BMI270_Handle_t *dev);

/* Begin */
int8_t BMI270_BeginI2C(BMI270_Handle_t *dev, uint8_t address7b, I2C_HandleTypeDef *hi2c);
int8_t BMI270_BeginSPI(BMI270_Handle_t *dev, SPI_HandleTypeDef *hspi, GPIO_TypeDef *csPort, uint16_t csPin);

/* Sensor control */
int8_t BMI270_Reset(BMI270_Handle_t *dev);
int8_t BMI270_GetStatus(BMI270_Handle_t *dev, uint8_t *status);
int8_t BMI270_RemapAxes(BMI270_Handle_t *dev, bmi2_remap axes);

/* Data acquisition */
int8_t BMI270_GetSensorData(BMI270_Handle_t *dev);
int8_t BMI270_GetTemperature(BMI270_Handle_t *dev, float *tempC);

/* Accel/Gyro params */
int8_t BMI270_SetAccelODR(BMI270_Handle_t *dev, uint8_t odr);
int8_t BMI270_SetGyroODR(BMI270_Handle_t *dev, uint8_t odr);
int8_t BMI270_SetAccelPowerMode(BMI270_Handle_t *dev, uint8_t filterMode);
int8_t BMI270_SetGyroPowerMode(BMI270_Handle_t *dev, uint8_t filterMode, uint8_t noiseMode);
int8_t BMI270_SetAccelFilterBandwidth(BMI270_Handle_t *dev, uint8_t bandwidthParam);
int8_t BMI270_SetGyroFilterBandwidth(BMI270_Handle_t *dev, uint8_t bandwidthParam);

/* Advanced power save */
int8_t BMI270_EnableAdvancedPowerSave(BMI270_Handle_t *dev, bool enable);
int8_t BMI270_DisableAdvancedPowerSave(BMI270_Handle_t *dev);

/* Config control */
int8_t BMI270_SetConfigs(BMI270_Handle_t *dev, bmi2_sens_config *configs, uint8_t numConfigs);
int8_t BMI270_SetConfig(BMI270_Handle_t *dev, bmi2_sens_config config);
int8_t BMI270_GetConfigs(BMI270_Handle_t *dev, bmi2_sens_config *configs, uint8_t numConfigs);
int8_t BMI270_GetConfig(BMI270_Handle_t *dev, bmi2_sens_config *config);

/* Feature enable/disable */
int8_t BMI270_EnableFeatures(BMI270_Handle_t *dev, uint8_t *features, uint8_t numFeatures);
int8_t BMI270_EnableFeature(BMI270_Handle_t *dev, uint8_t feature);
int8_t BMI270_DisableFeatures(BMI270_Handle_t *dev, uint8_t *features, uint8_t numFeatures);
int8_t BMI270_DisableFeature(BMI270_Handle_t *dev, uint8_t feature);

/* Feature data */
int8_t BMI270_GetFeatureDataN(BMI270_Handle_t *dev, bmi2_feat_sensor_data *featureData, uint8_t numFeatures);
int8_t BMI270_GetFeatureData(BMI270_Handle_t *dev, bmi2_feat_sensor_data *featureData);

/* Interrupt control */
int8_t BMI270_MapInterruptToPin(BMI270_Handle_t *dev, uint8_t interruptSource, bmi2_hw_int_pin pin);
int8_t BMI270_SetInterruptPinConfig(BMI270_Handle_t *dev, bmi2_int_pin_config config);
int8_t BMI270_GetInterruptPinConfig(BMI270_Handle_t *dev, bmi2_int_pin_config *config);
int8_t BMI270_GetInterruptStatus(BMI270_Handle_t *dev, uint16_t *status);

/* FIFO control */
int8_t BMI270_SetFIFOConfig(BMI270_Handle_t *dev, BMI270_FIFOConfig config);
int8_t BMI270_SetFIFOFlags(BMI270_Handle_t *dev, uint16_t flags, bool enable);
int8_t BMI270_SetFIFODownSample(BMI270_Handle_t *dev, uint8_t sensorSelect, uint8_t downSample);
int8_t BMI270_SetFIFOFilter(BMI270_Handle_t *dev, uint8_t sensorSelect, bool filter);
int8_t BMI270_SetFIFOSelfWakeup(BMI270_Handle_t *dev, bool selfWakeUp);
int8_t BMI270_SetFIFOWatermark(BMI270_Handle_t *dev, uint16_t numData);
int8_t BMI270_GetFIFOLengthBytes(BMI270_Handle_t *dev, uint16_t *length);
int8_t BMI270_GetFIFOLength(BMI270_Handle_t *dev, uint16_t *length);
int8_t BMI270_GetFIFOData(BMI270_Handle_t *dev, BMI270_SensorData *data, uint16_t *numData);
int8_t BMI270_FlushFIFO(BMI270_Handle_t *dev);

/* Step / activity / wrist */
int8_t BMI270_GetStepCount(BMI270_Handle_t *dev, uint32_t *count);
int8_t BMI270_ResetStepCount(BMI270_Handle_t *dev);
int8_t BMI270_SetStepCountWatermark(BMI270_Handle_t *dev, uint16_t watermark);
int8_t BMI270_GetStepActivity(BMI270_Handle_t *dev, uint8_t *activity);
int8_t BMI270_GetWristGesture(BMI270_Handle_t *dev, uint8_t *gesture);

/* Calibration / self test */
int8_t BMI270_PerformAccelOffsetCalibration(BMI270_Handle_t *dev, uint8_t gravityDirection);
int8_t BMI270_PerformGyroOffsetCalibration(BMI270_Handle_t *dev);
int8_t BMI270_PerformComponentRetrim(BMI270_Handle_t *dev);
int8_t BMI270_SaveNVM(BMI270_Handle_t *dev);
int8_t BMI270_SelfTest(BMI270_Handle_t *dev);

/* AUX */
int8_t BMI270_SetAuxPullUps(BMI270_Handle_t *dev, uint8_t pullUpValue);
int8_t BMI270_ReadAux(BMI270_Handle_t *dev, uint8_t addr, uint8_t numBytes);
int8_t BMI270_WriteAuxN(BMI270_Handle_t *dev, uint8_t addr, uint8_t *data, uint8_t numBytes);
int8_t BMI270_WriteAux(BMI270_Handle_t *dev, uint8_t addr, uint8_t data);

/* Opcional: timeout HAL */
void BMI270_SetHalTimeoutMs(BMI270_Handle_t *dev, uint32_t timeoutMs);

#ifdef __cplusplus
}
#endif

#endif /* BMI270_STM32_H */
