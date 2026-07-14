#include "bmi270_stm32.h"
#include <stdlib.h>

/* ========== Forward privados ========== */
static int8_t BMI270_Begin(BMI270_Handle_t *dev);

static void BMI270_ConvertRawAccelData(BMI270_Handle_t *dev, bmi2_sens_axes_data *rawData, BMI270_SensorData *data);
static void BMI270_ConvertRawGyroData(BMI270_Handle_t *dev, bmi2_sens_axes_data *rawData, BMI270_SensorData *data);
static void BMI270_ConvertRawData(BMI270_Handle_t *dev, bmi2_sens_data *rawData, BMI270_SensorData *data);
static void BMI270_ConvertRawTemperature(uint16_t tempRaw, float *tempC);

static int8_t BMI270_ExtractFIFOData(BMI270_Handle_t *dev, BMI270_SensorData *data, bmi2_fifo_frame *fifoData, uint16_t *numFrames, uint8_t sensorSelect);

static BMI2_INTF_RETURN_TYPE BMI270_ReadRegisters(uint8_t regAddress, uint8_t *dataBuffer, uint32_t numBytes, void *interfacePtr);
static BMI2_INTF_RETURN_TYPE BMI270_ReadRegistersI2C(uint8_t regAddress, uint8_t *dataBuffer, uint32_t numBytes, BMI270_InterfaceData *interfaceData, uint32_t timeoutMs);
static BMI2_INTF_RETURN_TYPE BMI270_ReadRegistersSPI(uint8_t regAddress, uint8_t *dataBuffer, uint32_t numBytes, BMI270_InterfaceData *interfaceData, uint32_t timeoutMs);

static BMI2_INTF_RETURN_TYPE BMI270_WriteRegisters(uint8_t regAddress, const uint8_t *dataBuffer, uint32_t numBytes, void *interfacePtr);
static BMI2_INTF_RETURN_TYPE BMI270_WriteRegistersI2C(uint8_t regAddress, const uint8_t *dataBuffer, uint32_t numBytes, BMI270_InterfaceData *interfaceData, uint32_t timeoutMs);
static BMI2_INTF_RETURN_TYPE BMI270_WriteRegistersSPI(uint8_t regAddress, const uint8_t *dataBuffer, uint32_t numBytes, BMI270_InterfaceData *interfaceData, uint32_t timeoutMs);

static void BMI270_UsDelay(uint32_t period, void *interfacePtr);

static float BMI270_ConvertRawToGsScalar(uint8_t accRange);
static float BMI270_ConvertRawToDegSecScalar(uint8_t gyrRange);

static inline void BMI270_CS_LOW(BMI270_InterfaceData *itf)
{
    HAL_GPIO_WritePin(itf->spiCSPort, itf->spiCSPin, GPIO_PIN_RESET);
}
static inline void BMI270_CS_HIGH(BMI270_InterfaceData *itf)
{
    HAL_GPIO_WritePin(itf->spiCSPort, itf->spiCSPin, GPIO_PIN_SET);
}

/* ========== API ========== */

void BMI270_InitHandle(BMI270_Handle_t *dev)
{
    if (dev == NULL) return;
    memset(dev, 0, sizeof(*dev));
    dev->halTimeoutMs = 100;
}

void BMI270_SetHalTimeoutMs(BMI270_Handle_t *dev, uint32_t timeoutMs)
{
    if (dev == NULL) return;
    dev->halTimeoutMs = timeoutMs;
}

static int8_t BMI270_Begin(BMI270_Handle_t *dev)
{
    int8_t err = BMI2_OK;

    dev->sensor.read = BMI270_ReadRegisters;
    dev->sensor.write = BMI270_WriteRegisters;
    dev->sensor.delay_us = BMI270_UsDelay;
    dev->sensor.intf_ptr = dev;
    dev->sensor.read_write_len = 32;

    err = bmi270_init(&dev->sensor);
    if (err != BMI2_OK) return err;

    uint8_t features[] = { BMI2_ACCEL, BMI2_GYRO };
    err = BMI270_EnableFeatures(dev, features, 2);
    if (err != BMI2_OK) return err;

    bmi2_sens_config configs[2];
    configs[0].type = BMI2_ACCEL;
    configs[1].type = BMI2_GYRO;

    err = BMI270_GetConfigs(dev, configs, 2);
    if (err != BMI2_OK) return err;

    dev->rawToGs = BMI270_ConvertRawToGsScalar(configs[0].cfg.acc.range);
    dev->rawToDegSec = BMI270_ConvertRawToDegSecScalar(configs[1].cfg.gyr.range);

    return BMI2_OK;
}

int8_t BMI270_BeginI2C(BMI270_Handle_t *dev, uint8_t address7b, I2C_HandleTypeDef *hi2c)
{
    if ((address7b != BMI2_I2C_PRIM_ADDR) && (address7b != BMI2_I2C_SEC_ADDR))
        return BMI2_E_INVALID_INPUT;
    if ((dev == NULL) || (hi2c == NULL))
        return BMI2_E_INVALID_INPUT;

    dev->interfaceData.i2cAddress7b = address7b;
    dev->interfaceData.hi2c = hi2c;

    dev->sensor.intf = BMI2_I2C_INTF;
    dev->interfaceData.interface = BMI2_I2C_INTF;

    return BMI270_Begin(dev);
}

int8_t BMI270_BeginSPI(BMI270_Handle_t *dev, SPI_HandleTypeDef *hspi, GPIO_TypeDef *csPort, uint16_t csPin)
{
    if ((dev == NULL) || (hspi == NULL) || (csPort == NULL))
        return BMI2_E_INVALID_INPUT;

    dev->interfaceData.hspi = hspi;
    dev->interfaceData.spiCSPort = csPort;
    dev->interfaceData.spiCSPin = csPin;

    /* CS idle high */
    BMI270_CS_HIGH(&dev->interfaceData);

    dev->sensor.intf = BMI2_SPI_INTF;
    dev->interfaceData.interface = BMI2_SPI_INTF;
    dev->sensor.dummy_byte = 1;

    return BMI270_Begin(dev);
}

int8_t BMI270_Reset(BMI270_Handle_t *dev)
{
    return bmi2_soft_reset(&dev->sensor);
}

int8_t BMI270_GetStatus(BMI270_Handle_t *dev, uint8_t *status)
{
    return bmi2_get_status(status, &dev->sensor);
}

int8_t BMI270_RemapAxes(BMI270_Handle_t *dev, bmi2_remap axes)
{
    return bmi2_set_remap_axes(&axes, &dev->sensor);
}

static void BMI270_ConvertRawAccelData(BMI270_Handle_t *dev, bmi2_sens_axes_data *rawData, BMI270_SensorData *data)
{
    data->accelX = rawData->x * dev->rawToGs;
    data->accelY = rawData->y * dev->rawToGs;
    data->accelZ = rawData->z * dev->rawToGs;
}

static void BMI270_ConvertRawGyroData(BMI270_Handle_t *dev, bmi2_sens_axes_data *rawData, BMI270_SensorData *data)
{
    data->gyroX = rawData->x * dev->rawToDegSec;
    data->gyroY = rawData->y * dev->rawToDegSec;
    data->gyroZ = rawData->z * dev->rawToDegSec;
}

static void BMI270_ConvertRawData(BMI270_Handle_t *dev, bmi2_sens_data *rawData, BMI270_SensorData *data)
{
    BMI270_ConvertRawAccelData(dev, &rawData->acc, data);
    BMI270_ConvertRawGyroData(dev, &rawData->gyr, data);

    memcpy(data->auxData, rawData->aux_data, BMI2_AUX_NUM_BYTES);

    data->sensorTimeMillis = (uint32_t)(rawData->sens_time * 1000.0f * BMI2_SENSORTIME_RESOLUTION);
}

static void BMI270_ConvertRawTemperature(uint16_t tempRaw, float *tempC)
{
    *tempC = ((int16_t)tempRaw) / 512.0f + 23.0f;
}

int8_t BMI270_GetSensorData(BMI270_Handle_t *dev)
{
    int8_t err = BMI2_OK;
    bmi2_sens_data rawData;

    err = bmi2_get_sensor_data(&rawData, &dev->sensor);
    if (err != BMI2_OK) return err;

    BMI270_ConvertRawData(dev, &rawData, &dev->data);
    return BMI2_OK;
}

int8_t BMI270_GetTemperature(BMI270_Handle_t *dev, float *tempC)
{
    int8_t err = BMI2_OK;
    uint16_t tempRaw;

    err = bmi2_get_temperature_data(&tempRaw, &dev->sensor);
    if (err != BMI2_OK) return err;

    BMI270_ConvertRawTemperature(tempRaw, tempC);
    return BMI2_OK;
}

int8_t BMI270_SetAccelODR(BMI270_Handle_t *dev, uint8_t odr)
{
    int8_t err = BMI2_OK;
    bmi2_sens_config config;
    config.type = BMI2_ACCEL;

    err = BMI270_GetConfig(dev, &config);
    if (err != BMI2_OK) return err;

    config.cfg.acc.odr = odr;
    return BMI270_SetConfig(dev, config);
}

int8_t BMI270_SetGyroODR(BMI270_Handle_t *dev, uint8_t odr)
{
    int8_t err = BMI2_OK;
    bmi2_sens_config config;
    config.type = BMI2_GYRO;

    err = BMI270_GetConfig(dev, &config);
    if (err != BMI2_OK) return err;

    config.cfg.gyr.odr = odr;
    return BMI270_SetConfig(dev, config);
}

int8_t BMI270_SetAccelPowerMode(BMI270_Handle_t *dev, uint8_t filterMode)
{
    int8_t err = BMI2_OK;
    bmi2_sens_config config;
    config.type = BMI2_ACCEL;

    err = BMI270_GetConfig(dev, &config);
    if (err != BMI2_OK) return err;

    config.cfg.acc.filter_perf = filterMode;
    return BMI270_SetConfig(dev, config);
}

int8_t BMI270_SetGyroPowerMode(BMI270_Handle_t *dev, uint8_t filterMode, uint8_t noiseMode)
{
    int8_t err = BMI2_OK;
    bmi2_sens_config config;
    config.type = BMI2_GYRO;

    err = BMI270_GetConfig(dev, &config);
    if (err != BMI2_OK) return err;

    config.cfg.gyr.filter_perf = filterMode;
    config.cfg.gyr.noise_perf = noiseMode;
    return BMI270_SetConfig(dev, config);
}

int8_t BMI270_SetAccelFilterBandwidth(BMI270_Handle_t *dev, uint8_t bandwidthParam)
{
    int8_t err = BMI2_OK;
    bmi2_sens_config config;
    config.type = BMI2_ACCEL;

    err = BMI270_GetConfig(dev, &config);
    if (err != BMI2_OK) return err;

    config.cfg.acc.bwp = bandwidthParam;
    return BMI270_SetConfig(dev, config);
}

int8_t BMI270_SetGyroFilterBandwidth(BMI270_Handle_t *dev, uint8_t bandwidthParam)
{
    int8_t err = BMI2_OK;
    bmi2_sens_config config;
    config.type = BMI2_GYRO;

    err = BMI270_GetConfig(dev, &config);
    if (err != BMI2_OK) return err;

    config.cfg.gyr.bwp = bandwidthParam;
    return BMI270_SetConfig(dev, config);
}

int8_t BMI270_EnableAdvancedPowerSave(BMI270_Handle_t *dev, bool enable)
{
    return bmi2_set_adv_power_save(enable ? BMI2_ENABLE : BMI2_DISABLE, &dev->sensor);
}

int8_t BMI270_DisableAdvancedPowerSave(BMI270_Handle_t *dev)
{
    return BMI270_EnableAdvancedPowerSave(dev, false);
}

int8_t BMI270_SetConfigs(BMI270_Handle_t *dev, bmi2_sens_config *configs, uint8_t numConfigs)
{
    for (uint8_t i = 0; i < numConfigs; i++)
    {
        if (configs[i].type == BMI2_ACCEL)
            dev->rawToGs = BMI270_ConvertRawToGsScalar(configs[i].cfg.acc.range);
        else if (configs[i].type == BMI2_GYRO)
            dev->rawToDegSec = BMI270_ConvertRawToDegSecScalar(configs[i].cfg.gyr.range);
    }

    return bmi270_set_sensor_config(configs, numConfigs, &dev->sensor);
}

int8_t BMI270_SetConfig(BMI270_Handle_t *dev, bmi2_sens_config config)
{
    return BMI270_SetConfigs(dev, &config, 1);
}

int8_t BMI270_GetConfigs(BMI270_Handle_t *dev, bmi2_sens_config *configs, uint8_t numConfigs)
{
    return bmi270_get_sensor_config(configs, numConfigs, &dev->sensor);
}

int8_t BMI270_GetConfig(BMI270_Handle_t *dev, bmi2_sens_config *config)
{
    return BMI270_GetConfigs(dev, config, 1);
}

int8_t BMI270_EnableFeatures(BMI270_Handle_t *dev, uint8_t *features, uint8_t numFeatures)
{
    return bmi270_sensor_enable(features, numFeatures, &dev->sensor);
}

int8_t BMI270_EnableFeature(BMI270_Handle_t *dev, uint8_t feature)
{
    return BMI270_EnableFeatures(dev, &feature, 1);
}

int8_t BMI270_DisableFeatures(BMI270_Handle_t *dev, uint8_t *features, uint8_t numFeatures)
{
    return bmi270_sensor_disable(features, numFeatures, &dev->sensor);
}

int8_t BMI270_DisableFeature(BMI270_Handle_t *dev, uint8_t feature)
{
    return BMI270_DisableFeatures(dev, &feature, 1);
}

int8_t BMI270_GetFeatureDataN(BMI270_Handle_t *dev, bmi2_feat_sensor_data *featureData, uint8_t numFeatures)
{
    return bmi270_get_feature_data(featureData, numFeatures, &dev->sensor);
}

int8_t BMI270_GetFeatureData(BMI270_Handle_t *dev, bmi2_feat_sensor_data *featureData)
{
    return BMI270_GetFeatureDataN(dev, featureData, 1);
}

int8_t BMI270_MapInterruptToPin(BMI270_Handle_t *dev, uint8_t interruptSource, bmi2_hw_int_pin pin)
{
    switch (interruptSource)
    {
        case BMI2_FFULL_INT:
        case BMI2_FWM_INT:
        case BMI2_DRDY_INT:
        case BMI2_ERR_INT:
            return bmi2_map_data_int(interruptSource, pin, &dev->sensor);

        case BMI2_SIG_MOTION_INT:
        case BMI2_WRIST_GESTURE_INT:
        case BMI2_ANY_MOTION_INT:
        case BMI2_NO_MOTION_INT:
        case BMI2_STEP_COUNTER_INT:
        case BMI2_STEP_DETECTOR_INT:
        case BMI2_STEP_ACTIVITY_INT:
        case BMI2_WRIST_WEAR_WAKE_UP_INT:
            interruptSource -= BMI2_FEATURE_DATA_OFFSET;
            return bmi2_map_feat_int(interruptSource, pin, &dev->sensor);

        default:
            return BMI2_E_INVALID_INPUT;
    }
}

int8_t BMI270_SetInterruptPinConfig(BMI270_Handle_t *dev, bmi2_int_pin_config config)
{
    return bmi2_set_int_pin_config(&config, &dev->sensor);
}

int8_t BMI270_GetInterruptPinConfig(BMI270_Handle_t *dev, bmi2_int_pin_config *config)
{
    return bmi2_get_int_pin_config(config, &dev->sensor);
}

int8_t BMI270_GetInterruptStatus(BMI270_Handle_t *dev, uint16_t *status)
{
    return bmi2_get_int_status(status, &dev->sensor);
}

int8_t BMI270_SetFIFOConfig(BMI270_Handle_t *dev, BMI270_FIFOConfig config)
{
    int8_t err = BMI2_OK;

    err = BMI270_SetFIFOFlags(dev, config.flags, true);
    if (err != BMI2_OK) return err;

    err = BMI270_SetFIFOFlags(dev, (uint16_t)(~config.flags), false);
    if (err != BMI2_OK) return err;

    err = BMI270_SetFIFODownSample(dev, BMI2_ACCEL, config.accelDownSample);
    if (err != BMI2_OK) return err;

    err = BMI270_SetFIFODownSample(dev, BMI2_GYRO, config.gyroDownSample);
    if (err != BMI2_OK) return err;

    err = BMI270_SetFIFOFilter(dev, BMI2_ACCEL, config.accelFilter);
    if (err != BMI2_OK) return err;

    err = BMI270_SetFIFOFilter(dev, BMI2_GYRO, config.gyroFilter);
    if (err != BMI2_OK) return err;

    err = BMI270_SetFIFOSelfWakeup(dev, config.selfWakeUp);
    if (err != BMI2_OK) return err;

    err = BMI270_SetFIFOWatermark(dev, config.watermark);
    if (err != BMI2_OK) return err;

    return BMI2_OK;
}

int8_t BMI270_SetFIFOFlags(BMI270_Handle_t *dev, uint16_t flags, bool enable)
{
    int8_t err = bmi2_set_fifo_config(flags, enable, &dev->sensor);
    if (err != BMI2_OK) return err;

    if (enable) dev->fifoConfigFlags |= flags;
    else dev->fifoConfigFlags &= (uint16_t)(~flags);

    dev->bytesPerFIFOData = 0;
    dev->bytesPerFIFOData += ((dev->fifoConfigFlags & BMI2_FIFO_ACC_EN) != 0U) ? 6U : 0U;
    dev->bytesPerFIFOData += ((dev->fifoConfigFlags & BMI2_FIFO_GYR_EN) != 0U) ? 6U : 0U;
    dev->bytesPerFIFOData += ((dev->fifoConfigFlags & BMI2_FIFO_HEADER_EN) != 0U) ? 1U : 0U;

    return BMI2_OK;
}

int8_t BMI270_SetFIFODownSample(BMI270_Handle_t *dev, uint8_t sensorSelect, uint8_t downSample)
{
    return bmi2_set_fifo_down_sample(sensorSelect, downSample, &dev->sensor);
}

int8_t BMI270_SetFIFOFilter(BMI270_Handle_t *dev, uint8_t sensorSelect, bool filter)
{
    return bmi2_set_fifo_filter_data(sensorSelect, filter ? BMI2_ENABLE : BMI2_DISABLE, &dev->sensor);
}

int8_t BMI270_SetFIFOSelfWakeup(BMI270_Handle_t *dev, bool selfWakeUp)
{
    return bmi2_set_fifo_self_wake_up(selfWakeUp ? BMI2_ENABLE : BMI2_DISABLE, &dev->sensor);
}

int8_t BMI270_SetFIFOWatermark(BMI270_Handle_t *dev, uint16_t numData)
{
    if (dev->bytesPerFIFOData == 0U) return BMI2_E_INVALID_STATUS;
    return bmi2_set_fifo_wm((uint16_t)(numData * dev->bytesPerFIFOData), &dev->sensor);
}

int8_t BMI270_GetFIFOLengthBytes(BMI270_Handle_t *dev, uint16_t *length)
{
    return bmi2_get_fifo_length(length, &dev->sensor);
}

int8_t BMI270_GetFIFOLength(BMI270_Handle_t *dev, uint16_t *length)
{
    int8_t err = BMI270_GetFIFOLengthBytes(dev, length);
    if (err != BMI2_OK) return err;
    if (dev->bytesPerFIFOData == 0U) return BMI2_E_INVALID_STATUS;

    *length = (uint16_t)(*length / dev->bytesPerFIFOData);
    return BMI2_OK;
}

static int8_t BMI270_ExtractFIFOData(BMI270_Handle_t *dev, BMI270_SensorData *data, bmi2_fifo_frame *fifoData, uint16_t *numFrames, uint8_t sensorSelect)
{
    if (((sensorSelect == BMI2_ACCEL) && ((dev->fifoConfigFlags & BMI2_FIFO_ACC_EN) == 0U)) ||
        ((sensorSelect == BMI2_GYRO)  && ((dev->fifoConfigFlags & BMI2_FIFO_GYR_EN) == 0U)))
    {
        return BMI2_OK;
    }

    int8_t err;
    bmi2_sens_axes_data *rawData = (bmi2_sens_axes_data *)malloc((*numFrames) * sizeof(bmi2_sens_axes_data));
    if (rawData == NULL) return BMI2_E_COM_FAIL;

    if (sensorSelect == BMI2_ACCEL) err = bmi2_extract_accel(rawData, numFrames, fifoData, &dev->sensor);
    else                            err = bmi2_extract_gyro(rawData, numFrames, fifoData, &dev->sensor);

    if (err < BMI2_OK)
    {
        free(rawData);
        return err;
    }

    for (uint16_t i = 0; i < *numFrames; i++)
    {
        if (sensorSelect == BMI2_ACCEL) BMI270_ConvertRawAccelData(dev, &rawData[i], &data[i]);
        else                            BMI270_ConvertRawGyroData(dev, &rawData[i], &data[i]);
    }

    free(rawData);
    return err;
}

int8_t BMI270_GetFIFOData(BMI270_Handle_t *dev, BMI270_SensorData *data, uint16_t *numData)
{
    int8_t err = BMI2_OK;
    uint16_t numFIFOBytes = 0;

    err = BMI270_GetFIFOLengthBytes(dev, &numFIFOBytes);
    if (err != BMI2_OK) return err;
    if (numFIFOBytes == 0U) { *numData = 0; return BMI2_OK; }
    if (dev->bytesPerFIFOData == 0U) return BMI2_E_INVALID_STATUS;

    uint8_t *fifoBuffer = (uint8_t *)malloc(numFIFOBytes + dev->sensor.dummy_byte);
    if (fifoBuffer == NULL) return BMI2_E_COM_FAIL;

    uint16_t numFrames16 = (uint16_t)(numFIFOBytes / dev->bytesPerFIFOData);

    if (*numData > numFrames16) *numData = numFrames16;

    bmi2_fifo_frame fifoData;
    fifoData.length = (uint16_t)(numFIFOBytes + dev->sensor.dummy_byte);
    fifoData.data = fifoBuffer;

    err = bmi2_read_fifo_data(&fifoData, &dev->sensor);
    if (err != BMI2_OK)
    {
        free(fifoBuffer);
        return err;
    }

    err = BMI270_ExtractFIFOData(dev, data, &fifoData, numData, BMI2_ACCEL);
    if (err < BMI2_OK)
    {
        free(fifoBuffer);
        return err;
    }

    err = BMI270_ExtractFIFOData(dev, data, &fifoData, numData, BMI2_GYRO);
    if (err < BMI2_OK)
    {
        free(fifoBuffer);
        return err;
    }

    free(fifoBuffer);
    return BMI2_OK;
}

int8_t BMI270_FlushFIFO(BMI270_Handle_t *dev)
{
    return bmi2_set_command_register(BMI2_FIFO_FLUSH_CMD, &dev->sensor);
}

int8_t BMI270_GetStepCount(BMI270_Handle_t *dev, uint32_t *count)
{
    int8_t err;
    bmi2_feat_sensor_data featureData;
    featureData.type = BMI2_STEP_COUNTER;

    err = BMI270_GetFeatureData(dev, &featureData);
    if (err) return err;

    *count = featureData.sens_data.step_counter_output;
    return BMI2_OK;
}

int8_t BMI270_ResetStepCount(BMI270_Handle_t *dev)
{
    int8_t err;
    bmi2_sens_config config;
    config.type = BMI2_STEP_COUNTER;

    err = BMI270_GetConfig(dev, &config);
    if (err) return err;

    config.cfg.step_counter.reset_counter = true;
    return BMI270_SetConfig(dev, config);
}

int8_t BMI270_SetStepCountWatermark(BMI270_Handle_t *dev, uint16_t watermark)
{
    int8_t err;
    bmi2_sens_config config;
    config.type = BMI2_STEP_COUNTER;

    err = BMI270_GetConfig(dev, &config);
    if (err) return err;

    config.cfg.step_counter.watermark_level = watermark;
    return BMI270_SetConfig(dev, config);
}

int8_t BMI270_GetStepActivity(BMI270_Handle_t *dev, uint8_t *activity)
{
    int8_t err;
    bmi2_feat_sensor_data featureData;
    featureData.type = BMI2_STEP_ACTIVITY;

    err = BMI270_GetFeatureData(dev, &featureData);
    if (err) return err;

    *activity = featureData.sens_data.activity_output;
    return BMI2_OK;
}

int8_t BMI270_GetWristGesture(BMI270_Handle_t *dev, uint8_t *gesture)
{
    int8_t err;
    bmi2_feat_sensor_data featureData;
    featureData.type = BMI2_WRIST_GESTURE;

    err = BMI270_GetFeatureData(dev, &featureData);
    if (err) return err;

    *gesture = featureData.sens_data.wrist_gesture_output;
    return BMI2_OK;
}

int8_t BMI270_PerformAccelOffsetCalibration(BMI270_Handle_t *dev, uint8_t gravityDirection)
{
    bmi2_accel_foc_g_value gDir;
    gDir.x = ((gravityDirection & BMI2_GRAVITY_X) != 0U);
    gDir.y = ((gravityDirection & BMI2_GRAVITY_Y) != 0U);
    gDir.z = ((gravityDirection & BMI2_GRAVITY_Z) != 0U);
    gDir.sign = ((gravityDirection & BMI2_GRAVITY_POS) != 0U);

    return bmi2_perform_accel_foc(&gDir, &dev->sensor);
}

int8_t BMI270_PerformGyroOffsetCalibration(BMI270_Handle_t *dev)
{
    return bmi2_perform_gyro_foc(&dev->sensor);
}

int8_t BMI270_PerformComponentRetrim(BMI270_Handle_t *dev)
{
    int8_t err = bmi2_do_crt(&dev->sensor);
    if (err != BMI2_OK) return err;

    return BMI270_EnableFeature(dev, BMI2_GYRO);
}

int8_t BMI270_SaveNVM(BMI270_Handle_t *dev)
{
    return bmi2_nvm_prog(&dev->sensor);
}

int8_t BMI270_SelfTest(BMI270_Handle_t *dev)
{
    int8_t err = bmi2_perform_accel_self_test(&dev->sensor);
    if (err != BMI2_OK) return err;

    err = BMI270_Reset(dev);
    if (err != BMI2_OK) return err;

    err = bmi2_do_gyro_st(&dev->sensor);
    if (err != BMI2_OK) return err;

    return BMI270_Begin(dev);
}

int8_t BMI270_SetAuxPullUps(BMI270_Handle_t *dev, uint8_t pullUpValue)
{
    return BMI270_WriteRegisters(BMI2_AUX_IF_TRIM, &pullUpValue, 1, dev);
}

int8_t BMI270_ReadAux(BMI270_Handle_t *dev, uint8_t addr, uint8_t numBytes)
{
    return bmi2_read_aux_man_mode(addr, dev->data.auxData, numBytes, &dev->sensor);
}

int8_t BMI270_WriteAuxN(BMI270_Handle_t *dev, uint8_t addr, uint8_t *data, uint8_t numBytes)
{
    return bmi2_write_aux_man_mode(addr, data, numBytes, &dev->sensor);
}

int8_t BMI270_WriteAux(BMI270_Handle_t *dev, uint8_t addr, uint8_t data)
{
    return BMI270_WriteAuxN(dev, addr, &data, 1);
}

/* ========== Callbacks Bosch ========== */

static BMI2_INTF_RETURN_TYPE BMI270_ReadRegisters(uint8_t regAddress, uint8_t *dataBuffer, uint32_t numBytes, void *interfacePtr)
{
    if ((numBytes == 0U) || (interfacePtr == NULL)) return BMI2_E_COM_FAIL;

    BMI270_Handle_t *dev = (BMI270_Handle_t *)interfacePtr;
    BMI270_InterfaceData *itf = &dev->interfaceData;

    switch (itf->interface)
    {
        case BMI2_I2C_INTF:
            return BMI270_ReadRegistersI2C(regAddress, dataBuffer, numBytes, itf, dev->halTimeoutMs);
        case BMI2_SPI_INTF:
            return BMI270_ReadRegistersSPI(regAddress, dataBuffer, numBytes, itf, dev->halTimeoutMs);
        default:
            return BMI2_E_COM_FAIL;
    }
}

static BMI2_INTF_RETURN_TYPE BMI270_ReadRegistersI2C(uint8_t regAddress, uint8_t *dataBuffer, uint32_t numBytes, BMI270_InterfaceData *interfaceData, uint32_t timeoutMs)
{
    HAL_StatusTypeDef st = HAL_I2C_Mem_Read(interfaceData->hi2c,
                                            (uint16_t)(interfaceData->i2cAddress7b << 1),
                                            regAddress,
                                            I2C_MEMADD_SIZE_8BIT,
                                            dataBuffer,
                                            (uint16_t)numBytes,
                                            timeoutMs);
    return (st == HAL_OK) ? BMI2_OK : BMI2_E_COM_FAIL;
}

static BMI2_INTF_RETURN_TYPE BMI270_ReadRegistersSPI(uint8_t regAddress, uint8_t *dataBuffer, uint32_t numBytes, BMI270_InterfaceData *interfaceData, uint32_t timeoutMs)
{
    uint8_t addr = (uint8_t)(regAddress | 0x80U); /* read bit */
    HAL_StatusTypeDef st;

    BMI270_CS_LOW(interfaceData);

    st = HAL_SPI_Transmit(interfaceData->hspi, &addr, 1, timeoutMs);
    if (st == HAL_OK) st = HAL_SPI_Receive(interfaceData->hspi, dataBuffer, (uint16_t)numBytes, timeoutMs);

    BMI270_CS_HIGH(interfaceData);
    return (st == HAL_OK) ? BMI2_OK : BMI2_E_COM_FAIL;
}

static BMI2_INTF_RETURN_TYPE BMI270_WriteRegisters(uint8_t regAddress, const uint8_t *dataBuffer, uint32_t numBytes, void *interfacePtr)
{
    if ((numBytes == 0U) || (interfacePtr == NULL)) return BMI2_E_COM_FAIL;

    BMI270_Handle_t *dev = (BMI270_Handle_t *)interfacePtr;
    BMI270_InterfaceData *itf = &dev->interfaceData;

    switch (itf->interface)
    {
        case BMI2_I2C_INTF:
            return BMI270_WriteRegistersI2C(regAddress, dataBuffer, numBytes, itf, dev->halTimeoutMs);
        case BMI2_SPI_INTF:
            return BMI270_WriteRegistersSPI(regAddress, dataBuffer, numBytes, itf, dev->halTimeoutMs);
        default:
            return BMI2_E_COM_FAIL;
    }
}

static BMI2_INTF_RETURN_TYPE BMI270_WriteRegistersI2C(uint8_t regAddress, const uint8_t *dataBuffer, uint32_t numBytes, BMI270_InterfaceData *interfaceData, uint32_t timeoutMs)
{
    HAL_StatusTypeDef st = HAL_I2C_Mem_Write(interfaceData->hi2c,
                                             (uint16_t)(interfaceData->i2cAddress7b << 1),
                                             regAddress,
                                             I2C_MEMADD_SIZE_8BIT,
                                             (uint8_t *)dataBuffer,
                                             (uint16_t)numBytes,
                                             timeoutMs);
    return (st == HAL_OK) ? BMI2_OK : BMI2_E_COM_FAIL;
}

static BMI2_INTF_RETURN_TYPE BMI270_WriteRegistersSPI(uint8_t regAddress, const uint8_t *dataBuffer, uint32_t numBytes, BMI270_InterfaceData *interfaceData, uint32_t timeoutMs)
{
    HAL_StatusTypeDef st;

    BMI270_CS_LOW(interfaceData);

    st = HAL_SPI_Transmit(interfaceData->hspi, &regAddress, 1, timeoutMs);
    if (st == HAL_OK) st = HAL_SPI_Transmit(interfaceData->hspi, (uint8_t *)dataBuffer, (uint16_t)numBytes, timeoutMs);

    BMI270_CS_HIGH(interfaceData);
    return (st == HAL_OK) ? BMI2_OK : BMI2_E_COM_FAIL;
}

/* Delay callback:
 * Para secuencias de init funciona con HAL_Delay.
 * Si quieres precisión real en us para F411, te paso versión DWT en otro mensaje.
 */
static void BMI270_UsDelay(uint32_t period, void *interfacePtr)
{
    (void)interfacePtr;
    if (period == 0U) return;

    if (period < 1000U) HAL_Delay(1);
    else HAL_Delay(period / 1000U);
}

/* Helpers escala */
static float BMI270_ConvertRawToGsScalar(uint8_t accRange)
{
    return ((float)(2U << accRange) / 32768.0f);
}
static float BMI270_ConvertRawToDegSecScalar(uint8_t gyrRange)
{
    return ((float)(125U * (1U << (BMI2_GYR_RANGE_125 - gyrRange))) / 32768.0f);
}
