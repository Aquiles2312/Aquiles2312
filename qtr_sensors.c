/**
 * @file    qtr_sensors.c
 * @brief   STM32F411RE pure C port of the Pololu QTR8A reflectance sensor library.
 *
 * All Arduino-specific APIs have been replaced with STM32 HAL equivalents.
 * ADC conversions use HAL_ADC_PollForConversion(); emitter control uses a
 * standard HAL GPIO write.
 */

#include "qtr_sensors.h"

#include <stdlib.h>
#include <string.h>
#include <stdint.h>

/* ================================================================== */
/* Internal helpers                                                     */
/* ================================================================== */

/**
 * @brief Allocate (or re-allocate) the min/max arrays inside a CalibrationData.
 *
 * Safe to call multiple times – frees existing arrays before re-allocating.
 *
 * @param  cal   Pointer to CalibrationData to initialise.
 * @param  count Number of sensors.
 */
static void _calibration_init(CalibrationData *cal, uint8_t count)
{
    if (cal->initialized) {
        free(cal->minimum);
        free(cal->maximum);
    }

    cal->minimum = (uint16_t *)malloc(count * sizeof(uint16_t));
    cal->maximum = (uint16_t *)malloc(count * sizeof(uint16_t));
    cal->count   = count;

    if (cal->minimum && cal->maximum) {
        /* Seed min with the largest possible value, max with 0. */
        for (uint8_t i = 0; i < count; i++) {
            cal->minimum[i] = QTR_ADC_MAX;
            cal->maximum[i] = 0U;
        }
        cal->initialized = 1U;
    } else {
        /* Allocation failure – free whatever succeeded and mark uninitialised. */
        free(cal->minimum);
        free(cal->maximum);
        cal->minimum     = NULL;
        cal->maximum     = NULL;
        cal->initialized = 0U;
    }
}

/**
 * @brief Free the arrays inside a CalibrationData and reset it.
 * @param  cal  Pointer to CalibrationData to free.
 */
static void _calibration_free(CalibrationData *cal)
{
    if (cal->initialized) {
        free(cal->minimum);
        free(cal->maximum);
        cal->minimum     = NULL;
        cal->maximum     = NULL;
        cal->count       = 0U;
        cal->initialized = 0U;
    }
}

/**
 * @brief Read a single ADC sample from one sensor channel.
 *
 * Configures the ADC channel for a regular conversion, starts, polls for
 * completion, and returns the raw 12-bit result.
 *
 * @param  ch  Pointer to the QTRSensorChannel descriptor.
 * @return 12-bit ADC result (0 – 4095), or 0 on timeout.
 */
static uint16_t _read_adc_channel(const QTRSensorChannel *ch)
{
    ADC_ChannelConfTypeDef cfg = {0};
    cfg.Channel      = ch->channel;
    cfg.Rank         = ch->rank;
    cfg.SamplingTime = ADC_SAMPLETIME_84CYCLES; /* ~3 us at 84 MHz ADCCLK */

    if (HAL_ADC_ConfigChannel(ch->hadc, &cfg) != HAL_OK) {
        return 0U;
    }

    HAL_ADC_Start(ch->hadc);
    if (HAL_ADC_PollForConversion(ch->hadc, 10U) != HAL_OK) {
        HAL_ADC_Stop(ch->hadc);
        return 0U;
    }

    uint16_t value = (uint16_t)HAL_ADC_GetValue(ch->hadc);
    HAL_ADC_Stop(ch->hadc);
    return value;
}

/**
 * @brief Read raw ADC values for all sensors using the given emitter state.
 *
 * Averages samplesPerSensor readings and stores results in @p values.
 *
 * @param  qtr      Sensor array descriptor.
 * @param  values   Output buffer (sensorCount entries).
 * @param  emitters Emitter state to use during the measurement.
 */
static void _read_private(QTRSensors *qtr, uint16_t *values, QTREmitters emitters)
{
    QTRSensors_setEmitters(qtr, emitters);

    uint8_t samples = (qtr->samplesPerSensor > 0U) ? qtr->samplesPerSensor : 1U;

    for (uint8_t s = 0U; s < qtr->sensorCount; s++) {
        uint32_t sum = 0U;
        for (uint8_t n = 0U; n < samples; n++) {
            sum += _read_adc_channel(&qtr->channels[s]);
        }
        values[s] = (uint16_t)(sum / samples);
    }
}

/* ================================================================== */
/* Public API – initialisation                                          */
/* ================================================================== */

void QTRSensors_init(QTRSensors *qtr)
{
    if (qtr == NULL) {
        return;
    }

    /* Clamp sensor count. */
    if (qtr->sensorCount == 0U || qtr->sensorCount > QTR_MAX_SENSORS) {
        qtr->sensorCount = QTR_MAX_SENSORS;
    }

    if (qtr->samplesPerSensor == 0U) {
        qtr->samplesPerSensor = 1U;
    }

    /* Initialise calibration structures. */
    memset(&qtr->calibrationOn,  0, sizeof(CalibrationData));
    memset(&qtr->calibrationOff, 0, sizeof(CalibrationData));
    _calibration_init(&qtr->calibrationOn,  qtr->sensorCount);
    _calibration_init(&qtr->calibrationOff, qtr->sensorCount);

    qtr->_lastPosition = 0;
}

void QTRSensors_deinit(QTRSensors *qtr)
{
    if (qtr == NULL) {
        return;
    }
    _calibration_free(&qtr->calibrationOn);
    _calibration_free(&qtr->calibrationOff);
}

void QTRSensors_resetCalibration(QTRSensors *qtr)
{
    if (qtr == NULL) {
        return;
    }
    _calibration_init(&qtr->calibrationOn,  qtr->sensorCount);
    _calibration_init(&qtr->calibrationOff, qtr->sensorCount);
}

/* ================================================================== */
/* Public API – raw readings                                            */
/* ================================================================== */

void QTRSensors_readRaw(QTRSensors *qtr, uint16_t *values, QTREmitters emitters)
{
    if (qtr == NULL || values == NULL) {
        return;
    }
    _read_private(qtr, values, emitters);
    QTRSensors_setEmitters(qtr, QTREmittersOff);
}

void QTRSensors_read(QTRSensors *qtr, uint16_t *values, QTRReadMode mode)
{
    if (qtr == NULL || values == NULL) {
        return;
    }

    switch (mode) {
        case QTRReadModeOff:
            _read_private(qtr, values, QTREmittersOff);
            break;

        case QTRReadModeOn:
            _read_private(qtr, values, QTREmittersOn);
            break;

        case QTRReadModeOnAndOff: {
            uint16_t valuesOff[QTR_MAX_SENSORS] = {0};

            /* Read with emitters on, then off. */
            _read_private(qtr, values,    QTREmittersOn);
            _read_private(qtr, valuesOff, QTREmittersOff);

            /* Subtract ambient; clamp to 0. */
            for (uint8_t i = 0U; i < qtr->sensorCount; i++) {
                if (values[i] > valuesOff[i]) {
                    values[i] = values[i] - valuesOff[i];
                } else {
                    values[i] = 0U;
                }
            }
            break;
        }

        default:
            break;
    }

    QTRSensors_setEmitters(qtr, QTREmittersOff);
}

/* ================================================================== */
/* Public API – calibration                                             */
/* ================================================================== */

void QTRSensors_calibrate(QTRSensors *qtr, QTRReadMode mode)
{
    if (qtr == NULL) {
        return;
    }

    uint16_t valuesOn[QTR_MAX_SENSORS]  = {0};
    uint16_t valuesOff[QTR_MAX_SENSORS] = {0};

    for (uint8_t r = 0U; r < QTR_CALIBRATION_READS; r++) {
        if (mode == QTRReadModeOnAndOff) {
            /* Capture both states in one pass. */
            uint16_t tmp[QTR_MAX_SENSORS] = {0};
            _read_private(qtr, tmp,       QTREmittersOn);
            _read_private(qtr, valuesOff, QTREmittersOff);

            /* Subtract ambient. */
            for (uint8_t i = 0U; i < qtr->sensorCount; i++) {
                valuesOn[i] = (tmp[i] > valuesOff[i]) ? (tmp[i] - valuesOff[i]) : 0U;
            }
        } else if (mode == QTRReadModeOn) {
            _read_private(qtr, valuesOn, QTREmittersOn);
        } else {
            _read_private(qtr, valuesOff, QTREmittersOff);
        }

        /* Update On calibration data. */
        if (mode != QTRReadModeOff && qtr->calibrationOn.initialized) {
            for (uint8_t i = 0U; i < qtr->sensorCount; i++) {
                if (valuesOn[i] < qtr->calibrationOn.minimum[i]) {
                    qtr->calibrationOn.minimum[i] = valuesOn[i];
                }
                if (valuesOn[i] > qtr->calibrationOn.maximum[i]) {
                    qtr->calibrationOn.maximum[i] = valuesOn[i];
                }
            }
        }

        /* Update Off calibration data. */
        if (mode != QTRReadModeOn && qtr->calibrationOff.initialized) {
            for (uint8_t i = 0U; i < qtr->sensorCount; i++) {
                if (valuesOff[i] < qtr->calibrationOff.minimum[i]) {
                    qtr->calibrationOff.minimum[i] = valuesOff[i];
                }
                if (valuesOff[i] > qtr->calibrationOff.maximum[i]) {
                    qtr->calibrationOff.maximum[i] = valuesOff[i];
                }
            }
        }
    }

    QTRSensors_setEmitters(qtr, QTREmittersOff);
}

/* ================================================================== */
/* Public API – calibrated readings                                     */
/* ================================================================== */

int QTRSensors_readCalibrated(QTRSensors *qtr, uint16_t *values, QTRReadMode mode)
{
    if (qtr == NULL || values == NULL) {
        return -1;
    }

    /* Choose which calibration set to use. */
    CalibrationData *cal = (mode == QTRReadModeOff) ? &qtr->calibrationOff
                                                     : &qtr->calibrationOn;
    if (!cal->initialized) {
        return -1;
    }

    /* Get raw / compensated reading. */
    uint16_t raw[QTR_MAX_SENSORS] = {0};
    QTRSensors_read(qtr, raw, mode);

    for (uint8_t i = 0U; i < qtr->sensorCount; i++) {
        uint16_t calMin = cal->minimum[i];
        uint16_t calMax = cal->maximum[i];

        if (calMax <= calMin) {
            /* No valid calibration range – output 0. */
            values[i] = 0U;
            continue;
        }

        /* Map raw value into [0, QTR_CALIBRATED_MAX]. */
        int32_t norm;
        if (raw[i] <= calMin) {
            norm = 0;
        } else if (raw[i] >= calMax) {
            norm = (int32_t)QTR_CALIBRATED_MAX;
        } else {
            norm = (int32_t)((uint32_t)(raw[i] - calMin) * QTR_CALIBRATED_MAX
                             / (uint32_t)(calMax - calMin));
        }

        values[i] = (uint16_t)norm;
    }

    return 0;
}

/* ================================================================== */
/* Internal – line position engine                                      */
/* ================================================================== */

/**
 * @brief Compute weighted-average line position from calibrated values.
 *
 * @param  qtr           Sensor array descriptor.
 * @param  values        Calibrated sensor values [0, 1000].
 * @param  inverted      If non-zero, inverts values (white-line mode).
 * @param  threshold     Sensors whose (possibly inverted) value is below this
 *                       number are excluded from the average.
 * @return Position [0, (sensorCount-1)*1000], or the last valid position if
 *         no sensor exceeds the threshold.
 */
static int32_t _read_line(QTRSensors *qtr, uint16_t *values,
                          uint8_t inverted, uint16_t threshold)
{
    uint32_t avg    = 0U;
    uint32_t sum    = 0U;
    uint8_t  onLine = 0U;

    for (uint8_t i = 0U; i < qtr->sensorCount; i++) {
        uint16_t v = inverted ? (QTR_CALIBRATED_MAX - values[i]) : values[i];

        if (v > threshold) {
            onLine = 1U;
            avg += (uint32_t)v * (uint32_t)(i * QTR_CALIBRATED_MAX);
            sum += v;
        }
    }

    if (!onLine) {
        /*
         * No sensor detected the line.  Return the extreme position on the
         * side the line was last seen (helps with line-following recovery).
         */
        if (qtr->_lastPosition < (int32_t)((qtr->sensorCount / 2) * QTR_CALIBRATED_MAX)) {
            return 0;
        } else {
            return (int32_t)((qtr->sensorCount - 1U) * QTR_CALIBRATED_MAX);
        }
    }

    int32_t position = (int32_t)(avg / sum);
    qtr->_lastPosition = position;
    return position;
}

/* ================================================================== */
/* Public API – line position                                           */
/* ================================================================== */

int32_t QTRSensors_readLineBlack(QTRSensors *qtr, uint16_t *values,
                                 QTRReadMode mode, uint16_t whiteThresh)
{
    if (qtr == NULL || values == NULL) {
        return -1;
    }
    if (QTRSensors_readCalibrated(qtr, values, mode) != 0) {
        return -1;
    }
    return _read_line(qtr, values, 0U, whiteThresh);
}

int32_t QTRSensors_readLineWhite(QTRSensors *qtr, uint16_t *values,
                                 QTRReadMode mode, uint16_t blackThresh)
{
    if (qtr == NULL || values == NULL) {
        return -1;
    }
    if (QTRSensors_readCalibrated(qtr, values, mode) != 0) {
        return -1;
    }
    return _read_line(qtr, values, 1U, blackThresh);
}

/* ================================================================== */
/* Public API – emitter control                                         */
/* ================================================================== */

void QTRSensors_setEmitters(QTRSensors *qtr, QTREmitters emitters)
{
    if (qtr == NULL || qtr->emitterGpioPort == NULL) {
        return;
    }

    if (emitters == QTREmittersOn) {
        HAL_GPIO_WritePin(qtr->emitterGpioPort, qtr->emitterGpioPin,
                          qtr->emitterOnState);
    } else {
        GPIO_PinState offState = (qtr->emitterOnState == GPIO_PIN_SET)
                                 ? GPIO_PIN_RESET : GPIO_PIN_SET;
        HAL_GPIO_WritePin(qtr->emitterGpioPort, qtr->emitterGpioPin, offState);
    }
}
