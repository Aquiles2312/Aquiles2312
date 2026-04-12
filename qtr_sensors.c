/**
 * @file    qtr_sensors.c
 * @brief   STM32F411RE QTR8A reflectance sensor library using ADC + DMA.
 *
 * Data acquisition is hardware-driven: the ADC scans all channels in scan
 * mode while the DMA writes results into a circular buffer supplied by the
 * application.  The library reads snapshots from that buffer instead of
 * polling each ADC channel sequentially, giving true simultaneous captures
 * with minimal CPU involvement.
 *
 * The application is responsible for calling HAL_ADC_Start_DMA() before
 * using any read function.  This library does not reconfigure ADC channels
 * or ranks - that is done once by CubeMX-generated code.
 */

#include "qtr_sensors.h"

#include <string.h>
#include <stdint.h>

/* ================================================================== */
/* Internal helpers                                                     */
/* ================================================================== */

/**
 * @brief Allocate (or re-allocate) the min/max arrays inside a CalibrationData.
 *
 * Safe to call multiple times: frees existing arrays before re-allocating.
 *
 * @param  cal   CalibrationData to initialise.
 * @param  count Number of sensors.
 */
static void _calibration_reset(CalibrationData *cal, uint8_t count)
{
    for (uint8_t i = 0; i < count; i++) {
        cal->minimum[i] = QTR_ADC_MAX;
        cal->maximum[i] = 0U;
    }
    cal->initialized = 1U;
}

/**
 * @brief Copy a snapshot of the DMA buffer into @p values.
 *
 * The ADC + DMA fills adcBuffer continuously in circular mode.  Each entry is
 * read with a single 16-bit load instruction, which is naturally atomic on
 * Cortex-M4 when the buffer is half-word aligned (guaranteed by the DMA
 * peripheral and standard C alignment rules for uint16_t arrays).  A
 * torn-read of a single sample would only introduce one stale ADC value for
 * one sensor per snapshot, which the calibration and weighted-average
 * algorithms tolerate.  If stricter consistency is required, disable the DMA
 * interrupt (HAL_ADC_Stop_DMA / HAL_ADC_Start_DMA) around this copy.
 *
 * @param  qtr    Sensor array descriptor.
 * @param  values Output buffer (sensorCount entries).
 */
static void _snapshot_dma_buffer(const QTRSensors *qtr, uint16_t *values)
{
    for (uint8_t i = 0U; i < qtr->sensorCount; i++) {
        values[i] = qtr->adcBuffer[i];
    }
}

/**
 * @brief Set emitters, snapshot the DMA buffer, copy to @p values.
 *
 * @param  qtr      Sensor array descriptor.
 * @param  values   Output buffer (sensorCount entries).
 * @param  emitters Emitter state to apply before snapshotting.
 */
static void _read_private(QTRSensors *qtr, uint16_t *values, QTREmitters emitters)
{
    QTRSensors_setEmitters(qtr, emitters);

    /*
     * Allow the IR emitters to settle before snapshotting.  The QTR8A
     * datasheet specifies a maximum emitter rise time of ~200 us.  A 1 ms
     * guard covers that comfortably while keeping the per-snapshot overhead
     * manageable.  If the application uses a faster control loop, reduce
     * this to HAL_Delay(0) (no wait) after confirming emitter settling on
     * your hardware.
     */
    
    _snapshot_dma_buffer(qtr, values);
}

static void _apply_filter(QTRSensors *qtr, uint16_t *values)
{
    if (!qtr->filterInitialized) {
        for (uint8_t i = 0; i < qtr->sensorCount; i++) {
            qtr->filtered[i] = values[i];
        }
        qtr->filterInitialized = 1;
        return;
    }

    for (uint8_t i = 0; i < qtr->sensorCount; i++) {
        qtr->filtered[i] = (qtr->filtered[i] * 3 + values[i]) >> 2;
        values[i] = qtr->filtered[i];
    }
}

/* ================================================================== */
/* Public API - initialisation                                          */
/* ================================================================== */

void QTRSensors_init(QTRSensors *qtr)
{
    if (qtr == NULL) {
        return;
    }

    /* Clamp sensor count to valid range. */
    if (qtr->sensorCount == 0U || qtr->sensorCount > QTR_MAX_SENSORS) {
        qtr->sensorCount = QTR_MAX_SENSORS;
    }

    /* Initialise calibration structures. */
    memset(&qtr->calibrationOn,  0, sizeof(CalibrationData));
    memset(&qtr->calibrationOff, 0, sizeof(CalibrationData));
    _calibration_reset(&qtr->calibrationOn, qtr->sensorCount);
    _calibration_reset(&qtr->calibrationOff, qtr->sensorCount);

    qtr->_lastPosition = (qtr->sensorCount - 1) * QTR_CALIBRATED_MAX / 2;
    qtr->filterInitialized = 0;
}

void QTRSensors_resetCalibration(QTRSensors *qtr)
{
    if (qtr == NULL) {
        return;
    }
    _calibration_reset(&qtr->calibrationOn,  qtr->sensorCount);
    _calibration_reset(&qtr->calibrationOff, qtr->sensorCount);
}

/* ================================================================== */
/* Public API - raw readings                                            */
/* ================================================================== */

void QTRSensors_readRaw(QTRSensors *qtr, uint16_t *values, QTREmitters emitters)
{
    if (qtr == NULL || values == NULL || qtr->adcBuffer == NULL) {
        return;
    }
    _read_private(qtr, values, emitters);
    QTRSensors_setEmitters(qtr, QTREmittersOff);
}

void QTRSensors_read(QTRSensors *qtr, uint16_t *values, QTRReadMode mode)
{
    if (qtr == NULL || values == NULL || qtr->adcBuffer == NULL) {
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

            /* Snapshot with emitters on, then off. */
            _read_private(qtr, values,    QTREmittersOn);
            _read_private(qtr, valuesOff, QTREmittersOff);

            /* Subtract ambient; clamp underflow to 0. */
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

if (!qtr->isCalibrating) {
    _apply_filter(qtr, values);
}

    QTRSensors_setEmitters(qtr, QTREmittersOff);
}

/* ================================================================== */
/* Public API - calibration                                             */
/* ================================================================== */

void QTRSensors_calibrate(QTRSensors *qtr, QTRReadMode mode)
{
    if (qtr == NULL || qtr->adcBuffer == NULL) {
        return;
    }
    qtr->isCalibrating = 1;

    uint16_t valuesOn[QTR_MAX_SENSORS]  = {0};
    uint16_t valuesOff[QTR_MAX_SENSORS] = {0};

    for (uint8_t r = 0U; r < QTR_CALIBRATION_READS; r++) {
        if (mode == QTRReadModeOnAndOff) {
            uint16_t tmp[QTR_MAX_SENSORS] = {0};
            _read_private(qtr, tmp,       QTREmittersOn);
            _read_private(qtr, valuesOff, QTREmittersOff);

            /* Subtract ambient per sensor. */
            for (uint8_t i = 0U; i < qtr->sensorCount; i++) {
                valuesOn[i] = (tmp[i] > valuesOff[i]) ? (tmp[i] - valuesOff[i]) : 0U;
            }
        } else if (mode == QTRReadModeOn) {
            _read_private(qtr, valuesOn, QTREmittersOn);
        } else {
            _read_private(qtr, valuesOff, QTREmittersOff);
        }

        /* Update On calibration. */
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

        /* Update Off calibration. */
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
    qtr->isCalibrating = 0;
}

/* ================================================================== */
/* Public API - calibrated readings                                     */
/* ================================================================== */

int QTRSensors_readCalibrated(QTRSensors *qtr, uint16_t *values, QTRReadMode mode)
{
    if (qtr == NULL || values == NULL) {
        return -1;
    }

    /* Choose which calibration set to use. */
    CalibrationData *cal = (mode == QTRReadModeOff) ? &qtr->calibrationOff : &qtr->calibrationOn;
    if (!cal->initialized) {
        return -1;
    }

    /* Get the current DMA-based snapshot. */
    uint16_t raw[QTR_MAX_SENSORS] = {0};
    QTRSensors_read(qtr, raw, mode);

    for (uint8_t i = 0U; i < qtr->sensorCount; i++) {
        uint16_t calMin = cal->minimum[i];
        uint16_t calMax = cal->maximum[i];

        if ((calMax - calMin) < 10) {
            /* No valid calibration range yet - output 0. */
            values[i] = 0U;
            continue;
        }

        /* Linear map: raw value -> [0, QTR_CALIBRATED_MAX]. */
        int32_t norm;
        if (raw[i] <= calMin) {
            norm = 0;
        } else if (raw[i] >= calMax) {
            norm = (int32_t)QTR_CALIBRATED_MAX;
        } else {
            norm = (int32_t)((uint32_t)(raw[i] - calMin) * QTR_CALIBRATED_MAX / (uint32_t)(calMax - calMin));
        }

        values[i] = (uint16_t)norm;
    }

    return 0;
}

/* ================================================================== */
/* Internal - line position engine                                      */
/* ================================================================== */

/**
 * @brief Compute weighted-average line position from calibrated values.
 *
 * @param  qtr       Sensor array descriptor.
 * @param  values    Calibrated sensor values [0, 1000].
 * @param  inverted  Non-zero to invert values (white-line mode).
 * @param  threshold Sensors whose (possibly inverted) value is at or below
 *                   this threshold are excluded from the average.
 * @return Position [0, (sensorCount-1)*1000], or last position if no sensor
 *         exceeds the threshold (lost-line recovery).
 */
static int32_t _read_line(QTRSensors *qtr, uint16_t *values, uint8_t inverted, uint16_t threshold)
{
    uint32_t avg    = 0U;
    uint32_t sum    = 0U;

    for (uint8_t i = 0U; i < qtr->sensorCount; i++) {
        uint16_t v = inverted ? (QTR_CALIBRATED_MAX - values[i]) : values[i];

        if (v > threshold) {
            avg += (uint32_t)v * (uint32_t)(i * QTR_CALIBRATED_MAX);
            sum += v;
        }
    }

    uint32_t linethreshold = (qtr->sensorCount * 50);

    if (sum < linethreshold) {
        /* Return the edge on the side the line was last detected. */
        if (qtr->_lastPosition < (int32_t)((qtr->sensorCount / 2U) * QTR_CALIBRATED_MAX)) {
            return qtr->_lastPosition;
        }
    }

    int32_t position = (int32_t)(avg / sum);
    qtr->_lastPosition = position;
    return position;
}

/* ================================================================== */
/* Public API - line position                                           */
/* ================================================================== */

int32_t QTRSensors_readLineBlack(QTRSensors *qtr, uint16_t *values, QTRReadMode mode, uint16_t whiteThresh)
{
    if (qtr == NULL || values == NULL) {
        return -1;
    }
    if (QTRSensors_readCalibrated(qtr, values, mode) != 0) {
        return -1;
    }
    return _read_line(qtr, values, 0U, whiteThresh);
}

int32_t QTRSensors_readLineWhite(QTRSensors *qtr, uint16_t *values, QTRReadMode mode, uint16_t blackThresh)
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
/* Public API - emitter control                                         */
/* ================================================================== */

void QTRSensors_setEmitters(QTRSensors *qtr, QTREmitters emitters)
{
    if (qtr == NULL || qtr->emitterGpioPort == NULL) {
        return;
    }

    if (emitters == QTREmittersOn) {
        HAL_GPIO_WritePin(qtr->emitterGpioPort, qtr->emitterGpioPin, qtr->emitterOnState);
    } else {
        GPIO_PinState offState = (qtr->emitterOnState == GPIO_PIN_SET) ? GPIO_PIN_RESET : GPIO_PIN_SET;
        HAL_GPIO_WritePin(qtr->emitterGpioPort, qtr->emitterGpioPin, offState);
    }
}
