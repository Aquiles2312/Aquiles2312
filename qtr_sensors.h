/**
 * @file    qtr_sensors.h
 * @brief   STM32F411RE pure C port of the Pololu QTR8A reflectance sensor library.
 *
 * Removed all Arduino dependencies; uses STM32 HAL ADC for analog readings.
 * Supports 8 analog sensors (QTR8A), calibration, line-position calculation,
 * and optional emitter (IR LED) control via a GPIO pin.
 */

#ifndef QTR_SENSORS_H
#define QTR_SENSORS_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stddef.h>
#include "stm32f4xx_hal.h"

/* ------------------------------------------------------------------ */
/* Constants                                                            */
/* ------------------------------------------------------------------ */

/** Maximum number of sensors supported. */
#define QTR_MAX_SENSORS        8U

/** Raw ADC value for a 12-bit STM32 ADC (0-4095). */
#define QTR_ADC_MAX            4095U

/** Number of calibration readings taken per sensor per call. */
#define QTR_CALIBRATION_READS  10U

/** Scale for calibrated sensor output (0-1000). */
#define QTR_CALIBRATED_MAX     1000U

/* ------------------------------------------------------------------ */
/* Enumerations                                                         */
/* ------------------------------------------------------------------ */

/**
 * @brief Sensor type.
 *
 * Only analogue (RC or A-type) is relevant here; kept for API compatibility.
 */
typedef enum {
    QTRTypeUndefined = 0,
    QTRTypeRC,      /**< RC (digital) sensor - not used in this port. */
    QTRTypeAnalog   /**< Analogue sensor (QTR8A). */
} QTRType;

/**
 * @brief Which emitters to use when taking a reading.
 */
typedef enum {
    QTREmittersOff  = 0, /**< All emitters off. */
    QTREmittersOn   = 1, /**< All emitters on.  */
    QTREmittersSingle    /**< Only the emitter for the current sensor on (not supported). */
} QTREmitters;

/**
 * @brief Read mode – controls emitter state during measurement.
 */
typedef enum {
    QTRReadModeOff,       /**< Measure with emitters off (ambient light). */
    QTRReadModeOn,        /**< Measure with emitters on only. */
    QTRReadModeOnAndOff   /**< Measure with emitters on, subtract ambient reading. */
} QTRReadMode;

/* ------------------------------------------------------------------ */
/* Structures                                                           */
/* ------------------------------------------------------------------ */

/**
 * @brief Calibration data for one sensor array (min/max arrays).
 */
typedef struct {
    uint16_t *minimum; /**< Per-sensor minimum raw value seen during calibration. */
    uint16_t *maximum; /**< Per-sensor maximum raw value seen during calibration. */
    uint8_t   count;   /**< Number of sensors this data covers. */
    uint8_t   initialized; /**< Non-zero if arrays have been allocated. */
} CalibrationData;

/**
 * @brief ADC channel descriptor – one entry per physical sensor.
 */
typedef struct {
    ADC_HandleTypeDef *hadc;    /**< Pointer to the HAL ADC handle for this sensor. */
    uint32_t           channel; /**< ADC channel number (e.g. ADC_CHANNEL_0). */
    uint32_t           rank;    /**< Conversion rank (1-based, for regular group). */
} QTRSensorChannel;

/**
 * @brief Main QTR sensor array descriptor.
 *
 * Populate the public fields before calling QTRSensors_init().
 */
typedef struct {
    /* --- Public configuration (fill before calling QTRSensors_init) --- */
    QTRSensorChannel channels[QTR_MAX_SENSORS]; /**< ADC channel descriptors. */
    uint8_t          sensorCount;               /**< Number of active sensors (1-8). */

    /** Optional emitter enable pin. Set gpioPort to NULL to disable. */
    GPIO_TypeDef    *emitterGpioPort;
    uint16_t         emitterGpioPin;
    GPIO_PinState    emitterOnState; /**< State that turns emitters ON. */

    uint8_t          samplesPerSensor; /**< ADC readings averaged per sensor (>=1). */
    QTRType          type;             /**< Sensor type (should be QTRTypeAnalog). */

    /* --- Private runtime state (managed by library) --- */
    CalibrationData  calibrationOn;  /**< Calibration taken with emitters on. */
    CalibrationData  calibrationOff; /**< Calibration taken with emitters off. */

    int32_t          _lastPosition;  /**< Last valid line position (for lost-line recovery). */
} QTRSensors;

/* ------------------------------------------------------------------ */
/* Function declarations                                                */
/* ------------------------------------------------------------------ */

/**
 * @brief  Initialise the QTR sensor library for a given sensor array.
 *
 * Allocates calibration arrays.  Call QTRSensors_deinit() to free memory.
 *
 * @param  qtr  Pointer to a user-allocated QTRSensors structure that has
 *              already had its public fields populated.
 */
void QTRSensors_init(QTRSensors *qtr);

/**
 * @brief  Free calibration memory and reset the structure.
 * @param  qtr  Pointer to an initialised QTRSensors structure.
 */
void QTRSensors_deinit(QTRSensors *qtr);

/**
 * @brief  Reset (erase) calibration data so the next calibrate call starts fresh.
 * @param  qtr  Pointer to an initialised QTRSensors structure.
 */
void QTRSensors_resetCalibration(QTRSensors *qtr);

/* --- Raw readings -------------------------------------------------- */

/**
 * @brief  Read raw ADC values from all sensors.
 *
 * Results are 12-bit values (0 – 4095).  Averages samplesPerSensor readings.
 *
 * @param  qtr      Pointer to an initialised QTRSensors structure.
 * @param  values   Output buffer; caller must provide at least sensorCount entries.
 * @param  emitters Which emitters to enable during the measurement.
 */
void QTRSensors_readRaw(QTRSensors *qtr, uint16_t *values, QTREmitters emitters);

/**
 * @brief  Read raw values according to the chosen read mode.
 *
 * For QTRReadModeOnAndOff the ambient (off) reading is subtracted from the
 * lit (on) reading; any underflow is clamped to 0.
 *
 * @param  qtr    Pointer to an initialised QTRSensors structure.
 * @param  values Output buffer (sensorCount entries).
 * @param  mode   Read mode.
 */
void QTRSensors_read(QTRSensors *qtr, uint16_t *values, QTRReadMode mode);

/* --- Calibration --------------------------------------------------- */

/**
 * @brief  Perform one calibration pass (call repeatedly ~250 ms).
 *
 * Reads QTR_CALIBRATION_READS samples per sensor and updates the stored
 * minimum/maximum for both the On and Off measurements.
 *
 * @param  qtr   Pointer to an initialised QTRSensors structure.
 * @param  mode  Read mode used during calibration.
 */
void QTRSensors_calibrate(QTRSensors *qtr, QTRReadMode mode);

/**
 * @brief  Read calibrated values in the range [0, 1000].
 *
 * A value of 0 means the sensor is above the lightest surface seen during
 * calibration; 1000 means darkest.
 *
 * @param  qtr    Pointer to an initialised QTRSensors structure.
 * @param  values Output buffer (sensorCount entries).
 * @param  mode   Read mode (must match the mode used during calibration).
 * @return 0 on success, -1 if not calibrated.
 */
int  QTRSensors_readCalibrated(QTRSensors *qtr, uint16_t *values, QTRReadMode mode);

/* --- Line position ------------------------------------------------- */

/**
 * @brief  Return the position of a black line on a white background.
 *
 * Uses a weighted average of calibrated sensor values.
 * Result ranges from 0 (line under sensor 0) to (sensorCount-1)*1000
 * (line under the last sensor).
 *
 * @param  qtr          Pointer to an initialised QTRSensors structure.
 * @param  values       Calibrated values buffer (filled by this function).
 * @param  mode         Read mode.
 * @param  whiteThresh  Sensors above this calibrated value are ignored
 *                      (typically 500).
 * @return Line position, or -1 if all sensors read below whiteThresh.
 */
int32_t QTRSensors_readLineBlack(QTRSensors *qtr, uint16_t *values,
                                 QTRReadMode mode, uint16_t whiteThresh);

/**
 * @brief  Return the position of a white line on a black background.
 *
 * Inverts the calibrated sensor readings before computing position.
 *
 * @param  qtr          Pointer to an initialised QTRSensors structure.
 * @param  values       Calibrated values buffer (filled by this function).
 * @param  mode         Read mode.
 * @param  blackThresh  Sensors below this calibrated value are ignored
 *                      (typically 500).
 * @return Line position, or -1 if all sensors read above blackThresh.
 */
int32_t QTRSensors_readLineWhite(QTRSensors *qtr, uint16_t *values,
                                 QTRReadMode mode, uint16_t blackThresh);

/* --- Emitter control ----------------------------------------------- */

/**
 * @brief  Turn the IR emitters on or off via the configured GPIO pin.
 *
 * Does nothing if emitterGpioPort is NULL.
 *
 * @param  qtr      Pointer to an initialised QTRSensors structure.
 * @param  emitters QTREmittersOn or QTREmittersOff.
 */
void QTRSensors_setEmitters(QTRSensors *qtr, QTREmitters emitters);

#ifdef __cplusplus
}
#endif

#endif /* QTR_SENSORS_H */
