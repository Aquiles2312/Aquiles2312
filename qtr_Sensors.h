/**
 * @file    qtr_sensors.h
 * @brief   STM32F411RE QTR8A reflectance sensor library using ADC + DMA.
 *
 * Data acquisition is fully hardware-driven: the ADC scans all channels in
 * sequence while the DMA transfers results into a user-supplied buffer.
 * The library reads from that buffer instead of polling the ADC sensor by
 * sensor, giving simultaneous captures and zero CPU overhead during conversion.
 *
 * Prerequisites (configure in STM32CubeMX before using this library):
 *   - ADC: Scan Conversion Mode ENABLED
 *   - ADC: Continuous Conversion Mode ENABLED (recommended)
 *   - ADC: DMA Continuous Requests ENABLED
 *   - DMA:  circular mode, half-word (16-bit) transfers
 *
 * Usage:
 * @code
 *   uint16_t adc_buffer[8];
 *   HAL_ADC_Start_DMA(&hadc1, (uint32_t *)adc_buffer, 8);
 *
 *   QTRSensors qtr = {
 *       .hadc         = &hadc1,
 *       .adcBuffer    = adc_buffer,
 *       .sensorCount  = 8,
 *       .emitterGpioPort = GPIOA,
 *       .emitterGpioPin  = GPIO_PIN_5,
 *       .emitterOnState  = GPIO_PIN_SET,
 *   };
 *   QTRSensors_init(&qtr);
 * @endcode
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

/** Maximum raw value from a 12-bit STM32 ADC (0-4095). */
#define QTR_ADC_MAX            4095U

/** Number of DMA buffer snapshots taken per calibration call. */
#define QTR_CALIBRATION_READS  10U

/** Scale for calibrated sensor output (0-1000). */
#define QTR_CALIBRATED_MAX     1000U

/* ------------------------------------------------------------------ */
/* Enumerations                                                         */
/* ------------------------------------------------------------------ */

/**
 * @brief Sensor type (kept for API compatibility).
 */
typedef enum {
    QTRTypeUndefined = 0,
    QTRTypeRC,      /**< RC (digital) sensor - not used in this port. */
    QTRTypeAnalog   /**< Analogue sensor (QTR8A). */
} QTRType;

/**
 * @brief Emitter state for a reading.
 */
typedef enum {
    QTREmittersOff    = 0, /**< All emitters off. */
    QTREmittersOn     = 1, /**< All emitters on.  */
    QTREmittersSingle = 2  /**< Per-sensor emitter (not supported). */
} QTREmitters;

/**
 * @brief Read mode - controls emitter state during measurement.
 */
typedef enum {
    QTRReadModeOff,       /**< Snapshot with emitters off (ambient light). */
    QTRReadModeOn,        /**< Snapshot with emitters on. */
    QTRReadModeOnAndOff   /**< Snapshot on, then off; ambient is subtracted. */
} QTRReadMode;

/* ------------------------------------------------------------------ */
/* Structures                                                           */
/* ------------------------------------------------------------------ */

/**
 * @brief Per-sensor min/max calibration data.
 */
typedef struct {
    uint16_t *minimum;     /**< Per-sensor minimum raw value (from DMA buffer). */
    uint16_t *maximum;     /**< Per-sensor maximum raw value (from DMA buffer). */
    uint8_t   count;       /**< Number of sensors covered by this data. */
    uint8_t   initialized; /**< Non-zero when arrays are allocated and seeded. */
} CalibrationData;

/**
 * @brief Main QTR sensor array descriptor.
 *
 * Populate the public fields, then call QTRSensors_init().
 * The ADC must already be running in DMA mode before calling any read
 * function (start it with HAL_ADC_Start_DMA() in your application).
 */
typedef struct {
    /* --- Public configuration (set before QTRSensors_init) ---------- */

    /**
     * HAL ADC handle whose DMA is filling adcBuffer.
     * Used only for HAL_ADC_Start_DMA() restart if needed; the library
     * does NOT reconfigure channels or ranks - that is CubeMX's job.
     */
    ADC_HandleTypeDef *hadc;

    /**
     * Pointer to the DMA destination buffer supplied to HAL_ADC_Start_DMA().
     * Buffer length must be >= sensorCount.
     * The order of entries MUST match the ADC scan rank order set in CubeMX.
     */
    volatile uint16_t *adcBuffer;

    uint8_t           sensorCount;     /**< Number of active sensors (1-8). */

    /** Optional emitter enable GPIO. Set to NULL to disable emitter control. */
    GPIO_TypeDef     *emitterGpioPort;
    uint16_t          emitterGpioPin;
    GPIO_PinState     emitterOnState;  /**< GPIO state that turns emitters ON. */

    QTRType           type;            /**< Sensor type (QTRTypeAnalog). */

    /* --- Private runtime state (managed by library) ----------------- */
    CalibrationData   calibrationOn;   /**< Calibration with emitters on. */
    CalibrationData   calibrationOff;  /**< Calibration with emitters off. */
    int32_t           _lastPosition;   /**< Last valid line position (recovery). */
} QTRSensors;

/* ------------------------------------------------------------------ */
/* Function declarations                                                */
/* ------------------------------------------------------------------ */

/**
 * @brief  Initialise the library and allocate calibration arrays.
 *
 * Call QTRSensors_deinit() when done to release memory.
 *
 * @param  qtr  Descriptor with public fields already populated.
 */
void QTRSensors_init(QTRSensors *qtr);

/**
 * @brief  Free calibration memory and reset runtime state.
 * @param  qtr  Initialised descriptor.
 */
void QTRSensors_deinit(QTRSensors *qtr);

/**
 * @brief  Erase calibration so the next calibrate call starts fresh.
 * @param  qtr  Initialised descriptor.
 */
void QTRSensors_resetCalibration(QTRSensors *qtr);

/* --- Raw readings -------------------------------------------------- */

/**
 * @brief  Copy the current DMA buffer snapshot into @p values.
 *
 * @param  qtr      Initialised descriptor.
 * @param  values   Output buffer; caller provides >= sensorCount entries.
 * @param  emitters Emitter state to set before snapshotting.
 */
void QTRSensors_readRaw(QTRSensors *qtr, uint16_t *values, QTREmitters emitters);

/**
 * @brief  Snapshot the DMA buffer according to the chosen read mode.
 *
 * For QTRReadModeOnAndOff the ambient (off) snapshot is subtracted from
 * the lit (on) snapshot; underflow is clamped to 0.
 *
 * @param  qtr    Initialised descriptor.
 * @param  values Output buffer (sensorCount entries).
 * @param  mode   Read mode.
 */
void QTRSensors_read(QTRSensors *qtr, uint16_t *values, QTRReadMode mode);

/* --- Calibration --------------------------------------------------- */

/**
 * @brief  Perform one calibration pass.
 *
 * Takes QTR_CALIBRATION_READS snapshots from the DMA buffer and updates
 * the stored per-sensor min/max values.  Call repeatedly while sweeping
 * the sensor array over the surface (~250 ms total recommended).
 *
 * @param  qtr   Initialised descriptor.
 * @param  mode  Read mode to use during calibration.
 */
void QTRSensors_calibrate(QTRSensors *qtr, QTRReadMode mode);

/**
 * @brief  Read calibrated values scaled to [0, 1000].
 *
 * 0 = lightest surface seen during calibration; 1000 = darkest.
 *
 * @param  qtr    Initialised descriptor.
 * @param  values Output buffer (sensorCount entries).
 * @param  mode   Read mode (must match the mode used during calibration).
 * @return 0 on success, -1 if calibration data is missing.
 */
int  QTRSensors_readCalibrated(QTRSensors *qtr, uint16_t *values, QTRReadMode mode);

/* --- Line position ------------------------------------------------- */

/**
 * @brief  Return the position of a black line on a white background.
 *
 * Weighted average of calibrated values.
 * Range: 0 (line under sensor 0) to (sensorCount-1)*1000 (last sensor).
 *
 * @param  qtr          Initialised descriptor.
 * @param  values       Buffer filled with calibrated readings (sensorCount entries).
 * @param  mode         Read mode.
 * @param  whiteThresh  Sensors with calibrated value below this threshold are
 *                      excluded (typically 500).
 * @return Line position, or -1 if calibration data is missing.
 */
int32_t QTRSensors_readLineBlack(QTRSensors *qtr, uint16_t *values,
                                 QTRReadMode mode, uint16_t whiteThresh);

/**
 * @brief  Return the position of a white line on a black background.
 *
 * Inverts calibrated readings before computing the weighted average.
 *
 * @param  qtr          Initialised descriptor.
 * @param  values       Buffer filled with calibrated readings (sensorCount entries).
 * @param  mode         Read mode.
 * @param  blackThresh  Sensors with inverted calibrated value below this
 *                      threshold are excluded (typically 500).
 * @return Line position, or -1 if calibration data is missing.
 */
int32_t QTRSensors_readLineWhite(QTRSensors *qtr, uint16_t *values,
                                 QTRReadMode mode, uint16_t blackThresh);

/* --- Emitter control ----------------------------------------------- */

/**
 * @brief  Drive the emitter GPIO pin on or off.
 *
 * Does nothing when emitterGpioPort is NULL.
 *
 * @param  qtr      Initialised descriptor.
 * @param  emitters QTREmittersOn or QTREmittersOff.
 */
void QTRSensors_setEmitters(QTRSensors *qtr, QTREmitters emitters);

#ifdef __cplusplus
}
#endif

#endif /* QTR_SENSORS_H */
