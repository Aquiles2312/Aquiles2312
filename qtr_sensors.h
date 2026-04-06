// qtr_sensors.h

#ifndef QTR_SENSORS_H
#define QTR_SENSORS_H

#include <stdint.h>

// Type definitions
typedef uint8_t QTRSensorPin;

typedef struct {
    QTRSensorPin pin;
    uint16_t value;
} QTRSensor;

// Enumeration for sensor states
typedef enum {
    QTR_ACTIVE,
    QTR_INACTIVE
} QTRSensorState;

// Function declarations
void QTR_setPin(QTRSensor *sensor, QTRSensorPin pin);
void QTR_read(QTRSensor *sensor);
QTRSensorState QTR_getState(QTRSensor *sensor);

#endif // QTR_SENSORS_H
