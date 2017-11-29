#pragma once

enum ComponentType {
    SONAR,
    IRANALOG,
    IRDIGITAL,
    LEDSTRIP
};

struct SubMessage {
    /**
     * type of the message:
     * 0 - Sonar
     * 1 - IR Sensor analog
     * 2 - IR Sensor digital
     * 3 - LED Strip
     */
    uint8_t type;

    /**
     * Unique id of the component
     */
    uint8_t id;

    /**
     * Length of the data in this submessage in bytes
     */
    uint8_t length;

    /**
     * Byte array with data
     */
    uint8_t data[16];
};