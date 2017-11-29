/*
 * Copyright (C) 2017  Serhii Vasylchenko
 *
 * This file is part of paparazzi.
 *
 * paparazzi is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2, or (at your option)
 * any later version.
 *
 * paparazzi is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with paparazzi; see the file COPYING.  If not, write to
 * the Free Software Foundation, 59 Temple Place - Suite 330,
 * Boston, MA 02111-1307, USA.
 */

#include "modules/uart_board_manager/uart_board_manager.h"
#include "mcu_periph/uart.h"

#define LED_COUNT 8
#define BUF_SIZE 128 // max size for the message in bytes, both incoming and outgoing
#define BUF_SIZE_ENCODED 256 // max possible size of encoded message with with escape symbols
#define MAX_MSG_NUMBER 10 // max number of sub messages in one message

// better not to use 0xFF because it's a common used byte for color
#define STOP_BYTE 0xFE
#define START_BYTE 0xFD
#define ESCAPE_BYTE 0xFC

uint8_t ledstrip_default[4 * LED_COUNT] = {
	// id, red, green, blue
	0, 0x00, 0x00, 0xFF,
	1, 0x00, 0x00, 0xFF,
	2, 0x00, 0x00, 0xFF,
	3, 0x00, 0x00, 0xFF,
	4, 0x00, 0xFF, 0x00,
	5, 0xFF, 0x00, 0x00,
	6, 0xFF, 0x00, 0x00,
	7, 0x00, 0xFF, 0x00,
}

static void sendMessages(void) {
	toBoardMsg[0] = toBoardMsgCount;

    // add current submessages
    int pos = 0;
    for (int i = 0; i < toBoardMsgCount; i++) {
        toBoardMsg[++pos] = subMessagesToBoard[i]->id;
        toBoardMsg[++pos] = subMessagesToBoard[i]->type;
        toBoardMsg[++pos] = subMessagesToBoard[i]->length;
        for (int j = 0; j < subMessagesToBoard[i]->length; j++) {
            toBoardMsg[++pos] = subMessagesToBoard[i]->data[j];
        }
    }

    calculateChecksum(toBoardMsg, toBoardMsgLength);
    toBoardMsgLength++;
	
	// add start, stop and escape bytes
	encode();
	
	// send result byte array via uart
	
}

static void encode(void) {
	toBoardMsgEncodedLength = toBoardMsgLength;

    uint8_t pos = -1;
    // start byte in the beginning
    toBoardMsgEncoded[++pos] = START_BYTE;
    
    for (int i = 0; i < toBoardMsgLength; i++) {
        if (toBoardMsg[i] == START_BYTE || toBoardMsg[i] == STOP_BYTE || toBoardMsg[i] == ESCAPE_BYTE) {
            // escape this character - put escape byte before and convert the problematic byte
            toPaparazziMsgEncoded[++pos] = ESCAPE_BYTE;
            // use XOR between escape byte and problematic byte for conversion
            toPaparazziMsgEncoded[++pos] = ESCAPE_BYTE ^ toBoardMsg[i];

            toPaparazziMsgEncodedLength++;
        } else {
            toPaparazziMsgEncoded[++pos] = toBoardMsg[i];
        }
    }

    // stop byte in the end
    toPaparazziMsgEncoded[++pos] = STOP_BYTE;
}

static bool decode(uint8_t size) {
	uint8_t pos = -1;

    if (fromBoardMsgEncoded[0] != START_BYTE || fromPaparazziMsgEncoded[size - 1] != STOP_BYTE) {
        // start or stop byte is not found in the right place
        return false;
    }

    for (int i = 1; i < size - 1; i++) { // ignore start and stop bytes at this point
        if (fromBoardMsgEncoded[i] == ESCAPE_BYTE) {
            // next character is escaped, we need to convert it back
            fromPaparazziMsg[++pos] = ESCAPE_BYTE ^ fromPaparazziMsgEncoded[++i];
        } else {
            fromPaparazziMsg[++pos] = fromPaparazziMsgEncoded[i];
        }
    }

    return true;
}

static void setLEDDefault(void) {
	if (toBoardMsgCount < MAX_MSG_NUMBER) {
		Submessage subMessage;
		subMessage.type = LEDSTRIP;
		subMessage.id = 0x00; // id is not relevant for led strip
		subMessage.length = 4 * LED_COUNT; // set all LEDs
		
		for (uint8_t i = 0; i < 4 * LED_COUNT; i++) {
			submessage.data[i] = ledstrip_default[i];
		}
			
		subMessagesToBoard[toBoardMsgCount] = subMessage;
	    toBoardMsgCount++;
		toBoardMsgLength += 3 + subMessage.length;
	}
}

static void setLEDAll(uint8_t red, uint8_t green, uint8_t blue) {
	if (toBoardMsgCount < MAX_MSG_NUMBER) {
		Submessage subMessage;
		subMessage.type = LEDSTRIP;
		subMessage.id = 0x00; // id is not relevant for led strip
		subMessage.length = 4;
		
		subMessage.data[0] = 0xFF;
		subMessage.data[1] = red;
		subMessage.data[2] = green;
		subMessage.data[3] = blue;
		
		subMessagesToBoard[toBoardMsgCount] = subMessage;
		toBoardMsgCount++;
		toBoardMsgLength += 3 + subMessage.length;
	}
}

static void setLEDAllRed(void) {
	setLEDAll(0xFF, 0, 0);
}

static void setLEDAllGreen(void) {
	setLEDAll(0, 0xFF, 0);
}

static void setLEDAllBlue(void) {
	setLEDAll(0, 0, 0xFF);
}

static void setLEDAllDark(void) {
	setLEDAll(0, 0, 0);
}

static void setLEDAllCustom(uint32_t* id_color[LED_COUNT]) {
	if (toBoardMsgCount < MAX_MSG_NUMBER) {
		Submessage subMessage;
		subMessage.type = LEDSTRIP;
		subMessage.id = 0x00; // id is not relevant for led strip
		subMessage.length = 4 * LED_COUNT; // set all LEDs
		
		uint8_t pos = -1:
		for (uint8_t i = 0; i < LED_COUNT; i++) {
			submessage.data[++pos] = id_color >> 24; // first byte from left - id
			submessage.data[++pos] = id_color >> 16; // second byte - red
			submessage.data[++pos] = id_color >> 8; // third byte - green
			submessage.data[++pos] = id_color; // fourth byte - blue
		}
		
		subMessagesToBoard[toBoardMsgCount] = subMessage;
		toBoardMsgCount++;
		toBoardMsgLength += 3 + subMessage.length;
	}
}

static void setLEDCustom(uint8_t id, uint8_t red, uint8_t green, uint8_t blue) {
	if (toBoardMsgCount < MAX_MSG_NUMBER) {
		Submessage subMessage;
		subMessage.type = LEDSTRIP;
		subMessage.id = 0x00; // id is not relevant for led strip
		subMessage.length = 4;
		
		subMessage.data[0] = id;
		subMessage.data[1] = red;
		subMessage.data[2] = green;
		subMessage.data[3] = blue;
		
		subMessagesToBoard[toBoardMsgCount] = subMessage;
		toBoardMsgCount++;
		toBoardMsgLength += 3 + subMessage.length;
	}
}
