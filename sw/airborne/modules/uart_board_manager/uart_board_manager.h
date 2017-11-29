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
 
#include "std.h"
#include "SubMessage.h"

extern void sendMessages(void);
extern void setLEDDefault(void);
extern void setLEDALL(uint8_t red, uint8_t green, uint8_t blue) 
extern void setLEDAllRed(void);
extern void setLEDAllGreen(void);
extern void setLEDAllBlue(void);
extern void setLEDAllDark(void);
extern void setLEDAllCustom(uint32_t* id_color[LED_COUNT]);
extern void setLEDCustom(uint8_t id, uint8_t red, uint8_t green, uint8_t blue);

void encode(void);
bool decode(uint8_t size);
void calculateChecksum(uint8_t *pkt, uint8_t const length);

SubMessage* subMessagesToBoard[MAX_MSG_NUMBER];

uint8_t toBoardMsgCount = 0;
uint8_t toBoardMsgLength;
uint8_t toBoardMsgEncodedLength;
uint8_t toBoardMsg[BUF_SIZE];
uint8_t toBoardMsgEncoded[BUF_SIZE_ENCODED];