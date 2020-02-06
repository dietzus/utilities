/*
 * Motionplanner.c
 *
 *  Created on: 27.12.2019
 *      Author: Martin
 */

#include "usbd_cdc_if.h"
#include "MotionPlanner.h"
#include "pin.h"
#include <string.h>
#include <ctype.h>
#include <stdio.h>

const uint8_t MSTEPS[MAX_AXIS] = {16, 16, 16, 16};
const uint16_t FSPR[MAX_AXIS] = {200, 200, 200, 200};
const uint8_t ACEL[MAX_AXIS] = {100, 100, 100, 100};

static uint8_t initialized = 0;
static Axis_t axis[MAX_AXIS];
static uint8_t numAxis = 0;
static uint8_t maxV[MAX_AXIS] = {1, 1, 1, 1};

static GCodePara_t oldGCode;
static GCodePara_t curGCode;
static GCodePara_t newGCode;
static uint32_t oldPos[MAX_AXIS] = {0, 0, 0, 0};
static uint32_t curPos[MAX_AXIS] = {0, 0, 0, 0};
static uint32_t newPos[MAX_AXIS] = {0, 0, 0, 0};
static int16_t oldV[MAX_AXIS] = {0, 0, 0, 0}; 		//0=stillstand, 1=maxV, 255=minV
static int16_t curV[MAX_AXIS] = {0, 0, 0, 0}; 		//0=stillstand, 1=maxV, 255=minV
static int16_t newV[MAX_AXIS] = {0, 0, 0, 0}; 		//0=stillstand, 1=maxV, 255=minV
static uint8_t oldD[MAX_AXIS] = {1, 1, 1, 1}; 		//0=pos wird kleiner, 1= pos wird groesser
static uint8_t curD[MAX_AXIS] = {1, 1, 1, 1}; 		//0=pos wird kleiner, 1= pos wird groesser
static uint8_t newD[MAX_AXIS] = {1, 1, 1, 1}; 		//0=pos wird kleiner, 1= pos wird groesser
static uint32_t startRamp[MAX_AXIS] = {0, 0, 0, 0};
static uint32_t endRamp[MAX_AXIS] = {0, 0, 0, 0};

static uint8_t isNewGCode = 1;

Axis_t * getAxis() {
	return &axis[0];
}

uint8_t getNrAxis() {
	return numAxis;
}

void initMotionAxis(Axis_t *axis) {
	axis->planningDone = 1;
	axis->curPos = 0;
	axis->desPos = 0;
	axis->curDir = 0;
	initStepBuffer(&(axis->stepbuffer));
}

void initStepBuffer(StepBuffer_t *buffer) {
	for(uint8_t i=0; i<MOTION_BUFFER; i++) {
		buffer->buffer0[0][i] = 0;
		buffer->buffer1[0][i] = 0;
		buffer->buffer0[1][i] = 0;
		buffer->buffer1[1][i] = 0;
	}
	buffer->isActive0 = 0;
	buffer->isReady0 = 1;
	buffer->isActive1 = 0;
	buffer->isReady1 = 1;
	buffer->curBuf = 0;
	buffer->startPosBuf0 = 0;
	buffer->startPosBuf0 = 1;
	buffer->endPosBuf0 = 0;
	buffer->endPosBuf0 = 1;
	buffer->curBufPos = 0;
}

uint8_t initMotionPlanner(uint8_t num) {
	if (initialized) {
		return numAxis;
	}

	uint8_t i = MAX_AXIS;

	if(num < i) {
		i = num;
	}
	numAxis = i;

	for (int j=0; j<i; j++) {
		char tempchar;

		switch(j) {
		case 0:
			tempchar = 'X';
			break;
		case 1:
			tempchar = 'Y';
			break;
		case 2:
			tempchar = 'Z';
			break;
		default:
			tempchar = j + 65;
		}

		initAxisPins(&axis[j], j);
		axis[j].name[0] = tempchar;
		axis[j].spr = MSTEPS[j] * FSPR[j];
		axis[j].acel = ACEL[j];
		initMotionAxis(&(axis[j]));
		curPos[j] = 0;
	}

	initialized = 1;
	return numAxis;
}

uint8_t initAxisPins(Axis_t *axis, uint8_t i) {
	pin_t temppins[3];

	switch (i) {
	case 0:
		temppins[0].Port = GPIOB;
		temppins[0].num = GPIO_PIN_1;
		temppins[1].Port = GPIOB;
		temppins[1].num = GPIO_PIN_0;
		temppins[2].Port = GPIOB;
		temppins[2].num = GPIO_PIN_12;
		break;
	case 1:
		temppins[0].Port = GPIOA;
		temppins[0].num = GPIO_PIN_3;
		temppins[1].Port = GPIOA;
		temppins[1].num = GPIO_PIN_2;
		temppins[2].Port = GPIOB;
		temppins[2].num = GPIO_PIN_13;
		break;
	case 2:
		temppins[0].Port = GPIOA;
		temppins[0].num = GPIO_PIN_1;
		temppins[1].Port = GPIOA;
		temppins[1].num = GPIO_PIN_0;
		temppins[2].Port = GPIOB;
		temppins[2].num = GPIO_PIN_14;
		break;
	case 3:
		temppins[0].Port = GPIOB;
		temppins[0].num = GPIO_PIN_6;
		temppins[1].Port = GPIOB;
		temppins[1].num = GPIO_PIN_7;
		temppins[2].Port = GPIOB;
		temppins[2].num = GPIO_PIN_15;
		break;
	default:
		return 0;
	}

	axis->steppin = temppins[0];
	axis->dirpin = temppins[1];
	axis->enpin = temppins[2];

	return 1;
}

uint8_t calcMove() {

	for(uint8_t i=0; i<MAX_AXIS; i++) {
		axis[i].planningDone = 0;
		uint8_t tempdelay;
		uint8_t *tempbuffer;

		if(axis[i].stepbuffer.isActive0) {
			tempbuffer = &(axis[i].stepbuffer.buffer1[0][0]);
		} else {
			tempbuffer = &(axis[i].stepbuffer.buffer0[0][0]);
		}

		for(uint8_t j=0; j<MOTION_BUFFER; j++) {
			if(getVelo(j, &tempbuffer[j])) {
				getNewGCode();
			}
		}

		axis[i].planningDone = 1;
	}



	return 0;
}

void getNewGCode() {
	copyGCode(&oldGCode, &curGCode);
	copyGCode(&curGCode, &newGCode);
	if(!getMovementCom(&newGCode))  {					//TODO: make it intelligent
		if(isNewGCode) {
			printf("No new movement command available.");
			isNewGCode = 0;
		}
	} else {
		isNewGCode = 1;
	}
}

uint8_t calcDebugMove(uint8_t i) {
	Axis_t *tempaxis = &axis[i];
	StepBuffer_t *tempbuffer = &(axis[i].stepbuffer);
	uint8_t tempdelay = 0;
	uint8_t tempdir = 1;

	tempaxis->planningDone = 0;
	uint32_t tempEndPos;
	if(tempbuffer->isActive0) {
		tempEndPos = tempbuffer->endPosBuf1 + 1;
		tempdir = tempbuffer->buffer0[1][MOTION_BUFFER-1];
		for(uint8_t i=0; i<MOTION_BUFFER; i++) {
			if(getVeloAtPointDebug(tempaxis, &tempdelay, 0, 20000, tempEndPos) == 1) {
				if(isNextMovementPara()) {

				}
				tempdir ^= 1;
				tempEndPos = 0;
			} else {
				tempEndPos++;
			}
			tempbuffer->buffer1[0][i] = tempdelay;
			tempbuffer->buffer1[1][i] = tempdir;
		}
		tempbuffer->endPosBuf0 = tempEndPos;
		tempbuffer->isReady1 = 1;
	} else {
		tempEndPos = tempbuffer->endPosBuf0 + 1;
		tempdir = tempbuffer->buffer1[1][MOTION_BUFFER-1];
		for(uint8_t i=0; i<MOTION_BUFFER; i++) {
			if(getVeloAtPointDebug(tempaxis, &tempdelay, 0, 20000, tempEndPos) == 1) {
				tempdir ^= 1;
				tempEndPos = 0;
			} else {
				tempEndPos++;
			}
			tempbuffer->buffer0[0][i] = tempdelay;
			tempbuffer->buffer0[1][i] = tempdir;
		}
		tempbuffer->endPosBuf1 = tempEndPos;
		tempbuffer->isReady0 = 1;
	}

	tempaxis->planningDone = 1;

	return 1;
}

uint8_t getVeloAtPointDebug(Axis_t *axis, uint8_t *curValue, uint32_t startPos, uint32_t endPos, uint32_t curPos) {
	uint8_t acel = axis->acel;
	uint8_t maxVal = 255;
	uint32_t ramp = maxVal * axis->spr / acel;

	if(curPos >= endPos) {
		*curValue = maxVal;
		return 1;
	}

	uint32_t relCurPos = curPos - startPos;
	uint32_t totSteps = endPos - startPos;
	uint32_t hTotSteps = totSteps / 2;

	if(relCurPos == 0 || (totSteps - relCurPos) == 0) {
		relCurPos = 1;
	}

	if(hTotSteps <= ramp) {
		if(relCurPos <= hTotSteps) {
			*curValue = maxVal - (maxVal - ramp / relCurPos);
		} else {
			*curValue = maxVal - (maxVal - ramp / (totSteps - relCurPos));
		}
	} else {
		if(relCurPos <= ramp) {
			*curValue = maxVal - (maxVal - ramp / relCurPos);
		} else if (relCurPos >= (totSteps - ramp)) {
			*curValue = maxVal - (maxVal - ramp / (totSteps - relCurPos));
		} else {
			*curValue = 1;
		}
	}

	if(*curValue == 0) {
		*curValue = 1;
	}

	return 0;
}

uint8_t getVelo (uint8_t curAxis, uint8_t *curValue) {
	static uint32_t relCurPos[MAX_AXIS];
	static uint32_t totSteps[MAX_AXIS];
	static uint8_t shortMove[MAX_AXIS];
	static uint32_t crossover[MAX_AXIS];

	uint8_t maxVal = 255;
	uint32_t tempint;

	if(isNewGCode) {
		for(int i=0; i<MAX_AXIS; i++) {
			relCurPos[i] = abs(curPos[i] - oldPos[i]);
			totSteps[i] = abs(newPos[i] - oldPos[i]);

			if(relCurPos[i] == 0 || (totSteps[i] - relCurPos[i]) == 0) {
				relCurPos[i] = 1;								//TODO: ist das schlau?
			}

			if(totSteps[i] > (startRamp[i] + endRamp[i])) {
				shortMove[i] = 0;
			} else {
				shortMove[i] = 1;
				uint8_t found = 0;
				uint8_t crfound = 0;
				int16_t tempdif = 256;
				int16_t tempdif2 = 256;
				tempint = 0;
				crossover[i] = 0;
				while (!crfound) {
					tempdif2 = abs((oldPos[i] + ACEL[i] * tempint * (curD[i] * 2 - 1)) - (newPos[i] + ACEL[i] * tempint * (curD[i] * (-2) + 1)));
					if(tempdif2 < tempdif) {
						tempdif = tempdif2;
						crossover[i] = tempint;
						found = 1;
					} else if (found) {
						crfound = 1;
					}
					tempint++;
				}
			}
		}
		isNewGCode = 0;
	}

	if(shortMove[curAxis]) {
		if(relCurPos[curAxis] < crossover[curAxis]) {
			*curValue = maxVal - (maxVal - ramp / relCurPos);
		} else {
			*curValue = maxVal - (maxVal - ramp / (totSteps - relCurPos));
		}
	} else {
		if(relCurPos[curAxis] < startRamp[curAxis]) {

		} else if (relCurPos[curAxis] > (newPos[curAxis] - endRamp[curAxis])) {

		} else {
			*curValue = maxV[curAxis];
		}
	}

	curPos[curAxis]++;
	if(curPos[curAxis] == newPos[curAxis]) {
		return 1;
	}
	return 0;
}

void getLowerRamps(uint32_t *startpos, GCodePara_t *move, uint32_t *ramps) {

	uint32_t tempint = 0;
	uint8_t tempiter = 0;

	if((move->validX) && (move->X != startpos[tempiter])) {
		if((oldD[tempiter] == curD[tempiter])) {
			if(curV[tempiter] <= maxV[tempiter]) {
				tempint = maxV[tempiter] - curV[tempiter];
			} else {
				tempint = curV[tempiter] - maxV[tempiter];
			}
		} else {
			tempint = curV[tempiter] + maxV[tempiter];
		}
			tempint /= ACEL[tempiter];
	}
	oldD[tempiter] = curD[tempiter];
	curD[tempiter] = newD[tempiter];
	ramps[tempiter] = tempint;
	tempiter++;

	if((move->validY) && (move->Y != startpos[tempiter])) {
		if((oldD[tempiter] == curD[tempiter])) {
			if(curV[tempiter] <= maxV[tempiter]) {
				tempint = maxV[tempiter] - curV[tempiter];
			} else {
				tempint = curV[tempiter] - maxV[tempiter];
			}
		} else {
			tempint = curV[tempiter] + maxV[tempiter];
		}
			tempint /= ACEL[tempiter];
	}
	oldD[tempiter] = curD[tempiter];
	curD[tempiter] = newD[tempiter];
	ramps[tempiter] = tempint;
	tempiter++;

	if((move->validZ) && (move->Z != startpos[tempiter])) {
		if((oldD[tempiter] == curD[tempiter])) {
			if(curV[tempiter] <= maxV[tempiter]) {
				tempint = maxV[tempiter] - curV[tempiter];
			} else {
				tempint = curV[tempiter] - maxV[tempiter];
			}
		} else {
			tempint = curV[tempiter] + maxV[tempiter];
		}
			tempint /= ACEL[tempiter];
	}
	oldD[tempiter] = curD[tempiter];
	curD[tempiter] = newD[tempiter];
	ramps[tempiter] = tempint;
	tempiter++;

	if((move->validE) && (move->E != startpos[tempiter])) {
		if((oldD[tempiter] == curD[tempiter])) {
			if(curV[tempiter] <= maxV[tempiter]) {
				tempint = maxV[tempiter] - curV[tempiter];
			} else {
				tempint = curV[tempiter] - maxV[tempiter];
			}
		} else {
			tempint = curV[tempiter] + maxV[tempiter];
		}
			tempint /= ACEL[tempiter];
	}
	oldD[tempiter] = curD[tempiter];
	curD[tempiter] = newD[tempiter];
	ramps[tempiter] = tempint;
}

void getUpperRamps(uint32_t *endpos, GCodePara_t *move, uint32_t *ramps) {
	uint32_t tempint = 0;
	uint8_t tempiter = 0;

	if((move->validX) && (move->X != endpos[tempiter])) {
		if((curD[tempiter] == newD[tempiter])) {
			if(newV[tempiter] <= maxV[tempiter]) {
				tempint = maxV[tempiter] - newV[tempiter];
			} else {
				tempint = newV[tempiter] - maxV[tempiter];
			}
		} else {
			tempint = newV[tempiter] + maxV[tempiter];
		}
		tempint /= (2 * ACEL[tempiter]);
	}
	ramps[tempiter] = tempint;
	tempiter++;

	if((move->validY) && (move->Y != endpos[tempiter])) {
		if((curD[tempiter] == newD[tempiter])) {
			if(newV[tempiter] <= maxV[tempiter]) {
				tempint = maxV[tempiter] - newV[tempiter];
			} else {
				tempint = newV[tempiter] - maxV[tempiter];
			}
		} else {
			tempint = newV[tempiter] + maxV[tempiter];
		}
		tempint /= (2 * ACEL[tempiter]);
	}
	ramps[tempiter] = tempint;
	tempiter++;

	if((move->validZ) && (move->Z != endpos[tempiter])) {
		if((curD[tempiter] == newD[tempiter])) {
			if(newV[tempiter] <= maxV[tempiter]) {
				tempint = maxV[tempiter] - newV[tempiter];
			} else {
				tempint = newV[tempiter] - maxV[tempiter];
			}
		} else {
			tempint = newV[tempiter] + maxV[tempiter];
		}
		tempint /= (2 * ACEL[tempiter]);
	}
	ramps[tempiter] = tempint;
	tempiter++;

	if((move->validE) && (move->E != endpos[tempiter])) {
		if((curD[tempiter] == newD[tempiter])) {
			if(newV[tempiter] <= maxV[tempiter]) {
				tempint = maxV[tempiter] - newV[tempiter];
			} else {
				tempint = newV[tempiter] - maxV[tempiter];
			}
		} else {
			tempint = newV[tempiter] + maxV[tempiter];
		}
		tempint /= (2 * ACEL[tempiter]);
	}
	ramps[tempiter] = tempint;
}

void zeroCheckRamp(uint32_t *ramps) {
	for(int i=0; i<MAX_AXIS; i++) {
		if(ramps[i] == 0) ramps[i] = 1;
	}
}

void getNewDir(uint32_t *curPos, GCodePara_t *move) {
	uint8_t tempint = 0;

	if(move->validX) {
		if(move->X < curPos[tempint]) {
			newD[tempint] = 0;
		} else {
			newD[tempint] = 1;
		}
	}
	tempint++;

	if(move->validY) {
		if(move->Y < curPos[tempint]) {
			newD[tempint] = 0;
		} else {
			newD[tempint] = 1;
		}
	}
	tempint++;

	if(move->validZ) {
		if(move->Z < curPos[tempint]) {
			newD[tempint] = 0;
		} else {
			newD[tempint] = 1;
		}
	}
	tempint++;

	if(move->validE) {
		if(move->E < curPos[tempint]) {
			newD[tempint] = 0;
		} else {
			newD[tempint] = 1;
		}
	}
}
