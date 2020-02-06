/*
 * MotionPlanner.h
 *
 *  Created on: 27.12.2019
 *      Author: Martin
 */

#ifndef INC_MOTIONPLANNER_H_
#define INC_MOTIONPLANNER_H_

#include "GCode.h"
#include "pin.h"

#define MOTION_BUFFER 150
#define MAX_AXIS 4

typedef struct StepBuffer {
	uint8_t buffer0[2][MOTION_BUFFER];
	uint8_t isActive0;
	uint8_t isReady0;
	uint8_t buffer1[2][MOTION_BUFFER];
	uint8_t isActive1;
	uint8_t isReady1;
	uint8_t curBuf;
	uint8_t curBufPos;
	uint32_t startPosBuf0;
	uint32_t startPosBuf1;
	uint32_t endPosBuf0;
	uint32_t endPosBuf1;
	GCodePara_t MotionBuffer[3];
} StepBuffer_t;

typedef struct Axis {
	char name[1];
	pin_t steppin;
	pin_t dirpin;
	pin_t enpin;
	uint16_t spr;
	uint8_t acel;
	uint8_t planningDone;
	uint32_t curPos;
	uint32_t desPos;
	uint8_t curDir;
	StepBuffer_t stepbuffer;
} Axis_t;

Axis_t * getAxis();
uint8_t getNrAxis();

//Initialization related functions
uint8_t initMotionPlanner(uint8_t num);
void initStepBuffer(StepBuffer_t *buffer);
void initMotionAxis(Axis_t *axis);

//Pin related functions
uint8_t initAxisPins(Axis_t *, uint8_t);

//Movement related functions
void getLowerRamps(uint32_t *, GCodePara_t *, uint32_t *);
void getUpperRamps(uint32_t *, GCodePara_t *, uint32_t *);
uint8_t calcDebugMove(uint8_t i);
uint8_t getVeloAtPointDebug(Axis_t *, uint8_t *, uint32_t, uint32_t, uint32_t);
uint8_t getVelo(uint8_t, uint8_t *);
void getNewDir(uint32_t *, GCodePara_t *);
void getNewGCode();
void zeroCheckRamp(uint32_t *);

#endif /* INC_MOTIONPLANNER_H_ */
