
#ifndef FEEDBACK_CONTROL_TOP_H
#define FEEDBACK_CONTROL_TOP_H
#include <stdio.h>
#include <stdlib.h>
#include "FeedbackMotorController.h"

struct AxisFeedbackState {
	double setpoint;
	double currentValue;
	double KP;
	double KI;
	double KD;
};
struct TopFeedbackStateStruct {
	struct AxisFeedbackState *rollState;
	struct AxisFeedbackState *pitchState;
	struct AxisFeedbackState *yawState;
	struct AxisFeedbackState *VxState;
	struct AxisFeedbackState *zState;
};
void *TopControlLoop(void * feedbackArgs);
#endif
