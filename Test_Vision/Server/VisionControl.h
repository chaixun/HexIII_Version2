#ifndef VISIONCONTROL_H
#define VISIONCONTROL_H
#include "Kinect_Test.h"

int visionAdjust(double *, bool *);
int visionStepUp(Kinect &, double *);
int visionStepDown(Kinect &, double *);
int visionStepOver(Kinect &, double *);

#endif // VISIONCONTROL_H
