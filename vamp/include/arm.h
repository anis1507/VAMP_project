#ifndef __ARM_H__
#define __ARM_H__

#include "utils.h"

typedef struct {
    int joint_number;
    int rod_number;
    Rectangle* rods;
    char** axis;
    int (*joint_angles_range)[2];
} Arm;

void Arm_init(Arm* arm, int joint_number, int rod_number);
void Arm_destroy(Arm* arm);
void Arm_printInfo(const Arm* arm);
void Arm_calculatePosture(const Arm* arm, Configuration* config);
void Arm_printPosture(const Arm* arm, Configuration* config);
int Arm_collisionDetection(Arm* arm, Configuration* config, Sphere* obstacles, int obstacles_count);
validation_result Arm_motionValidation(Arm* arm, Configuration* A, Configuration* B, Sphere* obstacles, int obstacles_count, int DEBUG);

#endif