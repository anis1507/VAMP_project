#define _GNU_SOURCE
#include "arm.h"
#include "utils.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <time.h>
int segment_sphere_collision(Point p1, Point p2, Point center, float radius);

void Arm_init(Arm* arm, int joint_number, int rod_number) {
    arm->joint_number = joint_number;
    arm->rod_number = rod_number;
    arm->rods = (Rectangle*)calloc(rod_number, sizeof(Rectangle));
    arm->axis = (char**)calloc(joint_number, sizeof(char*));
    for (int i = 0; i < joint_number; ++i) {
        arm->axis[i] = (char*)malloc(2);
        strcpy(arm->axis[i], "z");
    }
    arm->joint_angles_range = (int(*)[2])calloc(joint_number, sizeof(int[2]));
    for (int i = 0; i < joint_number; ++i) {
        arm->joint_angles_range[i][0] = 0;
        arm->joint_angles_range[i][1] = 180;
    }
}

void Arm_destroy(Arm* arm) {
    for (int i = 0; i < arm->joint_number; ++i) {
        free(arm->axis[i]);
    }
    free(arm->axis);
    free(arm->rods);
    free(arm->joint_angles_range);
}

void Arm_printInfo(const Arm* arm) {
    printf("\n#===============================================================#\n");
    printf("|                           Arm Information                    |\n");
    printf("#===============================================================#\n");
    printf("| Number of joints: %d                                         |\n", arm->joint_number);
    printf("| Number of rods:   %d                                         |\n", arm->rod_number);
    printf("#===============================================================#\n\n");
    for (int i = 0; i < arm->joint_number; ++i) {
        printf("| Joint %d: Axis: %s, Angle range: [%d, %d]%*s|\n",
            i + 1,
            arm->axis[i],
            arm->joint_angles_range[i][0],
            arm->joint_angles_range[i][1],
            52 - (int)strlen(arm->axis[i]), "");
    }
    printf("#===============================================================#\n\n");
}

void Arm_calculatePosture(const Arm* arm, Configuration* config) {
    Point last_top_center = Point_create(0, 0, 0);
    arm->rods[0].bottom_center = last_top_center;
    arm->rods[0].top_center = Point_create(0, 0, arm->rods[0].height);
    arm->rods[0].center = Point_create(
        (arm->rods[0].bottom_center.x + arm->rods[0].top_center.x) * 0.5f,
        (arm->rods[0].bottom_center.y + arm->rods[0].top_center.y) * 0.5f,
        (arm->rods[0].bottom_center.z + arm->rods[0].top_center.z) * 0.5f
    );

    int xy_rotation_angle = config->joint_angles[0];
    int z_rotation_angle;
    if (strcmp(arm->axis[0], "x") == 0) {
        z_rotation_angle = config->joint_angles[1];
    } else {
        z_rotation_angle = 180 - config->joint_angles[1];
    }

    arm->rods[0].side_vector = Vec3_create(sin(RAD(xy_rotation_angle)), -cos(RAD(xy_rotation_angle)), 0);

    for (int i = 1; i < arm->rod_number - 1; i++) {
        if (i != 1) {
            if (strcmp(arm->axis[i], "x") == 0) {
                z_rotation_angle = z_rotation_angle - (90 - config->joint_angles[i]);
            } else if (strcmp(arm->axis[i], "-x") == 0) {
                z_rotation_angle = z_rotation_angle - (90 - (180 - config->joint_angles[i]));
            }
        }
        arm->rods[i].bottom_center = arm->rods[i - 1].top_center;
        Vec3 bottom_center = Vec3_create(
            arm->rods[i].bottom_center.x,
            arm->rods[i].bottom_center.y,
            arm->rods[i].bottom_center.z
        );
        Vec3 local_top_center = Vec3_create(
            arm->rods[i].height * cos(RAD(z_rotation_angle)) * cos(RAD(xy_rotation_angle)),
            arm->rods[i].height * cos(RAD(z_rotation_angle)) * sin(RAD(xy_rotation_angle)),
            arm->rods[i].height * sin(RAD(z_rotation_angle))
        );
        Vec3 top_center = Vec3_add(&bottom_center, &local_top_center);
        arm->rods[i].top_center = Point_create((float)top_center.x, (float)top_center.y, (float)top_center.z);
        arm->rods[i].side_vector = arm->rods[0].side_vector;
        arm->rods[i].center = Point_create(
            (arm->rods[i].bottom_center.x + arm->rods[i].top_center.x) * 0.5f,
            (arm->rods[i].bottom_center.y + arm->rods[i].top_center.y) * 0.5f,
            (arm->rods[i].bottom_center.z + arm->rods[i].top_center.z) * 0.5f
        );
    }

    arm->rods[arm->rod_number - 1].bottom_center = arm->rods[arm->rod_number - 2].top_center;
    Vec3 bottom_center = Vec3_create(
        arm->rods[arm->rod_number - 1].bottom_center.x,
        arm->rods[arm->rod_number - 1].bottom_center.y,
        arm->rods[arm->rod_number - 1].bottom_center.z
    );
    Vec3 prev_top = Vec3_create(
        arm->rods[arm->rod_number - 2].top_center.x,
        arm->rods[arm->rod_number - 2].top_center.y,
        arm->rods[arm->rod_number - 2].top_center.z
    );
    Vec3 prev_bottom = Vec3_create(
        arm->rods[arm->rod_number - 2].bottom_center.x,
        arm->rods[arm->rod_number - 2].bottom_center.y,
        arm->rods[arm->rod_number - 2].bottom_center.z
    );
    Vec3 diff = Vec3_sub(&prev_top, &prev_bottom);
    Vec3 direction = Vec3_normalize(&diff);
    Vec3 local_top_center = Vec3_mul(&direction, arm->rods[arm->rod_number - 1].height);
    Vec3 top_center = Vec3_add(&bottom_center, &local_top_center);
    arm->rods[arm->rod_number - 1].top_center = Point_create((float)top_center.x, (float)top_center.y, (float)top_center.z);
    arm->rods[arm->rod_number - 1].side_vector = arm->rods[0].side_vector;
    arm->rods[arm->rod_number - 1].center = Point_create(
        (arm->rods[arm->rod_number - 1].bottom_center.x + arm->rods[arm->rod_number - 1].top_center.x) * 0.5f,
        (arm->rods[arm->rod_number - 1].bottom_center.y + arm->rods[arm->rod_number - 1].top_center.y) * 0.5f,
        (arm->rods[arm->rod_number - 1].bottom_center.z + arm->rods[arm->rod_number - 1].top_center.z) * 0.5f
    );
}

void Arm_printPosture(const Arm* arm, Configuration* config) {
    Arm_calculatePosture(arm, config);
    printf("Posture:\n");
    for (int i = 0; i < arm->rod_number; i++) {
        printf("Rod %d: (%.2f, %.2f, %.2f) -> (%.2f, %.2f, %.2f) | Center: (%.2f, %.2f, %.2f) | Side vector: (%.2f, %.2f, %.2f)\n",
            i + 1,
            arm->rods[i].bottom_center.x, arm->rods[i].bottom_center.y, arm->rods[i].bottom_center.z,
            arm->rods[i].top_center.x, arm->rods[i].top_center.y, arm->rods[i].top_center.z,
            arm->rods[i].center.x, arm->rods[i].center.y, arm->rods[i].center.z,
            arm->rods[i].side_vector.x, arm->rods[i].side_vector.y, arm->rods[i].side_vector.z
        );
    }
    printf("\n");
}

int Arm_collisionDetection(Arm* arm, Configuration* config, Sphere* obstacles, int obstacles_count) {
    Arm_calculatePosture(arm, config);
    for (int i = 0; i < obstacles_count; ++i) {
        for (int j = 0; j < arm->rod_number; ++j) {
            if (segment_sphere_collision(
                    arm->rods[j].bottom_center,
                    arm->rods[j].top_center,
                    obstacles[i].center,
                    obstacles[i].radius)) {
                return 1;
            }
        }
    }
    return 0;
}

validation_result Arm_motionValidation(Arm* arm, Configuration* q1, Configuration* q2, Sphere* obstacles, int obstacles_number, int step_size) {
    validation_result result = {1, 0.0, 0.0};
    int steps = 0;
    for (int j = 0; j < q1->joint_number; ++j) {
        int diff = abs(q2->joint_angles[j] - q1->joint_angles[j]);
        int n = diff / step_size;
        if (n > steps) steps = n;
    }
    if (steps < 1) steps = 1;
    Configuration q_interp = Configuration_create(q1->joint_number);

    for (int s = 1; s <= steps; ++s) {
        for (int j = 0; j < q1->joint_number; ++j) {
            q_interp.joint_angles[j] = q1->joint_angles[j] + (q2->joint_angles[j] - q1->joint_angles[j]) * s / steps;
        }

        clock_t t_fk_start = clock();
        Arm_calculatePosture(arm, &q_interp);
        clock_t t_fk_end = clock();
        result.FK_time += ((double)(t_fk_end - t_fk_start)) * 1000.0 / CLOCKS_PER_SEC;

        clock_t t_cc_start = clock();
        if (Arm_collisionDetection(arm, &q_interp, obstacles, obstacles_number)) {
            clock_t t_cc_end = clock();
            result.CC_time += ((double)(t_cc_end - t_cc_start)) * 1000.0 / CLOCKS_PER_SEC;
            result.valid = 0;
            break;
        }
        clock_t t_cc_end = clock();
        result.CC_time += ((double)(t_cc_end - t_cc_start)) * 1000.0 / CLOCKS_PER_SEC;
    }
    Configuration_destroy(&q_interp);
    return result;
}

validation_result Arm_batchCollisionDetection(Arm* arm, Configuration* configs, int configs_count, Sphere* obstacles, int obstacles_count, int DEBUG) {
    validation_result result = {1, 0.0, 0.0};
    for (int i = 0; i < configs_count; ++i) {
        Arm_calculatePosture(arm, &configs[i]);
        if (Arm_collisionDetection(arm, &configs[i], obstacles, obstacles_count)) {
            result.valid = 0;
            break;
        }
    }
    return result;
}

int segment_sphere_collision(Point p1, Point p2, Point center, float radius) {
    float vx = p2.x - p1.x;
    float vy = p2.y - p1.y;
    float vz = p2.z - p1.z;
    float wx = center.x - p1.x;
    float wy = center.y - p1.y;
    float wz = center.z - p1.z;
    float c1 = vx*wx + vy*wy + vz*wz;
    float c2 = vx*vx + vy*vy + vz*vz;
    float t = (c2 == 0) ? 0 : c1 / c2;
    if (t < 0) t = 0;
    if (t > 1) t = 1;
    float px = p1.x + t * vx;
    float py = p1.y + t * vy;
    float pz = p1.z + t * vz;
    float dx = px - center.x;
    float dy = py - center.y;
    float dz = pz - center.z;
    float dist2 = dx*dx + dy*dy + dz*dz;
    return dist2 <= radius*radius;
}