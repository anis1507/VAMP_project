#ifndef __UTILS_H__
#define __UTILS_H__

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <time.h>
#include <stdint.h>

#define RED "\e[0;31m"
#define NONE "\e[0m"
#define CYAN "\e[0;36m"
#define GREEN "\e[0;32m"
#define BOLD "\e[1m"
#define PURPLE "\e[0;35m"

#define PI 3.1415926
#define RAD(X) ((float)(X) * PI / 180.0)
#define DEG(X) ((X) * 180.0 / PI)
#define CHECKINGPOINTSNUMBER 32

typedef struct {
    int joint_number;
    int* joint_angles;
} Configuration;

static inline Configuration Configuration_create(int joint_number) {
    Configuration c;
    c.joint_number = joint_number;
    c.joint_angles = (int*)calloc(joint_number, sizeof(int));
    return c;
}

static inline void Configuration_destroy(Configuration* config) {
    if (config->joint_angles) free(config->joint_angles);
    config->joint_angles = NULL;
}

static inline Configuration Configuration_subtract(const Configuration* a, const Configuration* b) {
    Configuration c = Configuration_create(a->joint_number);
    for (int i = 0; i < a->joint_number; ++i)
        c.joint_angles[i] = a->joint_angles[i] - b->joint_angles[i];
    return c;
}

static inline int Configuration_equals(const Configuration* a, const Configuration* b) {
    if (a->joint_number != b->joint_number) return 0;
    for (int i = 0; i < a->joint_number; ++i)
        if (a->joint_angles[i] != b->joint_angles[i]) return 0;
    return 1;
}

typedef struct {
    double x, y, z;
} Vec3;

static inline Vec3 Vec3_create(double x, double y, double z) {
    Vec3 v = {x, y, z};
    return v;
}

static inline Vec3 Vec3_add(const Vec3* a, const Vec3* b) {
    return Vec3_create(a->x + b->x, a->y + b->y, a->z + b->z);
}

static inline Vec3 Vec3_sub(const Vec3* a, const Vec3* b) {
    return Vec3_create(a->x - b->x, a->y - b->y, a->z - b->z);
}

static inline Vec3 Vec3_mul(const Vec3* a, double scalar) {
    return Vec3_create(a->x * scalar, a->y * scalar, a->z * scalar);
}

static inline Vec3 Vec3_normalize(const Vec3* v) {
    double len = sqrt(v->x * v->x + v->y * v->y + v->z * v->z);
    if (len == 0) return Vec3_create(0, 0, 0);
    return Vec3_create(v->x / len, v->y / len, v->z / len);
}

static inline float Vec3_dot(const Vec3* a, const Vec3* b) {
    return (float)(a->x * b->x + a->y * b->y + a->z * b->z);
}

static inline Vec3 Vec3_cross(const Vec3* a, const Vec3* b) {
    return Vec3_create(
        a->y * b->z - a->z * b->y,
        a->z * b->x - a->x * b->z,
        a->x * b->y - a->y * b->x
    );
}

typedef struct {
    float x, y, z;
} Point;

static inline Point Point_create(float x, float y, float z) {
    Point p = {x, y, z};
    return p;
}

typedef struct {
    Point bottom_center;
    Point top_center;
    Point center;
    float width, length, height;
    Vec3 side_vector;
} Rectangle;

typedef struct {
    float radius;
    Point center;
} Sphere;

typedef struct {
    int valid;
    double FK_time;
    double CC_time;
} validation_result;

#endif
