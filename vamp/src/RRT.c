#include "RRT.h"
#include "arm.h"
#include <stdlib.h>
#include <string.h>
#include <math.h>

void RRT_init(RRT* rrt, int joint_number, int max_nodes) {
    rrt->tree = (Configuration*)calloc(max_nodes, sizeof(Configuration));
    rrt->tree_size = 0;
    rrt->tree_capacity = max_nodes;
    rrt->parent_indices = (int*)calloc(max_nodes, sizeof(int));
    rrt->step_size = 5;
}

void RRT_destroy(RRT* rrt) {
    for (int i = 0; i < rrt->tree_size; ++i)
        Configuration_destroy(&rrt->tree[i]);
    free(rrt->tree);
    free(rrt->parent_indices);
}

int RRT_findNearest(RRT* rrt, Configuration* q) {
    int nearest_index = 0;
    double min_norm_square = 0.0;
    for (int i = 0; i < rrt->tree[0].joint_number; ++i) {
        double diff = RAD((double)rrt->tree[0].joint_angles[i]) - RAD((double)q->joint_angles[i]);
        min_norm_square += diff * diff;
    }
    for (int i = 1; i < rrt->tree_size; ++i) {
        double norm_square = 0.0;
        for (int j = 0; j < rrt->tree[i].joint_number; ++j) {
            double diff = RAD((double)rrt->tree[i].joint_angles[j]) - RAD((double)q->joint_angles[j]);
            norm_square += diff * diff;
        }
        if (norm_square < min_norm_square) {
            min_norm_square = norm_square;
            nearest_index = i;
        }
    }
    return nearest_index;
}

void RRT_connect(RRT* rrt, int parent_index, Configuration* child_config) {
    if (rrt->tree_size >= rrt->tree_capacity) return;
    rrt->tree[rrt->tree_size] = Configuration_create(child_config->joint_number);
    memcpy(rrt->tree[rrt->tree_size].joint_angles, child_config->joint_angles, sizeof(int) * child_config->joint_number);
    rrt->parent_indices[rrt->tree_size] = parent_index;
    rrt->tree_size++;
}

int* RRT_getPath(RRT* rrt, int start_index, int goal_index, int* path_length) {
    int* path = (int*)malloc(rrt->tree_size * sizeof(int));
    int idx = goal_index;
    int count = 0;
    while (idx != start_index) {
        path[count++] = idx;
        idx = rrt->parent_indices[idx];
    }
    path[count++] = start_index;
    *path_length = count;
    return path;
}

int RRT_isExist(RRT* rrt, Configuration* q) {
    for (int i = 0; i < rrt->tree_size; ++i) {
        if (Configuration_equals(&rrt->tree[i], q)) return 1;
    }
    return 0;
}

int RRT_extend(RRT* rrt, Configuration* q_rand, Arm* arm, Sphere* obstacles, int obstacles_count, int step_size, int* new_index) {
    int nearest_index = RRT_findNearest(rrt, q_rand);
    Configuration* q_near = &rrt->tree[nearest_index];
    Configuration q_new = Configuration_create(q_rand->joint_number);

    int reached = 1;
    for (int s = 1; s <= step_size; ++s) {
        for (int j = 0; j < q_rand->joint_number; ++j) {
            int diff = q_rand->joint_angles[j] - q_near->joint_angles[j];
            q_new.joint_angles[j] = q_near->joint_angles[j] + (int)round((double)diff * s / step_size);
        }
        if (Arm_collisionDetection(arm, &q_new, obstacles, obstacles_count)) {
            reached = 0;
            break;
        }
    }

    if (reached && !RRT_isExist(rrt, &q_new)) {
        RRT_connect(rrt, nearest_index, &q_new);
        if (new_index) *new_index = rrt->tree_size - 1;
        Configuration_destroy(&q_new);
        return 1;
    }
    Configuration_destroy(&q_new);
    return 0;
}