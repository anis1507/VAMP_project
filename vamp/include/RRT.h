#ifndef __RRT_H__
#define __RRT_H__

#include "utils.h"
#include "arm.h"

typedef struct {
    Configuration* tree;
    int tree_size;
    int tree_capacity;
    int* parent_indices;
    int step_size;
} RRT;

void RRT_init(RRT* rrt, int joint_number, int max_nodes);
void RRT_destroy(RRT* rrt);
int RRT_findNearest(RRT* rrt, Configuration* q);
void RRT_connect(RRT* rrt, int parent_index, Configuration* child_config);
int* RRT_getPath(RRT* rrt, int start_index, int goal_index, int* path_length);
int RRT_isExist(RRT* rrt, Configuration* q);
int RRT_extend(RRT* rrt, Configuration* q_rand, Arm* arm, Sphere* obstacles, int obstacles_count, int step_size, int* new_index);

#endif