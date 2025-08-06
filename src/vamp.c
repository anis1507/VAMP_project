#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <sys/time.h>
#include <time.h>
#include <unistd.h>
#include <stdint.h>
#include "arm.h"
#include "RRT.h"
#include "utils.h"

long get_nanoseconds() {
    struct timeval tv;
    gettimeofday(&tv, NULL);
    return (long)tv.tv_sec * 1000000000L + (long)tv.tv_usec * 1000;
}

Configuration sampleConfiguration(Configuration* goal, Arm* arm) {
    Configuration config = Configuration_create(arm->joint_number);
    for (int i = 0; i < arm->joint_number; ++i) {
        int min = arm->joint_angles_range[i][0];
        int max = arm->joint_angles_range[i][1];
        config.joint_angles[i] = rand() % (max - min + 1) + min;
    }
    return config;
}

int isGoalReached(Configuration* current, Configuration* goal, int threshold, int joint_number) {
    for (int i = 0; i < joint_number; ++i) {
        if (abs(current->joint_angles[i] - goal->joint_angles[i]) > threshold)
            return 0;
    }
    return 1;
}

int computeGoalBias(Configuration* start, Configuration* goal, int joint_number, int obstacle_count) {
    double distance = 0.0;
    for (int i = 0; i < joint_number; ++i)
        distance += abs(start->joint_angles[i] - goal->joint_angles[i]);

    double normalized_distance = distance / (joint_number * 180.0);
    double obstacle_penalty = 1.0 / (1.0 + obstacle_count);

    int probability = (int)(70.0 * obstacle_penalty * (1.0 - normalized_distance));
    if (probability < 5) probability = 5;
    if (probability > 70) probability = 70;
    return probability;
}

int main(int argc, char* argv[]) {
    long t_start = get_nanoseconds();
    double motion_validation_time = 0.0, fk_time = 0.0, cc_time = 0.0;
    int mv_count = 0, fk_count = 0, cc_count = 0;

    char* input_dir = "input";
    char* output_dir = "output";
    char arm_file[256], start_goal_file[256], path_file[256], timing_file[256], obstacle_file[256];

    snprintf(arm_file, sizeof(arm_file), "%s/arm_description.txt", input_dir);
    snprintf(start_goal_file, sizeof(start_goal_file), "%s/start_end.txt", input_dir);
    snprintf(path_file, sizeof(path_file), "%s/path.txt", output_dir);
    snprintf(timing_file, sizeof(timing_file), "%s/execution_time.txt", output_dir);
    snprintf(obstacle_file, sizeof(obstacle_file), "%s/obstacles.txt", input_dir);

    int reach_threshold = 2;
    int max_iterations = 10000;
    int step_size = 5;

    if (argc > 1) snprintf(arm_file, sizeof(arm_file), "%s", argv[1]);
    if (argc > 2) snprintf(start_goal_file, sizeof(start_goal_file), "%s", argv[2]);
    if (argc > 3) snprintf(path_file, sizeof(path_file), "%s", argv[3]);
    if (argc > 4) snprintf(obstacle_file, sizeof(obstacle_file), "%s", argv[4]);
    if (argc > 5) snprintf(timing_file, sizeof(timing_file), "%s", argv[5]);
    if (argc > 6) max_iterations = atoi(argv[6]);
    if (argc > 7) reach_threshold = atoi(argv[7]);

    int seed = (argc > 8) ? atoi(argv[8]) : (int)time(NULL);
    srand(seed);

    FILE* in = fopen(arm_file, "r");
    if (!in) { printf("Error opening %s\n", arm_file); return 1; }

    int joint_number, rod_number;
    fscanf(in, "%d %d", &joint_number, &rod_number);
    Arm arm;
    Arm_init(&arm, joint_number, rod_number);
    fclose(in);

    Arm_printInfo(&arm);

    in = fopen(start_goal_file, "r");
    if (!in) { printf("Error opening %s\n", start_goal_file); return 1; }

    Configuration start = Configuration_create(joint_number);
    for (int i = 0; i < joint_number; ++i) fscanf(in, "%d", &start.joint_angles[i]);
    Configuration goal = Configuration_create(joint_number);
    for (int i = 0; i < joint_number; ++i) fscanf(in, "%d", &goal.joint_angles[i]);
    fclose(in);

    printf(RED BOLD "Start configuration: " NONE);
    for (int i = 0; i < joint_number; ++i) printf("%d ", start.joint_angles[i]);
    printf("\n" RED BOLD "Goal configuration: " NONE);
    for (int i = 0; i < joint_number; ++i) printf("%d ", goal.joint_angles[i]);
    printf("\n");

    in = fopen(obstacle_file, "r");
    if (!in) { printf("Error opening %s\n", obstacle_file); return 1; }

    int obstacle_count;
    fscanf(in, "%d", &obstacle_count);
    Sphere* obstacles = (Sphere*)calloc(obstacle_count, sizeof(Sphere));
    for (int i = 0; i < obstacle_count; ++i) {
        fscanf(in, "%f %f %f %f",
               &obstacles[i].center.x,
               &obstacles[i].center.y,
               &obstacles[i].center.z,
               &obstacles[i].radius);
    }
    fclose(in);

    if (Arm_collisionDetection(&arm, &start, obstacles, obstacle_count)) {
        printf(RED BOLD "Start configuration is in collision!\n" NONE);
        return 1;
    }
    if (Arm_collisionDetection(&arm, &goal, obstacles, obstacle_count)) {
        printf(RED BOLD "Goal configuration is in collision!\n" NONE);
        return 1;
    }

    RRT rrt;
    RRT_init(&rrt, joint_number, 10000);
    rrt.tree[0] = Configuration_create(joint_number);
    memcpy(rrt.tree[0].joint_angles, start.joint_angles, sizeof(int) * joint_number);
    rrt.tree_size = 1;
    rrt.parent_indices[0] = -1;

    Configuration current = start;
    int steps = 0;
    int goal_bias = computeGoalBias(&start, &goal, joint_number, obstacle_count);

    while (!isGoalReached(&current, &goal, reach_threshold, joint_number) && steps < max_iterations) {
        Configuration q_rand;
        int biased = rand() % 100;

        if (biased <= goal_bias) {
            q_rand = Configuration_create(joint_number);
            memcpy(q_rand.joint_angles, goal.joint_angles, sizeof(int) * joint_number);
        } else {
            q_rand = sampleConfiguration(&goal, &arm);
        }

        int nearest_index = RRT_findNearest(&rrt, &q_rand);
        Configuration* q_near = &rrt.tree[nearest_index];
        Configuration q_new = Configuration_create(joint_number);

        int norm = 0;
        for (int i = 0; i < joint_number; ++i) {
            norm += abs(q_rand.joint_angles[i] - q_near->joint_angles[i]);
        }

        if (norm == 0) {
            memcpy(q_new.joint_angles, q_near->joint_angles, sizeof(int) * joint_number);
        } else {
            for (int i = 0; i < joint_number; ++i) {
                double step = ((double)(q_rand.joint_angles[i] - q_near->joint_angles[i]) / norm) * step_size;
                int moved = (int)round(step);
                q_new.joint_angles[i] = q_near->joint_angles[i] + moved;
            }
        }

        long t_mv_start = get_nanoseconds();
        validation_result result = Arm_motionValidation(&arm, q_near, &q_new, obstacles, obstacle_count, step_size);
        long t_mv_end = get_nanoseconds();

        motion_validation_time += (t_mv_end - t_mv_start) / 1e6;
        fk_time += result.FK_time;
        cc_time += result.CC_time;
        mv_count++;
        fk_count++;
        cc_count++;

        if (!result.valid || RRT_isExist(&rrt, &q_new)) {
            Configuration_destroy(&q_rand);
            Configuration_destroy(&q_new);
            continue;
        }

        RRT_connect(&rrt, nearest_index, &q_new);

        if (isGoalReached(&q_new, &goal, reach_threshold, joint_number)) {
            validation_result final = Arm_motionValidation(&arm, &q_new, &goal, obstacles, obstacle_count, step_size);
            motion_validation_time += final.FK_time + final.CC_time;
            fk_time += final.FK_time;
            cc_time += final.CC_time;
            mv_count++;
            fk_count++;
            cc_count++;

            if (final.valid && !RRT_isExist(&rrt, &goal)) {
                RRT_connect(&rrt, rrt.tree_size - 1, &goal);
                current = goal;
                Configuration_destroy(&q_rand);
                steps++;
                break;
            }
        }

        current = q_new;
        Configuration_destroy(&q_rand);
        steps++;
    }

    FILE* out = fopen(path_file, "w");
    if (!out) { printf("Error opening %s\n", path_file); return 1; }

    if (isGoalReached(&current, &goal, reach_threshold, joint_number)) {
        int path_length;
        int* path = RRT_getPath(&rrt, 0, rrt.tree_size - 1, &path_length);
        for (int i = path_length - 1; i >= 0; --i) {
            int idx = path[i];
            uint8_t angles[5];
            fprintf(out, "{");
            for (int j = 0; j < joint_number; ++j) {
                fprintf(out, "%d", rrt.tree[idx].joint_angles[j]);
                angles[j] = (uint8_t)rrt.tree[idx].joint_angles[j];
                if (j < joint_number - 1) fprintf(out, ", ");
            }
            fprintf(out, "}\n");

            char cmd[256];
            snprintf(cmd, sizeof(cmd), "./gpio_sender %d %d %d %d %d",
                     angles[0], angles[1], angles[2], angles[3], angles[4]);
            int ret = system(cmd);
            if (ret != 0) {
                fprintf(stderr, "Error executing gpio_sender\n");
            }
        }
        free(path);
        printf(GREEN BOLD "Path found!\n" NONE);
    } else {
        fprintf(out, "No path found\n");
        printf(RED BOLD "Path not found!\n" NONE);
    }
    fclose(out);

    long t_end = get_nanoseconds();
    double total_time = (t_end - t_start) / 1e6;

    FILE* timing_out = fopen(timing_file, "w");
    if (timing_out) {
        double mv_avg = mv_count ? motion_validation_time / mv_count : 0.0;
        double fk_avg = fk_count ? fk_time / fk_count : 0.0;
        double cc_avg = cc_count ? cc_time / cc_count : 0.0;

        fprintf(timing_out, "Total time: %.4f ms\n", total_time);
        fprintf(timing_out, "Motion validation avg time: %.6f ms\n", mv_avg);
        fprintf(timing_out, "Forward kinematic avg time: %.6f ms\n", fk_avg);
        fprintf(timing_out, "Collision check avg time: %.7f ms\n", cc_avg);
        fprintf(timing_out, "MV ratio: %.5f%%\n", (motion_validation_time * 100.0 / total_time));
        fprintf(timing_out, "FK ratio: %.6f%%\n", (fk_time * 100.0 / total_time));
        fprintf(timing_out, "CC ratio: %.5f%%\n", (cc_time * 100.0 / total_time));
        fclose(timing_out);
    }

    Arm_destroy(&arm);
    RRT_destroy(&rrt);
    Configuration_destroy(&start);
    Configuration_destroy(&goal);
    free(obstacles);

    return 0;
}
