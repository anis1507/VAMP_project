#include <stdio.h>
#include <stdlib.h>

#define FILE_PATH "input/start_end.txt"

int main() {
    int start[5] = {90, 90, 90, 90, 90};  // Initial start values
    int end[5];

    while (1) {
        // Ask the user for the 5 target angles
        printf("Enter the 5 target angles: ");
        if (scanf("%d %d %d %d %d", &end[0], &end[1], &end[2], &end[3], &end[4]) != 5) {
            printf("Invalid input. Please try again.\n");
            while(getchar() != '\n'); // clear input buffer
            continue;
        }

        // Write to file
        FILE *file = fopen(FILE_PATH, "w");
        if (!file) {
            perror("Error opening file");
            return 1;
        }

        // Write start then end
        fprintf(file, "%d %d %d %d %d\n", start[0], start[1], start[2], start[3], start[4]);
        fprintf(file, "%d %d %d %d %d\n", end[0], end[1], end[2], end[3], end[4]);
        fclose(file);

        // Update start for the next iteration
        for (int i = 0; i < 5; i++) {
            start[i] = end[i];
        }

        // Execute the VAMP program
        int ret = system("./vamp");
        if (ret != 0) {
            printf("Error while executing ./vamp\n");
        }

        printf("\n--- New iteration ---\n");
    }

    return 0;
}

