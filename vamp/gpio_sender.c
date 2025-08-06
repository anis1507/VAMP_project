#include <gpiod.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>

#define CHIPNAME "gpiochip0"
#define LINE_DATA 120
#define BIT_DELAY_US 2000

void send_bit(struct gpiod_line* data_line, int bit) {
    gpiod_line_set_value(data_line, bit);
    usleep(BIT_DELAY_US);
}

void send_byte_serial(struct gpiod_line* data_line, uint8_t byte) {
    send_bit(data_line, 0);  // Start bit

    for (int i = 0; i < 8; ++i) {
        send_bit(data_line, (byte >> i) & 1);  // LSB first
    }

    send_bit(data_line, 1);  // Stop bit
    usleep(BIT_DELAY_US * 2); // Small delay between bytes
}

int main(int argc, char* argv[]) {
    if (argc != 6) {
        fprintf(stderr, "Usage: %s a1 a2 a3 a4 a5\n", argv[0]);
        return 1;
    }

    uint8_t angles[5];
    for (int i = 0; i < 5; ++i)
        angles[i] = (uint8_t)atoi(argv[i + 1]);

    struct gpiod_chip* chip = gpiod_chip_open_by_name(CHIPNAME);
    if (!chip) {
        perror("gpiod_chip_open_by_name");
        return 1;
    }

    struct gpiod_line* data_line = gpiod_chip_get_line(chip, LINE_DATA);
    if (!data_line) {
        perror("gpiod_chip_get_line");
        gpiod_chip_close(chip);
        return 1;
    }

    if (gpiod_line_request_output(data_line, "data", 1) < 0) {
        perror("gpiod_line_request_output");
        gpiod_chip_close(chip);
        return 1;
    }

    send_byte_serial(data_line, 181); // Start marker

    for (int i = 0; i < 5; ++i)
        send_byte_serial(data_line, angles[i]);

    send_byte_serial(data_line, 255); // End marker

    gpiod_chip_close(chip);
    return 0;
}

