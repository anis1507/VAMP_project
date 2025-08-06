// gpio_sender.h
#ifndef GPIO_SENDER_H
#define GPIO_SENDER_H

#include <stdint.h>

struct gpiod_line;

void send_5angles_block(struct gpiod_line* data, uint8_t angles[5]);

struct gpiod_line* gpio_init_line();

#endif // GPIO_SENDER_H
