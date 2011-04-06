#ifndef FIREWORKS_H
#define FIREWORKS_H

enum {
  AM_FIREWORKS_MSG = 10
};

enum {
  LED_PERIOD = 1000
};

typedef nx_struct fireworks_update_msg {
	nx_uint16_t id;
	nx_uint16_t node_count;
	nx_uint16_t local_id;
} fireworks_update_msg;

typedef nx_struct fireworks_node_msg {
	nx_uint16_t real_node_id;
} fireworks_node_msg;

uint8_t pattern;

#define PATTERN_0_ADDR 105
#define PATTERN_1_ADDR 106

#endif
