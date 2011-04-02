#ifndef FIREWORKS_H
#define FIREWORKS_H

typedef nx_struct fireworks_update_msg {
	nx_uint16_t id;
	nx_uint16_t node_count;
} fireworks_update_msg;

typedef nx_struct fireworks_node_msg {
	nx_uint16_t real_node_id;
} fireworks_node_msg;



enum {
  AM_FIREWORKS_MSG = 33,
};

enum {
  LED_PERIOD = 1000,
};

#endif
