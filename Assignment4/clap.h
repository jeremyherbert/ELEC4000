#ifndef CLAP_H
#define CLAP_H

enum {
  AM_CLAP_MSG = 10
};


enum {
  THRESHOLD = 300
};


typedef nx_struct clap_msg {
	nx_uint16_t id;
	nx_uint16_t time;
} clap_msg;



#endif
