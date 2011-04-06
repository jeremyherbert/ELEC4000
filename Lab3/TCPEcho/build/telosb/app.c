#define nx_struct struct
#define nx_union union
#define dbg(mode, format, ...) ((void)0)
#define dbg_clear(mode, format, ...) ((void)0)
#define dbg_active(mode) 0
# 151 "/usr/lib/gcc-lib/msp430/3.2.3/include/stddef.h" 3
typedef int ptrdiff_t;
#line 213
typedef unsigned int size_t;
#line 325
typedef int wchar_t;
# 8 "/usr/lib/ncc/deputy_nodeputy.h"
struct __nesc_attr_nonnull {
}  ;
#line 9
struct __nesc_attr_bnd {
#line 9
  void *lo, *hi;
}  ;
#line 10
struct __nesc_attr_bnd_nok {
#line 10
  void *lo, *hi;
}  ;
#line 11
struct __nesc_attr_count {
#line 11
  int n;
}  ;
#line 12
struct __nesc_attr_count_nok {
#line 12
  int n;
}  ;
#line 13
struct __nesc_attr_one {
}  ;
#line 14
struct __nesc_attr_one_nok {
}  ;
#line 15
struct __nesc_attr_dmemset {
#line 15
  int a1, a2, a3;
}  ;
#line 16
struct __nesc_attr_dmemcpy {
#line 16
  int a1, a2, a3;
}  ;
#line 17
struct __nesc_attr_nts {
}  ;
# 38 "/usr/msp430/include/sys/inttypes.h" 3
typedef signed char int8_t;
typedef unsigned char uint8_t;

typedef int int16_t;
typedef unsigned int uint16_t;

typedef long int32_t;
typedef unsigned long uint32_t;

typedef long long int64_t;
typedef unsigned long long uint64_t;




typedef int16_t intptr_t;
typedef uint16_t uintptr_t;
# 235 "/usr/lib/ncc/nesc_nx.h"
static __inline uint8_t __nesc_ntoh_uint8(const void * source)  ;




static __inline uint8_t __nesc_hton_uint8(void * target, uint8_t value)  ;





static __inline uint8_t __nesc_ntoh_leuint8(const void * source)  ;




static __inline uint8_t __nesc_hton_leuint8(void * target, uint8_t value)  ;





static __inline int8_t __nesc_ntoh_int8(const void * source)  ;
#line 257
static __inline int8_t __nesc_hton_int8(void * target, int8_t value)  ;






static __inline uint16_t __nesc_ntoh_uint16(const void * source)  ;




static __inline uint16_t __nesc_hton_uint16(void * target, uint16_t value)  ;






static __inline uint16_t __nesc_ntoh_leuint16(const void * source)  ;




static __inline uint16_t __nesc_hton_leuint16(void * target, uint16_t value)  ;
#line 294
static __inline uint32_t __nesc_ntoh_uint32(const void * source)  ;






static __inline uint32_t __nesc_hton_uint32(void * target, uint32_t value)  ;
#line 385
typedef struct { unsigned char data[1]; } __attribute__((packed)) nx_int8_t;typedef int8_t __nesc_nxbase_nx_int8_t  ;
typedef struct { unsigned char data[2]; } __attribute__((packed)) nx_int16_t;typedef int16_t __nesc_nxbase_nx_int16_t  ;
typedef struct { unsigned char data[4]; } __attribute__((packed)) nx_int32_t;typedef int32_t __nesc_nxbase_nx_int32_t  ;
typedef struct { unsigned char data[8]; } __attribute__((packed)) nx_int64_t;typedef int64_t __nesc_nxbase_nx_int64_t  ;
typedef struct { unsigned char data[1]; } __attribute__((packed)) nx_uint8_t;typedef uint8_t __nesc_nxbase_nx_uint8_t  ;
typedef struct { unsigned char data[2]; } __attribute__((packed)) nx_uint16_t;typedef uint16_t __nesc_nxbase_nx_uint16_t  ;
typedef struct { unsigned char data[4]; } __attribute__((packed)) nx_uint32_t;typedef uint32_t __nesc_nxbase_nx_uint32_t  ;
typedef struct { unsigned char data[8]; } __attribute__((packed)) nx_uint64_t;typedef uint64_t __nesc_nxbase_nx_uint64_t  ;


typedef struct { unsigned char data[1]; } __attribute__((packed)) nxle_int8_t;typedef int8_t __nesc_nxbase_nxle_int8_t  ;
typedef struct { unsigned char data[2]; } __attribute__((packed)) nxle_int16_t;typedef int16_t __nesc_nxbase_nxle_int16_t  ;
typedef struct { unsigned char data[4]; } __attribute__((packed)) nxle_int32_t;typedef int32_t __nesc_nxbase_nxle_int32_t  ;
typedef struct { unsigned char data[8]; } __attribute__((packed)) nxle_int64_t;typedef int64_t __nesc_nxbase_nxle_int64_t  ;
typedef struct { unsigned char data[1]; } __attribute__((packed)) nxle_uint8_t;typedef uint8_t __nesc_nxbase_nxle_uint8_t  ;
typedef struct { unsigned char data[2]; } __attribute__((packed)) nxle_uint16_t;typedef uint16_t __nesc_nxbase_nxle_uint16_t  ;
typedef struct { unsigned char data[4]; } __attribute__((packed)) nxle_uint32_t;typedef uint32_t __nesc_nxbase_nxle_uint32_t  ;
typedef struct { unsigned char data[8]; } __attribute__((packed)) nxle_uint64_t;typedef uint64_t __nesc_nxbase_nxle_uint64_t  ;
# 41 "/usr/msp430/include/sys/types.h" 3
typedef unsigned char u_char;
typedef unsigned short u_short;
typedef unsigned int u_int;
typedef unsigned long u_long;
typedef unsigned short ushort;
typedef unsigned int uint;

typedef uint8_t u_int8_t;
typedef uint16_t u_int16_t;
typedef uint32_t u_int32_t;
typedef uint64_t u_int64_t;

typedef u_int64_t u_quad_t;
typedef int64_t quad_t;
typedef quad_t *qaddr_t;

typedef char *caddr_t;
typedef const char *c_caddr_t;
typedef volatile char *v_caddr_t;
typedef u_int32_t fixpt_t;
typedef u_int32_t gid_t;
typedef u_int32_t in_addr_t;
typedef u_int16_t in_port_t;
typedef u_int32_t ino_t;
typedef long key_t;
typedef u_int16_t mode_t;
typedef u_int16_t nlink_t;
typedef quad_t rlim_t;
typedef int32_t segsz_t;
typedef int32_t swblk_t;
typedef int32_t ufs_daddr_t;
typedef int32_t ufs_time_t;
typedef u_int32_t uid_t;
# 39 "/usr/msp430/include/string.h" 3
extern int memcmp(const void *arg_0x4029ba18, const void *arg_0x4029bbb0, size_t arg_0x4029bd48);


extern void *memset(void *arg_0x4029d220, int arg_0x4029d378, size_t arg_0x4029d510);



extern char *strcpy(char *arg_0x4029fcd8, const char *arg_0x4029fe70);

extern size_t strlen(const char *arg_0x402a3a30);



extern char *strncpy(char *arg_0x402a63e0, const char *arg_0x402a6578, size_t arg_0x402a6710);










extern void *memset(void *arg_0x402ab118, int arg_0x402ab270, size_t arg_0x402ab408);
# 59 "/usr/msp430/include/stdlib.h" 3
#line 56
typedef struct __nesc_unnamed4242 {
  int quot;
  int rem;
} div_t;







#line 64
typedef struct __nesc_unnamed4243 {
  long quot;
  long rem;
} ldiv_t;
# 122 "/usr/msp430/include/sys/config.h" 3
typedef long int __int32_t;
typedef unsigned long int __uint32_t;
# 12 "/usr/msp430/include/sys/_types.h" 3
typedef long _off_t;
typedef long _ssize_t;
# 28 "/usr/msp430/include/sys/reent.h" 3
typedef __uint32_t __ULong;


struct _glue {

  struct _glue *_next;
  int _niobs;
  struct __sFILE *_iobs;
};

struct _Bigint {

  struct _Bigint *_next;
  int _k, _maxwds, _sign, _wds;
  __ULong _x[1];
};


struct __tm {

  int __tm_sec;
  int __tm_min;
  int __tm_hour;
  int __tm_mday;
  int __tm_mon;
  int __tm_year;
  int __tm_wday;
  int __tm_yday;
  int __tm_isdst;
};







struct _atexit {
  struct _atexit *_next;
  int _ind;
  void (*_fns[32])(void );
};








struct __sbuf {
  unsigned char *_base;
  int _size;
};






typedef long _fpos_t;
#line 116
struct __sFILE {
  unsigned char *_p;
  int _r;
  int _w;
  short _flags;
  short _file;
  struct __sbuf _bf;
  int _lbfsize;


  void *_cookie;

  int (*_read)(void *_cookie, char *_buf, int _n);
  int (*_write)(void *_cookie, const char *_buf, int _n);

  _fpos_t (*_seek)(void *_cookie, _fpos_t _offset, int _whence);
  int (*_close)(void *_cookie);


  struct __sbuf _ub;
  unsigned char *_up;
  int _ur;


  unsigned char _ubuf[3];
  unsigned char _nbuf[1];


  struct __sbuf _lb;


  int _blksize;
  int _offset;

  struct _reent *_data;
};
#line 174
struct _rand48 {
  unsigned short _seed[3];
  unsigned short _mult[3];
  unsigned short _add;
};









struct _reent {


  int _errno;




  struct __sFILE *_stdin, *_stdout, *_stderr;

  int _inc;
  char _emergency[25];

  int _current_category;
  const char *_current_locale;

  int __sdidinit;

  void (*__cleanup)(struct _reent *arg_0x402c8510);


  struct _Bigint *_result;
  int _result_k;
  struct _Bigint *_p5s;
  struct _Bigint **_freelist;


  int _cvtlen;
  char *_cvtbuf;

  union __nesc_unnamed4244 {

    struct __nesc_unnamed4245 {

      unsigned int _unused_rand;
      char *_strtok_last;
      char _asctime_buf[26];
      struct __tm _localtime_buf;
      int _gamma_signgam;
      __extension__ unsigned long long _rand_next;
      struct _rand48 _r48;
    } _reent;



    struct __nesc_unnamed4246 {


      unsigned char *_nextf[30];
      unsigned int _nmalloc[30];
    } _unused;
  } _new;


  struct _atexit *_atexit;
  struct _atexit _atexit0;


  void (**_sig_func)(int arg_0x402ccb88);




  struct _glue __sglue;
  struct __sFILE __sf[3];
};
#line 273
struct _reent;
# 18 "/usr/msp430/include/math.h" 3
union __dmath {

  __uint32_t i[2];
  double d;
};




union __dmath;
#line 208
struct exception {


  int type;
  char *name;
  double arg1;
  double arg2;
  double retval;
  int err;
};
#line 261
enum __fdlibm_version {

  __fdlibm_ieee = -1, 
  __fdlibm_svid, 
  __fdlibm_xopen, 
  __fdlibm_posix
};




enum __fdlibm_version;
# 23 "/home/tinyos/local/src/tinyos-2.x/tos/system/tos.h"
typedef uint8_t bool;
enum __nesc_unnamed4247 {
#line 24
  FALSE = 0, TRUE = 1
};
typedef nx_int8_t nx_bool;
uint16_t TOS_NODE_ID = 1;






struct __nesc_attr_atmostonce {
};
#line 35
struct __nesc_attr_atleastonce {
};
#line 36
struct __nesc_attr_exactlyonce {
};
# 40 "/home/tinyos/local/src/tinyos-2.x/tos/types/TinyError.h"
enum __nesc_unnamed4248 {
  SUCCESS = 0, 
  FAIL = 1, 
  ESIZE = 2, 
  ECANCEL = 3, 
  EOFF = 4, 
  EBUSY = 5, 
  EINVAL = 6, 
  ERETRY = 7, 
  ERESERVE = 8, 
  EALREADY = 9, 
  ENOMEM = 10, 
  ENOACK = 11, 
  ELAST = 11
};

typedef uint8_t error_t  ;

static inline error_t ecombine(error_t r1, error_t r2)  ;
# 39 "/usr/msp430/include/msp430/iostructures.h" 3
#line 27
typedef union port {
  volatile unsigned char reg_p;
  volatile struct __nesc_unnamed4249 {
    unsigned char __p0 : 1, 
    __p1 : 1, 
    __p2 : 1, 
    __p3 : 1, 
    __p4 : 1, 
    __p5 : 1, 
    __p6 : 1, 
    __p7 : 1;
  } __pin;
} __attribute((packed))  ioregister_t;
#line 108
struct port_full_t {
  ioregister_t in;
  ioregister_t out;
  ioregister_t dir;
  ioregister_t ifg;
  ioregister_t ies;
  ioregister_t ie;
  ioregister_t sel;
};









struct port_simple_t {
  ioregister_t in;
  ioregister_t out;
  ioregister_t dir;
  ioregister_t sel;
};




struct port_full_t;



struct port_full_t;



struct port_simple_t;



struct port_simple_t;



struct port_simple_t;



struct port_simple_t;
# 116 "/usr/msp430/include/msp430/gpio.h" 3
volatile unsigned char P1OUT __asm ("0x0021");

volatile unsigned char P1DIR __asm ("0x0022");

volatile unsigned char P1IFG __asm ("0x0023");

volatile unsigned char P1IES __asm ("0x0024");

volatile unsigned char P1IE __asm ("0x0025");

volatile unsigned char P1SEL __asm ("0x0026");










volatile unsigned char P2OUT __asm ("0x0029");

volatile unsigned char P2DIR __asm ("0x002A");

volatile unsigned char P2IFG __asm ("0x002B");



volatile unsigned char P2IE __asm ("0x002D");

volatile unsigned char P2SEL __asm ("0x002E");










volatile unsigned char P3OUT __asm ("0x0019");

volatile unsigned char P3DIR __asm ("0x001A");

volatile unsigned char P3SEL __asm ("0x001B");










volatile unsigned char P4OUT __asm ("0x001D");

volatile unsigned char P4DIR __asm ("0x001E");

volatile unsigned char P4SEL __asm ("0x001F");










volatile unsigned char P5OUT __asm ("0x0031");

volatile unsigned char P5DIR __asm ("0x0032");

volatile unsigned char P5SEL __asm ("0x0033");










volatile unsigned char P6OUT __asm ("0x0035");

volatile unsigned char P6DIR __asm ("0x0036");

volatile unsigned char P6SEL __asm ("0x0037");
# 92 "/usr/msp430/include/msp430/usart.h" 3
volatile unsigned char U0CTL __asm ("0x0070");

volatile unsigned char U0TCTL __asm ("0x0071");



volatile unsigned char U0MCTL __asm ("0x0073");

volatile unsigned char U0BR0 __asm ("0x0074");

volatile unsigned char U0BR1 __asm ("0x0075");

volatile unsigned char U0RXBUF __asm ("0x0076");
#line 277
volatile unsigned char U1TCTL __asm ("0x0079");
# 27 "/usr/msp430/include/msp430/timera.h" 3
volatile unsigned int TA0CTL __asm ("0x0160");

volatile unsigned int TA0R __asm ("0x0170");


volatile unsigned int TA0CCTL0 __asm ("0x0162");

volatile unsigned int TA0CCTL1 __asm ("0x0164");
#line 70
volatile unsigned int TA0CCTL2 __asm ("0x0166");
#line 127
#line 118
typedef struct __nesc_unnamed4250 {
  volatile unsigned 
  taifg : 1, 
  taie : 1, 
  taclr : 1, 
  dummy : 1, 
  tamc : 2, 
  taid : 2, 
  tassel : 2;
} __attribute((packed))  tactl_t;
#line 143
#line 129
typedef struct __nesc_unnamed4251 {
  volatile unsigned 
  ccifg : 1, 
  cov : 1, 
  out : 1, 
  cci : 1, 
  ccie : 1, 
  outmod : 3, 
  cap : 1, 
  dummy : 1, 
  scci : 1, 
  scs : 1, 
  ccis : 2, 
  cm : 2;
} __attribute((packed))  tacctl_t;


struct timera_t {
  tactl_t ctl;
  tacctl_t cctl0;
  tacctl_t cctl1;
  tacctl_t cctl2;
  volatile unsigned dummy[4];
  volatile unsigned tar;
  volatile unsigned taccr0;
  volatile unsigned taccr1;
  volatile unsigned taccr2;
};



struct timera_t;
# 26 "/usr/msp430/include/msp430/timerb.h" 3
volatile unsigned int TBR __asm ("0x0190");


volatile unsigned int TBCCTL0 __asm ("0x0182");





volatile unsigned int TBCCR0 __asm ("0x0192");
#line 76
#line 64
typedef struct __nesc_unnamed4252 {
  volatile unsigned 
  tbifg : 1, 
  tbie : 1, 
  tbclr : 1, 
  dummy1 : 1, 
  tbmc : 2, 
  tbid : 2, 
  tbssel : 2, 
  dummy2 : 1, 
  tbcntl : 2, 
  tbclgrp : 2;
} __attribute((packed))  tbctl_t;
#line 91
#line 78
typedef struct __nesc_unnamed4253 {
  volatile unsigned 
  ccifg : 1, 
  cov : 1, 
  out : 1, 
  cci : 1, 
  ccie : 1, 
  outmod : 3, 
  cap : 1, 
  clld : 2, 
  scs : 1, 
  ccis : 2, 
  cm : 2;
} __attribute((packed))  tbcctl_t;


struct timerb_t {
  tbctl_t ctl;
  tbcctl_t cctl0;
  tbcctl_t cctl1;
  tbcctl_t cctl2;

  tbcctl_t cctl3;
  tbcctl_t cctl4;
  tbcctl_t cctl5;
  tbcctl_t cctl6;



  volatile unsigned tbr;
  volatile unsigned tbccr0;
  volatile unsigned tbccr1;
  volatile unsigned tbccr2;

  volatile unsigned tbccr3;
  volatile unsigned tbccr4;
  volatile unsigned tbccr5;
  volatile unsigned tbccr6;
};





struct timerb_t;
# 20 "/usr/msp430/include/msp430/basic_clock.h" 3
volatile unsigned char DCOCTL __asm ("0x0056");

volatile unsigned char BCSCTL1 __asm ("0x0057");

volatile unsigned char BCSCTL2 __asm ("0x0058");
# 18 "/usr/msp430/include/msp430/adc12.h" 3
volatile unsigned int ADC12CTL0 __asm ("0x01A0");

volatile unsigned int ADC12CTL1 __asm ("0x01A2");
#line 42
#line 30
typedef struct __nesc_unnamed4254 {
  volatile unsigned 
  adc12sc : 1, 
  enc : 1, 
  adc12tovie : 1, 
  adc12ovie : 1, 
  adc12on : 1, 
  refon : 1, 
  r2_5v : 1, 
  msc : 1, 
  sht0 : 4, 
  sht1 : 4;
} __attribute((packed))  adc12ctl0_t;
#line 54
#line 44
typedef struct __nesc_unnamed4255 {
  volatile unsigned 
  adc12busy : 1, 
  conseq : 2, 
  adc12ssel : 2, 
  adc12div : 3, 
  issh : 1, 
  shp : 1, 
  shs : 2, 
  cstartadd : 4;
} __attribute((packed))  adc12ctl1_t;
#line 74
#line 56
typedef struct __nesc_unnamed4256 {
  volatile unsigned 
  bit0 : 1, 
  bit1 : 1, 
  bit2 : 1, 
  bit3 : 1, 
  bit4 : 1, 
  bit5 : 1, 
  bit6 : 1, 
  bit7 : 1, 
  bit8 : 1, 
  bit9 : 1, 
  bit10 : 1, 
  bit11 : 1, 
  bit12 : 1, 
  bit13 : 1, 
  bit14 : 1, 
  bit15 : 1;
} __attribute((packed))  adc12xflg_t;


struct adc12_t {
  adc12ctl0_t ctl0;
  adc12ctl1_t ctl1;
  adc12xflg_t ifg;
  adc12xflg_t ie;
  adc12xflg_t iv;
};




struct adc12_t;
# 83 "/usr/msp430/include/msp430x16x.h" 3
volatile unsigned char ME1 __asm ("0x0004");





volatile unsigned char ME2 __asm ("0x0005");
# 158 "/home/tinyos/local/src/tinyos-2.x/tos/chips/msp430/msp430hardware.h"
static volatile uint8_t U0CTLnr __asm ("0x0070");
static volatile uint8_t I2CTCTLnr __asm ("0x0071");
static volatile uint8_t I2CDCTLnr __asm ("0x0072");
#line 193
typedef uint8_t mcu_power_t  ;
static inline mcu_power_t mcombine(mcu_power_t m1, mcu_power_t m2)  ;


enum __nesc_unnamed4257 {
  MSP430_POWER_ACTIVE = 0, 
  MSP430_POWER_LPM0 = 1, 
  MSP430_POWER_LPM1 = 2, 
  MSP430_POWER_LPM2 = 3, 
  MSP430_POWER_LPM3 = 4, 
  MSP430_POWER_LPM4 = 5
};

static inline void __nesc_disable_interrupt(void )  ;





static inline void __nesc_enable_interrupt(void )  ;




typedef bool __nesc_atomic_t;
__nesc_atomic_t __nesc_atomic_start(void );
void __nesc_atomic_end(__nesc_atomic_t reenable_interrupts);






__nesc_atomic_t __nesc_atomic_start(void )   ;







void __nesc_atomic_end(__nesc_atomic_t reenable_interrupts)   ;
#line 248
typedef struct { unsigned char data[4]; } __attribute__((packed)) nx_float;typedef float __nesc_nxbase_nx_float  ;
# 8 "/home/tinyos/local/src/tinyos-2.x/tos/platforms/telosb/hardware.h"
enum __nesc_unnamed4258 {
  TOS_SLEEP_NONE = MSP430_POWER_ACTIVE
};
#line 36
static inline void TOSH_SET_SIMO0_PIN()  ;
#line 36
static inline void TOSH_CLR_SIMO0_PIN()  ;
#line 36
static inline void TOSH_MAKE_SIMO0_OUTPUT()  ;
static inline void TOSH_SET_UCLK0_PIN()  ;
#line 37
static inline void TOSH_CLR_UCLK0_PIN()  ;
#line 37
static inline void TOSH_MAKE_UCLK0_OUTPUT()  ;
#line 79
enum __nesc_unnamed4259 {

  TOSH_HUMIDITY_ADDR = 5, 
  TOSH_HUMIDTEMP_ADDR = 3, 
  TOSH_HUMIDITY_RESET = 0x1E
};



static inline void TOSH_SET_FLASH_CS_PIN()  ;
#line 88
static inline void TOSH_CLR_FLASH_CS_PIN()  ;
#line 88
static inline void TOSH_MAKE_FLASH_CS_OUTPUT()  ;
static inline void TOSH_SET_FLASH_HOLD_PIN()  ;
#line 89
static inline void TOSH_MAKE_FLASH_HOLD_OUTPUT()  ;
# 52 "/usr/msp430/include/stdint.h" 3
typedef signed char int_least8_t;
typedef int int_least16_t;
typedef long int int_least32_t;
__extension__ 
#line 55
typedef long long int int_least64_t;


typedef unsigned char uint_least8_t;
typedef unsigned int uint_least16_t;
typedef unsigned long int uint_least32_t;
__extension__ 
#line 61
typedef unsigned long long int uint_least64_t;





typedef signed char int_fast8_t;
typedef int int_fast16_t;
typedef long int int_fast32_t;
__extension__ 
#line 70
typedef long long int int_fast64_t;


typedef unsigned char uint_fast8_t;
typedef unsigned int uint_fast16_t;
typedef unsigned long int uint_fast32_t;
__extension__ 
#line 76
typedef unsigned long long int uint_fast64_t;
#line 88
__extension__ 
#line 88
typedef long long int intmax_t;
__extension__ 
#line 89
typedef unsigned long long int uintmax_t;
# 42 "/home/tinyos/local/src/tinyos-2.x/support/sdk/c/blip/lib6lowpan/blip-platform.h"
enum __nesc_unnamed4260 {
  IP_NUMBER_FRAGMENTS = 14
};
# 40 "/home/tinyos/local/src/tinyos-2.x/support/sdk/c/blip/lib6lowpan/6lowpan.h"
typedef uint8_t ip6_addr_t[16];
typedef uint16_t cmpr_ip6_addr_t;
# 33 "/home/tinyos/local/src/tinyos-2.x/tos/types/Ieee154.h"
typedef uint16_t ieee154_panid_t;
typedef uint16_t ieee154_saddr_t;

enum __nesc_unnamed4261 {
  IEEE154_BROADCAST_ADDR = 0xffff
};
# 57 "/home/tinyos/local/src/tinyos-2.x/support/sdk/c/blip/lib6lowpan/6lowpan.h"
extern uint8_t globalPrefix;

extern uint8_t linklocal_prefix[8];

uint8_t cmpPfx(ip6_addr_t a, uint8_t *pfx);

void ip_memclr(uint8_t *buf, uint16_t len);
void *ip_memcpy(void *dst0, const void *src0, uint16_t len);
#line 81
#line 74
typedef struct packed_lowmsg {
  uint8_t headers;
  uint8_t len;

  ieee154_saddr_t src;
  ieee154_saddr_t dst;
  uint8_t *data;
} packed_lowmsg_t;





enum __nesc_unnamed4262 {
  LOWMSG_MESH_HDR = 1 << 0, 
  LOWMSG_BCAST_HDR = 1 << 1, 
  LOWMSG_FRAG1_HDR = 1 << 2, 
  LOWMSG_FRAGN_HDR = 1 << 3, 
  LOWMSG_NALP = 1 << 4, 
  LOWMSG_IPNH_HDR = 1 << 5
};




enum __nesc_unnamed4263 {
  LOWMSG_MESH_LEN = 5, 
  LOWMSG_BCAST_LEN = 2, 
  LOWMSG_FRAG1_LEN = 4, 
  LOWMSG_FRAGN_LEN = 5
};

enum __nesc_unnamed4264 {
  LOWPAN_LINK_MTU = 110, 
  INET_MTU = 1280, 
  LIB6LOWPAN_MAX_LEN = LOWPAN_LINK_MTU
};




enum __nesc_unnamed4265 {
  LOWPAN_NALP_PATTERN = 0x0, 
  LOWPAN_MESH_PATTERN = 0x2, 
  LOWPAN_FRAG1_PATTERN = 0x18, 
  LOWPAN_FRAGN_PATTERN = 0x1c, 
  LOWPAN_BCAST_PATTERN = 0x50, 
  LOWPAN_HC1_PATTERN = 0x42, 
  LOWPAN_HC_LOCAL_PATTERN = 0x3, 
  LOWPAN_HC_CRP_PATTERN = 0x4
};

enum __nesc_unnamed4266 {
  LOWPAN_MESH_V_MASK = 0x20, 
  LOWPAN_MESH_F_MASK = 0x10, 
  LOWPAN_MESH_HOPS_MASK = 0x0f
};




enum __nesc_unnamed4267 {
  LOWPAN_IPHC_VTF_MASK = 0x80, 
  LOWPAN_IPHC_VTF_INLINE = 0, 
  LOWPAN_IPHC_NH_MASK = 0x40, 
  LOWPAN_IPHC_NH_INLINE = 0, 
  LOWPAN_IPHC_HLIM_MASK = 0x20, 
  LOWPAN_IPHC_HLIM_INLINE = 0, 

  LOWPAN_IPHC_SC_OFFSET = 3, 
  LOWPAN_IPHC_DST_OFFSET = 1, 
  LOWPAN_IPHC_ADDRFLAGS_MASK = 0x3, 

  LOWPAN_IPHC_ADDR_128 = 0x0, 
  LOWPAN_IPHC_ADDR_64 = 0x1, 
  LOWPAN_IPHC_ADDR_16 = 0x2, 
  LOWPAN_IPHC_ADDR_0 = 0x3, 

  LOWPAN_IPHC_SHORT_MASK = 0x80, 
  LOWPAN_IPHC_SHORT_LONG_MASK = 0xe0, 

  LOWPAN_IPHC_HC1_MCAST = 0x80, 
  LOWPAN_IPHC_HC_MCAST = 0xa0, 

  LOWPAN_HC_MCAST_SCOPE_MASK = 0x1e, 
  LOWPAN_HC_MCAST_SCOPE_OFFSET = 1, 

  LOWPAN_UDP_PORT_BASE_MASK = 0xfff0, 
  LOWPAN_UDP_PORT_BASE = 0xf0b0, 
  LOWPAN_UDP_DISPATCH = 0x80, 

  LOWPAN_UDP_S_MASK = 0x40, 
  LOWPAN_UDP_D_MASK = 0x20, 
  LOWPAN_UDP_C_MASK = 0x10
};





struct topology_entry {
  uint8_t etx;
  uint8_t conf;
  ieee154_saddr_t hwaddr;
};
struct topology_header {
  uint16_t seqno;
  struct topology_entry topo[0];
};
struct topology_header_package {
  uint16_t reporter;
  uint16_t len;
  uint16_t seqno;
  struct topology_entry topo[0];
};
# 28 "/home/tinyos/local/src/tinyos-2.x/tos/chips/msp430/timer/Msp430Timer.h"
enum __nesc_unnamed4268 {
  MSP430TIMER_CM_NONE = 0, 
  MSP430TIMER_CM_RISING = 1, 
  MSP430TIMER_CM_FALLING = 2, 
  MSP430TIMER_CM_BOTH = 3, 

  MSP430TIMER_STOP_MODE = 0, 
  MSP430TIMER_UP_MODE = 1, 
  MSP430TIMER_CONTINUOUS_MODE = 2, 
  MSP430TIMER_UPDOWN_MODE = 3, 

  MSP430TIMER_TACLK = 0, 
  MSP430TIMER_TBCLK = 0, 
  MSP430TIMER_ACLK = 1, 
  MSP430TIMER_SMCLK = 2, 
  MSP430TIMER_INCLK = 3, 

  MSP430TIMER_CLOCKDIV_1 = 0, 
  MSP430TIMER_CLOCKDIV_2 = 1, 
  MSP430TIMER_CLOCKDIV_4 = 2, 
  MSP430TIMER_CLOCKDIV_8 = 3
};
#line 64
#line 51
typedef struct __nesc_unnamed4269 {

  int ccifg : 1;
  int cov : 1;
  int out : 1;
  int cci : 1;
  int ccie : 1;
  int outmod : 3;
  int cap : 1;
  int clld : 2;
  int scs : 1;
  int ccis : 2;
  int cm : 2;
} msp430_compare_control_t;
#line 76
#line 66
typedef struct __nesc_unnamed4270 {

  int taifg : 1;
  int taie : 1;
  int taclr : 1;
  int _unused0 : 1;
  int mc : 2;
  int id : 2;
  int tassel : 2;
  int _unused1 : 6;
} msp430_timer_a_control_t;
#line 91
#line 78
typedef struct __nesc_unnamed4271 {

  int tbifg : 1;
  int tbie : 1;
  int tbclr : 1;
  int _unused0 : 1;
  int mc : 2;
  int id : 2;
  int tbssel : 2;
  int _unused1 : 1;
  int cntl : 2;
  int tbclgrp : 2;
  int _unused2 : 1;
} msp430_timer_b_control_t;
# 32 "/home/tinyos/local/src/tinyos-2.x/tos/types/Leds.h"
enum __nesc_unnamed4272 {
  LEDS_LED0 = 1 << 0, 
  LEDS_LED1 = 1 << 1, 
  LEDS_LED2 = 1 << 2, 
  LEDS_LED3 = 1 << 3, 
  LEDS_LED4 = 1 << 4, 
  LEDS_LED5 = 1 << 5, 
  LEDS_LED6 = 1 << 6, 
  LEDS_LED7 = 1 << 7
};
# 39 "/home/tinyos/local/src/tinyos-2.x/tos/chips/cc2420/CC2420.h"
typedef uint8_t cc2420_status_t;
#line 93
#line 87
typedef nx_struct security_header_t {
  unsigned char __nesc_filler0[1];


  nx_uint32_t frameCounter;
  nx_uint8_t keyID[1];
} __attribute__((packed)) security_header_t;
#line 113
#line 95
typedef nx_struct cc2420_header_t {
  nxle_uint8_t length;
  nxle_uint16_t fcf;
  nxle_uint8_t dsn;
  nxle_uint16_t destpan;
  nxle_uint16_t dest;
  nxle_uint16_t src;







  nxle_uint8_t network;


  nxle_uint8_t type;
} __attribute__((packed)) cc2420_header_t;





#line 118
typedef nx_struct cc2420_footer_t {
} __attribute__((packed)) cc2420_footer_t;
#line 143
#line 128
typedef nx_struct cc2420_metadata_t {
  nx_uint8_t rssi;
  nx_uint8_t lqi;
  nx_uint8_t tx_power;
  nx_bool crc;
  nx_bool ack;
  nx_bool timesync;
  nx_uint32_t timestamp;
  nx_uint16_t rxInterval;



  nx_uint16_t maxRetries;
  nx_uint16_t retryDelay;
} __attribute__((packed)) 
cc2420_metadata_t;





#line 146
typedef nx_struct cc2420_packet_t {
  cc2420_header_t packet;
  nx_uint8_t data[];
} __attribute__((packed)) cc2420_packet_t;
#line 179
enum __nesc_unnamed4273 {

  MAC_HEADER_SIZE = sizeof(cc2420_header_t ) - 1, 

  MAC_FOOTER_SIZE = sizeof(uint16_t ), 

  MAC_PACKET_SIZE = MAC_HEADER_SIZE + 102 + MAC_FOOTER_SIZE, 

  CC2420_SIZE = MAC_HEADER_SIZE + MAC_FOOTER_SIZE, 

  AM_OVERHEAD = 2
};

enum cc2420_enums {
  CC2420_TIME_ACK_TURNAROUND = 7, 
  CC2420_TIME_VREN = 20, 
  CC2420_TIME_SYMBOL = 2, 
  CC2420_BACKOFF_PERIOD = 20 / CC2420_TIME_SYMBOL, 
  CC2420_MIN_BACKOFF = 20 / CC2420_TIME_SYMBOL, 
  CC2420_ACK_WAIT_DELAY = 256
};

enum cc2420_status_enums {
  CC2420_STATUS_RSSI_VALID = 1 << 1, 
  CC2420_STATUS_LOCK = 1 << 2, 
  CC2420_STATUS_TX_ACTIVE = 1 << 3, 
  CC2420_STATUS_ENC_BUSY = 1 << 4, 
  CC2420_STATUS_TX_UNDERFLOW = 1 << 5, 
  CC2420_STATUS_XOSC16M_STABLE = 1 << 6
};

enum cc2420_config_reg_enums {
  CC2420_SNOP = 0x00, 
  CC2420_SXOSCON = 0x01, 
  CC2420_STXCAL = 0x02, 
  CC2420_SRXON = 0x03, 
  CC2420_STXON = 0x04, 
  CC2420_STXONCCA = 0x05, 
  CC2420_SRFOFF = 0x06, 
  CC2420_SXOSCOFF = 0x07, 
  CC2420_SFLUSHRX = 0x08, 
  CC2420_SFLUSHTX = 0x09, 
  CC2420_SACK = 0x0a, 
  CC2420_SACKPEND = 0x0b, 
  CC2420_SRXDEC = 0x0c, 
  CC2420_STXENC = 0x0d, 
  CC2420_SAES = 0x0e, 
  CC2420_MAIN = 0x10, 
  CC2420_MDMCTRL0 = 0x11, 
  CC2420_MDMCTRL1 = 0x12, 
  CC2420_RSSI = 0x13, 
  CC2420_SYNCWORD = 0x14, 
  CC2420_TXCTRL = 0x15, 
  CC2420_RXCTRL0 = 0x16, 
  CC2420_RXCTRL1 = 0x17, 
  CC2420_FSCTRL = 0x18, 
  CC2420_SECCTRL0 = 0x19, 
  CC2420_SECCTRL1 = 0x1a, 
  CC2420_BATTMON = 0x1b, 
  CC2420_IOCFG0 = 0x1c, 
  CC2420_IOCFG1 = 0x1d, 
  CC2420_MANFIDL = 0x1e, 
  CC2420_MANFIDH = 0x1f, 
  CC2420_FSMTC = 0x20, 
  CC2420_MANAND = 0x21, 
  CC2420_MANOR = 0x22, 
  CC2420_AGCCTRL = 0x23, 
  CC2420_AGCTST0 = 0x24, 
  CC2420_AGCTST1 = 0x25, 
  CC2420_AGCTST2 = 0x26, 
  CC2420_FSTST0 = 0x27, 
  CC2420_FSTST1 = 0x28, 
  CC2420_FSTST2 = 0x29, 
  CC2420_FSTST3 = 0x2a, 
  CC2420_RXBPFTST = 0x2b, 
  CC2420_FMSTATE = 0x2c, 
  CC2420_ADCTST = 0x2d, 
  CC2420_DACTST = 0x2e, 
  CC2420_TOPTST = 0x2f, 
  CC2420_TXFIFO = 0x3e, 
  CC2420_RXFIFO = 0x3f
};

enum cc2420_ram_addr_enums {
  CC2420_RAM_TXFIFO = 0x000, 
  CC2420_RAM_RXFIFO = 0x080, 
  CC2420_RAM_KEY0 = 0x100, 
  CC2420_RAM_RXNONCE = 0x110, 
  CC2420_RAM_SABUF = 0x120, 
  CC2420_RAM_KEY1 = 0x130, 
  CC2420_RAM_TXNONCE = 0x140, 
  CC2420_RAM_CBCSTATE = 0x150, 
  CC2420_RAM_IEEEADR = 0x160, 
  CC2420_RAM_PANID = 0x168, 
  CC2420_RAM_SHORTADR = 0x16a
};

enum cc2420_nonce_enums {
  CC2420_NONCE_BLOCK_COUNTER = 0, 
  CC2420_NONCE_KEY_SEQ_COUNTER = 2, 
  CC2420_NONCE_FRAME_COUNTER = 3, 
  CC2420_NONCE_SOURCE_ADDRESS = 7, 
  CC2420_NONCE_FLAGS = 15
};

enum cc2420_main_enums {
  CC2420_MAIN_RESETn = 15, 
  CC2420_MAIN_ENC_RESETn = 14, 
  CC2420_MAIN_DEMOD_RESETn = 13, 
  CC2420_MAIN_MOD_RESETn = 12, 
  CC2420_MAIN_FS_RESETn = 11, 
  CC2420_MAIN_XOSC16M_BYPASS = 0
};

enum cc2420_mdmctrl0_enums {
  CC2420_MDMCTRL0_RESERVED_FRAME_MODE = 13, 
  CC2420_MDMCTRL0_PAN_COORDINATOR = 12, 
  CC2420_MDMCTRL0_ADR_DECODE = 11, 
  CC2420_MDMCTRL0_CCA_HYST = 8, 
  CC2420_MDMCTRL0_CCA_MOD = 6, 
  CC2420_MDMCTRL0_AUTOCRC = 5, 
  CC2420_MDMCTRL0_AUTOACK = 4, 
  CC2420_MDMCTRL0_PREAMBLE_LENGTH = 0
};

enum cc2420_mdmctrl1_enums {
  CC2420_MDMCTRL1_CORR_THR = 6, 
  CC2420_MDMCTRL1_DEMOD_AVG_MODE = 5, 
  CC2420_MDMCTRL1_MODULATION_MODE = 4, 
  CC2420_MDMCTRL1_TX_MODE = 2, 
  CC2420_MDMCTRL1_RX_MODE = 0
};

enum cc2420_rssi_enums {
  CC2420_RSSI_CCA_THR = 8, 
  CC2420_RSSI_RSSI_VAL = 0
};

enum cc2420_syncword_enums {
  CC2420_SYNCWORD_SYNCWORD = 0
};

enum cc2420_txctrl_enums {
  CC2420_TXCTRL_TXMIXBUF_CUR = 14, 
  CC2420_TXCTRL_TX_TURNAROUND = 13, 
  CC2420_TXCTRL_TXMIX_CAP_ARRAY = 11, 
  CC2420_TXCTRL_TXMIX_CURRENT = 9, 
  CC2420_TXCTRL_PA_CURRENT = 6, 
  CC2420_TXCTRL_RESERVED = 5, 
  CC2420_TXCTRL_PA_LEVEL = 0
};

enum cc2420_rxctrl0_enums {
  CC2420_RXCTRL0_RXMIXBUF_CUR = 12, 
  CC2420_RXCTRL0_HIGH_LNA_GAIN = 10, 
  CC2420_RXCTRL0_MED_LNA_GAIN = 8, 
  CC2420_RXCTRL0_LOW_LNA_GAIN = 6, 
  CC2420_RXCTRL0_HIGH_LNA_CURRENT = 4, 
  CC2420_RXCTRL0_MED_LNA_CURRENT = 2, 
  CC2420_RXCTRL0_LOW_LNA_CURRENT = 0
};

enum cc2420_rxctrl1_enums {
  CC2420_RXCTRL1_RXBPF_LOCUR = 13, 
  CC2420_RXCTRL1_RXBPF_MIDCUR = 12, 
  CC2420_RXCTRL1_LOW_LOWGAIN = 11, 
  CC2420_RXCTRL1_MED_LOWGAIN = 10, 
  CC2420_RXCTRL1_HIGH_HGM = 9, 
  CC2420_RXCTRL1_MED_HGM = 8, 
  CC2420_RXCTRL1_LNA_CAP_ARRAY = 6, 
  CC2420_RXCTRL1_RXMIX_TAIL = 4, 
  CC2420_RXCTRL1_RXMIX_VCM = 2, 
  CC2420_RXCTRL1_RXMIX_CURRENT = 0
};

enum cc2420_rsctrl_enums {
  CC2420_FSCTRL_LOCK_THR = 14, 
  CC2420_FSCTRL_CAL_DONE = 13, 
  CC2420_FSCTRL_CAL_RUNNING = 12, 
  CC2420_FSCTRL_LOCK_LENGTH = 11, 
  CC2420_FSCTRL_LOCK_STATUS = 10, 
  CC2420_FSCTRL_FREQ = 0
};

enum cc2420_secctrl0_enums {
  CC2420_SECCTRL0_RXFIFO_PROTECTION = 9, 
  CC2420_SECCTRL0_SEC_CBC_HEAD = 8, 
  CC2420_SECCTRL0_SEC_SAKEYSEL = 7, 
  CC2420_SECCTRL0_SEC_TXKEYSEL = 6, 
  CC2420_SECCTRL0_SEC_RXKEYSEL = 5, 
  CC2420_SECCTRL0_SEC_M = 2, 
  CC2420_SECCTRL0_SEC_MODE = 0
};

enum cc2420_secctrl1_enums {
  CC2420_SECCTRL1_SEC_TXL = 8, 
  CC2420_SECCTRL1_SEC_RXL = 0
};

enum cc2420_battmon_enums {
  CC2420_BATTMON_BATT_OK = 6, 
  CC2420_BATTMON_BATTMON_EN = 5, 
  CC2420_BATTMON_BATTMON_VOLTAGE = 0
};

enum cc2420_iocfg0_enums {
  CC2420_IOCFG0_BCN_ACCEPT = 11, 
  CC2420_IOCFG0_FIFO_POLARITY = 10, 
  CC2420_IOCFG0_FIFOP_POLARITY = 9, 
  CC2420_IOCFG0_SFD_POLARITY = 8, 
  CC2420_IOCFG0_CCA_POLARITY = 7, 
  CC2420_IOCFG0_FIFOP_THR = 0
};

enum cc2420_iocfg1_enums {
  CC2420_IOCFG1_HSSD_SRC = 10, 
  CC2420_IOCFG1_SFDMUX = 5, 
  CC2420_IOCFG1_CCAMUX = 0
};

enum cc2420_manfidl_enums {
  CC2420_MANFIDL_PARTNUM = 12, 
  CC2420_MANFIDL_MANFID = 0
};

enum cc2420_manfidh_enums {
  CC2420_MANFIDH_VERSION = 12, 
  CC2420_MANFIDH_PARTNUM = 0
};

enum cc2420_fsmtc_enums {
  CC2420_FSMTC_TC_RXCHAIN2RX = 13, 
  CC2420_FSMTC_TC_SWITCH2TX = 10, 
  CC2420_FSMTC_TC_PAON2TX = 6, 
  CC2420_FSMTC_TC_TXEND2SWITCH = 3, 
  CC2420_FSMTC_TC_TXEND2PAOFF = 0
};

enum cc2420_sfdmux_enums {
  CC2420_SFDMUX_SFD = 0, 
  CC2420_SFDMUX_XOSC16M_STABLE = 24
};

enum cc2420_security_enums {
  CC2420_NO_SEC = 0, 
  CC2420_CBC_MAC = 1, 
  CC2420_CTR = 2, 
  CC2420_CCM = 3, 
  NO_SEC = 0, 
  CBC_MAC_4 = 1, 
  CBC_MAC_8 = 2, 
  CBC_MAC_16 = 3, 
  CTR = 4, 
  CCM_4 = 5, 
  CCM_8 = 6, 
  CCM_16 = 7
};


enum __nesc_unnamed4274 {

  CC2420_INVALID_TIMESTAMP = 0x80000000L
};
# 6 "/home/tinyos/local/src/tinyos-2.x/tos/types/AM.h"
typedef nx_uint8_t nx_am_id_t;
typedef nx_uint8_t nx_am_group_t;
typedef nx_uint16_t nx_am_addr_t;

typedef uint8_t am_id_t;
typedef uint8_t am_group_t;
typedef uint16_t am_addr_t;

enum __nesc_unnamed4275 {
  AM_BROADCAST_ADDR = 0xffff
};









enum __nesc_unnamed4276 {
  TOS_AM_GROUP = 0x22, 
  TOS_AM_ADDRESS = 1
};
# 72 "/home/tinyos/local/src/tinyos-2.x/tos/lib/serial/Serial.h"
typedef uint8_t uart_id_t;



enum __nesc_unnamed4277 {
  HDLC_FLAG_BYTE = 0x7e, 
  HDLC_CTLESC_BYTE = 0x7d
};



enum __nesc_unnamed4278 {
  TOS_SERIAL_ACTIVE_MESSAGE_ID = 0, 
  TOS_SERIAL_CC1000_ID = 1, 
  TOS_SERIAL_802_15_4_ID = 2, 
  TOS_SERIAL_UNKNOWN_ID = 255
};


enum __nesc_unnamed4279 {
  SERIAL_PROTO_ACK = 67, 
  SERIAL_PROTO_PACKET_ACK = 68, 
  SERIAL_PROTO_PACKET_NOACK = 69, 
  SERIAL_PROTO_PACKET_UNKNOWN = 255
};
#line 110
#line 98
typedef struct radio_stats {
  uint8_t version;
  uint8_t flags;
  uint8_t reserved;
  uint8_t platform;
  uint16_t MTU;
  uint16_t radio_crc_fail;
  uint16_t radio_queue_drops;
  uint16_t serial_crc_fail;
  uint16_t serial_tx_fail;
  uint16_t serial_short_packets;
  uint16_t serial_proto_drops;
} radio_stats_t;







#line 112
typedef nx_struct serial_header {
  nx_am_addr_t dest;
  nx_am_addr_t src;
  nx_uint8_t length;
  nx_am_group_t group;
  nx_am_id_t type;
} __attribute__((packed)) serial_header_t;




#line 120
typedef nx_struct serial_packet {
  serial_header_t header;
  nx_uint8_t data[];
} __attribute__((packed)) serial_packet_t;



#line 125
typedef nx_struct serial_metadata {
  nx_uint8_t ack;
} __attribute__((packed)) serial_metadata_t;
# 48 "/home/tinyos/local/src/tinyos-2.x/tos/platforms/telosa/platform_message.h"
#line 45
typedef union message_header {
  cc2420_header_t cc2420;
  serial_header_t serial;
} message_header_t;



#line 50
typedef union TOSRadioFooter {
  cc2420_footer_t cc2420;
} message_footer_t;




#line 54
typedef union TOSRadioMetadata {
  cc2420_metadata_t cc2420;
  serial_metadata_t serial;
} message_metadata_t;
# 19 "/home/tinyos/local/src/tinyos-2.x/tos/types/message.h"
#line 14
typedef nx_struct message_t {
  nx_uint8_t header[sizeof(message_header_t )];
  nx_uint8_t data[102];
  nx_uint8_t footer[sizeof(message_footer_t )];
  nx_uint8_t metadata[sizeof(message_metadata_t )];
} __attribute__((packed)) message_t;
# 38 "/home/tinyos/local/src/tinyos-2.x/support/sdk/c/blip/lib6lowpan/ip.h"
enum __nesc_unnamed4280 {




  FRAG_EXPIRE_TIME = 4096
};





struct in6_addr {

  union __nesc_unnamed4281 {

    uint8_t u6_addr8[16];
    uint16_t u6_addr16[8];
    uint32_t u6_addr32[4];
  } in6_u;
};




struct sockaddr_in6 {
  uint16_t sin6_port;
  struct in6_addr sin6_addr;
};






struct ip6_hdr {
  uint8_t vlfc[4];
  uint16_t plen;
  uint8_t nxt_hdr;
  uint8_t hlim;
  struct in6_addr ip6_src;
  struct in6_addr ip6_dst;
} __attribute((packed)) ;









struct ip6_ext {
  uint8_t nxt_hdr;
  uint8_t len;
  uint8_t data[0];
};

struct tlv_hdr {
  uint8_t type;
  uint8_t len;
};



enum __nesc_unnamed4282 {
  IANA_ICMP = 58, 
  IANA_UDP = 17, 
  IANA_TCP = 6, 



  IPV6_HOP = 0, 
  IPV6_DEST = 60, 
  IPV6_ROUTING = 43, 
  IPV6_FRAG = 44, 
  IPV6_AUTH = 51, 
  IPV6_SEC = 50, 
  IPV6_MOBILITY = 135, 
  IPV6_NONEXT = 59
};






enum __nesc_unnamed4283 {
  IP6ROUTE_TYPE_SOURCE = 0, 
  IP6ROUTE_TYPE_INVAL = 1, 
  IP6ROUTE_FLAG_CONTROLLER = 0x8, 
  IP6ROUTE_FLAG_MASK = IP6ROUTE_FLAG_CONTROLLER, 

  IP_EXT_SOURCE_DISPATCH = 0x40, 
  IP_EXT_SOURCE_MASK = 0xc0, 


  IP_EXT_SOURCE_CONTROLLER = 0x40, 




  IP_EXT_SOURCE_INSTALL = 0x10, 
  IP_EXT_SOURCE_INSTALL_MASK = 0x10, 





  IP_EXT_SOURCE_INST_SRC = 0x20, 
  IP_EXT_SOURCE_INST_DST = 0x40, 


  TLV_TYPE_TOPOLOGY = 0x0a, 


  TLV_TYPE_INSTALL = 0x0b, 
  TLV_TYPE_FLOW = 0x0c, 
  TLV_TYPE_MCASTSEQ = 0x0d
};

struct ip6_route {
  uint8_t nxt_hdr;
  uint8_t len;
  uint8_t type;
  uint8_t segs_remain;
  uint16_t hops[0];
};






struct icmp6_hdr {
  uint8_t type;
  uint8_t code;
  uint16_t cksum;
};

enum __nesc_unnamed4284 {
  ICMP_TYPE_ECHO_DEST_UNREACH = 1, 
  ICMP_TYPE_ECHO_PKT_TOO_BIG = 2, 
  ICMP_TYPE_ECHO_TIME_EXCEEDED = 3, 
  ICMP_TYPE_ECHO_PARAM_PROBLEM = 4, 
  ICMP_TYPE_ECHO_REQUEST = 128, 
  ICMP_TYPE_ECHO_REPLY = 129, 
  ICMP_TYPE_ROUTER_SOL = 133, 
  ICMP_TYPE_ROUTER_ADV = 134, 
  ICMP_TYPE_NEIGHBOR_SOL = 135, 
  ICMP_TYPE_NEIGHBOR_ADV = 136, 
  ICMP_NEIGHBOR_HOPLIMIT = 255, 

  ICMP_CODE_HOPLIMIT_EXCEEDED = 0, 
  ICMP_CODE_ASSEMBLY_EXCEEDED = 1
};




struct udp_hdr {
  uint16_t srcport;
  uint16_t dstport;
  uint16_t len;
  uint16_t chksum;
};




enum __nesc_unnamed4285 {
  TCP_FLAG_FIN = 0x1, 
  TCP_FLAG_SYN = 0x2, 
  TCP_FLAG_RST = 0x4, 
  TCP_FLAG_PSH = 0x8, 
  TCP_FLAG_ACK = 0x10, 
  TCP_FLAG_URG = 0x20, 
  TCP_FLAG_ECE = 0x40, 
  TCP_FLAG_CWR = 0x80
};

struct tcp_hdr {
  uint16_t srcport;
  uint16_t dstport;
  uint32_t seqno;
  uint32_t ackno;
  uint8_t offset;
  uint8_t flags;
  uint16_t window;
  uint16_t chksum;
  uint16_t urgent;
};




struct ip_metadata {
  ieee154_saddr_t sender;
  uint8_t lqi;
  uint8_t padding[1];
};

struct flow_match {
  cmpr_ip6_addr_t src;
  cmpr_ip6_addr_t dest;
};

struct rinstall_header {
  struct flow_match match;
  uint8_t flags;
  uint8_t path_len;
  cmpr_ip6_addr_t path[0];
};

enum __nesc_unnamed4286 {

  HYDRO_INSTALL_METHOD_MASK = 0x03, 
  HYDRO_METHOD_HOP = 0x01, 
  HYDRO_METHOD_SOURCE = 0x02, 


  HYDRO_INSTALL_REVERSE = 0x04, 


  HYDRO_INSTALL_UNINSTALL_MASK = 0x08
};


enum __nesc_unnamed4287 {
  T_INVAL_NEIGH = 0xef, 
  T_SET_NEIGH = 0xee
};
#line 287
struct generic_header {



  uint8_t len;
  union __nesc_unnamed4288 {

    struct ip6_ext *ext;
    struct ip6_route *sh;
    struct udp_hdr *udp;
    uint8_t *data;
  } hdr;
  struct generic_header *next;
};

enum __nesc_unnamed4289 {
  IP_NOHEADERS = 1 << 0, 
  IP_MCAST = 1 << 1, 
  IP_NOADDRESS = 1 << 2
};

struct split_ip_msg {
  struct generic_header *headers;
  uint16_t data_len;
  uint8_t *data;








  struct ip6_hdr hdr;
  uint8_t next[0];
};





void inet_pton6(char *addr, struct in6_addr *dest);
int inet_ntop6(struct in6_addr *addr, char *buf, int cnt);
# 21 "/home/tinyos/local/src/tinyos-2.x/support/sdk/c/blip/lib6lowpan/nwbyte.h"
static __inline uint32_t __attribute((unused)) ntoh32(uint32_t i);
# 58 "/home/tinyos/local/src/tinyos-2.x/support/sdk/c/blip/lib6lowpan/lib6lowpan.h"
uint16_t getHeaderBitmap(packed_lowmsg_t *lowmsg);





uint8_t *getLowpanPayload(packed_lowmsg_t *lowmsg);
#line 79
__inline uint8_t hasFrag1Header(packed_lowmsg_t *msg);
__inline uint8_t hasFragNHeader(packed_lowmsg_t *msg);
#line 105
__inline uint8_t getFragDgramSize(packed_lowmsg_t *msg, uint16_t *size);
__inline uint8_t getFragDgramTag(packed_lowmsg_t *msg, uint16_t *tag);
__inline uint8_t getFragDgramOffset(packed_lowmsg_t *msg, uint8_t *size);


__inline uint8_t setFragDgramTag(packed_lowmsg_t *msg, uint16_t tag);
#line 158
#line 139
typedef struct __nesc_unnamed4290 {

  uint8_t nxt_hdr;

  uint8_t *payload_start;

  uint8_t *header_end;

  uint8_t payload_offset;

  uint8_t *hlim;



  uint8_t *transport_ptr;

  struct ip6_ext *hdr_hop;
  struct ip6_route *hdr_route;
  struct ip6_ext *hdr_dest;
} unpack_info_t;

uint8_t *unpackHeaders(packed_lowmsg_t *pkt, unpack_info_t *u_info, 
uint8_t *dest, uint16_t len);

void adjustPlen(struct ip6_hdr *ip, unpack_info_t *u_info);





extern uint16_t lib6lowpan_frag_tag;
#line 181
#line 171
typedef struct __nesc_unnamed4291 {
  uint16_t tag;
  uint16_t size;
  void *buf;
  uint16_t bytes_rcvd;

  uint8_t timeout;
  uint8_t nxt_hdr;
  uint8_t *transport_hdr;
  struct ip_metadata metadata;
} reconstruct_t;




#line 183
typedef struct __nesc_unnamed4292 {
  uint16_t tag;
  uint16_t offset;
} fragment_t;
#line 199
uint8_t getNextFrag(struct split_ip_msg *msg, fragment_t *progress, 
uint8_t *buf, uint16_t len);


enum __nesc_unnamed4293 {
  T_FAILED1 = 0, 
  T_FAILED2 = 1, 
  T_UNUSED = 2, 
  T_ACTIVE = 3, 
  T_ZOMBIE = 4
};
# 59 "/home/tinyos/local/src/tinyos-2.x/tos/lib/net/blip/Statistics.h"
#line 40
typedef nx_struct __nesc_unnamed4294 {
  nx_uint16_t sent;
  nx_uint16_t forwarded;
  nx_uint8_t rx_drop;
  nx_uint8_t tx_drop;
  nx_uint8_t fw_drop;
  nx_uint8_t rx_total;
  nx_uint8_t encfail;
} __attribute__((packed)) 
#line 59
ip_statistics_t;







#line 62
typedef nx_struct __nesc_unnamed4295 {
  nx_uint8_t hop_limit;
  nx_uint16_t parent;
  nx_uint16_t parent_metric;
  nx_uint16_t parent_etx;
} __attribute__((packed)) route_statistics_t;










#line 69
typedef nx_struct __nesc_unnamed4296 {
  nx_uint8_t sol_rx;
  nx_uint8_t sol_tx;
  nx_uint8_t adv_rx;
  nx_uint8_t adv_tx;
  nx_uint8_t echo_rx;
  nx_uint8_t echo_tx;
  nx_uint8_t unk_rx;
  nx_uint16_t rx;
} __attribute__((packed)) icmp_statistics_t;






#line 81
typedef nx_struct __nesc_unnamed4297 {
  nx_uint16_t sent;
  nx_uint16_t rcvd;
  nx_uint16_t cksum;
} __attribute__((packed)) udp_statistics_t;
# 29 "/home/tinyos/local/src/tinyos-2.x/tos/lib/net/blip/IPDispatch.h"
enum __nesc_unnamed4298 {
  N_PARENTS = 3, 
  N_EPOCHS = 2, 
  N_EPOCHS_COUNTED = 1, 
  N_RECONSTRUCTIONS = 2, 
  N_FORWARD_ENT = IP_NUMBER_FRAGMENTS
};

enum __nesc_unnamed4299 {
  CONF_EVICT_THRESHOLD = 5, 
  CONF_PROM_THRESHOLD = 5, 
  MAX_CONSEC_FAILURES = 11, 
  PATH_COST_DIFF_THRESH = 10, 
  LQI_DIFF_THRESH = 10, 
  LINK_EVICT_THRESH = 50, 
  RANDOM_ROUTE = 20, 
  LQI_ADMIT_THRESH = 0x200
};


static uint16_t adjustLQI(uint8_t val);


enum __nesc_unnamed4300 {
  WITHIN_THRESH = 1, 
  ABOVE_THRESH = 2, 
  BELOW_THRESH = 3
};


enum __nesc_unnamed4301 {
  TGEN_BASE_TIME = 512, 
  TGEN_MAX_INTERVAL = 60L * 1024L * 5L
};








struct epoch_stats {
  uint16_t success;
  uint16_t total;
  uint16_t receptions;
};

struct report_stats {
  uint8_t messages;
  uint8_t transmissions;
  uint8_t successes;
};

enum __nesc_unnamed4302 {
  T_PIN_OFFSET = 0, 
  T_PIN_MASK = 1 << T_PIN_OFFSET, 
  T_VALID_OFFSET = 2, 
  T_VALID_MASK = 1 << T_VALID_OFFSET, 
  T_MARKED_OFFSET = 3, 
  T_MARKED_MASK = 1 << T_MARKED_OFFSET, 
  T_MATURE_OFFSET = 4, 
  T_MATURE_MASK = 1 << T_MATURE_OFFSET, 
  T_EVICT_OFFSET = 5, 
  T_EVICT_MASK = 1 << T_EVICT_OFFSET
};

enum __nesc_unnamed4303 {



  N_NEIGH = 8, 
  N_LOW_NEIGH = 2, 
  N_FREE_NEIGH = N_NEIGH - N_LOW_NEIGH, 
  N_FLOW_ENT = 6, 
  N_FLOW_CHOICES = 2, 
  N_PARENT_CHOICES = 3, 
  T_DEF_PARENT = 0xfffd, 
  T_DEF_PARENT_SLOT = 0
};










#line 110
typedef struct __nesc_unnamed4304 {


  ieee154_saddr_t dest[N_FLOW_CHOICES + N_PARENT_CHOICES + 2];
  uint8_t current : 4;
  uint8_t nchoices : 4;
  uint8_t retries;
  uint8_t actRetries;
  uint16_t delay;
} send_policy_t;







#line 121
typedef struct __nesc_unnamed4305 {
  send_policy_t policy;
  uint8_t frags_sent;
  bool failed;
  uint8_t refcount;
  uint8_t local_flow_label;
} send_info_t;




#line 129
typedef struct __nesc_unnamed4306 {
  send_info_t *info;
  message_t *msg;
} send_entry_t;







#line 134
typedef struct __nesc_unnamed4307 {
  uint8_t timeout;
  ieee154_saddr_t l2_src;
  uint16_t old_tag;
  uint16_t new_tag;
  send_info_t *s_info;
} forward_entry_t;
#line 158
enum __nesc_unnamed4308 {
  F_VALID_MASK = 0x01, 

  F_FULL_PATH_OFFSET = 1, 
  F_FULL_PATH_MASK = 0x02, 

  MAX_PATH_LENGTH = 10, 
  N_FULL_PATH_ENTRIES = N_FLOW_CHOICES * N_FLOW_ENT
};

struct flow_path {
  uint8_t path_len;
  cmpr_ip6_addr_t path[MAX_PATH_LENGTH];
};

struct f_entry {
  uint8_t flags;
  union  {
    struct flow_path *pathE;
    cmpr_ip6_addr_t nextHop;
  } ;
};




struct flow_entry {
  uint8_t flags;
  uint8_t count;
  struct flow_match match;
  struct f_entry entries[N_FLOW_CHOICES];
};
#line 206
struct neigh_entry {
  uint8_t flags;
  uint8_t hops;
  ieee154_saddr_t neighbor;
  uint16_t costEstimate;
  uint16_t linkEstimate;
  struct epoch_stats stats[N_EPOCHS];
};
#line 240
#line 237
typedef enum __nesc_unnamed4309 {
  S_FORWARD, 
  S_REQ
} send_type_t;
# 27 "UDPReport.h"
nx_struct udp_report {
  nx_uint16_t seqno;
  nx_uint16_t sender;
  ip_statistics_t ip;
  udp_statistics_t udp;
  icmp_statistics_t icmp;
  route_statistics_t route;
} __attribute__((packed));
# 43 "/usr/lib/gcc-lib/msp430/3.2.3/include/stdarg.h" 3
typedef __builtin_va_list __gnuc_va_list;
#line 110
typedef __gnuc_va_list va_list;
# 50 "/usr/msp430/include/stdio.h" 3
int __attribute((format(printf, 3, 4))) snprintf(char *buf, size_t size, const char *fmt, ...);
# 118 "/home/tinyos/local/src/tinyos-2.x/tos/lib/net/blip/PrintfUART.h"
static inline void printfUART_init();
# 29 "/home/tinyos/local/src/tinyos-2.x/tos/lib/timer/Timer.h"
typedef struct __nesc_unnamed4310 {
#line 29
  int notUsed;
} 
#line 29
TMilli;
typedef struct __nesc_unnamed4311 {
#line 30
  int notUsed;
} 
#line 30
T32khz;
typedef struct __nesc_unnamed4312 {
#line 31
  int notUsed;
} 
#line 31
TMicro;
# 38 "/home/tinyos/local/src/tinyos-2.x/tos/chips/cc2420/IEEE802154.h"
enum ieee154_fcf_enums {
  IEEE154_FCF_FRAME_TYPE = 0, 
  IEEE154_FCF_SECURITY_ENABLED = 3, 
  IEEE154_FCF_FRAME_PENDING = 4, 
  IEEE154_FCF_ACK_REQ = 5, 
  IEEE154_FCF_INTRAPAN = 6, 
  IEEE154_FCF_DEST_ADDR_MODE = 10, 
  IEEE154_FCF_SRC_ADDR_MODE = 14
};

enum ieee154_fcf_type_enums {
  IEEE154_TYPE_BEACON = 0, 
  IEEE154_TYPE_DATA = 1, 
  IEEE154_TYPE_ACK = 2, 
  IEEE154_TYPE_MAC_CMD = 3
};

enum iee154_fcf_addr_mode_enums {
  IEEE154_ADDR_NONE = 0, 
  IEEE154_ADDR_SHORT = 2, 
  IEEE154_ADDR_EXT = 3
};
# 56 "/home/tinyos/local/src/tinyos-2.x/tos/chips/msp430/usart/msp430usart.h"
#line 48
typedef enum __nesc_unnamed4313 {

  USART_NONE = 0, 
  USART_UART = 1, 
  USART_UART_TX = 2, 
  USART_UART_RX = 3, 
  USART_SPI = 4, 
  USART_I2C = 5
} msp430_usartmode_t;










#line 58
typedef struct __nesc_unnamed4314 {
  unsigned int swrst : 1;
  unsigned int mm : 1;
  unsigned int sync : 1;
  unsigned int listen : 1;
  unsigned int clen : 1;
  unsigned int spb : 1;
  unsigned int pev : 1;
  unsigned int pena : 1;
} __attribute((packed))  msp430_uctl_t;









#line 69
typedef struct __nesc_unnamed4315 {
  unsigned int txept : 1;
  unsigned int stc : 1;
  unsigned int txwake : 1;
  unsigned int urxse : 1;
  unsigned int ssel : 2;
  unsigned int ckpl : 1;
  unsigned int ckph : 1;
} __attribute((packed))  msp430_utctl_t;










#line 79
typedef struct __nesc_unnamed4316 {
  unsigned int rxerr : 1;
  unsigned int rxwake : 1;
  unsigned int urxwie : 1;
  unsigned int urxeie : 1;
  unsigned int brk : 1;
  unsigned int oe : 1;
  unsigned int pe : 1;
  unsigned int fe : 1;
} __attribute((packed))  msp430_urctl_t;
#line 116
#line 99
typedef struct __nesc_unnamed4317 {
  unsigned int ubr : 16;

  unsigned int  : 1;
  unsigned int mm : 1;
  unsigned int  : 1;
  unsigned int listen : 1;
  unsigned int clen : 1;
  unsigned int  : 3;

  unsigned int  : 1;
  unsigned int stc : 1;
  unsigned int  : 2;
  unsigned int ssel : 2;
  unsigned int ckpl : 1;
  unsigned int ckph : 1;
  unsigned int  : 0;
} msp430_spi_config_t;





#line 118
typedef struct __nesc_unnamed4318 {
  uint16_t ubr;
  uint8_t uctl;
  uint8_t utctl;
} msp430_spi_registers_t;




#line 124
typedef union __nesc_unnamed4319 {
  msp430_spi_config_t spiConfig;
  msp430_spi_registers_t spiRegisters;
} msp430_spi_union_config_t;

msp430_spi_union_config_t msp430_spi_default_config = { 
{ 
.ubr = 0x0002, 
.ssel = 0x02, 
.clen = 1, 
.listen = 0, 
.mm = 1, 
.ckph = 1, 
.ckpl = 0, 
.stc = 1 } };
#line 169
#line 150
typedef enum __nesc_unnamed4320 {

  UBR_32KHZ_1200 = 0x001B, UMCTL_32KHZ_1200 = 0x94, 
  UBR_32KHZ_1800 = 0x0012, UMCTL_32KHZ_1800 = 0x84, 
  UBR_32KHZ_2400 = 0x000D, UMCTL_32KHZ_2400 = 0x6D, 
  UBR_32KHZ_4800 = 0x0006, UMCTL_32KHZ_4800 = 0x77, 
  UBR_32KHZ_9600 = 0x0003, UMCTL_32KHZ_9600 = 0x29, 

  UBR_1MHZ_1200 = 0x0369, UMCTL_1MHZ_1200 = 0x7B, 
  UBR_1MHZ_1800 = 0x0246, UMCTL_1MHZ_1800 = 0x55, 
  UBR_1MHZ_2400 = 0x01B4, UMCTL_1MHZ_2400 = 0xDF, 
  UBR_1MHZ_4800 = 0x00DA, UMCTL_1MHZ_4800 = 0xAA, 
  UBR_1MHZ_9600 = 0x006D, UMCTL_1MHZ_9600 = 0x44, 
  UBR_1MHZ_19200 = 0x0036, UMCTL_1MHZ_19200 = 0xB5, 
  UBR_1MHZ_38400 = 0x001B, UMCTL_1MHZ_38400 = 0x94, 
  UBR_1MHZ_57600 = 0x0012, UMCTL_1MHZ_57600 = 0x84, 
  UBR_1MHZ_76800 = 0x000D, UMCTL_1MHZ_76800 = 0x6D, 
  UBR_1MHZ_115200 = 0x0009, UMCTL_1MHZ_115200 = 0x10, 
  UBR_1MHZ_230400 = 0x0004, UMCTL_1MHZ_230400 = 0x55
} msp430_uart_rate_t;
#line 200
#line 171
typedef struct __nesc_unnamed4321 {
  unsigned int ubr : 16;

  unsigned int umctl : 8;

  unsigned int  : 1;
  unsigned int mm : 1;
  unsigned int  : 1;
  unsigned int listen : 1;
  unsigned int clen : 1;
  unsigned int spb : 1;
  unsigned int pev : 1;
  unsigned int pena : 1;
  unsigned int  : 0;

  unsigned int  : 3;
  unsigned int urxse : 1;
  unsigned int ssel : 2;
  unsigned int ckpl : 1;
  unsigned int  : 1;

  unsigned int  : 2;
  unsigned int urxwie : 1;
  unsigned int urxeie : 1;
  unsigned int  : 4;
  unsigned int  : 0;

  unsigned int utxe : 1;
  unsigned int urxe : 1;
} msp430_uart_config_t;








#line 202
typedef struct __nesc_unnamed4322 {
  uint16_t ubr;
  uint8_t umctl;
  uint8_t uctl;
  uint8_t utctl;
  uint8_t urctl;
  uint8_t ume;
} msp430_uart_registers_t;




#line 211
typedef union __nesc_unnamed4323 {
  msp430_uart_config_t uartConfig;
  msp430_uart_registers_t uartRegisters;
} msp430_uart_union_config_t;
#line 248
#line 240
typedef struct __nesc_unnamed4324 {
  unsigned int i2cstt : 1;
  unsigned int i2cstp : 1;
  unsigned int i2cstb : 1;
  unsigned int i2cctrx : 1;
  unsigned int i2cssel : 2;
  unsigned int i2ccrm : 1;
  unsigned int i2cword : 1;
} __attribute((packed))  msp430_i2ctctl_t;
#line 276
#line 253
typedef struct __nesc_unnamed4325 {
  unsigned int  : 1;
  unsigned int mst : 1;
  unsigned int  : 1;
  unsigned int listen : 1;
  unsigned int xa : 1;
  unsigned int  : 1;
  unsigned int txdmaen : 1;
  unsigned int rxdmaen : 1;

  unsigned int  : 4;
  unsigned int i2cssel : 2;
  unsigned int i2crm : 1;
  unsigned int i2cword : 1;

  unsigned int i2cpsc : 8;

  unsigned int i2csclh : 8;

  unsigned int i2cscll : 8;

  unsigned int i2coa : 10;
  unsigned int  : 6;
} msp430_i2c_config_t;








#line 278
typedef struct __nesc_unnamed4326 {
  uint8_t uctl;
  uint8_t i2ctctl;
  uint8_t i2cpsc;
  uint8_t i2csclh;
  uint8_t i2cscll;
  uint16_t i2coa;
} msp430_i2c_registers_t;




#line 287
typedef union __nesc_unnamed4327 {
  msp430_i2c_config_t i2cConfig;
  msp430_i2c_registers_t i2cRegisters;
} msp430_i2c_union_config_t;
#line 309
typedef uint8_t uart_speed_t;
typedef uint8_t uart_parity_t;
typedef uint8_t uart_duplex_t;

enum __nesc_unnamed4328 {
  TOS_UART_1200 = 0, 
  TOS_UART_1800 = 1, 
  TOS_UART_2400 = 2, 
  TOS_UART_4800 = 3, 
  TOS_UART_9600 = 4, 
  TOS_UART_19200 = 5, 
  TOS_UART_38400 = 6, 
  TOS_UART_57600 = 7, 
  TOS_UART_76800 = 8, 
  TOS_UART_115200 = 9, 
  TOS_UART_230400 = 10
};

enum __nesc_unnamed4329 {
  TOS_UART_OFF, 
  TOS_UART_RONLY, 
  TOS_UART_TONLY, 
  TOS_UART_DUPLEX
};

enum __nesc_unnamed4330 {
  TOS_UART_PARITY_NONE, 
  TOS_UART_PARITY_EVEN, 
  TOS_UART_PARITY_ODD
};
# 64 "/home/tinyos/local/src/tinyos-2.x/tos/chips/msp430/dma/Msp430Dma.h"
enum __nesc_unnamed4331 {
  DMA_CHANNELS = 3
};

enum __nesc_unnamed4332 {
  DMA_CHANNEL0 = 0, 
  DMA_CHANNEL1 = 1, 
  DMA_CHANNEL2 = 2, 
  DMA_CHANNEL_UNKNOWN = 3
};

enum __nesc_unnamed4333 {
  DMA_CHANNEL_AVAILABLE = 0, 
  DMA_CHANNEL_IN_USE = 1
};



enum __nesc_unnamed4334 {
  DMA0TSEL_SHIFT = 0, 
  DMA1TSEL_SHIFT = 4, 
  DMA2TSEL_SHIFT = 8, 
  DMATSEL_MASK = (uint16_t )0xf, 
  DMA0TSEL_MASK = 0xf, 
  DMA1TSEL_MASK = 0xf0, 
  DMA2TSEL_MASK = 0xf00
};
#line 110
#line 93
typedef enum __nesc_unnamed4335 {
  DMA_TRIGGER_DMAREQ = 0x0, 
  DMA_TRIGGER_TACCR2 = 0x1, 
  DMA_TRIGGER_TBCCR2 = 0x2, 
  DMA_TRIGGER_URXIFG0 = 0x3, 
  DMA_TRIGGER_UTXIFG0 = 0x4, 
  DMA_TRIGGER_DAC12IFG = 0x5, 
  DMA_TRIGGER_ADC12IFGx = 0x6, 
  DMA_TRIGGER_TACCR0 = 0x7, 
  DMA_TRIGGER_TBCCR0 = 0x8, 
  DMA_TRIGGER_URXIFG1 = 0x9, 
  DMA_TRIGGER_UTXIFG1 = 0xa, 
  DMA_TRIGGER_MULT = 0xb, 
  DMA_TRIGGER_DMAxIFG = 0xe, 


  DMA_TRIGGER_DMAE0 = 0xf
} dma_trigger_t;




#line 112
typedef struct dma_channel_trigger_s {
  unsigned int trigger : 4;
  unsigned int reserved : 12;
} __attribute((packed))  dma_channel_trigger_t;



enum __nesc_unnamed4336 {
  DISABLE_NMI = 0, 
  ENABLE_NMI = 1
};

enum __nesc_unnamed4337 {
  NOT_ROUND_ROBIN = 0, 
  ROUND_ROBIN = 1
};

enum __nesc_unnamed4338 {
  NOT_ON_FETCH = 0, 
  ON_FETCH = 1
};






#line 134
typedef struct dma_state_s {
  unsigned int enableNMI : 1;
  unsigned int roundRobin : 1;
  unsigned int onFetch : 1;
  unsigned int reserved : 13;
} __attribute((packed))  dma_state_t;





enum __nesc_unnamed4339 {
  DMADT_SHIFT = 12, 
  DMADT_MASK = 0x7
};








#line 150
typedef enum __nesc_unnamed4340 {
  DMA_SINGLE_TRANSFER = 0x0, 
  DMA_BLOCK_TRANSFER = 0x1, 
  DMA_BURST_BLOCK_TRANSFER = 0x2, 
  DMA_REPEATED_SINGLE_TRANSFER = 0x4, 
  DMA_REPEATED_BLOCK_TRANSFER = 0x5, 
  DMA_REPEATED_BURST_BLOCK_TRANSFER = 0x7
} dma_transfer_mode_t;


enum __nesc_unnamed4341 {
  DMASRCINCR_SHIFT = 8, 
  DMADSTINCR_SHIFT = 10, 
  DMAINCR_MASK = 0x3
};





#line 166
typedef enum __nesc_unnamed4342 {
  DMA_ADDRESS_UNCHANGED = 0x0, 
  DMA_ADDRESS_DECREMENTED = 0x2, 
  DMA_ADDRESS_INCREMENTED = 0x3
} dma_incr_t;




#line 172
typedef enum __nesc_unnamed4343 {
  DMA_WORD = 0x0, 
  DMA_BYTE = 0x1
} dma_byte_t;





#line 178
typedef enum __nesc_unnamed4344 {
  DMA_EDGE_SENSITIVE = 0x0, 
  DMA_LEVEL_SENSITIVE = 0x1
} dma_level_t;
#line 196
#line 183
typedef struct dma_channel_state_s {
  unsigned int request : 1;
  unsigned int abort : 1;
  unsigned int interruptEnable : 1;
  unsigned int interruptFlag : 1;
  unsigned int enable : 1;
  unsigned int level : 1;
  unsigned int srcByte : 1;
  unsigned int dstByte : 1;
  unsigned int srcIncrement : 2;
  unsigned int dstIncrement : 2;
  unsigned int transferMode : 3;
  unsigned int reserved2 : 1;
} __attribute((packed))  dma_channel_state_t;
# 33 "/home/tinyos/local/src/tinyos-2.x/tos/types/Resource.h"
typedef uint8_t resource_client_id_t;
# 32 "/home/tinyos/local/src/tinyos-2.x/tos/chips/cc2420/CC2420TimeSyncMessage.h"
typedef nx_uint32_t timesync_radio_t;





#line 34
typedef struct timesync_footer_t {

  nx_am_id_t type;
  timesync_radio_t timestamp;
} timesync_footer_t;
# 37 "/home/tinyos/local/src/tinyos-2.x/support/sdk/c/blip/lib6lowpan/in_cksum.h"
#line 34
typedef struct __nesc_unnamed4345 {
  const uint8_t *ptr;
  int len;
} vec_t;

extern int in_cksum(const vec_t *vec, int veclen);



extern uint16_t msg_cksum(struct split_ip_msg *msg, uint8_t nxt_hdr);
# 36 "/home/tinyos/local/src/tinyos-2.x/support/sdk/c/blip/lib6lowpan/ip_malloc.h"
typedef uint16_t bndrt_t;

void ip_malloc_init();
void *ip_malloc(uint16_t sz);
void ip_free(void *ptr);
# 31 "/home/tinyos/local/src/tinyos-2.x/tos/lib/net/blip/table.h"
#line 27
typedef struct __nesc_unnamed4346 {
  void *data;
  uint16_t elt_len;
  uint16_t n_elts;
} table_t;

void table_init(table_t *table, void *data, uint16_t elt_len, uint16_t n_elts);
void *table_search(table_t *table, int (*pred)(void *arg_0x41137a20));
void table_map(table_t *table, void (*fn)(void *arg_0x411363c0));
# 26 "/home/tinyos/local/src/tinyos-2.x/tos/lib/net/blip/IPAddressP.nc"
extern struct in6_addr __my_address;
extern uint8_t globalPrefix;
# 2 "/home/tinyos/local/src/tinyos-2.x/tos/lib/net/blip/platform/CC2420ReadLqiC.nc"
static uint16_t adjustLQI(uint8_t val);
# 25 "/home/tinyos/local/src/tinyos-2.x/tos/lib/net/blip/ICMP.h"
enum __nesc_unnamed4347 {
  ICMP_EXT_TYPE_PREFIX = 3, 
  ICMP_EXT_TYPE_BEACON = 17
};


enum __nesc_unnamed4348 {

  TRICKLE_JITTER = 10240, 

  TRICKLE_PERIOD = 4096, 


  TRICKLE_MAX = TRICKLE_PERIOD << 5
};
#line 60
#line 54
typedef nx_struct icmp6_echo_hdr {
  nx_uint8_t type;
  nx_uint8_t code;
  nx_uint16_t cksum;
  nx_uint16_t ident;
  nx_uint16_t seqno;
} __attribute__((packed)) icmp_echo_hdr_t;
#line 72
#line 62
typedef nx_struct radv {
  nx_uint8_t type;
  nx_uint8_t code;
  nx_uint16_t cksum;
  nx_uint8_t hlim;
  nx_uint8_t flags;
  nx_uint16_t lifetime;
  nx_uint32_t reachable_time;
  nx_uint32_t retrans_time;
  nx_uint8_t options[0];
} __attribute__((packed)) radv_t;






#line 74
typedef nx_struct rsol {
  nx_uint8_t type;
  nx_uint8_t code;
  nx_uint16_t cksum;
  nx_uint32_t reserved;
} __attribute__((packed)) rsol_t;










#line 81
typedef nx_struct rpfx {
  nx_uint8_t type;
  nx_uint8_t length;
  nx_uint8_t pfx_len;
  nx_uint8_t flags;
  nx_uint32_t valid_lifetime;
  nx_uint32_t preferred_lifetime;
  nx_uint32_t reserved;
  nx_uint8_t prefix[16];
} __attribute__((packed)) pfx_t;







#line 92
typedef nx_struct __nesc_unnamed4349 {
  nx_uint8_t type;
  nx_uint8_t length;
  nx_uint16_t metric;
  nx_uint16_t seqno;
  nx_uint8_t pad[2];
} __attribute__((packed)) rqual_t;

struct icmp_stats {
  uint16_t seq;
  uint8_t ttl;
  uint32_t rtt;
};
# 37 "/home/tinyos/local/src/tinyos-2.x/tos/chips/sht11/SensirionSht11.h"
enum __nesc_unnamed4350 {
  SHT11_TEMPERATURE_BITS = 14, 
  SHT11_HUMIDITY_BITS = 12
};

enum __nesc_unnamed4351 {
  SHT11_STATUS_LOW_RES_BIT = 1 << 0, 
  SHT11_STATUS_NO_RELOAD_BIT = 1 << 1, 
  SHT11_STATUS_HEATER_ON_BIT = 1 << 2, 
  SHT11_STATUS_LOW_BATTERY_BIT = 1 << 6
};
# 25 "/home/tinyos/local/src/tinyos-2.x/tos/lib/net/blip/shell/Shell.h"
enum __nesc_unnamed4352 {
  MAX_REPLY_LEN = 128
};
typedef TMilli TCPEchoP__StatusTimer__precision_tag;
typedef ip_statistics_t TCPEchoP__IPStats__stat_str;
typedef udp_statistics_t TCPEchoP__UDPStats__stat_str;
typedef route_statistics_t TCPEchoP__RouteStats__stat_str;
typedef icmp_statistics_t TCPEchoP__ICMPStats__stat_str;
enum /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Timer*/Msp430Timer32khzC__0____nesc_unnamed4353 {
  Msp430Timer32khzC__0__ALARM_ID = 0U
};
typedef T32khz /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__0__frequency_tag;
typedef /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__0__frequency_tag /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__0__Alarm__precision_tag;
typedef uint16_t /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__0__Alarm__size_type;
typedef T32khz /*Msp430Counter32khzC.Counter*/Msp430CounterC__0__frequency_tag;
typedef /*Msp430Counter32khzC.Counter*/Msp430CounterC__0__frequency_tag /*Msp430Counter32khzC.Counter*/Msp430CounterC__0__Counter__precision_tag;
typedef uint16_t /*Msp430Counter32khzC.Counter*/Msp430CounterC__0__Counter__size_type;
typedef TMilli /*CounterMilli32C.Transform*/TransformCounterC__0__to_precision_tag;
typedef uint32_t /*CounterMilli32C.Transform*/TransformCounterC__0__to_size_type;
typedef T32khz /*CounterMilli32C.Transform*/TransformCounterC__0__from_precision_tag;
typedef uint16_t /*CounterMilli32C.Transform*/TransformCounterC__0__from_size_type;
typedef uint32_t /*CounterMilli32C.Transform*/TransformCounterC__0__upper_count_type;
typedef /*CounterMilli32C.Transform*/TransformCounterC__0__from_precision_tag /*CounterMilli32C.Transform*/TransformCounterC__0__CounterFrom__precision_tag;
typedef /*CounterMilli32C.Transform*/TransformCounterC__0__from_size_type /*CounterMilli32C.Transform*/TransformCounterC__0__CounterFrom__size_type;
typedef /*CounterMilli32C.Transform*/TransformCounterC__0__to_precision_tag /*CounterMilli32C.Transform*/TransformCounterC__0__Counter__precision_tag;
typedef /*CounterMilli32C.Transform*/TransformCounterC__0__to_size_type /*CounterMilli32C.Transform*/TransformCounterC__0__Counter__size_type;
typedef TMilli /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__to_precision_tag;
typedef uint32_t /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__to_size_type;
typedef T32khz /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__from_precision_tag;
typedef uint16_t /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__from_size_type;
typedef /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__to_precision_tag /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__Alarm__precision_tag;
typedef /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__to_size_type /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__Alarm__size_type;
typedef /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__from_precision_tag /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__AlarmFrom__precision_tag;
typedef /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__from_size_type /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__AlarmFrom__size_type;
typedef /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__to_precision_tag /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__Counter__precision_tag;
typedef /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__to_size_type /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__Counter__size_type;
typedef TMilli /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__precision_tag;
typedef /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__precision_tag /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__Alarm__precision_tag;
typedef uint32_t /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__Alarm__size_type;
typedef /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__precision_tag /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__Timer__precision_tag;
typedef TMilli /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__precision_tag;
typedef /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__precision_tag /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__TimerFrom__precision_tag;
typedef /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__precision_tag /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__Timer__precision_tag;
typedef TMilli /*HilTimerMilliC.CounterToLocalTimeC*/CounterToLocalTimeC__0__precision_tag;
typedef /*HilTimerMilliC.CounterToLocalTimeC*/CounterToLocalTimeC__0__precision_tag /*HilTimerMilliC.CounterToLocalTimeC*/CounterToLocalTimeC__0__LocalTime__precision_tag;
typedef /*HilTimerMilliC.CounterToLocalTimeC*/CounterToLocalTimeC__0__precision_tag /*HilTimerMilliC.CounterToLocalTimeC*/CounterToLocalTimeC__0__Counter__precision_tag;
typedef uint32_t /*HilTimerMilliC.CounterToLocalTimeC*/CounterToLocalTimeC__0__Counter__size_type;
typedef T32khz CC2420ControlP__StartupTimer__precision_tag;
typedef uint32_t CC2420ControlP__StartupTimer__size_type;
typedef uint16_t CC2420ControlP__ReadRssi__val_t;
enum /*AlarmMultiplexC.Alarm.Alarm32khz32C.AlarmC.Msp430Timer*/Msp430Timer32khzC__1____nesc_unnamed4354 {
  Msp430Timer32khzC__1__ALARM_ID = 1U
};
typedef T32khz /*AlarmMultiplexC.Alarm.Alarm32khz32C.AlarmC.Msp430Alarm*/Msp430AlarmC__1__frequency_tag;
typedef /*AlarmMultiplexC.Alarm.Alarm32khz32C.AlarmC.Msp430Alarm*/Msp430AlarmC__1__frequency_tag /*AlarmMultiplexC.Alarm.Alarm32khz32C.AlarmC.Msp430Alarm*/Msp430AlarmC__1__Alarm__precision_tag;
typedef uint16_t /*AlarmMultiplexC.Alarm.Alarm32khz32C.AlarmC.Msp430Alarm*/Msp430AlarmC__1__Alarm__size_type;
typedef T32khz /*Counter32khz32C.Transform*/TransformCounterC__1__to_precision_tag;
typedef uint32_t /*Counter32khz32C.Transform*/TransformCounterC__1__to_size_type;
typedef T32khz /*Counter32khz32C.Transform*/TransformCounterC__1__from_precision_tag;
typedef uint16_t /*Counter32khz32C.Transform*/TransformCounterC__1__from_size_type;
typedef uint16_t /*Counter32khz32C.Transform*/TransformCounterC__1__upper_count_type;
typedef /*Counter32khz32C.Transform*/TransformCounterC__1__from_precision_tag /*Counter32khz32C.Transform*/TransformCounterC__1__CounterFrom__precision_tag;
typedef /*Counter32khz32C.Transform*/TransformCounterC__1__from_size_type /*Counter32khz32C.Transform*/TransformCounterC__1__CounterFrom__size_type;
typedef /*Counter32khz32C.Transform*/TransformCounterC__1__to_precision_tag /*Counter32khz32C.Transform*/TransformCounterC__1__Counter__precision_tag;
typedef /*Counter32khz32C.Transform*/TransformCounterC__1__to_size_type /*Counter32khz32C.Transform*/TransformCounterC__1__Counter__size_type;
typedef T32khz /*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC__1__to_precision_tag;
typedef uint32_t /*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC__1__to_size_type;
typedef T32khz /*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC__1__from_precision_tag;
typedef uint16_t /*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC__1__from_size_type;
typedef /*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC__1__to_precision_tag /*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC__1__Alarm__precision_tag;
typedef /*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC__1__to_size_type /*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC__1__Alarm__size_type;
typedef /*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC__1__from_precision_tag /*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC__1__AlarmFrom__precision_tag;
typedef /*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC__1__from_size_type /*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC__1__AlarmFrom__size_type;
typedef /*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC__1__to_precision_tag /*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC__1__Counter__precision_tag;
typedef /*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC__1__to_size_type /*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC__1__Counter__size_type;
enum /*CC2420ControlC.Spi*/CC2420SpiC__0____nesc_unnamed4355 {
  CC2420SpiC__0__CLIENT_ID = 0U
};
enum /*CC2420SpiWireC.HplCC2420SpiC.SpiC*/Msp430Spi0C__0____nesc_unnamed4356 {
  Msp430Spi0C__0__CLIENT_ID = 0U
};
enum Msp430SpiDma0P____nesc_unnamed4357 {
  Msp430SpiDma0P__DMA_CHANNELS = 3
};
enum Msp430SpiDma0P____nesc_unnamed4358 {
  Msp430SpiDma0P__DMA_CHANNEL0 = 0, Msp430SpiDma0P__DMA_CHANNEL1 = 1, Msp430SpiDma0P__DMA_CHANNEL2 = 2, Msp430SpiDma0P__DMA_CHANNEL_UNKNOWN = 3
};
enum Msp430SpiDma0P____nesc_unnamed4359 {
  Msp430SpiDma0P__DMA_CHANNEL_AVAILABLE = 0, Msp430SpiDma0P__DMA_CHANNEL_IN_USE = 1
};
enum Msp430SpiDma0P____nesc_unnamed4360 {
  Msp430SpiDma0P__DMA0TSEL_SHIFT = 0, Msp430SpiDma0P__DMA1TSEL_SHIFT = 4, Msp430SpiDma0P__DMA2TSEL_SHIFT = 8, Msp430SpiDma0P__DMATSEL_MASK = (uint16_t )0xf, Msp430SpiDma0P__DMA0TSEL_MASK = 0xf, Msp430SpiDma0P__DMA1TSEL_MASK = 0xf0, Msp430SpiDma0P__DMA2TSEL_MASK = 0xf00
};
typedef enum Msp430SpiDma0P____nesc_unnamed4361 {
  Msp430SpiDma0P__DMA_TRIGGER_DMAREQ = 0x0, Msp430SpiDma0P__DMA_TRIGGER_TACCR2 = 0x1, Msp430SpiDma0P__DMA_TRIGGER_TBCCR2 = 0x2, Msp430SpiDma0P__DMA_TRIGGER_URXIFG0 = 0x3, Msp430SpiDma0P__DMA_TRIGGER_UTXIFG0 = 0x4, Msp430SpiDma0P__DMA_TRIGGER_DAC12IFG = 0x5, Msp430SpiDma0P__DMA_TRIGGER_ADC12IFGx = 0x6, Msp430SpiDma0P__DMA_TRIGGER_TACCR0 = 0x7, Msp430SpiDma0P__DMA_TRIGGER_TBCCR0 = 0x8, Msp430SpiDma0P__DMA_TRIGGER_URXIFG1 = 0x9, Msp430SpiDma0P__DMA_TRIGGER_UTXIFG1 = 0xa, Msp430SpiDma0P__DMA_TRIGGER_MULT = 0xb, Msp430SpiDma0P__DMA_TRIGGER_DMAxIFG = 0xe, Msp430SpiDma0P__DMA_TRIGGER_DMAE0 = 0xf
} Msp430SpiDma0P__dma_trigger_t;
typedef struct Msp430SpiDma0P__dma_channel_trigger_s {
  unsigned int trigger : 4;
  unsigned int reserved : 12;
} __attribute((packed))  Msp430SpiDma0P__dma_channel_trigger_t;
enum Msp430SpiDma0P____nesc_unnamed4362 {
  Msp430SpiDma0P__DISABLE_NMI = 0, Msp430SpiDma0P__ENABLE_NMI = 1
};
enum Msp430SpiDma0P____nesc_unnamed4363 {
  Msp430SpiDma0P__NOT_ROUND_ROBIN = 0, Msp430SpiDma0P__ROUND_ROBIN = 1
};
enum Msp430SpiDma0P____nesc_unnamed4364 {
  Msp430SpiDma0P__NOT_ON_FETCH = 0, Msp430SpiDma0P__ON_FETCH = 1
};
typedef struct Msp430SpiDma0P__dma_state_s {
  unsigned int enableNMI : 1;
  unsigned int roundRobin : 1;
  unsigned int onFetch : 1;
  unsigned int reserved : 13;
} __attribute((packed))  Msp430SpiDma0P__dma_state_t;
enum Msp430SpiDma0P____nesc_unnamed4365 {
  Msp430SpiDma0P__DMADT_SHIFT = 12, Msp430SpiDma0P__DMADT_MASK = 0x7
};
typedef enum Msp430SpiDma0P____nesc_unnamed4366 {
  Msp430SpiDma0P__DMA_SINGLE_TRANSFER = 0x0, Msp430SpiDma0P__DMA_BLOCK_TRANSFER = 0x1, Msp430SpiDma0P__DMA_BURST_BLOCK_TRANSFER = 0x2, Msp430SpiDma0P__DMA_REPEATED_SINGLE_TRANSFER = 0x4, Msp430SpiDma0P__DMA_REPEATED_BLOCK_TRANSFER = 0x5, Msp430SpiDma0P__DMA_REPEATED_BURST_BLOCK_TRANSFER = 0x7
} Msp430SpiDma0P__dma_transfer_mode_t;
enum Msp430SpiDma0P____nesc_unnamed4367 {
  Msp430SpiDma0P__DMASRCINCR_SHIFT = 8, Msp430SpiDma0P__DMADSTINCR_SHIFT = 10, Msp430SpiDma0P__DMAINCR_MASK = 0x3
};
typedef enum Msp430SpiDma0P____nesc_unnamed4368 {
  Msp430SpiDma0P__DMA_ADDRESS_UNCHANGED = 0x0, Msp430SpiDma0P__DMA_ADDRESS_DECREMENTED = 0x2, Msp430SpiDma0P__DMA_ADDRESS_INCREMENTED = 0x3
} Msp430SpiDma0P__dma_incr_t;
typedef enum Msp430SpiDma0P____nesc_unnamed4369 {
  Msp430SpiDma0P__DMA_WORD = 0x0, Msp430SpiDma0P__DMA_BYTE = 0x1
} Msp430SpiDma0P__dma_byte_t;
typedef enum Msp430SpiDma0P____nesc_unnamed4370 {
  Msp430SpiDma0P__DMA_EDGE_SENSITIVE = 0x0, Msp430SpiDma0P__DMA_LEVEL_SENSITIVE = 0x1
} Msp430SpiDma0P__dma_level_t;
typedef struct Msp430SpiDma0P__dma_channel_state_s {
  unsigned int request : 1;
  unsigned int abort : 1;
  unsigned int interruptEnable : 1;
  unsigned int interruptFlag : 1;
  unsigned int enable : 1;
  unsigned int level : 1;
  unsigned int srcByte : 1;
  unsigned int dstByte : 1;
  unsigned int srcIncrement : 2;
  unsigned int dstIncrement : 2;
  unsigned int transferMode : 3;
  unsigned int reserved2 : 1;
} __attribute((packed))  Msp430SpiDma0P__dma_channel_state_t;
enum /*CC2420SpiWireC.HplCC2420SpiC.SpiC.UsartC*/Msp430Usart0C__0____nesc_unnamed4371 {
  Msp430Usart0C__0__CLIENT_ID = 0U
};
enum /*CC2420ControlC.SyncSpiC*/CC2420SpiC__1____nesc_unnamed4372 {
  CC2420SpiC__1__CLIENT_ID = 1U
};
enum /*CC2420ControlC.RssiResource*/CC2420SpiC__2____nesc_unnamed4373 {
  CC2420SpiC__2__CLIENT_ID = 2U
};
typedef T32khz CC2420TransmitP__PacketTimeStamp__precision_tag;
typedef uint32_t CC2420TransmitP__PacketTimeStamp__size_type;
typedef T32khz CC2420TransmitP__BackoffTimer__precision_tag;
typedef uint32_t CC2420TransmitP__BackoffTimer__size_type;
enum /*CC2420TransmitC.Spi*/CC2420SpiC__3____nesc_unnamed4374 {
  CC2420SpiC__3__CLIENT_ID = 3U
};
typedef T32khz CC2420ReceiveP__PacketTimeStamp__precision_tag;
typedef uint32_t CC2420ReceiveP__PacketTimeStamp__size_type;
typedef T32khz CC2420PacketP__PacketTimeStamp32khz__precision_tag;
typedef uint32_t CC2420PacketP__PacketTimeStamp32khz__size_type;
typedef T32khz CC2420PacketP__LocalTime32khz__precision_tag;
typedef TMilli CC2420PacketP__LocalTimeMilli__precision_tag;
typedef TMilli CC2420PacketP__PacketTimeStampMilli__precision_tag;
typedef uint32_t CC2420PacketP__PacketTimeStampMilli__size_type;
typedef T32khz /*CC2420PacketC.CounterToLocalTimeC*/CounterToLocalTimeC__1__precision_tag;
typedef /*CC2420PacketC.CounterToLocalTimeC*/CounterToLocalTimeC__1__precision_tag /*CC2420PacketC.CounterToLocalTimeC*/CounterToLocalTimeC__1__LocalTime__precision_tag;
typedef /*CC2420PacketC.CounterToLocalTimeC*/CounterToLocalTimeC__1__precision_tag /*CC2420PacketC.CounterToLocalTimeC*/CounterToLocalTimeC__1__Counter__precision_tag;
typedef uint32_t /*CC2420PacketC.CounterToLocalTimeC*/CounterToLocalTimeC__1__Counter__size_type;
enum /*CC2420ReceiveC.Spi*/CC2420SpiC__4____nesc_unnamed4375 {
  CC2420SpiC__4__CLIENT_ID = 4U
};
typedef uint16_t RandomMlcgC__SeedInit__parameter;
enum CC2420TinyosNetworkC____nesc_unnamed4376 {
  CC2420TinyosNetworkC__TINYOS_N_NETWORKS = 1U
};
typedef TMilli PacketLinkP__DelayTimer__precision_tag;
typedef send_info_t IPDispatchP__SendInfoPool__t;
typedef send_entry_t *IPDispatchP__SendQueue__t;
typedef TMilli IPDispatchP__ExpireTimer__precision_tag;
typedef message_t IPDispatchP__FragPool__t;
typedef ip_statistics_t IPDispatchP__Statistics__stat_str;
typedef send_entry_t IPDispatchP__SendEntryPool__t;
typedef TMilli IPRoutingP__SortTimer__precision_tag;
typedef route_statistics_t IPRoutingP__Statistics__stat_str;
typedef TMilli IPRoutingP__TrafficGenTimer__precision_tag;
typedef message_t /*IPDispatchC.FragPool*/PoolC__0__pool_t;
typedef /*IPDispatchC.FragPool*/PoolC__0__pool_t /*IPDispatchC.FragPool.PoolP*/PoolP__0__pool_t;
typedef /*IPDispatchC.FragPool.PoolP*/PoolP__0__pool_t /*IPDispatchC.FragPool.PoolP*/PoolP__0__Pool__t;
typedef send_entry_t /*IPDispatchC.SendEntryPool*/PoolC__1__pool_t;
typedef /*IPDispatchC.SendEntryPool*/PoolC__1__pool_t /*IPDispatchC.SendEntryPool.PoolP*/PoolP__1__pool_t;
typedef /*IPDispatchC.SendEntryPool.PoolP*/PoolP__1__pool_t /*IPDispatchC.SendEntryPool.PoolP*/PoolP__1__Pool__t;
typedef send_entry_t */*IPDispatchC.QueueC*/QueueC__0__queue_t;
typedef /*IPDispatchC.QueueC*/QueueC__0__queue_t /*IPDispatchC.QueueC*/QueueC__0__Queue__t;
typedef send_info_t /*IPDispatchC.SendInfoPool*/PoolC__2__pool_t;
typedef /*IPDispatchC.SendInfoPool*/PoolC__2__pool_t /*IPDispatchC.SendInfoPool.PoolP*/PoolP__2__pool_t;
typedef /*IPDispatchC.SendInfoPool.PoolP*/PoolP__2__pool_t /*IPDispatchC.SendInfoPool.PoolP*/PoolP__2__Pool__t;
typedef TMilli ICMPResponderP__PingTimer__precision_tag;
typedef TMilli ICMPResponderP__LocalTime__precision_tag;
typedef TMilli ICMPResponderP__Advertisement__precision_tag;
typedef icmp_statistics_t ICMPResponderP__Statistics__stat_str;
typedef TMilli ICMPResponderP__Solicitation__precision_tag;
typedef udp_statistics_t UdpP__Statistics__stat_str;
typedef TMilli TcpP__Timer__precision_tag;
typedef TMilli HttpdP__TimerTemp__precision_tag;
typedef uint16_t HttpdP__ReadTemp__val_t;
typedef uint16_t /*TCPEchoC.Sht11C.SensirionSht11ReaderP*/SensirionSht11ReaderP__0__Humidity__val_t;
typedef uint16_t /*TCPEchoC.Sht11C.SensirionSht11ReaderP*/SensirionSht11ReaderP__0__Temperature__val_t;
typedef TMilli /*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP__0__Timer__precision_tag;
typedef TMilli HplSensirionSht11P__Timer__precision_tag;
enum /*TCPEchoC.Sht11C*/SensirionSht11C__0____nesc_unnamed4377 {
  SensirionSht11C__0__TEMP_KEY = 0U
};
enum /*TCPEchoC.Sht11C*/SensirionSht11C__0____nesc_unnamed4378 {
  SensirionSht11C__0__HUM_KEY = 1U
};
typedef TMilli UDPShellP__Uptime__precision_tag;
typedef uint32_t UDPShellP__Uptime__size_type;
# 51 "/home/tinyos/local/src/tinyos-2.x/tos/interfaces/Init.nc"
static error_t PlatformP__Init__init(void );
#line 51
static error_t MotePlatformC__Init__init(void );
# 35 "/home/tinyos/local/src/tinyos-2.x/tos/chips/msp430/timer/Msp430ClockInit.nc"
static void Msp430ClockP__Msp430ClockInit__defaultInitClocks(void );
#line 32
static void Msp430ClockP__Msp430ClockInit__default__initTimerB(void );



static void Msp430ClockP__Msp430ClockInit__defaultInitTimerA(void );
#line 31
static void Msp430ClockP__Msp430ClockInit__default__initTimerA(void );





static void Msp430ClockP__Msp430ClockInit__defaultInitTimerB(void );
#line 34
static void Msp430ClockP__Msp430ClockInit__defaultSetupDcoCalibrate(void );
#line 29
static void Msp430ClockP__Msp430ClockInit__default__setupDcoCalibrate(void );
static void Msp430ClockP__Msp430ClockInit__default__initClocks(void );
# 54 "/home/tinyos/local/src/tinyos-2.x/tos/interfaces/McuPowerOverride.nc"
static mcu_power_t Msp430ClockP__McuPowerOverride__lowestState(void );
# 51 "/home/tinyos/local/src/tinyos-2.x/tos/interfaces/Init.nc"
static error_t Msp430ClockP__Init__init(void );
# 28 "/home/tinyos/local/src/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerEvent.nc"
static void /*Msp430TimerC.Msp430TimerA*/Msp430TimerP__0__VectorTimerX0__fired(void );
#line 28
static void /*Msp430TimerC.Msp430TimerA*/Msp430TimerP__0__Overflow__fired(void );
#line 28
static void /*Msp430TimerC.Msp430TimerA*/Msp430TimerP__0__VectorTimerX1__fired(void );
#line 28
static void /*Msp430TimerC.Msp430TimerA*/Msp430TimerP__0__Event__default__fired(
# 40 "/home/tinyos/local/src/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerP.nc"
uint8_t arg_0x406403c8);
# 28 "/home/tinyos/local/src/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerEvent.nc"
static void /*Msp430TimerC.Msp430TimerB*/Msp430TimerP__1__VectorTimerX0__fired(void );
#line 28
static void /*Msp430TimerC.Msp430TimerB*/Msp430TimerP__1__Overflow__fired(void );
#line 28
static void /*Msp430TimerC.Msp430TimerB*/Msp430TimerP__1__VectorTimerX1__fired(void );
#line 28
static void /*Msp430TimerC.Msp430TimerB*/Msp430TimerP__1__Event__default__fired(
# 40 "/home/tinyos/local/src/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerP.nc"
uint8_t arg_0x406403c8);
# 34 "/home/tinyos/local/src/tinyos-2.x/tos/chips/msp430/timer/Msp430Timer.nc"
static uint16_t /*Msp430TimerC.Msp430TimerB*/Msp430TimerP__1__Timer__get(void );
static bool /*Msp430TimerC.Msp430TimerB*/Msp430TimerP__1__Timer__isOverflowPending(void );
# 33 "/home/tinyos/local/src/tinyos-2.x/tos/chips/msp430/timer/Msp430Capture.nc"
static uint16_t /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0__Capture__getEvent(void );
#line 75
static void /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0__Capture__default__captured(uint16_t time);
# 31 "/home/tinyos/local/src/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerControl.nc"
static msp430_compare_control_t /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0__Control__getControl(void );
# 28 "/home/tinyos/local/src/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerEvent.nc"
static void /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0__Event__fired(void );
# 34 "/home/tinyos/local/src/tinyos-2.x/tos/chips/msp430/timer/Msp430Compare.nc"
static void /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0__Compare__default__fired(void );
# 37 "/home/tinyos/local/src/tinyos-2.x/tos/chips/msp430/timer/Msp430Timer.nc"
static void /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0__Timer__overflow(void );
# 33 "/home/tinyos/local/src/tinyos-2.x/tos/chips/msp430/timer/Msp430Capture.nc"
static uint16_t /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP__1__Capture__getEvent(void );
#line 75
static void /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP__1__Capture__default__captured(uint16_t time);
# 31 "/home/tinyos/local/src/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerControl.nc"
static msp430_compare_control_t /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP__1__Control__getControl(void );
# 28 "/home/tinyos/local/src/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerEvent.nc"
static void /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP__1__Event__fired(void );
# 34 "/home/tinyos/local/src/tinyos-2.x/tos/chips/msp430/timer/Msp430Compare.nc"
static void /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP__1__Compare__default__fired(void );
# 37 "/home/tinyos/local/src/tinyos-2.x/tos/chips/msp430/timer/Msp430Timer.nc"
static void /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP__1__Timer__overflow(void );
# 33 "/home/tinyos/local/src/tinyos-2.x/tos/chips/msp430/timer/Msp430Capture.nc"
static uint16_t /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP__2__Capture__getEvent(void );
#line 75
static void /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP__2__Capture__default__captured(uint16_t time);
# 31 "/home/tinyos/local/src/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerControl.nc"
static msp430_compare_control_t /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP__2__Control__getControl(void );
# 28 "/home/tinyos/local/src/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerEvent.nc"
static void /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP__2__Event__fired(void );
# 34 "/home/tinyos/local/src/tinyos-2.x/tos/chips/msp430/timer/Msp430Compare.nc"
static void /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP__2__Compare__default__fired(void );
# 37 "/home/tinyos/local/src/tinyos-2.x/tos/chips/msp430/timer/Msp430Timer.nc"
static void /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP__2__Timer__overflow(void );
# 33 "/home/tinyos/local/src/tinyos-2.x/tos/chips/msp430/timer/Msp430Capture.nc"
static uint16_t /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Capture__getEvent(void );
#line 75
static void /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Capture__default__captured(uint16_t time);
# 31 "/home/tinyos/local/src/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerControl.nc"
static msp430_compare_control_t /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Control__getControl(void );
#line 46
static void /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Control__enableEvents(void );
#line 36
static void /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Control__setControlAsCompare(void );










static void /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Control__disableEvents(void );
#line 33
static void /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Control__clearPendingInterrupt(void );
# 28 "/home/tinyos/local/src/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerEvent.nc"
static void /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Event__fired(void );
# 30 "/home/tinyos/local/src/tinyos-2.x/tos/chips/msp430/timer/Msp430Compare.nc"
static void /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Compare__setEvent(uint16_t time);

static void /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Compare__setEventFromNow(uint16_t delta);
# 37 "/home/tinyos/local/src/tinyos-2.x/tos/chips/msp430/timer/Msp430Timer.nc"
static void /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Timer__overflow(void );
# 33 "/home/tinyos/local/src/tinyos-2.x/tos/chips/msp430/timer/Msp430Capture.nc"
static uint16_t /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__Capture__getEvent(void );
#line 57
static void /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__Capture__clearOverflow(void );
# 44 "/home/tinyos/local/src/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerControl.nc"
static void /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__Control__setControlAsCapture(uint8_t cm);
#line 31
static msp430_compare_control_t /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__Control__getControl(void );
#line 46
static void /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__Control__enableEvents(void );
static void /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__Control__disableEvents(void );
#line 33
static void /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__Control__clearPendingInterrupt(void );
# 28 "/home/tinyos/local/src/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerEvent.nc"
static void /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__Event__fired(void );
# 34 "/home/tinyos/local/src/tinyos-2.x/tos/chips/msp430/timer/Msp430Compare.nc"
static void /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__Compare__default__fired(void );
# 37 "/home/tinyos/local/src/tinyos-2.x/tos/chips/msp430/timer/Msp430Timer.nc"
static void /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__Timer__overflow(void );
# 33 "/home/tinyos/local/src/tinyos-2.x/tos/chips/msp430/timer/Msp430Capture.nc"
static uint16_t /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__Capture__getEvent(void );
#line 75
static void /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__Capture__default__captured(uint16_t time);
# 31 "/home/tinyos/local/src/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerControl.nc"
static msp430_compare_control_t /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__Control__getControl(void );
#line 46
static void /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__Control__enableEvents(void );
#line 36
static void /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__Control__setControlAsCompare(void );










static void /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__Control__disableEvents(void );
#line 33
static void /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__Control__clearPendingInterrupt(void );
# 28 "/home/tinyos/local/src/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerEvent.nc"
static void /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__Event__fired(void );
# 30 "/home/tinyos/local/src/tinyos-2.x/tos/chips/msp430/timer/Msp430Compare.nc"
static void /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__Compare__setEvent(uint16_t time);

static void /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__Compare__setEventFromNow(uint16_t delta);
# 37 "/home/tinyos/local/src/tinyos-2.x/tos/chips/msp430/timer/Msp430Timer.nc"
static void /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__Timer__overflow(void );
# 33 "/home/tinyos/local/src/tinyos-2.x/tos/chips/msp430/timer/Msp430Capture.nc"
static uint16_t /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP__6__Capture__getEvent(void );
#line 75
static void /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP__6__Capture__default__captured(uint16_t time);
# 31 "/home/tinyos/local/src/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerControl.nc"
static msp430_compare_control_t /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP__6__Control__getControl(void );
# 28 "/home/tinyos/local/src/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerEvent.nc"
static void /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP__6__Event__fired(void );
# 34 "/home/tinyos/local/src/tinyos-2.x/tos/chips/msp430/timer/Msp430Compare.nc"
static void /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP__6__Compare__default__fired(void );
# 37 "/home/tinyos/local/src/tinyos-2.x/tos/chips/msp430/timer/Msp430Timer.nc"
static void /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP__6__Timer__overflow(void );
# 33 "/home/tinyos/local/src/tinyos-2.x/tos/chips/msp430/timer/Msp430Capture.nc"
static uint16_t /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP__7__Capture__getEvent(void );
#line 75
static void /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP__7__Capture__default__captured(uint16_t time);
# 31 "/home/tinyos/local/src/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerControl.nc"
static msp430_compare_control_t /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP__7__Control__getControl(void );
# 28 "/home/tinyos/local/src/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerEvent.nc"
static void /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP__7__Event__fired(void );
# 34 "/home/tinyos/local/src/tinyos-2.x/tos/chips/msp430/timer/Msp430Compare.nc"
static void /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP__7__Compare__default__fired(void );
# 37 "/home/tinyos/local/src/tinyos-2.x/tos/chips/msp430/timer/Msp430Timer.nc"
static void /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP__7__Timer__overflow(void );
# 33 "/home/tinyos/local/src/tinyos-2.x/tos/chips/msp430/timer/Msp430Capture.nc"
static uint16_t /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP__8__Capture__getEvent(void );
#line 75
static void /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP__8__Capture__default__captured(uint16_t time);
# 31 "/home/tinyos/local/src/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerControl.nc"
static msp430_compare_control_t /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP__8__Control__getControl(void );
# 28 "/home/tinyos/local/src/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerEvent.nc"
static void /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP__8__Event__fired(void );
# 34 "/home/tinyos/local/src/tinyos-2.x/tos/chips/msp430/timer/Msp430Compare.nc"
static void /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP__8__Compare__default__fired(void );
# 37 "/home/tinyos/local/src/tinyos-2.x/tos/chips/msp430/timer/Msp430Timer.nc"
static void /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP__8__Timer__overflow(void );
# 33 "/home/tinyos/local/src/tinyos-2.x/tos/chips/msp430/timer/Msp430Capture.nc"
static uint16_t /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP__9__Capture__getEvent(void );
#line 75
static void /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP__9__Capture__default__captured(uint16_t time);
# 31 "/home/tinyos/local/src/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerControl.nc"
static msp430_compare_control_t /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP__9__Control__getControl(void );
# 28 "/home/tinyos/local/src/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerEvent.nc"
static void /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP__9__Event__fired(void );
# 34 "/home/tinyos/local/src/tinyos-2.x/tos/chips/msp430/timer/Msp430Compare.nc"
static void /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP__9__Compare__default__fired(void );
# 37 "/home/tinyos/local/src/tinyos-2.x/tos/chips/msp430/timer/Msp430Timer.nc"
static void /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP__9__Timer__overflow(void );
# 59 "/home/tinyos/local/src/tinyos-2.x/tos/interfaces/McuSleep.nc"
static void McuSleepC__McuSleep__sleep(void );
# 56 "/home/tinyos/local/src/tinyos-2.x/tos/interfaces/TaskBasic.nc"
static error_t SchedulerBasicP__TaskBasic__postTask(
# 45 "/home/tinyos/local/src/tinyos-2.x/tos/system/SchedulerBasicP.nc"
uint8_t arg_0x4059ab40);
# 64 "/home/tinyos/local/src/tinyos-2.x/tos/interfaces/TaskBasic.nc"
static void SchedulerBasicP__TaskBasic__default__runTask(
# 45 "/home/tinyos/local/src/tinyos-2.x/tos/system/SchedulerBasicP.nc"
uint8_t arg_0x4059ab40);
# 46 "/home/tinyos/local/src/tinyos-2.x/tos/interfaces/Scheduler.nc"
static void SchedulerBasicP__Scheduler__init(void );
#line 61
static void SchedulerBasicP__Scheduler__taskLoop(void );
#line 54
static bool SchedulerBasicP__Scheduler__runNextTask(void );
# 51 "/home/tinyos/local/src/tinyos-2.x/tos/interfaces/Init.nc"
static error_t LedsP__Init__init(void );
# 56 "/home/tinyos/local/src/tinyos-2.x/tos/interfaces/Leds.nc"
static void LedsP__Leds__led0Toggle(void );
#line 72
static void LedsP__Leds__led1Toggle(void );
#line 89
static void LedsP__Leds__led2Toggle(void );
#line 106
static uint8_t LedsP__Leds__get(void );
# 59 "/home/tinyos/local/src/tinyos-2.x/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
static bool /*HplMsp430GeneralIOC.P10*/HplMsp430GeneralIOP__0__IO__get(void );
#line 52
static uint8_t /*HplMsp430GeneralIOC.P10*/HplMsp430GeneralIOP__0__IO__getRaw(void );






static bool /*HplMsp430GeneralIOC.P13*/HplMsp430GeneralIOP__3__IO__get(void );
#line 52
static uint8_t /*HplMsp430GeneralIOC.P13*/HplMsp430GeneralIOP__3__IO__getRaw(void );
#line 64
static void /*HplMsp430GeneralIOC.P14*/HplMsp430GeneralIOP__4__IO__makeInput(void );
#line 59
static bool /*HplMsp430GeneralIOC.P14*/HplMsp430GeneralIOP__4__IO__get(void );
#line 52
static uint8_t /*HplMsp430GeneralIOC.P14*/HplMsp430GeneralIOP__4__IO__getRaw(void );
#line 64
static void /*HplMsp430GeneralIOC.P15*/HplMsp430GeneralIOP__5__IO__makeInput(void );






static void /*HplMsp430GeneralIOC.P15*/HplMsp430GeneralIOP__5__IO__makeOutput(void );
#line 59
static bool /*HplMsp430GeneralIOC.P15*/HplMsp430GeneralIOP__5__IO__get(void );
#line 52
static uint8_t /*HplMsp430GeneralIOC.P15*/HplMsp430GeneralIOP__5__IO__getRaw(void );
#line 34
static void /*HplMsp430GeneralIOC.P15*/HplMsp430GeneralIOP__5__IO__set(void );




static void /*HplMsp430GeneralIOC.P15*/HplMsp430GeneralIOP__5__IO__clr(void );
#line 64
static void /*HplMsp430GeneralIOC.P16*/HplMsp430GeneralIOP__6__IO__makeInput(void );






static void /*HplMsp430GeneralIOC.P16*/HplMsp430GeneralIOP__6__IO__makeOutput(void );
#line 34
static void /*HplMsp430GeneralIOC.P16*/HplMsp430GeneralIOP__6__IO__set(void );




static void /*HplMsp430GeneralIOC.P16*/HplMsp430GeneralIOP__6__IO__clr(void );
#line 71
static void /*HplMsp430GeneralIOC.P17*/HplMsp430GeneralIOP__7__IO__makeOutput(void );
#line 34
static void /*HplMsp430GeneralIOC.P17*/HplMsp430GeneralIOP__7__IO__set(void );




static void /*HplMsp430GeneralIOC.P17*/HplMsp430GeneralIOP__7__IO__clr(void );
#line 85
static void /*HplMsp430GeneralIOC.P31*/HplMsp430GeneralIOP__17__IO__selectIOFunc(void );
#line 78
static void /*HplMsp430GeneralIOC.P31*/HplMsp430GeneralIOP__17__IO__selectModuleFunc(void );






static void /*HplMsp430GeneralIOC.P32*/HplMsp430GeneralIOP__18__IO__selectIOFunc(void );
#line 78
static void /*HplMsp430GeneralIOC.P32*/HplMsp430GeneralIOP__18__IO__selectModuleFunc(void );






static void /*HplMsp430GeneralIOC.P33*/HplMsp430GeneralIOP__19__IO__selectIOFunc(void );
#line 78
static void /*HplMsp430GeneralIOC.P33*/HplMsp430GeneralIOP__19__IO__selectModuleFunc(void );






static void /*HplMsp430GeneralIOC.P34*/HplMsp430GeneralIOP__20__IO__selectIOFunc(void );
#line 85
static void /*HplMsp430GeneralIOC.P35*/HplMsp430GeneralIOP__21__IO__selectIOFunc(void );
#line 64
static void /*HplMsp430GeneralIOC.P41*/HplMsp430GeneralIOP__25__IO__makeInput(void );
#line 59
static bool /*HplMsp430GeneralIOC.P41*/HplMsp430GeneralIOP__25__IO__get(void );
#line 85
static void /*HplMsp430GeneralIOC.P41*/HplMsp430GeneralIOP__25__IO__selectIOFunc(void );
#line 52
static uint8_t /*HplMsp430GeneralIOC.P41*/HplMsp430GeneralIOP__25__IO__getRaw(void );
#line 78
static void /*HplMsp430GeneralIOC.P41*/HplMsp430GeneralIOP__25__IO__selectModuleFunc(void );
#line 71
static void /*HplMsp430GeneralIOC.P42*/HplMsp430GeneralIOP__26__IO__makeOutput(void );
#line 34
static void /*HplMsp430GeneralIOC.P42*/HplMsp430GeneralIOP__26__IO__set(void );




static void /*HplMsp430GeneralIOC.P42*/HplMsp430GeneralIOP__26__IO__clr(void );
#line 71
static void /*HplMsp430GeneralIOC.P45*/HplMsp430GeneralIOP__29__IO__makeOutput(void );
#line 34
static void /*HplMsp430GeneralIOC.P45*/HplMsp430GeneralIOP__29__IO__set(void );




static void /*HplMsp430GeneralIOC.P45*/HplMsp430GeneralIOP__29__IO__clr(void );
#line 71
static void /*HplMsp430GeneralIOC.P46*/HplMsp430GeneralIOP__30__IO__makeOutput(void );
#line 34
static void /*HplMsp430GeneralIOC.P46*/HplMsp430GeneralIOP__30__IO__set(void );




static void /*HplMsp430GeneralIOC.P46*/HplMsp430GeneralIOP__30__IO__clr(void );




static void /*HplMsp430GeneralIOC.P54*/HplMsp430GeneralIOP__36__IO__toggle(void );
#line 71
static void /*HplMsp430GeneralIOC.P54*/HplMsp430GeneralIOP__36__IO__makeOutput(void );
#line 59
static bool /*HplMsp430GeneralIOC.P54*/HplMsp430GeneralIOP__36__IO__get(void );
#line 52
static uint8_t /*HplMsp430GeneralIOC.P54*/HplMsp430GeneralIOP__36__IO__getRaw(void );
#line 34
static void /*HplMsp430GeneralIOC.P54*/HplMsp430GeneralIOP__36__IO__set(void );









static void /*HplMsp430GeneralIOC.P55*/HplMsp430GeneralIOP__37__IO__toggle(void );
#line 71
static void /*HplMsp430GeneralIOC.P55*/HplMsp430GeneralIOP__37__IO__makeOutput(void );
#line 59
static bool /*HplMsp430GeneralIOC.P55*/HplMsp430GeneralIOP__37__IO__get(void );
#line 52
static uint8_t /*HplMsp430GeneralIOC.P55*/HplMsp430GeneralIOP__37__IO__getRaw(void );
#line 34
static void /*HplMsp430GeneralIOC.P55*/HplMsp430GeneralIOP__37__IO__set(void );









static void /*HplMsp430GeneralIOC.P56*/HplMsp430GeneralIOP__38__IO__toggle(void );
#line 71
static void /*HplMsp430GeneralIOC.P56*/HplMsp430GeneralIOP__38__IO__makeOutput(void );
#line 59
static bool /*HplMsp430GeneralIOC.P56*/HplMsp430GeneralIOP__38__IO__get(void );
#line 52
static uint8_t /*HplMsp430GeneralIOC.P56*/HplMsp430GeneralIOP__38__IO__getRaw(void );
#line 34
static void /*HplMsp430GeneralIOC.P56*/HplMsp430GeneralIOP__38__IO__set(void );
# 31 "/home/tinyos/local/src/tinyos-2.x/tos/interfaces/GeneralIO.nc"
static void /*PlatformLedsC.Led0Impl*/Msp430GpioC__0__GeneralIO__toggle(void );
static bool /*PlatformLedsC.Led0Impl*/Msp430GpioC__0__GeneralIO__get(void );


static void /*PlatformLedsC.Led0Impl*/Msp430GpioC__0__GeneralIO__makeOutput(void );
#line 29
static void /*PlatformLedsC.Led0Impl*/Msp430GpioC__0__GeneralIO__set(void );

static void /*PlatformLedsC.Led1Impl*/Msp430GpioC__1__GeneralIO__toggle(void );
static bool /*PlatformLedsC.Led1Impl*/Msp430GpioC__1__GeneralIO__get(void );


static void /*PlatformLedsC.Led1Impl*/Msp430GpioC__1__GeneralIO__makeOutput(void );
#line 29
static void /*PlatformLedsC.Led1Impl*/Msp430GpioC__1__GeneralIO__set(void );

static void /*PlatformLedsC.Led2Impl*/Msp430GpioC__2__GeneralIO__toggle(void );
static bool /*PlatformLedsC.Led2Impl*/Msp430GpioC__2__GeneralIO__get(void );


static void /*PlatformLedsC.Led2Impl*/Msp430GpioC__2__GeneralIO__makeOutput(void );
#line 29
static void /*PlatformLedsC.Led2Impl*/Msp430GpioC__2__GeneralIO__set(void );
# 72 "/home/tinyos/local/src/tinyos-2.x/tos/lib/timer/Timer.nc"
static void TCPEchoP__StatusTimer__fired(void );
# 49 "/home/tinyos/local/src/tinyos-2.x/tos/interfaces/Boot.nc"
static void TCPEchoP__Boot__booted(void );
# 24 "/home/tinyos/local/src/tinyos-2.x/tos/lib/net/blip/interfaces/UDP.nc"
static void TCPEchoP__Status__recvfrom(struct sockaddr_in6 *src, void *payload, 
uint16_t len, struct ip_metadata *meta);
# 92 "/home/tinyos/local/src/tinyos-2.x/tos/interfaces/SplitControl.nc"
static void TCPEchoP__RadioControl__startDone(error_t error);
#line 117
static void TCPEchoP__RadioControl__stopDone(error_t error);
# 24 "/home/tinyos/local/src/tinyos-2.x/tos/lib/net/blip/interfaces/UDP.nc"
static void TCPEchoP__Echo__recvfrom(struct sockaddr_in6 *src, void *payload, 
uint16_t len, struct ip_metadata *meta);
# 16 "/home/tinyos/local/src/tinyos-2.x/tos/lib/net/blip/interfaces/Tcp.nc"
static bool TCPEchoP__TcpEcho__accept(struct sockaddr_in6 *from, 
void **tx_buf, int *tx_buf_len);
#line 34
static void TCPEchoP__TcpEcho__recv(void *payload, uint16_t len);
#line 47
static void TCPEchoP__TcpEcho__closed(error_t e);




static void TCPEchoP__TcpEcho__acked(void );
# 34 "/home/tinyos/local/src/tinyos-2.x/tos/chips/msp430/timer/Msp430Compare.nc"
static void /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__0__Msp430Compare__fired(void );
# 37 "/home/tinyos/local/src/tinyos-2.x/tos/chips/msp430/timer/Msp430Timer.nc"
static void /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__0__Msp430Timer__overflow(void );
# 92 "/home/tinyos/local/src/tinyos-2.x/tos/lib/timer/Alarm.nc"
static void /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__0__Alarm__startAt(/*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__0__Alarm__size_type t0, /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__0__Alarm__size_type dt);
#line 62
static void /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__0__Alarm__stop(void );
# 51 "/home/tinyos/local/src/tinyos-2.x/tos/interfaces/Init.nc"
static error_t /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__0__Init__init(void );
# 37 "/home/tinyos/local/src/tinyos-2.x/tos/chips/msp430/timer/Msp430Timer.nc"
static void /*Msp430Counter32khzC.Counter*/Msp430CounterC__0__Msp430Timer__overflow(void );
# 53 "/home/tinyos/local/src/tinyos-2.x/tos/lib/timer/Counter.nc"
static /*Msp430Counter32khzC.Counter*/Msp430CounterC__0__Counter__size_type /*Msp430Counter32khzC.Counter*/Msp430CounterC__0__Counter__get(void );






static bool /*Msp430Counter32khzC.Counter*/Msp430CounterC__0__Counter__isOverflowPending(void );










static void /*CounterMilli32C.Transform*/TransformCounterC__0__CounterFrom__overflow(void );
#line 53
static /*CounterMilli32C.Transform*/TransformCounterC__0__Counter__size_type /*CounterMilli32C.Transform*/TransformCounterC__0__Counter__get(void );
# 98 "/home/tinyos/local/src/tinyos-2.x/tos/lib/timer/Alarm.nc"
static /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__Alarm__size_type /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__Alarm__getNow(void );
#line 92
static void /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__Alarm__startAt(/*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__Alarm__size_type t0, /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__Alarm__size_type dt);
#line 105
static /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__Alarm__size_type /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__Alarm__getAlarm(void );
#line 62
static void /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__Alarm__stop(void );




static void /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__AlarmFrom__fired(void );
# 71 "/home/tinyos/local/src/tinyos-2.x/tos/lib/timer/Counter.nc"
static void /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__Counter__overflow(void );
# 64 "/home/tinyos/local/src/tinyos-2.x/tos/interfaces/TaskBasic.nc"
static void /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__fired__runTask(void );
# 67 "/home/tinyos/local/src/tinyos-2.x/tos/lib/timer/Alarm.nc"
static void /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__Alarm__fired(void );
# 125 "/home/tinyos/local/src/tinyos-2.x/tos/lib/timer/Timer.nc"
static uint32_t /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__Timer__getNow(void );
#line 118
static void /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__Timer__startOneShotAt(uint32_t t0, uint32_t dt);
#line 67
static void /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__Timer__stop(void );
# 64 "/home/tinyos/local/src/tinyos-2.x/tos/interfaces/TaskBasic.nc"
static void /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__updateFromTimer__runTask(void );
# 72 "/home/tinyos/local/src/tinyos-2.x/tos/lib/timer/Timer.nc"
static void /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__TimerFrom__fired(void );
#line 72
static void /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__Timer__default__fired(
# 37 "/home/tinyos/local/src/tinyos-2.x/tos/lib/timer/VirtualizeTimerC.nc"
uint8_t arg_0x409729f0);
# 81 "/home/tinyos/local/src/tinyos-2.x/tos/lib/timer/Timer.nc"
static bool /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__Timer__isRunning(
# 37 "/home/tinyos/local/src/tinyos-2.x/tos/lib/timer/VirtualizeTimerC.nc"
uint8_t arg_0x409729f0);
# 53 "/home/tinyos/local/src/tinyos-2.x/tos/lib/timer/Timer.nc"
static void /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__Timer__startPeriodic(
# 37 "/home/tinyos/local/src/tinyos-2.x/tos/lib/timer/VirtualizeTimerC.nc"
uint8_t arg_0x409729f0, 
# 53 "/home/tinyos/local/src/tinyos-2.x/tos/lib/timer/Timer.nc"
uint32_t dt);








static void /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__Timer__startOneShot(
# 37 "/home/tinyos/local/src/tinyos-2.x/tos/lib/timer/VirtualizeTimerC.nc"
uint8_t arg_0x409729f0, 
# 62 "/home/tinyos/local/src/tinyos-2.x/tos/lib/timer/Timer.nc"
uint32_t dt);




static void /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__Timer__stop(
# 37 "/home/tinyos/local/src/tinyos-2.x/tos/lib/timer/VirtualizeTimerC.nc"
uint8_t arg_0x409729f0);
# 50 "/home/tinyos/local/src/tinyos-2.x/tos/lib/timer/LocalTime.nc"
static uint32_t /*HilTimerMilliC.CounterToLocalTimeC*/CounterToLocalTimeC__0__LocalTime__get(void );
# 71 "/home/tinyos/local/src/tinyos-2.x/tos/lib/timer/Counter.nc"
static void /*HilTimerMilliC.CounterToLocalTimeC*/CounterToLocalTimeC__0__Counter__overflow(void );
# 83 "/home/tinyos/local/src/tinyos-2.x/tos/interfaces/SplitControl.nc"
static error_t CC2420CsmaP__SplitControl__start(void );
#line 109
static error_t CC2420CsmaP__SplitControl__stop(void );
# 95 "/home/tinyos/local/src/tinyos-2.x/tos/chips/cc2420/interfaces/RadioBackoff.nc"
static void CC2420CsmaP__RadioBackoff__default__requestCca(message_t * msg);
#line 81
static void CC2420CsmaP__RadioBackoff__default__requestInitialBackoff(message_t * msg);






static void CC2420CsmaP__RadioBackoff__default__requestCongestionBackoff(message_t * msg);
#line 81
static void CC2420CsmaP__SubBackoff__requestInitialBackoff(message_t * msg);






static void CC2420CsmaP__SubBackoff__requestCongestionBackoff(message_t * msg);
# 73 "/home/tinyos/local/src/tinyos-2.x/tos/chips/cc2420/interfaces/CC2420Transmit.nc"
static void CC2420CsmaP__CC2420Transmit__sendDone(message_t * p_msg, error_t error);
# 64 "/home/tinyos/local/src/tinyos-2.x/tos/interfaces/Send.nc"
static error_t CC2420CsmaP__Send__send(
#line 56
message_t * msg, 







uint8_t len);
# 76 "/home/tinyos/local/src/tinyos-2.x/tos/chips/cc2420/interfaces/CC2420Power.nc"
static void CC2420CsmaP__CC2420Power__startOscillatorDone(void );
#line 56
static void CC2420CsmaP__CC2420Power__startVRegDone(void );
# 92 "/home/tinyos/local/src/tinyos-2.x/tos/interfaces/Resource.nc"
static void CC2420CsmaP__Resource__granted(void );
# 64 "/home/tinyos/local/src/tinyos-2.x/tos/interfaces/TaskBasic.nc"
static void CC2420CsmaP__sendDone_task__runTask(void );
#line 64
static void CC2420CsmaP__stopDone_task__runTask(void );
#line 64
static void CC2420CsmaP__startDone_task__runTask(void );
# 86 "/home/tinyos/local/src/tinyos-2.x/tos/chips/cc2420/interfaces/CC2420Config.nc"
static bool CC2420ControlP__CC2420Config__isAddressRecognitionEnabled(void );
#line 110
static bool CC2420ControlP__CC2420Config__isAutoAckEnabled(void );
#line 105
static bool CC2420ControlP__CC2420Config__isHwAutoAckDefault(void );
#line 64
static uint16_t CC2420ControlP__CC2420Config__getShortAddr(void );
#line 52
static error_t CC2420ControlP__CC2420Config__sync(void );
#line 70
static uint16_t CC2420ControlP__CC2420Config__getPanAddr(void );
# 67 "/home/tinyos/local/src/tinyos-2.x/tos/lib/timer/Alarm.nc"
static void CC2420ControlP__StartupTimer__fired(void );
# 63 "/home/tinyos/local/src/tinyos-2.x/tos/interfaces/Read.nc"
static void CC2420ControlP__ReadRssi__default__readDone(error_t result, CC2420ControlP__ReadRssi__val_t val);
# 64 "/home/tinyos/local/src/tinyos-2.x/tos/interfaces/TaskBasic.nc"
static void CC2420ControlP__syncDone__runTask(void );
# 51 "/home/tinyos/local/src/tinyos-2.x/tos/interfaces/Init.nc"
static error_t CC2420ControlP__Init__init(void );
# 92 "/home/tinyos/local/src/tinyos-2.x/tos/interfaces/Resource.nc"
static void CC2420ControlP__SpiResource__granted(void );
#line 92
static void CC2420ControlP__SyncResource__granted(void );
# 71 "/home/tinyos/local/src/tinyos-2.x/tos/chips/cc2420/interfaces/CC2420Power.nc"
static error_t CC2420ControlP__CC2420Power__startOscillator(void );
#line 90
static error_t CC2420ControlP__CC2420Power__rxOn(void );
#line 51
static error_t CC2420ControlP__CC2420Power__startVReg(void );
#line 63
static error_t CC2420ControlP__CC2420Power__stopVReg(void );
# 64 "/home/tinyos/local/src/tinyos-2.x/tos/interfaces/TaskBasic.nc"
static void CC2420ControlP__sync__runTask(void );
# 110 "/home/tinyos/local/src/tinyos-2.x/tos/interfaces/Resource.nc"
static error_t CC2420ControlP__Resource__release(void );
#line 78
static error_t CC2420ControlP__Resource__request(void );
# 57 "/home/tinyos/local/src/tinyos-2.x/tos/interfaces/GpioInterrupt.nc"
static void CC2420ControlP__InterruptCCA__fired(void );
# 92 "/home/tinyos/local/src/tinyos-2.x/tos/interfaces/Resource.nc"
static void CC2420ControlP__RssiResource__granted(void );
# 34 "/home/tinyos/local/src/tinyos-2.x/tos/chips/msp430/timer/Msp430Compare.nc"
static void /*AlarmMultiplexC.Alarm.Alarm32khz32C.AlarmC.Msp430Alarm*/Msp430AlarmC__1__Msp430Compare__fired(void );
# 37 "/home/tinyos/local/src/tinyos-2.x/tos/chips/msp430/timer/Msp430Timer.nc"
static void /*AlarmMultiplexC.Alarm.Alarm32khz32C.AlarmC.Msp430Alarm*/Msp430AlarmC__1__Msp430Timer__overflow(void );
# 92 "/home/tinyos/local/src/tinyos-2.x/tos/lib/timer/Alarm.nc"
static void /*AlarmMultiplexC.Alarm.Alarm32khz32C.AlarmC.Msp430Alarm*/Msp430AlarmC__1__Alarm__startAt(/*AlarmMultiplexC.Alarm.Alarm32khz32C.AlarmC.Msp430Alarm*/Msp430AlarmC__1__Alarm__size_type t0, /*AlarmMultiplexC.Alarm.Alarm32khz32C.AlarmC.Msp430Alarm*/Msp430AlarmC__1__Alarm__size_type dt);
#line 62
static void /*AlarmMultiplexC.Alarm.Alarm32khz32C.AlarmC.Msp430Alarm*/Msp430AlarmC__1__Alarm__stop(void );
# 51 "/home/tinyos/local/src/tinyos-2.x/tos/interfaces/Init.nc"
static error_t /*AlarmMultiplexC.Alarm.Alarm32khz32C.AlarmC.Msp430Alarm*/Msp430AlarmC__1__Init__init(void );
# 71 "/home/tinyos/local/src/tinyos-2.x/tos/lib/timer/Counter.nc"
static void /*Counter32khz32C.Transform*/TransformCounterC__1__CounterFrom__overflow(void );
#line 53
static /*Counter32khz32C.Transform*/TransformCounterC__1__Counter__size_type /*Counter32khz32C.Transform*/TransformCounterC__1__Counter__get(void );
# 98 "/home/tinyos/local/src/tinyos-2.x/tos/lib/timer/Alarm.nc"
static /*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC__1__Alarm__size_type /*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC__1__Alarm__getNow(void );
#line 92
static void /*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC__1__Alarm__startAt(/*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC__1__Alarm__size_type t0, /*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC__1__Alarm__size_type dt);
#line 55
static void /*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC__1__Alarm__start(/*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC__1__Alarm__size_type dt);






static void /*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC__1__Alarm__stop(void );




static void /*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC__1__AlarmFrom__fired(void );
# 71 "/home/tinyos/local/src/tinyos-2.x/tos/lib/timer/Counter.nc"
static void /*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC__1__Counter__overflow(void );
# 33 "/home/tinyos/local/src/tinyos-2.x/tos/interfaces/GeneralIO.nc"
static void /*HplCC2420PinsC.CCAM*/Msp430GpioC__3__GeneralIO__makeInput(void );
#line 32
static bool /*HplCC2420PinsC.CCAM*/Msp430GpioC__3__GeneralIO__get(void );


static void /*HplCC2420PinsC.CSNM*/Msp430GpioC__4__GeneralIO__makeOutput(void );
#line 29
static void /*HplCC2420PinsC.CSNM*/Msp430GpioC__4__GeneralIO__set(void );
static void /*HplCC2420PinsC.CSNM*/Msp430GpioC__4__GeneralIO__clr(void );

static bool /*HplCC2420PinsC.FIFOM*/Msp430GpioC__5__GeneralIO__get(void );
#line 32
static bool /*HplCC2420PinsC.FIFOPM*/Msp430GpioC__6__GeneralIO__get(void );


static void /*HplCC2420PinsC.RSTNM*/Msp430GpioC__7__GeneralIO__makeOutput(void );
#line 29
static void /*HplCC2420PinsC.RSTNM*/Msp430GpioC__7__GeneralIO__set(void );
static void /*HplCC2420PinsC.RSTNM*/Msp430GpioC__7__GeneralIO__clr(void );


static void /*HplCC2420PinsC.SFDM*/Msp430GpioC__8__GeneralIO__makeInput(void );
#line 32
static bool /*HplCC2420PinsC.SFDM*/Msp430GpioC__8__GeneralIO__get(void );


static void /*HplCC2420PinsC.VRENM*/Msp430GpioC__9__GeneralIO__makeOutput(void );
#line 29
static void /*HplCC2420PinsC.VRENM*/Msp430GpioC__9__GeneralIO__set(void );
static void /*HplCC2420PinsC.VRENM*/Msp430GpioC__9__GeneralIO__clr(void );
# 75 "/home/tinyos/local/src/tinyos-2.x/tos/chips/msp430/timer/Msp430Capture.nc"
static void /*HplCC2420InterruptsC.CaptureSFDC*/GpioCaptureC__0__Msp430Capture__captured(uint16_t time);
# 43 "/home/tinyos/local/src/tinyos-2.x/tos/interfaces/GpioCapture.nc"
static error_t /*HplCC2420InterruptsC.CaptureSFDC*/GpioCaptureC__0__Capture__captureFallingEdge(void );
#line 55
static void /*HplCC2420InterruptsC.CaptureSFDC*/GpioCaptureC__0__Capture__disable(void );
#line 42
static error_t /*HplCC2420InterruptsC.CaptureSFDC*/GpioCaptureC__0__Capture__captureRisingEdge(void );
# 41 "/home/tinyos/local/src/tinyos-2.x/tos/chips/msp430/pins/HplMsp430Interrupt.nc"
static void HplMsp430InterruptP__Port14__clear(void );
#line 36
static void HplMsp430InterruptP__Port14__disable(void );
#line 56
static void HplMsp430InterruptP__Port14__edge(bool low_to_high);
#line 31
static void HplMsp430InterruptP__Port14__enable(void );









static void HplMsp430InterruptP__Port26__clear(void );
#line 61
static void HplMsp430InterruptP__Port26__default__fired(void );
#line 41
static void HplMsp430InterruptP__Port17__clear(void );
#line 61
static void HplMsp430InterruptP__Port17__default__fired(void );
#line 41
static void HplMsp430InterruptP__Port21__clear(void );
#line 61
static void HplMsp430InterruptP__Port21__default__fired(void );
#line 41
static void HplMsp430InterruptP__Port12__clear(void );
#line 61
static void HplMsp430InterruptP__Port12__default__fired(void );
#line 41
static void HplMsp430InterruptP__Port24__clear(void );
#line 61
static void HplMsp430InterruptP__Port24__default__fired(void );
#line 41
static void HplMsp430InterruptP__Port15__clear(void );
#line 36
static void HplMsp430InterruptP__Port15__disable(void );
#line 56
static void HplMsp430InterruptP__Port15__edge(bool low_to_high);
#line 31
static void HplMsp430InterruptP__Port15__enable(void );









static void HplMsp430InterruptP__Port27__clear(void );
#line 61
static void HplMsp430InterruptP__Port27__default__fired(void );
#line 41
static void HplMsp430InterruptP__Port10__clear(void );
#line 36
static void HplMsp430InterruptP__Port10__disable(void );
#line 56
static void HplMsp430InterruptP__Port10__edge(bool low_to_high);
#line 31
static void HplMsp430InterruptP__Port10__enable(void );









static void HplMsp430InterruptP__Port22__clear(void );
#line 61
static void HplMsp430InterruptP__Port22__default__fired(void );
#line 41
static void HplMsp430InterruptP__Port13__clear(void );
#line 61
static void HplMsp430InterruptP__Port13__default__fired(void );
#line 41
static void HplMsp430InterruptP__Port25__clear(void );
#line 61
static void HplMsp430InterruptP__Port25__default__fired(void );
#line 41
static void HplMsp430InterruptP__Port16__clear(void );
#line 61
static void HplMsp430InterruptP__Port16__default__fired(void );
#line 41
static void HplMsp430InterruptP__Port20__clear(void );
#line 61
static void HplMsp430InterruptP__Port20__default__fired(void );
#line 41
static void HplMsp430InterruptP__Port11__clear(void );
#line 61
static void HplMsp430InterruptP__Port11__default__fired(void );
#line 41
static void HplMsp430InterruptP__Port23__clear(void );
#line 61
static void HplMsp430InterruptP__Port23__default__fired(void );
#line 61
static void /*HplCC2420InterruptsC.InterruptCCAC*/Msp430InterruptC__0__HplInterrupt__fired(void );
# 50 "/home/tinyos/local/src/tinyos-2.x/tos/interfaces/GpioInterrupt.nc"
static error_t /*HplCC2420InterruptsC.InterruptCCAC*/Msp430InterruptC__0__Interrupt__disable(void );
#line 42
static error_t /*HplCC2420InterruptsC.InterruptCCAC*/Msp430InterruptC__0__Interrupt__enableRisingEdge(void );
# 61 "/home/tinyos/local/src/tinyos-2.x/tos/chips/msp430/pins/HplMsp430Interrupt.nc"
static void /*HplCC2420InterruptsC.InterruptFIFOPC*/Msp430InterruptC__1__HplInterrupt__fired(void );
# 50 "/home/tinyos/local/src/tinyos-2.x/tos/interfaces/GpioInterrupt.nc"
static error_t /*HplCC2420InterruptsC.InterruptFIFOPC*/Msp430InterruptC__1__Interrupt__disable(void );
#line 43
static error_t /*HplCC2420InterruptsC.InterruptFIFOPC*/Msp430InterruptC__1__Interrupt__enableFallingEdge(void );
# 71 "/home/tinyos/local/src/tinyos-2.x/tos/interfaces/SpiPacket.nc"
static void CC2420SpiP__SpiPacket__sendDone(
#line 64
uint8_t * txBuf, 
uint8_t * rxBuf, 





uint16_t len, 
error_t error);
# 62 "/home/tinyos/local/src/tinyos-2.x/tos/chips/cc2420/interfaces/CC2420Fifo.nc"
static error_t CC2420SpiP__Fifo__continueRead(
# 46 "/home/tinyos/local/src/tinyos-2.x/tos/chips/cc2420/spi/CC2420SpiP.nc"
uint8_t arg_0x40bbb010, 
# 62 "/home/tinyos/local/src/tinyos-2.x/tos/chips/cc2420/interfaces/CC2420Fifo.nc"
uint8_t * data, uint8_t length);
#line 91
static void CC2420SpiP__Fifo__default__writeDone(
# 46 "/home/tinyos/local/src/tinyos-2.x/tos/chips/cc2420/spi/CC2420SpiP.nc"
uint8_t arg_0x40bbb010, 
# 91 "/home/tinyos/local/src/tinyos-2.x/tos/chips/cc2420/interfaces/CC2420Fifo.nc"
uint8_t * data, uint8_t length, error_t error);
#line 82
static cc2420_status_t CC2420SpiP__Fifo__write(
# 46 "/home/tinyos/local/src/tinyos-2.x/tos/chips/cc2420/spi/CC2420SpiP.nc"
uint8_t arg_0x40bbb010, 
# 82 "/home/tinyos/local/src/tinyos-2.x/tos/chips/cc2420/interfaces/CC2420Fifo.nc"
uint8_t * data, uint8_t length);
#line 51
static cc2420_status_t CC2420SpiP__Fifo__beginRead(
# 46 "/home/tinyos/local/src/tinyos-2.x/tos/chips/cc2420/spi/CC2420SpiP.nc"
uint8_t arg_0x40bbb010, 
# 51 "/home/tinyos/local/src/tinyos-2.x/tos/chips/cc2420/interfaces/CC2420Fifo.nc"
uint8_t * data, uint8_t length);
#line 71
static void CC2420SpiP__Fifo__default__readDone(
# 46 "/home/tinyos/local/src/tinyos-2.x/tos/chips/cc2420/spi/CC2420SpiP.nc"
uint8_t arg_0x40bbb010, 
# 71 "/home/tinyos/local/src/tinyos-2.x/tos/chips/cc2420/interfaces/CC2420Fifo.nc"
uint8_t * data, uint8_t length, error_t error);
# 31 "/home/tinyos/local/src/tinyos-2.x/tos/chips/cc2420/interfaces/ChipSpiResource.nc"
static void CC2420SpiP__ChipSpiResource__abortRelease(void );







static error_t CC2420SpiP__ChipSpiResource__attemptRelease(void );
# 92 "/home/tinyos/local/src/tinyos-2.x/tos/interfaces/Resource.nc"
static void CC2420SpiP__SpiResource__granted(void );
# 63 "/home/tinyos/local/src/tinyos-2.x/tos/chips/cc2420/interfaces/CC2420Ram.nc"
static cc2420_status_t CC2420SpiP__Ram__write(
# 47 "/home/tinyos/local/src/tinyos-2.x/tos/chips/cc2420/spi/CC2420SpiP.nc"
uint16_t arg_0x40bbba38, 
# 63 "/home/tinyos/local/src/tinyos-2.x/tos/chips/cc2420/interfaces/CC2420Ram.nc"
uint8_t offset, uint8_t * data, uint8_t length);
# 47 "/home/tinyos/local/src/tinyos-2.x/tos/chips/cc2420/interfaces/CC2420Register.nc"
static cc2420_status_t CC2420SpiP__Reg__read(
# 48 "/home/tinyos/local/src/tinyos-2.x/tos/chips/cc2420/spi/CC2420SpiP.nc"
uint8_t arg_0x40bba200, 
# 47 "/home/tinyos/local/src/tinyos-2.x/tos/chips/cc2420/interfaces/CC2420Register.nc"
uint16_t *data);







static cc2420_status_t CC2420SpiP__Reg__write(
# 48 "/home/tinyos/local/src/tinyos-2.x/tos/chips/cc2420/spi/CC2420SpiP.nc"
uint8_t arg_0x40bba200, 
# 55 "/home/tinyos/local/src/tinyos-2.x/tos/chips/cc2420/interfaces/CC2420Register.nc"
uint16_t data);
# 110 "/home/tinyos/local/src/tinyos-2.x/tos/interfaces/Resource.nc"
static error_t CC2420SpiP__Resource__release(
# 45 "/home/tinyos/local/src/tinyos-2.x/tos/chips/cc2420/spi/CC2420SpiP.nc"
uint8_t arg_0x40bbc5c8);
# 87 "/home/tinyos/local/src/tinyos-2.x/tos/interfaces/Resource.nc"
static error_t CC2420SpiP__Resource__immediateRequest(
# 45 "/home/tinyos/local/src/tinyos-2.x/tos/chips/cc2420/spi/CC2420SpiP.nc"
uint8_t arg_0x40bbc5c8);
# 78 "/home/tinyos/local/src/tinyos-2.x/tos/interfaces/Resource.nc"
static error_t CC2420SpiP__Resource__request(
# 45 "/home/tinyos/local/src/tinyos-2.x/tos/chips/cc2420/spi/CC2420SpiP.nc"
uint8_t arg_0x40bbc5c8);
# 92 "/home/tinyos/local/src/tinyos-2.x/tos/interfaces/Resource.nc"
static void CC2420SpiP__Resource__default__granted(
# 45 "/home/tinyos/local/src/tinyos-2.x/tos/chips/cc2420/spi/CC2420SpiP.nc"
uint8_t arg_0x40bbc5c8);
# 118 "/home/tinyos/local/src/tinyos-2.x/tos/interfaces/Resource.nc"
static bool CC2420SpiP__Resource__isOwner(
# 45 "/home/tinyos/local/src/tinyos-2.x/tos/chips/cc2420/spi/CC2420SpiP.nc"
uint8_t arg_0x40bbc5c8);
# 64 "/home/tinyos/local/src/tinyos-2.x/tos/interfaces/TaskBasic.nc"
static void CC2420SpiP__grant__runTask(void );
# 45 "/home/tinyos/local/src/tinyos-2.x/tos/chips/cc2420/interfaces/CC2420Strobe.nc"
static cc2420_status_t CC2420SpiP__Strobe__strobe(
# 49 "/home/tinyos/local/src/tinyos-2.x/tos/chips/cc2420/spi/CC2420SpiP.nc"
uint8_t arg_0x40bba9b8);
# 51 "/home/tinyos/local/src/tinyos-2.x/tos/interfaces/Init.nc"
static error_t StateImplP__Init__init(void );
# 71 "/home/tinyos/local/src/tinyos-2.x/tos/interfaces/State.nc"
static uint8_t StateImplP__State__getState(
# 67 "/home/tinyos/local/src/tinyos-2.x/tos/system/StateImplP.nc"
uint8_t arg_0x40c1fd18);
# 56 "/home/tinyos/local/src/tinyos-2.x/tos/interfaces/State.nc"
static void StateImplP__State__toIdle(
# 67 "/home/tinyos/local/src/tinyos-2.x/tos/system/StateImplP.nc"
uint8_t arg_0x40c1fd18);
# 66 "/home/tinyos/local/src/tinyos-2.x/tos/interfaces/State.nc"
static bool StateImplP__State__isState(
# 67 "/home/tinyos/local/src/tinyos-2.x/tos/system/StateImplP.nc"
uint8_t arg_0x40c1fd18, 
# 66 "/home/tinyos/local/src/tinyos-2.x/tos/interfaces/State.nc"
uint8_t myState);
#line 61
static bool StateImplP__State__isIdle(
# 67 "/home/tinyos/local/src/tinyos-2.x/tos/system/StateImplP.nc"
uint8_t arg_0x40c1fd18);
# 45 "/home/tinyos/local/src/tinyos-2.x/tos/interfaces/State.nc"
static error_t StateImplP__State__requestState(
# 67 "/home/tinyos/local/src/tinyos-2.x/tos/system/StateImplP.nc"
uint8_t arg_0x40c1fd18, 
# 45 "/home/tinyos/local/src/tinyos-2.x/tos/interfaces/State.nc"
uint8_t reqState);





static void StateImplP__State__forceState(
# 67 "/home/tinyos/local/src/tinyos-2.x/tos/system/StateImplP.nc"
uint8_t arg_0x40c1fd18, 
# 51 "/home/tinyos/local/src/tinyos-2.x/tos/interfaces/State.nc"
uint8_t reqState);
# 55 "/home/tinyos/local/src/tinyos-2.x/tos/interfaces/ResourceConfigure.nc"
static void /*Msp430SpiDma0P.SpiP*/Msp430SpiDmaP__0__ResourceConfigure__unconfigure(
# 49 "/home/tinyos/local/src/tinyos-2.x/tos/chips/msp430/usart/Msp430SpiDmaP.nc"
uint8_t arg_0x40c70c00);
# 49 "/home/tinyos/local/src/tinyos-2.x/tos/interfaces/ResourceConfigure.nc"
static void /*Msp430SpiDma0P.SpiP*/Msp430SpiDmaP__0__ResourceConfigure__configure(
# 49 "/home/tinyos/local/src/tinyos-2.x/tos/chips/msp430/usart/Msp430SpiDmaP.nc"
uint8_t arg_0x40c70c00);
# 59 "/home/tinyos/local/src/tinyos-2.x/tos/interfaces/SpiPacket.nc"
static error_t /*Msp430SpiDma0P.SpiP*/Msp430SpiDmaP__0__SpiPacket__send(
# 51 "/home/tinyos/local/src/tinyos-2.x/tos/chips/msp430/usart/Msp430SpiDmaP.nc"
uint8_t arg_0x40c6e740, 
# 48 "/home/tinyos/local/src/tinyos-2.x/tos/interfaces/SpiPacket.nc"
uint8_t * txBuf, 

uint8_t * rxBuf, 








uint16_t len);
#line 71
static void /*Msp430SpiDma0P.SpiP*/Msp430SpiDmaP__0__SpiPacket__default__sendDone(
# 51 "/home/tinyos/local/src/tinyos-2.x/tos/chips/msp430/usart/Msp430SpiDmaP.nc"
uint8_t arg_0x40c6e740, 
# 64 "/home/tinyos/local/src/tinyos-2.x/tos/interfaces/SpiPacket.nc"
uint8_t * txBuf, 
uint8_t * rxBuf, 





uint16_t len, 
error_t error);
# 39 "/home/tinyos/local/src/tinyos-2.x/tos/chips/msp430/usart/Msp430SpiConfigure.nc"
static msp430_spi_union_config_t */*Msp430SpiDma0P.SpiP*/Msp430SpiDmaP__0__Msp430SpiConfigure__default__getConfig(
# 56 "/home/tinyos/local/src/tinyos-2.x/tos/chips/msp430/usart/Msp430SpiDmaP.nc"
uint8_t arg_0x40c6c418);
# 34 "/home/tinyos/local/src/tinyos-2.x/tos/interfaces/SpiByte.nc"
static uint8_t /*Msp430SpiDma0P.SpiP*/Msp430SpiDmaP__0__SpiByte__write(uint8_t tx);
# 96 "/home/tinyos/local/src/tinyos-2.x/tos/chips/msp430/dma/Msp430DmaChannel.nc"
static void /*Msp430SpiDma0P.SpiP*/Msp430SpiDmaP__0__DmaChannel1__transferDone(error_t success);
# 110 "/home/tinyos/local/src/tinyos-2.x/tos/interfaces/Resource.nc"
static error_t /*Msp430SpiDma0P.SpiP*/Msp430SpiDmaP__0__UsartResource__default__release(
# 55 "/home/tinyos/local/src/tinyos-2.x/tos/chips/msp430/usart/Msp430SpiDmaP.nc"
uint8_t arg_0x40c6d9a0);
# 87 "/home/tinyos/local/src/tinyos-2.x/tos/interfaces/Resource.nc"
static error_t /*Msp430SpiDma0P.SpiP*/Msp430SpiDmaP__0__UsartResource__default__immediateRequest(
# 55 "/home/tinyos/local/src/tinyos-2.x/tos/chips/msp430/usart/Msp430SpiDmaP.nc"
uint8_t arg_0x40c6d9a0);
# 78 "/home/tinyos/local/src/tinyos-2.x/tos/interfaces/Resource.nc"
static error_t /*Msp430SpiDma0P.SpiP*/Msp430SpiDmaP__0__UsartResource__default__request(
# 55 "/home/tinyos/local/src/tinyos-2.x/tos/chips/msp430/usart/Msp430SpiDmaP.nc"
uint8_t arg_0x40c6d9a0);
# 92 "/home/tinyos/local/src/tinyos-2.x/tos/interfaces/Resource.nc"
static void /*Msp430SpiDma0P.SpiP*/Msp430SpiDmaP__0__UsartResource__granted(
# 55 "/home/tinyos/local/src/tinyos-2.x/tos/chips/msp430/usart/Msp430SpiDmaP.nc"
uint8_t arg_0x40c6d9a0);
# 118 "/home/tinyos/local/src/tinyos-2.x/tos/interfaces/Resource.nc"
static bool /*Msp430SpiDma0P.SpiP*/Msp430SpiDmaP__0__UsartResource__default__isOwner(
# 55 "/home/tinyos/local/src/tinyos-2.x/tos/chips/msp430/usart/Msp430SpiDmaP.nc"
uint8_t arg_0x40c6d9a0);
# 110 "/home/tinyos/local/src/tinyos-2.x/tos/interfaces/Resource.nc"
static error_t /*Msp430SpiDma0P.SpiP*/Msp430SpiDmaP__0__Resource__release(
# 48 "/home/tinyos/local/src/tinyos-2.x/tos/chips/msp430/usart/Msp430SpiDmaP.nc"
uint8_t arg_0x40c701b8);
# 87 "/home/tinyos/local/src/tinyos-2.x/tos/interfaces/Resource.nc"
static error_t /*Msp430SpiDma0P.SpiP*/Msp430SpiDmaP__0__Resource__immediateRequest(
# 48 "/home/tinyos/local/src/tinyos-2.x/tos/chips/msp430/usart/Msp430SpiDmaP.nc"
uint8_t arg_0x40c701b8);
# 78 "/home/tinyos/local/src/tinyos-2.x/tos/interfaces/Resource.nc"
static error_t /*Msp430SpiDma0P.SpiP*/Msp430SpiDmaP__0__Resource__request(
# 48 "/home/tinyos/local/src/tinyos-2.x/tos/chips/msp430/usart/Msp430SpiDmaP.nc"
uint8_t arg_0x40c701b8);
# 92 "/home/tinyos/local/src/tinyos-2.x/tos/interfaces/Resource.nc"
static void /*Msp430SpiDma0P.SpiP*/Msp430SpiDmaP__0__Resource__default__granted(
# 48 "/home/tinyos/local/src/tinyos-2.x/tos/chips/msp430/usart/Msp430SpiDmaP.nc"
uint8_t arg_0x40c701b8);
# 118 "/home/tinyos/local/src/tinyos-2.x/tos/interfaces/Resource.nc"
static bool /*Msp430SpiDma0P.SpiP*/Msp430SpiDmaP__0__Resource__isOwner(
# 48 "/home/tinyos/local/src/tinyos-2.x/tos/chips/msp430/usart/Msp430SpiDmaP.nc"
uint8_t arg_0x40c701b8);
# 96 "/home/tinyos/local/src/tinyos-2.x/tos/chips/msp430/dma/Msp430DmaChannel.nc"
static void /*Msp430SpiDma0P.SpiP*/Msp430SpiDmaP__0__DmaChannel2__transferDone(error_t success);
# 54 "/home/tinyos/local/src/tinyos-2.x/tos/chips/msp430/usart/HplMsp430UsartInterrupts.nc"
static void /*Msp430SpiDma0P.SpiP*/Msp430SpiDmaP__0__UsartInterrupts__rxDone(uint8_t data);
#line 49
static void /*Msp430SpiDma0P.SpiP*/Msp430SpiDmaP__0__UsartInterrupts__txDone(void );
# 64 "/home/tinyos/local/src/tinyos-2.x/tos/interfaces/TaskBasic.nc"
static void /*Msp430SpiDma0P.SpiP*/Msp430SpiDmaP__0__signalDone_task__runTask(void );
# 197 "/home/tinyos/local/src/tinyos-2.x/tos/chips/msp430/usart/HplMsp430Usart.nc"
static void HplMsp430Usart0P__Usart__clrRxIntr(void );
#line 97
static void HplMsp430Usart0P__Usart__resetUsart(bool reset);
#line 179
static void HplMsp430Usart0P__Usart__disableIntr(void );
#line 90
static void HplMsp430Usart0P__Usart__setUmctl(uint8_t umctl);
#line 207
static void HplMsp430Usart0P__Usart__clrIntr(void );
#line 80
static void HplMsp430Usart0P__Usart__setUbr(uint16_t ubr);
#line 224
static void HplMsp430Usart0P__Usart__tx(uint8_t data);
#line 128
static void HplMsp430Usart0P__Usart__disableUart(void );
#line 153
static void HplMsp430Usart0P__Usart__enableSpi(void );
#line 168
static void HplMsp430Usart0P__Usart__setModeSpi(msp430_spi_union_config_t *config);
#line 231
static uint8_t HplMsp430Usart0P__Usart__rx(void );
#line 192
static bool HplMsp430Usart0P__Usart__isRxIntrPending(void );
#line 158
static void HplMsp430Usart0P__Usart__disableSpi(void );
# 28 "/home/tinyos/local/src/tinyos-2.x/tos/chips/msp430/dma/HplMsp430DmaInterrupt.nc"
static void /*HplMsp430DmaC.Dma0*/HplMsp430DmaXP__0__Interrupt__fired(void );
# 28 "/home/tinyos/local/src/tinyos-2.x/tos/chips/msp430/dma/HplMsp430DmaChannel.nc"
static error_t /*HplMsp430DmaC.Dma1*/HplMsp430DmaXP__1__DMA__setTrigger(dma_trigger_t trigger);
#line 65
static void /*HplMsp430DmaC.Dma1*/HplMsp430DmaXP__1__DMA__setStateRaw(uint16_t state, uint16_t trigger, void *src, void *dest, uint16_t size);
#line 49
static void /*HplMsp430DmaC.Dma1*/HplMsp430DmaXP__1__DMA__enableDMA(void );
#line 64
static void /*HplMsp430DmaC.Dma1*/HplMsp430DmaXP__1__DMA__setState(dma_channel_state_t s, dma_channel_trigger_t t, void *src, void *dest, uint16_t size);
# 28 "/home/tinyos/local/src/tinyos-2.x/tos/chips/msp430/dma/HplMsp430DmaInterrupt.nc"
static void /*HplMsp430DmaC.Dma1*/HplMsp430DmaXP__1__Interrupt__fired(void );
# 28 "/home/tinyos/local/src/tinyos-2.x/tos/chips/msp430/dma/HplMsp430DmaChannel.nc"
static error_t /*HplMsp430DmaC.Dma2*/HplMsp430DmaXP__2__DMA__setTrigger(dma_trigger_t trigger);
#line 65
static void /*HplMsp430DmaC.Dma2*/HplMsp430DmaXP__2__DMA__setStateRaw(uint16_t state, uint16_t trigger, void *src, void *dest, uint16_t size);
#line 49
static void /*HplMsp430DmaC.Dma2*/HplMsp430DmaXP__2__DMA__enableDMA(void );
#line 64
static void /*HplMsp430DmaC.Dma2*/HplMsp430DmaXP__2__DMA__setState(dma_channel_state_t s, dma_channel_trigger_t t, void *src, void *dest, uint16_t size);
# 28 "/home/tinyos/local/src/tinyos-2.x/tos/chips/msp430/dma/HplMsp430DmaInterrupt.nc"
static void /*HplMsp430DmaC.Dma2*/HplMsp430DmaXP__2__Interrupt__fired(void );
# 74 "/home/tinyos/local/src/tinyos-2.x/tos/chips/msp430/dma/HplMsp430DmaChannel.nc"
static void /*Msp430DmaC.Channel0P*/Msp430DmaChannelP__0__HplChannel__transferDone(error_t success);
# 96 "/home/tinyos/local/src/tinyos-2.x/tos/chips/msp430/dma/Msp430DmaChannel.nc"
static void /*Msp430DmaC.Channel0P*/Msp430DmaChannelP__0__Channel__default__transferDone(error_t success);
# 74 "/home/tinyos/local/src/tinyos-2.x/tos/chips/msp430/dma/HplMsp430DmaChannel.nc"
static void /*Msp430DmaC.Channel1P*/Msp430DmaChannelP__1__HplChannel__transferDone(error_t success);
# 38 "/home/tinyos/local/src/tinyos-2.x/tos/chips/msp430/dma/Msp430DmaChannel.nc"
static error_t /*Msp430DmaC.Channel1P*/Msp430DmaChannelP__1__Channel__setupTransfer(dma_transfer_mode_t transfer_mode, 
dma_trigger_t trigger, 
dma_level_t level, 
void *src_addr, 
void *dst_addr, 
uint16_t size, 
dma_byte_t src_byte, 
dma_byte_t dst_byte, 
dma_incr_t src_incr, 
dma_incr_t dst_incr);
#line 73
static error_t /*Msp430DmaC.Channel1P*/Msp430DmaChannelP__1__Channel__startTransfer(void );
# 74 "/home/tinyos/local/src/tinyos-2.x/tos/chips/msp430/dma/HplMsp430DmaChannel.nc"
static void /*Msp430DmaC.Channel2P*/Msp430DmaChannelP__2__HplChannel__transferDone(error_t success);
# 38 "/home/tinyos/local/src/tinyos-2.x/tos/chips/msp430/dma/Msp430DmaChannel.nc"
static error_t /*Msp430DmaC.Channel2P*/Msp430DmaChannelP__2__Channel__setupTransfer(dma_transfer_mode_t transfer_mode, 
dma_trigger_t trigger, 
dma_level_t level, 
void *src_addr, 
void *dst_addr, 
uint16_t size, 
dma_byte_t src_byte, 
dma_byte_t dst_byte, 
dma_incr_t src_incr, 
dma_incr_t dst_incr);
#line 73
static error_t /*Msp430DmaC.Channel2P*/Msp430DmaChannelP__2__Channel__startTransfer(void );
# 74 "/home/tinyos/local/src/tinyos-2.x/tos/chips/msp430/dma/HplMsp430DmaChannel.nc"
static void Msp430DmaControlP__HplChannel1__transferDone(error_t success);
#line 74
static void Msp430DmaControlP__HplChannel2__transferDone(error_t success);
#line 74
static void Msp430DmaControlP__HplChannel0__transferDone(error_t success);
# 54 "/home/tinyos/local/src/tinyos-2.x/tos/chips/msp430/usart/HplMsp430UsartInterrupts.nc"
static void /*Msp430UsartShare0P.UsartShareP*/Msp430UsartShareP__0__Interrupts__default__rxDone(
# 39 "/home/tinyos/local/src/tinyos-2.x/tos/chips/msp430/usart/Msp430UsartShareP.nc"
uint8_t arg_0x40df4cf8, 
# 54 "/home/tinyos/local/src/tinyos-2.x/tos/chips/msp430/usart/HplMsp430UsartInterrupts.nc"
uint8_t data);
#line 49
static void /*Msp430UsartShare0P.UsartShareP*/Msp430UsartShareP__0__Interrupts__default__txDone(
# 39 "/home/tinyos/local/src/tinyos-2.x/tos/chips/msp430/usart/Msp430UsartShareP.nc"
uint8_t arg_0x40df4cf8);
# 39 "/home/tinyos/local/src/tinyos-2.x/tos/chips/msp430/usart/HplMsp430I2CInterrupts.nc"
static void /*Msp430UsartShare0P.UsartShareP*/Msp430UsartShareP__0__RawI2CInterrupts__fired(void );
#line 39
static void /*Msp430UsartShare0P.UsartShareP*/Msp430UsartShareP__0__I2CInterrupts__default__fired(
# 40 "/home/tinyos/local/src/tinyos-2.x/tos/chips/msp430/usart/Msp430UsartShareP.nc"
uint8_t arg_0x40e245c8);
# 54 "/home/tinyos/local/src/tinyos-2.x/tos/chips/msp430/usart/HplMsp430UsartInterrupts.nc"
static void /*Msp430UsartShare0P.UsartShareP*/Msp430UsartShareP__0__RawInterrupts__rxDone(uint8_t data);
#line 49
static void /*Msp430UsartShare0P.UsartShareP*/Msp430UsartShareP__0__RawInterrupts__txDone(void );
# 51 "/home/tinyos/local/src/tinyos-2.x/tos/interfaces/Init.nc"
static error_t /*Msp430UsartShare0P.ArbiterC.Queue*/FcfsResourceQueueC__1__Init__init(void );
# 69 "/home/tinyos/local/src/tinyos-2.x/tos/interfaces/ResourceQueue.nc"
static error_t /*Msp430UsartShare0P.ArbiterC.Queue*/FcfsResourceQueueC__1__FcfsQueue__enqueue(resource_client_id_t id);
#line 43
static bool /*Msp430UsartShare0P.ArbiterC.Queue*/FcfsResourceQueueC__1__FcfsQueue__isEmpty(void );








static bool /*Msp430UsartShare0P.ArbiterC.Queue*/FcfsResourceQueueC__1__FcfsQueue__isEnqueued(resource_client_id_t id);







static resource_client_id_t /*Msp430UsartShare0P.ArbiterC.Queue*/FcfsResourceQueueC__1__FcfsQueue__dequeue(void );
# 43 "/home/tinyos/local/src/tinyos-2.x/tos/interfaces/ResourceRequested.nc"
static void /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0__ResourceRequested__default__requested(
# 55 "/home/tinyos/local/src/tinyos-2.x/tos/system/ArbiterP.nc"
uint8_t arg_0x40e33d38);
# 51 "/home/tinyos/local/src/tinyos-2.x/tos/interfaces/ResourceRequested.nc"
static void /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0__ResourceRequested__default__immediateRequested(
# 55 "/home/tinyos/local/src/tinyos-2.x/tos/system/ArbiterP.nc"
uint8_t arg_0x40e33d38);
# 55 "/home/tinyos/local/src/tinyos-2.x/tos/interfaces/ResourceConfigure.nc"
static void /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0__ResourceConfigure__default__unconfigure(
# 60 "/home/tinyos/local/src/tinyos-2.x/tos/system/ArbiterP.nc"
uint8_t arg_0x40e31148);
# 49 "/home/tinyos/local/src/tinyos-2.x/tos/interfaces/ResourceConfigure.nc"
static void /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0__ResourceConfigure__default__configure(
# 60 "/home/tinyos/local/src/tinyos-2.x/tos/system/ArbiterP.nc"
uint8_t arg_0x40e31148);
# 56 "/home/tinyos/local/src/tinyos-2.x/tos/interfaces/ResourceDefaultOwner.nc"
static error_t /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0__ResourceDefaultOwner__release(void );
#line 73
static void /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0__ResourceDefaultOwner__default__requested(void );
#line 46
static void /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0__ResourceDefaultOwner__default__granted(void );
#line 81
static void /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0__ResourceDefaultOwner__default__immediateRequested(void );
# 110 "/home/tinyos/local/src/tinyos-2.x/tos/interfaces/Resource.nc"
static error_t /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0__Resource__release(
# 54 "/home/tinyos/local/src/tinyos-2.x/tos/system/ArbiterP.nc"
uint8_t arg_0x40e33310);
# 87 "/home/tinyos/local/src/tinyos-2.x/tos/interfaces/Resource.nc"
static error_t /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0__Resource__immediateRequest(
# 54 "/home/tinyos/local/src/tinyos-2.x/tos/system/ArbiterP.nc"
uint8_t arg_0x40e33310);
# 78 "/home/tinyos/local/src/tinyos-2.x/tos/interfaces/Resource.nc"
static error_t /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0__Resource__request(
# 54 "/home/tinyos/local/src/tinyos-2.x/tos/system/ArbiterP.nc"
uint8_t arg_0x40e33310);
# 92 "/home/tinyos/local/src/tinyos-2.x/tos/interfaces/Resource.nc"
static void /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0__Resource__default__granted(
# 54 "/home/tinyos/local/src/tinyos-2.x/tos/system/ArbiterP.nc"
uint8_t arg_0x40e33310);
# 118 "/home/tinyos/local/src/tinyos-2.x/tos/interfaces/Resource.nc"
static bool /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0__Resource__isOwner(
# 54 "/home/tinyos/local/src/tinyos-2.x/tos/system/ArbiterP.nc"
uint8_t arg_0x40e33310);
# 80 "/home/tinyos/local/src/tinyos-2.x/tos/interfaces/ArbiterInfo.nc"
static bool /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0__ArbiterInfo__inUse(void );







static uint8_t /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0__ArbiterInfo__userId(void );
# 64 "/home/tinyos/local/src/tinyos-2.x/tos/interfaces/TaskBasic.nc"
static void /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0__grantedTask__runTask(void );
# 7 "/home/tinyos/local/src/tinyos-2.x/tos/chips/msp430/usart/HplMsp430I2C.nc"
static void HplMsp430I2C0P__HplI2C__clearModeI2C(void );
#line 6
static bool HplMsp430I2C0P__HplI2C__isI2C(void );
# 44 "/home/tinyos/local/src/tinyos-2.x/tos/system/ActiveMessageAddressC.nc"
static am_addr_t ActiveMessageAddressC__amAddress(void );
# 50 "/home/tinyos/local/src/tinyos-2.x/tos/interfaces/ActiveMessageAddress.nc"
static am_addr_t ActiveMessageAddressC__ActiveMessageAddress__amAddress(void );




static am_group_t ActiveMessageAddressC__ActiveMessageAddress__amGroup(void );
# 66 "/home/tinyos/local/src/tinyos-2.x/tos/chips/cc2420/interfaces/RadioBackoff.nc"
static void CC2420TransmitP__RadioBackoff__setCongestionBackoff(uint16_t backoffTime);
#line 60
static void CC2420TransmitP__RadioBackoff__setInitialBackoff(uint16_t backoffTime);
# 50 "/home/tinyos/local/src/tinyos-2.x/tos/interfaces/GpioCapture.nc"
static void CC2420TransmitP__CaptureSFD__captured(uint16_t time);
# 67 "/home/tinyos/local/src/tinyos-2.x/tos/lib/timer/Alarm.nc"
static void CC2420TransmitP__BackoffTimer__fired(void );
# 63 "/home/tinyos/local/src/tinyos-2.x/tos/chips/cc2420/interfaces/CC2420Receive.nc"
static void CC2420TransmitP__CC2420Receive__receive(uint8_t type, message_t * message);
# 51 "/home/tinyos/local/src/tinyos-2.x/tos/chips/cc2420/interfaces/CC2420Transmit.nc"
static error_t CC2420TransmitP__Send__send(message_t * p_msg, bool useCca);
# 24 "/home/tinyos/local/src/tinyos-2.x/tos/chips/cc2420/interfaces/ChipSpiResource.nc"
static void CC2420TransmitP__ChipSpiResource__releasing(void );
# 51 "/home/tinyos/local/src/tinyos-2.x/tos/interfaces/Init.nc"
static error_t CC2420TransmitP__Init__init(void );
# 92 "/home/tinyos/local/src/tinyos-2.x/tos/interfaces/Resource.nc"
static void CC2420TransmitP__SpiResource__granted(void );
# 74 "/home/tinyos/local/src/tinyos-2.x/tos/interfaces/StdControl.nc"
static error_t CC2420TransmitP__StdControl__start(void );









static error_t CC2420TransmitP__StdControl__stop(void );
# 91 "/home/tinyos/local/src/tinyos-2.x/tos/chips/cc2420/interfaces/CC2420Fifo.nc"
static void CC2420TransmitP__TXFIFO__writeDone(uint8_t * data, uint8_t length, error_t error);
#line 71
static void CC2420TransmitP__TXFIFO__readDone(uint8_t * data, uint8_t length, error_t error);
# 53 "/home/tinyos/local/src/tinyos-2.x/tos/chips/cc2420/interfaces/CC2420Config.nc"
static void CC2420ReceiveP__CC2420Config__syncDone(error_t error);
# 64 "/home/tinyos/local/src/tinyos-2.x/tos/interfaces/TaskBasic.nc"
static void CC2420ReceiveP__receiveDone_task__runTask(void );
# 55 "/home/tinyos/local/src/tinyos-2.x/tos/chips/cc2420/interfaces/CC2420Receive.nc"
static void CC2420ReceiveP__CC2420Receive__sfd_dropped(void );
#line 49
static void CC2420ReceiveP__CC2420Receive__sfd(uint32_t time);
# 51 "/home/tinyos/local/src/tinyos-2.x/tos/interfaces/Init.nc"
static error_t CC2420ReceiveP__Init__init(void );
# 92 "/home/tinyos/local/src/tinyos-2.x/tos/interfaces/Resource.nc"
static void CC2420ReceiveP__SpiResource__granted(void );
# 91 "/home/tinyos/local/src/tinyos-2.x/tos/chips/cc2420/interfaces/CC2420Fifo.nc"
static void CC2420ReceiveP__RXFIFO__writeDone(uint8_t * data, uint8_t length, error_t error);
#line 71
static void CC2420ReceiveP__RXFIFO__readDone(uint8_t * data, uint8_t length, error_t error);
# 57 "/home/tinyos/local/src/tinyos-2.x/tos/interfaces/GpioInterrupt.nc"
static void CC2420ReceiveP__InterruptFIFOP__fired(void );
# 74 "/home/tinyos/local/src/tinyos-2.x/tos/interfaces/StdControl.nc"
static error_t CC2420ReceiveP__StdControl__start(void );









static error_t CC2420ReceiveP__StdControl__stop(void );
# 75 "/home/tinyos/local/src/tinyos-2.x/tos/chips/cc2420/interfaces/CC2420Packet.nc"
static uint8_t CC2420PacketP__CC2420Packet__getNetwork(message_t *p_msg);
#line 72
static uint8_t CC2420PacketP__CC2420Packet__getLqi(message_t *p_msg);
# 59 "/home/tinyos/local/src/tinyos-2.x/tos/interfaces/PacketTimeStamp.nc"
static void CC2420PacketP__PacketTimeStamp32khz__clear(
#line 55
message_t * msg);
#line 67
static void CC2420PacketP__PacketTimeStamp32khz__set(
#line 62
message_t * msg, 




CC2420PacketP__PacketTimeStamp32khz__size_type value);
# 42 "/home/tinyos/local/src/tinyos-2.x/tos/chips/cc2420/interfaces/CC2420PacketBody.nc"
static cc2420_header_t * CC2420PacketP__CC2420PacketBody__getHeader(message_t * msg);




static cc2420_metadata_t * CC2420PacketP__CC2420PacketBody__getMetadata(message_t * msg);
# 47 "/home/tinyos/local/src/tinyos-2.x/tos/chips/cc2420/interfaces/PacketTimeSyncOffset.nc"
static uint8_t CC2420PacketP__PacketTimeSyncOffset__get(
#line 42
message_t * msg);
#line 39
static bool CC2420PacketP__PacketTimeSyncOffset__isSet(
#line 35
message_t * msg);
# 48 "/home/tinyos/local/src/tinyos-2.x/tos/interfaces/PacketAcknowledgements.nc"
static error_t CC2420PacketP__Acks__requestAck(
#line 42
message_t * msg);
#line 74
static bool CC2420PacketP__Acks__wasAcked(
#line 69
message_t * msg);
# 71 "/home/tinyos/local/src/tinyos-2.x/tos/lib/timer/Counter.nc"
static void /*CC2420PacketC.CounterToLocalTimeC*/CounterToLocalTimeC__1__Counter__overflow(void );
# 41 "/home/tinyos/local/src/tinyos-2.x/tos/interfaces/Random.nc"
static uint16_t RandomMlcgC__Random__rand16(void );
#line 35
static uint32_t RandomMlcgC__Random__rand32(void );
# 51 "/home/tinyos/local/src/tinyos-2.x/tos/interfaces/Init.nc"
static error_t RandomMlcgC__Init__init(void );
# 89 "/home/tinyos/local/src/tinyos-2.x/tos/interfaces/Send.nc"
static void UniqueSendP__SubSend__sendDone(
#line 85
message_t * msg, 



error_t error);
#line 64
static error_t UniqueSendP__Send__send(
#line 56
message_t * msg, 







uint8_t len);
# 51 "/home/tinyos/local/src/tinyos-2.x/tos/interfaces/Init.nc"
static error_t UniqueSendP__Init__init(void );
# 67 "/home/tinyos/local/src/tinyos-2.x/tos/interfaces/Receive.nc"
static 
#line 63
message_t * 



UniqueReceiveP__SubReceive__receive(
#line 60
message_t * msg, 
void * payload, 





uint8_t len);
# 51 "/home/tinyos/local/src/tinyos-2.x/tos/interfaces/Init.nc"
static error_t UniqueReceiveP__Init__init(void );
# 67 "/home/tinyos/local/src/tinyos-2.x/tos/interfaces/Receive.nc"
static 
#line 63
message_t * 



UniqueReceiveP__DuplicateReceive__default__receive(
#line 60
message_t * msg, 
void * payload, 





uint8_t len);
# 89 "/home/tinyos/local/src/tinyos-2.x/tos/interfaces/Send.nc"
static void CC2420TinyosNetworkP__SubSend__sendDone(
#line 85
message_t * msg, 



error_t error);
# 67 "/home/tinyos/local/src/tinyos-2.x/tos/interfaces/Receive.nc"
static 
#line 63
message_t * 



CC2420TinyosNetworkP__SubReceive__receive(
#line 60
message_t * msg, 
void * payload, 





uint8_t len);
# 64 "/home/tinyos/local/src/tinyos-2.x/tos/interfaces/TaskBasic.nc"
static void CC2420TinyosNetworkP__grantTask__runTask(void );
# 89 "/home/tinyos/local/src/tinyos-2.x/tos/interfaces/Send.nc"
static void CC2420TinyosNetworkP__ActiveSend__default__sendDone(
#line 85
message_t * msg, 



error_t error);
# 110 "/home/tinyos/local/src/tinyos-2.x/tos/interfaces/Resource.nc"
static error_t CC2420TinyosNetworkP__Resource__release(
# 46 "/home/tinyos/local/src/tinyos-2.x/tos/chips/cc2420/lowpan/CC2420TinyosNetworkP.nc"
uint8_t arg_0x41096628);
# 78 "/home/tinyos/local/src/tinyos-2.x/tos/interfaces/Resource.nc"
static error_t CC2420TinyosNetworkP__Resource__request(
# 46 "/home/tinyos/local/src/tinyos-2.x/tos/chips/cc2420/lowpan/CC2420TinyosNetworkP.nc"
uint8_t arg_0x41096628);
# 92 "/home/tinyos/local/src/tinyos-2.x/tos/interfaces/Resource.nc"
static void CC2420TinyosNetworkP__Resource__default__granted(
# 46 "/home/tinyos/local/src/tinyos-2.x/tos/chips/cc2420/lowpan/CC2420TinyosNetworkP.nc"
uint8_t arg_0x41096628);
# 64 "/home/tinyos/local/src/tinyos-2.x/tos/interfaces/Send.nc"
static error_t CC2420TinyosNetworkP__BareSend__send(
#line 56
message_t * msg, 







uint8_t len);
#line 114
static 
#line 112
void * 

CC2420TinyosNetworkP__BareSend__getPayload(
#line 111
message_t * msg, 


uint8_t len);
# 67 "/home/tinyos/local/src/tinyos-2.x/tos/interfaces/Receive.nc"
static 
#line 63
message_t * 



CC2420TinyosNetworkP__ActiveReceive__default__receive(
#line 60
message_t * msg, 
void * payload, 





uint8_t len);
# 51 "/home/tinyos/local/src/tinyos-2.x/tos/interfaces/Init.nc"
static error_t /*CC2420TinyosNetworkC.FcfsResourceQueueC*/FcfsResourceQueueC__0__Init__init(void );
# 69 "/home/tinyos/local/src/tinyos-2.x/tos/interfaces/ResourceQueue.nc"
static error_t /*CC2420TinyosNetworkC.FcfsResourceQueueC*/FcfsResourceQueueC__0__FcfsQueue__enqueue(resource_client_id_t id);
#line 43
static bool /*CC2420TinyosNetworkC.FcfsResourceQueueC*/FcfsResourceQueueC__0__FcfsQueue__isEmpty(void );








static bool /*CC2420TinyosNetworkC.FcfsResourceQueueC*/FcfsResourceQueueC__0__FcfsQueue__isEnqueued(resource_client_id_t id);







static resource_client_id_t /*CC2420TinyosNetworkC.FcfsResourceQueueC*/FcfsResourceQueueC__0__FcfsQueue__dequeue(void );
# 89 "/home/tinyos/local/src/tinyos-2.x/tos/interfaces/Send.nc"
static void PacketLinkP__SubSend__sendDone(
#line 85
message_t * msg, 



error_t error);
# 64 "/home/tinyos/local/src/tinyos-2.x/tos/interfaces/TaskBasic.nc"
static void PacketLinkP__send__runTask(void );
# 72 "/home/tinyos/local/src/tinyos-2.x/tos/lib/timer/Timer.nc"
static void PacketLinkP__DelayTimer__fired(void );
# 64 "/home/tinyos/local/src/tinyos-2.x/tos/interfaces/Send.nc"
static error_t PacketLinkP__Send__send(
#line 56
message_t * msg, 







uint8_t len);
# 65 "/home/tinyos/local/src/tinyos-2.x/tos/interfaces/PacketLink.nc"
static uint16_t PacketLinkP__PacketLink__getRetryDelay(
#line 62
message_t * msg);
#line 46
static void PacketLinkP__PacketLink__setRetries(
#line 42
message_t * msg, 



uint16_t maxRetries);
#line 59
static uint16_t PacketLinkP__PacketLink__getRetries(
#line 56
message_t * msg);
#line 53
static void PacketLinkP__PacketLink__setRetryDelay(message_t *msg, uint16_t retryDelay);
#line 71
static bool PacketLinkP__PacketLink__wasDelivered(
#line 68
message_t * msg);
# 89 "/home/tinyos/local/src/tinyos-2.x/tos/interfaces/Send.nc"
static void CC2420Ieee154MessageP__SubSend__sendDone(
#line 85
message_t * msg, 



error_t error);
# 53 "/home/tinyos/local/src/tinyos-2.x/tos/chips/cc2420/interfaces/CC2420Config.nc"
static void CC2420Ieee154MessageP__CC2420Config__syncDone(error_t error);
# 67 "/home/tinyos/local/src/tinyos-2.x/tos/interfaces/Packet.nc"
static uint8_t CC2420Ieee154MessageP__Packet__payloadLength(
#line 63
message_t * msg);
#line 115
static 
#line 112
void * 


CC2420Ieee154MessageP__Packet__getPayload(
#line 110
message_t * msg, 




uint8_t len);
#line 95
static uint8_t CC2420Ieee154MessageP__Packet__maxPayloadLength(void );
#line 83
static void CC2420Ieee154MessageP__Packet__setPayloadLength(
#line 79
message_t * msg, 



uint8_t len);
# 30 "/home/tinyos/local/src/tinyos-2.x/tos/interfaces/Ieee154Packet.nc"
static ieee154_saddr_t CC2420Ieee154MessageP__Ieee154Packet__source(message_t *msg);
#line 28
static ieee154_saddr_t CC2420Ieee154MessageP__Ieee154Packet__destination(message_t *msg);



static void CC2420Ieee154MessageP__Ieee154Packet__setDestination(message_t *msg, ieee154_saddr_t addr);
# 56 "/home/tinyos/local/src/tinyos-2.x/tos/interfaces/Ieee154Send.nc"
static error_t CC2420Ieee154MessageP__Ieee154Send__send(ieee154_saddr_t addr, message_t *msg, uint8_t len);
# 83 "/home/tinyos/local/src/tinyos-2.x/tos/interfaces/SplitControl.nc"
static error_t IPDispatchP__SplitControl__start(void );
# 49 "/home/tinyos/local/src/tinyos-2.x/tos/interfaces/Boot.nc"
static void IPDispatchP__Boot__booted(void );
# 92 "/home/tinyos/local/src/tinyos-2.x/tos/interfaces/SplitControl.nc"
static void IPDispatchP__RadioControl__startDone(error_t error);
#line 117
static void IPDispatchP__RadioControl__stopDone(error_t error);
# 72 "/home/tinyos/local/src/tinyos-2.x/tos/lib/timer/Timer.nc"
static void IPDispatchP__ExpireTimer__fired(void );
# 64 "/home/tinyos/local/src/tinyos-2.x/tos/interfaces/TaskBasic.nc"
static void IPDispatchP__sendTask__runTask(void );
# 31 "/home/tinyos/local/src/tinyos-2.x/tos/lib/net/blip/interfaces/ICMP.nc"
static void IPDispatchP__ICMP__solicitationDone(void );
# 34 "/home/tinyos/local/src/tinyos-2.x/tos/lib/net/blip/interfaces/Statistics.nc"
static void IPDispatchP__Statistics__clear(void );
#line 29
static void IPDispatchP__Statistics__get(IPDispatchP__Statistics__stat_str *stats);
# 86 "/home/tinyos/local/src/tinyos-2.x/tos/interfaces/Ieee154Send.nc"
static void IPDispatchP__Ieee154Send__sendDone(message_t *msg, error_t error);
# 67 "/home/tinyos/local/src/tinyos-2.x/tos/interfaces/Receive.nc"
static 
#line 63
message_t * 



IPDispatchP__Ieee154Receive__receive(
#line 60
message_t * msg, 
void * payload, 





uint8_t len);
# 15 "/home/tinyos/local/src/tinyos-2.x/tos/lib/net/blip/interfaces/IP.nc"
static error_t IPDispatchP__IP__send(
# 90 "/home/tinyos/local/src/tinyos-2.x/tos/lib/net/blip/IPDispatchP.nc"
uint8_t arg_0x41134178, 
# 15 "/home/tinyos/local/src/tinyos-2.x/tos/lib/net/blip/interfaces/IP.nc"
struct split_ip_msg *msg);

static error_t IPDispatchP__IP__bareSend(
# 90 "/home/tinyos/local/src/tinyos-2.x/tos/lib/net/blip/IPDispatchP.nc"
uint8_t arg_0x41134178, 
# 17 "/home/tinyos/local/src/tinyos-2.x/tos/lib/net/blip/interfaces/IP.nc"
struct split_ip_msg *msg, 
struct ip6_route *route, 
int flags);





static void IPDispatchP__IP__default__recv(
# 90 "/home/tinyos/local/src/tinyos-2.x/tos/lib/net/blip/IPDispatchP.nc"
uint8_t arg_0x41134178, 
# 25 "/home/tinyos/local/src/tinyos-2.x/tos/lib/net/blip/interfaces/IP.nc"
struct ip6_hdr *iph, void *payload, struct ip_metadata *meta);
# 29 "/home/tinyos/local/src/tinyos-2.x/tos/lib/net/blip/interfaces/IPAddress.nc"
static void IPAddressP__IPAddress__getLLAddr(struct in6_addr *addr);


static void IPAddressP__IPAddress__setSource(struct ip6_hdr *hdr);
#line 25
static ieee154_saddr_t IPAddressP__IPAddress__getShortAddr(void );




static void IPAddressP__IPAddress__getIPAddr(struct in6_addr *addr);



static void IPAddressP__IPAddress__setPrefix(uint8_t *prefix);
#line 28
static struct in6_addr *IPAddressP__IPAddress__getPublicAddr(void );







static bool IPAddressP__IPAddress__haveAddress(void );
# 6 "/home/tinyos/local/src/tinyos-2.x/tos/lib/net/blip/interfaces/IPExtensions.nc"
static void IPRoutingP__IPExtensions__handleExtensions(uint8_t label, 
struct ip6_hdr *iph, 
struct ip6_ext *hop, 
struct ip6_ext *dst, 
struct ip6_route *route, 
uint8_t nxt_hdr);





static void IPRoutingP__IPExtensions__reportTransmission(uint8_t label, send_policy_t *send);
# 49 "/home/tinyos/local/src/tinyos-2.x/tos/interfaces/Boot.nc"
static void IPRoutingP__Boot__booted(void );
# 72 "/home/tinyos/local/src/tinyos-2.x/tos/lib/timer/Timer.nc"
static void IPRoutingP__SortTimer__fired(void );
# 31 "/home/tinyos/local/src/tinyos-2.x/tos/lib/net/blip/interfaces/ICMP.nc"
static void IPRoutingP__ICMP__solicitationDone(void );
# 5 "/home/tinyos/local/src/tinyos-2.x/tos/lib/net/blip/interfaces/TLVHeader.nc"
static struct tlv_hdr *IPRoutingP__DestinationExt__getHeader(int label, int nxt_hdr, 
struct ip6_hdr *msg);
# 34 "/home/tinyos/local/src/tinyos-2.x/tos/lib/net/blip/interfaces/Statistics.nc"
static void IPRoutingP__Statistics__clear(void );
#line 29
static void IPRoutingP__Statistics__get(IPRoutingP__Statistics__stat_str *stats);
# 25 "/home/tinyos/local/src/tinyos-2.x/tos/lib/net/blip/interfaces/IP.nc"
static void IPRoutingP__TGenSend__recv(struct ip6_hdr *iph, void *payload, struct ip_metadata *meta);
# 72 "/home/tinyos/local/src/tinyos-2.x/tos/lib/timer/Timer.nc"
static void IPRoutingP__TrafficGenTimer__fired(void );
# 79 "/home/tinyos/local/src/tinyos-2.x/tos/lib/net/blip/interfaces/IPRouting.nc"
static bool IPRoutingP__IPRouting__hasRoute(void );
#line 48
static error_t IPRoutingP__IPRouting__getNextHop(struct ip6_hdr *hdr, 
struct ip6_route *routing_hdr, 
ieee154_saddr_t prev_hop, 
send_policy_t *ret);
#line 83
static void IPRoutingP__IPRouting__reset(void );
#line 60
static uint16_t IPRoutingP__IPRouting__getQuality(void );
#line 58
static uint8_t IPRoutingP__IPRouting__getHopLimit(void );







static void IPRoutingP__IPRouting__reportAdvertisement(ieee154_saddr_t neigh, uint8_t hops, 
uint8_t lqi, uint16_t cost);
#line 40
static bool IPRoutingP__IPRouting__isForMe(struct ip6_hdr *a);
#line 74
static void IPRoutingP__IPRouting__reportReception(ieee154_saddr_t neigh, uint8_t lqi);






static struct ip6_route *IPRoutingP__IPRouting__insertRoutingHeader(struct split_ip_msg *msg);
# 72 "/home/tinyos/local/src/tinyos-2.x/tos/interfaces/Leds.nc"
static void NoLedsC__Leds__led1Toggle(void );
# 86 "/home/tinyos/local/src/tinyos-2.x/tos/interfaces/Ieee154Send.nc"
static void ResourceSendP__SubSend__sendDone(message_t *msg, error_t error);
#line 56
static error_t ResourceSendP__Ieee154Send__send(ieee154_saddr_t addr, message_t *msg, uint8_t len);
# 92 "/home/tinyos/local/src/tinyos-2.x/tos/interfaces/Resource.nc"
static void ResourceSendP__Resource__granted(void );
# 3 "/home/tinyos/local/src/tinyos-2.x/tos/lib/net/blip/interfaces/ReadLqi.nc"
static uint8_t CC2420ReadLqiC__ReadLqi__read(message_t *msg);
# 97 "/home/tinyos/local/src/tinyos-2.x/tos/interfaces/Pool.nc"
static 
#line 94
/*IPDispatchC.FragPool.PoolP*/PoolP__0__Pool__t * 


/*IPDispatchC.FragPool.PoolP*/PoolP__0__Pool__get(void );
#line 89
static error_t /*IPDispatchC.FragPool.PoolP*/PoolP__0__Pool__put(
#line 85
/*IPDispatchC.FragPool.PoolP*/PoolP__0__Pool__t * newVal);
# 51 "/home/tinyos/local/src/tinyos-2.x/tos/interfaces/Init.nc"
static error_t /*IPDispatchC.FragPool.PoolP*/PoolP__0__Init__init(void );
# 97 "/home/tinyos/local/src/tinyos-2.x/tos/interfaces/Pool.nc"
static 
#line 94
/*IPDispatchC.SendEntryPool.PoolP*/PoolP__1__Pool__t * 


/*IPDispatchC.SendEntryPool.PoolP*/PoolP__1__Pool__get(void );
#line 89
static error_t /*IPDispatchC.SendEntryPool.PoolP*/PoolP__1__Pool__put(
#line 85
/*IPDispatchC.SendEntryPool.PoolP*/PoolP__1__Pool__t * newVal);
# 51 "/home/tinyos/local/src/tinyos-2.x/tos/interfaces/Init.nc"
static error_t /*IPDispatchC.SendEntryPool.PoolP*/PoolP__1__Init__init(void );
# 73 "/home/tinyos/local/src/tinyos-2.x/tos/interfaces/Queue.nc"
static 
#line 71
/*IPDispatchC.QueueC*/QueueC__0__Queue__t  

/*IPDispatchC.QueueC*/QueueC__0__Queue__head(void );
#line 90
static error_t /*IPDispatchC.QueueC*/QueueC__0__Queue__enqueue(
#line 86
/*IPDispatchC.QueueC*/QueueC__0__Queue__t  newVal);
#line 65
static uint8_t /*IPDispatchC.QueueC*/QueueC__0__Queue__maxSize(void );
#line 81
static 
#line 79
/*IPDispatchC.QueueC*/QueueC__0__Queue__t  

/*IPDispatchC.QueueC*/QueueC__0__Queue__dequeue(void );
#line 50
static bool /*IPDispatchC.QueueC*/QueueC__0__Queue__empty(void );







static uint8_t /*IPDispatchC.QueueC*/QueueC__0__Queue__size(void );
# 97 "/home/tinyos/local/src/tinyos-2.x/tos/interfaces/Pool.nc"
static 
#line 94
/*IPDispatchC.SendInfoPool.PoolP*/PoolP__2__Pool__t * 


/*IPDispatchC.SendInfoPool.PoolP*/PoolP__2__Pool__get(void );
#line 89
static error_t /*IPDispatchC.SendInfoPool.PoolP*/PoolP__2__Pool__put(
#line 85
/*IPDispatchC.SendInfoPool.PoolP*/PoolP__2__Pool__t * newVal);
# 51 "/home/tinyos/local/src/tinyos-2.x/tos/interfaces/Init.nc"
static error_t /*IPDispatchC.SendInfoPool.PoolP*/PoolP__2__Init__init(void );
# 72 "/home/tinyos/local/src/tinyos-2.x/tos/lib/timer/Timer.nc"
static void ICMPResponderP__PingTimer__fired(void );
#line 72
static void ICMPResponderP__Advertisement__fired(void );
# 33 "/home/tinyos/local/src/tinyos-2.x/tos/lib/net/blip/interfaces/ICMP.nc"
static void ICMPResponderP__ICMP__sendAdvertisements(void );
#line 28
static void ICMPResponderP__ICMP__sendSolicitations(void );






static void ICMPResponderP__ICMP__sendTimeExceeded(struct ip6_hdr *hdr, unpack_info_t *u_info, uint16_t amount_here);
#line 25
static uint16_t ICMPResponderP__ICMP__cksum(struct split_ip_msg *msg, uint8_t nxt_hdr);
# 34 "/home/tinyos/local/src/tinyos-2.x/tos/lib/net/blip/interfaces/Statistics.nc"
static void ICMPResponderP__Statistics__clear(void );
#line 29
static void ICMPResponderP__Statistics__get(ICMPResponderP__Statistics__stat_str *stats);
# 72 "/home/tinyos/local/src/tinyos-2.x/tos/lib/timer/Timer.nc"
static void ICMPResponderP__Solicitation__fired(void );
# 10 "/home/tinyos/local/src/tinyos-2.x/tos/lib/net/blip/interfaces/ICMPPing.nc"
static void ICMPResponderP__ICMPPing__default__pingDone(
# 35 "/home/tinyos/local/src/tinyos-2.x/tos/lib/net/blip/ICMPResponderP.nc"
uint16_t arg_0x412fc2e0, 
# 10 "/home/tinyos/local/src/tinyos-2.x/tos/lib/net/blip/interfaces/ICMPPing.nc"
uint16_t ping_rcv, uint16_t ping_n);
#line 8
static void ICMPResponderP__ICMPPing__default__pingReply(
# 35 "/home/tinyos/local/src/tinyos-2.x/tos/lib/net/blip/ICMPResponderP.nc"
uint16_t arg_0x412fc2e0, 
# 8 "/home/tinyos/local/src/tinyos-2.x/tos/lib/net/blip/interfaces/ICMPPing.nc"
struct in6_addr *source, struct icmp_stats *stats);
#line 6
static error_t ICMPResponderP__ICMPPing__ping(
# 35 "/home/tinyos/local/src/tinyos-2.x/tos/lib/net/blip/ICMPResponderP.nc"
uint16_t arg_0x412fc2e0, 
# 6 "/home/tinyos/local/src/tinyos-2.x/tos/lib/net/blip/interfaces/ICMPPing.nc"
struct in6_addr *target, uint16_t period, uint16_t n);
# 25 "/home/tinyos/local/src/tinyos-2.x/tos/lib/net/blip/interfaces/IP.nc"
static void ICMPResponderP__IP__recv(struct ip6_hdr *iph, void *payload, struct ip_metadata *meta);
# 5 "/home/tinyos/local/src/tinyos-2.x/tos/lib/net/blip/interfaces/TLVHeader.nc"
static struct tlv_hdr *IPExtensionP__HopByHopExt__default__getHeader(
# 16 "/home/tinyos/local/src/tinyos-2.x/tos/lib/net/blip/IPExtensionP.nc"
uint8_t arg_0x413a5710, 
# 5 "/home/tinyos/local/src/tinyos-2.x/tos/lib/net/blip/interfaces/TLVHeader.nc"
int label, int nxt_hdr, 
struct ip6_hdr *msg);
# 6 "/home/tinyos/local/src/tinyos-2.x/tos/lib/net/blip/interfaces/InternalIPExtension.nc"
static void IPExtensionP__InternalIPExtension__ip_free(void );
#line 4
static void IPExtensionP__InternalIPExtension__addHeaders(struct split_ip_msg *msg, uint8_t nxt_hdr, uint16_t label);
# 51 "/home/tinyos/local/src/tinyos-2.x/tos/interfaces/Init.nc"
static error_t IPExtensionP__Init__init(void );
# 5 "/home/tinyos/local/src/tinyos-2.x/tos/lib/net/blip/interfaces/TLVHeader.nc"
static struct tlv_hdr *IPExtensionP__DestinationExt__default__getHeader(
# 17 "/home/tinyos/local/src/tinyos-2.x/tos/lib/net/blip/IPExtensionP.nc"
uint8_t arg_0x413a5f20, 
# 5 "/home/tinyos/local/src/tinyos-2.x/tos/lib/net/blip/interfaces/TLVHeader.nc"
int label, int nxt_hdr, 
struct ip6_hdr *msg);
# 51 "/home/tinyos/local/src/tinyos-2.x/tos/interfaces/Init.nc"
static error_t UdpP__Init__init(void );
# 16 "/home/tinyos/local/src/tinyos-2.x/tos/lib/net/blip/interfaces/UDP.nc"
static error_t UdpP__UDP__sendto(
# 7 "/home/tinyos/local/src/tinyos-2.x/tos/lib/net/blip/UdpP.nc"
uint8_t arg_0x413c2e40, 
# 16 "/home/tinyos/local/src/tinyos-2.x/tos/lib/net/blip/interfaces/UDP.nc"
struct sockaddr_in6 *dest, void *payload, 
uint16_t len);
#line 10
static error_t UdpP__UDP__bind(
# 7 "/home/tinyos/local/src/tinyos-2.x/tos/lib/net/blip/UdpP.nc"
uint8_t arg_0x413c2e40, 
# 10 "/home/tinyos/local/src/tinyos-2.x/tos/lib/net/blip/interfaces/UDP.nc"
uint16_t port);
#line 24
static void UdpP__UDP__default__recvfrom(
# 7 "/home/tinyos/local/src/tinyos-2.x/tos/lib/net/blip/UdpP.nc"
uint8_t arg_0x413c2e40, 
# 24 "/home/tinyos/local/src/tinyos-2.x/tos/lib/net/blip/interfaces/UDP.nc"
struct sockaddr_in6 *src, void *payload, 
uint16_t len, struct ip_metadata *meta);
# 34 "/home/tinyos/local/src/tinyos-2.x/tos/lib/net/blip/interfaces/Statistics.nc"
static void UdpP__Statistics__clear(void );
#line 29
static void UdpP__Statistics__get(UdpP__Statistics__stat_str *stats);
# 25 "/home/tinyos/local/src/tinyos-2.x/tos/lib/net/blip/interfaces/IP.nc"
static void UdpP__IP__recv(struct ip6_hdr *iph, void *payload, struct ip_metadata *meta);
# 72 "/home/tinyos/local/src/tinyos-2.x/tos/lib/timer/Timer.nc"
static void TcpP__Timer__fired(void );
# 32 "/home/tinyos/local/src/tinyos-2.x/tos/lib/net/blip/interfaces/Tcp.nc"
static error_t TcpP__Tcp__send(
# 6 "/home/tinyos/local/src/tinyos-2.x/tos/lib/net/blip/TcpP.nc"
uint8_t arg_0x413d8b78, 
# 32 "/home/tinyos/local/src/tinyos-2.x/tos/lib/net/blip/interfaces/Tcp.nc"
void *payload, uint16_t len);
#line 9
static error_t TcpP__Tcp__bind(
# 6 "/home/tinyos/local/src/tinyos-2.x/tos/lib/net/blip/TcpP.nc"
uint8_t arg_0x413d8b78, 
# 9 "/home/tinyos/local/src/tinyos-2.x/tos/lib/net/blip/interfaces/Tcp.nc"
uint16_t port);






static bool TcpP__Tcp__default__accept(
# 6 "/home/tinyos/local/src/tinyos-2.x/tos/lib/net/blip/TcpP.nc"
uint8_t arg_0x413d8b78, 
# 16 "/home/tinyos/local/src/tinyos-2.x/tos/lib/net/blip/interfaces/Tcp.nc"
struct sockaddr_in6 *from, 
void **tx_buf, int *tx_buf_len);
#line 34
static void TcpP__Tcp__default__recv(
# 6 "/home/tinyos/local/src/tinyos-2.x/tos/lib/net/blip/TcpP.nc"
uint8_t arg_0x413d8b78, 
# 34 "/home/tinyos/local/src/tinyos-2.x/tos/lib/net/blip/interfaces/Tcp.nc"
void *payload, uint16_t len);




static error_t TcpP__Tcp__close(
# 6 "/home/tinyos/local/src/tinyos-2.x/tos/lib/net/blip/TcpP.nc"
uint8_t arg_0x413d8b78);
# 47 "/home/tinyos/local/src/tinyos-2.x/tos/lib/net/blip/interfaces/Tcp.nc"
static void TcpP__Tcp__default__closed(
# 6 "/home/tinyos/local/src/tinyos-2.x/tos/lib/net/blip/TcpP.nc"
uint8_t arg_0x413d8b78, 
# 47 "/home/tinyos/local/src/tinyos-2.x/tos/lib/net/blip/interfaces/Tcp.nc"
error_t e);




static void TcpP__Tcp__default__acked(
# 6 "/home/tinyos/local/src/tinyos-2.x/tos/lib/net/blip/TcpP.nc"
uint8_t arg_0x413d8b78);
# 49 "/home/tinyos/local/src/tinyos-2.x/tos/interfaces/Boot.nc"
static void TcpP__Boot__booted(void );
# 51 "/home/tinyos/local/src/tinyos-2.x/tos/interfaces/Init.nc"
static error_t TcpP__Init__init(void );
# 25 "/home/tinyos/local/src/tinyos-2.x/tos/lib/net/blip/interfaces/IP.nc"
static void TcpP__IP__recv(struct ip6_hdr *iph, void *payload, struct ip_metadata *meta);
# 49 "/home/tinyos/local/src/tinyos-2.x/tos/interfaces/Boot.nc"
static void HttpdP__Boot__booted(void );
# 16 "/home/tinyos/local/src/tinyos-2.x/tos/lib/net/blip/interfaces/Tcp.nc"
static bool HttpdP__Tcp__accept(struct sockaddr_in6 *from, 
void **tx_buf, int *tx_buf_len);
#line 34
static void HttpdP__Tcp__recv(void *payload, uint16_t len);
#line 47
static void HttpdP__Tcp__closed(error_t e);




static void HttpdP__Tcp__acked(void );
# 72 "/home/tinyos/local/src/tinyos-2.x/tos/lib/timer/Timer.nc"
static void HttpdP__TimerTemp__fired(void );
# 63 "/home/tinyos/local/src/tinyos-2.x/tos/interfaces/Read.nc"
static void HttpdP__ReadTemp__readDone(error_t result, HttpdP__ReadTemp__val_t val);
# 84 "/home/tinyos/local/src/tinyos-2.x/tos/chips/sht11/SensirionSht11.nc"
static void /*TCPEchoC.Sht11C.SensirionSht11ReaderP*/SensirionSht11ReaderP__0__Sht11Hum__measureHumidityDone(error_t result, uint16_t val);
#line 116
static void /*TCPEchoC.Sht11C.SensirionSht11ReaderP*/SensirionSht11ReaderP__0__Sht11Hum__writeStatusRegDone(error_t result);
#line 100
static void /*TCPEchoC.Sht11C.SensirionSht11ReaderP*/SensirionSht11ReaderP__0__Sht11Hum__readStatusRegDone(error_t result, uint8_t val);
#line 54
static void /*TCPEchoC.Sht11C.SensirionSht11ReaderP*/SensirionSht11ReaderP__0__Sht11Hum__resetDone(error_t result);
#line 69
static void /*TCPEchoC.Sht11C.SensirionSht11ReaderP*/SensirionSht11ReaderP__0__Sht11Hum__measureTemperatureDone(error_t result, uint16_t val);
# 63 "/home/tinyos/local/src/tinyos-2.x/tos/interfaces/Read.nc"
static void /*TCPEchoC.Sht11C.SensirionSht11ReaderP*/SensirionSht11ReaderP__0__Humidity__default__readDone(error_t result, /*TCPEchoC.Sht11C.SensirionSht11ReaderP*/SensirionSht11ReaderP__0__Humidity__val_t val);
# 92 "/home/tinyos/local/src/tinyos-2.x/tos/interfaces/Resource.nc"
static void /*TCPEchoC.Sht11C.SensirionSht11ReaderP*/SensirionSht11ReaderP__0__TempResource__granted(void );
# 84 "/home/tinyos/local/src/tinyos-2.x/tos/chips/sht11/SensirionSht11.nc"
static void /*TCPEchoC.Sht11C.SensirionSht11ReaderP*/SensirionSht11ReaderP__0__Sht11Temp__measureHumidityDone(error_t result, uint16_t val);
#line 116
static void /*TCPEchoC.Sht11C.SensirionSht11ReaderP*/SensirionSht11ReaderP__0__Sht11Temp__writeStatusRegDone(error_t result);
#line 100
static void /*TCPEchoC.Sht11C.SensirionSht11ReaderP*/SensirionSht11ReaderP__0__Sht11Temp__readStatusRegDone(error_t result, uint8_t val);
#line 54
static void /*TCPEchoC.Sht11C.SensirionSht11ReaderP*/SensirionSht11ReaderP__0__Sht11Temp__resetDone(error_t result);
#line 69
static void /*TCPEchoC.Sht11C.SensirionSht11ReaderP*/SensirionSht11ReaderP__0__Sht11Temp__measureTemperatureDone(error_t result, uint16_t val);
# 92 "/home/tinyos/local/src/tinyos-2.x/tos/interfaces/Resource.nc"
static void /*TCPEchoC.Sht11C.SensirionSht11ReaderP*/SensirionSht11ReaderP__0__HumResource__granted(void );
# 55 "/home/tinyos/local/src/tinyos-2.x/tos/interfaces/Read.nc"
static error_t /*TCPEchoC.Sht11C.SensirionSht11ReaderP*/SensirionSht11ReaderP__0__Temperature__read(void );
# 57 "/home/tinyos/local/src/tinyos-2.x/tos/interfaces/GpioInterrupt.nc"
static void /*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP__0__InterruptDATA__fired(void );
# 64 "/home/tinyos/local/src/tinyos-2.x/tos/interfaces/TaskBasic.nc"
static void /*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP__0__readSensor__runTask(void );
#line 64
static void /*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP__0__signalStatusDone__runTask(void );
# 84 "/home/tinyos/local/src/tinyos-2.x/tos/chips/sht11/SensirionSht11.nc"
static void /*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP__0__SensirionSht11__default__measureHumidityDone(
# 54 "/home/tinyos/local/src/tinyos-2.x/tos/chips/sht11/SensirionSht11LogicP.nc"
uint8_t arg_0x4150f970, 
# 84 "/home/tinyos/local/src/tinyos-2.x/tos/chips/sht11/SensirionSht11.nc"
error_t result, uint16_t val);
#line 76
static error_t /*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP__0__SensirionSht11__measureHumidity(
# 54 "/home/tinyos/local/src/tinyos-2.x/tos/chips/sht11/SensirionSht11LogicP.nc"
uint8_t arg_0x4150f970);
# 61 "/home/tinyos/local/src/tinyos-2.x/tos/chips/sht11/SensirionSht11.nc"
static error_t /*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP__0__SensirionSht11__measureTemperature(
# 54 "/home/tinyos/local/src/tinyos-2.x/tos/chips/sht11/SensirionSht11LogicP.nc"
uint8_t arg_0x4150f970);
# 116 "/home/tinyos/local/src/tinyos-2.x/tos/chips/sht11/SensirionSht11.nc"
static void /*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP__0__SensirionSht11__default__writeStatusRegDone(
# 54 "/home/tinyos/local/src/tinyos-2.x/tos/chips/sht11/SensirionSht11LogicP.nc"
uint8_t arg_0x4150f970, 
# 116 "/home/tinyos/local/src/tinyos-2.x/tos/chips/sht11/SensirionSht11.nc"
error_t result);
#line 100
static void /*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP__0__SensirionSht11__default__readStatusRegDone(
# 54 "/home/tinyos/local/src/tinyos-2.x/tos/chips/sht11/SensirionSht11LogicP.nc"
uint8_t arg_0x4150f970, 
# 100 "/home/tinyos/local/src/tinyos-2.x/tos/chips/sht11/SensirionSht11.nc"
error_t result, uint8_t val);
#line 54
static void /*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP__0__SensirionSht11__default__resetDone(
# 54 "/home/tinyos/local/src/tinyos-2.x/tos/chips/sht11/SensirionSht11LogicP.nc"
uint8_t arg_0x4150f970, 
# 54 "/home/tinyos/local/src/tinyos-2.x/tos/chips/sht11/SensirionSht11.nc"
error_t result);
#line 69
static void /*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP__0__SensirionSht11__default__measureTemperatureDone(
# 54 "/home/tinyos/local/src/tinyos-2.x/tos/chips/sht11/SensirionSht11LogicP.nc"
uint8_t arg_0x4150f970, 
# 69 "/home/tinyos/local/src/tinyos-2.x/tos/chips/sht11/SensirionSht11.nc"
error_t result, uint16_t val);
# 72 "/home/tinyos/local/src/tinyos-2.x/tos/lib/timer/Timer.nc"
static void /*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP__0__Timer__fired(void );
# 33 "/home/tinyos/local/src/tinyos-2.x/tos/interfaces/GeneralIO.nc"
static void /*HplSensirionSht11C.DATAM*/Msp430GpioC__10__GeneralIO__makeInput(void );
#line 32
static bool /*HplSensirionSht11C.DATAM*/Msp430GpioC__10__GeneralIO__get(void );


static void /*HplSensirionSht11C.DATAM*/Msp430GpioC__10__GeneralIO__makeOutput(void );
#line 29
static void /*HplSensirionSht11C.DATAM*/Msp430GpioC__10__GeneralIO__set(void );
static void /*HplSensirionSht11C.DATAM*/Msp430GpioC__10__GeneralIO__clr(void );


static void /*HplSensirionSht11C.SCKM*/Msp430GpioC__11__GeneralIO__makeInput(void );

static void /*HplSensirionSht11C.SCKM*/Msp430GpioC__11__GeneralIO__makeOutput(void );
#line 29
static void /*HplSensirionSht11C.SCKM*/Msp430GpioC__11__GeneralIO__set(void );
static void /*HplSensirionSht11C.SCKM*/Msp430GpioC__11__GeneralIO__clr(void );




static void /*HplSensirionSht11C.PWRM*/Msp430GpioC__12__GeneralIO__makeOutput(void );
#line 29
static void /*HplSensirionSht11C.PWRM*/Msp430GpioC__12__GeneralIO__set(void );
static void /*HplSensirionSht11C.PWRM*/Msp430GpioC__12__GeneralIO__clr(void );
# 83 "/home/tinyos/local/src/tinyos-2.x/tos/interfaces/SplitControl.nc"
static error_t HplSensirionSht11P__SplitControl__start(void );
#line 109
static error_t HplSensirionSht11P__SplitControl__stop(void );
# 64 "/home/tinyos/local/src/tinyos-2.x/tos/interfaces/TaskBasic.nc"
static void HplSensirionSht11P__stopTask__runTask(void );
# 72 "/home/tinyos/local/src/tinyos-2.x/tos/lib/timer/Timer.nc"
static void HplSensirionSht11P__Timer__fired(void );
# 61 "/home/tinyos/local/src/tinyos-2.x/tos/chips/msp430/pins/HplMsp430Interrupt.nc"
static void /*HplSensirionSht11C.InterruptDATAC*/Msp430InterruptC__2__HplInterrupt__fired(void );
# 50 "/home/tinyos/local/src/tinyos-2.x/tos/interfaces/GpioInterrupt.nc"
static error_t /*HplSensirionSht11C.InterruptDATAC*/Msp430InterruptC__2__Interrupt__disable(void );
#line 43
static error_t /*HplSensirionSht11C.InterruptDATAC*/Msp430InterruptC__2__Interrupt__enableFallingEdge(void );
# 51 "/home/tinyos/local/src/tinyos-2.x/tos/interfaces/Init.nc"
static error_t /*HplSensirionSht11C.Arbiter.Queue*/FcfsResourceQueueC__2__Init__init(void );
# 69 "/home/tinyos/local/src/tinyos-2.x/tos/interfaces/ResourceQueue.nc"
static error_t /*HplSensirionSht11C.Arbiter.Queue*/FcfsResourceQueueC__2__FcfsQueue__enqueue(resource_client_id_t id);
#line 43
static bool /*HplSensirionSht11C.Arbiter.Queue*/FcfsResourceQueueC__2__FcfsQueue__isEmpty(void );








static bool /*HplSensirionSht11C.Arbiter.Queue*/FcfsResourceQueueC__2__FcfsQueue__isEnqueued(resource_client_id_t id);







static resource_client_id_t /*HplSensirionSht11C.Arbiter.Queue*/FcfsResourceQueueC__2__FcfsQueue__dequeue(void );
# 43 "/home/tinyos/local/src/tinyos-2.x/tos/interfaces/ResourceRequested.nc"
static void /*HplSensirionSht11C.Arbiter.Arbiter*/ArbiterP__1__ResourceRequested__default__requested(
# 55 "/home/tinyos/local/src/tinyos-2.x/tos/system/ArbiterP.nc"
uint8_t arg_0x40e33d38);
# 55 "/home/tinyos/local/src/tinyos-2.x/tos/interfaces/ResourceConfigure.nc"
static void /*HplSensirionSht11C.Arbiter.Arbiter*/ArbiterP__1__ResourceConfigure__default__unconfigure(
# 60 "/home/tinyos/local/src/tinyos-2.x/tos/system/ArbiterP.nc"
uint8_t arg_0x40e31148);
# 49 "/home/tinyos/local/src/tinyos-2.x/tos/interfaces/ResourceConfigure.nc"
static void /*HplSensirionSht11C.Arbiter.Arbiter*/ArbiterP__1__ResourceConfigure__default__configure(
# 60 "/home/tinyos/local/src/tinyos-2.x/tos/system/ArbiterP.nc"
uint8_t arg_0x40e31148);
# 56 "/home/tinyos/local/src/tinyos-2.x/tos/interfaces/ResourceDefaultOwner.nc"
static error_t /*HplSensirionSht11C.Arbiter.Arbiter*/ArbiterP__1__ResourceDefaultOwner__release(void );








static bool /*HplSensirionSht11C.Arbiter.Arbiter*/ArbiterP__1__ResourceDefaultOwner__isOwner(void );
# 110 "/home/tinyos/local/src/tinyos-2.x/tos/interfaces/Resource.nc"
static error_t /*HplSensirionSht11C.Arbiter.Arbiter*/ArbiterP__1__Resource__release(
# 54 "/home/tinyos/local/src/tinyos-2.x/tos/system/ArbiterP.nc"
uint8_t arg_0x40e33310);
# 78 "/home/tinyos/local/src/tinyos-2.x/tos/interfaces/Resource.nc"
static error_t /*HplSensirionSht11C.Arbiter.Arbiter*/ArbiterP__1__Resource__request(
# 54 "/home/tinyos/local/src/tinyos-2.x/tos/system/ArbiterP.nc"
uint8_t arg_0x40e33310);
# 92 "/home/tinyos/local/src/tinyos-2.x/tos/interfaces/Resource.nc"
static void /*HplSensirionSht11C.Arbiter.Arbiter*/ArbiterP__1__Resource__default__granted(
# 54 "/home/tinyos/local/src/tinyos-2.x/tos/system/ArbiterP.nc"
uint8_t arg_0x40e33310);
# 64 "/home/tinyos/local/src/tinyos-2.x/tos/interfaces/TaskBasic.nc"
static void /*HplSensirionSht11C.Arbiter.Arbiter*/ArbiterP__1__grantedTask__runTask(void );
# 92 "/home/tinyos/local/src/tinyos-2.x/tos/interfaces/SplitControl.nc"
static void /*HplSensirionSht11C.SplitControlPowerManagerC.PowerManager*/PowerManagerP__0__SplitControl__startDone(error_t error);
#line 117
static void /*HplSensirionSht11C.SplitControlPowerManagerC.PowerManager*/PowerManagerP__0__SplitControl__stopDone(error_t error);
# 52 "/home/tinyos/local/src/tinyos-2.x/tos/lib/power/PowerDownCleanup.nc"
static void /*HplSensirionSht11C.SplitControlPowerManagerC.PowerManager*/PowerManagerP__0__PowerDownCleanup__default__cleanup(void );
# 64 "/home/tinyos/local/src/tinyos-2.x/tos/interfaces/TaskBasic.nc"
static void /*HplSensirionSht11C.SplitControlPowerManagerC.PowerManager*/PowerManagerP__0__stopTask__runTask(void );
# 73 "/home/tinyos/local/src/tinyos-2.x/tos/interfaces/ResourceDefaultOwner.nc"
static void /*HplSensirionSht11C.SplitControlPowerManagerC.PowerManager*/PowerManagerP__0__ResourceDefaultOwner__requested(void );
#line 46
static void /*HplSensirionSht11C.SplitControlPowerManagerC.PowerManager*/PowerManagerP__0__ResourceDefaultOwner__granted(void );
# 64 "/home/tinyos/local/src/tinyos-2.x/tos/interfaces/TaskBasic.nc"
static void /*HplSensirionSht11C.SplitControlPowerManagerC.PowerManager*/PowerManagerP__0__startTask__runTask(void );
# 74 "/home/tinyos/local/src/tinyos-2.x/tos/interfaces/StdControl.nc"
static error_t /*HplSensirionSht11C.SplitControlPowerManagerC.PowerManager*/PowerManagerP__0__StdControl__default__start(void );









static error_t /*HplSensirionSht11C.SplitControlPowerManagerC.PowerManager*/PowerManagerP__0__StdControl__default__stop(void );
# 49 "/home/tinyos/local/src/tinyos-2.x/tos/interfaces/Boot.nc"
static void UDPShellP__Boot__booted(void );
# 71 "/home/tinyos/local/src/tinyos-2.x/tos/lib/timer/Counter.nc"
static void UDPShellP__Uptime__overflow(void );
# 24 "/home/tinyos/local/src/tinyos-2.x/tos/lib/net/blip/interfaces/UDP.nc"
static void UDPShellP__UDP__recvfrom(struct sockaddr_in6 *src, void *payload, 
uint16_t len, struct ip_metadata *meta);
# 11 "/home/tinyos/local/src/tinyos-2.x/tos/lib/net/blip/shell/ShellCommand.nc"
static char *UDPShellP__ShellCommand__default__eval(
# 30 "/home/tinyos/local/src/tinyos-2.x/tos/lib/net/blip/shell/UDPShellP.nc"
uint8_t arg_0x415a4698, 
# 11 "/home/tinyos/local/src/tinyos-2.x/tos/lib/net/blip/shell/ShellCommand.nc"
int argc, char **argv);
# 3 "/home/tinyos/local/src/tinyos-2.x/tos/lib/net/blip/shell/RegisterShellCommand.nc"
static char *UDPShellP__RegisterShellCommand__default__getCommandName(
# 31 "/home/tinyos/local/src/tinyos-2.x/tos/lib/net/blip/shell/UDPShellP.nc"
uint8_t arg_0x415a3650);
# 10 "/home/tinyos/local/src/tinyos-2.x/tos/lib/net/blip/interfaces/ICMPPing.nc"
static void UDPShellP__ICMPPing__pingDone(uint16_t ping_rcv, uint16_t ping_n);
#line 8
static void UDPShellP__ICMPPing__pingReply(struct in6_addr *source, struct icmp_stats *stats);
# 51 "/home/tinyos/local/src/tinyos-2.x/tos/interfaces/Init.nc"
static error_t PlatformP__MoteInit__init(void );
#line 51
static error_t PlatformP__MoteClockInit__init(void );
#line 51
static error_t PlatformP__LedsInit__init(void );
# 10 "/home/tinyos/local/src/tinyos-2.x/tos/platforms/telosa/PlatformP.nc"
static inline error_t PlatformP__Init__init(void );
# 6 "/home/tinyos/local/src/tinyos-2.x/tos/platforms/telosb/MotePlatformC.nc"
static __inline void MotePlatformC__uwait(uint16_t u);




static __inline void MotePlatformC__TOSH_wait(void );




static void MotePlatformC__TOSH_FLASH_M25P_DP_bit(bool set);










static inline void MotePlatformC__TOSH_FLASH_M25P_DP(void );
#line 56
static inline error_t MotePlatformC__Init__init(void );
# 32 "/home/tinyos/local/src/tinyos-2.x/tos/chips/msp430/timer/Msp430ClockInit.nc"
static void Msp430ClockP__Msp430ClockInit__initTimerB(void );
#line 31
static void Msp430ClockP__Msp430ClockInit__initTimerA(void );
#line 29
static void Msp430ClockP__Msp430ClockInit__setupDcoCalibrate(void );
static void Msp430ClockP__Msp430ClockInit__initClocks(void );
# 40 "/home/tinyos/local/src/tinyos-2.x/tos/chips/msp430/timer/Msp430ClockP.nc"
static volatile uint8_t Msp430ClockP__IE1 __asm ("0x0000");
static volatile uint16_t Msp430ClockP__TA0CTL __asm ("0x0160");
static volatile uint16_t Msp430ClockP__TA0IV __asm ("0x012E");
static volatile uint16_t Msp430ClockP__TBCTL __asm ("0x0180");
static volatile uint16_t Msp430ClockP__TBIV __asm ("0x011E");

enum Msp430ClockP____nesc_unnamed4379 {

  Msp430ClockP__ACLK_CALIB_PERIOD = 8, 
  Msp430ClockP__TARGET_DCO_DELTA = 4096 / 32 * Msp430ClockP__ACLK_CALIB_PERIOD
};

static inline mcu_power_t Msp430ClockP__McuPowerOverride__lowestState(void );



static inline void Msp430ClockP__Msp430ClockInit__defaultSetupDcoCalibrate(void );
#line 68
static inline void Msp430ClockP__Msp430ClockInit__defaultInitClocks(void );
#line 89
static inline void Msp430ClockP__Msp430ClockInit__defaultInitTimerA(void );
#line 104
static inline void Msp430ClockP__Msp430ClockInit__defaultInitTimerB(void );
#line 119
static inline void Msp430ClockP__Msp430ClockInit__default__setupDcoCalibrate(void );




static inline void Msp430ClockP__Msp430ClockInit__default__initClocks(void );




static inline void Msp430ClockP__Msp430ClockInit__default__initTimerA(void );




static inline void Msp430ClockP__Msp430ClockInit__default__initTimerB(void );





static inline void Msp430ClockP__startTimerA(void );
#line 152
static inline void Msp430ClockP__startTimerB(void );
#line 164
static void Msp430ClockP__set_dco_calib(int calib);





static inline uint16_t Msp430ClockP__test_calib_busywait_delta(int calib);
#line 193
static inline void Msp430ClockP__busyCalibrateDco(void );
#line 218
static inline error_t Msp430ClockP__Init__init(void );
# 28 "/home/tinyos/local/src/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerEvent.nc"
static void /*Msp430TimerC.Msp430TimerA*/Msp430TimerP__0__Event__fired(
# 40 "/home/tinyos/local/src/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerP.nc"
uint8_t arg_0x406403c8);
# 37 "/home/tinyos/local/src/tinyos-2.x/tos/chips/msp430/timer/Msp430Timer.nc"
static void /*Msp430TimerC.Msp430TimerA*/Msp430TimerP__0__Timer__overflow(void );
# 115 "/home/tinyos/local/src/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerP.nc"
static inline void /*Msp430TimerC.Msp430TimerA*/Msp430TimerP__0__VectorTimerX0__fired(void );




static inline void /*Msp430TimerC.Msp430TimerA*/Msp430TimerP__0__VectorTimerX1__fired(void );





static inline void /*Msp430TimerC.Msp430TimerA*/Msp430TimerP__0__Overflow__fired(void );








static inline void /*Msp430TimerC.Msp430TimerA*/Msp430TimerP__0__Event__default__fired(uint8_t n);
# 28 "/home/tinyos/local/src/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerEvent.nc"
static void /*Msp430TimerC.Msp430TimerB*/Msp430TimerP__1__Event__fired(
# 40 "/home/tinyos/local/src/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerP.nc"
uint8_t arg_0x406403c8);
# 37 "/home/tinyos/local/src/tinyos-2.x/tos/chips/msp430/timer/Msp430Timer.nc"
static void /*Msp430TimerC.Msp430TimerB*/Msp430TimerP__1__Timer__overflow(void );
# 51 "/home/tinyos/local/src/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerP.nc"
static uint16_t /*Msp430TimerC.Msp430TimerB*/Msp430TimerP__1__Timer__get(void );
#line 70
static inline bool /*Msp430TimerC.Msp430TimerB*/Msp430TimerP__1__Timer__isOverflowPending(void );
#line 115
static inline void /*Msp430TimerC.Msp430TimerB*/Msp430TimerP__1__VectorTimerX0__fired(void );




static inline void /*Msp430TimerC.Msp430TimerB*/Msp430TimerP__1__VectorTimerX1__fired(void );





static inline void /*Msp430TimerC.Msp430TimerB*/Msp430TimerP__1__Overflow__fired(void );








static void /*Msp430TimerC.Msp430TimerB*/Msp430TimerP__1__Event__default__fired(uint8_t n);
# 75 "/home/tinyos/local/src/tinyos-2.x/tos/chips/msp430/timer/Msp430Capture.nc"
static void /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0__Capture__captured(uint16_t time);
# 34 "/home/tinyos/local/src/tinyos-2.x/tos/chips/msp430/timer/Msp430Compare.nc"
static void /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0__Compare__fired(void );
# 44 "/home/tinyos/local/src/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
typedef msp430_compare_control_t /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0__cc_t;


static inline /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0__cc_t /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0__int2CC(uint16_t x)  ;
#line 74
static inline /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0__cc_t /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0__Control__getControl(void );
#line 139
static inline uint16_t /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0__Capture__getEvent(void );
#line 169
static void /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0__Event__fired(void );







static inline void /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0__Capture__default__captured(uint16_t n);



static inline void /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0__Compare__default__fired(void );



static inline void /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0__Timer__overflow(void );
# 75 "/home/tinyos/local/src/tinyos-2.x/tos/chips/msp430/timer/Msp430Capture.nc"
static void /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP__1__Capture__captured(uint16_t time);
# 34 "/home/tinyos/local/src/tinyos-2.x/tos/chips/msp430/timer/Msp430Compare.nc"
static void /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP__1__Compare__fired(void );
# 44 "/home/tinyos/local/src/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
typedef msp430_compare_control_t /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP__1__cc_t;


static inline /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP__1__cc_t /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP__1__int2CC(uint16_t x)  ;
#line 74
static inline /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP__1__cc_t /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP__1__Control__getControl(void );
#line 139
static inline uint16_t /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP__1__Capture__getEvent(void );
#line 169
static void /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP__1__Event__fired(void );







static inline void /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP__1__Capture__default__captured(uint16_t n);



static inline void /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP__1__Compare__default__fired(void );



static inline void /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP__1__Timer__overflow(void );
# 75 "/home/tinyos/local/src/tinyos-2.x/tos/chips/msp430/timer/Msp430Capture.nc"
static void /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP__2__Capture__captured(uint16_t time);
# 34 "/home/tinyos/local/src/tinyos-2.x/tos/chips/msp430/timer/Msp430Compare.nc"
static void /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP__2__Compare__fired(void );
# 44 "/home/tinyos/local/src/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
typedef msp430_compare_control_t /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP__2__cc_t;


static inline /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP__2__cc_t /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP__2__int2CC(uint16_t x)  ;
#line 74
static inline /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP__2__cc_t /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP__2__Control__getControl(void );
#line 139
static inline uint16_t /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP__2__Capture__getEvent(void );
#line 169
static void /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP__2__Event__fired(void );







static inline void /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP__2__Capture__default__captured(uint16_t n);



static inline void /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP__2__Compare__default__fired(void );



static inline void /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP__2__Timer__overflow(void );
# 75 "/home/tinyos/local/src/tinyos-2.x/tos/chips/msp430/timer/Msp430Capture.nc"
static void /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Capture__captured(uint16_t time);
# 34 "/home/tinyos/local/src/tinyos-2.x/tos/chips/msp430/timer/Msp430Compare.nc"
static void /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Compare__fired(void );
# 34 "/home/tinyos/local/src/tinyos-2.x/tos/chips/msp430/timer/Msp430Timer.nc"
static uint16_t /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Timer__get(void );
# 44 "/home/tinyos/local/src/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
typedef msp430_compare_control_t /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__cc_t;

static inline uint16_t /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__CC2int(/*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__cc_t x)  ;
static inline /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__cc_t /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__int2CC(uint16_t x)  ;

static inline uint16_t /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__compareControl(void );
#line 74
static inline /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__cc_t /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Control__getControl(void );









static inline void /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Control__clearPendingInterrupt(void );









static inline void /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Control__setControlAsCompare(void );
#line 119
static inline void /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Control__enableEvents(void );




static inline void /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Control__disableEvents(void );
#line 139
static inline uint16_t /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Capture__getEvent(void );




static inline void /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Compare__setEvent(uint16_t x);









static inline void /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Compare__setEventFromNow(uint16_t x);
#line 169
static inline void /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Event__fired(void );







static inline void /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Capture__default__captured(uint16_t n);







static inline void /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Timer__overflow(void );
# 75 "/home/tinyos/local/src/tinyos-2.x/tos/chips/msp430/timer/Msp430Capture.nc"
static void /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__Capture__captured(uint16_t time);
# 34 "/home/tinyos/local/src/tinyos-2.x/tos/chips/msp430/timer/Msp430Compare.nc"
static void /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__Compare__fired(void );
# 44 "/home/tinyos/local/src/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
typedef msp430_compare_control_t /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__cc_t;

static inline uint16_t /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__CC2int(/*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__cc_t x)  ;
static inline /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__cc_t /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__int2CC(uint16_t x)  ;
#line 61
static inline uint16_t /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__captureControl(uint8_t l_cm);
#line 74
static inline /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__cc_t /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__Control__getControl(void );









static inline void /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__Control__clearPendingInterrupt(void );
#line 99
static inline void /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__Control__setControlAsCapture(uint8_t cm);
#line 119
static inline void /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__Control__enableEvents(void );




static inline void /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__Control__disableEvents(void );
#line 139
static inline uint16_t /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__Capture__getEvent(void );
#line 164
static inline void /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__Capture__clearOverflow(void );




static inline void /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__Event__fired(void );
#line 181
static inline void /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__Compare__default__fired(void );



static inline void /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__Timer__overflow(void );
# 75 "/home/tinyos/local/src/tinyos-2.x/tos/chips/msp430/timer/Msp430Capture.nc"
static void /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__Capture__captured(uint16_t time);
# 34 "/home/tinyos/local/src/tinyos-2.x/tos/chips/msp430/timer/Msp430Compare.nc"
static void /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__Compare__fired(void );
# 34 "/home/tinyos/local/src/tinyos-2.x/tos/chips/msp430/timer/Msp430Timer.nc"
static uint16_t /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__Timer__get(void );
# 44 "/home/tinyos/local/src/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
typedef msp430_compare_control_t /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__cc_t;

static inline uint16_t /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__CC2int(/*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__cc_t x)  ;
static inline /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__cc_t /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__int2CC(uint16_t x)  ;

static inline uint16_t /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__compareControl(void );
#line 74
static inline /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__cc_t /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__Control__getControl(void );









static inline void /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__Control__clearPendingInterrupt(void );









static inline void /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__Control__setControlAsCompare(void );
#line 119
static inline void /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__Control__enableEvents(void );




static inline void /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__Control__disableEvents(void );
#line 139
static inline uint16_t /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__Capture__getEvent(void );




static inline void /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__Compare__setEvent(uint16_t x);









static inline void /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__Compare__setEventFromNow(uint16_t x);
#line 169
static inline void /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__Event__fired(void );







static inline void /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__Capture__default__captured(uint16_t n);







static inline void /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__Timer__overflow(void );
# 75 "/home/tinyos/local/src/tinyos-2.x/tos/chips/msp430/timer/Msp430Capture.nc"
static void /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP__6__Capture__captured(uint16_t time);
# 34 "/home/tinyos/local/src/tinyos-2.x/tos/chips/msp430/timer/Msp430Compare.nc"
static void /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP__6__Compare__fired(void );
# 44 "/home/tinyos/local/src/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
typedef msp430_compare_control_t /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP__6__cc_t;


static inline /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP__6__cc_t /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP__6__int2CC(uint16_t x)  ;
#line 74
static inline /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP__6__cc_t /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP__6__Control__getControl(void );
#line 139
static inline uint16_t /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP__6__Capture__getEvent(void );
#line 169
static inline void /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP__6__Event__fired(void );







static inline void /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP__6__Capture__default__captured(uint16_t n);



static inline void /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP__6__Compare__default__fired(void );



static inline void /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP__6__Timer__overflow(void );
# 75 "/home/tinyos/local/src/tinyos-2.x/tos/chips/msp430/timer/Msp430Capture.nc"
static void /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP__7__Capture__captured(uint16_t time);
# 34 "/home/tinyos/local/src/tinyos-2.x/tos/chips/msp430/timer/Msp430Compare.nc"
static void /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP__7__Compare__fired(void );
# 44 "/home/tinyos/local/src/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
typedef msp430_compare_control_t /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP__7__cc_t;


static inline /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP__7__cc_t /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP__7__int2CC(uint16_t x)  ;
#line 74
static inline /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP__7__cc_t /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP__7__Control__getControl(void );
#line 139
static inline uint16_t /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP__7__Capture__getEvent(void );
#line 169
static inline void /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP__7__Event__fired(void );







static inline void /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP__7__Capture__default__captured(uint16_t n);



static inline void /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP__7__Compare__default__fired(void );



static inline void /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP__7__Timer__overflow(void );
# 75 "/home/tinyos/local/src/tinyos-2.x/tos/chips/msp430/timer/Msp430Capture.nc"
static void /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP__8__Capture__captured(uint16_t time);
# 34 "/home/tinyos/local/src/tinyos-2.x/tos/chips/msp430/timer/Msp430Compare.nc"
static void /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP__8__Compare__fired(void );
# 44 "/home/tinyos/local/src/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
typedef msp430_compare_control_t /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP__8__cc_t;


static inline /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP__8__cc_t /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP__8__int2CC(uint16_t x)  ;
#line 74
static inline /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP__8__cc_t /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP__8__Control__getControl(void );
#line 139
static inline uint16_t /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP__8__Capture__getEvent(void );
#line 169
static inline void /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP__8__Event__fired(void );







static inline void /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP__8__Capture__default__captured(uint16_t n);



static inline void /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP__8__Compare__default__fired(void );



static inline void /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP__8__Timer__overflow(void );
# 75 "/home/tinyos/local/src/tinyos-2.x/tos/chips/msp430/timer/Msp430Capture.nc"
static void /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP__9__Capture__captured(uint16_t time);
# 34 "/home/tinyos/local/src/tinyos-2.x/tos/chips/msp430/timer/Msp430Compare.nc"
static void /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP__9__Compare__fired(void );
# 44 "/home/tinyos/local/src/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
typedef msp430_compare_control_t /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP__9__cc_t;


static inline /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP__9__cc_t /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP__9__int2CC(uint16_t x)  ;
#line 74
static inline /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP__9__cc_t /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP__9__Control__getControl(void );
#line 139
static inline uint16_t /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP__9__Capture__getEvent(void );
#line 169
static inline void /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP__9__Event__fired(void );







static inline void /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP__9__Capture__default__captured(uint16_t n);



static inline void /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP__9__Compare__default__fired(void );



static inline void /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP__9__Timer__overflow(void );
# 28 "/home/tinyos/local/src/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerEvent.nc"
static void Msp430TimerCommonP__VectorTimerB1__fired(void );
#line 28
static void Msp430TimerCommonP__VectorTimerA0__fired(void );
#line 28
static void Msp430TimerCommonP__VectorTimerA1__fired(void );
#line 28
static void Msp430TimerCommonP__VectorTimerB0__fired(void );
# 11 "/home/tinyos/local/src/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerCommonP.nc"
void sig_TIMERA0_VECTOR(void ) __attribute((wakeup)) __attribute((interrupt(12)))  ;
void sig_TIMERA1_VECTOR(void ) __attribute((wakeup)) __attribute((interrupt(10)))  ;
void sig_TIMERB0_VECTOR(void ) __attribute((wakeup)) __attribute((interrupt(26)))  ;
void sig_TIMERB1_VECTOR(void ) __attribute((wakeup)) __attribute((interrupt(24)))  ;
# 54 "/home/tinyos/local/src/tinyos-2.x/tos/interfaces/McuPowerOverride.nc"
static mcu_power_t McuSleepC__McuPowerOverride__lowestState(void );
# 51 "/home/tinyos/local/src/tinyos-2.x/tos/chips/msp430/McuSleepC.nc"
bool McuSleepC__dirty = TRUE;
mcu_power_t McuSleepC__powerState = MSP430_POWER_ACTIVE;




const uint16_t McuSleepC__msp430PowerBits[MSP430_POWER_LPM4 + 1] = { 
0, 
0x0010, 
0x0040 + 0x0010, 
0x0080 + 0x0010, 
0x0080 + 0x0040 + 0x0010, 
0x0080 + 0x0040 + 0x0020 + 0x0010 };


static inline mcu_power_t McuSleepC__getPowerState(void );
#line 104
static inline void McuSleepC__computePowerState(void );




static inline void McuSleepC__McuSleep__sleep(void );
# 51 "/home/tinyos/local/src/tinyos-2.x/tos/interfaces/Init.nc"
static error_t RealMainP__SoftwareInit__init(void );
# 49 "/home/tinyos/local/src/tinyos-2.x/tos/interfaces/Boot.nc"
static void RealMainP__Boot__booted(void );
# 51 "/home/tinyos/local/src/tinyos-2.x/tos/interfaces/Init.nc"
static error_t RealMainP__PlatformInit__init(void );
# 46 "/home/tinyos/local/src/tinyos-2.x/tos/interfaces/Scheduler.nc"
static void RealMainP__Scheduler__init(void );
#line 61
static void RealMainP__Scheduler__taskLoop(void );
#line 54
static bool RealMainP__Scheduler__runNextTask(void );
# 52 "/home/tinyos/local/src/tinyos-2.x/tos/system/RealMainP.nc"
int main(void )   ;
# 64 "/home/tinyos/local/src/tinyos-2.x/tos/interfaces/TaskBasic.nc"
static void SchedulerBasicP__TaskBasic__runTask(
# 45 "/home/tinyos/local/src/tinyos-2.x/tos/system/SchedulerBasicP.nc"
uint8_t arg_0x4059ab40);
# 59 "/home/tinyos/local/src/tinyos-2.x/tos/interfaces/McuSleep.nc"
static void SchedulerBasicP__McuSleep__sleep(void );
# 50 "/home/tinyos/local/src/tinyos-2.x/tos/system/SchedulerBasicP.nc"
enum SchedulerBasicP____nesc_unnamed4380 {

  SchedulerBasicP__NUM_TASKS = 20U, 
  SchedulerBasicP__NO_TASK = 255
};

uint8_t SchedulerBasicP__m_head;
uint8_t SchedulerBasicP__m_tail;
uint8_t SchedulerBasicP__m_next[SchedulerBasicP__NUM_TASKS];








static __inline uint8_t SchedulerBasicP__popTask(void );
#line 86
static inline bool SchedulerBasicP__isWaiting(uint8_t id);




static inline bool SchedulerBasicP__pushTask(uint8_t id);
#line 113
static inline void SchedulerBasicP__Scheduler__init(void );









static bool SchedulerBasicP__Scheduler__runNextTask(void );
#line 138
static inline void SchedulerBasicP__Scheduler__taskLoop(void );
#line 159
static error_t SchedulerBasicP__TaskBasic__postTask(uint8_t id);




static void SchedulerBasicP__TaskBasic__default__runTask(uint8_t id);
# 31 "/home/tinyos/local/src/tinyos-2.x/tos/interfaces/GeneralIO.nc"
static void LedsP__Led0__toggle(void );
static bool LedsP__Led0__get(void );


static void LedsP__Led0__makeOutput(void );
#line 29
static void LedsP__Led0__set(void );

static void LedsP__Led1__toggle(void );
static bool LedsP__Led1__get(void );


static void LedsP__Led1__makeOutput(void );
#line 29
static void LedsP__Led1__set(void );

static void LedsP__Led2__toggle(void );
static bool LedsP__Led2__get(void );


static void LedsP__Led2__makeOutput(void );
#line 29
static void LedsP__Led2__set(void );
# 45 "/home/tinyos/local/src/tinyos-2.x/tos/system/LedsP.nc"
static inline error_t LedsP__Init__init(void );
#line 73
static inline void LedsP__Leds__led0Toggle(void );
#line 88
static inline void LedsP__Leds__led1Toggle(void );
#line 103
static inline void LedsP__Leds__led2Toggle(void );




static inline uint8_t LedsP__Leds__get(void );
# 48 "/home/tinyos/local/src/tinyos-2.x/tos/chips/msp430/pins/HplMsp430GeneralIOP.nc"
static inline uint8_t /*HplMsp430GeneralIOC.P10*/HplMsp430GeneralIOP__0__IO__getRaw(void );
static inline bool /*HplMsp430GeneralIOC.P10*/HplMsp430GeneralIOP__0__IO__get(void );
#line 48
static inline uint8_t /*HplMsp430GeneralIOC.P13*/HplMsp430GeneralIOP__3__IO__getRaw(void );
static inline bool /*HplMsp430GeneralIOC.P13*/HplMsp430GeneralIOP__3__IO__get(void );
#line 48
static inline uint8_t /*HplMsp430GeneralIOC.P14*/HplMsp430GeneralIOP__4__IO__getRaw(void );
static inline bool /*HplMsp430GeneralIOC.P14*/HplMsp430GeneralIOP__4__IO__get(void );
static inline void /*HplMsp430GeneralIOC.P14*/HplMsp430GeneralIOP__4__IO__makeInput(void );
#line 45
static void /*HplMsp430GeneralIOC.P15*/HplMsp430GeneralIOP__5__IO__set(void );
static void /*HplMsp430GeneralIOC.P15*/HplMsp430GeneralIOP__5__IO__clr(void );

static inline uint8_t /*HplMsp430GeneralIOC.P15*/HplMsp430GeneralIOP__5__IO__getRaw(void );
static inline bool /*HplMsp430GeneralIOC.P15*/HplMsp430GeneralIOP__5__IO__get(void );
static void /*HplMsp430GeneralIOC.P15*/HplMsp430GeneralIOP__5__IO__makeInput(void );

static void /*HplMsp430GeneralIOC.P15*/HplMsp430GeneralIOP__5__IO__makeOutput(void );
#line 45
static void /*HplMsp430GeneralIOC.P16*/HplMsp430GeneralIOP__6__IO__set(void );
static void /*HplMsp430GeneralIOC.P16*/HplMsp430GeneralIOP__6__IO__clr(void );



static inline void /*HplMsp430GeneralIOC.P16*/HplMsp430GeneralIOP__6__IO__makeInput(void );

static inline void /*HplMsp430GeneralIOC.P16*/HplMsp430GeneralIOP__6__IO__makeOutput(void );
#line 45
static inline void /*HplMsp430GeneralIOC.P17*/HplMsp430GeneralIOP__7__IO__set(void );
static inline void /*HplMsp430GeneralIOC.P17*/HplMsp430GeneralIOP__7__IO__clr(void );





static inline void /*HplMsp430GeneralIOC.P17*/HplMsp430GeneralIOP__7__IO__makeOutput(void );

static inline void /*HplMsp430GeneralIOC.P31*/HplMsp430GeneralIOP__17__IO__selectModuleFunc(void );

static inline void /*HplMsp430GeneralIOC.P31*/HplMsp430GeneralIOP__17__IO__selectIOFunc(void );
#line 54
static inline void /*HplMsp430GeneralIOC.P32*/HplMsp430GeneralIOP__18__IO__selectModuleFunc(void );

static inline void /*HplMsp430GeneralIOC.P32*/HplMsp430GeneralIOP__18__IO__selectIOFunc(void );
#line 54
static inline void /*HplMsp430GeneralIOC.P33*/HplMsp430GeneralIOP__19__IO__selectModuleFunc(void );

static inline void /*HplMsp430GeneralIOC.P33*/HplMsp430GeneralIOP__19__IO__selectIOFunc(void );
#line 56
static inline void /*HplMsp430GeneralIOC.P34*/HplMsp430GeneralIOP__20__IO__selectIOFunc(void );
#line 56
static inline void /*HplMsp430GeneralIOC.P35*/HplMsp430GeneralIOP__21__IO__selectIOFunc(void );
#line 48
static inline uint8_t /*HplMsp430GeneralIOC.P41*/HplMsp430GeneralIOP__25__IO__getRaw(void );
static inline bool /*HplMsp430GeneralIOC.P41*/HplMsp430GeneralIOP__25__IO__get(void );
static inline void /*HplMsp430GeneralIOC.P41*/HplMsp430GeneralIOP__25__IO__makeInput(void );



static inline void /*HplMsp430GeneralIOC.P41*/HplMsp430GeneralIOP__25__IO__selectModuleFunc(void );

static inline void /*HplMsp430GeneralIOC.P41*/HplMsp430GeneralIOP__25__IO__selectIOFunc(void );
#line 45
static void /*HplMsp430GeneralIOC.P42*/HplMsp430GeneralIOP__26__IO__set(void );
static void /*HplMsp430GeneralIOC.P42*/HplMsp430GeneralIOP__26__IO__clr(void );





static inline void /*HplMsp430GeneralIOC.P42*/HplMsp430GeneralIOP__26__IO__makeOutput(void );
#line 45
static inline void /*HplMsp430GeneralIOC.P45*/HplMsp430GeneralIOP__29__IO__set(void );
static inline void /*HplMsp430GeneralIOC.P45*/HplMsp430GeneralIOP__29__IO__clr(void );





static inline void /*HplMsp430GeneralIOC.P45*/HplMsp430GeneralIOP__29__IO__makeOutput(void );
#line 45
static void /*HplMsp430GeneralIOC.P46*/HplMsp430GeneralIOP__30__IO__set(void );
static void /*HplMsp430GeneralIOC.P46*/HplMsp430GeneralIOP__30__IO__clr(void );





static inline void /*HplMsp430GeneralIOC.P46*/HplMsp430GeneralIOP__30__IO__makeOutput(void );
#line 45
static inline void /*HplMsp430GeneralIOC.P54*/HplMsp430GeneralIOP__36__IO__set(void );

static void /*HplMsp430GeneralIOC.P54*/HplMsp430GeneralIOP__36__IO__toggle(void );
static inline uint8_t /*HplMsp430GeneralIOC.P54*/HplMsp430GeneralIOP__36__IO__getRaw(void );
static inline bool /*HplMsp430GeneralIOC.P54*/HplMsp430GeneralIOP__36__IO__get(void );


static inline void /*HplMsp430GeneralIOC.P54*/HplMsp430GeneralIOP__36__IO__makeOutput(void );
#line 45
static inline void /*HplMsp430GeneralIOC.P55*/HplMsp430GeneralIOP__37__IO__set(void );

static inline void /*HplMsp430GeneralIOC.P55*/HplMsp430GeneralIOP__37__IO__toggle(void );
static inline uint8_t /*HplMsp430GeneralIOC.P55*/HplMsp430GeneralIOP__37__IO__getRaw(void );
static inline bool /*HplMsp430GeneralIOC.P55*/HplMsp430GeneralIOP__37__IO__get(void );


static inline void /*HplMsp430GeneralIOC.P55*/HplMsp430GeneralIOP__37__IO__makeOutput(void );
#line 45
static inline void /*HplMsp430GeneralIOC.P56*/HplMsp430GeneralIOP__38__IO__set(void );

static void /*HplMsp430GeneralIOC.P56*/HplMsp430GeneralIOP__38__IO__toggle(void );
static inline uint8_t /*HplMsp430GeneralIOC.P56*/HplMsp430GeneralIOP__38__IO__getRaw(void );
static inline bool /*HplMsp430GeneralIOC.P56*/HplMsp430GeneralIOP__38__IO__get(void );


static inline void /*HplMsp430GeneralIOC.P56*/HplMsp430GeneralIOP__38__IO__makeOutput(void );
# 44 "/home/tinyos/local/src/tinyos-2.x/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
static void /*PlatformLedsC.Led0Impl*/Msp430GpioC__0__HplGeneralIO__toggle(void );
#line 71
static void /*PlatformLedsC.Led0Impl*/Msp430GpioC__0__HplGeneralIO__makeOutput(void );
#line 59
static bool /*PlatformLedsC.Led0Impl*/Msp430GpioC__0__HplGeneralIO__get(void );
#line 34
static void /*PlatformLedsC.Led0Impl*/Msp430GpioC__0__HplGeneralIO__set(void );
# 37 "/home/tinyos/local/src/tinyos-2.x/tos/chips/msp430/pins/Msp430GpioC.nc"
static inline void /*PlatformLedsC.Led0Impl*/Msp430GpioC__0__GeneralIO__set(void );

static inline void /*PlatformLedsC.Led0Impl*/Msp430GpioC__0__GeneralIO__toggle(void );
static inline bool /*PlatformLedsC.Led0Impl*/Msp430GpioC__0__GeneralIO__get(void );


static inline void /*PlatformLedsC.Led0Impl*/Msp430GpioC__0__GeneralIO__makeOutput(void );
# 44 "/home/tinyos/local/src/tinyos-2.x/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
static void /*PlatformLedsC.Led1Impl*/Msp430GpioC__1__HplGeneralIO__toggle(void );
#line 71
static void /*PlatformLedsC.Led1Impl*/Msp430GpioC__1__HplGeneralIO__makeOutput(void );
#line 59
static bool /*PlatformLedsC.Led1Impl*/Msp430GpioC__1__HplGeneralIO__get(void );
#line 34
static void /*PlatformLedsC.Led1Impl*/Msp430GpioC__1__HplGeneralIO__set(void );
# 37 "/home/tinyos/local/src/tinyos-2.x/tos/chips/msp430/pins/Msp430GpioC.nc"
static inline void /*PlatformLedsC.Led1Impl*/Msp430GpioC__1__GeneralIO__set(void );

static inline void /*PlatformLedsC.Led1Impl*/Msp430GpioC__1__GeneralIO__toggle(void );
static inline bool /*PlatformLedsC.Led1Impl*/Msp430GpioC__1__GeneralIO__get(void );


static inline void /*PlatformLedsC.Led1Impl*/Msp430GpioC__1__GeneralIO__makeOutput(void );
# 44 "/home/tinyos/local/src/tinyos-2.x/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
static void /*PlatformLedsC.Led2Impl*/Msp430GpioC__2__HplGeneralIO__toggle(void );
#line 71
static void /*PlatformLedsC.Led2Impl*/Msp430GpioC__2__HplGeneralIO__makeOutput(void );
#line 59
static bool /*PlatformLedsC.Led2Impl*/Msp430GpioC__2__HplGeneralIO__get(void );
#line 34
static void /*PlatformLedsC.Led2Impl*/Msp430GpioC__2__HplGeneralIO__set(void );
# 37 "/home/tinyos/local/src/tinyos-2.x/tos/chips/msp430/pins/Msp430GpioC.nc"
static inline void /*PlatformLedsC.Led2Impl*/Msp430GpioC__2__GeneralIO__set(void );

static inline void /*PlatformLedsC.Led2Impl*/Msp430GpioC__2__GeneralIO__toggle(void );
static inline bool /*PlatformLedsC.Led2Impl*/Msp430GpioC__2__GeneralIO__get(void );


static inline void /*PlatformLedsC.Led2Impl*/Msp430GpioC__2__GeneralIO__makeOutput(void );
# 53 "/home/tinyos/local/src/tinyos-2.x/tos/lib/timer/Timer.nc"
static void TCPEchoP__StatusTimer__startPeriodic(uint32_t dt);








static void TCPEchoP__StatusTimer__startOneShot(uint32_t dt);
# 16 "/home/tinyos/local/src/tinyos-2.x/tos/lib/net/blip/interfaces/UDP.nc"
static error_t TCPEchoP__Status__sendto(struct sockaddr_in6 *dest, void *payload, 
uint16_t len);
#line 10
static error_t TCPEchoP__Status__bind(uint16_t port);
# 83 "/home/tinyos/local/src/tinyos-2.x/tos/interfaces/SplitControl.nc"
static error_t TCPEchoP__RadioControl__start(void );
# 41 "/home/tinyos/local/src/tinyos-2.x/tos/interfaces/Random.nc"
static uint16_t TCPEchoP__Random__rand16(void );
# 16 "/home/tinyos/local/src/tinyos-2.x/tos/lib/net/blip/interfaces/UDP.nc"
static error_t TCPEchoP__Echo__sendto(struct sockaddr_in6 *dest, void *payload, 
uint16_t len);
#line 10
static error_t TCPEchoP__Echo__bind(uint16_t port);
# 34 "/home/tinyos/local/src/tinyos-2.x/tos/lib/net/blip/interfaces/Statistics.nc"
static void TCPEchoP__IPStats__clear(void );
#line 29
static void TCPEchoP__IPStats__get(TCPEchoP__IPStats__stat_str *stats);
# 56 "/home/tinyos/local/src/tinyos-2.x/tos/interfaces/Leds.nc"
static void TCPEchoP__Leds__led0Toggle(void );
#line 89
static void TCPEchoP__Leds__led2Toggle(void );
# 32 "/home/tinyos/local/src/tinyos-2.x/tos/lib/net/blip/interfaces/Tcp.nc"
static error_t TCPEchoP__TcpEcho__send(void *payload, uint16_t len);
#line 9
static error_t TCPEchoP__TcpEcho__bind(uint16_t port);
# 29 "/home/tinyos/local/src/tinyos-2.x/tos/lib/net/blip/interfaces/Statistics.nc"
static void TCPEchoP__UDPStats__get(TCPEchoP__UDPStats__stat_str *stats);




static void TCPEchoP__RouteStats__clear(void );
#line 29
static void TCPEchoP__RouteStats__get(TCPEchoP__RouteStats__stat_str *stats);




static void TCPEchoP__ICMPStats__clear(void );
#line 29
static void TCPEchoP__ICMPStats__get(TCPEchoP__ICMPStats__stat_str *stats);
# 58 "TCPEchoP.nc"
bool TCPEchoP__timerStarted;
nx_struct udp_report TCPEchoP__stats;
struct sockaddr_in6 TCPEchoP__route_dest;







static inline void TCPEchoP__Boot__booted(void );
#line 91
static inline void TCPEchoP__RadioControl__startDone(error_t e);



static inline void TCPEchoP__RadioControl__stopDone(error_t e);



static inline void TCPEchoP__Status__recvfrom(struct sockaddr_in6 *from, void *data, 
uint16_t len, struct ip_metadata *meta);



static inline void TCPEchoP__Echo__recvfrom(struct sockaddr_in6 *from, void *data, 
uint16_t len, struct ip_metadata *meta);




enum TCPEchoP____nesc_unnamed4381 {
  TCPEchoP__STATUS_SIZE = sizeof(ip_statistics_t ) + 
  sizeof(route_statistics_t ) + 
  sizeof(icmp_statistics_t ) + sizeof(udp_statistics_t )
};


static inline void TCPEchoP__StatusTimer__fired(void );
#line 140
char TCPEchoP__tcp_buf[150];

static inline bool TCPEchoP__TcpEcho__accept(struct sockaddr_in6 *from, 
void **tx_buf, int *tx_buf_len);







static inline void TCPEchoP__TcpEcho__recv(void *payload, uint16_t len);



static inline void TCPEchoP__TcpEcho__closed(error_t e);



static inline void TCPEchoP__TcpEcho__acked(void );
# 30 "/home/tinyos/local/src/tinyos-2.x/tos/chips/msp430/timer/Msp430Compare.nc"
static void /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__0__Msp430Compare__setEvent(uint16_t time);

static void /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__0__Msp430Compare__setEventFromNow(uint16_t delta);
# 34 "/home/tinyos/local/src/tinyos-2.x/tos/chips/msp430/timer/Msp430Timer.nc"
static uint16_t /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__0__Msp430Timer__get(void );
# 67 "/home/tinyos/local/src/tinyos-2.x/tos/lib/timer/Alarm.nc"
static void /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__0__Alarm__fired(void );
# 46 "/home/tinyos/local/src/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerControl.nc"
static void /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__0__Msp430TimerControl__enableEvents(void );
#line 36
static void /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__0__Msp430TimerControl__setControlAsCompare(void );










static void /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__0__Msp430TimerControl__disableEvents(void );
#line 33
static void /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__0__Msp430TimerControl__clearPendingInterrupt(void );
# 42 "/home/tinyos/local/src/tinyos-2.x/tos/chips/msp430/timer/Msp430AlarmC.nc"
static inline error_t /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__0__Init__init(void );
#line 54
static inline void /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__0__Alarm__stop(void );




static inline void /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__0__Msp430Compare__fired(void );










static inline void /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__0__Alarm__startAt(uint16_t t0, uint16_t dt);
#line 103
static inline void /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__0__Msp430Timer__overflow(void );
# 34 "/home/tinyos/local/src/tinyos-2.x/tos/chips/msp430/timer/Msp430Timer.nc"
static uint16_t /*Msp430Counter32khzC.Counter*/Msp430CounterC__0__Msp430Timer__get(void );
static bool /*Msp430Counter32khzC.Counter*/Msp430CounterC__0__Msp430Timer__isOverflowPending(void );
# 71 "/home/tinyos/local/src/tinyos-2.x/tos/lib/timer/Counter.nc"
static void /*Msp430Counter32khzC.Counter*/Msp430CounterC__0__Counter__overflow(void );
# 38 "/home/tinyos/local/src/tinyos-2.x/tos/chips/msp430/timer/Msp430CounterC.nc"
static inline uint16_t /*Msp430Counter32khzC.Counter*/Msp430CounterC__0__Counter__get(void );




static inline bool /*Msp430Counter32khzC.Counter*/Msp430CounterC__0__Counter__isOverflowPending(void );









static inline void /*Msp430Counter32khzC.Counter*/Msp430CounterC__0__Msp430Timer__overflow(void );
# 53 "/home/tinyos/local/src/tinyos-2.x/tos/lib/timer/Counter.nc"
static /*CounterMilli32C.Transform*/TransformCounterC__0__CounterFrom__size_type /*CounterMilli32C.Transform*/TransformCounterC__0__CounterFrom__get(void );






static bool /*CounterMilli32C.Transform*/TransformCounterC__0__CounterFrom__isOverflowPending(void );










static void /*CounterMilli32C.Transform*/TransformCounterC__0__Counter__overflow(void );
# 56 "/home/tinyos/local/src/tinyos-2.x/tos/lib/timer/TransformCounterC.nc"
/*CounterMilli32C.Transform*/TransformCounterC__0__upper_count_type /*CounterMilli32C.Transform*/TransformCounterC__0__m_upper;

enum /*CounterMilli32C.Transform*/TransformCounterC__0____nesc_unnamed4382 {

  TransformCounterC__0__LOW_SHIFT_RIGHT = 5, 
  TransformCounterC__0__HIGH_SHIFT_LEFT = 8 * sizeof(/*CounterMilli32C.Transform*/TransformCounterC__0__from_size_type ) - /*CounterMilli32C.Transform*/TransformCounterC__0__LOW_SHIFT_RIGHT, 
  TransformCounterC__0__NUM_UPPER_BITS = 8 * sizeof(/*CounterMilli32C.Transform*/TransformCounterC__0__to_size_type ) - 8 * sizeof(/*CounterMilli32C.Transform*/TransformCounterC__0__from_size_type ) + 5, 



  TransformCounterC__0__OVERFLOW_MASK = /*CounterMilli32C.Transform*/TransformCounterC__0__NUM_UPPER_BITS ? ((/*CounterMilli32C.Transform*/TransformCounterC__0__upper_count_type )2 << (/*CounterMilli32C.Transform*/TransformCounterC__0__NUM_UPPER_BITS - 1)) - 1 : 0
};

static /*CounterMilli32C.Transform*/TransformCounterC__0__to_size_type /*CounterMilli32C.Transform*/TransformCounterC__0__Counter__get(void );
#line 122
static inline void /*CounterMilli32C.Transform*/TransformCounterC__0__CounterFrom__overflow(void );
# 67 "/home/tinyos/local/src/tinyos-2.x/tos/lib/timer/Alarm.nc"
static void /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__Alarm__fired(void );
#line 92
static void /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__AlarmFrom__startAt(/*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__AlarmFrom__size_type t0, /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__AlarmFrom__size_type dt);
#line 62
static void /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__AlarmFrom__stop(void );
# 53 "/home/tinyos/local/src/tinyos-2.x/tos/lib/timer/Counter.nc"
static /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__Counter__size_type /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__Counter__get(void );
# 66 "/home/tinyos/local/src/tinyos-2.x/tos/lib/timer/TransformAlarmC.nc"
/*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__to_size_type /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__m_t0;
/*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__to_size_type /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__m_dt;

enum /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0____nesc_unnamed4383 {

  TransformAlarmC__0__MAX_DELAY_LOG2 = 8 * sizeof(/*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__from_size_type ) - 1 - 5, 
  TransformAlarmC__0__MAX_DELAY = (/*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__to_size_type )1 << /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__MAX_DELAY_LOG2
};

static inline /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__to_size_type /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__Alarm__getNow(void );




static inline /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__to_size_type /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__Alarm__getAlarm(void );










static inline void /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__Alarm__stop(void );




static void /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__set_alarm(void );
#line 136
static void /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__Alarm__startAt(/*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__to_size_type t0, /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__to_size_type dt);
#line 151
static inline void /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__AlarmFrom__fired(void );
#line 166
static inline void /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__Counter__overflow(void );
# 56 "/home/tinyos/local/src/tinyos-2.x/tos/interfaces/TaskBasic.nc"
static error_t /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__fired__postTask(void );
# 98 "/home/tinyos/local/src/tinyos-2.x/tos/lib/timer/Alarm.nc"
static /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__Alarm__size_type /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__Alarm__getNow(void );
#line 92
static void /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__Alarm__startAt(/*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__Alarm__size_type t0, /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__Alarm__size_type dt);
#line 105
static /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__Alarm__size_type /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__Alarm__getAlarm(void );
#line 62
static void /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__Alarm__stop(void );
# 72 "/home/tinyos/local/src/tinyos-2.x/tos/lib/timer/Timer.nc"
static void /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__Timer__fired(void );
# 63 "/home/tinyos/local/src/tinyos-2.x/tos/lib/timer/AlarmToTimerC.nc"
enum /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0____nesc_unnamed4384 {
#line 63
  AlarmToTimerC__0__fired = 0U
};
#line 63
typedef int /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0____nesc_sillytask_fired[/*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__fired];
#line 44
uint32_t /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__m_dt;
bool /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__m_oneshot;

static inline void /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__start(uint32_t t0, uint32_t dt, bool oneshot);
#line 60
static inline void /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__Timer__stop(void );


static inline void /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__fired__runTask(void );






static inline void /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__Alarm__fired(void );
#line 82
static inline void /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__Timer__startOneShotAt(uint32_t t0, uint32_t dt);


static inline uint32_t /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__Timer__getNow(void );
# 56 "/home/tinyos/local/src/tinyos-2.x/tos/interfaces/TaskBasic.nc"
static error_t /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__updateFromTimer__postTask(void );
# 125 "/home/tinyos/local/src/tinyos-2.x/tos/lib/timer/Timer.nc"
static uint32_t /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__TimerFrom__getNow(void );
#line 118
static void /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__TimerFrom__startOneShotAt(uint32_t t0, uint32_t dt);
#line 67
static void /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__TimerFrom__stop(void );




static void /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__Timer__fired(
# 37 "/home/tinyos/local/src/tinyos-2.x/tos/lib/timer/VirtualizeTimerC.nc"
uint8_t arg_0x409729f0);
#line 60
enum /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0____nesc_unnamed4385 {
#line 60
  VirtualizeTimerC__0__updateFromTimer = 1U
};
#line 60
typedef int /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0____nesc_sillytask_updateFromTimer[/*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__updateFromTimer];
#line 42
enum /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0____nesc_unnamed4386 {

  VirtualizeTimerC__0__NUM_TIMERS = 13U, 
  VirtualizeTimerC__0__END_OF_LIST = 255
};








#line 48
typedef struct /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0____nesc_unnamed4387 {

  uint32_t t0;
  uint32_t dt;
  bool isoneshot : 1;
  bool isrunning : 1;
  bool _reserved : 6;
} /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__Timer_t;

/*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__Timer_t /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__m_timers[/*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__NUM_TIMERS];




static void /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__fireTimers(uint32_t now);
#line 89
static inline void /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__updateFromTimer__runTask(void );
#line 128
static inline void /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__TimerFrom__fired(void );




static void /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__startTimer(uint8_t num, uint32_t t0, uint32_t dt, bool isoneshot);









static void /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__Timer__startPeriodic(uint8_t num, uint32_t dt);




static inline void /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__Timer__startOneShot(uint8_t num, uint32_t dt);




static inline void /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__Timer__stop(uint8_t num);




static inline bool /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__Timer__isRunning(uint8_t num);
#line 193
static inline void /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__Timer__default__fired(uint8_t num);
# 53 "/home/tinyos/local/src/tinyos-2.x/tos/lib/timer/Counter.nc"
static /*HilTimerMilliC.CounterToLocalTimeC*/CounterToLocalTimeC__0__Counter__size_type /*HilTimerMilliC.CounterToLocalTimeC*/CounterToLocalTimeC__0__Counter__get(void );
# 42 "/home/tinyos/local/src/tinyos-2.x/tos/lib/timer/CounterToLocalTimeC.nc"
static inline uint32_t /*HilTimerMilliC.CounterToLocalTimeC*/CounterToLocalTimeC__0__LocalTime__get(void );




static inline void /*HilTimerMilliC.CounterToLocalTimeC*/CounterToLocalTimeC__0__Counter__overflow(void );
# 92 "/home/tinyos/local/src/tinyos-2.x/tos/interfaces/SplitControl.nc"
static void CC2420CsmaP__SplitControl__startDone(error_t error);
#line 117
static void CC2420CsmaP__SplitControl__stopDone(error_t error);
# 95 "/home/tinyos/local/src/tinyos-2.x/tos/chips/cc2420/interfaces/RadioBackoff.nc"
static void CC2420CsmaP__RadioBackoff__requestCca(message_t * msg);
#line 81
static void CC2420CsmaP__RadioBackoff__requestInitialBackoff(message_t * msg);






static void CC2420CsmaP__RadioBackoff__requestCongestionBackoff(message_t * msg);
#line 66
static void CC2420CsmaP__SubBackoff__setCongestionBackoff(uint16_t backoffTime);
#line 60
static void CC2420CsmaP__SubBackoff__setInitialBackoff(uint16_t backoffTime);
# 51 "/home/tinyos/local/src/tinyos-2.x/tos/chips/cc2420/interfaces/CC2420Transmit.nc"
static error_t CC2420CsmaP__CC2420Transmit__send(message_t * p_msg, bool useCca);
# 89 "/home/tinyos/local/src/tinyos-2.x/tos/interfaces/Send.nc"
static void CC2420CsmaP__Send__sendDone(
#line 85
message_t * msg, 



error_t error);
# 41 "/home/tinyos/local/src/tinyos-2.x/tos/interfaces/Random.nc"
static uint16_t CC2420CsmaP__Random__rand16(void );
# 74 "/home/tinyos/local/src/tinyos-2.x/tos/interfaces/StdControl.nc"
static error_t CC2420CsmaP__SubControl__start(void );









static error_t CC2420CsmaP__SubControl__stop(void );
# 42 "/home/tinyos/local/src/tinyos-2.x/tos/chips/cc2420/interfaces/CC2420PacketBody.nc"
static cc2420_header_t * CC2420CsmaP__CC2420PacketBody__getHeader(message_t * msg);




static cc2420_metadata_t * CC2420CsmaP__CC2420PacketBody__getMetadata(message_t * msg);
# 71 "/home/tinyos/local/src/tinyos-2.x/tos/chips/cc2420/interfaces/CC2420Power.nc"
static error_t CC2420CsmaP__CC2420Power__startOscillator(void );
#line 90
static error_t CC2420CsmaP__CC2420Power__rxOn(void );
#line 51
static error_t CC2420CsmaP__CC2420Power__startVReg(void );
#line 63
static error_t CC2420CsmaP__CC2420Power__stopVReg(void );
# 110 "/home/tinyos/local/src/tinyos-2.x/tos/interfaces/Resource.nc"
static error_t CC2420CsmaP__Resource__release(void );
#line 78
static error_t CC2420CsmaP__Resource__request(void );
# 66 "/home/tinyos/local/src/tinyos-2.x/tos/interfaces/State.nc"
static bool CC2420CsmaP__SplitControlState__isState(uint8_t myState);
#line 45
static error_t CC2420CsmaP__SplitControlState__requestState(uint8_t reqState);





static void CC2420CsmaP__SplitControlState__forceState(uint8_t reqState);
# 56 "/home/tinyos/local/src/tinyos-2.x/tos/interfaces/TaskBasic.nc"
static error_t CC2420CsmaP__sendDone_task__postTask(void );
#line 56
static error_t CC2420CsmaP__stopDone_task__postTask(void );
#line 56
static error_t CC2420CsmaP__startDone_task__postTask(void );
# 74 "/home/tinyos/local/src/tinyos-2.x/tos/chips/cc2420/csma/CC2420CsmaP.nc"
enum CC2420CsmaP____nesc_unnamed4388 {
#line 74
  CC2420CsmaP__startDone_task = 2U
};
#line 74
typedef int CC2420CsmaP____nesc_sillytask_startDone_task[CC2420CsmaP__startDone_task];
enum CC2420CsmaP____nesc_unnamed4389 {
#line 75
  CC2420CsmaP__stopDone_task = 3U
};
#line 75
typedef int CC2420CsmaP____nesc_sillytask_stopDone_task[CC2420CsmaP__stopDone_task];
enum CC2420CsmaP____nesc_unnamed4390 {
#line 76
  CC2420CsmaP__sendDone_task = 4U
};
#line 76
typedef int CC2420CsmaP____nesc_sillytask_sendDone_task[CC2420CsmaP__sendDone_task];
#line 58
enum CC2420CsmaP____nesc_unnamed4391 {
  CC2420CsmaP__S_STOPPED, 
  CC2420CsmaP__S_STARTING, 
  CC2420CsmaP__S_STARTED, 
  CC2420CsmaP__S_STOPPING, 
  CC2420CsmaP__S_TRANSMITTING
};

message_t * CC2420CsmaP__m_msg;

error_t CC2420CsmaP__sendErr = SUCCESS;


bool CC2420CsmaP__ccaOn;






static inline void CC2420CsmaP__shutdown(void );


static error_t CC2420CsmaP__SplitControl__start(void );
#line 96
static inline error_t CC2420CsmaP__SplitControl__stop(void );
#line 122
static error_t CC2420CsmaP__Send__send(message_t *p_msg, uint8_t len);
#line 202
static inline void CC2420CsmaP__CC2420Transmit__sendDone(message_t *p_msg, error_t err);




static inline void CC2420CsmaP__CC2420Power__startVRegDone(void );



static inline void CC2420CsmaP__Resource__granted(void );



static inline void CC2420CsmaP__CC2420Power__startOscillatorDone(void );




static inline void CC2420CsmaP__SubBackoff__requestInitialBackoff(message_t *msg);






static inline void CC2420CsmaP__SubBackoff__requestCongestionBackoff(message_t *msg);
#line 241
static inline void CC2420CsmaP__sendDone_task__runTask(void );
#line 254
static inline void CC2420CsmaP__startDone_task__runTask(void );







static inline void CC2420CsmaP__stopDone_task__runTask(void );









static inline void CC2420CsmaP__shutdown(void );
#line 285
static inline void CC2420CsmaP__RadioBackoff__default__requestInitialBackoff(message_t *msg);


static inline void CC2420CsmaP__RadioBackoff__default__requestCongestionBackoff(message_t *msg);


static inline void CC2420CsmaP__RadioBackoff__default__requestCca(message_t *msg);
# 53 "/home/tinyos/local/src/tinyos-2.x/tos/chips/cc2420/interfaces/CC2420Config.nc"
static void CC2420ControlP__CC2420Config__syncDone(error_t error);
# 55 "/home/tinyos/local/src/tinyos-2.x/tos/chips/cc2420/interfaces/CC2420Register.nc"
static cc2420_status_t CC2420ControlP__RXCTRL1__write(uint16_t data);
# 55 "/home/tinyos/local/src/tinyos-2.x/tos/lib/timer/Alarm.nc"
static void CC2420ControlP__StartupTimer__start(CC2420ControlP__StartupTimer__size_type dt);
# 55 "/home/tinyos/local/src/tinyos-2.x/tos/chips/cc2420/interfaces/CC2420Register.nc"
static cc2420_status_t CC2420ControlP__MDMCTRL0__write(uint16_t data);
# 35 "/home/tinyos/local/src/tinyos-2.x/tos/interfaces/GeneralIO.nc"
static void CC2420ControlP__RSTN__makeOutput(void );
#line 29
static void CC2420ControlP__RSTN__set(void );
static void CC2420ControlP__RSTN__clr(void );
# 63 "/home/tinyos/local/src/tinyos-2.x/tos/interfaces/Read.nc"
static void CC2420ControlP__ReadRssi__readDone(error_t result, CC2420ControlP__ReadRssi__val_t val);
# 56 "/home/tinyos/local/src/tinyos-2.x/tos/interfaces/TaskBasic.nc"
static error_t CC2420ControlP__syncDone__postTask(void );
# 47 "/home/tinyos/local/src/tinyos-2.x/tos/chips/cc2420/interfaces/CC2420Register.nc"
static cc2420_status_t CC2420ControlP__RSSI__read(uint16_t *data);







static cc2420_status_t CC2420ControlP__IOCFG0__write(uint16_t data);
# 50 "/home/tinyos/local/src/tinyos-2.x/tos/interfaces/ActiveMessageAddress.nc"
static am_addr_t CC2420ControlP__ActiveMessageAddress__amAddress(void );




static am_group_t CC2420ControlP__ActiveMessageAddress__amGroup(void );
# 35 "/home/tinyos/local/src/tinyos-2.x/tos/interfaces/GeneralIO.nc"
static void CC2420ControlP__CSN__makeOutput(void );
#line 29
static void CC2420ControlP__CSN__set(void );
static void CC2420ControlP__CSN__clr(void );




static void CC2420ControlP__VREN__makeOutput(void );
#line 29
static void CC2420ControlP__VREN__set(void );
static void CC2420ControlP__VREN__clr(void );
# 45 "/home/tinyos/local/src/tinyos-2.x/tos/chips/cc2420/interfaces/CC2420Strobe.nc"
static cc2420_status_t CC2420ControlP__SXOSCON__strobe(void );
# 110 "/home/tinyos/local/src/tinyos-2.x/tos/interfaces/Resource.nc"
static error_t CC2420ControlP__SpiResource__release(void );
#line 78
static error_t CC2420ControlP__SpiResource__request(void );
#line 110
static error_t CC2420ControlP__SyncResource__release(void );
#line 78
static error_t CC2420ControlP__SyncResource__request(void );
# 76 "/home/tinyos/local/src/tinyos-2.x/tos/chips/cc2420/interfaces/CC2420Power.nc"
static void CC2420ControlP__CC2420Power__startOscillatorDone(void );
#line 56
static void CC2420ControlP__CC2420Power__startVRegDone(void );
# 55 "/home/tinyos/local/src/tinyos-2.x/tos/chips/cc2420/interfaces/CC2420Register.nc"
static cc2420_status_t CC2420ControlP__IOCFG1__write(uint16_t data);
#line 55
static cc2420_status_t CC2420ControlP__FSCTRL__write(uint16_t data);
# 45 "/home/tinyos/local/src/tinyos-2.x/tos/chips/cc2420/interfaces/CC2420Strobe.nc"
static cc2420_status_t CC2420ControlP__SRXON__strobe(void );
# 92 "/home/tinyos/local/src/tinyos-2.x/tos/interfaces/Resource.nc"
static void CC2420ControlP__Resource__granted(void );
# 63 "/home/tinyos/local/src/tinyos-2.x/tos/chips/cc2420/interfaces/CC2420Ram.nc"
static cc2420_status_t CC2420ControlP__PANID__write(uint8_t offset, uint8_t * data, uint8_t length);
# 50 "/home/tinyos/local/src/tinyos-2.x/tos/interfaces/GpioInterrupt.nc"
static error_t CC2420ControlP__InterruptCCA__disable(void );
#line 42
static error_t CC2420ControlP__InterruptCCA__enableRisingEdge(void );
# 110 "/home/tinyos/local/src/tinyos-2.x/tos/interfaces/Resource.nc"
static error_t CC2420ControlP__RssiResource__release(void );
# 45 "/home/tinyos/local/src/tinyos-2.x/tos/chips/cc2420/interfaces/CC2420Strobe.nc"
static cc2420_status_t CC2420ControlP__SRFOFF__strobe(void );
# 117 "/home/tinyos/local/src/tinyos-2.x/tos/chips/cc2420/control/CC2420ControlP.nc"
enum CC2420ControlP____nesc_unnamed4392 {
#line 117
  CC2420ControlP__sync = 5U
};
#line 117
typedef int CC2420ControlP____nesc_sillytask_sync[CC2420ControlP__sync];
enum CC2420ControlP____nesc_unnamed4393 {
#line 118
  CC2420ControlP__syncDone = 6U
};
#line 118
typedef int CC2420ControlP____nesc_sillytask_syncDone[CC2420ControlP__syncDone];
#line 85
#line 79
typedef enum CC2420ControlP____nesc_unnamed4394 {
  CC2420ControlP__S_VREG_STOPPED, 
  CC2420ControlP__S_VREG_STARTING, 
  CC2420ControlP__S_VREG_STARTED, 
  CC2420ControlP__S_XOSC_STARTING, 
  CC2420ControlP__S_XOSC_STARTED
} CC2420ControlP__cc2420_control_state_t;

uint8_t CC2420ControlP__m_channel;

uint8_t CC2420ControlP__m_tx_power;

uint16_t CC2420ControlP__m_pan;

uint16_t CC2420ControlP__m_short_addr;

bool CC2420ControlP__m_sync_busy;


bool CC2420ControlP__autoAckEnabled;


bool CC2420ControlP__hwAutoAckDefault;


bool CC2420ControlP__addressRecognition;


bool CC2420ControlP__hwAddressRecognition;

CC2420ControlP__cc2420_control_state_t CC2420ControlP__m_state = CC2420ControlP__S_VREG_STOPPED;



static void CC2420ControlP__writeFsctrl(void );
static void CC2420ControlP__writeMdmctrl0(void );
static void CC2420ControlP__writeId(void );





static inline error_t CC2420ControlP__Init__init(void );
#line 171
static inline error_t CC2420ControlP__Resource__request(void );







static inline error_t CC2420ControlP__Resource__release(void );







static inline error_t CC2420ControlP__CC2420Power__startVReg(void );
#line 199
static error_t CC2420ControlP__CC2420Power__stopVReg(void );







static inline error_t CC2420ControlP__CC2420Power__startOscillator(void );
#line 249
static inline error_t CC2420ControlP__CC2420Power__rxOn(void );
#line 279
static uint16_t CC2420ControlP__CC2420Config__getShortAddr(void );







static inline uint16_t CC2420ControlP__CC2420Config__getPanAddr(void );
#line 300
static inline error_t CC2420ControlP__CC2420Config__sync(void );
#line 332
static inline bool CC2420ControlP__CC2420Config__isAddressRecognitionEnabled(void );
#line 359
static inline bool CC2420ControlP__CC2420Config__isHwAutoAckDefault(void );






static inline bool CC2420ControlP__CC2420Config__isAutoAckEnabled(void );









static inline void CC2420ControlP__SyncResource__granted(void );
#line 390
static inline void CC2420ControlP__SpiResource__granted(void );




static inline void CC2420ControlP__RssiResource__granted(void );
#line 408
static inline void CC2420ControlP__StartupTimer__fired(void );









static inline void CC2420ControlP__InterruptCCA__fired(void );
#line 442
static inline void CC2420ControlP__sync__runTask(void );



static inline void CC2420ControlP__syncDone__runTask(void );









static void CC2420ControlP__writeFsctrl(void );
#line 473
static void CC2420ControlP__writeMdmctrl0(void );
#line 492
static void CC2420ControlP__writeId(void );
#line 509
static inline void CC2420ControlP__ReadRssi__default__readDone(error_t error, uint16_t data);
# 30 "/home/tinyos/local/src/tinyos-2.x/tos/chips/msp430/timer/Msp430Compare.nc"
static void /*AlarmMultiplexC.Alarm.Alarm32khz32C.AlarmC.Msp430Alarm*/Msp430AlarmC__1__Msp430Compare__setEvent(uint16_t time);

static void /*AlarmMultiplexC.Alarm.Alarm32khz32C.AlarmC.Msp430Alarm*/Msp430AlarmC__1__Msp430Compare__setEventFromNow(uint16_t delta);
# 34 "/home/tinyos/local/src/tinyos-2.x/tos/chips/msp430/timer/Msp430Timer.nc"
static uint16_t /*AlarmMultiplexC.Alarm.Alarm32khz32C.AlarmC.Msp430Alarm*/Msp430AlarmC__1__Msp430Timer__get(void );
# 67 "/home/tinyos/local/src/tinyos-2.x/tos/lib/timer/Alarm.nc"
static void /*AlarmMultiplexC.Alarm.Alarm32khz32C.AlarmC.Msp430Alarm*/Msp430AlarmC__1__Alarm__fired(void );
# 46 "/home/tinyos/local/src/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerControl.nc"
static void /*AlarmMultiplexC.Alarm.Alarm32khz32C.AlarmC.Msp430Alarm*/Msp430AlarmC__1__Msp430TimerControl__enableEvents(void );
#line 36
static void /*AlarmMultiplexC.Alarm.Alarm32khz32C.AlarmC.Msp430Alarm*/Msp430AlarmC__1__Msp430TimerControl__setControlAsCompare(void );










static void /*AlarmMultiplexC.Alarm.Alarm32khz32C.AlarmC.Msp430Alarm*/Msp430AlarmC__1__Msp430TimerControl__disableEvents(void );
#line 33
static void /*AlarmMultiplexC.Alarm.Alarm32khz32C.AlarmC.Msp430Alarm*/Msp430AlarmC__1__Msp430TimerControl__clearPendingInterrupt(void );
# 42 "/home/tinyos/local/src/tinyos-2.x/tos/chips/msp430/timer/Msp430AlarmC.nc"
static inline error_t /*AlarmMultiplexC.Alarm.Alarm32khz32C.AlarmC.Msp430Alarm*/Msp430AlarmC__1__Init__init(void );
#line 54
static inline void /*AlarmMultiplexC.Alarm.Alarm32khz32C.AlarmC.Msp430Alarm*/Msp430AlarmC__1__Alarm__stop(void );




static inline void /*AlarmMultiplexC.Alarm.Alarm32khz32C.AlarmC.Msp430Alarm*/Msp430AlarmC__1__Msp430Compare__fired(void );










static inline void /*AlarmMultiplexC.Alarm.Alarm32khz32C.AlarmC.Msp430Alarm*/Msp430AlarmC__1__Alarm__startAt(uint16_t t0, uint16_t dt);
#line 103
static inline void /*AlarmMultiplexC.Alarm.Alarm32khz32C.AlarmC.Msp430Alarm*/Msp430AlarmC__1__Msp430Timer__overflow(void );
# 53 "/home/tinyos/local/src/tinyos-2.x/tos/lib/timer/Counter.nc"
static /*Counter32khz32C.Transform*/TransformCounterC__1__CounterFrom__size_type /*Counter32khz32C.Transform*/TransformCounterC__1__CounterFrom__get(void );






static bool /*Counter32khz32C.Transform*/TransformCounterC__1__CounterFrom__isOverflowPending(void );










static void /*Counter32khz32C.Transform*/TransformCounterC__1__Counter__overflow(void );
# 56 "/home/tinyos/local/src/tinyos-2.x/tos/lib/timer/TransformCounterC.nc"
/*Counter32khz32C.Transform*/TransformCounterC__1__upper_count_type /*Counter32khz32C.Transform*/TransformCounterC__1__m_upper;

enum /*Counter32khz32C.Transform*/TransformCounterC__1____nesc_unnamed4395 {

  TransformCounterC__1__LOW_SHIFT_RIGHT = 0, 
  TransformCounterC__1__HIGH_SHIFT_LEFT = 8 * sizeof(/*Counter32khz32C.Transform*/TransformCounterC__1__from_size_type ) - /*Counter32khz32C.Transform*/TransformCounterC__1__LOW_SHIFT_RIGHT, 
  TransformCounterC__1__NUM_UPPER_BITS = 8 * sizeof(/*Counter32khz32C.Transform*/TransformCounterC__1__to_size_type ) - 8 * sizeof(/*Counter32khz32C.Transform*/TransformCounterC__1__from_size_type ) + 0, 



  TransformCounterC__1__OVERFLOW_MASK = /*Counter32khz32C.Transform*/TransformCounterC__1__NUM_UPPER_BITS ? ((/*Counter32khz32C.Transform*/TransformCounterC__1__upper_count_type )2 << (/*Counter32khz32C.Transform*/TransformCounterC__1__NUM_UPPER_BITS - 1)) - 1 : 0
};

static /*Counter32khz32C.Transform*/TransformCounterC__1__to_size_type /*Counter32khz32C.Transform*/TransformCounterC__1__Counter__get(void );
#line 122
static inline void /*Counter32khz32C.Transform*/TransformCounterC__1__CounterFrom__overflow(void );
# 67 "/home/tinyos/local/src/tinyos-2.x/tos/lib/timer/Alarm.nc"
static void /*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC__1__Alarm__fired(void );
#line 92
static void /*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC__1__AlarmFrom__startAt(/*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC__1__AlarmFrom__size_type t0, /*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC__1__AlarmFrom__size_type dt);
#line 62
static void /*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC__1__AlarmFrom__stop(void );
# 53 "/home/tinyos/local/src/tinyos-2.x/tos/lib/timer/Counter.nc"
static /*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC__1__Counter__size_type /*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC__1__Counter__get(void );
# 66 "/home/tinyos/local/src/tinyos-2.x/tos/lib/timer/TransformAlarmC.nc"
/*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC__1__to_size_type /*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC__1__m_t0;
/*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC__1__to_size_type /*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC__1__m_dt;

enum /*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC__1____nesc_unnamed4396 {

  TransformAlarmC__1__MAX_DELAY_LOG2 = 8 * sizeof(/*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC__1__from_size_type ) - 1 - 0, 
  TransformAlarmC__1__MAX_DELAY = (/*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC__1__to_size_type )1 << /*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC__1__MAX_DELAY_LOG2
};

static inline /*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC__1__to_size_type /*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC__1__Alarm__getNow(void );
#line 91
static inline void /*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC__1__Alarm__stop(void );




static void /*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC__1__set_alarm(void );
#line 136
static void /*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC__1__Alarm__startAt(/*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC__1__to_size_type t0, /*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC__1__to_size_type dt);









static inline void /*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC__1__Alarm__start(/*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC__1__to_size_type dt);




static inline void /*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC__1__AlarmFrom__fired(void );
#line 166
static inline void /*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC__1__Counter__overflow(void );
# 64 "/home/tinyos/local/src/tinyos-2.x/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
static void /*HplCC2420PinsC.CCAM*/Msp430GpioC__3__HplGeneralIO__makeInput(void );
#line 59
static bool /*HplCC2420PinsC.CCAM*/Msp430GpioC__3__HplGeneralIO__get(void );
# 40 "/home/tinyos/local/src/tinyos-2.x/tos/chips/msp430/pins/Msp430GpioC.nc"
static inline bool /*HplCC2420PinsC.CCAM*/Msp430GpioC__3__GeneralIO__get(void );
static inline void /*HplCC2420PinsC.CCAM*/Msp430GpioC__3__GeneralIO__makeInput(void );
# 71 "/home/tinyos/local/src/tinyos-2.x/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
static void /*HplCC2420PinsC.CSNM*/Msp430GpioC__4__HplGeneralIO__makeOutput(void );
#line 34
static void /*HplCC2420PinsC.CSNM*/Msp430GpioC__4__HplGeneralIO__set(void );




static void /*HplCC2420PinsC.CSNM*/Msp430GpioC__4__HplGeneralIO__clr(void );
# 37 "/home/tinyos/local/src/tinyos-2.x/tos/chips/msp430/pins/Msp430GpioC.nc"
static inline void /*HplCC2420PinsC.CSNM*/Msp430GpioC__4__GeneralIO__set(void );
static inline void /*HplCC2420PinsC.CSNM*/Msp430GpioC__4__GeneralIO__clr(void );




static inline void /*HplCC2420PinsC.CSNM*/Msp430GpioC__4__GeneralIO__makeOutput(void );
# 59 "/home/tinyos/local/src/tinyos-2.x/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
static bool /*HplCC2420PinsC.FIFOM*/Msp430GpioC__5__HplGeneralIO__get(void );
# 40 "/home/tinyos/local/src/tinyos-2.x/tos/chips/msp430/pins/Msp430GpioC.nc"
static inline bool /*HplCC2420PinsC.FIFOM*/Msp430GpioC__5__GeneralIO__get(void );
# 59 "/home/tinyos/local/src/tinyos-2.x/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
static bool /*HplCC2420PinsC.FIFOPM*/Msp430GpioC__6__HplGeneralIO__get(void );
# 40 "/home/tinyos/local/src/tinyos-2.x/tos/chips/msp430/pins/Msp430GpioC.nc"
static inline bool /*HplCC2420PinsC.FIFOPM*/Msp430GpioC__6__GeneralIO__get(void );
# 71 "/home/tinyos/local/src/tinyos-2.x/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
static void /*HplCC2420PinsC.RSTNM*/Msp430GpioC__7__HplGeneralIO__makeOutput(void );
#line 34
static void /*HplCC2420PinsC.RSTNM*/Msp430GpioC__7__HplGeneralIO__set(void );




static void /*HplCC2420PinsC.RSTNM*/Msp430GpioC__7__HplGeneralIO__clr(void );
# 37 "/home/tinyos/local/src/tinyos-2.x/tos/chips/msp430/pins/Msp430GpioC.nc"
static inline void /*HplCC2420PinsC.RSTNM*/Msp430GpioC__7__GeneralIO__set(void );
static inline void /*HplCC2420PinsC.RSTNM*/Msp430GpioC__7__GeneralIO__clr(void );




static inline void /*HplCC2420PinsC.RSTNM*/Msp430GpioC__7__GeneralIO__makeOutput(void );
# 64 "/home/tinyos/local/src/tinyos-2.x/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
static void /*HplCC2420PinsC.SFDM*/Msp430GpioC__8__HplGeneralIO__makeInput(void );
#line 59
static bool /*HplCC2420PinsC.SFDM*/Msp430GpioC__8__HplGeneralIO__get(void );
# 40 "/home/tinyos/local/src/tinyos-2.x/tos/chips/msp430/pins/Msp430GpioC.nc"
static inline bool /*HplCC2420PinsC.SFDM*/Msp430GpioC__8__GeneralIO__get(void );
static inline void /*HplCC2420PinsC.SFDM*/Msp430GpioC__8__GeneralIO__makeInput(void );
# 71 "/home/tinyos/local/src/tinyos-2.x/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
static void /*HplCC2420PinsC.VRENM*/Msp430GpioC__9__HplGeneralIO__makeOutput(void );
#line 34
static void /*HplCC2420PinsC.VRENM*/Msp430GpioC__9__HplGeneralIO__set(void );




static void /*HplCC2420PinsC.VRENM*/Msp430GpioC__9__HplGeneralIO__clr(void );
# 37 "/home/tinyos/local/src/tinyos-2.x/tos/chips/msp430/pins/Msp430GpioC.nc"
static inline void /*HplCC2420PinsC.VRENM*/Msp430GpioC__9__GeneralIO__set(void );
static inline void /*HplCC2420PinsC.VRENM*/Msp430GpioC__9__GeneralIO__clr(void );




static inline void /*HplCC2420PinsC.VRENM*/Msp430GpioC__9__GeneralIO__makeOutput(void );
# 57 "/home/tinyos/local/src/tinyos-2.x/tos/chips/msp430/timer/Msp430Capture.nc"
static void /*HplCC2420InterruptsC.CaptureSFDC*/GpioCaptureC__0__Msp430Capture__clearOverflow(void );
# 50 "/home/tinyos/local/src/tinyos-2.x/tos/interfaces/GpioCapture.nc"
static void /*HplCC2420InterruptsC.CaptureSFDC*/GpioCaptureC__0__Capture__captured(uint16_t time);
# 44 "/home/tinyos/local/src/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerControl.nc"
static void /*HplCC2420InterruptsC.CaptureSFDC*/GpioCaptureC__0__Msp430TimerControl__setControlAsCapture(uint8_t cm);

static void /*HplCC2420InterruptsC.CaptureSFDC*/GpioCaptureC__0__Msp430TimerControl__enableEvents(void );
static void /*HplCC2420InterruptsC.CaptureSFDC*/GpioCaptureC__0__Msp430TimerControl__disableEvents(void );
#line 33
static void /*HplCC2420InterruptsC.CaptureSFDC*/GpioCaptureC__0__Msp430TimerControl__clearPendingInterrupt(void );
# 85 "/home/tinyos/local/src/tinyos-2.x/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
static void /*HplCC2420InterruptsC.CaptureSFDC*/GpioCaptureC__0__GeneralIO__selectIOFunc(void );
#line 78
static void /*HplCC2420InterruptsC.CaptureSFDC*/GpioCaptureC__0__GeneralIO__selectModuleFunc(void );
# 38 "/home/tinyos/local/src/tinyos-2.x/tos/chips/msp430/timer/GpioCaptureC.nc"
static error_t /*HplCC2420InterruptsC.CaptureSFDC*/GpioCaptureC__0__enableCapture(uint8_t mode);
#line 50
static inline error_t /*HplCC2420InterruptsC.CaptureSFDC*/GpioCaptureC__0__Capture__captureRisingEdge(void );



static inline error_t /*HplCC2420InterruptsC.CaptureSFDC*/GpioCaptureC__0__Capture__captureFallingEdge(void );



static inline void /*HplCC2420InterruptsC.CaptureSFDC*/GpioCaptureC__0__Capture__disable(void );






static inline void /*HplCC2420InterruptsC.CaptureSFDC*/GpioCaptureC__0__Msp430Capture__captured(uint16_t time);
# 61 "/home/tinyos/local/src/tinyos-2.x/tos/chips/msp430/pins/HplMsp430Interrupt.nc"
static void HplMsp430InterruptP__Port14__fired(void );
#line 61
static void HplMsp430InterruptP__Port26__fired(void );
#line 61
static void HplMsp430InterruptP__Port17__fired(void );
#line 61
static void HplMsp430InterruptP__Port21__fired(void );
#line 61
static void HplMsp430InterruptP__Port12__fired(void );
#line 61
static void HplMsp430InterruptP__Port24__fired(void );
#line 61
static void HplMsp430InterruptP__Port15__fired(void );
#line 61
static void HplMsp430InterruptP__Port27__fired(void );
#line 61
static void HplMsp430InterruptP__Port10__fired(void );
#line 61
static void HplMsp430InterruptP__Port22__fired(void );
#line 61
static void HplMsp430InterruptP__Port13__fired(void );
#line 61
static void HplMsp430InterruptP__Port25__fired(void );
#line 61
static void HplMsp430InterruptP__Port16__fired(void );
#line 61
static void HplMsp430InterruptP__Port20__fired(void );
#line 61
static void HplMsp430InterruptP__Port11__fired(void );
#line 61
static void HplMsp430InterruptP__Port23__fired(void );
# 53 "/home/tinyos/local/src/tinyos-2.x/tos/chips/msp430/pins/HplMsp430InterruptP.nc"
void sig_PORT1_VECTOR(void ) __attribute((wakeup)) __attribute((interrupt(8)))  ;
#line 68
static inline void HplMsp430InterruptP__Port11__default__fired(void );
static inline void HplMsp430InterruptP__Port12__default__fired(void );
static inline void HplMsp430InterruptP__Port13__default__fired(void );


static inline void HplMsp430InterruptP__Port16__default__fired(void );
static inline void HplMsp430InterruptP__Port17__default__fired(void );
static inline void HplMsp430InterruptP__Port10__enable(void );



static inline void HplMsp430InterruptP__Port14__enable(void );
static inline void HplMsp430InterruptP__Port15__enable(void );


static inline void HplMsp430InterruptP__Port10__disable(void );



static inline void HplMsp430InterruptP__Port14__disable(void );
static inline void HplMsp430InterruptP__Port15__disable(void );


static inline void HplMsp430InterruptP__Port10__clear(void );
static inline void HplMsp430InterruptP__Port11__clear(void );
static inline void HplMsp430InterruptP__Port12__clear(void );
static inline void HplMsp430InterruptP__Port13__clear(void );
static inline void HplMsp430InterruptP__Port14__clear(void );
static inline void HplMsp430InterruptP__Port15__clear(void );
static inline void HplMsp430InterruptP__Port16__clear(void );
static inline void HplMsp430InterruptP__Port17__clear(void );








static inline void HplMsp430InterruptP__Port10__edge(bool l2h);
#line 131
static inline void HplMsp430InterruptP__Port14__edge(bool l2h);





static inline void HplMsp430InterruptP__Port15__edge(bool l2h);
#line 158
void sig_PORT2_VECTOR(void ) __attribute((wakeup)) __attribute((interrupt(2)))  ;
#line 171
static inline void HplMsp430InterruptP__Port20__default__fired(void );
static inline void HplMsp430InterruptP__Port21__default__fired(void );
static inline void HplMsp430InterruptP__Port22__default__fired(void );
static inline void HplMsp430InterruptP__Port23__default__fired(void );
static inline void HplMsp430InterruptP__Port24__default__fired(void );
static inline void HplMsp430InterruptP__Port25__default__fired(void );
static inline void HplMsp430InterruptP__Port26__default__fired(void );
static inline void HplMsp430InterruptP__Port27__default__fired(void );
#line 195
static inline void HplMsp430InterruptP__Port20__clear(void );
static inline void HplMsp430InterruptP__Port21__clear(void );
static inline void HplMsp430InterruptP__Port22__clear(void );
static inline void HplMsp430InterruptP__Port23__clear(void );
static inline void HplMsp430InterruptP__Port24__clear(void );
static inline void HplMsp430InterruptP__Port25__clear(void );
static inline void HplMsp430InterruptP__Port26__clear(void );
static inline void HplMsp430InterruptP__Port27__clear(void );
# 41 "/home/tinyos/local/src/tinyos-2.x/tos/chips/msp430/pins/HplMsp430Interrupt.nc"
static void /*HplCC2420InterruptsC.InterruptCCAC*/Msp430InterruptC__0__HplInterrupt__clear(void );
#line 36
static void /*HplCC2420InterruptsC.InterruptCCAC*/Msp430InterruptC__0__HplInterrupt__disable(void );
#line 56
static void /*HplCC2420InterruptsC.InterruptCCAC*/Msp430InterruptC__0__HplInterrupt__edge(bool low_to_high);
#line 31
static void /*HplCC2420InterruptsC.InterruptCCAC*/Msp430InterruptC__0__HplInterrupt__enable(void );
# 57 "/home/tinyos/local/src/tinyos-2.x/tos/interfaces/GpioInterrupt.nc"
static void /*HplCC2420InterruptsC.InterruptCCAC*/Msp430InterruptC__0__Interrupt__fired(void );
# 41 "/home/tinyos/local/src/tinyos-2.x/tos/chips/msp430/pins/Msp430InterruptC.nc"
static inline error_t /*HplCC2420InterruptsC.InterruptCCAC*/Msp430InterruptC__0__enable(bool rising);








static inline error_t /*HplCC2420InterruptsC.InterruptCCAC*/Msp430InterruptC__0__Interrupt__enableRisingEdge(void );







static inline error_t /*HplCC2420InterruptsC.InterruptCCAC*/Msp430InterruptC__0__Interrupt__disable(void );







static inline void /*HplCC2420InterruptsC.InterruptCCAC*/Msp430InterruptC__0__HplInterrupt__fired(void );
# 41 "/home/tinyos/local/src/tinyos-2.x/tos/chips/msp430/pins/HplMsp430Interrupt.nc"
static void /*HplCC2420InterruptsC.InterruptFIFOPC*/Msp430InterruptC__1__HplInterrupt__clear(void );
#line 36
static void /*HplCC2420InterruptsC.InterruptFIFOPC*/Msp430InterruptC__1__HplInterrupt__disable(void );
#line 56
static void /*HplCC2420InterruptsC.InterruptFIFOPC*/Msp430InterruptC__1__HplInterrupt__edge(bool low_to_high);
#line 31
static void /*HplCC2420InterruptsC.InterruptFIFOPC*/Msp430InterruptC__1__HplInterrupt__enable(void );
# 57 "/home/tinyos/local/src/tinyos-2.x/tos/interfaces/GpioInterrupt.nc"
static void /*HplCC2420InterruptsC.InterruptFIFOPC*/Msp430InterruptC__1__Interrupt__fired(void );
# 41 "/home/tinyos/local/src/tinyos-2.x/tos/chips/msp430/pins/Msp430InterruptC.nc"
static inline error_t /*HplCC2420InterruptsC.InterruptFIFOPC*/Msp430InterruptC__1__enable(bool rising);
#line 54
static inline error_t /*HplCC2420InterruptsC.InterruptFIFOPC*/Msp430InterruptC__1__Interrupt__enableFallingEdge(void );



static inline error_t /*HplCC2420InterruptsC.InterruptFIFOPC*/Msp430InterruptC__1__Interrupt__disable(void );







static inline void /*HplCC2420InterruptsC.InterruptFIFOPC*/Msp430InterruptC__1__HplInterrupt__fired(void );
# 59 "/home/tinyos/local/src/tinyos-2.x/tos/interfaces/SpiPacket.nc"
static error_t CC2420SpiP__SpiPacket__send(
#line 48
uint8_t * txBuf, 

uint8_t * rxBuf, 








uint16_t len);
# 91 "/home/tinyos/local/src/tinyos-2.x/tos/chips/cc2420/interfaces/CC2420Fifo.nc"
static void CC2420SpiP__Fifo__writeDone(
# 46 "/home/tinyos/local/src/tinyos-2.x/tos/chips/cc2420/spi/CC2420SpiP.nc"
uint8_t arg_0x40bbb010, 
# 91 "/home/tinyos/local/src/tinyos-2.x/tos/chips/cc2420/interfaces/CC2420Fifo.nc"
uint8_t * data, uint8_t length, error_t error);
#line 71
static void CC2420SpiP__Fifo__readDone(
# 46 "/home/tinyos/local/src/tinyos-2.x/tos/chips/cc2420/spi/CC2420SpiP.nc"
uint8_t arg_0x40bbb010, 
# 71 "/home/tinyos/local/src/tinyos-2.x/tos/chips/cc2420/interfaces/CC2420Fifo.nc"
uint8_t * data, uint8_t length, error_t error);
# 24 "/home/tinyos/local/src/tinyos-2.x/tos/chips/cc2420/interfaces/ChipSpiResource.nc"
static void CC2420SpiP__ChipSpiResource__releasing(void );
# 34 "/home/tinyos/local/src/tinyos-2.x/tos/interfaces/SpiByte.nc"
static uint8_t CC2420SpiP__SpiByte__write(uint8_t tx);
# 56 "/home/tinyos/local/src/tinyos-2.x/tos/interfaces/State.nc"
static void CC2420SpiP__WorkingState__toIdle(void );




static bool CC2420SpiP__WorkingState__isIdle(void );
#line 45
static error_t CC2420SpiP__WorkingState__requestState(uint8_t reqState);
# 110 "/home/tinyos/local/src/tinyos-2.x/tos/interfaces/Resource.nc"
static error_t CC2420SpiP__SpiResource__release(void );
#line 87
static error_t CC2420SpiP__SpiResource__immediateRequest(void );
#line 78
static error_t CC2420SpiP__SpiResource__request(void );
#line 118
static bool CC2420SpiP__SpiResource__isOwner(void );
#line 92
static void CC2420SpiP__Resource__granted(
# 45 "/home/tinyos/local/src/tinyos-2.x/tos/chips/cc2420/spi/CC2420SpiP.nc"
uint8_t arg_0x40bbc5c8);
# 56 "/home/tinyos/local/src/tinyos-2.x/tos/interfaces/TaskBasic.nc"
static error_t CC2420SpiP__grant__postTask(void );
# 88 "/home/tinyos/local/src/tinyos-2.x/tos/chips/cc2420/spi/CC2420SpiP.nc"
enum CC2420SpiP____nesc_unnamed4397 {
#line 88
  CC2420SpiP__grant = 7U
};
#line 88
typedef int CC2420SpiP____nesc_sillytask_grant[CC2420SpiP__grant];
#line 63
enum CC2420SpiP____nesc_unnamed4398 {
  CC2420SpiP__RESOURCE_COUNT = 5U, 
  CC2420SpiP__NO_HOLDER = 0xFF
};


enum CC2420SpiP____nesc_unnamed4399 {
  CC2420SpiP__S_IDLE, 
  CC2420SpiP__S_BUSY
};


uint16_t CC2420SpiP__m_addr;


uint8_t CC2420SpiP__m_requests = 0;


uint8_t CC2420SpiP__m_holder = CC2420SpiP__NO_HOLDER;


bool CC2420SpiP__release;


static error_t CC2420SpiP__attemptRelease(void );







static inline void CC2420SpiP__ChipSpiResource__abortRelease(void );






static inline error_t CC2420SpiP__ChipSpiResource__attemptRelease(void );




static error_t CC2420SpiP__Resource__request(uint8_t id);
#line 126
static error_t CC2420SpiP__Resource__immediateRequest(uint8_t id);
#line 149
static error_t CC2420SpiP__Resource__release(uint8_t id);
#line 178
static inline uint8_t CC2420SpiP__Resource__isOwner(uint8_t id);





static inline void CC2420SpiP__SpiResource__granted(void );




static cc2420_status_t CC2420SpiP__Fifo__beginRead(uint8_t addr, uint8_t *data, 
uint8_t len);
#line 209
static inline error_t CC2420SpiP__Fifo__continueRead(uint8_t addr, uint8_t *data, 
uint8_t len);



static inline cc2420_status_t CC2420SpiP__Fifo__write(uint8_t addr, uint8_t *data, 
uint8_t len);
#line 260
static cc2420_status_t CC2420SpiP__Ram__write(uint16_t addr, uint8_t offset, 
uint8_t *data, 
uint8_t len);
#line 287
static inline cc2420_status_t CC2420SpiP__Reg__read(uint8_t addr, uint16_t *data);
#line 305
static cc2420_status_t CC2420SpiP__Reg__write(uint8_t addr, uint16_t data);
#line 318
static cc2420_status_t CC2420SpiP__Strobe__strobe(uint8_t addr);










static void CC2420SpiP__SpiPacket__sendDone(uint8_t *tx_buf, uint8_t *rx_buf, 
uint16_t len, error_t error);








static error_t CC2420SpiP__attemptRelease(void );
#line 358
static inline void CC2420SpiP__grant__runTask(void );








static inline void CC2420SpiP__Resource__default__granted(uint8_t id);


static inline void CC2420SpiP__Fifo__default__readDone(uint8_t addr, uint8_t *rx_buf, uint8_t rx_len, error_t error);


static inline void CC2420SpiP__Fifo__default__writeDone(uint8_t addr, uint8_t *tx_buf, uint8_t tx_len, error_t error);
# 74 "/home/tinyos/local/src/tinyos-2.x/tos/system/StateImplP.nc"
uint8_t StateImplP__state[5U];

enum StateImplP____nesc_unnamed4400 {
  StateImplP__S_IDLE = 0
};


static inline error_t StateImplP__Init__init(void );
#line 96
static error_t StateImplP__State__requestState(uint8_t id, uint8_t reqState);
#line 111
static inline void StateImplP__State__forceState(uint8_t id, uint8_t reqState);






static inline void StateImplP__State__toIdle(uint8_t id);







static inline bool StateImplP__State__isIdle(uint8_t id);






static bool StateImplP__State__isState(uint8_t id, uint8_t myState);









static uint8_t StateImplP__State__getState(uint8_t id);
# 71 "/home/tinyos/local/src/tinyos-2.x/tos/interfaces/SpiPacket.nc"
static void /*Msp430SpiDma0P.SpiP*/Msp430SpiDmaP__0__SpiPacket__sendDone(
# 51 "/home/tinyos/local/src/tinyos-2.x/tos/chips/msp430/usart/Msp430SpiDmaP.nc"
uint8_t arg_0x40c6e740, 
# 64 "/home/tinyos/local/src/tinyos-2.x/tos/interfaces/SpiPacket.nc"
uint8_t * txBuf, 
uint8_t * rxBuf, 





uint16_t len, 
error_t error);
# 39 "/home/tinyos/local/src/tinyos-2.x/tos/chips/msp430/usart/Msp430SpiConfigure.nc"
static msp430_spi_union_config_t */*Msp430SpiDma0P.SpiP*/Msp430SpiDmaP__0__Msp430SpiConfigure__getConfig(
# 56 "/home/tinyos/local/src/tinyos-2.x/tos/chips/msp430/usart/Msp430SpiDmaP.nc"
uint8_t arg_0x40c6c418);
# 197 "/home/tinyos/local/src/tinyos-2.x/tos/chips/msp430/usart/HplMsp430Usart.nc"
static void /*Msp430SpiDma0P.SpiP*/Msp430SpiDmaP__0__Usart__clrRxIntr(void );
#line 97
static void /*Msp430SpiDma0P.SpiP*/Msp430SpiDmaP__0__Usart__resetUsart(bool reset);
#line 224
static void /*Msp430SpiDma0P.SpiP*/Msp430SpiDmaP__0__Usart__tx(uint8_t data);
#line 168
static void /*Msp430SpiDma0P.SpiP*/Msp430SpiDmaP__0__Usart__setModeSpi(msp430_spi_union_config_t *config);
#line 231
static uint8_t /*Msp430SpiDma0P.SpiP*/Msp430SpiDmaP__0__Usart__rx(void );
#line 192
static bool /*Msp430SpiDma0P.SpiP*/Msp430SpiDmaP__0__Usart__isRxIntrPending(void );
#line 158
static void /*Msp430SpiDma0P.SpiP*/Msp430SpiDmaP__0__Usart__disableSpi(void );
# 38 "/home/tinyos/local/src/tinyos-2.x/tos/chips/msp430/dma/Msp430DmaChannel.nc"
static error_t /*Msp430SpiDma0P.SpiP*/Msp430SpiDmaP__0__DmaChannel1__setupTransfer(dma_transfer_mode_t transfer_mode, 
dma_trigger_t trigger, 
dma_level_t level, 
void *src_addr, 
void *dst_addr, 
uint16_t size, 
dma_byte_t src_byte, 
dma_byte_t dst_byte, 
dma_incr_t src_incr, 
dma_incr_t dst_incr);
#line 73
static error_t /*Msp430SpiDma0P.SpiP*/Msp430SpiDmaP__0__DmaChannel1__startTransfer(void );
# 110 "/home/tinyos/local/src/tinyos-2.x/tos/interfaces/Resource.nc"
static error_t /*Msp430SpiDma0P.SpiP*/Msp430SpiDmaP__0__UsartResource__release(
# 55 "/home/tinyos/local/src/tinyos-2.x/tos/chips/msp430/usart/Msp430SpiDmaP.nc"
uint8_t arg_0x40c6d9a0);
# 87 "/home/tinyos/local/src/tinyos-2.x/tos/interfaces/Resource.nc"
static error_t /*Msp430SpiDma0P.SpiP*/Msp430SpiDmaP__0__UsartResource__immediateRequest(
# 55 "/home/tinyos/local/src/tinyos-2.x/tos/chips/msp430/usart/Msp430SpiDmaP.nc"
uint8_t arg_0x40c6d9a0);
# 78 "/home/tinyos/local/src/tinyos-2.x/tos/interfaces/Resource.nc"
static error_t /*Msp430SpiDma0P.SpiP*/Msp430SpiDmaP__0__UsartResource__request(
# 55 "/home/tinyos/local/src/tinyos-2.x/tos/chips/msp430/usart/Msp430SpiDmaP.nc"
uint8_t arg_0x40c6d9a0);
# 118 "/home/tinyos/local/src/tinyos-2.x/tos/interfaces/Resource.nc"
static bool /*Msp430SpiDma0P.SpiP*/Msp430SpiDmaP__0__UsartResource__isOwner(
# 55 "/home/tinyos/local/src/tinyos-2.x/tos/chips/msp430/usart/Msp430SpiDmaP.nc"
uint8_t arg_0x40c6d9a0);
# 92 "/home/tinyos/local/src/tinyos-2.x/tos/interfaces/Resource.nc"
static void /*Msp430SpiDma0P.SpiP*/Msp430SpiDmaP__0__Resource__granted(
# 48 "/home/tinyos/local/src/tinyos-2.x/tos/chips/msp430/usart/Msp430SpiDmaP.nc"
uint8_t arg_0x40c701b8);
# 38 "/home/tinyos/local/src/tinyos-2.x/tos/chips/msp430/dma/Msp430DmaChannel.nc"
static error_t /*Msp430SpiDma0P.SpiP*/Msp430SpiDmaP__0__DmaChannel2__setupTransfer(dma_transfer_mode_t transfer_mode, 
dma_trigger_t trigger, 
dma_level_t level, 
void *src_addr, 
void *dst_addr, 
uint16_t size, 
dma_byte_t src_byte, 
dma_byte_t dst_byte, 
dma_incr_t src_incr, 
dma_incr_t dst_incr);
#line 73
static error_t /*Msp430SpiDma0P.SpiP*/Msp430SpiDmaP__0__DmaChannel2__startTransfer(void );
# 56 "/home/tinyos/local/src/tinyos-2.x/tos/interfaces/TaskBasic.nc"
static error_t /*Msp430SpiDma0P.SpiP*/Msp430SpiDmaP__0__signalDone_task__postTask(void );
# 74 "/home/tinyos/local/src/tinyos-2.x/tos/chips/msp430/usart/Msp430SpiDmaP.nc"
enum /*Msp430SpiDma0P.SpiP*/Msp430SpiDmaP__0____nesc_unnamed4401 {
#line 74
  Msp430SpiDmaP__0__signalDone_task = 8U
};
#line 74
typedef int /*Msp430SpiDma0P.SpiP*/Msp430SpiDmaP__0____nesc_sillytask_signalDone_task[/*Msp430SpiDma0P.SpiP*/Msp430SpiDmaP__0__signalDone_task];
#line 67
uint8_t */*Msp430SpiDma0P.SpiP*/Msp430SpiDmaP__0__m_tx_buf;
uint8_t */*Msp430SpiDma0P.SpiP*/Msp430SpiDmaP__0__m_rx_buf;
uint16_t /*Msp430SpiDma0P.SpiP*/Msp430SpiDmaP__0__m_len;
uint8_t /*Msp430SpiDma0P.SpiP*/Msp430SpiDmaP__0__m_client;
uint8_t /*Msp430SpiDma0P.SpiP*/Msp430SpiDmaP__0__m_dump;

static inline void /*Msp430SpiDma0P.SpiP*/Msp430SpiDmaP__0__signalDone(error_t error);


static inline error_t /*Msp430SpiDma0P.SpiP*/Msp430SpiDmaP__0__Resource__immediateRequest(uint8_t id);



static inline error_t /*Msp430SpiDma0P.SpiP*/Msp430SpiDmaP__0__Resource__request(uint8_t id);



static inline error_t /*Msp430SpiDma0P.SpiP*/Msp430SpiDmaP__0__Resource__release(uint8_t id);



static inline void /*Msp430SpiDma0P.SpiP*/Msp430SpiDmaP__0__ResourceConfigure__configure(uint8_t id);



static inline void /*Msp430SpiDma0P.SpiP*/Msp430SpiDmaP__0__ResourceConfigure__unconfigure(uint8_t id);





static inline void /*Msp430SpiDma0P.SpiP*/Msp430SpiDmaP__0__UsartResource__granted(uint8_t id);



static inline uint8_t /*Msp430SpiDma0P.SpiP*/Msp430SpiDmaP__0__Resource__isOwner(uint8_t id);



static inline error_t /*Msp430SpiDma0P.SpiP*/Msp430SpiDmaP__0__UsartResource__default__isOwner(uint8_t id);
static inline error_t /*Msp430SpiDma0P.SpiP*/Msp430SpiDmaP__0__UsartResource__default__request(uint8_t id);
static inline error_t /*Msp430SpiDma0P.SpiP*/Msp430SpiDmaP__0__UsartResource__default__immediateRequest(uint8_t id);
static inline error_t /*Msp430SpiDma0P.SpiP*/Msp430SpiDmaP__0__UsartResource__default__release(uint8_t id);
static inline msp430_spi_union_config_t */*Msp430SpiDma0P.SpiP*/Msp430SpiDmaP__0__Msp430SpiConfigure__default__getConfig(uint8_t id);



static inline void /*Msp430SpiDma0P.SpiP*/Msp430SpiDmaP__0__Resource__default__granted(uint8_t id);

static uint8_t /*Msp430SpiDma0P.SpiP*/Msp430SpiDmaP__0__SpiByte__write(uint8_t tx);








static error_t /*Msp430SpiDma0P.SpiP*/Msp430SpiDmaP__0__SpiPacket__send(uint8_t id, uint8_t *tx_buf, 
uint8_t *rx_buf, 
uint16_t len);
#line 180
static inline void /*Msp430SpiDma0P.SpiP*/Msp430SpiDmaP__0__signalDone_task__runTask(void );



static inline void /*Msp430SpiDma0P.SpiP*/Msp430SpiDmaP__0__DmaChannel1__transferDone(error_t error);



static inline void /*Msp430SpiDma0P.SpiP*/Msp430SpiDmaP__0__DmaChannel2__transferDone(error_t error);

static inline void /*Msp430SpiDma0P.SpiP*/Msp430SpiDmaP__0__signalDone(error_t error);



static inline void /*Msp430SpiDma0P.SpiP*/Msp430SpiDmaP__0__UsartInterrupts__txDone(void );
static inline void /*Msp430SpiDma0P.SpiP*/Msp430SpiDmaP__0__UsartInterrupts__rxDone(uint8_t data);

static inline void /*Msp430SpiDma0P.SpiP*/Msp430SpiDmaP__0__SpiPacket__default__sendDone(uint8_t id, uint8_t *tx_buf, uint8_t *rx_buf, uint16_t len, error_t error);
# 85 "/home/tinyos/local/src/tinyos-2.x/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
static void HplMsp430Usart0P__UCLK__selectIOFunc(void );
#line 78
static void HplMsp430Usart0P__UCLK__selectModuleFunc(void );
# 54 "/home/tinyos/local/src/tinyos-2.x/tos/chips/msp430/usart/HplMsp430UsartInterrupts.nc"
static void HplMsp430Usart0P__Interrupts__rxDone(uint8_t data);
#line 49
static void HplMsp430Usart0P__Interrupts__txDone(void );
# 85 "/home/tinyos/local/src/tinyos-2.x/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
static void HplMsp430Usart0P__URXD__selectIOFunc(void );
#line 85
static void HplMsp430Usart0P__UTXD__selectIOFunc(void );
# 7 "/home/tinyos/local/src/tinyos-2.x/tos/chips/msp430/usart/HplMsp430I2C.nc"
static void HplMsp430Usart0P__HplI2C__clearModeI2C(void );
#line 6
static bool HplMsp430Usart0P__HplI2C__isI2C(void );
# 85 "/home/tinyos/local/src/tinyos-2.x/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
static void HplMsp430Usart0P__SOMI__selectIOFunc(void );
#line 78
static void HplMsp430Usart0P__SOMI__selectModuleFunc(void );
# 39 "/home/tinyos/local/src/tinyos-2.x/tos/chips/msp430/usart/HplMsp430I2CInterrupts.nc"
static void HplMsp430Usart0P__I2CInterrupts__fired(void );
# 85 "/home/tinyos/local/src/tinyos-2.x/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
static void HplMsp430Usart0P__SIMO__selectIOFunc(void );
#line 78
static void HplMsp430Usart0P__SIMO__selectModuleFunc(void );
# 89 "/home/tinyos/local/src/tinyos-2.x/tos/chips/msp430/usart/HplMsp430Usart0P.nc"
static volatile uint8_t HplMsp430Usart0P__IE1 __asm ("0x0000");
static volatile uint8_t HplMsp430Usart0P__ME1 __asm ("0x0004");
static volatile uint8_t HplMsp430Usart0P__IFG1 __asm ("0x0002");
static volatile uint8_t HplMsp430Usart0P__U0TCTL __asm ("0x0071");

static volatile uint8_t HplMsp430Usart0P__U0TXBUF __asm ("0x0077");

void sig_UART0RX_VECTOR(void ) __attribute((wakeup)) __attribute((interrupt(18)))  ;




void sig_UART0TX_VECTOR(void ) __attribute((wakeup)) __attribute((interrupt(16)))  ;
#line 132
static inline void HplMsp430Usart0P__Usart__setUbr(uint16_t control);










static inline void HplMsp430Usart0P__Usart__setUmctl(uint8_t control);







static inline void HplMsp430Usart0P__Usart__resetUsart(bool reset);
#line 207
static inline void HplMsp430Usart0P__Usart__disableUart(void );
#line 238
static inline void HplMsp430Usart0P__Usart__enableSpi(void );








static void HplMsp430Usart0P__Usart__disableSpi(void );








static inline void HplMsp430Usart0P__configSpi(msp430_spi_union_config_t *config);








static void HplMsp430Usart0P__Usart__setModeSpi(msp430_spi_union_config_t *config);
#line 330
static inline bool HplMsp430Usart0P__Usart__isRxIntrPending(void );










static inline void HplMsp430Usart0P__Usart__clrRxIntr(void );



static inline void HplMsp430Usart0P__Usart__clrIntr(void );
#line 357
static inline void HplMsp430Usart0P__Usart__disableIntr(void );
#line 382
static inline void HplMsp430Usart0P__Usart__tx(uint8_t data);



static inline uint8_t HplMsp430Usart0P__Usart__rx(void );
# 28 "/home/tinyos/local/src/tinyos-2.x/tos/chips/msp430/dma/HplMsp430DmaInterrupt.nc"
static void HplMsp430DmaP__Interrupt__fired(void );
# 72 "/home/tinyos/local/src/tinyos-2.x/tos/chips/msp430/dma/HplMsp430DmaP.nc"
void sig_DACDMA_VECTOR(void ) __attribute((wakeup)) __attribute((interrupt(0)))  ;
# 74 "/home/tinyos/local/src/tinyos-2.x/tos/chips/msp430/dma/HplMsp430DmaChannel.nc"
static void /*HplMsp430DmaC.Dma0*/HplMsp430DmaXP__0__DMA__transferDone(error_t success);
# 82 "/home/tinyos/local/src/tinyos-2.x/tos/chips/msp430/dma/HplMsp430DmaXP.nc"
static inline void /*HplMsp430DmaC.Dma0*/HplMsp430DmaXP__0__Interrupt__fired(void );
# 74 "/home/tinyos/local/src/tinyos-2.x/tos/chips/msp430/dma/HplMsp430DmaChannel.nc"
static void /*HplMsp430DmaC.Dma1*/HplMsp430DmaXP__1__DMA__transferDone(error_t success);
# 75 "/home/tinyos/local/src/tinyos-2.x/tos/chips/msp430/dma/HplMsp430DmaXP.nc"
static volatile uint16_t /*HplMsp430DmaC.Dma1*/HplMsp430DmaXP__1__DMACTL0 __asm ("0x0122");






static inline void /*HplMsp430DmaC.Dma1*/HplMsp430DmaXP__1__Interrupt__fired(void );








static inline error_t /*HplMsp430DmaC.Dma1*/HplMsp430DmaXP__1__DMA__setTrigger(dma_trigger_t trigger);
#line 195
static inline void /*HplMsp430DmaC.Dma1*/HplMsp430DmaXP__1__DMA__enableDMA(void );
#line 235
static inline void /*HplMsp430DmaC.Dma1*/HplMsp430DmaXP__1__DMA__setState(dma_channel_state_t s, 
dma_channel_trigger_t t, 
void *src, void *dest, 
uint16_t size);




static inline void /*HplMsp430DmaC.Dma1*/HplMsp430DmaXP__1__DMA__setStateRaw(uint16_t s, uint16_t t, 
void *src, void *dest, 
uint16_t size);
# 74 "/home/tinyos/local/src/tinyos-2.x/tos/chips/msp430/dma/HplMsp430DmaChannel.nc"
static void /*HplMsp430DmaC.Dma2*/HplMsp430DmaXP__2__DMA__transferDone(error_t success);
# 75 "/home/tinyos/local/src/tinyos-2.x/tos/chips/msp430/dma/HplMsp430DmaXP.nc"
static volatile uint16_t /*HplMsp430DmaC.Dma2*/HplMsp430DmaXP__2__DMACTL0 __asm ("0x0122");






static inline void /*HplMsp430DmaC.Dma2*/HplMsp430DmaXP__2__Interrupt__fired(void );








static inline error_t /*HplMsp430DmaC.Dma2*/HplMsp430DmaXP__2__DMA__setTrigger(dma_trigger_t trigger);
#line 195
static inline void /*HplMsp430DmaC.Dma2*/HplMsp430DmaXP__2__DMA__enableDMA(void );
#line 235
static inline void /*HplMsp430DmaC.Dma2*/HplMsp430DmaXP__2__DMA__setState(dma_channel_state_t s, 
dma_channel_trigger_t t, 
void *src, void *dest, 
uint16_t size);




static inline void /*HplMsp430DmaC.Dma2*/HplMsp430DmaXP__2__DMA__setStateRaw(uint16_t s, uint16_t t, 
void *src, void *dest, 
uint16_t size);
# 96 "/home/tinyos/local/src/tinyos-2.x/tos/chips/msp430/dma/Msp430DmaChannel.nc"
static void /*Msp430DmaC.Channel0P*/Msp430DmaChannelP__0__Channel__transferDone(error_t success);
# 143 "/home/tinyos/local/src/tinyos-2.x/tos/chips/msp430/dma/Msp430DmaChannelP.nc"
static inline void /*Msp430DmaC.Channel0P*/Msp430DmaChannelP__0__HplChannel__transferDone(error_t error);



static inline void /*Msp430DmaC.Channel0P*/Msp430DmaChannelP__0__Channel__default__transferDone(error_t error);
# 49 "/home/tinyos/local/src/tinyos-2.x/tos/chips/msp430/dma/HplMsp430DmaChannel.nc"
static void /*Msp430DmaC.Channel1P*/Msp430DmaChannelP__1__HplChannel__enableDMA(void );
#line 64
static void /*Msp430DmaC.Channel1P*/Msp430DmaChannelP__1__HplChannel__setState(dma_channel_state_t s, dma_channel_trigger_t t, void *src, void *dest, uint16_t size);
# 96 "/home/tinyos/local/src/tinyos-2.x/tos/chips/msp430/dma/Msp430DmaChannel.nc"
static void /*Msp430DmaC.Channel1P*/Msp430DmaChannelP__1__Channel__transferDone(error_t success);
# 71 "/home/tinyos/local/src/tinyos-2.x/tos/chips/msp430/dma/Msp430DmaChannelP.nc"
dma_channel_state_t /*Msp430DmaC.Channel1P*/Msp430DmaChannelP__1__gChannelState;
dma_channel_trigger_t /*Msp430DmaC.Channel1P*/Msp430DmaChannelP__1__gChannelTrigger;







static inline error_t /*Msp430DmaC.Channel1P*/Msp430DmaChannelP__1__Channel__setupTransfer(dma_transfer_mode_t transfer_mode, 
dma_trigger_t trigger, 
dma_level_t level, 
void *src_addr, 
void *dst_addr, 
uint16_t size, 
dma_byte_t src_byte, 
dma_byte_t dst_byte, 
dma_incr_t src_incr, 
dma_incr_t dst_incr);
#line 112
static inline error_t /*Msp430DmaC.Channel1P*/Msp430DmaChannelP__1__Channel__startTransfer(void );
#line 143
static inline void /*Msp430DmaC.Channel1P*/Msp430DmaChannelP__1__HplChannel__transferDone(error_t error);
# 49 "/home/tinyos/local/src/tinyos-2.x/tos/chips/msp430/dma/HplMsp430DmaChannel.nc"
static void /*Msp430DmaC.Channel2P*/Msp430DmaChannelP__2__HplChannel__enableDMA(void );
#line 64
static void /*Msp430DmaC.Channel2P*/Msp430DmaChannelP__2__HplChannel__setState(dma_channel_state_t s, dma_channel_trigger_t t, void *src, void *dest, uint16_t size);
# 96 "/home/tinyos/local/src/tinyos-2.x/tos/chips/msp430/dma/Msp430DmaChannel.nc"
static void /*Msp430DmaC.Channel2P*/Msp430DmaChannelP__2__Channel__transferDone(error_t success);
# 71 "/home/tinyos/local/src/tinyos-2.x/tos/chips/msp430/dma/Msp430DmaChannelP.nc"
dma_channel_state_t /*Msp430DmaC.Channel2P*/Msp430DmaChannelP__2__gChannelState;
dma_channel_trigger_t /*Msp430DmaC.Channel2P*/Msp430DmaChannelP__2__gChannelTrigger;







static inline error_t /*Msp430DmaC.Channel2P*/Msp430DmaChannelP__2__Channel__setupTransfer(dma_transfer_mode_t transfer_mode, 
dma_trigger_t trigger, 
dma_level_t level, 
void *src_addr, 
void *dst_addr, 
uint16_t size, 
dma_byte_t src_byte, 
dma_byte_t dst_byte, 
dma_incr_t src_incr, 
dma_incr_t dst_incr);
#line 112
static inline error_t /*Msp430DmaC.Channel2P*/Msp430DmaChannelP__2__Channel__startTransfer(void );
#line 143
static inline void /*Msp430DmaC.Channel2P*/Msp430DmaChannelP__2__HplChannel__transferDone(error_t error);
# 96 "/home/tinyos/local/src/tinyos-2.x/tos/chips/msp430/dma/Msp430DmaControlP.nc"
static inline void Msp430DmaControlP__HplChannel0__transferDone(error_t error);
static inline void Msp430DmaControlP__HplChannel1__transferDone(error_t error);
static inline void Msp430DmaControlP__HplChannel2__transferDone(error_t error);
# 80 "/home/tinyos/local/src/tinyos-2.x/tos/interfaces/ArbiterInfo.nc"
static bool /*Msp430UsartShare0P.UsartShareP*/Msp430UsartShareP__0__ArbiterInfo__inUse(void );







static uint8_t /*Msp430UsartShare0P.UsartShareP*/Msp430UsartShareP__0__ArbiterInfo__userId(void );
# 54 "/home/tinyos/local/src/tinyos-2.x/tos/chips/msp430/usart/HplMsp430UsartInterrupts.nc"
static void /*Msp430UsartShare0P.UsartShareP*/Msp430UsartShareP__0__Interrupts__rxDone(
# 39 "/home/tinyos/local/src/tinyos-2.x/tos/chips/msp430/usart/Msp430UsartShareP.nc"
uint8_t arg_0x40df4cf8, 
# 54 "/home/tinyos/local/src/tinyos-2.x/tos/chips/msp430/usart/HplMsp430UsartInterrupts.nc"
uint8_t data);
#line 49
static void /*Msp430UsartShare0P.UsartShareP*/Msp430UsartShareP__0__Interrupts__txDone(
# 39 "/home/tinyos/local/src/tinyos-2.x/tos/chips/msp430/usart/Msp430UsartShareP.nc"
uint8_t arg_0x40df4cf8);
# 39 "/home/tinyos/local/src/tinyos-2.x/tos/chips/msp430/usart/HplMsp430I2CInterrupts.nc"
static void /*Msp430UsartShare0P.UsartShareP*/Msp430UsartShareP__0__I2CInterrupts__fired(
# 40 "/home/tinyos/local/src/tinyos-2.x/tos/chips/msp430/usart/Msp430UsartShareP.nc"
uint8_t arg_0x40e245c8);








static inline void /*Msp430UsartShare0P.UsartShareP*/Msp430UsartShareP__0__RawInterrupts__txDone(void );




static inline void /*Msp430UsartShare0P.UsartShareP*/Msp430UsartShareP__0__RawInterrupts__rxDone(uint8_t data);




static inline void /*Msp430UsartShare0P.UsartShareP*/Msp430UsartShareP__0__RawI2CInterrupts__fired(void );




static inline void /*Msp430UsartShare0P.UsartShareP*/Msp430UsartShareP__0__Interrupts__default__txDone(uint8_t id);
static inline void /*Msp430UsartShare0P.UsartShareP*/Msp430UsartShareP__0__Interrupts__default__rxDone(uint8_t id, uint8_t data);
static inline void /*Msp430UsartShare0P.UsartShareP*/Msp430UsartShareP__0__I2CInterrupts__default__fired(uint8_t id);
# 39 "/home/tinyos/local/src/tinyos-2.x/tos/system/FcfsResourceQueueC.nc"
enum /*Msp430UsartShare0P.ArbiterC.Queue*/FcfsResourceQueueC__1____nesc_unnamed4402 {
#line 39
  FcfsResourceQueueC__1__NO_ENTRY = 0xFF
};
uint8_t /*Msp430UsartShare0P.ArbiterC.Queue*/FcfsResourceQueueC__1__resQ[1U];
uint8_t /*Msp430UsartShare0P.ArbiterC.Queue*/FcfsResourceQueueC__1__qHead = /*Msp430UsartShare0P.ArbiterC.Queue*/FcfsResourceQueueC__1__NO_ENTRY;
uint8_t /*Msp430UsartShare0P.ArbiterC.Queue*/FcfsResourceQueueC__1__qTail = /*Msp430UsartShare0P.ArbiterC.Queue*/FcfsResourceQueueC__1__NO_ENTRY;

static inline error_t /*Msp430UsartShare0P.ArbiterC.Queue*/FcfsResourceQueueC__1__Init__init(void );




static inline bool /*Msp430UsartShare0P.ArbiterC.Queue*/FcfsResourceQueueC__1__FcfsQueue__isEmpty(void );



static inline bool /*Msp430UsartShare0P.ArbiterC.Queue*/FcfsResourceQueueC__1__FcfsQueue__isEnqueued(resource_client_id_t id);



static inline resource_client_id_t /*Msp430UsartShare0P.ArbiterC.Queue*/FcfsResourceQueueC__1__FcfsQueue__dequeue(void );
#line 72
static inline error_t /*Msp430UsartShare0P.ArbiterC.Queue*/FcfsResourceQueueC__1__FcfsQueue__enqueue(resource_client_id_t id);
# 43 "/home/tinyos/local/src/tinyos-2.x/tos/interfaces/ResourceRequested.nc"
static void /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0__ResourceRequested__requested(
# 55 "/home/tinyos/local/src/tinyos-2.x/tos/system/ArbiterP.nc"
uint8_t arg_0x40e33d38);
# 51 "/home/tinyos/local/src/tinyos-2.x/tos/interfaces/ResourceRequested.nc"
static void /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0__ResourceRequested__immediateRequested(
# 55 "/home/tinyos/local/src/tinyos-2.x/tos/system/ArbiterP.nc"
uint8_t arg_0x40e33d38);
# 55 "/home/tinyos/local/src/tinyos-2.x/tos/interfaces/ResourceConfigure.nc"
static void /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0__ResourceConfigure__unconfigure(
# 60 "/home/tinyos/local/src/tinyos-2.x/tos/system/ArbiterP.nc"
uint8_t arg_0x40e31148);
# 49 "/home/tinyos/local/src/tinyos-2.x/tos/interfaces/ResourceConfigure.nc"
static void /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0__ResourceConfigure__configure(
# 60 "/home/tinyos/local/src/tinyos-2.x/tos/system/ArbiterP.nc"
uint8_t arg_0x40e31148);
# 69 "/home/tinyos/local/src/tinyos-2.x/tos/interfaces/ResourceQueue.nc"
static error_t /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0__Queue__enqueue(resource_client_id_t id);
#line 43
static bool /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0__Queue__isEmpty(void );
#line 60
static resource_client_id_t /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0__Queue__dequeue(void );
# 73 "/home/tinyos/local/src/tinyos-2.x/tos/interfaces/ResourceDefaultOwner.nc"
static void /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0__ResourceDefaultOwner__requested(void );
#line 46
static void /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0__ResourceDefaultOwner__granted(void );
#line 81
static void /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0__ResourceDefaultOwner__immediateRequested(void );
# 92 "/home/tinyos/local/src/tinyos-2.x/tos/interfaces/Resource.nc"
static void /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0__Resource__granted(
# 54 "/home/tinyos/local/src/tinyos-2.x/tos/system/ArbiterP.nc"
uint8_t arg_0x40e33310);
# 56 "/home/tinyos/local/src/tinyos-2.x/tos/interfaces/TaskBasic.nc"
static error_t /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0__grantedTask__postTask(void );
# 75 "/home/tinyos/local/src/tinyos-2.x/tos/system/ArbiterP.nc"
enum /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0____nesc_unnamed4403 {
#line 75
  ArbiterP__0__grantedTask = 9U
};
#line 75
typedef int /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0____nesc_sillytask_grantedTask[/*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0__grantedTask];
#line 67
enum /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0____nesc_unnamed4404 {
#line 67
  ArbiterP__0__RES_CONTROLLED, ArbiterP__0__RES_GRANTING, ArbiterP__0__RES_IMM_GRANTING, ArbiterP__0__RES_BUSY
};
#line 68
enum /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0____nesc_unnamed4405 {
#line 68
  ArbiterP__0__default_owner_id = 1U
};
#line 69
enum /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0____nesc_unnamed4406 {
#line 69
  ArbiterP__0__NO_RES = 0xFF
};
uint8_t /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0__state = /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0__RES_CONTROLLED;
uint8_t /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0__resId = /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0__default_owner_id;
uint8_t /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0__reqResId;



static inline error_t /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0__Resource__request(uint8_t id);
#line 90
static inline error_t /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0__Resource__immediateRequest(uint8_t id);
#line 108
static inline error_t /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0__Resource__release(uint8_t id);
#line 130
static error_t /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0__ResourceDefaultOwner__release(void );
#line 150
static bool /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0__ArbiterInfo__inUse(void );
#line 163
static uint8_t /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0__ArbiterInfo__userId(void );










static uint8_t /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0__Resource__isOwner(uint8_t id);
#line 187
static inline void /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0__grantedTask__runTask(void );
#line 199
static inline void /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0__Resource__default__granted(uint8_t id);

static inline void /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0__ResourceRequested__default__requested(uint8_t id);

static inline void /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0__ResourceRequested__default__immediateRequested(uint8_t id);

static inline void /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0__ResourceDefaultOwner__default__granted(void );

static inline void /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0__ResourceDefaultOwner__default__requested(void );


static inline void /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0__ResourceDefaultOwner__default__immediateRequested(void );


static inline void /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0__ResourceConfigure__default__configure(uint8_t id);

static inline void /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0__ResourceConfigure__default__unconfigure(uint8_t id);
# 97 "/home/tinyos/local/src/tinyos-2.x/tos/chips/msp430/usart/HplMsp430Usart.nc"
static void HplMsp430I2C0P__HplUsart__resetUsart(bool reset);
# 49 "/home/tinyos/local/src/tinyos-2.x/tos/chips/msp430/usart/HplMsp430I2C0P.nc"
static volatile uint8_t HplMsp430I2C0P__U0CTL __asm ("0x0070");





static inline bool HplMsp430I2C0P__HplI2C__isI2C(void );



static inline void HplMsp430I2C0P__HplI2C__clearModeI2C(void );
# 51 "/home/tinyos/local/src/tinyos-2.x/tos/system/ActiveMessageAddressC.nc"
am_addr_t ActiveMessageAddressC__addr = TOS_AM_ADDRESS;


am_group_t ActiveMessageAddressC__group = TOS_AM_GROUP;






static inline am_addr_t ActiveMessageAddressC__ActiveMessageAddress__amAddress(void );
#line 82
static inline am_group_t ActiveMessageAddressC__ActiveMessageAddress__amGroup(void );
#line 95
static inline am_addr_t ActiveMessageAddressC__amAddress(void );
# 81 "/home/tinyos/local/src/tinyos-2.x/tos/chips/cc2420/interfaces/RadioBackoff.nc"
static void CC2420TransmitP__RadioBackoff__requestInitialBackoff(message_t * msg);






static void CC2420TransmitP__RadioBackoff__requestCongestionBackoff(message_t * msg);
# 59 "/home/tinyos/local/src/tinyos-2.x/tos/interfaces/PacketTimeStamp.nc"
static void CC2420TransmitP__PacketTimeStamp__clear(
#line 55
message_t * msg);
#line 67
static void CC2420TransmitP__PacketTimeStamp__set(
#line 62
message_t * msg, 




CC2420TransmitP__PacketTimeStamp__size_type value);
# 45 "/home/tinyos/local/src/tinyos-2.x/tos/chips/cc2420/interfaces/CC2420Strobe.nc"
static cc2420_status_t CC2420TransmitP__STXONCCA__strobe(void );
# 43 "/home/tinyos/local/src/tinyos-2.x/tos/interfaces/GpioCapture.nc"
static error_t CC2420TransmitP__CaptureSFD__captureFallingEdge(void );
#line 55
static void CC2420TransmitP__CaptureSFD__disable(void );
#line 42
static error_t CC2420TransmitP__CaptureSFD__captureRisingEdge(void );
# 98 "/home/tinyos/local/src/tinyos-2.x/tos/lib/timer/Alarm.nc"
static CC2420TransmitP__BackoffTimer__size_type CC2420TransmitP__BackoffTimer__getNow(void );
#line 55
static void CC2420TransmitP__BackoffTimer__start(CC2420TransmitP__BackoffTimer__size_type dt);






static void CC2420TransmitP__BackoffTimer__stop(void );
# 63 "/home/tinyos/local/src/tinyos-2.x/tos/chips/cc2420/interfaces/CC2420Ram.nc"
static cc2420_status_t CC2420TransmitP__TXFIFO_RAM__write(uint8_t offset, uint8_t * data, uint8_t length);
# 55 "/home/tinyos/local/src/tinyos-2.x/tos/chips/cc2420/interfaces/CC2420Register.nc"
static cc2420_status_t CC2420TransmitP__TXCTRL__write(uint16_t data);
# 55 "/home/tinyos/local/src/tinyos-2.x/tos/chips/cc2420/interfaces/CC2420Receive.nc"
static void CC2420TransmitP__CC2420Receive__sfd_dropped(void );
#line 49
static void CC2420TransmitP__CC2420Receive__sfd(uint32_t time);
# 73 "/home/tinyos/local/src/tinyos-2.x/tos/chips/cc2420/interfaces/CC2420Transmit.nc"
static void CC2420TransmitP__Send__sendDone(message_t * p_msg, error_t error);
# 31 "/home/tinyos/local/src/tinyos-2.x/tos/chips/cc2420/interfaces/ChipSpiResource.nc"
static void CC2420TransmitP__ChipSpiResource__abortRelease(void );







static error_t CC2420TransmitP__ChipSpiResource__attemptRelease(void );
# 45 "/home/tinyos/local/src/tinyos-2.x/tos/chips/cc2420/interfaces/CC2420Strobe.nc"
static cc2420_status_t CC2420TransmitP__SFLUSHTX__strobe(void );
# 35 "/home/tinyos/local/src/tinyos-2.x/tos/interfaces/GeneralIO.nc"
static void CC2420TransmitP__CSN__makeOutput(void );
#line 29
static void CC2420TransmitP__CSN__set(void );
static void CC2420TransmitP__CSN__clr(void );
# 42 "/home/tinyos/local/src/tinyos-2.x/tos/chips/cc2420/interfaces/CC2420PacketBody.nc"
static cc2420_header_t * CC2420TransmitP__CC2420PacketBody__getHeader(message_t * msg);




static cc2420_metadata_t * CC2420TransmitP__CC2420PacketBody__getMetadata(message_t * msg);
# 47 "/home/tinyos/local/src/tinyos-2.x/tos/chips/cc2420/interfaces/PacketTimeSyncOffset.nc"
static uint8_t CC2420TransmitP__PacketTimeSyncOffset__get(
#line 42
message_t * msg);
#line 39
static bool CC2420TransmitP__PacketTimeSyncOffset__isSet(
#line 35
message_t * msg);
# 110 "/home/tinyos/local/src/tinyos-2.x/tos/interfaces/Resource.nc"
static error_t CC2420TransmitP__SpiResource__release(void );
#line 87
static error_t CC2420TransmitP__SpiResource__immediateRequest(void );
#line 78
static error_t CC2420TransmitP__SpiResource__request(void );
# 33 "/home/tinyos/local/src/tinyos-2.x/tos/interfaces/GeneralIO.nc"
static void CC2420TransmitP__CCA__makeInput(void );
#line 32
static bool CC2420TransmitP__CCA__get(void );
# 45 "/home/tinyos/local/src/tinyos-2.x/tos/chips/cc2420/interfaces/CC2420Strobe.nc"
static cc2420_status_t CC2420TransmitP__SNOP__strobe(void );
# 33 "/home/tinyos/local/src/tinyos-2.x/tos/interfaces/GeneralIO.nc"
static void CC2420TransmitP__SFD__makeInput(void );
#line 32
static bool CC2420TransmitP__SFD__get(void );
# 82 "/home/tinyos/local/src/tinyos-2.x/tos/chips/cc2420/interfaces/CC2420Fifo.nc"
static cc2420_status_t CC2420TransmitP__TXFIFO__write(uint8_t * data, uint8_t length);
# 45 "/home/tinyos/local/src/tinyos-2.x/tos/chips/cc2420/interfaces/CC2420Strobe.nc"
static cc2420_status_t CC2420TransmitP__STXON__strobe(void );
# 99 "/home/tinyos/local/src/tinyos-2.x/tos/chips/cc2420/transmit/CC2420TransmitP.nc"
#line 89
typedef enum CC2420TransmitP____nesc_unnamed4407 {
  CC2420TransmitP__S_STOPPED, 
  CC2420TransmitP__S_STARTED, 
  CC2420TransmitP__S_LOAD, 
  CC2420TransmitP__S_SAMPLE_CCA, 
  CC2420TransmitP__S_BEGIN_TRANSMIT, 
  CC2420TransmitP__S_SFD, 
  CC2420TransmitP__S_EFD, 
  CC2420TransmitP__S_ACK_WAIT, 
  CC2420TransmitP__S_CANCEL
} CC2420TransmitP__cc2420_transmit_state_t;





enum CC2420TransmitP____nesc_unnamed4408 {
  CC2420TransmitP__CC2420_ABORT_PERIOD = 320
};
#line 120
message_t * CC2420TransmitP__m_msg;

bool CC2420TransmitP__m_cca;

uint8_t CC2420TransmitP__m_tx_power;

CC2420TransmitP__cc2420_transmit_state_t CC2420TransmitP__m_state = CC2420TransmitP__S_STOPPED;

bool CC2420TransmitP__m_receiving = FALSE;

uint16_t CC2420TransmitP__m_prev_time;


bool CC2420TransmitP__sfdHigh;


bool CC2420TransmitP__abortSpiRelease;


int8_t CC2420TransmitP__totalCcaChecks;


uint16_t CC2420TransmitP__myInitialBackoff;


uint16_t CC2420TransmitP__myCongestionBackoff;



static inline error_t CC2420TransmitP__send(message_t * p_msg, bool cca);

static void CC2420TransmitP__loadTXFIFO(void );
static void CC2420TransmitP__attemptSend(void );
static void CC2420TransmitP__congestionBackoff(void );
static error_t CC2420TransmitP__acquireSpiResource(void );
static inline error_t CC2420TransmitP__releaseSpiResource(void );
static void CC2420TransmitP__signalDone(error_t err);



static inline error_t CC2420TransmitP__Init__init(void );







static inline error_t CC2420TransmitP__StdControl__start(void );










static error_t CC2420TransmitP__StdControl__stop(void );
#line 192
static inline error_t CC2420TransmitP__Send__send(message_t * p_msg, bool useCca);
#line 243
static inline void CC2420TransmitP__RadioBackoff__setInitialBackoff(uint16_t backoffTime);







static inline void CC2420TransmitP__RadioBackoff__setCongestionBackoff(uint16_t backoffTime);







static __inline uint32_t CC2420TransmitP__getTime32(uint16_t time);
#line 278
static inline void CC2420TransmitP__CaptureSFD__captured(uint16_t time);
#line 375
static inline void CC2420TransmitP__ChipSpiResource__releasing(void );
#line 387
static inline void CC2420TransmitP__CC2420Receive__receive(uint8_t type, message_t *ack_msg);
#line 414
static inline void CC2420TransmitP__SpiResource__granted(void );
#line 452
static inline void CC2420TransmitP__TXFIFO__writeDone(uint8_t *tx_buf, uint8_t tx_len, 
error_t error);
#line 484
static inline void CC2420TransmitP__TXFIFO__readDone(uint8_t *tx_buf, uint8_t tx_len, 
error_t error);










static inline void CC2420TransmitP__BackoffTimer__fired(void );
#line 545
static inline error_t CC2420TransmitP__send(message_t * p_msg, bool cca);
#line 735
static void CC2420TransmitP__attemptSend(void );
#line 780
static void CC2420TransmitP__congestionBackoff(void );






static error_t CC2420TransmitP__acquireSpiResource(void );







static inline error_t CC2420TransmitP__releaseSpiResource(void );
#line 817
static void CC2420TransmitP__loadTXFIFO(void );
#line 842
static void CC2420TransmitP__signalDone(error_t err);
# 32 "/home/tinyos/local/src/tinyos-2.x/tos/interfaces/GeneralIO.nc"
static bool CC2420ReceiveP__FIFO__get(void );
# 86 "/home/tinyos/local/src/tinyos-2.x/tos/chips/cc2420/interfaces/CC2420Config.nc"
static bool CC2420ReceiveP__CC2420Config__isAddressRecognitionEnabled(void );
#line 110
static bool CC2420ReceiveP__CC2420Config__isAutoAckEnabled(void );
#line 105
static bool CC2420ReceiveP__CC2420Config__isHwAutoAckDefault(void );
#line 64
static uint16_t CC2420ReceiveP__CC2420Config__getShortAddr(void );
# 56 "/home/tinyos/local/src/tinyos-2.x/tos/interfaces/TaskBasic.nc"
static error_t CC2420ReceiveP__receiveDone_task__postTask(void );
# 59 "/home/tinyos/local/src/tinyos-2.x/tos/interfaces/PacketTimeStamp.nc"
static void CC2420ReceiveP__PacketTimeStamp__clear(
#line 55
message_t * msg);
#line 67
static void CC2420ReceiveP__PacketTimeStamp__set(
#line 62
message_t * msg, 




CC2420ReceiveP__PacketTimeStamp__size_type value);
# 32 "/home/tinyos/local/src/tinyos-2.x/tos/interfaces/GeneralIO.nc"
static bool CC2420ReceiveP__FIFOP__get(void );
# 63 "/home/tinyos/local/src/tinyos-2.x/tos/chips/cc2420/interfaces/CC2420Receive.nc"
static void CC2420ReceiveP__CC2420Receive__receive(uint8_t type, message_t * message);
# 45 "/home/tinyos/local/src/tinyos-2.x/tos/chips/cc2420/interfaces/CC2420Strobe.nc"
static cc2420_status_t CC2420ReceiveP__SACK__strobe(void );
# 29 "/home/tinyos/local/src/tinyos-2.x/tos/interfaces/GeneralIO.nc"
static void CC2420ReceiveP__CSN__set(void );
static void CC2420ReceiveP__CSN__clr(void );
# 42 "/home/tinyos/local/src/tinyos-2.x/tos/chips/cc2420/interfaces/CC2420PacketBody.nc"
static cc2420_header_t * CC2420ReceiveP__CC2420PacketBody__getHeader(message_t * msg);




static cc2420_metadata_t * CC2420ReceiveP__CC2420PacketBody__getMetadata(message_t * msg);
# 67 "/home/tinyos/local/src/tinyos-2.x/tos/interfaces/Receive.nc"
static 
#line 63
message_t * 



CC2420ReceiveP__Receive__receive(
#line 60
message_t * msg, 
void * payload, 





uint8_t len);
# 110 "/home/tinyos/local/src/tinyos-2.x/tos/interfaces/Resource.nc"
static error_t CC2420ReceiveP__SpiResource__release(void );
#line 87
static error_t CC2420ReceiveP__SpiResource__immediateRequest(void );
#line 78
static error_t CC2420ReceiveP__SpiResource__request(void );
#line 118
static bool CC2420ReceiveP__SpiResource__isOwner(void );
# 62 "/home/tinyos/local/src/tinyos-2.x/tos/chips/cc2420/interfaces/CC2420Fifo.nc"
static error_t CC2420ReceiveP__RXFIFO__continueRead(uint8_t * data, uint8_t length);
#line 51
static cc2420_status_t CC2420ReceiveP__RXFIFO__beginRead(uint8_t * data, uint8_t length);
# 50 "/home/tinyos/local/src/tinyos-2.x/tos/interfaces/GpioInterrupt.nc"
static error_t CC2420ReceiveP__InterruptFIFOP__disable(void );
#line 43
static error_t CC2420ReceiveP__InterruptFIFOP__enableFallingEdge(void );
# 45 "/home/tinyos/local/src/tinyos-2.x/tos/chips/cc2420/interfaces/CC2420Strobe.nc"
static cc2420_status_t CC2420ReceiveP__SFLUSHRX__strobe(void );
# 148 "/home/tinyos/local/src/tinyos-2.x/tos/chips/cc2420/receive/CC2420ReceiveP.nc"
enum CC2420ReceiveP____nesc_unnamed4409 {
#line 148
  CC2420ReceiveP__receiveDone_task = 10U
};
#line 148
typedef int CC2420ReceiveP____nesc_sillytask_receiveDone_task[CC2420ReceiveP__receiveDone_task];
#line 89
#line 81
typedef enum CC2420ReceiveP____nesc_unnamed4410 {
  CC2420ReceiveP__S_STOPPED, 
  CC2420ReceiveP__S_STARTED, 
  CC2420ReceiveP__S_RX_LENGTH, 
  CC2420ReceiveP__S_RX_DEC, 
  CC2420ReceiveP__S_RX_DEC_WAIT, 
  CC2420ReceiveP__S_RX_FCF, 
  CC2420ReceiveP__S_RX_PAYLOAD
} CC2420ReceiveP__cc2420_receive_state_t;

enum CC2420ReceiveP____nesc_unnamed4411 {
  CC2420ReceiveP__RXFIFO_SIZE = 128, 
  CC2420ReceiveP__TIMESTAMP_QUEUE_SIZE = 8, 
  CC2420ReceiveP__SACK_HEADER_LENGTH = 7
};

uint32_t CC2420ReceiveP__m_timestamp_queue[CC2420ReceiveP__TIMESTAMP_QUEUE_SIZE];

uint8_t CC2420ReceiveP__m_timestamp_head;

uint8_t CC2420ReceiveP__m_timestamp_size;





uint8_t CC2420ReceiveP__m_missed_packets;



bool CC2420ReceiveP__receivingPacket;


uint8_t CC2420ReceiveP__rxFrameLength;

uint8_t CC2420ReceiveP__m_bytes_left;

message_t * CC2420ReceiveP__m_p_rx_buf;

message_t CC2420ReceiveP__m_rx_buf;
#line 137
CC2420ReceiveP__cc2420_receive_state_t CC2420ReceiveP__m_state;



static void CC2420ReceiveP__reset_state(void );
static void CC2420ReceiveP__beginReceive(void );
static void CC2420ReceiveP__receive(void );
static void CC2420ReceiveP__waitForNextPacket(void );
static void CC2420ReceiveP__flush(void );
static inline bool CC2420ReceiveP__passesAddressCheck(message_t * msg);




static inline error_t CC2420ReceiveP__Init__init(void );





static inline error_t CC2420ReceiveP__StdControl__start(void );
#line 171
static error_t CC2420ReceiveP__StdControl__stop(void );
#line 186
static inline void CC2420ReceiveP__CC2420Receive__sfd(uint32_t time);








static inline void CC2420ReceiveP__CC2420Receive__sfd_dropped(void );
#line 212
static inline void CC2420ReceiveP__InterruptFIFOP__fired(void );
#line 508
static inline void CC2420ReceiveP__SpiResource__granted(void );
#line 525
static inline void CC2420ReceiveP__RXFIFO__readDone(uint8_t *rx_buf, uint8_t rx_len, 
error_t error);
#line 663
static inline void CC2420ReceiveP__RXFIFO__writeDone(uint8_t *tx_buf, uint8_t tx_len, error_t error);







static inline void CC2420ReceiveP__receiveDone_task__runTask(void );
#line 704
static inline void CC2420ReceiveP__CC2420Config__syncDone(error_t error);






static void CC2420ReceiveP__beginReceive(void );
#line 728
static void CC2420ReceiveP__flush(void );
#line 754
static void CC2420ReceiveP__receive(void );









static void CC2420ReceiveP__waitForNextPacket(void );
#line 808
static void CC2420ReceiveP__reset_state(void );










static inline bool CC2420ReceiveP__passesAddressCheck(message_t *msg);
# 65 "/home/tinyos/local/src/tinyos-2.x/tos/chips/cc2420/packet/CC2420PacketP.nc"
static error_t CC2420PacketP__Acks__requestAck(message_t *p_msg);









static inline bool CC2420PacketP__Acks__wasAcked(message_t *p_msg);
#line 94
static inline uint8_t CC2420PacketP__CC2420Packet__getLqi(message_t *p_msg);



static uint8_t CC2420PacketP__CC2420Packet__getNetwork(message_t *p_msg);
#line 114
static inline cc2420_header_t * CC2420PacketP__CC2420PacketBody__getHeader(message_t * msg);



static inline cc2420_metadata_t *CC2420PacketP__CC2420PacketBody__getMetadata(message_t *msg);
#line 137
static void CC2420PacketP__PacketTimeStamp32khz__clear(message_t *msg);





static inline void CC2420PacketP__PacketTimeStamp32khz__set(message_t *msg, uint32_t value);
#line 176
static inline bool CC2420PacketP__PacketTimeSyncOffset__isSet(message_t *msg);








static inline uint8_t CC2420PacketP__PacketTimeSyncOffset__get(message_t *msg);
# 47 "/home/tinyos/local/src/tinyos-2.x/tos/lib/timer/CounterToLocalTimeC.nc"
static inline void /*CC2420PacketC.CounterToLocalTimeC*/CounterToLocalTimeC__1__Counter__overflow(void );
# 41 "/home/tinyos/local/src/tinyos-2.x/tos/system/RandomMlcgC.nc"
uint32_t RandomMlcgC__seed;


static inline error_t RandomMlcgC__Init__init(void );
#line 58
static uint32_t RandomMlcgC__Random__rand32(void );
#line 78
static inline uint16_t RandomMlcgC__Random__rand16(void );
# 64 "/home/tinyos/local/src/tinyos-2.x/tos/interfaces/Send.nc"
static error_t UniqueSendP__SubSend__send(
#line 56
message_t * msg, 







uint8_t len);
#line 89
static void UniqueSendP__Send__sendDone(
#line 85
message_t * msg, 



error_t error);
# 41 "/home/tinyos/local/src/tinyos-2.x/tos/interfaces/Random.nc"
static uint16_t UniqueSendP__Random__rand16(void );
# 42 "/home/tinyos/local/src/tinyos-2.x/tos/chips/cc2420/interfaces/CC2420PacketBody.nc"
static cc2420_header_t * UniqueSendP__CC2420PacketBody__getHeader(message_t * msg);
# 56 "/home/tinyos/local/src/tinyos-2.x/tos/interfaces/State.nc"
static void UniqueSendP__State__toIdle(void );
#line 45
static error_t UniqueSendP__State__requestState(uint8_t reqState);
# 54 "/home/tinyos/local/src/tinyos-2.x/tos/chips/cc2420/unique/UniqueSendP.nc"
uint8_t UniqueSendP__localSendId;

enum UniqueSendP____nesc_unnamed4412 {
  UniqueSendP__S_IDLE, 
  UniqueSendP__S_SENDING
};


static inline error_t UniqueSendP__Init__init(void );
#line 75
static inline error_t UniqueSendP__Send__send(message_t *msg, uint8_t len);
#line 104
static inline void UniqueSendP__SubSend__sendDone(message_t *msg, error_t error);
# 67 "/home/tinyos/local/src/tinyos-2.x/tos/interfaces/Receive.nc"
static 
#line 63
message_t * 



UniqueReceiveP__Receive__receive(
#line 60
message_t * msg, 
void * payload, 





uint8_t len);
# 42 "/home/tinyos/local/src/tinyos-2.x/tos/chips/cc2420/interfaces/CC2420PacketBody.nc"
static cc2420_header_t * UniqueReceiveP__CC2420PacketBody__getHeader(message_t * msg);
# 67 "/home/tinyos/local/src/tinyos-2.x/tos/interfaces/Receive.nc"
static 
#line 63
message_t * 



UniqueReceiveP__DuplicateReceive__receive(
#line 60
message_t * msg, 
void * payload, 





uint8_t len);
# 59 "/home/tinyos/local/src/tinyos-2.x/tos/chips/cc2420/unique/UniqueReceiveP.nc"
#line 56
struct UniqueReceiveP____nesc_unnamed4413 {
  am_addr_t source;
  uint8_t dsn;
} UniqueReceiveP__receivedMessages[4];

uint8_t UniqueReceiveP__writeIndex = 0;


uint8_t UniqueReceiveP__recycleSourceElement;

enum UniqueReceiveP____nesc_unnamed4414 {
  UniqueReceiveP__INVALID_ELEMENT = 0xFF
};


static inline error_t UniqueReceiveP__Init__init(void );









static inline bool UniqueReceiveP__hasSeen(uint16_t msgSource, uint8_t msgDsn);
static inline void UniqueReceiveP__insert(uint16_t msgSource, uint8_t msgDsn);


static inline message_t *UniqueReceiveP__SubReceive__receive(message_t *msg, void *payload, 
uint8_t len);
#line 111
static inline bool UniqueReceiveP__hasSeen(uint16_t msgSource, uint8_t msgDsn);
#line 137
static inline void UniqueReceiveP__insert(uint16_t msgSource, uint8_t msgDsn);
#line 158
static inline message_t *UniqueReceiveP__DuplicateReceive__default__receive(message_t *msg, void *payload, uint8_t len);
# 64 "/home/tinyos/local/src/tinyos-2.x/tos/interfaces/Send.nc"
static error_t CC2420TinyosNetworkP__SubSend__send(
#line 56
message_t * msg, 







uint8_t len);
# 56 "/home/tinyos/local/src/tinyos-2.x/tos/interfaces/TaskBasic.nc"
static error_t CC2420TinyosNetworkP__grantTask__postTask(void );
# 75 "/home/tinyos/local/src/tinyos-2.x/tos/chips/cc2420/interfaces/CC2420Packet.nc"
static uint8_t CC2420TinyosNetworkP__CC2420Packet__getNetwork(message_t *p_msg);
# 89 "/home/tinyos/local/src/tinyos-2.x/tos/interfaces/Send.nc"
static void CC2420TinyosNetworkP__ActiveSend__sendDone(
#line 85
message_t * msg, 



error_t error);
# 69 "/home/tinyos/local/src/tinyos-2.x/tos/interfaces/ResourceQueue.nc"
static error_t CC2420TinyosNetworkP__Queue__enqueue(resource_client_id_t id);
#line 43
static bool CC2420TinyosNetworkP__Queue__isEmpty(void );
#line 60
static resource_client_id_t CC2420TinyosNetworkP__Queue__dequeue(void );
# 42 "/home/tinyos/local/src/tinyos-2.x/tos/chips/cc2420/interfaces/CC2420PacketBody.nc"
static cc2420_header_t * CC2420TinyosNetworkP__CC2420PacketBody__getHeader(message_t * msg);




static cc2420_metadata_t * CC2420TinyosNetworkP__CC2420PacketBody__getMetadata(message_t * msg);
# 67 "/home/tinyos/local/src/tinyos-2.x/tos/interfaces/Receive.nc"
static 
#line 63
message_t * 



CC2420TinyosNetworkP__BareReceive__receive(
#line 60
message_t * msg, 
void * payload, 





uint8_t len);
# 92 "/home/tinyos/local/src/tinyos-2.x/tos/interfaces/Resource.nc"
static void CC2420TinyosNetworkP__Resource__granted(
# 46 "/home/tinyos/local/src/tinyos-2.x/tos/chips/cc2420/lowpan/CC2420TinyosNetworkP.nc"
uint8_t arg_0x41096628);
# 89 "/home/tinyos/local/src/tinyos-2.x/tos/interfaces/Send.nc"
static void CC2420TinyosNetworkP__BareSend__sendDone(
#line 85
message_t * msg, 



error_t error);
# 67 "/home/tinyos/local/src/tinyos-2.x/tos/interfaces/Receive.nc"
static 
#line 63
message_t * 



CC2420TinyosNetworkP__ActiveReceive__receive(
#line 60
message_t * msg, 
void * payload, 





uint8_t len);
# 148 "/home/tinyos/local/src/tinyos-2.x/tos/chips/cc2420/lowpan/CC2420TinyosNetworkP.nc"
enum CC2420TinyosNetworkP____nesc_unnamed4415 {
#line 148
  CC2420TinyosNetworkP__grantTask = 11U
};
#line 148
typedef int CC2420TinyosNetworkP____nesc_sillytask_grantTask[CC2420TinyosNetworkP__grantTask];
#line 66
enum CC2420TinyosNetworkP____nesc_unnamed4416 {
  CC2420TinyosNetworkP__OWNER_NONE = 0xff, 
  CC2420TinyosNetworkP__TINYOS_N_NETWORKS = 1U
};

uint8_t CC2420TinyosNetworkP__resource_owner = CC2420TinyosNetworkP__OWNER_NONE;
#line 71
uint8_t CC2420TinyosNetworkP__next_owner;
#line 95
static inline error_t CC2420TinyosNetworkP__BareSend__send(message_t *msg, uint8_t len);
#line 107
static inline void *CC2420TinyosNetworkP__BareSend__getPayload(message_t *msg, uint8_t len);










static inline void CC2420TinyosNetworkP__SubSend__sendDone(message_t *msg, error_t error);








static inline message_t *CC2420TinyosNetworkP__SubReceive__receive(message_t *msg, void *payload, uint8_t len);
#line 148
static inline void CC2420TinyosNetworkP__grantTask__runTask(void );
#line 167
static inline error_t CC2420TinyosNetworkP__Resource__request(uint8_t id);
#line 198
static inline error_t CC2420TinyosNetworkP__Resource__release(uint8_t id);
#line 216
static inline message_t *CC2420TinyosNetworkP__ActiveReceive__default__receive(message_t *msg, void *payload, uint8_t len);


static inline void CC2420TinyosNetworkP__ActiveSend__default__sendDone(message_t *msg, error_t error);


static inline void CC2420TinyosNetworkP__Resource__default__granted(uint8_t client);
# 39 "/home/tinyos/local/src/tinyos-2.x/tos/system/FcfsResourceQueueC.nc"
enum /*CC2420TinyosNetworkC.FcfsResourceQueueC*/FcfsResourceQueueC__0____nesc_unnamed4417 {
#line 39
  FcfsResourceQueueC__0__NO_ENTRY = 0xFF
};
uint8_t /*CC2420TinyosNetworkC.FcfsResourceQueueC*/FcfsResourceQueueC__0__resQ[1];
uint8_t /*CC2420TinyosNetworkC.FcfsResourceQueueC*/FcfsResourceQueueC__0__qHead = /*CC2420TinyosNetworkC.FcfsResourceQueueC*/FcfsResourceQueueC__0__NO_ENTRY;
uint8_t /*CC2420TinyosNetworkC.FcfsResourceQueueC*/FcfsResourceQueueC__0__qTail = /*CC2420TinyosNetworkC.FcfsResourceQueueC*/FcfsResourceQueueC__0__NO_ENTRY;

static inline error_t /*CC2420TinyosNetworkC.FcfsResourceQueueC*/FcfsResourceQueueC__0__Init__init(void );




static inline bool /*CC2420TinyosNetworkC.FcfsResourceQueueC*/FcfsResourceQueueC__0__FcfsQueue__isEmpty(void );



static inline bool /*CC2420TinyosNetworkC.FcfsResourceQueueC*/FcfsResourceQueueC__0__FcfsQueue__isEnqueued(resource_client_id_t id);



static inline resource_client_id_t /*CC2420TinyosNetworkC.FcfsResourceQueueC*/FcfsResourceQueueC__0__FcfsQueue__dequeue(void );
#line 72
static inline error_t /*CC2420TinyosNetworkC.FcfsResourceQueueC*/FcfsResourceQueueC__0__FcfsQueue__enqueue(resource_client_id_t id);
# 64 "/home/tinyos/local/src/tinyos-2.x/tos/interfaces/Send.nc"
static error_t PacketLinkP__SubSend__send(
#line 56
message_t * msg, 







uint8_t len);
# 56 "/home/tinyos/local/src/tinyos-2.x/tos/interfaces/TaskBasic.nc"
static error_t PacketLinkP__send__postTask(void );
# 62 "/home/tinyos/local/src/tinyos-2.x/tos/lib/timer/Timer.nc"
static void PacketLinkP__DelayTimer__startOneShot(uint32_t dt);




static void PacketLinkP__DelayTimer__stop(void );
# 89 "/home/tinyos/local/src/tinyos-2.x/tos/interfaces/Send.nc"
static void PacketLinkP__Send__sendDone(
#line 85
message_t * msg, 



error_t error);
# 71 "/home/tinyos/local/src/tinyos-2.x/tos/interfaces/State.nc"
static uint8_t PacketLinkP__SendState__getState(void );
#line 56
static void PacketLinkP__SendState__toIdle(void );
#line 45
static error_t PacketLinkP__SendState__requestState(uint8_t reqState);
# 47 "/home/tinyos/local/src/tinyos-2.x/tos/chips/cc2420/interfaces/CC2420PacketBody.nc"
static cc2420_metadata_t * PacketLinkP__CC2420PacketBody__getMetadata(message_t * msg);
# 48 "/home/tinyos/local/src/tinyos-2.x/tos/interfaces/PacketAcknowledgements.nc"
static error_t PacketLinkP__PacketAcknowledgements__requestAck(
#line 42
message_t * msg);
#line 74
static bool PacketLinkP__PacketAcknowledgements__wasAcked(
#line 69
message_t * msg);
# 77 "/home/tinyos/local/src/tinyos-2.x/tos/chips/cc2420/link/PacketLinkP.nc"
enum PacketLinkP____nesc_unnamed4418 {
#line 77
  PacketLinkP__send = 12U
};
#line 77
typedef int PacketLinkP____nesc_sillytask_send[PacketLinkP__send];
#line 58
message_t *PacketLinkP__currentSendMsg;


uint8_t PacketLinkP__currentSendLen;


uint16_t PacketLinkP__totalRetries;





enum PacketLinkP____nesc_unnamed4419 {
  PacketLinkP__S_IDLE, 
  PacketLinkP__S_SENDING
};




static void PacketLinkP__signalDone(error_t error);









static inline void PacketLinkP__PacketLink__setRetries(message_t *msg, uint16_t maxRetries);








static inline void PacketLinkP__PacketLink__setRetryDelay(message_t *msg, uint16_t retryDelay);






static inline uint16_t PacketLinkP__PacketLink__getRetries(message_t *msg);






static inline uint16_t PacketLinkP__PacketLink__getRetryDelay(message_t *msg);






static inline bool PacketLinkP__PacketLink__wasDelivered(message_t *msg);
#line 130
static inline error_t PacketLinkP__Send__send(message_t *msg, uint8_t len);
#line 171
static inline void PacketLinkP__SubSend__sendDone(message_t *msg, error_t error);
#line 202
static inline void PacketLinkP__DelayTimer__fired(void );






static inline void PacketLinkP__send__runTask(void );










static void PacketLinkP__signalDone(error_t error);
# 64 "/home/tinyos/local/src/tinyos-2.x/tos/interfaces/Send.nc"
static error_t CC2420Ieee154MessageP__SubSend__send(
#line 56
message_t * msg, 







uint8_t len);
#line 114
static 
#line 112
void * 

CC2420Ieee154MessageP__SubSend__getPayload(
#line 111
message_t * msg, 


uint8_t len);
# 64 "/home/tinyos/local/src/tinyos-2.x/tos/chips/cc2420/interfaces/CC2420Config.nc"
static uint16_t CC2420Ieee154MessageP__CC2420Config__getShortAddr(void );





static uint16_t CC2420Ieee154MessageP__CC2420Config__getPanAddr(void );
# 42 "/home/tinyos/local/src/tinyos-2.x/tos/chips/cc2420/interfaces/CC2420PacketBody.nc"
static cc2420_header_t * CC2420Ieee154MessageP__CC2420PacketBody__getHeader(message_t * msg);
# 86 "/home/tinyos/local/src/tinyos-2.x/tos/interfaces/Ieee154Send.nc"
static void CC2420Ieee154MessageP__Ieee154Send__sendDone(message_t *msg, error_t error);
# 73 "/home/tinyos/local/src/tinyos-2.x/tos/chips/cc2420/CC2420Ieee154MessageP.nc"
static inline error_t CC2420Ieee154MessageP__Ieee154Send__send(ieee154_saddr_t addr, 
message_t *msg, 
uint8_t len);
#line 101
static ieee154_saddr_t CC2420Ieee154MessageP__Ieee154Packet__destination(message_t *msg);




static ieee154_saddr_t CC2420Ieee154MessageP__Ieee154Packet__source(message_t *msg);




static inline void CC2420Ieee154MessageP__Ieee154Packet__setDestination(message_t *msg, ieee154_saddr_t addr);
#line 146
static inline uint8_t CC2420Ieee154MessageP__Packet__payloadLength(message_t *msg);



static inline void CC2420Ieee154MessageP__Packet__setPayloadLength(message_t *msg, uint8_t len);



static inline uint8_t CC2420Ieee154MessageP__Packet__maxPayloadLength(void );



static inline void *CC2420Ieee154MessageP__Packet__getPayload(message_t *msg, uint8_t len);





static inline void CC2420Ieee154MessageP__SubSend__sendDone(message_t *msg, error_t result);




static inline void CC2420Ieee154MessageP__CC2420Config__syncDone(error_t error);
# 92 "/home/tinyos/local/src/tinyos-2.x/tos/interfaces/SplitControl.nc"
static void IPDispatchP__SplitControl__startDone(error_t error);
#line 117
static void IPDispatchP__SplitControl__stopDone(error_t error);
# 6 "/home/tinyos/local/src/tinyos-2.x/tos/lib/net/blip/interfaces/IPExtensions.nc"
static void IPDispatchP__IPExtensions__handleExtensions(uint8_t label, 
struct ip6_hdr *iph, 
struct ip6_ext *hop, 
struct ip6_ext *dst, 
struct ip6_route *route, 
uint8_t nxt_hdr);





static void IPDispatchP__IPExtensions__reportTransmission(uint8_t label, send_policy_t *send);
# 97 "/home/tinyos/local/src/tinyos-2.x/tos/interfaces/Pool.nc"
static 
#line 94
IPDispatchP__SendInfoPool__t * 


IPDispatchP__SendInfoPool__get(void );
#line 89
static error_t IPDispatchP__SendInfoPool__put(
#line 85
IPDispatchP__SendInfoPool__t * newVal);
# 67 "/home/tinyos/local/src/tinyos-2.x/tos/interfaces/Packet.nc"
static uint8_t IPDispatchP__Packet__payloadLength(
#line 63
message_t * msg);
#line 115
static 
#line 112
void * 


IPDispatchP__Packet__getPayload(
#line 110
message_t * msg, 




uint8_t len);
#line 95
static uint8_t IPDispatchP__Packet__maxPayloadLength(void );
#line 83
static void IPDispatchP__Packet__setPayloadLength(
#line 79
message_t * msg, 



uint8_t len);
# 30 "/home/tinyos/local/src/tinyos-2.x/tos/interfaces/Ieee154Packet.nc"
static ieee154_saddr_t IPDispatchP__Ieee154Packet__source(message_t *msg);
#line 28
static ieee154_saddr_t IPDispatchP__Ieee154Packet__destination(message_t *msg);



static void IPDispatchP__Ieee154Packet__setDestination(message_t *msg, ieee154_saddr_t addr);
# 83 "/home/tinyos/local/src/tinyos-2.x/tos/interfaces/SplitControl.nc"
static error_t IPDispatchP__RadioControl__start(void );
#line 109
static error_t IPDispatchP__RadioControl__stop(void );
# 3 "/home/tinyos/local/src/tinyos-2.x/tos/lib/net/blip/interfaces/ReadLqi.nc"
static uint8_t IPDispatchP__ReadLqi__read(message_t *msg);
# 73 "/home/tinyos/local/src/tinyos-2.x/tos/interfaces/Queue.nc"
static 
#line 71
IPDispatchP__SendQueue__t  

IPDispatchP__SendQueue__head(void );
#line 90
static error_t IPDispatchP__SendQueue__enqueue(
#line 86
IPDispatchP__SendQueue__t  newVal);
#line 81
static 
#line 79
IPDispatchP__SendQueue__t  

IPDispatchP__SendQueue__dequeue(void );
#line 50
static bool IPDispatchP__SendQueue__empty(void );
# 53 "/home/tinyos/local/src/tinyos-2.x/tos/lib/timer/Timer.nc"
static void IPDispatchP__ExpireTimer__startPeriodic(uint32_t dt);
# 25 "/home/tinyos/local/src/tinyos-2.x/tos/lib/net/blip/interfaces/IPAddress.nc"
static ieee154_saddr_t IPDispatchP__IPAddress__getShortAddr(void );
# 6 "/home/tinyos/local/src/tinyos-2.x/tos/lib/net/blip/interfaces/InternalIPExtension.nc"
static void IPDispatchP__InternalIPExtension__ip_free(void );
#line 4
static void IPDispatchP__InternalIPExtension__addHeaders(struct split_ip_msg *msg, uint8_t nxt_hdr, uint16_t label);
# 56 "/home/tinyos/local/src/tinyos-2.x/tos/interfaces/TaskBasic.nc"
static error_t IPDispatchP__sendTask__postTask(void );
# 28 "/home/tinyos/local/src/tinyos-2.x/tos/lib/net/blip/interfaces/ICMP.nc"
static void IPDispatchP__ICMP__sendSolicitations(void );






static void IPDispatchP__ICMP__sendTimeExceeded(struct ip6_hdr *hdr, unpack_info_t *u_info, uint16_t amount_here);
# 97 "/home/tinyos/local/src/tinyos-2.x/tos/interfaces/Pool.nc"
static 
#line 94
IPDispatchP__FragPool__t * 


IPDispatchP__FragPool__get(void );
#line 89
static error_t IPDispatchP__FragPool__put(
#line 85
IPDispatchP__FragPool__t * newVal);
# 72 "/home/tinyos/local/src/tinyos-2.x/tos/interfaces/Leds.nc"
static void IPDispatchP__Leds__led1Toggle(void );
# 56 "/home/tinyos/local/src/tinyos-2.x/tos/interfaces/Ieee154Send.nc"
static error_t IPDispatchP__Ieee154Send__send(ieee154_saddr_t addr, message_t *msg, uint8_t len);
# 46 "/home/tinyos/local/src/tinyos-2.x/tos/interfaces/PacketLink.nc"
static void IPDispatchP__PacketLink__setRetries(
#line 42
message_t * msg, 



uint16_t maxRetries);
#line 59
static uint16_t IPDispatchP__PacketLink__getRetries(
#line 56
message_t * msg);
#line 53
static void IPDispatchP__PacketLink__setRetryDelay(message_t *msg, uint16_t retryDelay);
#line 71
static bool IPDispatchP__PacketLink__wasDelivered(
#line 68
message_t * msg);
# 97 "/home/tinyos/local/src/tinyos-2.x/tos/interfaces/Pool.nc"
static 
#line 94
IPDispatchP__SendEntryPool__t * 


IPDispatchP__SendEntryPool__get(void );
#line 89
static error_t IPDispatchP__SendEntryPool__put(
#line 85
IPDispatchP__SendEntryPool__t * newVal);
# 48 "/home/tinyos/local/src/tinyos-2.x/tos/lib/net/blip/interfaces/IPRouting.nc"
static error_t IPDispatchP__IPRouting__getNextHop(struct ip6_hdr *hdr, 
struct ip6_route *routing_hdr, 
ieee154_saddr_t prev_hop, 
send_policy_t *ret);






static uint8_t IPDispatchP__IPRouting__getHopLimit(void );
#line 40
static bool IPDispatchP__IPRouting__isForMe(struct ip6_hdr *a);
#line 74
static void IPDispatchP__IPRouting__reportReception(ieee154_saddr_t neigh, uint8_t lqi);






static struct ip6_route *IPDispatchP__IPRouting__insertRoutingHeader(struct split_ip_msg *msg);
# 25 "/home/tinyos/local/src/tinyos-2.x/tos/lib/net/blip/interfaces/IP.nc"
static void IPDispatchP__IP__recv(
# 90 "/home/tinyos/local/src/tinyos-2.x/tos/lib/net/blip/IPDispatchP.nc"
uint8_t arg_0x41134178, 
# 25 "/home/tinyos/local/src/tinyos-2.x/tos/lib/net/blip/interfaces/IP.nc"
struct ip6_hdr *iph, void *payload, struct ip_metadata *meta);
# 176 "/home/tinyos/local/src/tinyos-2.x/tos/lib/net/blip/IPDispatchP.nc"
enum IPDispatchP____nesc_unnamed4420 {
#line 176
  IPDispatchP__sendTask = 13U
};
#line 176
typedef int IPDispatchP____nesc_sillytask_sendTask[IPDispatchP__sendTask];
#line 140
enum IPDispatchP____nesc_unnamed4421 {
  IPDispatchP__S_RUNNING, 
  IPDispatchP__S_STOPPED, 
  IPDispatchP__S_STOPPING
};
uint8_t IPDispatchP__state = IPDispatchP__S_STOPPED;
bool IPDispatchP__radioBusy;
uint8_t IPDispatchP__current_local_label = 0;
ip_statistics_t IPDispatchP__stats;
#line 161
table_t IPDispatchP__recon_cache;
#line 161
table_t IPDispatchP__forward_cache;




reconstruct_t IPDispatchP__recon_data[N_RECONSTRUCTIONS];



forward_entry_t IPDispatchP__forward_data[N_FORWARD_ENT];







static inline void IPDispatchP__reconstruct_clear(void *ent);






static inline void IPDispatchP__forward_clear(void *ent);




static inline int IPDispatchP__forward_unused(void *ent);






uint16_t IPDispatchP__forward_lookup_tag;
uint16_t IPDispatchP__forward_lookup_src;
static inline int IPDispatchP__forward_lookup(void *ent);
#line 211
static send_info_t *IPDispatchP__getSendInfo(void );










static inline error_t IPDispatchP__SplitControl__start(void );
#line 238
static inline void IPDispatchP__RadioControl__startDone(error_t error);










static inline void IPDispatchP__RadioControl__stopDone(error_t error);



static inline void IPDispatchP__Boot__booted(void );
#line 281
static inline void IPDispatchP__signalDone(reconstruct_t *recon);
#line 312
static inline void IPDispatchP__reconstruct_age(void *elt);
#line 331
static inline void IPDispatchP__forward_age(void *elt);
#line 347
static inline void IPDispatchP__ip_print_heap(void );










static inline void IPDispatchP__ExpireTimer__fired(void );
#line 375
static reconstruct_t *IPDispatchP__get_reconstruct(ieee154_saddr_t src, uint16_t tag);
#line 412
static inline void IPDispatchP__updateSourceRoute(ieee154_saddr_t prev_hop, struct ip6_route *sh);
#line 433
static inline message_t *IPDispatchP__handle1stFrag(message_t *msg, packed_lowmsg_t *lowmsg);
#line 651
static inline message_t *IPDispatchP__Ieee154Receive__receive(message_t *msg, void *msg_payload, uint8_t len);
#line 781
static inline void IPDispatchP__sendTask__runTask(void );
#line 847
static error_t IPDispatchP__IP__send(uint8_t prot, struct split_ip_msg *msg);




static inline error_t IPDispatchP__IP__bareSend(uint8_t prot, struct split_ip_msg *msg, 
struct ip6_route *route, 
int flags);
#line 974
static void IPDispatchP__Ieee154Send__sendDone(message_t *msg, error_t error);
#line 1040
static inline void IPDispatchP__ICMP__solicitationDone(void );






static inline void IPDispatchP__Statistics__get(ip_statistics_t *statistics);
#line 1065
static inline void IPDispatchP__Statistics__clear(void );



static void IPDispatchP__IP__default__recv(uint8_t nxt_hdr, struct ip6_hdr *iph, 
void *payload, 
struct ip_metadata *meta);
# 40 "/home/tinyos/local/src/tinyos-2.x/tos/lib/net/blip/IPAddressP.nc"
static inline ieee154_saddr_t IPAddressP__IPAddress__getShortAddr(void );
#line 53
static void IPAddressP__IPAddress__getLLAddr(struct in6_addr *addr);





static void IPAddressP__IPAddress__getIPAddr(struct in6_addr *addr);




static struct in6_addr *IPAddressP__IPAddress__getPublicAddr(void );




static inline void IPAddressP__IPAddress__setPrefix(uint8_t *pfx);





static inline bool IPAddressP__IPAddress__haveAddress(void );



static void IPAddressP__IPAddress__setSource(struct ip6_hdr *hdr);
# 41 "/home/tinyos/local/src/tinyos-2.x/tos/interfaces/Random.nc"
static uint16_t IPRoutingP__Random__rand16(void );
# 30 "/home/tinyos/local/src/tinyos-2.x/tos/lib/net/blip/interfaces/IPAddress.nc"
static void IPRoutingP__IPAddress__getIPAddr(struct in6_addr *addr);
#line 28
static struct in6_addr *IPRoutingP__IPAddress__getPublicAddr(void );
# 53 "/home/tinyos/local/src/tinyos-2.x/tos/lib/timer/Timer.nc"
static void IPRoutingP__SortTimer__startPeriodic(uint32_t dt);
# 33 "/home/tinyos/local/src/tinyos-2.x/tos/lib/net/blip/interfaces/ICMP.nc"
static void IPRoutingP__ICMP__sendAdvertisements(void );
#line 28
static void IPRoutingP__ICMP__sendSolicitations(void );
# 15 "/home/tinyos/local/src/tinyos-2.x/tos/lib/net/blip/interfaces/IP.nc"
static error_t IPRoutingP__TGenSend__send(struct split_ip_msg *msg);
# 81 "/home/tinyos/local/src/tinyos-2.x/tos/lib/timer/Timer.nc"
static bool IPRoutingP__TrafficGenTimer__isRunning(void );
#line 62
static void IPRoutingP__TrafficGenTimer__startOneShot(uint32_t dt);




static void IPRoutingP__TrafficGenTimer__stop(void );
# 51 "/home/tinyos/local/src/tinyos-2.x/tos/lib/net/blip/IPRoutingP.nc"
enum IPRoutingP____nesc_unnamed4422 {
  IPRoutingP__SHORT_EPOCH = 0, 
  IPRoutingP__LONG_EPOCH = 1
};



uint16_t IPRoutingP__last_qual;
uint8_t IPRoutingP__last_hops;
uint16_t IPRoutingP__reportSeqno;

bool IPRoutingP__soliciting;



struct neigh_entry *IPRoutingP__default_route;
uint16_t IPRoutingP__default_route_failures;

uint32_t IPRoutingP__traffic_interval;
bool IPRoutingP__traffic_sent;






struct neigh_entry IPRoutingP__neigh_table[N_NEIGH];

static inline void IPRoutingP__printTable(void );


static inline void IPRoutingP__updateRankings(void );
static void IPRoutingP__swapNodes(struct neigh_entry *highNode, struct neigh_entry *lowNode);
static uint8_t IPRoutingP__checkThresh(uint32_t firstVal, uint32_t secondVal, uint16_t thresh);
static void IPRoutingP__evictNeighbor(struct neigh_entry *neigh);
static uint16_t IPRoutingP__getMetric(struct neigh_entry *neigh);

static inline void IPRoutingP__clearStats(struct neigh_entry *r);
#line 109
static void IPRoutingP__restartTrafficGen(void );










static inline void IPRoutingP__TrafficGenTimer__fired(void );
#line 150
static inline void IPRoutingP__TGenSend__recv(struct ip6_hdr *iph, 
void *payload, 
struct ip_metadata *meta);



static void IPRoutingP__IPRouting__reset(void );
#line 188
static inline void IPRoutingP__Boot__booted(void );








static inline bool IPRoutingP__IPRouting__isForMe(struct ip6_hdr *hdr);
#line 287
static struct neigh_entry *IPRoutingP__getNeighEntry(cmpr_ip6_addr_t a);
#line 513
static inline void IPRoutingP__IPExtensions__handleExtensions(uint8_t label, 
struct ip6_hdr *iph, 
struct ip6_ext *hop, 
struct ip6_ext *dst, 
struct ip6_route *route, 
uint8_t nxt_hdr);
#line 622
static uint16_t IPRoutingP__getConfidence(struct neigh_entry *neigh);
#line 646
static inline uint16_t IPRoutingP__getSuccess(struct neigh_entry *neigh);
#line 658
static uint16_t IPRoutingP__getLinkCost(struct neigh_entry *neigh);









static inline void IPRoutingP__printTable(void );
#line 716
static uint16_t IPRoutingP__getMetric(struct neigh_entry *r);





static void IPRoutingP__chooseNewRandomDefault(bool force);
#line 774
static error_t IPRoutingP__IPRouting__getNextHop(struct ip6_hdr *hdr, 
struct ip6_route *sh, 
ieee154_saddr_t prev_hop, 
send_policy_t *ret);
#line 878
static uint8_t IPRoutingP__IPRouting__getHopLimit(void );






static uint16_t IPRoutingP__IPRouting__getQuality(void );
#line 911
static inline void IPRoutingP__IPRouting__reportAdvertisement(ieee154_saddr_t neigh, uint8_t hops, 
uint8_t lqi, uint16_t cost);
#line 1001
static inline void IPRoutingP__IPRouting__reportReception(ieee154_saddr_t neigh, uint8_t lqi);
#line 1019
static inline void IPRoutingP__IPExtensions__reportTransmission(uint8_t label, send_policy_t *policy);
#line 1095
static inline bool IPRoutingP__IPRouting__hasRoute(void );
#line 1156
static inline struct tlv_hdr *IPRoutingP__DestinationExt__getHeader(int label, int nxt_hdr, 
struct ip6_hdr *iph);
#line 1221
static inline struct ip6_route *IPRoutingP__IPRouting__insertRoutingHeader(struct split_ip_msg *msg);
#line 1247
static inline void IPRoutingP__SortTimer__fired(void );
#line 1282
static inline void IPRoutingP__ICMP__solicitationDone(void );
#line 1295
static inline void IPRoutingP__Statistics__get(route_statistics_t *statistics);
#line 1309
static inline void IPRoutingP__Statistics__clear(void );



static void IPRoutingP__evictNeighbor(struct neigh_entry *neigh);
#line 1344
static inline void IPRoutingP__updateRankings(void );
#line 1404
static void IPRoutingP__swapNodes(struct neigh_entry *highNode, struct neigh_entry *lowNode);










static uint8_t IPRoutingP__checkThresh(uint32_t firstVal, uint32_t secondVal, uint16_t thresh);
# 48 "/home/tinyos/local/src/tinyos-2.x/tos/system/NoLedsC.nc"
static inline void NoLedsC__Leds__led1Toggle(void );
# 56 "/home/tinyos/local/src/tinyos-2.x/tos/interfaces/Ieee154Send.nc"
static error_t ResourceSendP__SubSend__send(ieee154_saddr_t addr, message_t *msg, uint8_t len);
#line 86
static void ResourceSendP__Ieee154Send__sendDone(message_t *msg, error_t error);
# 110 "/home/tinyos/local/src/tinyos-2.x/tos/interfaces/Resource.nc"
static error_t ResourceSendP__Resource__release(void );
#line 78
static error_t ResourceSendP__Resource__request(void );
# 11 "/home/tinyos/local/src/tinyos-2.x/tos/lib/net/blip/ResourceSendP.nc"
ieee154_saddr_t ResourceSendP__m_addr;
message_t *ResourceSendP__m_msg = (void *)0;
uint8_t ResourceSendP__m_len;

static inline error_t ResourceSendP__Ieee154Send__send(ieee154_saddr_t addr, 
message_t *msg, 
uint8_t len);










static inline void ResourceSendP__SubSend__sendDone(message_t *msg, error_t result);





static void ResourceSendP__Resource__granted(void );
# 72 "/home/tinyos/local/src/tinyos-2.x/tos/chips/cc2420/interfaces/CC2420Packet.nc"
static uint8_t CC2420ReadLqiC__CC2420Packet__getLqi(message_t *p_msg);
# 12 "/home/tinyos/local/src/tinyos-2.x/tos/lib/net/blip/platform/CC2420ReadLqiC.nc"
static inline uint8_t CC2420ReadLqiC__ReadLqi__read(message_t *msg);
# 60 "/home/tinyos/local/src/tinyos-2.x/tos/system/PoolP.nc"
uint8_t /*IPDispatchC.FragPool.PoolP*/PoolP__0__free;
uint8_t /*IPDispatchC.FragPool.PoolP*/PoolP__0__index;
/*IPDispatchC.FragPool.PoolP*/PoolP__0__pool_t * /*IPDispatchC.FragPool.PoolP*/PoolP__0__queue[14];
/*IPDispatchC.FragPool.PoolP*/PoolP__0__pool_t /*IPDispatchC.FragPool.PoolP*/PoolP__0__pool[14];

static inline error_t /*IPDispatchC.FragPool.PoolP*/PoolP__0__Init__init(void );
#line 88
static /*IPDispatchC.FragPool.PoolP*/PoolP__0__pool_t */*IPDispatchC.FragPool.PoolP*/PoolP__0__Pool__get(void );
#line 103
static error_t /*IPDispatchC.FragPool.PoolP*/PoolP__0__Pool__put(/*IPDispatchC.FragPool.PoolP*/PoolP__0__pool_t *newVal);
#line 60
uint8_t /*IPDispatchC.SendEntryPool.PoolP*/PoolP__1__free;
uint8_t /*IPDispatchC.SendEntryPool.PoolP*/PoolP__1__index;
/*IPDispatchC.SendEntryPool.PoolP*/PoolP__1__pool_t * /*IPDispatchC.SendEntryPool.PoolP*/PoolP__1__queue[14];
/*IPDispatchC.SendEntryPool.PoolP*/PoolP__1__pool_t /*IPDispatchC.SendEntryPool.PoolP*/PoolP__1__pool[14];

static inline error_t /*IPDispatchC.SendEntryPool.PoolP*/PoolP__1__Init__init(void );
#line 88
static /*IPDispatchC.SendEntryPool.PoolP*/PoolP__1__pool_t */*IPDispatchC.SendEntryPool.PoolP*/PoolP__1__Pool__get(void );
#line 103
static error_t /*IPDispatchC.SendEntryPool.PoolP*/PoolP__1__Pool__put(/*IPDispatchC.SendEntryPool.PoolP*/PoolP__1__pool_t *newVal);
# 48 "/home/tinyos/local/src/tinyos-2.x/tos/system/QueueC.nc"
/*IPDispatchC.QueueC*/QueueC__0__queue_t  /*IPDispatchC.QueueC*/QueueC__0__queue[14];
uint8_t /*IPDispatchC.QueueC*/QueueC__0__head = 0;
uint8_t /*IPDispatchC.QueueC*/QueueC__0__tail = 0;
uint8_t /*IPDispatchC.QueueC*/QueueC__0__size = 0;

static inline bool /*IPDispatchC.QueueC*/QueueC__0__Queue__empty(void );



static inline uint8_t /*IPDispatchC.QueueC*/QueueC__0__Queue__size(void );



static inline uint8_t /*IPDispatchC.QueueC*/QueueC__0__Queue__maxSize(void );



static inline /*IPDispatchC.QueueC*/QueueC__0__queue_t /*IPDispatchC.QueueC*/QueueC__0__Queue__head(void );



static inline void /*IPDispatchC.QueueC*/QueueC__0__printQueue(void );
#line 85
static /*IPDispatchC.QueueC*/QueueC__0__queue_t /*IPDispatchC.QueueC*/QueueC__0__Queue__dequeue(void );
#line 97
static error_t /*IPDispatchC.QueueC*/QueueC__0__Queue__enqueue(/*IPDispatchC.QueueC*/QueueC__0__queue_t newVal);
# 60 "/home/tinyos/local/src/tinyos-2.x/tos/system/PoolP.nc"
uint8_t /*IPDispatchC.SendInfoPool.PoolP*/PoolP__2__free;
uint8_t /*IPDispatchC.SendInfoPool.PoolP*/PoolP__2__index;
/*IPDispatchC.SendInfoPool.PoolP*/PoolP__2__pool_t * /*IPDispatchC.SendInfoPool.PoolP*/PoolP__2__queue[14];
/*IPDispatchC.SendInfoPool.PoolP*/PoolP__2__pool_t /*IPDispatchC.SendInfoPool.PoolP*/PoolP__2__pool[14];

static inline error_t /*IPDispatchC.SendInfoPool.PoolP*/PoolP__2__Init__init(void );
#line 88
static inline /*IPDispatchC.SendInfoPool.PoolP*/PoolP__2__pool_t */*IPDispatchC.SendInfoPool.PoolP*/PoolP__2__Pool__get(void );
#line 103
static error_t /*IPDispatchC.SendInfoPool.PoolP*/PoolP__2__Pool__put(/*IPDispatchC.SendInfoPool.PoolP*/PoolP__2__pool_t *newVal);
# 81 "/home/tinyos/local/src/tinyos-2.x/tos/lib/timer/Timer.nc"
static bool ICMPResponderP__PingTimer__isRunning(void );
#line 53
static void ICMPResponderP__PingTimer__startPeriodic(uint32_t dt);
#line 67
static void ICMPResponderP__PingTimer__stop(void );
# 79 "/home/tinyos/local/src/tinyos-2.x/tos/lib/net/blip/interfaces/IPRouting.nc"
static bool ICMPResponderP__IPRouting__hasRoute(void );



static void ICMPResponderP__IPRouting__reset(void );
#line 60
static uint16_t ICMPResponderP__IPRouting__getQuality(void );
#line 58
static uint8_t ICMPResponderP__IPRouting__getHopLimit(void );







static void ICMPResponderP__IPRouting__reportAdvertisement(ieee154_saddr_t neigh, uint8_t hops, 
uint8_t lqi, uint16_t cost);
# 50 "/home/tinyos/local/src/tinyos-2.x/tos/lib/timer/LocalTime.nc"
static uint32_t ICMPResponderP__LocalTime__get(void );
# 41 "/home/tinyos/local/src/tinyos-2.x/tos/interfaces/Random.nc"
static uint16_t ICMPResponderP__Random__rand16(void );
# 29 "/home/tinyos/local/src/tinyos-2.x/tos/lib/net/blip/interfaces/IPAddress.nc"
static void ICMPResponderP__IPAddress__getLLAddr(struct in6_addr *addr);


static void ICMPResponderP__IPAddress__setSource(struct ip6_hdr *hdr);
#line 30
static void ICMPResponderP__IPAddress__getIPAddr(struct in6_addr *addr);



static void ICMPResponderP__IPAddress__setPrefix(uint8_t *prefix);
#line 28
static struct in6_addr *ICMPResponderP__IPAddress__getPublicAddr(void );
# 81 "/home/tinyos/local/src/tinyos-2.x/tos/lib/timer/Timer.nc"
static bool ICMPResponderP__Advertisement__isRunning(void );
#line 62
static void ICMPResponderP__Advertisement__startOneShot(uint32_t dt);
# 31 "/home/tinyos/local/src/tinyos-2.x/tos/lib/net/blip/interfaces/ICMP.nc"
static void ICMPResponderP__ICMP__solicitationDone(void );
# 81 "/home/tinyos/local/src/tinyos-2.x/tos/lib/timer/Timer.nc"
static bool ICMPResponderP__Solicitation__isRunning(void );
#line 62
static void ICMPResponderP__Solicitation__startOneShot(uint32_t dt);
# 10 "/home/tinyos/local/src/tinyos-2.x/tos/lib/net/blip/interfaces/ICMPPing.nc"
static void ICMPResponderP__ICMPPing__pingDone(
# 35 "/home/tinyos/local/src/tinyos-2.x/tos/lib/net/blip/ICMPResponderP.nc"
uint16_t arg_0x412fc2e0, 
# 10 "/home/tinyos/local/src/tinyos-2.x/tos/lib/net/blip/interfaces/ICMPPing.nc"
uint16_t ping_rcv, uint16_t ping_n);
#line 8
static void ICMPResponderP__ICMPPing__pingReply(
# 35 "/home/tinyos/local/src/tinyos-2.x/tos/lib/net/blip/ICMPResponderP.nc"
uint16_t arg_0x412fc2e0, 
# 8 "/home/tinyos/local/src/tinyos-2.x/tos/lib/net/blip/interfaces/ICMPPing.nc"
struct in6_addr *source, struct icmp_stats *stats);
# 15 "/home/tinyos/local/src/tinyos-2.x/tos/lib/net/blip/interfaces/IP.nc"
static error_t ICMPResponderP__IP__send(struct split_ip_msg *msg);
# 53 "/home/tinyos/local/src/tinyos-2.x/tos/lib/net/blip/ICMPResponderP.nc"
icmp_statistics_t ICMPResponderP__stats;
uint32_t ICMPResponderP__solicitation_period;
uint32_t ICMPResponderP__advertisement_period;
uint16_t ICMPResponderP__nd_seqno = 0;

uint16_t ICMPResponderP__ping_seq;
#line 58
uint16_t ICMPResponderP__ping_n;
#line 58
uint16_t ICMPResponderP__ping_rcv;
#line 58
uint16_t ICMPResponderP__ping_ident;
struct in6_addr ICMPResponderP__ping_dest;






static inline uint16_t ICMPResponderP__ICMP__cksum(struct split_ip_msg *msg, uint8_t nxt_hdr);




static void ICMPResponderP__ICMP__sendSolicitations(void );






static void ICMPResponderP__ICMP__sendAdvertisements(void );








static inline void ICMPResponderP__ICMP__sendTimeExceeded(struct ip6_hdr *hdr, unpack_info_t *u_info, uint16_t amount_here);
#line 150
static inline void ICMPResponderP__sendSolicitation(void );
#line 183
static inline void ICMPResponderP__sendPing(struct in6_addr *dest, uint16_t seqno);
#line 214
static inline void ICMPResponderP__handleRouterAdv(void *payload, uint16_t len, struct ip_metadata *meta);
#line 254
static inline void ICMPResponderP__sendAdvertisement(void );
#line 321
static inline void ICMPResponderP__IP__recv(struct ip6_hdr *iph, 
void *payload, 
struct ip_metadata *meta);
#line 383
static inline void ICMPResponderP__Solicitation__fired(void );










static inline void ICMPResponderP__Advertisement__fired(void );










static inline error_t ICMPResponderP__ICMPPing__ping(uint16_t client, struct in6_addr *target, uint16_t period, uint16_t n);
#line 417
static inline void ICMPResponderP__PingTimer__fired(void );
#line 430
static inline void ICMPResponderP__Statistics__get(icmp_statistics_t *statistics);



static inline void ICMPResponderP__Statistics__clear(void );



static inline void ICMPResponderP__ICMPPing__default__pingReply(uint16_t client, struct in6_addr *source, 
struct icmp_stats *ping_stats);


static inline void ICMPResponderP__ICMPPing__default__pingDone(uint16_t client, uint16_t n, uint16_t m);
# 5 "/home/tinyos/local/src/tinyos-2.x/tos/lib/net/blip/interfaces/TLVHeader.nc"
static struct tlv_hdr *IPExtensionP__HopByHopExt__getHeader(
# 16 "/home/tinyos/local/src/tinyos-2.x/tos/lib/net/blip/IPExtensionP.nc"
uint8_t arg_0x413a5710, 
# 5 "/home/tinyos/local/src/tinyos-2.x/tos/lib/net/blip/interfaces/TLVHeader.nc"
int label, int nxt_hdr, 
struct ip6_hdr *msg);
#line 5
static struct tlv_hdr *IPExtensionP__DestinationExt__getHeader(
# 17 "/home/tinyos/local/src/tinyos-2.x/tos/lib/net/blip/IPExtensionP.nc"
uint8_t arg_0x413a5f20, 
# 5 "/home/tinyos/local/src/tinyos-2.x/tos/lib/net/blip/interfaces/TLVHeader.nc"
int label, int nxt_hdr, 
struct ip6_hdr *msg);
# 22 "/home/tinyos/local/src/tinyos-2.x/tos/lib/net/blip/IPExtensionP.nc"
struct generic_header *IPExtensionP__ext_dest;
#line 22
struct generic_header *IPExtensionP__ext_hop;

static inline error_t IPExtensionP__Init__init(void );
#line 39
static struct generic_header *IPExtensionP__buildTLVHdr(struct split_ip_msg *msg, 
int which, 
int n, int nxt_hdr);
#line 87
static inline void IPExtensionP__InternalIPExtension__addHeaders(struct split_ip_msg *msg, 
uint8_t nxt_hdr, 
uint16_t label);










static inline void IPExtensionP__InternalIPExtension__ip_free(void );
#line 132
static inline struct tlv_hdr *IPExtensionP__DestinationExt__default__getHeader(uint8_t i, int label, int nxt_hdr, 
struct ip6_hdr *msg);




static inline struct tlv_hdr *IPExtensionP__HopByHopExt__default__getHeader(uint8_t i, int label, int nxt_hdr, 
struct ip6_hdr *msg);
# 32 "/home/tinyos/local/src/tinyos-2.x/tos/lib/net/blip/interfaces/IPAddress.nc"
static void UdpP__IPAddress__setSource(struct ip6_hdr *hdr);
# 24 "/home/tinyos/local/src/tinyos-2.x/tos/lib/net/blip/interfaces/UDP.nc"
static void UdpP__UDP__recvfrom(
# 7 "/home/tinyos/local/src/tinyos-2.x/tos/lib/net/blip/UdpP.nc"
uint8_t arg_0x413c2e40, 
# 24 "/home/tinyos/local/src/tinyos-2.x/tos/lib/net/blip/interfaces/UDP.nc"
struct sockaddr_in6 *src, void *payload, 
uint16_t len, struct ip_metadata *meta);
# 15 "/home/tinyos/local/src/tinyos-2.x/tos/lib/net/blip/interfaces/IP.nc"
static error_t UdpP__IP__send(struct split_ip_msg *msg);
# 19 "/home/tinyos/local/src/tinyos-2.x/tos/lib/net/blip/UdpP.nc"
enum UdpP____nesc_unnamed4423 {
  UdpP__N_CLIENTS = 3U
};

udp_statistics_t UdpP__stats;
uint16_t UdpP__local_ports[UdpP__N_CLIENTS];

enum UdpP____nesc_unnamed4424 {
  UdpP__LOCAL_PORT_START = 51024U, 
  UdpP__LOCAL_PORT_STOP = 54999U
};
uint16_t UdpP__last_localport = UdpP__LOCAL_PORT_START;

static inline uint16_t UdpP__alloc_lport(uint8_t clnt);
#line 50
static inline error_t UdpP__Init__init(void );





static error_t UdpP__UDP__bind(uint8_t clnt, uint16_t port);
#line 68
static inline void UdpP__IP__recv(struct ip6_hdr *iph, 
void *payload, 
struct ip_metadata *meta);
#line 128
static error_t UdpP__UDP__sendto(uint8_t clnt, struct sockaddr_in6 *dest, void *payload, 
uint16_t len);
#line 183
static inline void UdpP__Statistics__clear(void );





static inline void UdpP__Statistics__get(udp_statistics_t *buf);





static inline void UdpP__UDP__default__recvfrom(uint8_t clnt, struct sockaddr_in6 *from, void *payload, 
uint16_t len, struct ip_metadata *meta);
# 53 "/home/tinyos/local/src/tinyos-2.x/tos/lib/timer/Timer.nc"
static void TcpP__Timer__startPeriodic(uint32_t dt);
# 16 "/home/tinyos/local/src/tinyos-2.x/tos/lib/net/blip/interfaces/Tcp.nc"
static bool TcpP__Tcp__accept(
# 6 "/home/tinyos/local/src/tinyos-2.x/tos/lib/net/blip/TcpP.nc"
uint8_t arg_0x413d8b78, 
# 16 "/home/tinyos/local/src/tinyos-2.x/tos/lib/net/blip/interfaces/Tcp.nc"
struct sockaddr_in6 *from, 
void **tx_buf, int *tx_buf_len);
#line 34
static void TcpP__Tcp__recv(
# 6 "/home/tinyos/local/src/tinyos-2.x/tos/lib/net/blip/TcpP.nc"
uint8_t arg_0x413d8b78, 
# 34 "/home/tinyos/local/src/tinyos-2.x/tos/lib/net/blip/interfaces/Tcp.nc"
void *payload, uint16_t len);
#line 47
static void TcpP__Tcp__closed(
# 6 "/home/tinyos/local/src/tinyos-2.x/tos/lib/net/blip/TcpP.nc"
uint8_t arg_0x413d8b78, 
# 47 "/home/tinyos/local/src/tinyos-2.x/tos/lib/net/blip/interfaces/Tcp.nc"
error_t e);




static void TcpP__Tcp__acked(
# 6 "/home/tinyos/local/src/tinyos-2.x/tos/lib/net/blip/TcpP.nc"
uint8_t arg_0x413d8b78);
# 32 "/home/tinyos/local/src/tinyos-2.x/tos/lib/net/blip/interfaces/IPAddress.nc"
static void TcpP__IPAddress__setSource(struct ip6_hdr *hdr);
# 15 "/home/tinyos/local/src/tinyos-2.x/tos/lib/net/blip/interfaces/IP.nc"
static error_t TcpP__IP__send(struct split_ip_msg *msg);
# 17 "/home/tinyos/local/src/tinyos-2.x/tos/lib/net/blip/TcpP.nc"
enum TcpP____nesc_unnamed4425 {
  TcpP__N_CLIENTS = 2U
};
# 53 "/home/tinyos/local/src/tinyos-2.x/support/sdk/c/blip/libtcp/tcplib.h"
#line 41
typedef enum TcpP____nesc_unnamed4426 {
  TcpP__TCP_CLOSED = 0, 
  TcpP__TCP_LISTEN, 
  TcpP__TCP_SYN_RCVD, 
  TcpP__TCP_SYN_SENT, 
  TcpP__TCP_ESTABLISHED, 
  TcpP__TCP_CLOSE_WAIT, 
  TcpP__TCP_LAST_ACK, 
  TcpP__TCP_FIN_WAIT_1, 
  TcpP__TCP_FIN_WAIT_2, 
  TcpP__TCP_CLOSING, 
  TcpP__TCP_TIME_WAIT
} TcpP__tcplib_sock_state_t;

enum TcpP____nesc_unnamed4427 {
  TcpP__TCP_ACKPENDING = 0x3, 
  TcpP__TCP_DUPACKS = 0x3c, 
  TcpP__TCP_DUPACKS_OFF = 2, 
  TcpP__TCP_ACKSENT = 0x80
};

enum TcpP____nesc_unnamed4428 {

  TcpP__TCPLIB_TIMEWAIT_LEN = 1, 
  TcpP__TCPLIB_2MSL = 4, 

  TcpP__TCPLIB_GIVEUP = 6
};





struct TcpP__tcplib_sock {
  uint8_t flags;


  struct sockaddr_in6 l_ep;
  struct sockaddr_in6 r_ep;


  TcpP__tcplib_sock_state_t state;

  void *tx_buf;
  uint16_t tx_buf_len;




  uint16_t mss;

  uint16_t my_wind;


  uint16_t r_wind;
  uint16_t cwnd;
  uint16_t ssthresh;


  uint32_t seqno;

  uint32_t ackno;

  struct TcpP____nesc_unnamed4429 {
    int8_t retx;
  } timer;


  uint16_t retxcnt;
#line 130
  struct TcpP__tcplib_sock *next;
};
#line 148
static inline struct TcpP__tcplib_sock *TcpP__tcplib_accept(struct TcpP__tcplib_sock *conn, 
struct sockaddr_in6 *from);


static void TcpP__tcplib_send_out(struct split_ip_msg *msg, struct tcp_hdr *tcph);







static inline int TcpP__tcplib_process(struct ip6_hdr *ip_packet, void *payload);




static inline int TcpP__tcplib_timer_process(void );









static int TcpP__tcplib_init_sock(struct TcpP__tcplib_sock *sock);


static inline int TcpP__tcplib_bind(struct TcpP__tcplib_sock *sock, 
struct sockaddr_in6 *addr);
#line 193
static inline int TcpP__tcplib_send(struct TcpP__tcplib_sock *sock, 
void *data, int len);

static int TcpP__tcplib_close(struct TcpP__tcplib_sock *sock);
# 22 "/home/tinyos/local/src/tinyos-2.x/tos/lib/net/blip/TcpP.nc"
struct TcpP__tcplib_sock TcpP__socks[TcpP__N_CLIENTS];

static int TcpP__find_client(struct TcpP__tcplib_sock *conn);
#line 38
static inline void TcpP__tcplib_extern_recv(struct TcpP__tcplib_sock *sock, void *data, int len);





static inline void TcpP__tcplib_extern_closed(struct TcpP__tcplib_sock *sock);



static void TcpP__tcplib_extern_closedone(struct TcpP__tcplib_sock *sock);






static inline void TcpP__tcplib_extern_acked(struct TcpP__tcplib_sock *sock);
# 29 "/home/tinyos/local/src/tinyos-2.x/support/sdk/c/blip/libtcp/circ.c"
struct TcpP__circ_buf {
  uint8_t *data_start;
  uint8_t *data_head;
  uint16_t data_len;
  uint32_t head_seqno;
};

static inline int TcpP__circ_buf_init(void *data, int len, uint32_t seqno);
#line 48
static inline uint32_t TcpP__circ_get_seqno(void *buf);




static void TcpP__get_ptr_off_1(struct TcpP__circ_buf *b, uint32_t sseqno, int len, 
uint8_t **writeptr, int *w_len);
#line 74
static inline int TcpP__circ_shorten_head(void *buf, uint32_t seqno);
#line 87
static inline int TcpP__circ_buf_read(void *buf, uint32_t sseqno, 
uint8_t *data, int len);
#line 107
static inline int TcpP__circ_buf_write(char *buf, uint32_t sseqno, 
uint8_t *data, int len);
# 27 "/home/tinyos/local/src/tinyos-2.x/support/sdk/c/blip/libtcp/circ.h"
static inline int TcpP__circ_buf_init(void *data, int len, uint32_t seqno);


static inline int TcpP__circ_buf_write(char *buf, uint32_t sseqno, 
uint8_t *data, int len);


static inline int TcpP__circ_buf_read(void *buf, uint32_t sseqno, 
uint8_t *data, int len);


static inline int TcpP__circ_shorten_head(void *buf, uint32_t seqno);






static inline uint32_t TcpP__circ_get_seqno(void *buf);
# 39 "/home/tinyos/local/src/tinyos-2.x/support/sdk/c/blip/libtcp/tcplib.c"
static struct TcpP__tcplib_sock *TcpP__conns = (void *)0;







static __inline void TcpP__conn_add_once(struct TcpP__tcplib_sock *sock);
#line 59
inline static int TcpP__isInaddrAny(struct in6_addr *addr);
#line 90
inline static struct TcpP__tcplib_sock *TcpP__conn_lookup(struct ip6_hdr *iph, 
struct tcp_hdr *tcph);
#line 109
inline static int TcpP__conn_checkport(uint16_t port);









static inline struct tcp_hdr *TcpP__find_tcp_hdr(struct split_ip_msg *msg);







static struct split_ip_msg *TcpP__get_ipmsg(int plen);
#line 142
static void TcpP____tcplib_send(struct TcpP__tcplib_sock *sock, 
struct split_ip_msg *msg);
#line 161
static void TcpP__tcplib_send_ack(struct TcpP__tcplib_sock *sock, int fin_seqno, uint8_t flags);
#line 180
static void TcpP__tcplib_send_rst(struct ip6_hdr *iph, struct tcp_hdr *tcph);
#line 209
static int TcpP__tcplib_output(struct TcpP__tcplib_sock *sock, uint32_t sseqno);
#line 245
static int TcpP__tcplib_init_sock(struct TcpP__tcplib_sock *sock);
#line 259
inline static void TcpP__receive_data(struct TcpP__tcplib_sock *sock, struct tcp_hdr *tcph, int len);
#line 272
static void TcpP__reset_ssthresh(struct TcpP__tcplib_sock *conn);






static inline int TcpP__tcplib_process(struct ip6_hdr *iph, void *payload);
#line 542
static inline int TcpP__tcplib_bind(struct TcpP__tcplib_sock *sock, 
struct sockaddr_in6 *addr);
#line 588
static inline int TcpP__tcplib_send(struct TcpP__tcplib_sock *sock, void *data, int len);
#line 608
static inline void TcpP__tcplib_retx_expire(struct TcpP__tcplib_sock *sock);
#line 675
static int TcpP__tcplib_close(struct TcpP__tcplib_sock *sock);
#line 702
static inline int TcpP__tcplib_timer_process(void );
# 63 "/home/tinyos/local/src/tinyos-2.x/tos/lib/net/blip/TcpP.nc"
struct TcpP__tcplib_sock TcpP__socks[2U];

static inline struct TcpP__tcplib_sock *TcpP__tcplib_accept(struct TcpP__tcplib_sock *conn, 
struct sockaddr_in6 *from);
#line 81
static void TcpP__tcplib_send_out(struct split_ip_msg *msg, struct tcp_hdr *tcph);






static inline error_t TcpP__Init__init(void );







static inline void TcpP__Boot__booted(void );



static inline void TcpP__Timer__fired(void );



static inline void TcpP__IP__recv(struct ip6_hdr *iph, 
void *payload, 
struct ip_metadata *meta);






static error_t TcpP__Tcp__bind(uint8_t client, uint16_t port);
#line 128
static error_t TcpP__Tcp__send(uint8_t client, void *payload, uint16_t len);




static error_t TcpP__Tcp__close(uint8_t client);










static inline bool TcpP__Tcp__default__accept(uint8_t cid, struct sockaddr_in6 *from, 
void **tx_buf, int *tx_buf_len);




static inline void TcpP__Tcp__default__recv(uint8_t cid, void *payload, uint16_t len);
static inline void TcpP__Tcp__default__closed(uint8_t cid, error_t e);
static inline void TcpP__Tcp__default__acked(uint8_t cid);
# 32 "/home/tinyos/local/src/tinyos-2.x/tos/lib/net/blip/interfaces/Tcp.nc"
static error_t HttpdP__Tcp__send(void *payload, uint16_t len);
#line 9
static error_t HttpdP__Tcp__bind(uint16_t port);
#line 39
static error_t HttpdP__Tcp__close(void );
# 53 "/home/tinyos/local/src/tinyos-2.x/tos/lib/timer/Timer.nc"
static void HttpdP__TimerTemp__startPeriodic(uint32_t dt);
# 56 "/home/tinyos/local/src/tinyos-2.x/tos/interfaces/Leds.nc"
static void HttpdP__Leds__led0Toggle(void );
#line 72
static void HttpdP__Leds__led1Toggle(void );
#line 89
static void HttpdP__Leds__led2Toggle(void );
#line 106
static uint8_t HttpdP__Leds__get(void );
# 55 "/home/tinyos/local/src/tinyos-2.x/tos/interfaces/Read.nc"
static error_t HttpdP__ReadTemp__read(void );
# 12 "HttpdP.nc"
static char *HttpdP__http_okay = "HTTP/1.0 200 OK\r\n\r\n";
static int HttpdP__http_okay_len = 19;

uint16_t HttpdP__temp = 0;

enum HttpdP____nesc_unnamed4430 {
  HttpdP__S_IDLE, 
  HttpdP__S_CONNECTED, 
  HttpdP__S_REQUEST_PRE, 
  HttpdP__S_REQUEST, 
  HttpdP__S_HEADER, 
  HttpdP__S_BODY
};

enum HttpdP____nesc_unnamed4431 {
  HttpdP__HTTP_GET, 
  HttpdP__HTTP_POST
};

static inline void HttpdP__process_request(int verb, char *request, int len);
#line 85
int HttpdP__http_state;
int HttpdP__req_verb;
char HttpdP__request_buf[150];
#line 87
char *HttpdP__request;
char HttpdP__tcp_buf[100];

static inline void HttpdP__Boot__booted(void );





static inline void HttpdP__TimerTemp__fired(void );



static inline void HttpdP__ReadTemp__readDone(error_t result, uint16_t data);



static inline bool HttpdP__Tcp__accept(struct sockaddr_in6 *from, 
void **tx_buf, int *tx_buf_len);
#line 118
static inline void HttpdP__Tcp__recv(void *payload, uint16_t len);
#line 181
static inline void HttpdP__Tcp__closed(error_t e);






static inline void HttpdP__Tcp__acked(void );
# 76 "/home/tinyos/local/src/tinyos-2.x/tos/chips/sht11/SensirionSht11.nc"
static error_t /*TCPEchoC.Sht11C.SensirionSht11ReaderP*/SensirionSht11ReaderP__0__Sht11Hum__measureHumidity(void );
# 63 "/home/tinyos/local/src/tinyos-2.x/tos/interfaces/Read.nc"
static void /*TCPEchoC.Sht11C.SensirionSht11ReaderP*/SensirionSht11ReaderP__0__Humidity__readDone(error_t result, /*TCPEchoC.Sht11C.SensirionSht11ReaderP*/SensirionSht11ReaderP__0__Humidity__val_t val);
# 110 "/home/tinyos/local/src/tinyos-2.x/tos/interfaces/Resource.nc"
static error_t /*TCPEchoC.Sht11C.SensirionSht11ReaderP*/SensirionSht11ReaderP__0__TempResource__release(void );
#line 78
static error_t /*TCPEchoC.Sht11C.SensirionSht11ReaderP*/SensirionSht11ReaderP__0__TempResource__request(void );
# 61 "/home/tinyos/local/src/tinyos-2.x/tos/chips/sht11/SensirionSht11.nc"
static error_t /*TCPEchoC.Sht11C.SensirionSht11ReaderP*/SensirionSht11ReaderP__0__Sht11Temp__measureTemperature(void );
# 110 "/home/tinyos/local/src/tinyos-2.x/tos/interfaces/Resource.nc"
static error_t /*TCPEchoC.Sht11C.SensirionSht11ReaderP*/SensirionSht11ReaderP__0__HumResource__release(void );
# 63 "/home/tinyos/local/src/tinyos-2.x/tos/interfaces/Read.nc"
static void /*TCPEchoC.Sht11C.SensirionSht11ReaderP*/SensirionSht11ReaderP__0__Temperature__readDone(error_t result, /*TCPEchoC.Sht11C.SensirionSht11ReaderP*/SensirionSht11ReaderP__0__Temperature__val_t val);
# 60 "/home/tinyos/local/src/tinyos-2.x/tos/chips/sht11/SensirionSht11ReaderP.nc"
static inline error_t /*TCPEchoC.Sht11C.SensirionSht11ReaderP*/SensirionSht11ReaderP__0__Temperature__read(void );




static inline void /*TCPEchoC.Sht11C.SensirionSht11ReaderP*/SensirionSht11ReaderP__0__TempResource__granted(void );







static void /*TCPEchoC.Sht11C.SensirionSht11ReaderP*/SensirionSht11ReaderP__0__Sht11Temp__measureTemperatureDone(error_t result, uint16_t val);
#line 85
static inline void /*TCPEchoC.Sht11C.SensirionSht11ReaderP*/SensirionSht11ReaderP__0__HumResource__granted(void );







static inline void /*TCPEchoC.Sht11C.SensirionSht11ReaderP*/SensirionSht11ReaderP__0__Sht11Hum__measureHumidityDone(error_t result, uint16_t val);




static inline void /*TCPEchoC.Sht11C.SensirionSht11ReaderP*/SensirionSht11ReaderP__0__Sht11Temp__resetDone(error_t result);
static inline void /*TCPEchoC.Sht11C.SensirionSht11ReaderP*/SensirionSht11ReaderP__0__Sht11Temp__measureHumidityDone(error_t result, uint16_t val);
static inline void /*TCPEchoC.Sht11C.SensirionSht11ReaderP*/SensirionSht11ReaderP__0__Sht11Temp__readStatusRegDone(error_t result, uint8_t val);
static inline void /*TCPEchoC.Sht11C.SensirionSht11ReaderP*/SensirionSht11ReaderP__0__Sht11Temp__writeStatusRegDone(error_t result);

static inline void /*TCPEchoC.Sht11C.SensirionSht11ReaderP*/SensirionSht11ReaderP__0__Sht11Hum__resetDone(error_t result);
static inline void /*TCPEchoC.Sht11C.SensirionSht11ReaderP*/SensirionSht11ReaderP__0__Sht11Hum__measureTemperatureDone(error_t result, uint16_t val);
static inline void /*TCPEchoC.Sht11C.SensirionSht11ReaderP*/SensirionSht11ReaderP__0__Sht11Hum__readStatusRegDone(error_t result, uint8_t val);
static inline void /*TCPEchoC.Sht11C.SensirionSht11ReaderP*/SensirionSht11ReaderP__0__Sht11Hum__writeStatusRegDone(error_t result);


static inline void /*TCPEchoC.Sht11C.SensirionSht11ReaderP*/SensirionSht11ReaderP__0__Humidity__default__readDone(error_t result, uint16_t val);
# 50 "/home/tinyos/local/src/tinyos-2.x/tos/interfaces/GpioInterrupt.nc"
static error_t /*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP__0__InterruptDATA__disable(void );
#line 43
static error_t /*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP__0__InterruptDATA__enableFallingEdge(void );
# 56 "/home/tinyos/local/src/tinyos-2.x/tos/interfaces/TaskBasic.nc"
static error_t /*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP__0__readSensor__postTask(void );
#line 56
static error_t /*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP__0__signalStatusDone__postTask(void );
# 35 "/home/tinyos/local/src/tinyos-2.x/tos/interfaces/GeneralIO.nc"
static void /*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP__0__CLOCK__makeOutput(void );
#line 29
static void /*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP__0__CLOCK__set(void );
static void /*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP__0__CLOCK__clr(void );
# 84 "/home/tinyos/local/src/tinyos-2.x/tos/chips/sht11/SensirionSht11.nc"
static void /*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP__0__SensirionSht11__measureHumidityDone(
# 54 "/home/tinyos/local/src/tinyos-2.x/tos/chips/sht11/SensirionSht11LogicP.nc"
uint8_t arg_0x4150f970, 
# 84 "/home/tinyos/local/src/tinyos-2.x/tos/chips/sht11/SensirionSht11.nc"
error_t result, uint16_t val);
#line 116
static void /*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP__0__SensirionSht11__writeStatusRegDone(
# 54 "/home/tinyos/local/src/tinyos-2.x/tos/chips/sht11/SensirionSht11LogicP.nc"
uint8_t arg_0x4150f970, 
# 116 "/home/tinyos/local/src/tinyos-2.x/tos/chips/sht11/SensirionSht11.nc"
error_t result);
#line 100
static void /*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP__0__SensirionSht11__readStatusRegDone(
# 54 "/home/tinyos/local/src/tinyos-2.x/tos/chips/sht11/SensirionSht11LogicP.nc"
uint8_t arg_0x4150f970, 
# 100 "/home/tinyos/local/src/tinyos-2.x/tos/chips/sht11/SensirionSht11.nc"
error_t result, uint8_t val);
#line 54
static void /*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP__0__SensirionSht11__resetDone(
# 54 "/home/tinyos/local/src/tinyos-2.x/tos/chips/sht11/SensirionSht11LogicP.nc"
uint8_t arg_0x4150f970, 
# 54 "/home/tinyos/local/src/tinyos-2.x/tos/chips/sht11/SensirionSht11.nc"
error_t result);
#line 69
static void /*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP__0__SensirionSht11__measureTemperatureDone(
# 54 "/home/tinyos/local/src/tinyos-2.x/tos/chips/sht11/SensirionSht11LogicP.nc"
uint8_t arg_0x4150f970, 
# 69 "/home/tinyos/local/src/tinyos-2.x/tos/chips/sht11/SensirionSht11.nc"
error_t result, uint16_t val);
# 33 "/home/tinyos/local/src/tinyos-2.x/tos/interfaces/GeneralIO.nc"
static void /*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP__0__DATA__makeInput(void );
#line 32
static bool /*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP__0__DATA__get(void );


static void /*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP__0__DATA__makeOutput(void );
#line 29
static void /*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP__0__DATA__set(void );
static void /*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP__0__DATA__clr(void );
# 62 "/home/tinyos/local/src/tinyos-2.x/tos/lib/timer/Timer.nc"
static void /*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP__0__Timer__startOneShot(uint32_t dt);




static void /*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP__0__Timer__stop(void );
# 102 "/home/tinyos/local/src/tinyos-2.x/tos/chips/sht11/SensirionSht11LogicP.nc"
enum /*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP__0____nesc_unnamed4432 {
#line 102
  SensirionSht11LogicP__0__readSensor = 14U
};
#line 102
typedef int /*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP__0____nesc_sillytask_readSensor[/*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP__0__readSensor];
enum /*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP__0____nesc_unnamed4433 {
#line 103
  SensirionSht11LogicP__0__signalStatusDone = 15U
};
#line 103
typedef int /*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP__0____nesc_sillytask_signalStatusDone[/*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP__0__signalStatusDone];
#line 72
#line 66
typedef enum /*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP__0____nesc_unnamed4434 {
  SensirionSht11LogicP__0__CMD_MEASURE_TEMPERATURE = 0x3, 
  SensirionSht11LogicP__0__CMD_MEASURE_HUMIDITY = 0x5, 
  SensirionSht11LogicP__0__CMD_READ_STATUS = 0x7, 
  SensirionSht11LogicP__0__CMD_WRITE_STATUS = 0x6, 
  SensirionSht11LogicP__0__CMD_SOFT_RESET = 0x1E
} /*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP__0__sht_cmd_t;

enum /*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP__0____nesc_unnamed4435 {
  SensirionSht11LogicP__0__TIMEOUT_RESET = 11, 
  SensirionSht11LogicP__0__TIMEOUT_14BIT = 250, 
  SensirionSht11LogicP__0__TIMEOUT_12BIT = 250, 
  SensirionSht11LogicP__0__TIMEOUT_8BIT = 250
};

bool /*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP__0__on = TRUE;
bool /*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP__0__busy = FALSE;
uint8_t /*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP__0__status = 0;
/*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP__0__sht_cmd_t /*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP__0__cmd;
uint8_t /*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP__0__newStatus;
bool /*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP__0__writeFail = FALSE;

uint8_t /*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP__0__currentClient;

static error_t /*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP__0__performCommand(void );
static inline void /*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP__0__initPins(void );
static inline void /*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP__0__resetDevice(void );
static inline void /*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP__0__transmissionStart(void );
static inline void /*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP__0__sendCommand(uint8_t _cmd);
static void /*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP__0__writeByte(uint8_t byte);
static error_t /*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP__0__waitForResponse(void );
static void /*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP__0__enableInterrupt(void );
static uint8_t /*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP__0__readByte(void );
static inline void /*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP__0__ack(void );
static void /*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP__0__endTransmission(void );
#line 113
static inline error_t /*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP__0__SensirionSht11__measureTemperature(uint8_t client);







static inline error_t /*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP__0__SensirionSht11__measureHumidity(uint8_t client);
#line 149
static error_t /*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP__0__performCommand(void );
#line 220
static inline void /*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP__0__initPins(void );







static inline void /*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP__0__resetDevice(void );










static inline void /*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP__0__transmissionStart(void );
#line 251
static inline void /*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP__0__sendCommand(uint8_t _cmd);



static void /*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP__0__writeByte(uint8_t byte);
#line 268
static error_t /*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP__0__waitForResponse(void );
#line 281
static void /*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP__0__enableInterrupt(void );





static inline void /*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP__0__Timer__fired(void );
#line 315
static inline void /*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP__0__InterruptDATA__fired(void );




static inline void /*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP__0__readSensor__runTask(void );
#line 355
static uint8_t /*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP__0__readByte(void );
#line 372
static inline void /*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP__0__ack(void );








static void /*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP__0__endTransmission(void );






static inline void /*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP__0__signalStatusDone__runTask(void );
#line 406
static inline void /*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP__0__SensirionSht11__default__resetDone(uint8_t client, error_t result);
static inline void /*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP__0__SensirionSht11__default__measureTemperatureDone(uint8_t client, error_t result, uint16_t val);
static inline void /*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP__0__SensirionSht11__default__measureHumidityDone(uint8_t client, error_t result, uint16_t val);
static inline void /*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP__0__SensirionSht11__default__readStatusRegDone(uint8_t client, error_t result, uint8_t val);
static inline void /*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP__0__SensirionSht11__default__writeStatusRegDone(uint8_t client, error_t result);
# 64 "/home/tinyos/local/src/tinyos-2.x/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
static void /*HplSensirionSht11C.DATAM*/Msp430GpioC__10__HplGeneralIO__makeInput(void );






static void /*HplSensirionSht11C.DATAM*/Msp430GpioC__10__HplGeneralIO__makeOutput(void );
#line 59
static bool /*HplSensirionSht11C.DATAM*/Msp430GpioC__10__HplGeneralIO__get(void );
#line 34
static void /*HplSensirionSht11C.DATAM*/Msp430GpioC__10__HplGeneralIO__set(void );




static void /*HplSensirionSht11C.DATAM*/Msp430GpioC__10__HplGeneralIO__clr(void );
# 37 "/home/tinyos/local/src/tinyos-2.x/tos/chips/msp430/pins/Msp430GpioC.nc"
static inline void /*HplSensirionSht11C.DATAM*/Msp430GpioC__10__GeneralIO__set(void );
static inline void /*HplSensirionSht11C.DATAM*/Msp430GpioC__10__GeneralIO__clr(void );

static inline bool /*HplSensirionSht11C.DATAM*/Msp430GpioC__10__GeneralIO__get(void );
static inline void /*HplSensirionSht11C.DATAM*/Msp430GpioC__10__GeneralIO__makeInput(void );

static inline void /*HplSensirionSht11C.DATAM*/Msp430GpioC__10__GeneralIO__makeOutput(void );
# 64 "/home/tinyos/local/src/tinyos-2.x/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
static void /*HplSensirionSht11C.SCKM*/Msp430GpioC__11__HplGeneralIO__makeInput(void );






static void /*HplSensirionSht11C.SCKM*/Msp430GpioC__11__HplGeneralIO__makeOutput(void );
#line 34
static void /*HplSensirionSht11C.SCKM*/Msp430GpioC__11__HplGeneralIO__set(void );




static void /*HplSensirionSht11C.SCKM*/Msp430GpioC__11__HplGeneralIO__clr(void );
# 37 "/home/tinyos/local/src/tinyos-2.x/tos/chips/msp430/pins/Msp430GpioC.nc"
static inline void /*HplSensirionSht11C.SCKM*/Msp430GpioC__11__GeneralIO__set(void );
static inline void /*HplSensirionSht11C.SCKM*/Msp430GpioC__11__GeneralIO__clr(void );


static inline void /*HplSensirionSht11C.SCKM*/Msp430GpioC__11__GeneralIO__makeInput(void );

static inline void /*HplSensirionSht11C.SCKM*/Msp430GpioC__11__GeneralIO__makeOutput(void );
# 71 "/home/tinyos/local/src/tinyos-2.x/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
static void /*HplSensirionSht11C.PWRM*/Msp430GpioC__12__HplGeneralIO__makeOutput(void );
#line 34
static void /*HplSensirionSht11C.PWRM*/Msp430GpioC__12__HplGeneralIO__set(void );




static void /*HplSensirionSht11C.PWRM*/Msp430GpioC__12__HplGeneralIO__clr(void );
# 37 "/home/tinyos/local/src/tinyos-2.x/tos/chips/msp430/pins/Msp430GpioC.nc"
static inline void /*HplSensirionSht11C.PWRM*/Msp430GpioC__12__GeneralIO__set(void );
static inline void /*HplSensirionSht11C.PWRM*/Msp430GpioC__12__GeneralIO__clr(void );




static inline void /*HplSensirionSht11C.PWRM*/Msp430GpioC__12__GeneralIO__makeOutput(void );
# 92 "/home/tinyos/local/src/tinyos-2.x/tos/interfaces/SplitControl.nc"
static void HplSensirionSht11P__SplitControl__startDone(error_t error);
#line 117
static void HplSensirionSht11P__SplitControl__stopDone(error_t error);
# 56 "/home/tinyos/local/src/tinyos-2.x/tos/interfaces/TaskBasic.nc"
static error_t HplSensirionSht11P__stopTask__postTask(void );
# 33 "/home/tinyos/local/src/tinyos-2.x/tos/interfaces/GeneralIO.nc"
static void HplSensirionSht11P__SCK__makeInput(void );
#line 30
static void HplSensirionSht11P__SCK__clr(void );




static void HplSensirionSht11P__PWR__makeOutput(void );
#line 29
static void HplSensirionSht11P__PWR__set(void );
static void HplSensirionSht11P__PWR__clr(void );


static void HplSensirionSht11P__DATA__makeInput(void );
#line 30
static void HplSensirionSht11P__DATA__clr(void );
# 62 "/home/tinyos/local/src/tinyos-2.x/tos/lib/timer/Timer.nc"
static void HplSensirionSht11P__Timer__startOneShot(uint32_t dt);
# 50 "/home/tinyos/local/src/tinyos-2.x/tos/platforms/telosa/chips/sht11/HplSensirionSht11P.nc"
enum HplSensirionSht11P____nesc_unnamed4436 {
#line 50
  HplSensirionSht11P__stopTask = 16U
};
#line 50
typedef int HplSensirionSht11P____nesc_sillytask_stopTask[HplSensirionSht11P__stopTask];

static error_t HplSensirionSht11P__SplitControl__start(void );






static inline void HplSensirionSht11P__Timer__fired(void );



static inline error_t HplSensirionSht11P__SplitControl__stop(void );









static inline void HplSensirionSht11P__stopTask__runTask(void );
# 41 "/home/tinyos/local/src/tinyos-2.x/tos/chips/msp430/pins/HplMsp430Interrupt.nc"
static void /*HplSensirionSht11C.InterruptDATAC*/Msp430InterruptC__2__HplInterrupt__clear(void );
#line 36
static void /*HplSensirionSht11C.InterruptDATAC*/Msp430InterruptC__2__HplInterrupt__disable(void );
#line 56
static void /*HplSensirionSht11C.InterruptDATAC*/Msp430InterruptC__2__HplInterrupt__edge(bool low_to_high);
#line 31
static void /*HplSensirionSht11C.InterruptDATAC*/Msp430InterruptC__2__HplInterrupt__enable(void );
# 57 "/home/tinyos/local/src/tinyos-2.x/tos/interfaces/GpioInterrupt.nc"
static void /*HplSensirionSht11C.InterruptDATAC*/Msp430InterruptC__2__Interrupt__fired(void );
# 41 "/home/tinyos/local/src/tinyos-2.x/tos/chips/msp430/pins/Msp430InterruptC.nc"
static inline error_t /*HplSensirionSht11C.InterruptDATAC*/Msp430InterruptC__2__enable(bool rising);
#line 54
static inline error_t /*HplSensirionSht11C.InterruptDATAC*/Msp430InterruptC__2__Interrupt__enableFallingEdge(void );



static error_t /*HplSensirionSht11C.InterruptDATAC*/Msp430InterruptC__2__Interrupt__disable(void );







static inline void /*HplSensirionSht11C.InterruptDATAC*/Msp430InterruptC__2__HplInterrupt__fired(void );
# 39 "/home/tinyos/local/src/tinyos-2.x/tos/system/FcfsResourceQueueC.nc"
enum /*HplSensirionSht11C.Arbiter.Queue*/FcfsResourceQueueC__2____nesc_unnamed4437 {
#line 39
  FcfsResourceQueueC__2__NO_ENTRY = 0xFF
};
uint8_t /*HplSensirionSht11C.Arbiter.Queue*/FcfsResourceQueueC__2__resQ[2U];
uint8_t /*HplSensirionSht11C.Arbiter.Queue*/FcfsResourceQueueC__2__qHead = /*HplSensirionSht11C.Arbiter.Queue*/FcfsResourceQueueC__2__NO_ENTRY;
uint8_t /*HplSensirionSht11C.Arbiter.Queue*/FcfsResourceQueueC__2__qTail = /*HplSensirionSht11C.Arbiter.Queue*/FcfsResourceQueueC__2__NO_ENTRY;

static inline error_t /*HplSensirionSht11C.Arbiter.Queue*/FcfsResourceQueueC__2__Init__init(void );




static inline bool /*HplSensirionSht11C.Arbiter.Queue*/FcfsResourceQueueC__2__FcfsQueue__isEmpty(void );



static inline bool /*HplSensirionSht11C.Arbiter.Queue*/FcfsResourceQueueC__2__FcfsQueue__isEnqueued(resource_client_id_t id);



static inline resource_client_id_t /*HplSensirionSht11C.Arbiter.Queue*/FcfsResourceQueueC__2__FcfsQueue__dequeue(void );
#line 72
static inline error_t /*HplSensirionSht11C.Arbiter.Queue*/FcfsResourceQueueC__2__FcfsQueue__enqueue(resource_client_id_t id);
# 43 "/home/tinyos/local/src/tinyos-2.x/tos/interfaces/ResourceRequested.nc"
static void /*HplSensirionSht11C.Arbiter.Arbiter*/ArbiterP__1__ResourceRequested__requested(
# 55 "/home/tinyos/local/src/tinyos-2.x/tos/system/ArbiterP.nc"
uint8_t arg_0x40e33d38);
# 55 "/home/tinyos/local/src/tinyos-2.x/tos/interfaces/ResourceConfigure.nc"
static void /*HplSensirionSht11C.Arbiter.Arbiter*/ArbiterP__1__ResourceConfigure__unconfigure(
# 60 "/home/tinyos/local/src/tinyos-2.x/tos/system/ArbiterP.nc"
uint8_t arg_0x40e31148);
# 49 "/home/tinyos/local/src/tinyos-2.x/tos/interfaces/ResourceConfigure.nc"
static void /*HplSensirionSht11C.Arbiter.Arbiter*/ArbiterP__1__ResourceConfigure__configure(
# 60 "/home/tinyos/local/src/tinyos-2.x/tos/system/ArbiterP.nc"
uint8_t arg_0x40e31148);
# 69 "/home/tinyos/local/src/tinyos-2.x/tos/interfaces/ResourceQueue.nc"
static error_t /*HplSensirionSht11C.Arbiter.Arbiter*/ArbiterP__1__Queue__enqueue(resource_client_id_t id);
#line 43
static bool /*HplSensirionSht11C.Arbiter.Arbiter*/ArbiterP__1__Queue__isEmpty(void );
#line 60
static resource_client_id_t /*HplSensirionSht11C.Arbiter.Arbiter*/ArbiterP__1__Queue__dequeue(void );
# 73 "/home/tinyos/local/src/tinyos-2.x/tos/interfaces/ResourceDefaultOwner.nc"
static void /*HplSensirionSht11C.Arbiter.Arbiter*/ArbiterP__1__ResourceDefaultOwner__requested(void );
#line 46
static void /*HplSensirionSht11C.Arbiter.Arbiter*/ArbiterP__1__ResourceDefaultOwner__granted(void );
# 92 "/home/tinyos/local/src/tinyos-2.x/tos/interfaces/Resource.nc"
static void /*HplSensirionSht11C.Arbiter.Arbiter*/ArbiterP__1__Resource__granted(
# 54 "/home/tinyos/local/src/tinyos-2.x/tos/system/ArbiterP.nc"
uint8_t arg_0x40e33310);
# 56 "/home/tinyos/local/src/tinyos-2.x/tos/interfaces/TaskBasic.nc"
static error_t /*HplSensirionSht11C.Arbiter.Arbiter*/ArbiterP__1__grantedTask__postTask(void );
# 75 "/home/tinyos/local/src/tinyos-2.x/tos/system/ArbiterP.nc"
enum /*HplSensirionSht11C.Arbiter.Arbiter*/ArbiterP__1____nesc_unnamed4438 {
#line 75
  ArbiterP__1__grantedTask = 17U
};
#line 75
typedef int /*HplSensirionSht11C.Arbiter.Arbiter*/ArbiterP__1____nesc_sillytask_grantedTask[/*HplSensirionSht11C.Arbiter.Arbiter*/ArbiterP__1__grantedTask];
#line 67
enum /*HplSensirionSht11C.Arbiter.Arbiter*/ArbiterP__1____nesc_unnamed4439 {
#line 67
  ArbiterP__1__RES_CONTROLLED, ArbiterP__1__RES_GRANTING, ArbiterP__1__RES_IMM_GRANTING, ArbiterP__1__RES_BUSY
};
#line 68
enum /*HplSensirionSht11C.Arbiter.Arbiter*/ArbiterP__1____nesc_unnamed4440 {
#line 68
  ArbiterP__1__default_owner_id = 2U
};
#line 69
enum /*HplSensirionSht11C.Arbiter.Arbiter*/ArbiterP__1____nesc_unnamed4441 {
#line 69
  ArbiterP__1__NO_RES = 0xFF
};
uint8_t /*HplSensirionSht11C.Arbiter.Arbiter*/ArbiterP__1__state = /*HplSensirionSht11C.Arbiter.Arbiter*/ArbiterP__1__RES_CONTROLLED;
uint8_t /*HplSensirionSht11C.Arbiter.Arbiter*/ArbiterP__1__resId = /*HplSensirionSht11C.Arbiter.Arbiter*/ArbiterP__1__default_owner_id;
uint8_t /*HplSensirionSht11C.Arbiter.Arbiter*/ArbiterP__1__reqResId;



static inline error_t /*HplSensirionSht11C.Arbiter.Arbiter*/ArbiterP__1__Resource__request(uint8_t id);
#line 108
static error_t /*HplSensirionSht11C.Arbiter.Arbiter*/ArbiterP__1__Resource__release(uint8_t id);
#line 130
static inline error_t /*HplSensirionSht11C.Arbiter.Arbiter*/ArbiterP__1__ResourceDefaultOwner__release(void );
#line 181
static inline uint8_t /*HplSensirionSht11C.Arbiter.Arbiter*/ArbiterP__1__ResourceDefaultOwner__isOwner(void );





static inline void /*HplSensirionSht11C.Arbiter.Arbiter*/ArbiterP__1__grantedTask__runTask(void );
#line 199
static inline void /*HplSensirionSht11C.Arbiter.Arbiter*/ArbiterP__1__Resource__default__granted(uint8_t id);

static inline void /*HplSensirionSht11C.Arbiter.Arbiter*/ArbiterP__1__ResourceRequested__default__requested(uint8_t id);
#line 213
static inline void /*HplSensirionSht11C.Arbiter.Arbiter*/ArbiterP__1__ResourceConfigure__default__configure(uint8_t id);

static inline void /*HplSensirionSht11C.Arbiter.Arbiter*/ArbiterP__1__ResourceConfigure__default__unconfigure(uint8_t id);
# 83 "/home/tinyos/local/src/tinyos-2.x/tos/interfaces/SplitControl.nc"
static error_t /*HplSensirionSht11C.SplitControlPowerManagerC.PowerManager*/PowerManagerP__0__SplitControl__start(void );
#line 109
static error_t /*HplSensirionSht11C.SplitControlPowerManagerC.PowerManager*/PowerManagerP__0__SplitControl__stop(void );
# 52 "/home/tinyos/local/src/tinyos-2.x/tos/lib/power/PowerDownCleanup.nc"
static void /*HplSensirionSht11C.SplitControlPowerManagerC.PowerManager*/PowerManagerP__0__PowerDownCleanup__cleanup(void );
# 56 "/home/tinyos/local/src/tinyos-2.x/tos/interfaces/TaskBasic.nc"
static error_t /*HplSensirionSht11C.SplitControlPowerManagerC.PowerManager*/PowerManagerP__0__stopTask__postTask(void );
# 56 "/home/tinyos/local/src/tinyos-2.x/tos/interfaces/ResourceDefaultOwner.nc"
static error_t /*HplSensirionSht11C.SplitControlPowerManagerC.PowerManager*/PowerManagerP__0__ResourceDefaultOwner__release(void );








static bool /*HplSensirionSht11C.SplitControlPowerManagerC.PowerManager*/PowerManagerP__0__ResourceDefaultOwner__isOwner(void );
# 56 "/home/tinyos/local/src/tinyos-2.x/tos/interfaces/TaskBasic.nc"
static error_t /*HplSensirionSht11C.SplitControlPowerManagerC.PowerManager*/PowerManagerP__0__startTask__postTask(void );
# 74 "/home/tinyos/local/src/tinyos-2.x/tos/interfaces/StdControl.nc"
static error_t /*HplSensirionSht11C.SplitControlPowerManagerC.PowerManager*/PowerManagerP__0__StdControl__start(void );









static error_t /*HplSensirionSht11C.SplitControlPowerManagerC.PowerManager*/PowerManagerP__0__StdControl__stop(void );
# 63 "/home/tinyos/local/src/tinyos-2.x/tos/lib/power/PowerManagerP.nc"
enum /*HplSensirionSht11C.SplitControlPowerManagerC.PowerManager*/PowerManagerP__0____nesc_unnamed4442 {
#line 63
  PowerManagerP__0__startTask = 18U
};
#line 63
typedef int /*HplSensirionSht11C.SplitControlPowerManagerC.PowerManager*/PowerManagerP__0____nesc_sillytask_startTask[/*HplSensirionSht11C.SplitControlPowerManagerC.PowerManager*/PowerManagerP__0__startTask];




enum /*HplSensirionSht11C.SplitControlPowerManagerC.PowerManager*/PowerManagerP__0____nesc_unnamed4443 {
#line 68
  PowerManagerP__0__stopTask = 19U
};
#line 68
typedef int /*HplSensirionSht11C.SplitControlPowerManagerC.PowerManager*/PowerManagerP__0____nesc_sillytask_stopTask[/*HplSensirionSht11C.SplitControlPowerManagerC.PowerManager*/PowerManagerP__0__stopTask];
#line 60
bool /*HplSensirionSht11C.SplitControlPowerManagerC.PowerManager*/PowerManagerP__0__stopping = FALSE;
bool /*HplSensirionSht11C.SplitControlPowerManagerC.PowerManager*/PowerManagerP__0__requested = FALSE;

static inline void /*HplSensirionSht11C.SplitControlPowerManagerC.PowerManager*/PowerManagerP__0__startTask__runTask(void );




static inline void /*HplSensirionSht11C.SplitControlPowerManagerC.PowerManager*/PowerManagerP__0__stopTask__runTask(void );





static inline void /*HplSensirionSht11C.SplitControlPowerManagerC.PowerManager*/PowerManagerP__0__ResourceDefaultOwner__requested(void );









static inline error_t /*HplSensirionSht11C.SplitControlPowerManagerC.PowerManager*/PowerManagerP__0__StdControl__default__start(void );







static inline void /*HplSensirionSht11C.SplitControlPowerManagerC.PowerManager*/PowerManagerP__0__SplitControl__startDone(error_t error);




static inline void /*HplSensirionSht11C.SplitControlPowerManagerC.PowerManager*/PowerManagerP__0__ResourceDefaultOwner__granted(void );




static inline void /*HplSensirionSht11C.SplitControlPowerManagerC.PowerManager*/PowerManagerP__0__SplitControl__stopDone(error_t error);










static inline error_t /*HplSensirionSht11C.SplitControlPowerManagerC.PowerManager*/PowerManagerP__0__StdControl__default__stop(void );







static inline void /*HplSensirionSht11C.SplitControlPowerManagerC.PowerManager*/PowerManagerP__0__PowerDownCleanup__default__cleanup(void );
# 53 "/home/tinyos/local/src/tinyos-2.x/tos/lib/timer/Counter.nc"
static UDPShellP__Uptime__size_type UDPShellP__Uptime__get(void );
# 16 "/home/tinyos/local/src/tinyos-2.x/tos/lib/net/blip/interfaces/UDP.nc"
static error_t UDPShellP__UDP__sendto(struct sockaddr_in6 *dest, void *payload, 
uint16_t len);
#line 10
static error_t UDPShellP__UDP__bind(uint16_t port);
# 11 "/home/tinyos/local/src/tinyos-2.x/tos/lib/net/blip/shell/ShellCommand.nc"
static char *UDPShellP__ShellCommand__eval(
# 30 "/home/tinyos/local/src/tinyos-2.x/tos/lib/net/blip/shell/UDPShellP.nc"
uint8_t arg_0x415a4698, 
# 11 "/home/tinyos/local/src/tinyos-2.x/tos/lib/net/blip/shell/ShellCommand.nc"
int argc, char **argv);
# 3 "/home/tinyos/local/src/tinyos-2.x/tos/lib/net/blip/shell/RegisterShellCommand.nc"
static char *UDPShellP__RegisterShellCommand__getCommandName(
# 31 "/home/tinyos/local/src/tinyos-2.x/tos/lib/net/blip/shell/UDPShellP.nc"
uint8_t arg_0x415a3650);
# 6 "/home/tinyos/local/src/tinyos-2.x/tos/lib/net/blip/interfaces/ICMPPing.nc"
static error_t UDPShellP__ICMPPing__ping(struct in6_addr *target, uint16_t period, uint16_t n);
# 48 "/home/tinyos/local/src/tinyos-2.x/tos/lib/net/blip/shell/UDPShellP.nc"
struct sockaddr_in6 UDPShellP__session_endpoint;
uint32_t UDPShellP__boot_time;
uint64_t UDPShellP__uptime;

enum UDPShellP____nesc_unnamed4444 {
  UDPShellP__N_EXTERNAL = 0U
};


enum UDPShellP____nesc_unnamed4445 {
  UDPShellP__N_BUILTINS = 5, 

  UDPShellP__N_ARGS = 10, 
  UDPShellP__CMD_HELP = 0, 
  UDPShellP__CMD_ECHO = 1, 
  UDPShellP__CMD_PING6 = 2, 
  UDPShellP__CMD_TRACERT6 = 3, 

  UDPShellP__CMD_NO_CMD = 0xfe, 
  UDPShellP__CMDNAMSIZ = 10
};

struct UDPShellP__cmd_name {
  uint8_t c_len;
  char c_name[UDPShellP__CMDNAMSIZ];
};
struct UDPShellP__cmd_builtin {
  void (*action)(int arg_0x4159e4c0, char **arg_0x4159e658);
};

struct UDPShellP__cmd_name UDPShellP__externals[UDPShellP__N_EXTERNAL];


static inline void UDPShellP__Boot__booted(void );
#line 100
char UDPShellP__reply_buf[MAX_REPLY_LEN];
char *UDPShellP__help_str = "sdsh-0.9\tbuiltins: [help, echo, ping6, uptime, ident]\n";
const char *UDPShellP__ping_fmt = " icmp_seq=%i ttl=%i time=%i ms\n";
const char *UDPShellP__ping_summary = "%i packets transmitted, %i received\n";
char *UDPShellP__ident_string = "\t[app: "
"TCPEchoC""]\n\t[user: ""tinyos""]\n\t[host: ""lenny"
"]\n\t[time: ""0x4d9bbac5L""]\n";


static inline void UDPShellP__action_help(int argc, char **argv);
#line 149
static void UDPShellP__action_echo(int argc, char **argv);
#line 167
static void UDPShellP__action_ping6(int argc, char **argv);








static void UDPShellP__action_uptime(int argc, char **argv);
#line 188
static inline void UDPShellP__action_ident(int argc, char **argv);




struct UDPShellP__cmd_name UDPShellP__builtins[UDPShellP__N_BUILTINS] = { { 4, "help" }, 
{ 4, "echo" }, 
{ 5, "ping6" }, 
{ 6, "uptime" }, 
{ 5, "ident" } };
struct UDPShellP__cmd_builtin UDPShellP__builtin_actions[UDPShellP__N_BUILTINS] = { { UDPShellP__action_help }, 
{ UDPShellP__action_echo }, 
{ UDPShellP__action_ping6 }, 
{ UDPShellP__action_uptime }, 
{ UDPShellP__action_ident } };




static inline void UDPShellP__init_argv(char *cmd, uint16_t len, char **argv, int *argc);
#line 226
static int UDPShellP__lookup_cmd(char *cmd, int dbsize, struct UDPShellP__cmd_name *db);









static inline void UDPShellP__UDP__recvfrom(struct sockaddr_in6 *from, void *data, 
uint16_t len, struct ip_metadata *meta);
#line 262
static inline void UDPShellP__ICMPPing__pingReply(struct in6_addr *source, struct icmp_stats *stats);









static inline void UDPShellP__ICMPPing__pingDone(uint16_t ping_rcv, uint16_t ping_n);






static inline void UDPShellP__Uptime__overflow(void );





static inline char *UDPShellP__ShellCommand__default__eval(uint8_t cmd_id, int argc, char **argv);


static inline char *UDPShellP__RegisterShellCommand__default__getCommandName(uint8_t cmd_id);
# 212 "/home/tinyos/local/src/tinyos-2.x/tos/chips/msp430/msp430hardware.h"
static inline  void __nesc_enable_interrupt(void )
{
   __asm volatile ("eint");}

# 185 "/home/tinyos/local/src/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static inline void /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP__2__Timer__overflow(void )
{
}

#line 185
static inline void /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP__1__Timer__overflow(void )
{
}

#line 185
static inline void /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0__Timer__overflow(void )
{
}

# 37 "/home/tinyos/local/src/tinyos-2.x/tos/chips/msp430/timer/Msp430Timer.nc"
inline static void /*Msp430TimerC.Msp430TimerA*/Msp430TimerP__0__Timer__overflow(void ){
#line 37
  /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0__Timer__overflow();
#line 37
  /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP__1__Timer__overflow();
#line 37
  /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP__2__Timer__overflow();
#line 37
}
#line 37
# 126 "/home/tinyos/local/src/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerP.nc"
static inline void /*Msp430TimerC.Msp430TimerA*/Msp430TimerP__0__Overflow__fired(void )
{
  /*Msp430TimerC.Msp430TimerA*/Msp430TimerP__0__Timer__overflow();
}





static inline void /*Msp430TimerC.Msp430TimerA*/Msp430TimerP__0__Event__default__fired(uint8_t n)
{
}

# 28 "/home/tinyos/local/src/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerEvent.nc"
inline static void /*Msp430TimerC.Msp430TimerA*/Msp430TimerP__0__Event__fired(uint8_t arg_0x406403c8){
#line 28
  switch (arg_0x406403c8) {
#line 28
    case 0:
#line 28
      /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0__Event__fired();
#line 28
      break;
#line 28
    case 1:
#line 28
      /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP__1__Event__fired();
#line 28
      break;
#line 28
    case 2:
#line 28
      /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP__2__Event__fired();
#line 28
      break;
#line 28
    case 5:
#line 28
      /*Msp430TimerC.Msp430TimerA*/Msp430TimerP__0__Overflow__fired();
#line 28
      break;
#line 28
    default:
#line 28
      /*Msp430TimerC.Msp430TimerA*/Msp430TimerP__0__Event__default__fired(arg_0x406403c8);
#line 28
      break;
#line 28
    }
#line 28
}
#line 28
# 115 "/home/tinyos/local/src/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerP.nc"
static inline void /*Msp430TimerC.Msp430TimerA*/Msp430TimerP__0__VectorTimerX0__fired(void )
{
  /*Msp430TimerC.Msp430TimerA*/Msp430TimerP__0__Event__fired(0);
}

# 28 "/home/tinyos/local/src/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerEvent.nc"
inline static void Msp430TimerCommonP__VectorTimerA0__fired(void ){
#line 28
  /*Msp430TimerC.Msp430TimerA*/Msp430TimerP__0__VectorTimerX0__fired();
#line 28
}
#line 28
# 47 "/home/tinyos/local/src/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static inline  /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0__cc_t /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0__int2CC(uint16_t x)
#line 47
{
#line 47
  union /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0____nesc_unnamed4446 {
#line 47
    uint16_t f;
#line 47
    /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0__cc_t t;
  } 
#line 47
  c = { .f = x };

#line 47
  return c.t;
}

#line 74
static inline /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0__cc_t /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0__Control__getControl(void )
{
  return /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0__int2CC(* (volatile uint16_t * )354U);
}

#line 177
static inline void /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0__Capture__default__captured(uint16_t n)
{
}

# 75 "/home/tinyos/local/src/tinyos-2.x/tos/chips/msp430/timer/Msp430Capture.nc"
inline static void /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0__Capture__captured(uint16_t time){
#line 75
  /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0__Capture__default__captured(time);
#line 75
}
#line 75
# 139 "/home/tinyos/local/src/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static inline uint16_t /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0__Capture__getEvent(void )
{
  return * (volatile uint16_t * )370U;
}

#line 181
static inline void /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0__Compare__default__fired(void )
{
}

# 34 "/home/tinyos/local/src/tinyos-2.x/tos/chips/msp430/timer/Msp430Compare.nc"
inline static void /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0__Compare__fired(void ){
#line 34
  /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0__Compare__default__fired();
#line 34
}
#line 34
# 47 "/home/tinyos/local/src/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static inline  /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP__1__cc_t /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP__1__int2CC(uint16_t x)
#line 47
{
#line 47
  union /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP__1____nesc_unnamed4447 {
#line 47
    uint16_t f;
#line 47
    /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP__1__cc_t t;
  } 
#line 47
  c = { .f = x };

#line 47
  return c.t;
}

#line 74
static inline /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP__1__cc_t /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP__1__Control__getControl(void )
{
  return /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP__1__int2CC(* (volatile uint16_t * )356U);
}

#line 177
static inline void /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP__1__Capture__default__captured(uint16_t n)
{
}

# 75 "/home/tinyos/local/src/tinyos-2.x/tos/chips/msp430/timer/Msp430Capture.nc"
inline static void /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP__1__Capture__captured(uint16_t time){
#line 75
  /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP__1__Capture__default__captured(time);
#line 75
}
#line 75
# 139 "/home/tinyos/local/src/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static inline uint16_t /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP__1__Capture__getEvent(void )
{
  return * (volatile uint16_t * )372U;
}

#line 181
static inline void /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP__1__Compare__default__fired(void )
{
}

# 34 "/home/tinyos/local/src/tinyos-2.x/tos/chips/msp430/timer/Msp430Compare.nc"
inline static void /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP__1__Compare__fired(void ){
#line 34
  /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP__1__Compare__default__fired();
#line 34
}
#line 34
# 47 "/home/tinyos/local/src/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static inline  /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP__2__cc_t /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP__2__int2CC(uint16_t x)
#line 47
{
#line 47
  union /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP__2____nesc_unnamed4448 {
#line 47
    uint16_t f;
#line 47
    /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP__2__cc_t t;
  } 
#line 47
  c = { .f = x };

#line 47
  return c.t;
}

#line 74
static inline /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP__2__cc_t /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP__2__Control__getControl(void )
{
  return /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP__2__int2CC(* (volatile uint16_t * )358U);
}

#line 177
static inline void /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP__2__Capture__default__captured(uint16_t n)
{
}

# 75 "/home/tinyos/local/src/tinyos-2.x/tos/chips/msp430/timer/Msp430Capture.nc"
inline static void /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP__2__Capture__captured(uint16_t time){
#line 75
  /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP__2__Capture__default__captured(time);
#line 75
}
#line 75
# 139 "/home/tinyos/local/src/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static inline uint16_t /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP__2__Capture__getEvent(void )
{
  return * (volatile uint16_t * )374U;
}

#line 181
static inline void /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP__2__Compare__default__fired(void )
{
}

# 34 "/home/tinyos/local/src/tinyos-2.x/tos/chips/msp430/timer/Msp430Compare.nc"
inline static void /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP__2__Compare__fired(void ){
#line 34
  /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP__2__Compare__default__fired();
#line 34
}
#line 34
# 120 "/home/tinyos/local/src/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerP.nc"
static inline void /*Msp430TimerC.Msp430TimerA*/Msp430TimerP__0__VectorTimerX1__fired(void )
{
  uint8_t n = * (volatile uint16_t * )302U;

#line 123
  /*Msp430TimerC.Msp430TimerA*/Msp430TimerP__0__Event__fired(n >> 1);
}

# 28 "/home/tinyos/local/src/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerEvent.nc"
inline static void Msp430TimerCommonP__VectorTimerA1__fired(void ){
#line 28
  /*Msp430TimerC.Msp430TimerA*/Msp430TimerP__0__VectorTimerX1__fired();
#line 28
}
#line 28
# 115 "/home/tinyos/local/src/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerP.nc"
static inline void /*Msp430TimerC.Msp430TimerB*/Msp430TimerP__1__VectorTimerX0__fired(void )
{
  /*Msp430TimerC.Msp430TimerB*/Msp430TimerP__1__Event__fired(0);
}

# 28 "/home/tinyos/local/src/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerEvent.nc"
inline static void Msp430TimerCommonP__VectorTimerB0__fired(void ){
#line 28
  /*Msp430TimerC.Msp430TimerB*/Msp430TimerP__1__VectorTimerX0__fired();
#line 28
}
#line 28
# 185 "/home/tinyos/local/src/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static inline void /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP__9__Timer__overflow(void )
{
}

#line 185
static inline void /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP__8__Timer__overflow(void )
{
}

#line 185
static inline void /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP__7__Timer__overflow(void )
{
}

#line 185
static inline void /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP__6__Timer__overflow(void )
{
}

#line 185
static inline void /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__Timer__overflow(void )
{
}

#line 185
static inline void /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__Timer__overflow(void )
{
}

#line 185
static inline void /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Timer__overflow(void )
{
}

# 103 "/home/tinyos/local/src/tinyos-2.x/tos/chips/msp430/timer/Msp430AlarmC.nc"
static inline void /*AlarmMultiplexC.Alarm.Alarm32khz32C.AlarmC.Msp430Alarm*/Msp430AlarmC__1__Msp430Timer__overflow(void )
{
}

#line 103
static inline void /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__0__Msp430Timer__overflow(void )
{
}

# 47 "/home/tinyos/local/src/tinyos-2.x/tos/lib/timer/CounterToLocalTimeC.nc"
static inline void /*HilTimerMilliC.CounterToLocalTimeC*/CounterToLocalTimeC__0__Counter__overflow(void )
{
}

# 166 "/home/tinyos/local/src/tinyos-2.x/tos/lib/timer/TransformAlarmC.nc"
static inline void /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__Counter__overflow(void )
{
}

# 279 "/home/tinyos/local/src/tinyos-2.x/tos/lib/net/blip/shell/UDPShellP.nc"
static inline void UDPShellP__Uptime__overflow(void )
#line 279
{
  /* atomic removed: atomic calls only */
  UDPShellP__uptime += 0xffffffff;
}

# 71 "/home/tinyos/local/src/tinyos-2.x/tos/lib/timer/Counter.nc"
inline static void /*CounterMilli32C.Transform*/TransformCounterC__0__Counter__overflow(void ){
#line 71
  UDPShellP__Uptime__overflow();
#line 71
  /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__Counter__overflow();
#line 71
  /*HilTimerMilliC.CounterToLocalTimeC*/CounterToLocalTimeC__0__Counter__overflow();
#line 71
}
#line 71
# 122 "/home/tinyos/local/src/tinyos-2.x/tos/lib/timer/TransformCounterC.nc"
static inline void /*CounterMilli32C.Transform*/TransformCounterC__0__CounterFrom__overflow(void )
{
  /* atomic removed: atomic calls only */
  {
    /*CounterMilli32C.Transform*/TransformCounterC__0__m_upper++;
    if ((/*CounterMilli32C.Transform*/TransformCounterC__0__m_upper & /*CounterMilli32C.Transform*/TransformCounterC__0__OVERFLOW_MASK) == 0) {
      /*CounterMilli32C.Transform*/TransformCounterC__0__Counter__overflow();
      }
  }
}

# 166 "/home/tinyos/local/src/tinyos-2.x/tos/lib/timer/TransformAlarmC.nc"
static inline void /*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC__1__Counter__overflow(void )
{
}

# 47 "/home/tinyos/local/src/tinyos-2.x/tos/lib/timer/CounterToLocalTimeC.nc"
static inline void /*CC2420PacketC.CounterToLocalTimeC*/CounterToLocalTimeC__1__Counter__overflow(void )
{
}

# 71 "/home/tinyos/local/src/tinyos-2.x/tos/lib/timer/Counter.nc"
inline static void /*Counter32khz32C.Transform*/TransformCounterC__1__Counter__overflow(void ){
#line 71
  /*CC2420PacketC.CounterToLocalTimeC*/CounterToLocalTimeC__1__Counter__overflow();
#line 71
  /*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC__1__Counter__overflow();
#line 71
}
#line 71
# 122 "/home/tinyos/local/src/tinyos-2.x/tos/lib/timer/TransformCounterC.nc"
static inline void /*Counter32khz32C.Transform*/TransformCounterC__1__CounterFrom__overflow(void )
{
  /* atomic removed: atomic calls only */
  {
    /*Counter32khz32C.Transform*/TransformCounterC__1__m_upper++;
    if ((/*Counter32khz32C.Transform*/TransformCounterC__1__m_upper & /*Counter32khz32C.Transform*/TransformCounterC__1__OVERFLOW_MASK) == 0) {
      /*Counter32khz32C.Transform*/TransformCounterC__1__Counter__overflow();
      }
  }
}

# 71 "/home/tinyos/local/src/tinyos-2.x/tos/lib/timer/Counter.nc"
inline static void /*Msp430Counter32khzC.Counter*/Msp430CounterC__0__Counter__overflow(void ){
#line 71
  /*Counter32khz32C.Transform*/TransformCounterC__1__CounterFrom__overflow();
#line 71
  /*CounterMilli32C.Transform*/TransformCounterC__0__CounterFrom__overflow();
#line 71
}
#line 71
# 53 "/home/tinyos/local/src/tinyos-2.x/tos/chips/msp430/timer/Msp430CounterC.nc"
static inline void /*Msp430Counter32khzC.Counter*/Msp430CounterC__0__Msp430Timer__overflow(void )
{
  /*Msp430Counter32khzC.Counter*/Msp430CounterC__0__Counter__overflow();
}

# 37 "/home/tinyos/local/src/tinyos-2.x/tos/chips/msp430/timer/Msp430Timer.nc"
inline static void /*Msp430TimerC.Msp430TimerB*/Msp430TimerP__1__Timer__overflow(void ){
#line 37
  /*Msp430Counter32khzC.Counter*/Msp430CounterC__0__Msp430Timer__overflow();
#line 37
  /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__0__Msp430Timer__overflow();
#line 37
  /*AlarmMultiplexC.Alarm.Alarm32khz32C.AlarmC.Msp430Alarm*/Msp430AlarmC__1__Msp430Timer__overflow();
#line 37
  /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Timer__overflow();
#line 37
  /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__Timer__overflow();
#line 37
  /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__Timer__overflow();
#line 37
  /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP__6__Timer__overflow();
#line 37
  /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP__7__Timer__overflow();
#line 37
  /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP__8__Timer__overflow();
#line 37
  /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP__9__Timer__overflow();
#line 37
}
#line 37
# 126 "/home/tinyos/local/src/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerP.nc"
static inline void /*Msp430TimerC.Msp430TimerB*/Msp430TimerP__1__Overflow__fired(void )
{
  /*Msp430TimerC.Msp430TimerB*/Msp430TimerP__1__Timer__overflow();
}

# 56 "/home/tinyos/local/src/tinyos-2.x/tos/interfaces/TaskBasic.nc"
inline static error_t /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__fired__postTask(void ){
#line 56
  unsigned char result;
#line 56

#line 56
  result = SchedulerBasicP__TaskBasic__postTask(/*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__fired);
#line 56

#line 56
  return result;
#line 56
}
#line 56
# 70 "/home/tinyos/local/src/tinyos-2.x/tos/lib/timer/AlarmToTimerC.nc"
static inline void /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__Alarm__fired(void )
{
#line 71
  /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__fired__postTask();
}

# 67 "/home/tinyos/local/src/tinyos-2.x/tos/lib/timer/Alarm.nc"
inline static void /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__Alarm__fired(void ){
#line 67
  /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__Alarm__fired();
#line 67
}
#line 67
# 151 "/home/tinyos/local/src/tinyos-2.x/tos/lib/timer/TransformAlarmC.nc"
static inline void /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__AlarmFrom__fired(void )
{
  /* atomic removed: atomic calls only */
  {
    if (/*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__m_dt == 0) 
      {
        /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__Alarm__fired();
      }
    else 
      {
        /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__set_alarm();
      }
  }
}

# 67 "/home/tinyos/local/src/tinyos-2.x/tos/lib/timer/Alarm.nc"
inline static void /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__0__Alarm__fired(void ){
#line 67
  /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__AlarmFrom__fired();
#line 67
}
#line 67
# 124 "/home/tinyos/local/src/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static inline void /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Control__disableEvents(void )
{
  * (volatile uint16_t * )386U &= ~0x0010;
}

# 47 "/home/tinyos/local/src/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerControl.nc"
inline static void /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__0__Msp430TimerControl__disableEvents(void ){
#line 47
  /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Control__disableEvents();
#line 47
}
#line 47
# 59 "/home/tinyos/local/src/tinyos-2.x/tos/chips/msp430/timer/Msp430AlarmC.nc"
static inline void /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__0__Msp430Compare__fired(void )
{
  /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__0__Msp430TimerControl__disableEvents();
  /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__0__Alarm__fired();
}

# 34 "/home/tinyos/local/src/tinyos-2.x/tos/chips/msp430/timer/Msp430Compare.nc"
inline static void /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Compare__fired(void ){
#line 34
  /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__0__Msp430Compare__fired();
#line 34
}
#line 34
# 139 "/home/tinyos/local/src/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static inline uint16_t /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Capture__getEvent(void )
{
  return * (volatile uint16_t * )402U;
}

#line 177
static inline void /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Capture__default__captured(uint16_t n)
{
}

# 75 "/home/tinyos/local/src/tinyos-2.x/tos/chips/msp430/timer/Msp430Capture.nc"
inline static void /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Capture__captured(uint16_t time){
#line 75
  /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Capture__default__captured(time);
#line 75
}
#line 75
# 47 "/home/tinyos/local/src/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static inline  /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__cc_t /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__int2CC(uint16_t x)
#line 47
{
#line 47
  union /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3____nesc_unnamed4449 {
#line 47
    uint16_t f;
#line 47
    /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__cc_t t;
  } 
#line 47
  c = { .f = x };

#line 47
  return c.t;
}

#line 74
static inline /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__cc_t /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Control__getControl(void )
{
  return /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__int2CC(* (volatile uint16_t * )386U);
}

#line 169
static inline void /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Event__fired(void )
{
  if (/*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Control__getControl().cap) {
    /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Capture__captured(/*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Capture__getEvent());
    }
  else {
#line 174
    /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Compare__fired();
    }
}

# 86 "/home/tinyos/local/src/tinyos-2.x/tos/system/SchedulerBasicP.nc"
static inline bool SchedulerBasicP__isWaiting(uint8_t id)
{
  return SchedulerBasicP__m_next[id] != SchedulerBasicP__NO_TASK || SchedulerBasicP__m_tail == id;
}

static inline bool SchedulerBasicP__pushTask(uint8_t id)
{
  if (!SchedulerBasicP__isWaiting(id)) 
    {
      if (SchedulerBasicP__m_head == SchedulerBasicP__NO_TASK) 
        {
          SchedulerBasicP__m_head = id;
          SchedulerBasicP__m_tail = id;
        }
      else 
        {
          SchedulerBasicP__m_next[SchedulerBasicP__m_tail] = id;
          SchedulerBasicP__m_tail = id;
        }
      return TRUE;
    }
  else 
    {
      return FALSE;
    }
}

# 34 "/home/tinyos/local/src/tinyos-2.x/tos/chips/msp430/timer/Msp430Timer.nc"
inline static uint16_t /*Msp430Counter32khzC.Counter*/Msp430CounterC__0__Msp430Timer__get(void ){
#line 34
  unsigned int result;
#line 34

#line 34
  result = /*Msp430TimerC.Msp430TimerB*/Msp430TimerP__1__Timer__get();
#line 34

#line 34
  return result;
#line 34
}
#line 34
# 38 "/home/tinyos/local/src/tinyos-2.x/tos/chips/msp430/timer/Msp430CounterC.nc"
static inline uint16_t /*Msp430Counter32khzC.Counter*/Msp430CounterC__0__Counter__get(void )
{
  return /*Msp430Counter32khzC.Counter*/Msp430CounterC__0__Msp430Timer__get();
}

# 53 "/home/tinyos/local/src/tinyos-2.x/tos/lib/timer/Counter.nc"
inline static /*CounterMilli32C.Transform*/TransformCounterC__0__CounterFrom__size_type /*CounterMilli32C.Transform*/TransformCounterC__0__CounterFrom__get(void ){
#line 53
  unsigned int result;
#line 53

#line 53
  result = /*Msp430Counter32khzC.Counter*/Msp430CounterC__0__Counter__get();
#line 53

#line 53
  return result;
#line 53
}
#line 53
# 70 "/home/tinyos/local/src/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerP.nc"
static inline bool /*Msp430TimerC.Msp430TimerB*/Msp430TimerP__1__Timer__isOverflowPending(void )
{
  return * (volatile uint16_t * )384U & 1U;
}

# 35 "/home/tinyos/local/src/tinyos-2.x/tos/chips/msp430/timer/Msp430Timer.nc"
inline static bool /*Msp430Counter32khzC.Counter*/Msp430CounterC__0__Msp430Timer__isOverflowPending(void ){
#line 35
  unsigned char result;
#line 35

#line 35
  result = /*Msp430TimerC.Msp430TimerB*/Msp430TimerP__1__Timer__isOverflowPending();
#line 35

#line 35
  return result;
#line 35
}
#line 35
# 43 "/home/tinyos/local/src/tinyos-2.x/tos/chips/msp430/timer/Msp430CounterC.nc"
static inline bool /*Msp430Counter32khzC.Counter*/Msp430CounterC__0__Counter__isOverflowPending(void )
{
  return /*Msp430Counter32khzC.Counter*/Msp430CounterC__0__Msp430Timer__isOverflowPending();
}

# 60 "/home/tinyos/local/src/tinyos-2.x/tos/lib/timer/Counter.nc"
inline static bool /*CounterMilli32C.Transform*/TransformCounterC__0__CounterFrom__isOverflowPending(void ){
#line 60
  unsigned char result;
#line 60

#line 60
  result = /*Msp430Counter32khzC.Counter*/Msp430CounterC__0__Counter__isOverflowPending();
#line 60

#line 60
  return result;
#line 60
}
#line 60
# 119 "/home/tinyos/local/src/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static inline void /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Control__enableEvents(void )
{
  * (volatile uint16_t * )386U |= 0x0010;
}

# 46 "/home/tinyos/local/src/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerControl.nc"
inline static void /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__0__Msp430TimerControl__enableEvents(void ){
#line 46
  /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Control__enableEvents();
#line 46
}
#line 46
# 84 "/home/tinyos/local/src/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static inline void /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Control__clearPendingInterrupt(void )
{
  * (volatile uint16_t * )386U &= ~0x0001;
}

# 33 "/home/tinyos/local/src/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerControl.nc"
inline static void /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__0__Msp430TimerControl__clearPendingInterrupt(void ){
#line 33
  /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Control__clearPendingInterrupt();
#line 33
}
#line 33
# 144 "/home/tinyos/local/src/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static inline void /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Compare__setEvent(uint16_t x)
{
  * (volatile uint16_t * )402U = x;
}

# 30 "/home/tinyos/local/src/tinyos-2.x/tos/chips/msp430/timer/Msp430Compare.nc"
inline static void /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__0__Msp430Compare__setEvent(uint16_t time){
#line 30
  /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Compare__setEvent(time);
#line 30
}
#line 30
# 34 "/home/tinyos/local/src/tinyos-2.x/tos/chips/msp430/timer/Msp430Timer.nc"
inline static uint16_t /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Timer__get(void ){
#line 34
  unsigned int result;
#line 34

#line 34
  result = /*Msp430TimerC.Msp430TimerB*/Msp430TimerP__1__Timer__get();
#line 34

#line 34
  return result;
#line 34
}
#line 34
# 154 "/home/tinyos/local/src/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static inline void /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Compare__setEventFromNow(uint16_t x)
{
  * (volatile uint16_t * )402U = /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Timer__get() + x;
}

# 32 "/home/tinyos/local/src/tinyos-2.x/tos/chips/msp430/timer/Msp430Compare.nc"
inline static void /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__0__Msp430Compare__setEventFromNow(uint16_t delta){
#line 32
  /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Compare__setEventFromNow(delta);
#line 32
}
#line 32
# 34 "/home/tinyos/local/src/tinyos-2.x/tos/chips/msp430/timer/Msp430Timer.nc"
inline static uint16_t /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__0__Msp430Timer__get(void ){
#line 34
  unsigned int result;
#line 34

#line 34
  result = /*Msp430TimerC.Msp430TimerB*/Msp430TimerP__1__Timer__get();
#line 34

#line 34
  return result;
#line 34
}
#line 34
# 70 "/home/tinyos/local/src/tinyos-2.x/tos/chips/msp430/timer/Msp430AlarmC.nc"
static inline void /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__0__Alarm__startAt(uint16_t t0, uint16_t dt)
{
  /* atomic removed: atomic calls only */
  {
    uint16_t now = /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__0__Msp430Timer__get();
    uint16_t elapsed = now - t0;

#line 76
    if (elapsed >= dt) 
      {
        /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__0__Msp430Compare__setEventFromNow(2);
      }
    else 
      {
        uint16_t remaining = dt - elapsed;

#line 83
        if (remaining <= 2) {
          /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__0__Msp430Compare__setEventFromNow(2);
          }
        else {
#line 86
          /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__0__Msp430Compare__setEvent(now + remaining);
          }
      }
#line 88
    /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__0__Msp430TimerControl__clearPendingInterrupt();
    /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__0__Msp430TimerControl__enableEvents();
  }
}

# 92 "/home/tinyos/local/src/tinyos-2.x/tos/lib/timer/Alarm.nc"
inline static void /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__AlarmFrom__startAt(/*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__AlarmFrom__size_type t0, /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__AlarmFrom__size_type dt){
#line 92
  /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__0__Alarm__startAt(t0, dt);
#line 92
}
#line 92
# 181 "/home/tinyos/local/src/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static inline void /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__Compare__default__fired(void )
{
}

# 34 "/home/tinyos/local/src/tinyos-2.x/tos/chips/msp430/timer/Msp430Compare.nc"
inline static void /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__Compare__fired(void ){
#line 34
  /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__Compare__default__fired();
#line 34
}
#line 34
# 139 "/home/tinyos/local/src/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static inline uint16_t /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__Capture__getEvent(void )
{
  return * (volatile uint16_t * )404U;
}

# 276 "/usr/lib/ncc/nesc_nx.h"
static __inline  uint16_t __nesc_ntoh_leuint16(const void * source)
#line 276
{
  const uint8_t *base = source;

#line 278
  return ((uint16_t )base[1] << 8) | base[0];
}

#line 301
static __inline  uint32_t __nesc_hton_uint32(void * target, uint32_t value)
#line 301
{
  uint8_t *base = target;

#line 303
  base[3] = value;
  base[2] = value >> 8;
  base[1] = value >> 16;
  base[0] = value >> 24;
  return value;
}

#line 294
static __inline  uint32_t __nesc_ntoh_uint32(const void * source)
#line 294
{
  const uint8_t *base = source;

#line 296
  return ((((uint32_t )base[0] << 24) | (
  (uint32_t )base[1] << 16)) | (
  (uint32_t )base[2] << 8)) | base[3];
}

# 59 "/home/tinyos/local/src/tinyos-2.x/tos/interfaces/PacketTimeStamp.nc"
inline static void CC2420TransmitP__PacketTimeStamp__clear(message_t * msg){
#line 59
  CC2420PacketP__PacketTimeStamp32khz__clear(msg);
#line 59
}
#line 59
# 195 "/home/tinyos/local/src/tinyos-2.x/tos/chips/cc2420/receive/CC2420ReceiveP.nc"
static inline void CC2420ReceiveP__CC2420Receive__sfd_dropped(void )
#line 195
{
  if (CC2420ReceiveP__m_timestamp_size) {
      CC2420ReceiveP__m_timestamp_size--;
    }
}

# 55 "/home/tinyos/local/src/tinyos-2.x/tos/chips/cc2420/interfaces/CC2420Receive.nc"
inline static void CC2420TransmitP__CC2420Receive__sfd_dropped(void ){
#line 55
  CC2420ReceiveP__CC2420Receive__sfd_dropped();
#line 55
}
#line 55
# 50 "/home/tinyos/local/src/tinyos-2.x/tos/chips/msp430/timer/GpioCaptureC.nc"
static inline error_t /*HplCC2420InterruptsC.CaptureSFDC*/GpioCaptureC__0__Capture__captureRisingEdge(void )
#line 50
{
  return /*HplCC2420InterruptsC.CaptureSFDC*/GpioCaptureC__0__enableCapture(MSP430TIMER_CM_RISING);
}

# 42 "/home/tinyos/local/src/tinyos-2.x/tos/interfaces/GpioCapture.nc"
inline static error_t CC2420TransmitP__CaptureSFD__captureRisingEdge(void ){
#line 42
  unsigned char result;
#line 42

#line 42
  result = /*HplCC2420InterruptsC.CaptureSFDC*/GpioCaptureC__0__Capture__captureRisingEdge();
#line 42

#line 42
  return result;
#line 42
}
#line 42
# 48 "/home/tinyos/local/src/tinyos-2.x/tos/chips/msp430/pins/HplMsp430GeneralIOP.nc"
static inline uint8_t /*HplMsp430GeneralIOC.P41*/HplMsp430GeneralIOP__25__IO__getRaw(void )
#line 48
{
#line 48
  return * (volatile uint8_t * )28U & (0x01 << 1);
}

#line 49
static inline bool /*HplMsp430GeneralIOC.P41*/HplMsp430GeneralIOP__25__IO__get(void )
#line 49
{
#line 49
  return /*HplMsp430GeneralIOC.P41*/HplMsp430GeneralIOP__25__IO__getRaw() != 0;
}

# 59 "/home/tinyos/local/src/tinyos-2.x/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
inline static bool /*HplCC2420PinsC.SFDM*/Msp430GpioC__8__HplGeneralIO__get(void ){
#line 59
  unsigned char result;
#line 59

#line 59
  result = /*HplMsp430GeneralIOC.P41*/HplMsp430GeneralIOP__25__IO__get();
#line 59

#line 59
  return result;
#line 59
}
#line 59
# 40 "/home/tinyos/local/src/tinyos-2.x/tos/chips/msp430/pins/Msp430GpioC.nc"
static inline bool /*HplCC2420PinsC.SFDM*/Msp430GpioC__8__GeneralIO__get(void )
#line 40
{
#line 40
  return /*HplCC2420PinsC.SFDM*/Msp430GpioC__8__HplGeneralIO__get();
}

# 32 "/home/tinyos/local/src/tinyos-2.x/tos/interfaces/GeneralIO.nc"
inline static bool CC2420TransmitP__SFD__get(void ){
#line 32
  unsigned char result;
#line 32

#line 32
  result = /*HplCC2420PinsC.SFDM*/Msp430GpioC__8__GeneralIO__get();
#line 32

#line 32
  return result;
#line 32
}
#line 32
# 186 "/home/tinyos/local/src/tinyos-2.x/tos/chips/cc2420/receive/CC2420ReceiveP.nc"
static inline void CC2420ReceiveP__CC2420Receive__sfd(uint32_t time)
#line 186
{
  if (CC2420ReceiveP__m_timestamp_size < CC2420ReceiveP__TIMESTAMP_QUEUE_SIZE) {
      uint8_t tail = (CC2420ReceiveP__m_timestamp_head + CC2420ReceiveP__m_timestamp_size) % 
      CC2420ReceiveP__TIMESTAMP_QUEUE_SIZE;

#line 190
      CC2420ReceiveP__m_timestamp_queue[tail] = time;
      CC2420ReceiveP__m_timestamp_size++;
    }
}

# 49 "/home/tinyos/local/src/tinyos-2.x/tos/chips/cc2420/interfaces/CC2420Receive.nc"
inline static void CC2420TransmitP__CC2420Receive__sfd(uint32_t time){
#line 49
  CC2420ReceiveP__CC2420Receive__sfd(time);
#line 49
}
#line 49
# 54 "/home/tinyos/local/src/tinyos-2.x/tos/chips/msp430/timer/GpioCaptureC.nc"
static inline error_t /*HplCC2420InterruptsC.CaptureSFDC*/GpioCaptureC__0__Capture__captureFallingEdge(void )
#line 54
{
  return /*HplCC2420InterruptsC.CaptureSFDC*/GpioCaptureC__0__enableCapture(MSP430TIMER_CM_FALLING);
}

# 43 "/home/tinyos/local/src/tinyos-2.x/tos/interfaces/GpioCapture.nc"
inline static error_t CC2420TransmitP__CaptureSFD__captureFallingEdge(void ){
#line 43
  unsigned char result;
#line 43

#line 43
  result = /*HplCC2420InterruptsC.CaptureSFDC*/GpioCaptureC__0__Capture__captureFallingEdge();
#line 43

#line 43
  return result;
#line 43
}
#line 43
# 53 "/home/tinyos/local/src/tinyos-2.x/tos/lib/timer/Counter.nc"
inline static /*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC__1__Counter__size_type /*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC__1__Counter__get(void ){
#line 53
  unsigned long result;
#line 53

#line 53
  result = /*Counter32khz32C.Transform*/TransformCounterC__1__Counter__get();
#line 53

#line 53
  return result;
#line 53
}
#line 53
# 75 "/home/tinyos/local/src/tinyos-2.x/tos/lib/timer/TransformAlarmC.nc"
static inline /*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC__1__to_size_type /*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC__1__Alarm__getNow(void )
{
  return /*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC__1__Counter__get();
}

#line 146
static inline void /*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC__1__Alarm__start(/*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC__1__to_size_type dt)
{
  /*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC__1__Alarm__startAt(/*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC__1__Alarm__getNow(), dt);
}

# 55 "/home/tinyos/local/src/tinyos-2.x/tos/lib/timer/Alarm.nc"
inline static void CC2420TransmitP__BackoffTimer__start(CC2420TransmitP__BackoffTimer__size_type dt){
#line 55
  /*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC__1__Alarm__start(dt);
#line 55
}
#line 55
# 114 "/home/tinyos/local/src/tinyos-2.x/tos/chips/cc2420/packet/CC2420PacketP.nc"
static inline cc2420_header_t * CC2420PacketP__CC2420PacketBody__getHeader(message_t * msg)
#line 114
{
  return (cc2420_header_t * )((uint8_t *)msg + (size_t )& ((message_t *)0)->data - sizeof(cc2420_header_t ));
}

# 42 "/home/tinyos/local/src/tinyos-2.x/tos/chips/cc2420/interfaces/CC2420PacketBody.nc"
inline static cc2420_header_t * CC2420TransmitP__CC2420PacketBody__getHeader(message_t * msg){
#line 42
  nx_struct cc2420_header_t *result;
#line 42

#line 42
  result = CC2420PacketP__CC2420PacketBody__getHeader(msg);
#line 42

#line 42
  return result;
#line 42
}
#line 42
# 124 "/home/tinyos/local/src/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static inline void /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__Control__disableEvents(void )
{
  * (volatile uint16_t * )390U &= ~0x0010;
}

# 47 "/home/tinyos/local/src/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerControl.nc"
inline static void /*AlarmMultiplexC.Alarm.Alarm32khz32C.AlarmC.Msp430Alarm*/Msp430AlarmC__1__Msp430TimerControl__disableEvents(void ){
#line 47
  /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__Control__disableEvents();
#line 47
}
#line 47
# 54 "/home/tinyos/local/src/tinyos-2.x/tos/chips/msp430/timer/Msp430AlarmC.nc"
static inline void /*AlarmMultiplexC.Alarm.Alarm32khz32C.AlarmC.Msp430Alarm*/Msp430AlarmC__1__Alarm__stop(void )
{
  /*AlarmMultiplexC.Alarm.Alarm32khz32C.AlarmC.Msp430Alarm*/Msp430AlarmC__1__Msp430TimerControl__disableEvents();
}

# 62 "/home/tinyos/local/src/tinyos-2.x/tos/lib/timer/Alarm.nc"
inline static void /*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC__1__AlarmFrom__stop(void ){
#line 62
  /*AlarmMultiplexC.Alarm.Alarm32khz32C.AlarmC.Msp430Alarm*/Msp430AlarmC__1__Alarm__stop();
#line 62
}
#line 62
# 91 "/home/tinyos/local/src/tinyos-2.x/tos/lib/timer/TransformAlarmC.nc"
static inline void /*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC__1__Alarm__stop(void )
{
  /*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC__1__AlarmFrom__stop();
}

# 62 "/home/tinyos/local/src/tinyos-2.x/tos/lib/timer/Alarm.nc"
inline static void CC2420TransmitP__BackoffTimer__stop(void ){
#line 62
  /*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC__1__Alarm__stop();
#line 62
}
#line 62
# 110 "/home/tinyos/local/src/tinyos-2.x/tos/interfaces/Resource.nc"
inline static error_t CC2420TransmitP__SpiResource__release(void ){
#line 110
  unsigned char result;
#line 110

#line 110
  result = CC2420SpiP__Resource__release(/*CC2420TransmitC.Spi*/CC2420SpiC__3__CLIENT_ID);
#line 110

#line 110
  return result;
#line 110
}
#line 110
# 795 "/home/tinyos/local/src/tinyos-2.x/tos/chips/cc2420/transmit/CC2420TransmitP.nc"
static inline error_t CC2420TransmitP__releaseSpiResource(void )
#line 795
{
  CC2420TransmitP__SpiResource__release();
  return SUCCESS;
}

# 34 "/home/tinyos/local/src/tinyos-2.x/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
inline static void /*HplCC2420PinsC.CSNM*/Msp430GpioC__4__HplGeneralIO__set(void ){
#line 34
  /*HplMsp430GeneralIOC.P42*/HplMsp430GeneralIOP__26__IO__set();
#line 34
}
#line 34
# 37 "/home/tinyos/local/src/tinyos-2.x/tos/chips/msp430/pins/Msp430GpioC.nc"
static inline void /*HplCC2420PinsC.CSNM*/Msp430GpioC__4__GeneralIO__set(void )
#line 37
{
#line 37
  /*HplCC2420PinsC.CSNM*/Msp430GpioC__4__HplGeneralIO__set();
}

# 29 "/home/tinyos/local/src/tinyos-2.x/tos/interfaces/GeneralIO.nc"
inline static void CC2420TransmitP__CSN__set(void ){
#line 29
  /*HplCC2420PinsC.CSNM*/Msp430GpioC__4__GeneralIO__set();
#line 29
}
#line 29
# 63 "/home/tinyos/local/src/tinyos-2.x/tos/chips/cc2420/interfaces/CC2420Ram.nc"
inline static cc2420_status_t CC2420TransmitP__TXFIFO_RAM__write(uint8_t offset, uint8_t * data, uint8_t length){
#line 63
  unsigned char result;
#line 63

#line 63
  result = CC2420SpiP__Ram__write(CC2420_RAM_TXFIFO, offset, data, length);
#line 63

#line 63
  return result;
#line 63
}
#line 63
# 39 "/home/tinyos/local/src/tinyos-2.x/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
inline static void /*HplCC2420PinsC.CSNM*/Msp430GpioC__4__HplGeneralIO__clr(void ){
#line 39
  /*HplMsp430GeneralIOC.P42*/HplMsp430GeneralIOP__26__IO__clr();
#line 39
}
#line 39
# 38 "/home/tinyos/local/src/tinyos-2.x/tos/chips/msp430/pins/Msp430GpioC.nc"
static inline void /*HplCC2420PinsC.CSNM*/Msp430GpioC__4__GeneralIO__clr(void )
#line 38
{
#line 38
  /*HplCC2420PinsC.CSNM*/Msp430GpioC__4__HplGeneralIO__clr();
}

# 30 "/home/tinyos/local/src/tinyos-2.x/tos/interfaces/GeneralIO.nc"
inline static void CC2420TransmitP__CSN__clr(void ){
#line 30
  /*HplCC2420PinsC.CSNM*/Msp430GpioC__4__GeneralIO__clr();
#line 30
}
#line 30
# 246 "/usr/lib/ncc/nesc_nx.h"
static __inline  uint8_t __nesc_ntoh_leuint8(const void * source)
#line 246
{
  const uint8_t *base = source;

#line 248
  return base[0];
}

# 185 "/home/tinyos/local/src/tinyos-2.x/tos/chips/cc2420/packet/CC2420PacketP.nc"
static inline uint8_t CC2420PacketP__PacketTimeSyncOffset__get(message_t *msg)
{
  return __nesc_ntoh_leuint8(CC2420PacketP__CC2420PacketBody__getHeader(msg)->length.data)
   + (sizeof(cc2420_header_t ) - MAC_HEADER_SIZE)
   - MAC_FOOTER_SIZE
   - sizeof(timesync_radio_t );
}

# 47 "/home/tinyos/local/src/tinyos-2.x/tos/chips/cc2420/interfaces/PacketTimeSyncOffset.nc"
inline static uint8_t CC2420TransmitP__PacketTimeSyncOffset__get(message_t * msg){
#line 47
  unsigned char result;
#line 47

#line 47
  result = CC2420PacketP__PacketTimeSyncOffset__get(msg);
#line 47

#line 47
  return result;
#line 47
}
#line 47
# 235 "/usr/lib/ncc/nesc_nx.h"
static __inline  uint8_t __nesc_ntoh_uint8(const void * source)
#line 235
{
  const uint8_t *base = source;

#line 237
  return base[0];
}

#line 257
static __inline  int8_t __nesc_ntoh_int8(const void * source)
#line 257
{
#line 257
  return __nesc_ntoh_uint8(source);
}

# 118 "/home/tinyos/local/src/tinyos-2.x/tos/chips/cc2420/packet/CC2420PacketP.nc"
static inline cc2420_metadata_t *CC2420PacketP__CC2420PacketBody__getMetadata(message_t *msg)
#line 118
{
  return (cc2420_metadata_t *)msg->metadata;
}

#line 176
static inline bool CC2420PacketP__PacketTimeSyncOffset__isSet(message_t *msg)
{
  return __nesc_ntoh_int8(CC2420PacketP__CC2420PacketBody__getMetadata(msg)->timesync.data);
}

# 39 "/home/tinyos/local/src/tinyos-2.x/tos/chips/cc2420/interfaces/PacketTimeSyncOffset.nc"
inline static bool CC2420TransmitP__PacketTimeSyncOffset__isSet(message_t * msg){
#line 39
  unsigned char result;
#line 39

#line 39
  result = CC2420PacketP__PacketTimeSyncOffset__isSet(msg);
#line 39

#line 39
  return result;
#line 39
}
#line 39
# 143 "/home/tinyos/local/src/tinyos-2.x/tos/chips/cc2420/packet/CC2420PacketP.nc"
static inline void CC2420PacketP__PacketTimeStamp32khz__set(message_t *msg, uint32_t value)
{
  __nesc_hton_uint32(CC2420PacketP__CC2420PacketBody__getMetadata(msg)->timestamp.data, value);
}

# 67 "/home/tinyos/local/src/tinyos-2.x/tos/interfaces/PacketTimeStamp.nc"
inline static void CC2420TransmitP__PacketTimeStamp__set(message_t * msg, CC2420TransmitP__PacketTimeStamp__size_type value){
#line 67
  CC2420PacketP__PacketTimeStamp32khz__set(msg, value);
#line 67
}
#line 67
# 98 "/home/tinyos/local/src/tinyos-2.x/tos/lib/timer/Alarm.nc"
inline static CC2420TransmitP__BackoffTimer__size_type CC2420TransmitP__BackoffTimer__getNow(void ){
#line 98
  unsigned long result;
#line 98

#line 98
  result = /*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC__1__Alarm__getNow();
#line 98

#line 98
  return result;
#line 98
}
#line 98
# 259 "/home/tinyos/local/src/tinyos-2.x/tos/chips/cc2420/transmit/CC2420TransmitP.nc"
static __inline uint32_t CC2420TransmitP__getTime32(uint16_t time)
{
  uint32_t recent_time = CC2420TransmitP__BackoffTimer__getNow();

#line 262
  return recent_time + (int16_t )(time - recent_time);
}

#line 278
static inline void CC2420TransmitP__CaptureSFD__captured(uint16_t time)
#line 278
{
  unsigned char *__nesc_temp47;
  unsigned char *__nesc_temp46;
#line 279
  uint32_t time32;
  uint8_t sfd_state = 0;

  /* atomic removed: atomic calls only */
#line 281
  {
    time32 = CC2420TransmitP__getTime32(time);
    switch (CC2420TransmitP__m_state) {

        case CC2420TransmitP__S_SFD: 
          CC2420TransmitP__m_state = CC2420TransmitP__S_EFD;
        CC2420TransmitP__sfdHigh = TRUE;


        CC2420TransmitP__m_receiving = FALSE;
        CC2420TransmitP__CaptureSFD__captureFallingEdge();
        CC2420TransmitP__PacketTimeStamp__set(CC2420TransmitP__m_msg, time32);
        if (CC2420TransmitP__PacketTimeSyncOffset__isSet(CC2420TransmitP__m_msg)) {
            uint8_t absOffset = sizeof(message_header_t ) - sizeof(cc2420_header_t ) + CC2420TransmitP__PacketTimeSyncOffset__get(CC2420TransmitP__m_msg);
            timesync_radio_t *timesync = (timesync_radio_t *)((nx_uint8_t *)CC2420TransmitP__m_msg + absOffset);

            (__nesc_temp46 = (*timesync).data, __nesc_hton_uint32(__nesc_temp46, __nesc_ntoh_uint32(__nesc_temp46) - time32));
            CC2420TransmitP__CSN__clr();
            CC2420TransmitP__TXFIFO_RAM__write(absOffset, (uint8_t *)timesync, sizeof(timesync_radio_t ));
            CC2420TransmitP__CSN__set();

            (__nesc_temp47 = (*timesync).data, __nesc_hton_uint32(__nesc_temp47, __nesc_ntoh_uint32(__nesc_temp47) + time32));
          }

        if (__nesc_ntoh_leuint16(CC2420TransmitP__CC2420PacketBody__getHeader(CC2420TransmitP__m_msg)->fcf.data) & (1 << IEEE154_FCF_ACK_REQ)) {

            CC2420TransmitP__abortSpiRelease = TRUE;
          }
        CC2420TransmitP__releaseSpiResource();
        CC2420TransmitP__BackoffTimer__stop();

        if (CC2420TransmitP__SFD__get()) {
            break;
          }


        case CC2420TransmitP__S_EFD: 
          CC2420TransmitP__sfdHigh = FALSE;
        CC2420TransmitP__CaptureSFD__captureRisingEdge();

        if (__nesc_ntoh_leuint16(CC2420TransmitP__CC2420PacketBody__getHeader(CC2420TransmitP__m_msg)->fcf.data) & (1 << IEEE154_FCF_ACK_REQ)) {
            CC2420TransmitP__m_state = CC2420TransmitP__S_ACK_WAIT;
            CC2420TransmitP__BackoffTimer__start(CC2420_ACK_WAIT_DELAY);
          }
        else 
#line 324
          {
            CC2420TransmitP__signalDone(SUCCESS);
          }

        if (!CC2420TransmitP__SFD__get()) {
            break;
          }


        default: 

          if (!CC2420TransmitP__m_receiving && CC2420TransmitP__sfdHigh == FALSE) {
              CC2420TransmitP__sfdHigh = TRUE;
              CC2420TransmitP__CaptureSFD__captureFallingEdge();

              sfd_state = CC2420TransmitP__SFD__get();
              CC2420TransmitP__CC2420Receive__sfd(time32);
              CC2420TransmitP__m_receiving = TRUE;
              CC2420TransmitP__m_prev_time = time;
              if (CC2420TransmitP__SFD__get()) {

                  return;
                }
            }



        if (CC2420TransmitP__sfdHigh == TRUE) {
            CC2420TransmitP__sfdHigh = FALSE;
            CC2420TransmitP__CaptureSFD__captureRisingEdge();
            CC2420TransmitP__m_receiving = FALSE;








            if (sfd_state == 0 && time - CC2420TransmitP__m_prev_time < 10) {
                CC2420TransmitP__CC2420Receive__sfd_dropped();
                if (CC2420TransmitP__m_msg) {
                  CC2420TransmitP__PacketTimeStamp__clear(CC2420TransmitP__m_msg);
                  }
              }
#line 368
            break;
          }
      }
  }
}

# 50 "/home/tinyos/local/src/tinyos-2.x/tos/interfaces/GpioCapture.nc"
inline static void /*HplCC2420InterruptsC.CaptureSFDC*/GpioCaptureC__0__Capture__captured(uint16_t time){
#line 50
  CC2420TransmitP__CaptureSFD__captured(time);
#line 50
}
#line 50
# 164 "/home/tinyos/local/src/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static inline void /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__Capture__clearOverflow(void )
{
  * (volatile uint16_t * )388U &= ~0x0002;
}

# 57 "/home/tinyos/local/src/tinyos-2.x/tos/chips/msp430/timer/Msp430Capture.nc"
inline static void /*HplCC2420InterruptsC.CaptureSFDC*/GpioCaptureC__0__Msp430Capture__clearOverflow(void ){
#line 57
  /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__Capture__clearOverflow();
#line 57
}
#line 57
# 84 "/home/tinyos/local/src/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static inline void /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__Control__clearPendingInterrupt(void )
{
  * (volatile uint16_t * )388U &= ~0x0001;
}

# 33 "/home/tinyos/local/src/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerControl.nc"
inline static void /*HplCC2420InterruptsC.CaptureSFDC*/GpioCaptureC__0__Msp430TimerControl__clearPendingInterrupt(void ){
#line 33
  /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__Control__clearPendingInterrupt();
#line 33
}
#line 33
# 65 "/home/tinyos/local/src/tinyos-2.x/tos/chips/msp430/timer/GpioCaptureC.nc"
static inline void /*HplCC2420InterruptsC.CaptureSFDC*/GpioCaptureC__0__Msp430Capture__captured(uint16_t time)
#line 65
{
  /*HplCC2420InterruptsC.CaptureSFDC*/GpioCaptureC__0__Msp430TimerControl__clearPendingInterrupt();
  /*HplCC2420InterruptsC.CaptureSFDC*/GpioCaptureC__0__Msp430Capture__clearOverflow();
  /*HplCC2420InterruptsC.CaptureSFDC*/GpioCaptureC__0__Capture__captured(time);
}

# 75 "/home/tinyos/local/src/tinyos-2.x/tos/chips/msp430/timer/Msp430Capture.nc"
inline static void /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__Capture__captured(uint16_t time){
#line 75
  /*HplCC2420InterruptsC.CaptureSFDC*/GpioCaptureC__0__Msp430Capture__captured(time);
#line 75
}
#line 75
# 47 "/home/tinyos/local/src/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static inline  /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__cc_t /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__int2CC(uint16_t x)
#line 47
{
#line 47
  union /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4____nesc_unnamed4450 {
#line 47
    uint16_t f;
#line 47
    /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__cc_t t;
  } 
#line 47
  c = { .f = x };

#line 47
  return c.t;
}

#line 74
static inline /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__cc_t /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__Control__getControl(void )
{
  return /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__int2CC(* (volatile uint16_t * )388U);
}

#line 169
static inline void /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__Event__fired(void )
{
  if (/*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__Control__getControl().cap) {
    /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__Capture__captured(/*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__Capture__getEvent());
    }
  else {
#line 174
    /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__Compare__fired();
    }
}

# 53 "/home/tinyos/local/src/tinyos-2.x/tos/lib/timer/Counter.nc"
inline static /*Counter32khz32C.Transform*/TransformCounterC__1__CounterFrom__size_type /*Counter32khz32C.Transform*/TransformCounterC__1__CounterFrom__get(void ){
#line 53
  unsigned int result;
#line 53

#line 53
  result = /*Msp430Counter32khzC.Counter*/Msp430CounterC__0__Counter__get();
#line 53

#line 53
  return result;
#line 53
}
#line 53







inline static bool /*Counter32khz32C.Transform*/TransformCounterC__1__CounterFrom__isOverflowPending(void ){
#line 60
  unsigned char result;
#line 60

#line 60
  result = /*Msp430Counter32khzC.Counter*/Msp430CounterC__0__Counter__isOverflowPending();
#line 60

#line 60
  return result;
#line 60
}
#line 60
# 54 "/home/tinyos/local/src/tinyos-2.x/tos/chips/msp430/pins/HplMsp430GeneralIOP.nc"
static inline void /*HplMsp430GeneralIOC.P41*/HplMsp430GeneralIOP__25__IO__selectModuleFunc(void )
#line 54
{
  /* atomic removed: atomic calls only */
#line 54
  * (volatile uint8_t * )31U |= 0x01 << 1;
}

# 78 "/home/tinyos/local/src/tinyos-2.x/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
inline static void /*HplCC2420InterruptsC.CaptureSFDC*/GpioCaptureC__0__GeneralIO__selectModuleFunc(void ){
#line 78
  /*HplMsp430GeneralIOC.P41*/HplMsp430GeneralIOP__25__IO__selectModuleFunc();
#line 78
}
#line 78
# 46 "/home/tinyos/local/src/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static inline  uint16_t /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__CC2int(/*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__cc_t x)
#line 46
{
#line 46
  union /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4____nesc_unnamed4451 {
#line 46
    /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__cc_t f;
#line 46
    uint16_t t;
  } 
#line 46
  c = { .f = x };

#line 46
  return c.t;
}

#line 61
static inline uint16_t /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__captureControl(uint8_t l_cm)
{
  /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__cc_t x = { 
  .cm = l_cm & 0x03, 
  .ccis = 0, 
  .clld = 0, 
  .cap = 1, 
  .scs = 0, 
  .ccie = 0 };

  return /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__CC2int(x);
}

#line 99
static inline void /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__Control__setControlAsCapture(uint8_t cm)
{
  * (volatile uint16_t * )388U = /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__captureControl(cm);
}

# 44 "/home/tinyos/local/src/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerControl.nc"
inline static void /*HplCC2420InterruptsC.CaptureSFDC*/GpioCaptureC__0__Msp430TimerControl__setControlAsCapture(uint8_t cm){
#line 44
  /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__Control__setControlAsCapture(cm);
#line 44
}
#line 44
# 119 "/home/tinyos/local/src/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static inline void /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__Control__enableEvents(void )
{
  * (volatile uint16_t * )388U |= 0x0010;
}

# 46 "/home/tinyos/local/src/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerControl.nc"
inline static void /*HplCC2420InterruptsC.CaptureSFDC*/GpioCaptureC__0__Msp430TimerControl__enableEvents(void ){
#line 46
  /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__Control__enableEvents();
#line 46
}
#line 46
# 382 "/home/tinyos/local/src/tinyos-2.x/tos/chips/msp430/usart/HplMsp430Usart0P.nc"
static inline void HplMsp430Usart0P__Usart__tx(uint8_t data)
#line 382
{
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 383
    HplMsp430Usart0P__U0TXBUF = data;
#line 383
    __nesc_atomic_end(__nesc_atomic); }
}

# 224 "/home/tinyos/local/src/tinyos-2.x/tos/chips/msp430/usart/HplMsp430Usart.nc"
inline static void /*Msp430SpiDma0P.SpiP*/Msp430SpiDmaP__0__Usart__tx(uint8_t data){
#line 224
  HplMsp430Usart0P__Usart__tx(data);
#line 224
}
#line 224
# 330 "/home/tinyos/local/src/tinyos-2.x/tos/chips/msp430/usart/HplMsp430Usart0P.nc"
static inline bool HplMsp430Usart0P__Usart__isRxIntrPending(void )
#line 330
{
  if (HplMsp430Usart0P__IFG1 & (1 << 6)) {
      return TRUE;
    }
  return FALSE;
}

# 192 "/home/tinyos/local/src/tinyos-2.x/tos/chips/msp430/usart/HplMsp430Usart.nc"
inline static bool /*Msp430SpiDma0P.SpiP*/Msp430SpiDmaP__0__Usart__isRxIntrPending(void ){
#line 192
  unsigned char result;
#line 192

#line 192
  result = HplMsp430Usart0P__Usart__isRxIntrPending();
#line 192

#line 192
  return result;
#line 192
}
#line 192
# 341 "/home/tinyos/local/src/tinyos-2.x/tos/chips/msp430/usart/HplMsp430Usart0P.nc"
static inline void HplMsp430Usart0P__Usart__clrRxIntr(void )
#line 341
{
  HplMsp430Usart0P__IFG1 &= ~(1 << 6);
}

# 197 "/home/tinyos/local/src/tinyos-2.x/tos/chips/msp430/usart/HplMsp430Usart.nc"
inline static void /*Msp430SpiDma0P.SpiP*/Msp430SpiDmaP__0__Usart__clrRxIntr(void ){
#line 197
  HplMsp430Usart0P__Usart__clrRxIntr();
#line 197
}
#line 197
# 386 "/home/tinyos/local/src/tinyos-2.x/tos/chips/msp430/usart/HplMsp430Usart0P.nc"
static inline uint8_t HplMsp430Usart0P__Usart__rx(void )
#line 386
{
  uint8_t value;

#line 388
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 388
    value = U0RXBUF;
#line 388
    __nesc_atomic_end(__nesc_atomic); }
  return value;
}

# 231 "/home/tinyos/local/src/tinyos-2.x/tos/chips/msp430/usart/HplMsp430Usart.nc"
inline static uint8_t /*Msp430SpiDma0P.SpiP*/Msp430SpiDmaP__0__Usart__rx(void ){
#line 231
  unsigned char result;
#line 231

#line 231
  result = HplMsp430Usart0P__Usart__rx();
#line 231

#line 231
  return result;
#line 231
}
#line 231
# 118 "/home/tinyos/local/src/tinyos-2.x/tos/system/StateImplP.nc"
static inline void StateImplP__State__toIdle(uint8_t id)
#line 118
{
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 119
    StateImplP__state[id] = StateImplP__S_IDLE;
#line 119
    __nesc_atomic_end(__nesc_atomic); }
}

# 56 "/home/tinyos/local/src/tinyos-2.x/tos/interfaces/State.nc"
inline static void CC2420SpiP__WorkingState__toIdle(void ){
#line 56
  StateImplP__State__toIdle(0U);
#line 56
}
#line 56
# 95 "/home/tinyos/local/src/tinyos-2.x/tos/chips/cc2420/spi/CC2420SpiP.nc"
static inline void CC2420SpiP__ChipSpiResource__abortRelease(void )
#line 95
{
  /* atomic removed: atomic calls only */
#line 96
  CC2420SpiP__release = FALSE;
}

# 31 "/home/tinyos/local/src/tinyos-2.x/tos/chips/cc2420/interfaces/ChipSpiResource.nc"
inline static void CC2420TransmitP__ChipSpiResource__abortRelease(void ){
#line 31
  CC2420SpiP__ChipSpiResource__abortRelease();
#line 31
}
#line 31
# 375 "/home/tinyos/local/src/tinyos-2.x/tos/chips/cc2420/transmit/CC2420TransmitP.nc"
static inline void CC2420TransmitP__ChipSpiResource__releasing(void )
#line 375
{
  if (CC2420TransmitP__abortSpiRelease) {
      CC2420TransmitP__ChipSpiResource__abortRelease();
    }
}

# 24 "/home/tinyos/local/src/tinyos-2.x/tos/chips/cc2420/interfaces/ChipSpiResource.nc"
inline static void CC2420SpiP__ChipSpiResource__releasing(void ){
#line 24
  CC2420TransmitP__ChipSpiResource__releasing();
#line 24
}
#line 24
# 205 "/home/tinyos/local/src/tinyos-2.x/tos/system/ArbiterP.nc"
static inline void /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0__ResourceDefaultOwner__default__granted(void )
#line 205
{
}

# 46 "/home/tinyos/local/src/tinyos-2.x/tos/interfaces/ResourceDefaultOwner.nc"
inline static void /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0__ResourceDefaultOwner__granted(void ){
#line 46
  /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0__ResourceDefaultOwner__default__granted();
#line 46
}
#line 46
# 151 "/home/tinyos/local/src/tinyos-2.x/tos/chips/msp430/usart/HplMsp430Usart0P.nc"
static inline void HplMsp430Usart0P__Usart__resetUsart(bool reset)
#line 151
{
  if (reset) {
      U0CTL = 0x01;
    }
  else {
      U0CTL &= ~0x01;
    }
}

# 97 "/home/tinyos/local/src/tinyos-2.x/tos/chips/msp430/usart/HplMsp430Usart.nc"
inline static void /*Msp430SpiDma0P.SpiP*/Msp430SpiDmaP__0__Usart__resetUsart(bool reset){
#line 97
  HplMsp430Usart0P__Usart__resetUsart(reset);
#line 97
}
#line 97
#line 158
inline static void /*Msp430SpiDma0P.SpiP*/Msp430SpiDmaP__0__Usart__disableSpi(void ){
#line 158
  HplMsp430Usart0P__Usart__disableSpi();
#line 158
}
#line 158
# 92 "/home/tinyos/local/src/tinyos-2.x/tos/chips/msp430/usart/Msp430SpiDmaP.nc"
static inline void /*Msp430SpiDma0P.SpiP*/Msp430SpiDmaP__0__ResourceConfigure__unconfigure(uint8_t id)
#line 92
{
  /*Msp430SpiDma0P.SpiP*/Msp430SpiDmaP__0__Usart__resetUsart(TRUE);
  /*Msp430SpiDma0P.SpiP*/Msp430SpiDmaP__0__Usart__disableSpi();
  /*Msp430SpiDma0P.SpiP*/Msp430SpiDmaP__0__Usart__resetUsart(FALSE);
}

# 215 "/home/tinyos/local/src/tinyos-2.x/tos/system/ArbiterP.nc"
static inline void /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0__ResourceConfigure__default__unconfigure(uint8_t id)
#line 215
{
}

# 55 "/home/tinyos/local/src/tinyos-2.x/tos/interfaces/ResourceConfigure.nc"
inline static void /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0__ResourceConfigure__unconfigure(uint8_t arg_0x40e31148){
#line 55
  switch (arg_0x40e31148) {
#line 55
    case /*CC2420SpiWireC.HplCC2420SpiC.SpiC.UsartC*/Msp430Usart0C__0__CLIENT_ID:
#line 55
      /*Msp430SpiDma0P.SpiP*/Msp430SpiDmaP__0__ResourceConfigure__unconfigure(/*CC2420SpiWireC.HplCC2420SpiC.SpiC*/Msp430Spi0C__0__CLIENT_ID);
#line 55
      break;
#line 55
    default:
#line 55
      /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0__ResourceConfigure__default__unconfigure(arg_0x40e31148);
#line 55
      break;
#line 55
    }
#line 55
}
#line 55
# 56 "/home/tinyos/local/src/tinyos-2.x/tos/interfaces/TaskBasic.nc"
inline static error_t /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0__grantedTask__postTask(void ){
#line 56
  unsigned char result;
#line 56

#line 56
  result = SchedulerBasicP__TaskBasic__postTask(/*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0__grantedTask);
#line 56

#line 56
  return result;
#line 56
}
#line 56
# 58 "/home/tinyos/local/src/tinyos-2.x/tos/system/FcfsResourceQueueC.nc"
static inline resource_client_id_t /*Msp430UsartShare0P.ArbiterC.Queue*/FcfsResourceQueueC__1__FcfsQueue__dequeue(void )
#line 58
{
  /* atomic removed: atomic calls only */
#line 59
  {
    if (/*Msp430UsartShare0P.ArbiterC.Queue*/FcfsResourceQueueC__1__qHead != /*Msp430UsartShare0P.ArbiterC.Queue*/FcfsResourceQueueC__1__NO_ENTRY) {
        uint8_t id = /*Msp430UsartShare0P.ArbiterC.Queue*/FcfsResourceQueueC__1__qHead;

#line 62
        /*Msp430UsartShare0P.ArbiterC.Queue*/FcfsResourceQueueC__1__qHead = /*Msp430UsartShare0P.ArbiterC.Queue*/FcfsResourceQueueC__1__resQ[/*Msp430UsartShare0P.ArbiterC.Queue*/FcfsResourceQueueC__1__qHead];
        if (/*Msp430UsartShare0P.ArbiterC.Queue*/FcfsResourceQueueC__1__qHead == /*Msp430UsartShare0P.ArbiterC.Queue*/FcfsResourceQueueC__1__NO_ENTRY) {
          /*Msp430UsartShare0P.ArbiterC.Queue*/FcfsResourceQueueC__1__qTail = /*Msp430UsartShare0P.ArbiterC.Queue*/FcfsResourceQueueC__1__NO_ENTRY;
          }
#line 65
        /*Msp430UsartShare0P.ArbiterC.Queue*/FcfsResourceQueueC__1__resQ[id] = /*Msp430UsartShare0P.ArbiterC.Queue*/FcfsResourceQueueC__1__NO_ENTRY;
        {
          unsigned char __nesc_temp = 
#line 66
          id;

#line 66
          return __nesc_temp;
        }
      }
#line 68
    {
      unsigned char __nesc_temp = 
#line 68
      /*Msp430UsartShare0P.ArbiterC.Queue*/FcfsResourceQueueC__1__NO_ENTRY;

#line 68
      return __nesc_temp;
    }
  }
}

# 60 "/home/tinyos/local/src/tinyos-2.x/tos/interfaces/ResourceQueue.nc"
inline static resource_client_id_t /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0__Queue__dequeue(void ){
#line 60
  unsigned char result;
#line 60

#line 60
  result = /*Msp430UsartShare0P.ArbiterC.Queue*/FcfsResourceQueueC__1__FcfsQueue__dequeue();
#line 60

#line 60
  return result;
#line 60
}
#line 60
# 50 "/home/tinyos/local/src/tinyos-2.x/tos/system/FcfsResourceQueueC.nc"
static inline bool /*Msp430UsartShare0P.ArbiterC.Queue*/FcfsResourceQueueC__1__FcfsQueue__isEmpty(void )
#line 50
{
  /* atomic removed: atomic calls only */
#line 51
  {
    unsigned char __nesc_temp = 
#line 51
    /*Msp430UsartShare0P.ArbiterC.Queue*/FcfsResourceQueueC__1__qHead == /*Msp430UsartShare0P.ArbiterC.Queue*/FcfsResourceQueueC__1__NO_ENTRY;

#line 51
    return __nesc_temp;
  }
}

# 43 "/home/tinyos/local/src/tinyos-2.x/tos/interfaces/ResourceQueue.nc"
inline static bool /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0__Queue__isEmpty(void ){
#line 43
  unsigned char result;
#line 43

#line 43
  result = /*Msp430UsartShare0P.ArbiterC.Queue*/FcfsResourceQueueC__1__FcfsQueue__isEmpty();
#line 43

#line 43
  return result;
#line 43
}
#line 43
# 108 "/home/tinyos/local/src/tinyos-2.x/tos/system/ArbiterP.nc"
static inline error_t /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0__Resource__release(uint8_t id)
#line 108
{
  /* atomic removed: atomic calls only */
#line 109
  {
    if (/*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0__state == /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0__RES_BUSY && /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0__resId == id) {
        if (/*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0__Queue__isEmpty() == FALSE) {
            /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0__reqResId = /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0__Queue__dequeue();
            /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0__resId = /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0__NO_RES;
            /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0__state = /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0__RES_GRANTING;
            /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0__grantedTask__postTask();
            /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0__ResourceConfigure__unconfigure(id);
          }
        else {
            /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0__resId = /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0__default_owner_id;
            /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0__state = /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0__RES_CONTROLLED;
            /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0__ResourceConfigure__unconfigure(id);
            /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0__ResourceDefaultOwner__granted();
          }
        {
          unsigned char __nesc_temp = 
#line 124
          SUCCESS;

#line 124
          return __nesc_temp;
        }
      }
  }
#line 127
  return FAIL;
}

# 109 "/home/tinyos/local/src/tinyos-2.x/tos/chips/msp430/usart/Msp430SpiDmaP.nc"
static inline error_t /*Msp430SpiDma0P.SpiP*/Msp430SpiDmaP__0__UsartResource__default__release(uint8_t id)
#line 109
{
#line 109
  return FAIL;
}

# 110 "/home/tinyos/local/src/tinyos-2.x/tos/interfaces/Resource.nc"
inline static error_t /*Msp430SpiDma0P.SpiP*/Msp430SpiDmaP__0__UsartResource__release(uint8_t arg_0x40c6d9a0){
#line 110
  unsigned char result;
#line 110

#line 110
  switch (arg_0x40c6d9a0) {
#line 110
    case /*CC2420SpiWireC.HplCC2420SpiC.SpiC*/Msp430Spi0C__0__CLIENT_ID:
#line 110
      result = /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0__Resource__release(/*CC2420SpiWireC.HplCC2420SpiC.SpiC.UsartC*/Msp430Usart0C__0__CLIENT_ID);
#line 110
      break;
#line 110
    default:
#line 110
      result = /*Msp430SpiDma0P.SpiP*/Msp430SpiDmaP__0__UsartResource__default__release(arg_0x40c6d9a0);
#line 110
      break;
#line 110
    }
#line 110

#line 110
  return result;
#line 110
}
#line 110
# 84 "/home/tinyos/local/src/tinyos-2.x/tos/chips/msp430/usart/Msp430SpiDmaP.nc"
static inline error_t /*Msp430SpiDma0P.SpiP*/Msp430SpiDmaP__0__Resource__release(uint8_t id)
#line 84
{
  return /*Msp430SpiDma0P.SpiP*/Msp430SpiDmaP__0__UsartResource__release(id);
}

# 110 "/home/tinyos/local/src/tinyos-2.x/tos/interfaces/Resource.nc"
inline static error_t CC2420SpiP__SpiResource__release(void ){
#line 110
  unsigned char result;
#line 110

#line 110
  result = /*Msp430SpiDma0P.SpiP*/Msp430SpiDmaP__0__Resource__release(/*CC2420SpiWireC.HplCC2420SpiC.SpiC*/Msp430Spi0C__0__CLIENT_ID);
#line 110

#line 110
  return result;
#line 110
}
#line 110
# 56 "/home/tinyos/local/src/tinyos-2.x/tos/chips/msp430/pins/HplMsp430GeneralIOP.nc"
static inline void /*HplMsp430GeneralIOC.P31*/HplMsp430GeneralIOP__17__IO__selectIOFunc(void )
#line 56
{
  /* atomic removed: atomic calls only */
#line 56
  * (volatile uint8_t * )27U &= ~(0x01 << 1);
}

# 85 "/home/tinyos/local/src/tinyos-2.x/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
inline static void HplMsp430Usart0P__SIMO__selectIOFunc(void ){
#line 85
  /*HplMsp430GeneralIOC.P31*/HplMsp430GeneralIOP__17__IO__selectIOFunc();
#line 85
}
#line 85
# 56 "/home/tinyos/local/src/tinyos-2.x/tos/chips/msp430/pins/HplMsp430GeneralIOP.nc"
static inline void /*HplMsp430GeneralIOC.P32*/HplMsp430GeneralIOP__18__IO__selectIOFunc(void )
#line 56
{
  /* atomic removed: atomic calls only */
#line 56
  * (volatile uint8_t * )27U &= ~(0x01 << 2);
}

# 85 "/home/tinyos/local/src/tinyos-2.x/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
inline static void HplMsp430Usart0P__SOMI__selectIOFunc(void ){
#line 85
  /*HplMsp430GeneralIOC.P32*/HplMsp430GeneralIOP__18__IO__selectIOFunc();
#line 85
}
#line 85
# 56 "/home/tinyos/local/src/tinyos-2.x/tos/chips/msp430/pins/HplMsp430GeneralIOP.nc"
static inline void /*HplMsp430GeneralIOC.P33*/HplMsp430GeneralIOP__19__IO__selectIOFunc(void )
#line 56
{
  /* atomic removed: atomic calls only */
#line 56
  * (volatile uint8_t * )27U &= ~(0x01 << 3);
}

# 85 "/home/tinyos/local/src/tinyos-2.x/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
inline static void HplMsp430Usart0P__UCLK__selectIOFunc(void ){
#line 85
  /*HplMsp430GeneralIOC.P33*/HplMsp430GeneralIOP__19__IO__selectIOFunc();
#line 85
}
#line 85
# 119 "/home/tinyos/local/src/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static inline void /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__Control__enableEvents(void )
{
  * (volatile uint16_t * )390U |= 0x0010;
}

# 46 "/home/tinyos/local/src/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerControl.nc"
inline static void /*AlarmMultiplexC.Alarm.Alarm32khz32C.AlarmC.Msp430Alarm*/Msp430AlarmC__1__Msp430TimerControl__enableEvents(void ){
#line 46
  /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__Control__enableEvents();
#line 46
}
#line 46
# 84 "/home/tinyos/local/src/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static inline void /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__Control__clearPendingInterrupt(void )
{
  * (volatile uint16_t * )390U &= ~0x0001;
}

# 33 "/home/tinyos/local/src/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerControl.nc"
inline static void /*AlarmMultiplexC.Alarm.Alarm32khz32C.AlarmC.Msp430Alarm*/Msp430AlarmC__1__Msp430TimerControl__clearPendingInterrupt(void ){
#line 33
  /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__Control__clearPendingInterrupt();
#line 33
}
#line 33
# 144 "/home/tinyos/local/src/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static inline void /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__Compare__setEvent(uint16_t x)
{
  * (volatile uint16_t * )406U = x;
}

# 30 "/home/tinyos/local/src/tinyos-2.x/tos/chips/msp430/timer/Msp430Compare.nc"
inline static void /*AlarmMultiplexC.Alarm.Alarm32khz32C.AlarmC.Msp430Alarm*/Msp430AlarmC__1__Msp430Compare__setEvent(uint16_t time){
#line 30
  /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__Compare__setEvent(time);
#line 30
}
#line 30
# 34 "/home/tinyos/local/src/tinyos-2.x/tos/chips/msp430/timer/Msp430Timer.nc"
inline static uint16_t /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__Timer__get(void ){
#line 34
  unsigned int result;
#line 34

#line 34
  result = /*Msp430TimerC.Msp430TimerB*/Msp430TimerP__1__Timer__get();
#line 34

#line 34
  return result;
#line 34
}
#line 34
# 154 "/home/tinyos/local/src/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static inline void /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__Compare__setEventFromNow(uint16_t x)
{
  * (volatile uint16_t * )406U = /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__Timer__get() + x;
}

# 32 "/home/tinyos/local/src/tinyos-2.x/tos/chips/msp430/timer/Msp430Compare.nc"
inline static void /*AlarmMultiplexC.Alarm.Alarm32khz32C.AlarmC.Msp430Alarm*/Msp430AlarmC__1__Msp430Compare__setEventFromNow(uint16_t delta){
#line 32
  /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__Compare__setEventFromNow(delta);
#line 32
}
#line 32
# 34 "/home/tinyos/local/src/tinyos-2.x/tos/chips/msp430/timer/Msp430Timer.nc"
inline static uint16_t /*AlarmMultiplexC.Alarm.Alarm32khz32C.AlarmC.Msp430Alarm*/Msp430AlarmC__1__Msp430Timer__get(void ){
#line 34
  unsigned int result;
#line 34

#line 34
  result = /*Msp430TimerC.Msp430TimerB*/Msp430TimerP__1__Timer__get();
#line 34

#line 34
  return result;
#line 34
}
#line 34
# 70 "/home/tinyos/local/src/tinyos-2.x/tos/chips/msp430/timer/Msp430AlarmC.nc"
static inline void /*AlarmMultiplexC.Alarm.Alarm32khz32C.AlarmC.Msp430Alarm*/Msp430AlarmC__1__Alarm__startAt(uint16_t t0, uint16_t dt)
{
  /* atomic removed: atomic calls only */
  {
    uint16_t now = /*AlarmMultiplexC.Alarm.Alarm32khz32C.AlarmC.Msp430Alarm*/Msp430AlarmC__1__Msp430Timer__get();
    uint16_t elapsed = now - t0;

#line 76
    if (elapsed >= dt) 
      {
        /*AlarmMultiplexC.Alarm.Alarm32khz32C.AlarmC.Msp430Alarm*/Msp430AlarmC__1__Msp430Compare__setEventFromNow(2);
      }
    else 
      {
        uint16_t remaining = dt - elapsed;

#line 83
        if (remaining <= 2) {
          /*AlarmMultiplexC.Alarm.Alarm32khz32C.AlarmC.Msp430Alarm*/Msp430AlarmC__1__Msp430Compare__setEventFromNow(2);
          }
        else {
#line 86
          /*AlarmMultiplexC.Alarm.Alarm32khz32C.AlarmC.Msp430Alarm*/Msp430AlarmC__1__Msp430Compare__setEvent(now + remaining);
          }
      }
#line 88
    /*AlarmMultiplexC.Alarm.Alarm32khz32C.AlarmC.Msp430Alarm*/Msp430AlarmC__1__Msp430TimerControl__clearPendingInterrupt();
    /*AlarmMultiplexC.Alarm.Alarm32khz32C.AlarmC.Msp430Alarm*/Msp430AlarmC__1__Msp430TimerControl__enableEvents();
  }
}

# 92 "/home/tinyos/local/src/tinyos-2.x/tos/lib/timer/Alarm.nc"
inline static void /*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC__1__AlarmFrom__startAt(/*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC__1__AlarmFrom__size_type t0, /*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC__1__AlarmFrom__size_type dt){
#line 92
  /*AlarmMultiplexC.Alarm.Alarm32khz32C.AlarmC.Msp430Alarm*/Msp430AlarmC__1__Alarm__startAt(t0, dt);
#line 92
}
#line 92
# 102 "/home/tinyos/local/src/tinyos-2.x/tos/chips/cc2420/spi/CC2420SpiP.nc"
static inline error_t CC2420SpiP__ChipSpiResource__attemptRelease(void )
#line 102
{
  return CC2420SpiP__attemptRelease();
}

# 39 "/home/tinyos/local/src/tinyos-2.x/tos/chips/cc2420/interfaces/ChipSpiResource.nc"
inline static error_t CC2420TransmitP__ChipSpiResource__attemptRelease(void ){
#line 39
  unsigned char result;
#line 39

#line 39
  result = CC2420SpiP__ChipSpiResource__attemptRelease();
#line 39

#line 39
  return result;
#line 39
}
#line 39
# 78 "/home/tinyos/local/src/tinyos-2.x/tos/interfaces/Resource.nc"
inline static error_t CC2420ControlP__SpiResource__request(void ){
#line 78
  unsigned char result;
#line 78

#line 78
  result = CC2420SpiP__Resource__request(/*CC2420ControlC.Spi*/CC2420SpiC__0__CLIENT_ID);
#line 78

#line 78
  return result;
#line 78
}
#line 78
# 171 "/home/tinyos/local/src/tinyos-2.x/tos/chips/cc2420/control/CC2420ControlP.nc"
static inline error_t CC2420ControlP__Resource__request(void )
#line 171
{
  return CC2420ControlP__SpiResource__request();
}

# 78 "/home/tinyos/local/src/tinyos-2.x/tos/interfaces/Resource.nc"
inline static error_t CC2420CsmaP__Resource__request(void ){
#line 78
  unsigned char result;
#line 78

#line 78
  result = CC2420ControlP__Resource__request();
#line 78

#line 78
  return result;
#line 78
}
#line 78
# 207 "/home/tinyos/local/src/tinyos-2.x/tos/chips/cc2420/csma/CC2420CsmaP.nc"
static inline void CC2420CsmaP__CC2420Power__startVRegDone(void )
#line 207
{
  CC2420CsmaP__Resource__request();
}

# 56 "/home/tinyos/local/src/tinyos-2.x/tos/chips/cc2420/interfaces/CC2420Power.nc"
inline static void CC2420ControlP__CC2420Power__startVRegDone(void ){
#line 56
  CC2420CsmaP__CC2420Power__startVRegDone();
#line 56
}
#line 56
# 34 "/home/tinyos/local/src/tinyos-2.x/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
inline static void /*HplCC2420PinsC.RSTNM*/Msp430GpioC__7__HplGeneralIO__set(void ){
#line 34
  /*HplMsp430GeneralIOC.P46*/HplMsp430GeneralIOP__30__IO__set();
#line 34
}
#line 34
# 37 "/home/tinyos/local/src/tinyos-2.x/tos/chips/msp430/pins/Msp430GpioC.nc"
static inline void /*HplCC2420PinsC.RSTNM*/Msp430GpioC__7__GeneralIO__set(void )
#line 37
{
#line 37
  /*HplCC2420PinsC.RSTNM*/Msp430GpioC__7__HplGeneralIO__set();
}

# 29 "/home/tinyos/local/src/tinyos-2.x/tos/interfaces/GeneralIO.nc"
inline static void CC2420ControlP__RSTN__set(void ){
#line 29
  /*HplCC2420PinsC.RSTNM*/Msp430GpioC__7__GeneralIO__set();
#line 29
}
#line 29
# 39 "/home/tinyos/local/src/tinyos-2.x/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
inline static void /*HplCC2420PinsC.RSTNM*/Msp430GpioC__7__HplGeneralIO__clr(void ){
#line 39
  /*HplMsp430GeneralIOC.P46*/HplMsp430GeneralIOP__30__IO__clr();
#line 39
}
#line 39
# 38 "/home/tinyos/local/src/tinyos-2.x/tos/chips/msp430/pins/Msp430GpioC.nc"
static inline void /*HplCC2420PinsC.RSTNM*/Msp430GpioC__7__GeneralIO__clr(void )
#line 38
{
#line 38
  /*HplCC2420PinsC.RSTNM*/Msp430GpioC__7__HplGeneralIO__clr();
}

# 30 "/home/tinyos/local/src/tinyos-2.x/tos/interfaces/GeneralIO.nc"
inline static void CC2420ControlP__RSTN__clr(void ){
#line 30
  /*HplCC2420PinsC.RSTNM*/Msp430GpioC__7__GeneralIO__clr();
#line 30
}
#line 30
# 408 "/home/tinyos/local/src/tinyos-2.x/tos/chips/cc2420/control/CC2420ControlP.nc"
static inline void CC2420ControlP__StartupTimer__fired(void )
#line 408
{
  if (CC2420ControlP__m_state == CC2420ControlP__S_VREG_STARTING) {
      CC2420ControlP__m_state = CC2420ControlP__S_VREG_STARTED;
      CC2420ControlP__RSTN__clr();
      CC2420ControlP__RSTN__set();
      CC2420ControlP__CC2420Power__startVRegDone();
    }
}

# 45 "/home/tinyos/local/src/tinyos-2.x/tos/chips/cc2420/interfaces/CC2420Strobe.nc"
inline static cc2420_status_t CC2420TransmitP__SFLUSHTX__strobe(void ){
#line 45
  unsigned char result;
#line 45

#line 45
  result = CC2420SpiP__Strobe__strobe(CC2420_SFLUSHTX);
#line 45

#line 45
  return result;
#line 45
}
#line 45
# 48 "/home/tinyos/local/src/tinyos-2.x/tos/chips/msp430/pins/HplMsp430GeneralIOP.nc"
static inline uint8_t /*HplMsp430GeneralIOC.P14*/HplMsp430GeneralIOP__4__IO__getRaw(void )
#line 48
{
#line 48
  return * (volatile uint8_t * )32U & (0x01 << 4);
}

#line 49
static inline bool /*HplMsp430GeneralIOC.P14*/HplMsp430GeneralIOP__4__IO__get(void )
#line 49
{
#line 49
  return /*HplMsp430GeneralIOC.P14*/HplMsp430GeneralIOP__4__IO__getRaw() != 0;
}

# 59 "/home/tinyos/local/src/tinyos-2.x/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
inline static bool /*HplCC2420PinsC.CCAM*/Msp430GpioC__3__HplGeneralIO__get(void ){
#line 59
  unsigned char result;
#line 59

#line 59
  result = /*HplMsp430GeneralIOC.P14*/HplMsp430GeneralIOP__4__IO__get();
#line 59

#line 59
  return result;
#line 59
}
#line 59
# 40 "/home/tinyos/local/src/tinyos-2.x/tos/chips/msp430/pins/Msp430GpioC.nc"
static inline bool /*HplCC2420PinsC.CCAM*/Msp430GpioC__3__GeneralIO__get(void )
#line 40
{
#line 40
  return /*HplCC2420PinsC.CCAM*/Msp430GpioC__3__HplGeneralIO__get();
}

# 32 "/home/tinyos/local/src/tinyos-2.x/tos/interfaces/GeneralIO.nc"
inline static bool CC2420TransmitP__CCA__get(void ){
#line 32
  unsigned char result;
#line 32

#line 32
  result = /*HplCC2420PinsC.CCAM*/Msp430GpioC__3__GeneralIO__get();
#line 32

#line 32
  return result;
#line 32
}
#line 32
# 496 "/home/tinyos/local/src/tinyos-2.x/tos/chips/cc2420/transmit/CC2420TransmitP.nc"
static inline void CC2420TransmitP__BackoffTimer__fired(void )
#line 496
{
  /* atomic removed: atomic calls only */
#line 497
  {
    switch (CC2420TransmitP__m_state) {

        case CC2420TransmitP__S_SAMPLE_CCA: 


          if (CC2420TransmitP__CCA__get()) {
              CC2420TransmitP__m_state = CC2420TransmitP__S_BEGIN_TRANSMIT;
              CC2420TransmitP__BackoffTimer__start(CC2420_TIME_ACK_TURNAROUND);
            }
          else {
              CC2420TransmitP__congestionBackoff();
            }
        break;

        case CC2420TransmitP__S_BEGIN_TRANSMIT: 
          case CC2420TransmitP__S_CANCEL: 
            if (CC2420TransmitP__acquireSpiResource() == SUCCESS) {
                CC2420TransmitP__attemptSend();
              }
        break;

        case CC2420TransmitP__S_ACK_WAIT: 
          CC2420TransmitP__signalDone(SUCCESS);
        break;

        case CC2420TransmitP__S_SFD: 


          CC2420TransmitP__SFLUSHTX__strobe();
        CC2420TransmitP__CaptureSFD__captureRisingEdge();
        CC2420TransmitP__releaseSpiResource();
        CC2420TransmitP__signalDone(ERETRY);
        break;

        default: 
          break;
      }
  }
}

# 67 "/home/tinyos/local/src/tinyos-2.x/tos/lib/timer/Alarm.nc"
inline static void /*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC__1__Alarm__fired(void ){
#line 67
  CC2420TransmitP__BackoffTimer__fired();
#line 67
  CC2420ControlP__StartupTimer__fired();
#line 67
}
#line 67
# 151 "/home/tinyos/local/src/tinyos-2.x/tos/lib/timer/TransformAlarmC.nc"
static inline void /*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC__1__AlarmFrom__fired(void )
{
  /* atomic removed: atomic calls only */
  {
    if (/*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC__1__m_dt == 0) 
      {
        /*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC__1__Alarm__fired();
      }
    else 
      {
        /*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC__1__set_alarm();
      }
  }
}

# 67 "/home/tinyos/local/src/tinyos-2.x/tos/lib/timer/Alarm.nc"
inline static void /*AlarmMultiplexC.Alarm.Alarm32khz32C.AlarmC.Msp430Alarm*/Msp430AlarmC__1__Alarm__fired(void ){
#line 67
  /*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC__1__AlarmFrom__fired();
#line 67
}
#line 67
# 59 "/home/tinyos/local/src/tinyos-2.x/tos/chips/msp430/timer/Msp430AlarmC.nc"
static inline void /*AlarmMultiplexC.Alarm.Alarm32khz32C.AlarmC.Msp430Alarm*/Msp430AlarmC__1__Msp430Compare__fired(void )
{
  /*AlarmMultiplexC.Alarm.Alarm32khz32C.AlarmC.Msp430Alarm*/Msp430AlarmC__1__Msp430TimerControl__disableEvents();
  /*AlarmMultiplexC.Alarm.Alarm32khz32C.AlarmC.Msp430Alarm*/Msp430AlarmC__1__Alarm__fired();
}

# 34 "/home/tinyos/local/src/tinyos-2.x/tos/chips/msp430/timer/Msp430Compare.nc"
inline static void /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__Compare__fired(void ){
#line 34
  /*AlarmMultiplexC.Alarm.Alarm32khz32C.AlarmC.Msp430Alarm*/Msp430AlarmC__1__Msp430Compare__fired();
#line 34
}
#line 34
# 139 "/home/tinyos/local/src/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static inline uint16_t /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__Capture__getEvent(void )
{
  return * (volatile uint16_t * )406U;
}

#line 177
static inline void /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__Capture__default__captured(uint16_t n)
{
}

# 75 "/home/tinyos/local/src/tinyos-2.x/tos/chips/msp430/timer/Msp430Capture.nc"
inline static void /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__Capture__captured(uint16_t time){
#line 75
  /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__Capture__default__captured(time);
#line 75
}
#line 75
# 47 "/home/tinyos/local/src/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static inline  /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__cc_t /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__int2CC(uint16_t x)
#line 47
{
#line 47
  union /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5____nesc_unnamed4452 {
#line 47
    uint16_t f;
#line 47
    /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__cc_t t;
  } 
#line 47
  c = { .f = x };

#line 47
  return c.t;
}

#line 74
static inline /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__cc_t /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__Control__getControl(void )
{
  return /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__int2CC(* (volatile uint16_t * )390U);
}

#line 169
static inline void /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__Event__fired(void )
{
  if (/*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__Control__getControl().cap) {
    /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__Capture__captured(/*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__Capture__getEvent());
    }
  else {
#line 174
    /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__Compare__fired();
    }
}

# 288 "/home/tinyos/local/src/tinyos-2.x/tos/chips/cc2420/csma/CC2420CsmaP.nc"
static inline void CC2420CsmaP__RadioBackoff__default__requestCongestionBackoff(message_t *msg)
#line 288
{
}

# 88 "/home/tinyos/local/src/tinyos-2.x/tos/chips/cc2420/interfaces/RadioBackoff.nc"
inline static void CC2420CsmaP__RadioBackoff__requestCongestionBackoff(message_t * msg){
#line 88
  CC2420CsmaP__RadioBackoff__default__requestCongestionBackoff(msg);
#line 88
}
#line 88
# 78 "/home/tinyos/local/src/tinyos-2.x/tos/system/RandomMlcgC.nc"
static inline uint16_t RandomMlcgC__Random__rand16(void )
#line 78
{
  return (uint16_t )RandomMlcgC__Random__rand32();
}

# 41 "/home/tinyos/local/src/tinyos-2.x/tos/interfaces/Random.nc"
inline static uint16_t CC2420CsmaP__Random__rand16(void ){
#line 41
  unsigned int result;
#line 41

#line 41
  result = RandomMlcgC__Random__rand16();
#line 41

#line 41
  return result;
#line 41
}
#line 41
# 251 "/home/tinyos/local/src/tinyos-2.x/tos/chips/cc2420/transmit/CC2420TransmitP.nc"
static inline void CC2420TransmitP__RadioBackoff__setCongestionBackoff(uint16_t backoffTime)
#line 251
{
  CC2420TransmitP__myCongestionBackoff = backoffTime + 1;
}

# 66 "/home/tinyos/local/src/tinyos-2.x/tos/chips/cc2420/interfaces/RadioBackoff.nc"
inline static void CC2420CsmaP__SubBackoff__setCongestionBackoff(uint16_t backoffTime){
#line 66
  CC2420TransmitP__RadioBackoff__setCongestionBackoff(backoffTime);
#line 66
}
#line 66
# 227 "/home/tinyos/local/src/tinyos-2.x/tos/chips/cc2420/csma/CC2420CsmaP.nc"
static inline void CC2420CsmaP__SubBackoff__requestCongestionBackoff(message_t *msg)
#line 227
{
  CC2420CsmaP__SubBackoff__setCongestionBackoff(CC2420CsmaP__Random__rand16()
   % (0x7 * CC2420_BACKOFF_PERIOD) + CC2420_MIN_BACKOFF);

  CC2420CsmaP__RadioBackoff__requestCongestionBackoff(msg);
}

# 88 "/home/tinyos/local/src/tinyos-2.x/tos/chips/cc2420/interfaces/RadioBackoff.nc"
inline static void CC2420TransmitP__RadioBackoff__requestCongestionBackoff(message_t * msg){
#line 88
  CC2420CsmaP__SubBackoff__requestCongestionBackoff(msg);
#line 88
}
#line 88
# 87 "/home/tinyos/local/src/tinyos-2.x/tos/interfaces/Resource.nc"
inline static error_t CC2420TransmitP__SpiResource__immediateRequest(void ){
#line 87
  unsigned char result;
#line 87

#line 87
  result = CC2420SpiP__Resource__immediateRequest(/*CC2420TransmitC.Spi*/CC2420SpiC__3__CLIENT_ID);
#line 87

#line 87
  return result;
#line 87
}
#line 87
# 45 "/home/tinyos/local/src/tinyos-2.x/tos/interfaces/State.nc"
inline static error_t CC2420SpiP__WorkingState__requestState(uint8_t reqState){
#line 45
  unsigned char result;
#line 45

#line 45
  result = StateImplP__State__requestState(0U, reqState);
#line 45

#line 45
  return result;
#line 45
}
#line 45
# 106 "/home/tinyos/local/src/tinyos-2.x/tos/chips/msp430/usart/Msp430SpiDmaP.nc"
static inline error_t /*Msp430SpiDma0P.SpiP*/Msp430SpiDmaP__0__UsartResource__default__isOwner(uint8_t id)
#line 106
{
#line 106
  return FAIL;
}

# 118 "/home/tinyos/local/src/tinyos-2.x/tos/interfaces/Resource.nc"
inline static bool /*Msp430SpiDma0P.SpiP*/Msp430SpiDmaP__0__UsartResource__isOwner(uint8_t arg_0x40c6d9a0){
#line 118
  unsigned char result;
#line 118

#line 118
  switch (arg_0x40c6d9a0) {
#line 118
    case /*CC2420SpiWireC.HplCC2420SpiC.SpiC*/Msp430Spi0C__0__CLIENT_ID:
#line 118
      result = /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0__Resource__isOwner(/*CC2420SpiWireC.HplCC2420SpiC.SpiC.UsartC*/Msp430Usart0C__0__CLIENT_ID);
#line 118
      break;
#line 118
    default:
#line 118
      result = /*Msp430SpiDma0P.SpiP*/Msp430SpiDmaP__0__UsartResource__default__isOwner(arg_0x40c6d9a0);
#line 118
      break;
#line 118
    }
#line 118

#line 118
  return result;
#line 118
}
#line 118
# 102 "/home/tinyos/local/src/tinyos-2.x/tos/chips/msp430/usart/Msp430SpiDmaP.nc"
static inline uint8_t /*Msp430SpiDma0P.SpiP*/Msp430SpiDmaP__0__Resource__isOwner(uint8_t id)
#line 102
{
  return /*Msp430SpiDma0P.SpiP*/Msp430SpiDmaP__0__UsartResource__isOwner(id);
}

# 118 "/home/tinyos/local/src/tinyos-2.x/tos/interfaces/Resource.nc"
inline static bool CC2420SpiP__SpiResource__isOwner(void ){
#line 118
  unsigned char result;
#line 118

#line 118
  result = /*Msp430SpiDma0P.SpiP*/Msp430SpiDmaP__0__Resource__isOwner(/*CC2420SpiWireC.HplCC2420SpiC.SpiC*/Msp430Spi0C__0__CLIENT_ID);
#line 118

#line 118
  return result;
#line 118
}
#line 118
# 110 "/home/tinyos/local/src/tinyos-2.x/tos/chips/msp430/usart/Msp430SpiDmaP.nc"
static inline msp430_spi_union_config_t */*Msp430SpiDma0P.SpiP*/Msp430SpiDmaP__0__Msp430SpiConfigure__default__getConfig(uint8_t id)
#line 110
{
  return &msp430_spi_default_config;
}

# 39 "/home/tinyos/local/src/tinyos-2.x/tos/chips/msp430/usart/Msp430SpiConfigure.nc"
inline static msp430_spi_union_config_t */*Msp430SpiDma0P.SpiP*/Msp430SpiDmaP__0__Msp430SpiConfigure__getConfig(uint8_t arg_0x40c6c418){
#line 39
  union __nesc_unnamed4319 *result;
#line 39

#line 39
    result = /*Msp430SpiDma0P.SpiP*/Msp430SpiDmaP__0__Msp430SpiConfigure__default__getConfig(arg_0x40c6c418);
#line 39

#line 39
  return result;
#line 39
}
#line 39
# 168 "/home/tinyos/local/src/tinyos-2.x/tos/chips/msp430/usart/HplMsp430Usart.nc"
inline static void /*Msp430SpiDma0P.SpiP*/Msp430SpiDmaP__0__Usart__setModeSpi(msp430_spi_union_config_t *config){
#line 168
  HplMsp430Usart0P__Usart__setModeSpi(config);
#line 168
}
#line 168
# 88 "/home/tinyos/local/src/tinyos-2.x/tos/chips/msp430/usart/Msp430SpiDmaP.nc"
static inline void /*Msp430SpiDma0P.SpiP*/Msp430SpiDmaP__0__ResourceConfigure__configure(uint8_t id)
#line 88
{
  /*Msp430SpiDma0P.SpiP*/Msp430SpiDmaP__0__Usart__setModeSpi(/*Msp430SpiDma0P.SpiP*/Msp430SpiDmaP__0__Msp430SpiConfigure__getConfig(id));
}

# 213 "/home/tinyos/local/src/tinyos-2.x/tos/system/ArbiterP.nc"
static inline void /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0__ResourceConfigure__default__configure(uint8_t id)
#line 213
{
}

# 49 "/home/tinyos/local/src/tinyos-2.x/tos/interfaces/ResourceConfigure.nc"
inline static void /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0__ResourceConfigure__configure(uint8_t arg_0x40e31148){
#line 49
  switch (arg_0x40e31148) {
#line 49
    case /*CC2420SpiWireC.HplCC2420SpiC.SpiC.UsartC*/Msp430Usart0C__0__CLIENT_ID:
#line 49
      /*Msp430SpiDma0P.SpiP*/Msp430SpiDmaP__0__ResourceConfigure__configure(/*CC2420SpiWireC.HplCC2420SpiC.SpiC*/Msp430Spi0C__0__CLIENT_ID);
#line 49
      break;
#line 49
    default:
#line 49
      /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0__ResourceConfigure__default__configure(arg_0x40e31148);
#line 49
      break;
#line 49
    }
#line 49
}
#line 49
# 210 "/home/tinyos/local/src/tinyos-2.x/tos/system/ArbiterP.nc"
static inline void /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0__ResourceDefaultOwner__default__immediateRequested(void )
#line 210
{
  /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0__ResourceDefaultOwner__release();
}

# 81 "/home/tinyos/local/src/tinyos-2.x/tos/interfaces/ResourceDefaultOwner.nc"
inline static void /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0__ResourceDefaultOwner__immediateRequested(void ){
#line 81
  /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0__ResourceDefaultOwner__default__immediateRequested();
#line 81
}
#line 81
# 203 "/home/tinyos/local/src/tinyos-2.x/tos/system/ArbiterP.nc"
static inline void /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0__ResourceRequested__default__immediateRequested(uint8_t id)
#line 203
{
}

# 51 "/home/tinyos/local/src/tinyos-2.x/tos/interfaces/ResourceRequested.nc"
inline static void /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0__ResourceRequested__immediateRequested(uint8_t arg_0x40e33d38){
#line 51
    /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0__ResourceRequested__default__immediateRequested(arg_0x40e33d38);
#line 51
}
#line 51
# 90 "/home/tinyos/local/src/tinyos-2.x/tos/system/ArbiterP.nc"
static inline error_t /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0__Resource__immediateRequest(uint8_t id)
#line 90
{
  /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0__ResourceRequested__immediateRequested(/*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0__resId);
  /* atomic removed: atomic calls only */
#line 92
  {
    if (/*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0__state == /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0__RES_CONTROLLED) {
        /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0__state = /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0__RES_IMM_GRANTING;
        /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0__reqResId = id;
      }
    else {
        unsigned char __nesc_temp = 
#line 97
        FAIL;

#line 97
        return __nesc_temp;
      }
  }
#line 99
  /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0__ResourceDefaultOwner__immediateRequested();
  if (/*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0__resId == id) {
      /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0__ResourceConfigure__configure(/*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0__resId);
      return SUCCESS;
    }
  /* atomic removed: atomic calls only */
#line 104
  /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0__state = /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0__RES_CONTROLLED;
  return FAIL;
}

# 108 "/home/tinyos/local/src/tinyos-2.x/tos/chips/msp430/usart/Msp430SpiDmaP.nc"
static inline error_t /*Msp430SpiDma0P.SpiP*/Msp430SpiDmaP__0__UsartResource__default__immediateRequest(uint8_t id)
#line 108
{
#line 108
  return FAIL;
}

# 87 "/home/tinyos/local/src/tinyos-2.x/tos/interfaces/Resource.nc"
inline static error_t /*Msp430SpiDma0P.SpiP*/Msp430SpiDmaP__0__UsartResource__immediateRequest(uint8_t arg_0x40c6d9a0){
#line 87
  unsigned char result;
#line 87

#line 87
  switch (arg_0x40c6d9a0) {
#line 87
    case /*CC2420SpiWireC.HplCC2420SpiC.SpiC*/Msp430Spi0C__0__CLIENT_ID:
#line 87
      result = /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0__Resource__immediateRequest(/*CC2420SpiWireC.HplCC2420SpiC.SpiC.UsartC*/Msp430Usart0C__0__CLIENT_ID);
#line 87
      break;
#line 87
    default:
#line 87
      result = /*Msp430SpiDma0P.SpiP*/Msp430SpiDmaP__0__UsartResource__default__immediateRequest(arg_0x40c6d9a0);
#line 87
      break;
#line 87
    }
#line 87

#line 87
  return result;
#line 87
}
#line 87
# 76 "/home/tinyos/local/src/tinyos-2.x/tos/chips/msp430/usart/Msp430SpiDmaP.nc"
static inline error_t /*Msp430SpiDma0P.SpiP*/Msp430SpiDmaP__0__Resource__immediateRequest(uint8_t id)
#line 76
{
  return /*Msp430SpiDma0P.SpiP*/Msp430SpiDmaP__0__UsartResource__immediateRequest(id);
}

# 87 "/home/tinyos/local/src/tinyos-2.x/tos/interfaces/Resource.nc"
inline static error_t CC2420SpiP__SpiResource__immediateRequest(void ){
#line 87
  unsigned char result;
#line 87

#line 87
  result = /*Msp430SpiDma0P.SpiP*/Msp430SpiDmaP__0__Resource__immediateRequest(/*CC2420SpiWireC.HplCC2420SpiC.SpiC*/Msp430Spi0C__0__CLIENT_ID);
#line 87

#line 87
  return result;
#line 87
}
#line 87
# 97 "/home/tinyos/local/src/tinyos-2.x/tos/chips/msp430/usart/HplMsp430Usart.nc"
inline static void HplMsp430I2C0P__HplUsart__resetUsart(bool reset){
#line 97
  HplMsp430Usart0P__Usart__resetUsart(reset);
#line 97
}
#line 97
# 59 "/home/tinyos/local/src/tinyos-2.x/tos/chips/msp430/usart/HplMsp430I2C0P.nc"
static inline void HplMsp430I2C0P__HplI2C__clearModeI2C(void )
#line 59
{
  /* atomic removed: atomic calls only */
#line 60
  {
    HplMsp430I2C0P__U0CTL &= ~((0x20 | 0x04) | 0x01);
    HplMsp430I2C0P__HplUsart__resetUsart(TRUE);
  }
}

# 7 "/home/tinyos/local/src/tinyos-2.x/tos/chips/msp430/usart/HplMsp430I2C.nc"
inline static void HplMsp430Usart0P__HplI2C__clearModeI2C(void ){
#line 7
  HplMsp430I2C0P__HplI2C__clearModeI2C();
#line 7
}
#line 7
# 56 "/home/tinyos/local/src/tinyos-2.x/tos/chips/msp430/pins/HplMsp430GeneralIOP.nc"
static inline void /*HplMsp430GeneralIOC.P35*/HplMsp430GeneralIOP__21__IO__selectIOFunc(void )
#line 56
{
  /* atomic removed: atomic calls only */
#line 56
  * (volatile uint8_t * )27U &= ~(0x01 << 5);
}

# 85 "/home/tinyos/local/src/tinyos-2.x/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
inline static void HplMsp430Usart0P__URXD__selectIOFunc(void ){
#line 85
  /*HplMsp430GeneralIOC.P35*/HplMsp430GeneralIOP__21__IO__selectIOFunc();
#line 85
}
#line 85
# 56 "/home/tinyos/local/src/tinyos-2.x/tos/chips/msp430/pins/HplMsp430GeneralIOP.nc"
static inline void /*HplMsp430GeneralIOC.P34*/HplMsp430GeneralIOP__20__IO__selectIOFunc(void )
#line 56
{
  /* atomic removed: atomic calls only */
#line 56
  * (volatile uint8_t * )27U &= ~(0x01 << 4);
}

# 85 "/home/tinyos/local/src/tinyos-2.x/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
inline static void HplMsp430Usart0P__UTXD__selectIOFunc(void ){
#line 85
  /*HplMsp430GeneralIOC.P34*/HplMsp430GeneralIOP__20__IO__selectIOFunc();
#line 85
}
#line 85
# 207 "/home/tinyos/local/src/tinyos-2.x/tos/chips/msp430/usart/HplMsp430Usart0P.nc"
static inline void HplMsp430Usart0P__Usart__disableUart(void )
#line 207
{
  /* atomic removed: atomic calls only */
#line 208
  {
    HplMsp430Usart0P__ME1 &= ~((1 << 7) | (1 << 6));
    HplMsp430Usart0P__UTXD__selectIOFunc();
    HplMsp430Usart0P__URXD__selectIOFunc();
  }
}

#line 143
static inline void HplMsp430Usart0P__Usart__setUmctl(uint8_t control)
#line 143
{
  U0MCTL = control;
}

#line 132
static inline void HplMsp430Usart0P__Usart__setUbr(uint16_t control)
#line 132
{
  /* atomic removed: atomic calls only */
#line 133
  {
    U0BR0 = control & 0x00FF;
    U0BR1 = (control >> 8) & 0x00FF;
  }
}

#line 256
static inline void HplMsp430Usart0P__configSpi(msp430_spi_union_config_t *config)
#line 256
{

  U0CTL = (config->spiRegisters.uctl | 0x04) | 0x01;
  HplMsp430Usart0P__U0TCTL = config->spiRegisters.utctl;

  HplMsp430Usart0P__Usart__setUbr(config->spiRegisters.ubr);
  HplMsp430Usart0P__Usart__setUmctl(0x00);
}

# 54 "/home/tinyos/local/src/tinyos-2.x/tos/chips/msp430/pins/HplMsp430GeneralIOP.nc"
static inline void /*HplMsp430GeneralIOC.P33*/HplMsp430GeneralIOP__19__IO__selectModuleFunc(void )
#line 54
{
  /* atomic removed: atomic calls only */
#line 54
  * (volatile uint8_t * )27U |= 0x01 << 3;
}

# 78 "/home/tinyos/local/src/tinyos-2.x/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
inline static void HplMsp430Usart0P__UCLK__selectModuleFunc(void ){
#line 78
  /*HplMsp430GeneralIOC.P33*/HplMsp430GeneralIOP__19__IO__selectModuleFunc();
#line 78
}
#line 78
# 54 "/home/tinyos/local/src/tinyos-2.x/tos/chips/msp430/pins/HplMsp430GeneralIOP.nc"
static inline void /*HplMsp430GeneralIOC.P32*/HplMsp430GeneralIOP__18__IO__selectModuleFunc(void )
#line 54
{
  /* atomic removed: atomic calls only */
#line 54
  * (volatile uint8_t * )27U |= 0x01 << 2;
}

# 78 "/home/tinyos/local/src/tinyos-2.x/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
inline static void HplMsp430Usart0P__SOMI__selectModuleFunc(void ){
#line 78
  /*HplMsp430GeneralIOC.P32*/HplMsp430GeneralIOP__18__IO__selectModuleFunc();
#line 78
}
#line 78
# 54 "/home/tinyos/local/src/tinyos-2.x/tos/chips/msp430/pins/HplMsp430GeneralIOP.nc"
static inline void /*HplMsp430GeneralIOC.P31*/HplMsp430GeneralIOP__17__IO__selectModuleFunc(void )
#line 54
{
  /* atomic removed: atomic calls only */
#line 54
  * (volatile uint8_t * )27U |= 0x01 << 1;
}

# 78 "/home/tinyos/local/src/tinyos-2.x/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
inline static void HplMsp430Usart0P__SIMO__selectModuleFunc(void ){
#line 78
  /*HplMsp430GeneralIOC.P31*/HplMsp430GeneralIOP__17__IO__selectModuleFunc();
#line 78
}
#line 78
# 238 "/home/tinyos/local/src/tinyos-2.x/tos/chips/msp430/usart/HplMsp430Usart0P.nc"
static inline void HplMsp430Usart0P__Usart__enableSpi(void )
#line 238
{
  /* atomic removed: atomic calls only */
#line 239
  {
    HplMsp430Usart0P__SIMO__selectModuleFunc();
    HplMsp430Usart0P__SOMI__selectModuleFunc();
    HplMsp430Usart0P__UCLK__selectModuleFunc();
  }
  HplMsp430Usart0P__ME1 |= 1 << 6;
}

#line 345
static inline void HplMsp430Usart0P__Usart__clrIntr(void )
#line 345
{
  HplMsp430Usart0P__IFG1 &= ~((1 << 7) | (1 << 6));
}









static inline void HplMsp430Usart0P__Usart__disableIntr(void )
#line 357
{
  HplMsp430Usart0P__IE1 &= ~((1 << 7) | (1 << 6));
}

# 78 "/home/tinyos/local/src/tinyos-2.x/tos/interfaces/Resource.nc"
inline static error_t CC2420TransmitP__SpiResource__request(void ){
#line 78
  unsigned char result;
#line 78

#line 78
  result = CC2420SpiP__Resource__request(/*CC2420TransmitC.Spi*/CC2420SpiC__3__CLIENT_ID);
#line 78

#line 78
  return result;
#line 78
}
#line 78
# 207 "/home/tinyos/local/src/tinyos-2.x/tos/system/ArbiterP.nc"
static inline void /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0__ResourceDefaultOwner__default__requested(void )
#line 207
{
  /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0__ResourceDefaultOwner__release();
}

# 73 "/home/tinyos/local/src/tinyos-2.x/tos/interfaces/ResourceDefaultOwner.nc"
inline static void /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0__ResourceDefaultOwner__requested(void ){
#line 73
  /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0__ResourceDefaultOwner__default__requested();
#line 73
}
#line 73
# 54 "/home/tinyos/local/src/tinyos-2.x/tos/system/FcfsResourceQueueC.nc"
static inline bool /*Msp430UsartShare0P.ArbiterC.Queue*/FcfsResourceQueueC__1__FcfsQueue__isEnqueued(resource_client_id_t id)
#line 54
{
  /* atomic removed: atomic calls only */
#line 55
  {
    unsigned char __nesc_temp = 
#line 55
    /*Msp430UsartShare0P.ArbiterC.Queue*/FcfsResourceQueueC__1__resQ[id] != /*Msp430UsartShare0P.ArbiterC.Queue*/FcfsResourceQueueC__1__NO_ENTRY || /*Msp430UsartShare0P.ArbiterC.Queue*/FcfsResourceQueueC__1__qTail == id;

#line 55
    return __nesc_temp;
  }
}

#line 72
static inline error_t /*Msp430UsartShare0P.ArbiterC.Queue*/FcfsResourceQueueC__1__FcfsQueue__enqueue(resource_client_id_t id)
#line 72
{
  /* atomic removed: atomic calls only */
#line 73
  {
    if (!/*Msp430UsartShare0P.ArbiterC.Queue*/FcfsResourceQueueC__1__FcfsQueue__isEnqueued(id)) {
        if (/*Msp430UsartShare0P.ArbiterC.Queue*/FcfsResourceQueueC__1__qHead == /*Msp430UsartShare0P.ArbiterC.Queue*/FcfsResourceQueueC__1__NO_ENTRY) {
          /*Msp430UsartShare0P.ArbiterC.Queue*/FcfsResourceQueueC__1__qHead = id;
          }
        else {
#line 78
          /*Msp430UsartShare0P.ArbiterC.Queue*/FcfsResourceQueueC__1__resQ[/*Msp430UsartShare0P.ArbiterC.Queue*/FcfsResourceQueueC__1__qTail] = id;
          }
#line 79
        /*Msp430UsartShare0P.ArbiterC.Queue*/FcfsResourceQueueC__1__qTail = id;
        {
          unsigned char __nesc_temp = 
#line 80
          SUCCESS;

#line 80
          return __nesc_temp;
        }
      }
#line 82
    {
      unsigned char __nesc_temp = 
#line 82
      EBUSY;

#line 82
      return __nesc_temp;
    }
  }
}

# 69 "/home/tinyos/local/src/tinyos-2.x/tos/interfaces/ResourceQueue.nc"
inline static error_t /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0__Queue__enqueue(resource_client_id_t id){
#line 69
  unsigned char result;
#line 69

#line 69
  result = /*Msp430UsartShare0P.ArbiterC.Queue*/FcfsResourceQueueC__1__FcfsQueue__enqueue(id);
#line 69

#line 69
  return result;
#line 69
}
#line 69
# 201 "/home/tinyos/local/src/tinyos-2.x/tos/system/ArbiterP.nc"
static inline void /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0__ResourceRequested__default__requested(uint8_t id)
#line 201
{
}

# 43 "/home/tinyos/local/src/tinyos-2.x/tos/interfaces/ResourceRequested.nc"
inline static void /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0__ResourceRequested__requested(uint8_t arg_0x40e33d38){
#line 43
    /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0__ResourceRequested__default__requested(arg_0x40e33d38);
#line 43
}
#line 43
# 77 "/home/tinyos/local/src/tinyos-2.x/tos/system/ArbiterP.nc"
static inline error_t /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0__Resource__request(uint8_t id)
#line 77
{
  /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0__ResourceRequested__requested(/*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0__resId);
  /* atomic removed: atomic calls only */
#line 79
  {
    if (/*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0__state == /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0__RES_CONTROLLED) {
        /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0__state = /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0__RES_GRANTING;
        /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0__reqResId = id;
      }
    else {
        unsigned char __nesc_temp = 
#line 84
        /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0__Queue__enqueue(id);

#line 84
        return __nesc_temp;
      }
  }
#line 86
  /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0__ResourceDefaultOwner__requested();
  return SUCCESS;
}

# 107 "/home/tinyos/local/src/tinyos-2.x/tos/chips/msp430/usart/Msp430SpiDmaP.nc"
static inline error_t /*Msp430SpiDma0P.SpiP*/Msp430SpiDmaP__0__UsartResource__default__request(uint8_t id)
#line 107
{
#line 107
  return FAIL;
}

# 78 "/home/tinyos/local/src/tinyos-2.x/tos/interfaces/Resource.nc"
inline static error_t /*Msp430SpiDma0P.SpiP*/Msp430SpiDmaP__0__UsartResource__request(uint8_t arg_0x40c6d9a0){
#line 78
  unsigned char result;
#line 78

#line 78
  switch (arg_0x40c6d9a0) {
#line 78
    case /*CC2420SpiWireC.HplCC2420SpiC.SpiC*/Msp430Spi0C__0__CLIENT_ID:
#line 78
      result = /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0__Resource__request(/*CC2420SpiWireC.HplCC2420SpiC.SpiC.UsartC*/Msp430Usart0C__0__CLIENT_ID);
#line 78
      break;
#line 78
    default:
#line 78
      result = /*Msp430SpiDma0P.SpiP*/Msp430SpiDmaP__0__UsartResource__default__request(arg_0x40c6d9a0);
#line 78
      break;
#line 78
    }
#line 78

#line 78
  return result;
#line 78
}
#line 78
# 80 "/home/tinyos/local/src/tinyos-2.x/tos/chips/msp430/usart/Msp430SpiDmaP.nc"
static inline error_t /*Msp430SpiDma0P.SpiP*/Msp430SpiDmaP__0__Resource__request(uint8_t id)
#line 80
{
  return /*Msp430SpiDma0P.SpiP*/Msp430SpiDmaP__0__UsartResource__request(id);
}

# 78 "/home/tinyos/local/src/tinyos-2.x/tos/interfaces/Resource.nc"
inline static error_t CC2420SpiP__SpiResource__request(void ){
#line 78
  unsigned char result;
#line 78

#line 78
  result = /*Msp430SpiDma0P.SpiP*/Msp430SpiDmaP__0__Resource__request(/*CC2420SpiWireC.HplCC2420SpiC.SpiC*/Msp430Spi0C__0__CLIENT_ID);
#line 78

#line 78
  return result;
#line 78
}
#line 78
# 45 "/home/tinyos/local/src/tinyos-2.x/tos/chips/cc2420/interfaces/CC2420Strobe.nc"
inline static cc2420_status_t CC2420TransmitP__STXONCCA__strobe(void ){
#line 45
  unsigned char result;
#line 45

#line 45
  result = CC2420SpiP__Strobe__strobe(CC2420_STXONCCA);
#line 45

#line 45
  return result;
#line 45
}
#line 45
inline static cc2420_status_t CC2420TransmitP__STXON__strobe(void ){
#line 45
  unsigned char result;
#line 45

#line 45
  result = CC2420SpiP__Strobe__strobe(CC2420_STXON);
#line 45

#line 45
  return result;
#line 45
}
#line 45
inline static cc2420_status_t CC2420TransmitP__SNOP__strobe(void ){
#line 45
  unsigned char result;
#line 45

#line 45
  result = CC2420SpiP__Strobe__strobe(CC2420_SNOP);
#line 45

#line 45
  return result;
#line 45
}
#line 45
# 181 "/home/tinyos/local/src/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static inline void /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP__6__Compare__default__fired(void )
{
}

# 34 "/home/tinyos/local/src/tinyos-2.x/tos/chips/msp430/timer/Msp430Compare.nc"
inline static void /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP__6__Compare__fired(void ){
#line 34
  /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP__6__Compare__default__fired();
#line 34
}
#line 34
# 139 "/home/tinyos/local/src/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static inline uint16_t /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP__6__Capture__getEvent(void )
{
  return * (volatile uint16_t * )408U;
}

#line 177
static inline void /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP__6__Capture__default__captured(uint16_t n)
{
}

# 75 "/home/tinyos/local/src/tinyos-2.x/tos/chips/msp430/timer/Msp430Capture.nc"
inline static void /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP__6__Capture__captured(uint16_t time){
#line 75
  /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP__6__Capture__default__captured(time);
#line 75
}
#line 75
# 47 "/home/tinyos/local/src/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static inline  /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP__6__cc_t /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP__6__int2CC(uint16_t x)
#line 47
{
#line 47
  union /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP__6____nesc_unnamed4453 {
#line 47
    uint16_t f;
#line 47
    /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP__6__cc_t t;
  } 
#line 47
  c = { .f = x };

#line 47
  return c.t;
}

#line 74
static inline /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP__6__cc_t /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP__6__Control__getControl(void )
{
  return /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP__6__int2CC(* (volatile uint16_t * )392U);
}

#line 169
static inline void /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP__6__Event__fired(void )
{
  if (/*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP__6__Control__getControl().cap) {
    /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP__6__Capture__captured(/*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP__6__Capture__getEvent());
    }
  else {
#line 174
    /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP__6__Compare__fired();
    }
}




static inline void /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP__7__Compare__default__fired(void )
{
}

# 34 "/home/tinyos/local/src/tinyos-2.x/tos/chips/msp430/timer/Msp430Compare.nc"
inline static void /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP__7__Compare__fired(void ){
#line 34
  /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP__7__Compare__default__fired();
#line 34
}
#line 34
# 139 "/home/tinyos/local/src/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static inline uint16_t /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP__7__Capture__getEvent(void )
{
  return * (volatile uint16_t * )410U;
}

#line 177
static inline void /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP__7__Capture__default__captured(uint16_t n)
{
}

# 75 "/home/tinyos/local/src/tinyos-2.x/tos/chips/msp430/timer/Msp430Capture.nc"
inline static void /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP__7__Capture__captured(uint16_t time){
#line 75
  /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP__7__Capture__default__captured(time);
#line 75
}
#line 75
# 47 "/home/tinyos/local/src/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static inline  /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP__7__cc_t /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP__7__int2CC(uint16_t x)
#line 47
{
#line 47
  union /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP__7____nesc_unnamed4454 {
#line 47
    uint16_t f;
#line 47
    /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP__7__cc_t t;
  } 
#line 47
  c = { .f = x };

#line 47
  return c.t;
}

#line 74
static inline /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP__7__cc_t /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP__7__Control__getControl(void )
{
  return /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP__7__int2CC(* (volatile uint16_t * )394U);
}

#line 169
static inline void /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP__7__Event__fired(void )
{
  if (/*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP__7__Control__getControl().cap) {
    /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP__7__Capture__captured(/*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP__7__Capture__getEvent());
    }
  else {
#line 174
    /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP__7__Compare__fired();
    }
}




static inline void /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP__8__Compare__default__fired(void )
{
}

# 34 "/home/tinyos/local/src/tinyos-2.x/tos/chips/msp430/timer/Msp430Compare.nc"
inline static void /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP__8__Compare__fired(void ){
#line 34
  /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP__8__Compare__default__fired();
#line 34
}
#line 34
# 139 "/home/tinyos/local/src/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static inline uint16_t /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP__8__Capture__getEvent(void )
{
  return * (volatile uint16_t * )412U;
}

#line 177
static inline void /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP__8__Capture__default__captured(uint16_t n)
{
}

# 75 "/home/tinyos/local/src/tinyos-2.x/tos/chips/msp430/timer/Msp430Capture.nc"
inline static void /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP__8__Capture__captured(uint16_t time){
#line 75
  /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP__8__Capture__default__captured(time);
#line 75
}
#line 75
# 47 "/home/tinyos/local/src/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static inline  /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP__8__cc_t /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP__8__int2CC(uint16_t x)
#line 47
{
#line 47
  union /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP__8____nesc_unnamed4455 {
#line 47
    uint16_t f;
#line 47
    /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP__8__cc_t t;
  } 
#line 47
  c = { .f = x };

#line 47
  return c.t;
}

#line 74
static inline /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP__8__cc_t /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP__8__Control__getControl(void )
{
  return /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP__8__int2CC(* (volatile uint16_t * )396U);
}

#line 169
static inline void /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP__8__Event__fired(void )
{
  if (/*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP__8__Control__getControl().cap) {
    /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP__8__Capture__captured(/*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP__8__Capture__getEvent());
    }
  else {
#line 174
    /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP__8__Compare__fired();
    }
}




static inline void /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP__9__Compare__default__fired(void )
{
}

# 34 "/home/tinyos/local/src/tinyos-2.x/tos/chips/msp430/timer/Msp430Compare.nc"
inline static void /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP__9__Compare__fired(void ){
#line 34
  /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP__9__Compare__default__fired();
#line 34
}
#line 34
# 139 "/home/tinyos/local/src/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static inline uint16_t /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP__9__Capture__getEvent(void )
{
  return * (volatile uint16_t * )414U;
}

#line 177
static inline void /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP__9__Capture__default__captured(uint16_t n)
{
}

# 75 "/home/tinyos/local/src/tinyos-2.x/tos/chips/msp430/timer/Msp430Capture.nc"
inline static void /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP__9__Capture__captured(uint16_t time){
#line 75
  /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP__9__Capture__default__captured(time);
#line 75
}
#line 75
# 47 "/home/tinyos/local/src/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static inline  /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP__9__cc_t /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP__9__int2CC(uint16_t x)
#line 47
{
#line 47
  union /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP__9____nesc_unnamed4456 {
#line 47
    uint16_t f;
#line 47
    /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP__9__cc_t t;
  } 
#line 47
  c = { .f = x };

#line 47
  return c.t;
}

#line 74
static inline /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP__9__cc_t /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP__9__Control__getControl(void )
{
  return /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP__9__int2CC(* (volatile uint16_t * )398U);
}

#line 169
static inline void /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP__9__Event__fired(void )
{
  if (/*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP__9__Control__getControl().cap) {
    /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP__9__Capture__captured(/*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP__9__Capture__getEvent());
    }
  else {
#line 174
    /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP__9__Compare__fired();
    }
}

# 120 "/home/tinyos/local/src/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerP.nc"
static inline void /*Msp430TimerC.Msp430TimerB*/Msp430TimerP__1__VectorTimerX1__fired(void )
{
  uint8_t n = * (volatile uint16_t * )286U;

#line 123
  /*Msp430TimerC.Msp430TimerB*/Msp430TimerP__1__Event__fired(n >> 1);
}

# 28 "/home/tinyos/local/src/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerEvent.nc"
inline static void Msp430TimerCommonP__VectorTimerB1__fired(void ){
#line 28
  /*Msp430TimerC.Msp430TimerB*/Msp430TimerP__1__VectorTimerX1__fired();
#line 28
}
#line 28
# 113 "/home/tinyos/local/src/tinyos-2.x/tos/system/SchedulerBasicP.nc"
static inline void SchedulerBasicP__Scheduler__init(void )
{
  /* atomic removed: atomic calls only */
  {
    memset((void *)SchedulerBasicP__m_next, SchedulerBasicP__NO_TASK, sizeof SchedulerBasicP__m_next);
    SchedulerBasicP__m_head = SchedulerBasicP__NO_TASK;
    SchedulerBasicP__m_tail = SchedulerBasicP__NO_TASK;
  }
}

# 46 "/home/tinyos/local/src/tinyos-2.x/tos/interfaces/Scheduler.nc"
inline static void RealMainP__Scheduler__init(void ){
#line 46
  SchedulerBasicP__Scheduler__init();
#line 46
}
#line 46
# 45 "/home/tinyos/local/src/tinyos-2.x/tos/chips/msp430/pins/HplMsp430GeneralIOP.nc"
static inline void /*HplMsp430GeneralIOC.P56*/HplMsp430GeneralIOP__38__IO__set(void )
#line 45
{
  /* atomic removed: atomic calls only */
#line 45
  * (volatile uint8_t * )49U |= 0x01 << 6;
}

# 34 "/home/tinyos/local/src/tinyos-2.x/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
inline static void /*PlatformLedsC.Led2Impl*/Msp430GpioC__2__HplGeneralIO__set(void ){
#line 34
  /*HplMsp430GeneralIOC.P56*/HplMsp430GeneralIOP__38__IO__set();
#line 34
}
#line 34
# 37 "/home/tinyos/local/src/tinyos-2.x/tos/chips/msp430/pins/Msp430GpioC.nc"
static inline void /*PlatformLedsC.Led2Impl*/Msp430GpioC__2__GeneralIO__set(void )
#line 37
{
#line 37
  /*PlatformLedsC.Led2Impl*/Msp430GpioC__2__HplGeneralIO__set();
}

# 29 "/home/tinyos/local/src/tinyos-2.x/tos/interfaces/GeneralIO.nc"
inline static void LedsP__Led2__set(void ){
#line 29
  /*PlatformLedsC.Led2Impl*/Msp430GpioC__2__GeneralIO__set();
#line 29
}
#line 29
# 45 "/home/tinyos/local/src/tinyos-2.x/tos/chips/msp430/pins/HplMsp430GeneralIOP.nc"
static inline void /*HplMsp430GeneralIOC.P55*/HplMsp430GeneralIOP__37__IO__set(void )
#line 45
{
  /* atomic removed: atomic calls only */
#line 45
  * (volatile uint8_t * )49U |= 0x01 << 5;
}

# 34 "/home/tinyos/local/src/tinyos-2.x/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
inline static void /*PlatformLedsC.Led1Impl*/Msp430GpioC__1__HplGeneralIO__set(void ){
#line 34
  /*HplMsp430GeneralIOC.P55*/HplMsp430GeneralIOP__37__IO__set();
#line 34
}
#line 34
# 37 "/home/tinyos/local/src/tinyos-2.x/tos/chips/msp430/pins/Msp430GpioC.nc"
static inline void /*PlatformLedsC.Led1Impl*/Msp430GpioC__1__GeneralIO__set(void )
#line 37
{
#line 37
  /*PlatformLedsC.Led1Impl*/Msp430GpioC__1__HplGeneralIO__set();
}

# 29 "/home/tinyos/local/src/tinyos-2.x/tos/interfaces/GeneralIO.nc"
inline static void LedsP__Led1__set(void ){
#line 29
  /*PlatformLedsC.Led1Impl*/Msp430GpioC__1__GeneralIO__set();
#line 29
}
#line 29
# 45 "/home/tinyos/local/src/tinyos-2.x/tos/chips/msp430/pins/HplMsp430GeneralIOP.nc"
static inline void /*HplMsp430GeneralIOC.P54*/HplMsp430GeneralIOP__36__IO__set(void )
#line 45
{
  /* atomic removed: atomic calls only */
#line 45
  * (volatile uint8_t * )49U |= 0x01 << 4;
}

# 34 "/home/tinyos/local/src/tinyos-2.x/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
inline static void /*PlatformLedsC.Led0Impl*/Msp430GpioC__0__HplGeneralIO__set(void ){
#line 34
  /*HplMsp430GeneralIOC.P54*/HplMsp430GeneralIOP__36__IO__set();
#line 34
}
#line 34
# 37 "/home/tinyos/local/src/tinyos-2.x/tos/chips/msp430/pins/Msp430GpioC.nc"
static inline void /*PlatformLedsC.Led0Impl*/Msp430GpioC__0__GeneralIO__set(void )
#line 37
{
#line 37
  /*PlatformLedsC.Led0Impl*/Msp430GpioC__0__HplGeneralIO__set();
}

# 29 "/home/tinyos/local/src/tinyos-2.x/tos/interfaces/GeneralIO.nc"
inline static void LedsP__Led0__set(void ){
#line 29
  /*PlatformLedsC.Led0Impl*/Msp430GpioC__0__GeneralIO__set();
#line 29
}
#line 29
# 52 "/home/tinyos/local/src/tinyos-2.x/tos/chips/msp430/pins/HplMsp430GeneralIOP.nc"
static inline void /*HplMsp430GeneralIOC.P56*/HplMsp430GeneralIOP__38__IO__makeOutput(void )
#line 52
{
  /* atomic removed: atomic calls only */
#line 52
  * (volatile uint8_t * )50U |= 0x01 << 6;
}

# 71 "/home/tinyos/local/src/tinyos-2.x/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
inline static void /*PlatformLedsC.Led2Impl*/Msp430GpioC__2__HplGeneralIO__makeOutput(void ){
#line 71
  /*HplMsp430GeneralIOC.P56*/HplMsp430GeneralIOP__38__IO__makeOutput();
#line 71
}
#line 71
# 43 "/home/tinyos/local/src/tinyos-2.x/tos/chips/msp430/pins/Msp430GpioC.nc"
static inline void /*PlatformLedsC.Led2Impl*/Msp430GpioC__2__GeneralIO__makeOutput(void )
#line 43
{
#line 43
  /*PlatformLedsC.Led2Impl*/Msp430GpioC__2__HplGeneralIO__makeOutput();
}

# 35 "/home/tinyos/local/src/tinyos-2.x/tos/interfaces/GeneralIO.nc"
inline static void LedsP__Led2__makeOutput(void ){
#line 35
  /*PlatformLedsC.Led2Impl*/Msp430GpioC__2__GeneralIO__makeOutput();
#line 35
}
#line 35
# 52 "/home/tinyos/local/src/tinyos-2.x/tos/chips/msp430/pins/HplMsp430GeneralIOP.nc"
static inline void /*HplMsp430GeneralIOC.P55*/HplMsp430GeneralIOP__37__IO__makeOutput(void )
#line 52
{
  /* atomic removed: atomic calls only */
#line 52
  * (volatile uint8_t * )50U |= 0x01 << 5;
}

# 71 "/home/tinyos/local/src/tinyos-2.x/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
inline static void /*PlatformLedsC.Led1Impl*/Msp430GpioC__1__HplGeneralIO__makeOutput(void ){
#line 71
  /*HplMsp430GeneralIOC.P55*/HplMsp430GeneralIOP__37__IO__makeOutput();
#line 71
}
#line 71
# 43 "/home/tinyos/local/src/tinyos-2.x/tos/chips/msp430/pins/Msp430GpioC.nc"
static inline void /*PlatformLedsC.Led1Impl*/Msp430GpioC__1__GeneralIO__makeOutput(void )
#line 43
{
#line 43
  /*PlatformLedsC.Led1Impl*/Msp430GpioC__1__HplGeneralIO__makeOutput();
}

# 35 "/home/tinyos/local/src/tinyos-2.x/tos/interfaces/GeneralIO.nc"
inline static void LedsP__Led1__makeOutput(void ){
#line 35
  /*PlatformLedsC.Led1Impl*/Msp430GpioC__1__GeneralIO__makeOutput();
#line 35
}
#line 35
# 52 "/home/tinyos/local/src/tinyos-2.x/tos/chips/msp430/pins/HplMsp430GeneralIOP.nc"
static inline void /*HplMsp430GeneralIOC.P54*/HplMsp430GeneralIOP__36__IO__makeOutput(void )
#line 52
{
  /* atomic removed: atomic calls only */
#line 52
  * (volatile uint8_t * )50U |= 0x01 << 4;
}

# 71 "/home/tinyos/local/src/tinyos-2.x/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
inline static void /*PlatformLedsC.Led0Impl*/Msp430GpioC__0__HplGeneralIO__makeOutput(void ){
#line 71
  /*HplMsp430GeneralIOC.P54*/HplMsp430GeneralIOP__36__IO__makeOutput();
#line 71
}
#line 71
# 43 "/home/tinyos/local/src/tinyos-2.x/tos/chips/msp430/pins/Msp430GpioC.nc"
static inline void /*PlatformLedsC.Led0Impl*/Msp430GpioC__0__GeneralIO__makeOutput(void )
#line 43
{
#line 43
  /*PlatformLedsC.Led0Impl*/Msp430GpioC__0__HplGeneralIO__makeOutput();
}

# 35 "/home/tinyos/local/src/tinyos-2.x/tos/interfaces/GeneralIO.nc"
inline static void LedsP__Led0__makeOutput(void ){
#line 35
  /*PlatformLedsC.Led0Impl*/Msp430GpioC__0__GeneralIO__makeOutput();
#line 35
}
#line 35
# 45 "/home/tinyos/local/src/tinyos-2.x/tos/system/LedsP.nc"
static inline error_t LedsP__Init__init(void )
#line 45
{
  /* atomic removed: atomic calls only */
#line 46
  {
    ;
    LedsP__Led0__makeOutput();
    LedsP__Led1__makeOutput();
    LedsP__Led2__makeOutput();
    LedsP__Led0__set();
    LedsP__Led1__set();
    LedsP__Led2__set();
  }
  return SUCCESS;
}

# 51 "/home/tinyos/local/src/tinyos-2.x/tos/interfaces/Init.nc"
inline static error_t PlatformP__LedsInit__init(void ){
#line 51
  unsigned char result;
#line 51

#line 51
  result = LedsP__Init__init();
#line 51

#line 51
  return result;
#line 51
}
#line 51
# 36 "/home/tinyos/local/src/tinyos-2.x/tos/platforms/telosb/hardware.h"
static inline  void TOSH_SET_SIMO0_PIN()
#line 36
{
#line 36
  static volatile uint8_t r __asm ("0x0019");

#line 36
  r |= 1 << 1;
}

#line 37
static inline  void TOSH_SET_UCLK0_PIN()
#line 37
{
#line 37
  static volatile uint8_t r __asm ("0x0019");

#line 37
  r |= 1 << 3;
}

#line 88
static inline  void TOSH_SET_FLASH_CS_PIN()
#line 88
{
#line 88
  static volatile uint8_t r __asm ("0x001D");

#line 88
  r |= 1 << 4;
}

#line 37
static inline  void TOSH_CLR_UCLK0_PIN()
#line 37
{
#line 37
  static volatile uint8_t r __asm ("0x0019");

#line 37
  r &= ~(1 << 3);
}

#line 88
static inline  void TOSH_CLR_FLASH_CS_PIN()
#line 88
{
#line 88
  static volatile uint8_t r __asm ("0x001D");

#line 88
  r &= ~(1 << 4);
}

# 11 "/home/tinyos/local/src/tinyos-2.x/tos/platforms/telosb/MotePlatformC.nc"
static __inline void MotePlatformC__TOSH_wait(void )
#line 11
{
   __asm volatile ("nop"); __asm volatile ("nop");}

# 89 "/home/tinyos/local/src/tinyos-2.x/tos/platforms/telosb/hardware.h"
static inline  void TOSH_SET_FLASH_HOLD_PIN()
#line 89
{
#line 89
  static volatile uint8_t r __asm ("0x001D");

#line 89
  r |= 1 << 7;
}

#line 88
static inline  void TOSH_MAKE_FLASH_CS_OUTPUT()
#line 88
{
#line 88
  static volatile uint8_t r __asm ("0x001E");

#line 88
  r |= 1 << 4;
}

#line 89
static inline  void TOSH_MAKE_FLASH_HOLD_OUTPUT()
#line 89
{
#line 89
  static volatile uint8_t r __asm ("0x001E");

#line 89
  r |= 1 << 7;
}

#line 37
static inline  void TOSH_MAKE_UCLK0_OUTPUT()
#line 37
{
#line 37
  static volatile uint8_t r __asm ("0x001A");

#line 37
  r |= 1 << 3;
}

#line 36
static inline  void TOSH_MAKE_SIMO0_OUTPUT()
#line 36
{
#line 36
  static volatile uint8_t r __asm ("0x001A");

#line 36
  r |= 1 << 1;
}

# 27 "/home/tinyos/local/src/tinyos-2.x/tos/platforms/telosb/MotePlatformC.nc"
static inline void MotePlatformC__TOSH_FLASH_M25P_DP(void )
#line 27
{

  TOSH_MAKE_SIMO0_OUTPUT();
  TOSH_MAKE_UCLK0_OUTPUT();
  TOSH_MAKE_FLASH_HOLD_OUTPUT();
  TOSH_MAKE_FLASH_CS_OUTPUT();
  TOSH_SET_FLASH_HOLD_PIN();
  TOSH_SET_FLASH_CS_PIN();

  MotePlatformC__TOSH_wait();


  TOSH_CLR_FLASH_CS_PIN();
  TOSH_CLR_UCLK0_PIN();

  MotePlatformC__TOSH_FLASH_M25P_DP_bit(TRUE);
  MotePlatformC__TOSH_FLASH_M25P_DP_bit(FALSE);
  MotePlatformC__TOSH_FLASH_M25P_DP_bit(TRUE);
  MotePlatformC__TOSH_FLASH_M25P_DP_bit(TRUE);
  MotePlatformC__TOSH_FLASH_M25P_DP_bit(TRUE);
  MotePlatformC__TOSH_FLASH_M25P_DP_bit(FALSE);
  MotePlatformC__TOSH_FLASH_M25P_DP_bit(FALSE);
  MotePlatformC__TOSH_FLASH_M25P_DP_bit(TRUE);

  TOSH_SET_FLASH_CS_PIN();
  TOSH_SET_UCLK0_PIN();
  TOSH_SET_SIMO0_PIN();
}

#line 6
static __inline void MotePlatformC__uwait(uint16_t u)
#line 6
{
  uint16_t t0 = TA0R;

#line 8
  while (TA0R - t0 <= u) ;
}

#line 56
static inline error_t MotePlatformC__Init__init(void )
#line 56
{
  /* atomic removed: atomic calls only */

  {
    P1SEL = 0;
    P2SEL = 0;
    P3SEL = 0;
    P4SEL = 0;
    P5SEL = 0;
    P6SEL = 0;

    P1OUT = 0x00;
    P1DIR = 0xe0;

    P2OUT = 0x30;
    P2DIR = 0x7b;

    P3OUT = 0x00;
    P3DIR = 0xf1;

    P4OUT = 0xdd;
    P4DIR = 0xfd;

    P5OUT = 0xff;
    P5DIR = 0xff;

    P6OUT = 0x00;
    P6DIR = 0xff;

    P1IE = 0;
    P2IE = 0;






    MotePlatformC__uwait(1024 * 10);

    MotePlatformC__TOSH_FLASH_M25P_DP();
  }

  return SUCCESS;
}

# 51 "/home/tinyos/local/src/tinyos-2.x/tos/interfaces/Init.nc"
inline static error_t PlatformP__MoteInit__init(void ){
#line 51
  unsigned char result;
#line 51

#line 51
  result = MotePlatformC__Init__init();
#line 51

#line 51
  return result;
#line 51
}
#line 51
# 152 "/home/tinyos/local/src/tinyos-2.x/tos/chips/msp430/timer/Msp430ClockP.nc"
static inline void Msp430ClockP__startTimerB(void )
{

  Msp430ClockP__TBCTL = 0x0020 | (Msp430ClockP__TBCTL & ~(0x0020 | 0x0010));
}

#line 140
static inline void Msp430ClockP__startTimerA(void )
{

  Msp430ClockP__TA0CTL = 0x0020 | (Msp430ClockP__TA0CTL & ~(0x0020 | 0x0010));
}

#line 104
static inline void Msp430ClockP__Msp430ClockInit__defaultInitTimerB(void )
{
  TBR = 0;









  Msp430ClockP__TBCTL = 0x0100 | 0x0002;
}

#line 134
static inline void Msp430ClockP__Msp430ClockInit__default__initTimerB(void )
{
  Msp430ClockP__Msp430ClockInit__defaultInitTimerB();
}

# 32 "/home/tinyos/local/src/tinyos-2.x/tos/chips/msp430/timer/Msp430ClockInit.nc"
inline static void Msp430ClockP__Msp430ClockInit__initTimerB(void ){
#line 32
  Msp430ClockP__Msp430ClockInit__default__initTimerB();
#line 32
}
#line 32
# 89 "/home/tinyos/local/src/tinyos-2.x/tos/chips/msp430/timer/Msp430ClockP.nc"
static inline void Msp430ClockP__Msp430ClockInit__defaultInitTimerA(void )
{
  TA0R = 0;









  Msp430ClockP__TA0CTL = 0x0200 | 0x0002;
}

#line 129
static inline void Msp430ClockP__Msp430ClockInit__default__initTimerA(void )
{
  Msp430ClockP__Msp430ClockInit__defaultInitTimerA();
}

# 31 "/home/tinyos/local/src/tinyos-2.x/tos/chips/msp430/timer/Msp430ClockInit.nc"
inline static void Msp430ClockP__Msp430ClockInit__initTimerA(void ){
#line 31
  Msp430ClockP__Msp430ClockInit__default__initTimerA();
#line 31
}
#line 31
# 68 "/home/tinyos/local/src/tinyos-2.x/tos/chips/msp430/timer/Msp430ClockP.nc"
static inline void Msp430ClockP__Msp430ClockInit__defaultInitClocks(void )
{





  BCSCTL1 = 0x80 | (BCSCTL1 & ((0x04 | 0x02) | 0x01));







  BCSCTL2 = 0x04;


  Msp430ClockP__IE1 &= ~(1 << 1);
}

#line 124
static inline void Msp430ClockP__Msp430ClockInit__default__initClocks(void )
{
  Msp430ClockP__Msp430ClockInit__defaultInitClocks();
}

# 30 "/home/tinyos/local/src/tinyos-2.x/tos/chips/msp430/timer/Msp430ClockInit.nc"
inline static void Msp430ClockP__Msp430ClockInit__initClocks(void ){
#line 30
  Msp430ClockP__Msp430ClockInit__default__initClocks();
#line 30
}
#line 30
# 170 "/home/tinyos/local/src/tinyos-2.x/tos/chips/msp430/timer/Msp430ClockP.nc"
static inline uint16_t Msp430ClockP__test_calib_busywait_delta(int calib)
{
  int8_t aclk_count = 2;
  uint16_t dco_prev = 0;
  uint16_t dco_curr = 0;

  Msp430ClockP__set_dco_calib(calib);

  while (aclk_count-- > 0) 
    {
      TBCCR0 = TBR + Msp430ClockP__ACLK_CALIB_PERIOD;
      TBCCTL0 &= ~0x0001;
      while ((TBCCTL0 & 0x0001) == 0) ;
      dco_prev = dco_curr;
      dco_curr = TA0R;
    }

  return dco_curr - dco_prev;
}




static inline void Msp430ClockP__busyCalibrateDco(void )
{

  int calib;
  int step;






  for (calib = 0, step = 0x800; step != 0; step >>= 1) 
    {

      if (Msp430ClockP__test_calib_busywait_delta(calib | step) <= Msp430ClockP__TARGET_DCO_DELTA) {
        calib |= step;
        }
    }

  if ((calib & 0x0e0) == 0x0e0) {
    calib &= ~0x01f;
    }
  Msp430ClockP__set_dco_calib(calib);
}

#line 56
static inline void Msp430ClockP__Msp430ClockInit__defaultSetupDcoCalibrate(void )
{



  Msp430ClockP__TA0CTL = 0x0200 | 0x0020;
  Msp430ClockP__TBCTL = 0x0100 | 0x0020;
  BCSCTL1 = 0x80 | 0x04;
  BCSCTL2 = 0;
  TBCCTL0 = 0x4000;
}

#line 119
static inline void Msp430ClockP__Msp430ClockInit__default__setupDcoCalibrate(void )
{
  Msp430ClockP__Msp430ClockInit__defaultSetupDcoCalibrate();
}

# 29 "/home/tinyos/local/src/tinyos-2.x/tos/chips/msp430/timer/Msp430ClockInit.nc"
inline static void Msp430ClockP__Msp430ClockInit__setupDcoCalibrate(void ){
#line 29
  Msp430ClockP__Msp430ClockInit__default__setupDcoCalibrate();
#line 29
}
#line 29
# 218 "/home/tinyos/local/src/tinyos-2.x/tos/chips/msp430/timer/Msp430ClockP.nc"
static inline error_t Msp430ClockP__Init__init(void )
{

  Msp430ClockP__TA0CTL = 0x0004;
  Msp430ClockP__TA0IV = 0;
  Msp430ClockP__TBCTL = 0x0004;
  Msp430ClockP__TBIV = 0;
  /* atomic removed: atomic calls only */

  {
    Msp430ClockP__Msp430ClockInit__setupDcoCalibrate();
    Msp430ClockP__busyCalibrateDco();
    Msp430ClockP__Msp430ClockInit__initClocks();
    Msp430ClockP__Msp430ClockInit__initTimerA();
    Msp430ClockP__Msp430ClockInit__initTimerB();
    Msp430ClockP__startTimerA();
    Msp430ClockP__startTimerB();
  }

  return SUCCESS;
}

# 51 "/home/tinyos/local/src/tinyos-2.x/tos/interfaces/Init.nc"
inline static error_t PlatformP__MoteClockInit__init(void ){
#line 51
  unsigned char result;
#line 51

#line 51
  result = Msp430ClockP__Init__init();
#line 51

#line 51
  return result;
#line 51
}
#line 51
# 10 "/home/tinyos/local/src/tinyos-2.x/tos/platforms/telosa/PlatformP.nc"
static inline error_t PlatformP__Init__init(void )
#line 10
{
  PlatformP__MoteClockInit__init();
  PlatformP__MoteInit__init();
  PlatformP__LedsInit__init();
  return SUCCESS;
}

# 51 "/home/tinyos/local/src/tinyos-2.x/tos/interfaces/Init.nc"
inline static error_t RealMainP__PlatformInit__init(void ){
#line 51
  unsigned char result;
#line 51

#line 51
  result = PlatformP__Init__init();
#line 51

#line 51
  return result;
#line 51
}
#line 51
# 36 "/home/tinyos/local/src/tinyos-2.x/tos/platforms/telosb/hardware.h"
static inline  void TOSH_CLR_SIMO0_PIN()
#line 36
{
#line 36
  static volatile uint8_t r __asm ("0x0019");

#line 36
  r &= ~(1 << 1);
}

# 54 "/home/tinyos/local/src/tinyos-2.x/tos/interfaces/Scheduler.nc"
inline static bool RealMainP__Scheduler__runNextTask(void ){
#line 54
  unsigned char result;
#line 54

#line 54
  result = SchedulerBasicP__Scheduler__runNextTask();
#line 54

#line 54
  return result;
#line 54
}
#line 54
# 83 "/home/tinyos/local/src/tinyos-2.x/tos/interfaces/SplitControl.nc"
inline static error_t /*HplSensirionSht11C.SplitControlPowerManagerC.PowerManager*/PowerManagerP__0__SplitControl__start(void ){
#line 83
  unsigned char result;
#line 83

#line 83
  result = HplSensirionSht11P__SplitControl__start();
#line 83

#line 83
  return result;
#line 83
}
#line 83
# 84 "/home/tinyos/local/src/tinyos-2.x/tos/lib/power/PowerManagerP.nc"
static inline error_t /*HplSensirionSht11C.SplitControlPowerManagerC.PowerManager*/PowerManagerP__0__StdControl__default__start(void )
#line 84
{
  return SUCCESS;
}

# 74 "/home/tinyos/local/src/tinyos-2.x/tos/interfaces/StdControl.nc"
inline static error_t /*HplSensirionSht11C.SplitControlPowerManagerC.PowerManager*/PowerManagerP__0__StdControl__start(void ){
#line 74
  unsigned char result;
#line 74

#line 74
  result = /*HplSensirionSht11C.SplitControlPowerManagerC.PowerManager*/PowerManagerP__0__StdControl__default__start();
#line 74

#line 74
  return result;
#line 74
}
#line 74
# 63 "/home/tinyos/local/src/tinyos-2.x/tos/lib/power/PowerManagerP.nc"
static inline void /*HplSensirionSht11C.SplitControlPowerManagerC.PowerManager*/PowerManagerP__0__startTask__runTask(void )
#line 63
{
  /*HplSensirionSht11C.SplitControlPowerManagerC.PowerManager*/PowerManagerP__0__StdControl__start();
  /*HplSensirionSht11C.SplitControlPowerManagerC.PowerManager*/PowerManagerP__0__SplitControl__start();
}

# 52 "/home/tinyos/local/src/tinyos-2.x/tos/chips/msp430/pins/HplMsp430GeneralIOP.nc"
static inline void /*HplMsp430GeneralIOC.P17*/HplMsp430GeneralIOP__7__IO__makeOutput(void )
#line 52
{
#line 52
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 52
    * (volatile uint8_t * )34U |= 0x01 << 7;
#line 52
    __nesc_atomic_end(__nesc_atomic); }
}

# 71 "/home/tinyos/local/src/tinyos-2.x/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
inline static void /*HplSensirionSht11C.PWRM*/Msp430GpioC__12__HplGeneralIO__makeOutput(void ){
#line 71
  /*HplMsp430GeneralIOC.P17*/HplMsp430GeneralIOP__7__IO__makeOutput();
#line 71
}
#line 71
# 43 "/home/tinyos/local/src/tinyos-2.x/tos/chips/msp430/pins/Msp430GpioC.nc"
static inline void /*HplSensirionSht11C.PWRM*/Msp430GpioC__12__GeneralIO__makeOutput(void )
#line 43
{
#line 43
  /*HplSensirionSht11C.PWRM*/Msp430GpioC__12__HplGeneralIO__makeOutput();
}

# 35 "/home/tinyos/local/src/tinyos-2.x/tos/interfaces/GeneralIO.nc"
inline static void HplSensirionSht11P__PWR__makeOutput(void ){
#line 35
  /*HplSensirionSht11C.PWRM*/Msp430GpioC__12__GeneralIO__makeOutput();
#line 35
}
#line 35
# 45 "/home/tinyos/local/src/tinyos-2.x/tos/chips/msp430/pins/HplMsp430GeneralIOP.nc"
static inline void /*HplMsp430GeneralIOC.P17*/HplMsp430GeneralIOP__7__IO__set(void )
#line 45
{
#line 45
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 45
    * (volatile uint8_t * )33U |= 0x01 << 7;
#line 45
    __nesc_atomic_end(__nesc_atomic); }
}

# 34 "/home/tinyos/local/src/tinyos-2.x/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
inline static void /*HplSensirionSht11C.PWRM*/Msp430GpioC__12__HplGeneralIO__set(void ){
#line 34
  /*HplMsp430GeneralIOC.P17*/HplMsp430GeneralIOP__7__IO__set();
#line 34
}
#line 34
# 37 "/home/tinyos/local/src/tinyos-2.x/tos/chips/msp430/pins/Msp430GpioC.nc"
static inline void /*HplSensirionSht11C.PWRM*/Msp430GpioC__12__GeneralIO__set(void )
#line 37
{
#line 37
  /*HplSensirionSht11C.PWRM*/Msp430GpioC__12__HplGeneralIO__set();
}

# 29 "/home/tinyos/local/src/tinyos-2.x/tos/interfaces/GeneralIO.nc"
inline static void HplSensirionSht11P__PWR__set(void ){
#line 29
  /*HplSensirionSht11C.PWRM*/Msp430GpioC__12__GeneralIO__set();
#line 29
}
#line 29
# 53 "/home/tinyos/local/src/tinyos-2.x/tos/lib/timer/Counter.nc"
inline static /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__Counter__size_type /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__Counter__get(void ){
#line 53
  unsigned long result;
#line 53

#line 53
  result = /*CounterMilli32C.Transform*/TransformCounterC__0__Counter__get();
#line 53

#line 53
  return result;
#line 53
}
#line 53
# 75 "/home/tinyos/local/src/tinyos-2.x/tos/lib/timer/TransformAlarmC.nc"
static inline /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__to_size_type /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__Alarm__getNow(void )
{
  return /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__Counter__get();
}

# 98 "/home/tinyos/local/src/tinyos-2.x/tos/lib/timer/Alarm.nc"
inline static /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__Alarm__size_type /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__Alarm__getNow(void ){
#line 98
  unsigned long result;
#line 98

#line 98
  result = /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__Alarm__getNow();
#line 98

#line 98
  return result;
#line 98
}
#line 98
# 85 "/home/tinyos/local/src/tinyos-2.x/tos/lib/timer/AlarmToTimerC.nc"
static inline uint32_t /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__Timer__getNow(void )
{
#line 86
  return /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__Alarm__getNow();
}

# 125 "/home/tinyos/local/src/tinyos-2.x/tos/lib/timer/Timer.nc"
inline static uint32_t /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__TimerFrom__getNow(void ){
#line 125
  unsigned long result;
#line 125

#line 125
  result = /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__Timer__getNow();
#line 125

#line 125
  return result;
#line 125
}
#line 125
# 148 "/home/tinyos/local/src/tinyos-2.x/tos/lib/timer/VirtualizeTimerC.nc"
static inline void /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__Timer__startOneShot(uint8_t num, uint32_t dt)
{
  /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__startTimer(num, /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__TimerFrom__getNow(), dt, TRUE);
}

# 62 "/home/tinyos/local/src/tinyos-2.x/tos/lib/timer/Timer.nc"
inline static void HplSensirionSht11P__Timer__startOneShot(uint32_t dt){
#line 62
  /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__Timer__startOneShot(10U, dt);
#line 62
}
#line 62
# 56 "/home/tinyos/local/src/tinyos-2.x/tos/interfaces/TaskBasic.nc"
inline static error_t /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__updateFromTimer__postTask(void ){
#line 56
  unsigned char result;
#line 56

#line 56
  result = SchedulerBasicP__TaskBasic__postTask(/*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__updateFromTimer);
#line 56

#line 56
  return result;
#line 56
}
#line 56
inline static error_t HplSensirionSht11P__stopTask__postTask(void ){
#line 56
  unsigned char result;
#line 56

#line 56
  result = SchedulerBasicP__TaskBasic__postTask(HplSensirionSht11P__stopTask);
#line 56

#line 56
  return result;
#line 56
}
#line 56
# 46 "/home/tinyos/local/src/tinyos-2.x/tos/chips/msp430/pins/HplMsp430GeneralIOP.nc"
static inline void /*HplMsp430GeneralIOC.P17*/HplMsp430GeneralIOP__7__IO__clr(void )
#line 46
{
#line 46
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 46
    * (volatile uint8_t * )33U &= ~(0x01 << 7);
#line 46
    __nesc_atomic_end(__nesc_atomic); }
}

# 39 "/home/tinyos/local/src/tinyos-2.x/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
inline static void /*HplSensirionSht11C.PWRM*/Msp430GpioC__12__HplGeneralIO__clr(void ){
#line 39
  /*HplMsp430GeneralIOC.P17*/HplMsp430GeneralIOP__7__IO__clr();
#line 39
}
#line 39
# 38 "/home/tinyos/local/src/tinyos-2.x/tos/chips/msp430/pins/Msp430GpioC.nc"
static inline void /*HplSensirionSht11C.PWRM*/Msp430GpioC__12__GeneralIO__clr(void )
#line 38
{
#line 38
  /*HplSensirionSht11C.PWRM*/Msp430GpioC__12__HplGeneralIO__clr();
}

# 30 "/home/tinyos/local/src/tinyos-2.x/tos/interfaces/GeneralIO.nc"
inline static void HplSensirionSht11P__PWR__clr(void ){
#line 30
  /*HplSensirionSht11C.PWRM*/Msp430GpioC__12__GeneralIO__clr();
#line 30
}
#line 30
# 39 "/home/tinyos/local/src/tinyos-2.x/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
inline static void /*HplSensirionSht11C.DATAM*/Msp430GpioC__10__HplGeneralIO__clr(void ){
#line 39
  /*HplMsp430GeneralIOC.P15*/HplMsp430GeneralIOP__5__IO__clr();
#line 39
}
#line 39
# 38 "/home/tinyos/local/src/tinyos-2.x/tos/chips/msp430/pins/Msp430GpioC.nc"
static inline void /*HplSensirionSht11C.DATAM*/Msp430GpioC__10__GeneralIO__clr(void )
#line 38
{
#line 38
  /*HplSensirionSht11C.DATAM*/Msp430GpioC__10__HplGeneralIO__clr();
}

# 30 "/home/tinyos/local/src/tinyos-2.x/tos/interfaces/GeneralIO.nc"
inline static void HplSensirionSht11P__DATA__clr(void ){
#line 30
  /*HplSensirionSht11C.DATAM*/Msp430GpioC__10__GeneralIO__clr();
#line 30
}
#line 30
# 64 "/home/tinyos/local/src/tinyos-2.x/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
inline static void /*HplSensirionSht11C.DATAM*/Msp430GpioC__10__HplGeneralIO__makeInput(void ){
#line 64
  /*HplMsp430GeneralIOC.P15*/HplMsp430GeneralIOP__5__IO__makeInput();
#line 64
}
#line 64
# 41 "/home/tinyos/local/src/tinyos-2.x/tos/chips/msp430/pins/Msp430GpioC.nc"
static inline void /*HplSensirionSht11C.DATAM*/Msp430GpioC__10__GeneralIO__makeInput(void )
#line 41
{
#line 41
  /*HplSensirionSht11C.DATAM*/Msp430GpioC__10__HplGeneralIO__makeInput();
}

# 33 "/home/tinyos/local/src/tinyos-2.x/tos/interfaces/GeneralIO.nc"
inline static void HplSensirionSht11P__DATA__makeInput(void ){
#line 33
  /*HplSensirionSht11C.DATAM*/Msp430GpioC__10__GeneralIO__makeInput();
#line 33
}
#line 33
# 39 "/home/tinyos/local/src/tinyos-2.x/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
inline static void /*HplSensirionSht11C.SCKM*/Msp430GpioC__11__HplGeneralIO__clr(void ){
#line 39
  /*HplMsp430GeneralIOC.P16*/HplMsp430GeneralIOP__6__IO__clr();
#line 39
}
#line 39
# 38 "/home/tinyos/local/src/tinyos-2.x/tos/chips/msp430/pins/Msp430GpioC.nc"
static inline void /*HplSensirionSht11C.SCKM*/Msp430GpioC__11__GeneralIO__clr(void )
#line 38
{
#line 38
  /*HplSensirionSht11C.SCKM*/Msp430GpioC__11__HplGeneralIO__clr();
}

# 30 "/home/tinyos/local/src/tinyos-2.x/tos/interfaces/GeneralIO.nc"
inline static void HplSensirionSht11P__SCK__clr(void ){
#line 30
  /*HplSensirionSht11C.SCKM*/Msp430GpioC__11__GeneralIO__clr();
#line 30
}
#line 30
# 50 "/home/tinyos/local/src/tinyos-2.x/tos/chips/msp430/pins/HplMsp430GeneralIOP.nc"
static inline void /*HplMsp430GeneralIOC.P16*/HplMsp430GeneralIOP__6__IO__makeInput(void )
#line 50
{
#line 50
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 50
    * (volatile uint8_t * )34U &= ~(0x01 << 6);
#line 50
    __nesc_atomic_end(__nesc_atomic); }
}

# 64 "/home/tinyos/local/src/tinyos-2.x/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
inline static void /*HplSensirionSht11C.SCKM*/Msp430GpioC__11__HplGeneralIO__makeInput(void ){
#line 64
  /*HplMsp430GeneralIOC.P16*/HplMsp430GeneralIOP__6__IO__makeInput();
#line 64
}
#line 64
# 41 "/home/tinyos/local/src/tinyos-2.x/tos/chips/msp430/pins/Msp430GpioC.nc"
static inline void /*HplSensirionSht11C.SCKM*/Msp430GpioC__11__GeneralIO__makeInput(void )
#line 41
{
#line 41
  /*HplSensirionSht11C.SCKM*/Msp430GpioC__11__HplGeneralIO__makeInput();
}

# 33 "/home/tinyos/local/src/tinyos-2.x/tos/interfaces/GeneralIO.nc"
inline static void HplSensirionSht11P__SCK__makeInput(void ){
#line 33
  /*HplSensirionSht11C.SCKM*/Msp430GpioC__11__GeneralIO__makeInput();
#line 33
}
#line 33
# 63 "/home/tinyos/local/src/tinyos-2.x/tos/platforms/telosa/chips/sht11/HplSensirionSht11P.nc"
static inline error_t HplSensirionSht11P__SplitControl__stop(void )
#line 63
{
  HplSensirionSht11P__SCK__makeInput();
  HplSensirionSht11P__SCK__clr();
  HplSensirionSht11P__DATA__makeInput();
  HplSensirionSht11P__DATA__clr();
  HplSensirionSht11P__PWR__clr();
  HplSensirionSht11P__stopTask__postTask();
  return SUCCESS;
}

# 109 "/home/tinyos/local/src/tinyos-2.x/tos/interfaces/SplitControl.nc"
inline static error_t /*HplSensirionSht11C.SplitControlPowerManagerC.PowerManager*/PowerManagerP__0__SplitControl__stop(void ){
#line 109
  unsigned char result;
#line 109

#line 109
  result = HplSensirionSht11P__SplitControl__stop();
#line 109

#line 109
  return result;
#line 109
}
#line 109
# 113 "/home/tinyos/local/src/tinyos-2.x/tos/lib/power/PowerManagerP.nc"
static inline error_t /*HplSensirionSht11C.SplitControlPowerManagerC.PowerManager*/PowerManagerP__0__StdControl__default__stop(void )
#line 113
{
  return SUCCESS;
}

# 84 "/home/tinyos/local/src/tinyos-2.x/tos/interfaces/StdControl.nc"
inline static error_t /*HplSensirionSht11C.SplitControlPowerManagerC.PowerManager*/PowerManagerP__0__StdControl__stop(void ){
#line 84
  unsigned char result;
#line 84

#line 84
  result = /*HplSensirionSht11C.SplitControlPowerManagerC.PowerManager*/PowerManagerP__0__StdControl__default__stop();
#line 84

#line 84
  return result;
#line 84
}
#line 84
# 121 "/home/tinyos/local/src/tinyos-2.x/tos/lib/power/PowerManagerP.nc"
static inline void /*HplSensirionSht11C.SplitControlPowerManagerC.PowerManager*/PowerManagerP__0__PowerDownCleanup__default__cleanup(void )
#line 121
{
}

# 52 "/home/tinyos/local/src/tinyos-2.x/tos/lib/power/PowerDownCleanup.nc"
inline static void /*HplSensirionSht11C.SplitControlPowerManagerC.PowerManager*/PowerManagerP__0__PowerDownCleanup__cleanup(void ){
#line 52
  /*HplSensirionSht11C.SplitControlPowerManagerC.PowerManager*/PowerManagerP__0__PowerDownCleanup__default__cleanup();
#line 52
}
#line 52
# 68 "/home/tinyos/local/src/tinyos-2.x/tos/lib/power/PowerManagerP.nc"
static inline void /*HplSensirionSht11C.SplitControlPowerManagerC.PowerManager*/PowerManagerP__0__stopTask__runTask(void )
#line 68
{
  /*HplSensirionSht11C.SplitControlPowerManagerC.PowerManager*/PowerManagerP__0__PowerDownCleanup__cleanup();
  /*HplSensirionSht11C.SplitControlPowerManagerC.PowerManager*/PowerManagerP__0__StdControl__stop();
  /*HplSensirionSht11C.SplitControlPowerManagerC.PowerManager*/PowerManagerP__0__SplitControl__stop();
}

# 100 "HttpdP.nc"
static inline void HttpdP__ReadTemp__readDone(error_t result, uint16_t data)
#line 100
{
  if (result == SUCCESS) {
#line 101
    HttpdP__temp = data;
    }
}

# 63 "/home/tinyos/local/src/tinyos-2.x/tos/interfaces/Read.nc"
inline static void /*TCPEchoC.Sht11C.SensirionSht11ReaderP*/SensirionSht11ReaderP__0__Temperature__readDone(error_t result, /*TCPEchoC.Sht11C.SensirionSht11ReaderP*/SensirionSht11ReaderP__0__Temperature__val_t val){
#line 63
  HttpdP__ReadTemp__readDone(result, val);
#line 63
}
#line 63
# 110 "/home/tinyos/local/src/tinyos-2.x/tos/interfaces/Resource.nc"
inline static error_t /*TCPEchoC.Sht11C.SensirionSht11ReaderP*/SensirionSht11ReaderP__0__TempResource__release(void ){
#line 110
  unsigned char result;
#line 110

#line 110
  result = /*HplSensirionSht11C.Arbiter.Arbiter*/ArbiterP__1__Resource__release(/*TCPEchoC.Sht11C*/SensirionSht11C__0__TEMP_KEY);
#line 110

#line 110
  return result;
#line 110
}
#line 110
# 113 "/home/tinyos/local/src/tinyos-2.x/tos/chips/sht11/SensirionSht11LogicP.nc"
static inline error_t /*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP__0__SensirionSht11__measureTemperature(uint8_t client)
#line 113
{
  if (!/*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP__0__on) {
#line 114
      return EOFF;
    }
#line 115
  if (/*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP__0__busy) {
#line 115
      return EBUSY;
    }
  else 
#line 115
    {
#line 115
      /*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP__0__busy = TRUE;
    }
#line 116
  /*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP__0__cmd = /*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP__0__CMD_MEASURE_TEMPERATURE;
  /*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP__0__currentClient = client;
  return /*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP__0__performCommand();
}

# 61 "/home/tinyos/local/src/tinyos-2.x/tos/chips/sht11/SensirionSht11.nc"
inline static error_t /*TCPEchoC.Sht11C.SensirionSht11ReaderP*/SensirionSht11ReaderP__0__Sht11Temp__measureTemperature(void ){
#line 61
  unsigned char result;
#line 61

#line 61
  result = /*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP__0__SensirionSht11__measureTemperature(/*TCPEchoC.Sht11C*/SensirionSht11C__0__TEMP_KEY);
#line 61

#line 61
  return result;
#line 61
}
#line 61
# 65 "/home/tinyos/local/src/tinyos-2.x/tos/chips/sht11/SensirionSht11ReaderP.nc"
static inline void /*TCPEchoC.Sht11C.SensirionSht11ReaderP*/SensirionSht11ReaderP__0__TempResource__granted(void )
#line 65
{
  error_t result;

#line 67
  if ((result = /*TCPEchoC.Sht11C.SensirionSht11ReaderP*/SensirionSht11ReaderP__0__Sht11Temp__measureTemperature()) != SUCCESS) {
      /*TCPEchoC.Sht11C.SensirionSht11ReaderP*/SensirionSht11ReaderP__0__TempResource__release();
      /*TCPEchoC.Sht11C.SensirionSht11ReaderP*/SensirionSht11ReaderP__0__Temperature__readDone(result, 0);
    }
}

#line 109
static inline void /*TCPEchoC.Sht11C.SensirionSht11ReaderP*/SensirionSht11ReaderP__0__Humidity__default__readDone(error_t result, uint16_t val)
#line 109
{
}

# 63 "/home/tinyos/local/src/tinyos-2.x/tos/interfaces/Read.nc"
inline static void /*TCPEchoC.Sht11C.SensirionSht11ReaderP*/SensirionSht11ReaderP__0__Humidity__readDone(error_t result, /*TCPEchoC.Sht11C.SensirionSht11ReaderP*/SensirionSht11ReaderP__0__Humidity__val_t val){
#line 63
  /*TCPEchoC.Sht11C.SensirionSht11ReaderP*/SensirionSht11ReaderP__0__Humidity__default__readDone(result, val);
#line 63
}
#line 63
# 110 "/home/tinyos/local/src/tinyos-2.x/tos/interfaces/Resource.nc"
inline static error_t /*TCPEchoC.Sht11C.SensirionSht11ReaderP*/SensirionSht11ReaderP__0__HumResource__release(void ){
#line 110
  unsigned char result;
#line 110

#line 110
  result = /*HplSensirionSht11C.Arbiter.Arbiter*/ArbiterP__1__Resource__release(/*TCPEchoC.Sht11C*/SensirionSht11C__0__HUM_KEY);
#line 110

#line 110
  return result;
#line 110
}
#line 110
# 121 "/home/tinyos/local/src/tinyos-2.x/tos/chips/sht11/SensirionSht11LogicP.nc"
static inline error_t /*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP__0__SensirionSht11__measureHumidity(uint8_t client)
#line 121
{
  if (!/*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP__0__on) {
#line 122
      return EOFF;
    }
#line 123
  if (/*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP__0__busy) {
#line 123
      return EBUSY;
    }
  else 
#line 123
    {
#line 123
      /*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP__0__busy = TRUE;
    }
#line 124
  /*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP__0__cmd = /*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP__0__CMD_MEASURE_HUMIDITY;
  /*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP__0__currentClient = client;
  return /*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP__0__performCommand();
}

# 76 "/home/tinyos/local/src/tinyos-2.x/tos/chips/sht11/SensirionSht11.nc"
inline static error_t /*TCPEchoC.Sht11C.SensirionSht11ReaderP*/SensirionSht11ReaderP__0__Sht11Hum__measureHumidity(void ){
#line 76
  unsigned char result;
#line 76

#line 76
  result = /*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP__0__SensirionSht11__measureHumidity(/*TCPEchoC.Sht11C*/SensirionSht11C__0__HUM_KEY);
#line 76

#line 76
  return result;
#line 76
}
#line 76
# 85 "/home/tinyos/local/src/tinyos-2.x/tos/chips/sht11/SensirionSht11ReaderP.nc"
static inline void /*TCPEchoC.Sht11C.SensirionSht11ReaderP*/SensirionSht11ReaderP__0__HumResource__granted(void )
#line 85
{
  error_t result;

#line 87
  if ((result = /*TCPEchoC.Sht11C.SensirionSht11ReaderP*/SensirionSht11ReaderP__0__Sht11Hum__measureHumidity()) != SUCCESS) {
      /*TCPEchoC.Sht11C.SensirionSht11ReaderP*/SensirionSht11ReaderP__0__HumResource__release();
      /*TCPEchoC.Sht11C.SensirionSht11ReaderP*/SensirionSht11ReaderP__0__Humidity__readDone(result, 0);
    }
}

# 199 "/home/tinyos/local/src/tinyos-2.x/tos/system/ArbiterP.nc"
static inline void /*HplSensirionSht11C.Arbiter.Arbiter*/ArbiterP__1__Resource__default__granted(uint8_t id)
#line 199
{
}

# 92 "/home/tinyos/local/src/tinyos-2.x/tos/interfaces/Resource.nc"
inline static void /*HplSensirionSht11C.Arbiter.Arbiter*/ArbiterP__1__Resource__granted(uint8_t arg_0x40e33310){
#line 92
  switch (arg_0x40e33310) {
#line 92
    case /*TCPEchoC.Sht11C*/SensirionSht11C__0__TEMP_KEY:
#line 92
      /*TCPEchoC.Sht11C.SensirionSht11ReaderP*/SensirionSht11ReaderP__0__TempResource__granted();
#line 92
      break;
#line 92
    case /*TCPEchoC.Sht11C*/SensirionSht11C__0__HUM_KEY:
#line 92
      /*TCPEchoC.Sht11C.SensirionSht11ReaderP*/SensirionSht11ReaderP__0__HumResource__granted();
#line 92
      break;
#line 92
    default:
#line 92
      /*HplSensirionSht11C.Arbiter.Arbiter*/ArbiterP__1__Resource__default__granted(arg_0x40e33310);
#line 92
      break;
#line 92
    }
#line 92
}
#line 92
# 213 "/home/tinyos/local/src/tinyos-2.x/tos/system/ArbiterP.nc"
static inline void /*HplSensirionSht11C.Arbiter.Arbiter*/ArbiterP__1__ResourceConfigure__default__configure(uint8_t id)
#line 213
{
}

# 49 "/home/tinyos/local/src/tinyos-2.x/tos/interfaces/ResourceConfigure.nc"
inline static void /*HplSensirionSht11C.Arbiter.Arbiter*/ArbiterP__1__ResourceConfigure__configure(uint8_t arg_0x40e31148){
#line 49
    /*HplSensirionSht11C.Arbiter.Arbiter*/ArbiterP__1__ResourceConfigure__default__configure(arg_0x40e31148);
#line 49
}
#line 49
# 187 "/home/tinyos/local/src/tinyos-2.x/tos/system/ArbiterP.nc"
static inline void /*HplSensirionSht11C.Arbiter.Arbiter*/ArbiterP__1__grantedTask__runTask(void )
#line 187
{
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 188
    {
      /*HplSensirionSht11C.Arbiter.Arbiter*/ArbiterP__1__resId = /*HplSensirionSht11C.Arbiter.Arbiter*/ArbiterP__1__reqResId;
      /*HplSensirionSht11C.Arbiter.Arbiter*/ArbiterP__1__state = /*HplSensirionSht11C.Arbiter.Arbiter*/ArbiterP__1__RES_BUSY;
    }
#line 191
    __nesc_atomic_end(__nesc_atomic); }
  /*HplSensirionSht11C.Arbiter.Arbiter*/ArbiterP__1__ResourceConfigure__configure(/*HplSensirionSht11C.Arbiter.Arbiter*/ArbiterP__1__resId);
  /*HplSensirionSht11C.Arbiter.Arbiter*/ArbiterP__1__Resource__granted(/*HplSensirionSht11C.Arbiter.Arbiter*/ArbiterP__1__resId);
}

# 50 "/home/tinyos/local/src/tinyos-2.x/tos/interfaces/GpioInterrupt.nc"
inline static error_t /*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP__0__InterruptDATA__disable(void ){
#line 50
  unsigned char result;
#line 50

#line 50
  result = /*HplSensirionSht11C.InterruptDATAC*/Msp430InterruptC__2__Interrupt__disable();
#line 50

#line 50
  return result;
#line 50
}
#line 50
# 34 "/home/tinyos/local/src/tinyos-2.x/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
inline static void /*HplSensirionSht11C.DATAM*/Msp430GpioC__10__HplGeneralIO__set(void ){
#line 34
  /*HplMsp430GeneralIOC.P15*/HplMsp430GeneralIOP__5__IO__set();
#line 34
}
#line 34
# 37 "/home/tinyos/local/src/tinyos-2.x/tos/chips/msp430/pins/Msp430GpioC.nc"
static inline void /*HplSensirionSht11C.DATAM*/Msp430GpioC__10__GeneralIO__set(void )
#line 37
{
#line 37
  /*HplSensirionSht11C.DATAM*/Msp430GpioC__10__HplGeneralIO__set();
}

# 29 "/home/tinyos/local/src/tinyos-2.x/tos/interfaces/GeneralIO.nc"
inline static void /*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP__0__DATA__set(void ){
#line 29
  /*HplSensirionSht11C.DATAM*/Msp430GpioC__10__GeneralIO__set();
#line 29
}
#line 29




inline static void /*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP__0__DATA__makeInput(void ){
#line 33
  /*HplSensirionSht11C.DATAM*/Msp430GpioC__10__GeneralIO__makeInput();
#line 33
}
#line 33
#line 30
inline static void /*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP__0__CLOCK__clr(void ){
#line 30
  /*HplSensirionSht11C.SCKM*/Msp430GpioC__11__GeneralIO__clr();
#line 30
}
#line 30
# 52 "/home/tinyos/local/src/tinyos-2.x/tos/chips/msp430/pins/HplMsp430GeneralIOP.nc"
static inline void /*HplMsp430GeneralIOC.P16*/HplMsp430GeneralIOP__6__IO__makeOutput(void )
#line 52
{
#line 52
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 52
    * (volatile uint8_t * )34U |= 0x01 << 6;
#line 52
    __nesc_atomic_end(__nesc_atomic); }
}

# 71 "/home/tinyos/local/src/tinyos-2.x/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
inline static void /*HplSensirionSht11C.SCKM*/Msp430GpioC__11__HplGeneralIO__makeOutput(void ){
#line 71
  /*HplMsp430GeneralIOC.P16*/HplMsp430GeneralIOP__6__IO__makeOutput();
#line 71
}
#line 71
# 43 "/home/tinyos/local/src/tinyos-2.x/tos/chips/msp430/pins/Msp430GpioC.nc"
static inline void /*HplSensirionSht11C.SCKM*/Msp430GpioC__11__GeneralIO__makeOutput(void )
#line 43
{
#line 43
  /*HplSensirionSht11C.SCKM*/Msp430GpioC__11__HplGeneralIO__makeOutput();
}

# 35 "/home/tinyos/local/src/tinyos-2.x/tos/interfaces/GeneralIO.nc"
inline static void /*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP__0__CLOCK__makeOutput(void ){
#line 35
  /*HplSensirionSht11C.SCKM*/Msp430GpioC__11__GeneralIO__makeOutput();
#line 35
}
#line 35
# 220 "/home/tinyos/local/src/tinyos-2.x/tos/chips/sht11/SensirionSht11LogicP.nc"
static inline void /*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP__0__initPins(void )
#line 220
{
  /*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP__0__CLOCK__makeOutput();
  /*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP__0__CLOCK__clr();
  /*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP__0__DATA__makeInput();
  /*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP__0__DATA__set();
  /*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP__0__InterruptDATA__disable();
}

# 88 "/home/tinyos/local/src/tinyos-2.x/tos/chips/msp430/pins/HplMsp430InterruptP.nc"
static inline void HplMsp430InterruptP__Port15__disable(void )
#line 88
{
#line 88
  P1IE &= ~(1 << 5);
}

# 36 "/home/tinyos/local/src/tinyos-2.x/tos/chips/msp430/pins/HplMsp430Interrupt.nc"
inline static void /*HplSensirionSht11C.InterruptDATAC*/Msp430InterruptC__2__HplInterrupt__disable(void ){
#line 36
  HplMsp430InterruptP__Port15__disable();
#line 36
}
#line 36
# 34 "/home/tinyos/local/src/tinyos-2.x/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
inline static void /*HplSensirionSht11C.SCKM*/Msp430GpioC__11__HplGeneralIO__set(void ){
#line 34
  /*HplMsp430GeneralIOC.P16*/HplMsp430GeneralIOP__6__IO__set();
#line 34
}
#line 34
# 37 "/home/tinyos/local/src/tinyos-2.x/tos/chips/msp430/pins/Msp430GpioC.nc"
static inline void /*HplSensirionSht11C.SCKM*/Msp430GpioC__11__GeneralIO__set(void )
#line 37
{
#line 37
  /*HplSensirionSht11C.SCKM*/Msp430GpioC__11__HplGeneralIO__set();
}

# 29 "/home/tinyos/local/src/tinyos-2.x/tos/interfaces/GeneralIO.nc"
inline static void /*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP__0__CLOCK__set(void ){
#line 29
  /*HplSensirionSht11C.SCKM*/Msp430GpioC__11__GeneralIO__set();
#line 29
}
#line 29
# 71 "/home/tinyos/local/src/tinyos-2.x/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
inline static void /*HplSensirionSht11C.DATAM*/Msp430GpioC__10__HplGeneralIO__makeOutput(void ){
#line 71
  /*HplMsp430GeneralIOC.P15*/HplMsp430GeneralIOP__5__IO__makeOutput();
#line 71
}
#line 71
# 43 "/home/tinyos/local/src/tinyos-2.x/tos/chips/msp430/pins/Msp430GpioC.nc"
static inline void /*HplSensirionSht11C.DATAM*/Msp430GpioC__10__GeneralIO__makeOutput(void )
#line 43
{
#line 43
  /*HplSensirionSht11C.DATAM*/Msp430GpioC__10__HplGeneralIO__makeOutput();
}

# 35 "/home/tinyos/local/src/tinyos-2.x/tos/interfaces/GeneralIO.nc"
inline static void /*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP__0__DATA__makeOutput(void ){
#line 35
  /*HplSensirionSht11C.DATAM*/Msp430GpioC__10__GeneralIO__makeOutput();
#line 35
}
#line 35
# 228 "/home/tinyos/local/src/tinyos-2.x/tos/chips/sht11/SensirionSht11LogicP.nc"
static inline void /*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP__0__resetDevice(void )
#line 228
{
  uint8_t i;

#line 230
  /*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP__0__DATA__makeOutput();
  /*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP__0__DATA__set();
  /*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP__0__CLOCK__clr();
  for (i = 0; i < 9; i++) {
      /*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP__0__CLOCK__set();
      /*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP__0__CLOCK__clr();
    }
}

# 30 "/home/tinyos/local/src/tinyos-2.x/tos/interfaces/GeneralIO.nc"
inline static void /*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP__0__DATA__clr(void ){
#line 30
  /*HplSensirionSht11C.DATAM*/Msp430GpioC__10__GeneralIO__clr();
#line 30
}
#line 30
# 239 "/home/tinyos/local/src/tinyos-2.x/tos/chips/sht11/SensirionSht11LogicP.nc"
static inline void /*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP__0__transmissionStart(void )
#line 239
{
  /*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP__0__DATA__makeOutput();
  /*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP__0__DATA__set();
  /*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP__0__CLOCK__clr();
  /*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP__0__CLOCK__set();
  /*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP__0__DATA__clr();
  /*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP__0__CLOCK__clr();
  /*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP__0__CLOCK__set();
  /*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP__0__DATA__set();
  /*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP__0__CLOCK__clr();
}

static inline void /*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP__0__sendCommand(uint8_t _cmd)
#line 251
{
  /*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP__0__writeByte(_cmd);
}

# 48 "/home/tinyos/local/src/tinyos-2.x/tos/chips/msp430/pins/HplMsp430GeneralIOP.nc"
static inline uint8_t /*HplMsp430GeneralIOC.P15*/HplMsp430GeneralIOP__5__IO__getRaw(void )
#line 48
{
#line 48
  return * (volatile uint8_t * )32U & (0x01 << 5);
}

#line 49
static inline bool /*HplMsp430GeneralIOC.P15*/HplMsp430GeneralIOP__5__IO__get(void )
#line 49
{
#line 49
  return /*HplMsp430GeneralIOC.P15*/HplMsp430GeneralIOP__5__IO__getRaw() != 0;
}

# 59 "/home/tinyos/local/src/tinyos-2.x/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
inline static bool /*HplSensirionSht11C.DATAM*/Msp430GpioC__10__HplGeneralIO__get(void ){
#line 59
  unsigned char result;
#line 59

#line 59
  result = /*HplMsp430GeneralIOC.P15*/HplMsp430GeneralIOP__5__IO__get();
#line 59

#line 59
  return result;
#line 59
}
#line 59
# 40 "/home/tinyos/local/src/tinyos-2.x/tos/chips/msp430/pins/Msp430GpioC.nc"
static inline bool /*HplSensirionSht11C.DATAM*/Msp430GpioC__10__GeneralIO__get(void )
#line 40
{
#line 40
  return /*HplSensirionSht11C.DATAM*/Msp430GpioC__10__HplGeneralIO__get();
}

# 32 "/home/tinyos/local/src/tinyos-2.x/tos/interfaces/GeneralIO.nc"
inline static bool /*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP__0__DATA__get(void ){
#line 32
  unsigned char result;
#line 32

#line 32
  result = /*HplSensirionSht11C.DATAM*/Msp430GpioC__10__GeneralIO__get();
#line 32

#line 32
  return result;
#line 32
}
#line 32
# 80 "/home/tinyos/local/src/tinyos-2.x/tos/chips/msp430/pins/HplMsp430InterruptP.nc"
static inline void HplMsp430InterruptP__Port15__enable(void )
#line 80
{
#line 80
  P1IE |= 1 << 5;
}

# 31 "/home/tinyos/local/src/tinyos-2.x/tos/chips/msp430/pins/HplMsp430Interrupt.nc"
inline static void /*HplSensirionSht11C.InterruptDATAC*/Msp430InterruptC__2__HplInterrupt__enable(void ){
#line 31
  HplMsp430InterruptP__Port15__enable();
#line 31
}
#line 31
# 137 "/home/tinyos/local/src/tinyos-2.x/tos/chips/msp430/pins/HplMsp430InterruptP.nc"
static inline void HplMsp430InterruptP__Port15__edge(bool l2h)
#line 137
{
  /* atomic removed: atomic calls only */
#line 138
  {
    if (l2h) {
#line 139
      P1IES &= ~(1 << 5);
      }
    else {
#line 140
      P1IES |= 1 << 5;
      }
  }
}

# 56 "/home/tinyos/local/src/tinyos-2.x/tos/chips/msp430/pins/HplMsp430Interrupt.nc"
inline static void /*HplSensirionSht11C.InterruptDATAC*/Msp430InterruptC__2__HplInterrupt__edge(bool low_to_high){
#line 56
  HplMsp430InterruptP__Port15__edge(low_to_high);
#line 56
}
#line 56
# 41 "/home/tinyos/local/src/tinyos-2.x/tos/chips/msp430/pins/Msp430InterruptC.nc"
static inline error_t /*HplSensirionSht11C.InterruptDATAC*/Msp430InterruptC__2__enable(bool rising)
#line 41
{
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 42
    {
      /*HplSensirionSht11C.InterruptDATAC*/Msp430InterruptC__2__Interrupt__disable();
      /*HplSensirionSht11C.InterruptDATAC*/Msp430InterruptC__2__HplInterrupt__edge(rising);
      /*HplSensirionSht11C.InterruptDATAC*/Msp430InterruptC__2__HplInterrupt__enable();
    }
#line 46
    __nesc_atomic_end(__nesc_atomic); }
  return SUCCESS;
}





static inline error_t /*HplSensirionSht11C.InterruptDATAC*/Msp430InterruptC__2__Interrupt__enableFallingEdge(void )
#line 54
{
  return /*HplSensirionSht11C.InterruptDATAC*/Msp430InterruptC__2__enable(FALSE);
}

# 43 "/home/tinyos/local/src/tinyos-2.x/tos/interfaces/GpioInterrupt.nc"
inline static error_t /*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP__0__InterruptDATA__enableFallingEdge(void ){
#line 43
  unsigned char result;
#line 43

#line 43
  result = /*HplSensirionSht11C.InterruptDATAC*/Msp430InterruptC__2__Interrupt__enableFallingEdge();
#line 43

#line 43
  return result;
#line 43
}
#line 43
# 372 "/home/tinyos/local/src/tinyos-2.x/tos/chips/sht11/SensirionSht11LogicP.nc"
static inline void /*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP__0__ack(void )
#line 372
{
  /*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP__0__DATA__makeOutput();
  /*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP__0__DATA__clr();
  /*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP__0__CLOCK__set();
  /*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP__0__CLOCK__clr();
  /*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP__0__DATA__makeInput();
  /*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP__0__DATA__set();
}

# 56 "/home/tinyos/local/src/tinyos-2.x/tos/interfaces/TaskBasic.nc"
inline static error_t /*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP__0__signalStatusDone__postTask(void ){
#line 56
  unsigned char result;
#line 56

#line 56
  result = SchedulerBasicP__TaskBasic__postTask(/*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP__0__signalStatusDone);
#line 56

#line 56
  return result;
#line 56
}
#line 56
# 50 "/home/tinyos/local/src/tinyos-2.x/tos/system/FcfsResourceQueueC.nc"
static inline bool /*HplSensirionSht11C.Arbiter.Queue*/FcfsResourceQueueC__2__FcfsQueue__isEmpty(void )
#line 50
{
  /* atomic removed: atomic calls only */
#line 51
  {
    unsigned char __nesc_temp = 
#line 51
    /*HplSensirionSht11C.Arbiter.Queue*/FcfsResourceQueueC__2__qHead == /*HplSensirionSht11C.Arbiter.Queue*/FcfsResourceQueueC__2__NO_ENTRY;

#line 51
    return __nesc_temp;
  }
}

# 43 "/home/tinyos/local/src/tinyos-2.x/tos/interfaces/ResourceQueue.nc"
inline static bool /*HplSensirionSht11C.Arbiter.Arbiter*/ArbiterP__1__Queue__isEmpty(void ){
#line 43
  unsigned char result;
#line 43

#line 43
  result = /*HplSensirionSht11C.Arbiter.Queue*/FcfsResourceQueueC__2__FcfsQueue__isEmpty();
#line 43

#line 43
  return result;
#line 43
}
#line 43
# 58 "/home/tinyos/local/src/tinyos-2.x/tos/system/FcfsResourceQueueC.nc"
static inline resource_client_id_t /*HplSensirionSht11C.Arbiter.Queue*/FcfsResourceQueueC__2__FcfsQueue__dequeue(void )
#line 58
{
  /* atomic removed: atomic calls only */
#line 59
  {
    if (/*HplSensirionSht11C.Arbiter.Queue*/FcfsResourceQueueC__2__qHead != /*HplSensirionSht11C.Arbiter.Queue*/FcfsResourceQueueC__2__NO_ENTRY) {
        uint8_t id = /*HplSensirionSht11C.Arbiter.Queue*/FcfsResourceQueueC__2__qHead;

#line 62
        /*HplSensirionSht11C.Arbiter.Queue*/FcfsResourceQueueC__2__qHead = /*HplSensirionSht11C.Arbiter.Queue*/FcfsResourceQueueC__2__resQ[/*HplSensirionSht11C.Arbiter.Queue*/FcfsResourceQueueC__2__qHead];
        if (/*HplSensirionSht11C.Arbiter.Queue*/FcfsResourceQueueC__2__qHead == /*HplSensirionSht11C.Arbiter.Queue*/FcfsResourceQueueC__2__NO_ENTRY) {
          /*HplSensirionSht11C.Arbiter.Queue*/FcfsResourceQueueC__2__qTail = /*HplSensirionSht11C.Arbiter.Queue*/FcfsResourceQueueC__2__NO_ENTRY;
          }
#line 65
        /*HplSensirionSht11C.Arbiter.Queue*/FcfsResourceQueueC__2__resQ[id] = /*HplSensirionSht11C.Arbiter.Queue*/FcfsResourceQueueC__2__NO_ENTRY;
        {
          unsigned char __nesc_temp = 
#line 66
          id;

#line 66
          return __nesc_temp;
        }
      }
#line 68
    {
      unsigned char __nesc_temp = 
#line 68
      /*HplSensirionSht11C.Arbiter.Queue*/FcfsResourceQueueC__2__NO_ENTRY;

#line 68
      return __nesc_temp;
    }
  }
}

# 60 "/home/tinyos/local/src/tinyos-2.x/tos/interfaces/ResourceQueue.nc"
inline static resource_client_id_t /*HplSensirionSht11C.Arbiter.Arbiter*/ArbiterP__1__Queue__dequeue(void ){
#line 60
  unsigned char result;
#line 60

#line 60
  result = /*HplSensirionSht11C.Arbiter.Queue*/FcfsResourceQueueC__2__FcfsQueue__dequeue();
#line 60

#line 60
  return result;
#line 60
}
#line 60
# 215 "/home/tinyos/local/src/tinyos-2.x/tos/system/ArbiterP.nc"
static inline void /*HplSensirionSht11C.Arbiter.Arbiter*/ArbiterP__1__ResourceConfigure__default__unconfigure(uint8_t id)
#line 215
{
}

# 55 "/home/tinyos/local/src/tinyos-2.x/tos/interfaces/ResourceConfigure.nc"
inline static void /*HplSensirionSht11C.Arbiter.Arbiter*/ArbiterP__1__ResourceConfigure__unconfigure(uint8_t arg_0x40e31148){
#line 55
    /*HplSensirionSht11C.Arbiter.Arbiter*/ArbiterP__1__ResourceConfigure__default__unconfigure(arg_0x40e31148);
#line 55
}
#line 55
# 56 "/home/tinyos/local/src/tinyos-2.x/tos/interfaces/TaskBasic.nc"
inline static error_t /*HplSensirionSht11C.SplitControlPowerManagerC.PowerManager*/PowerManagerP__0__stopTask__postTask(void ){
#line 56
  unsigned char result;
#line 56

#line 56
  result = SchedulerBasicP__TaskBasic__postTask(/*HplSensirionSht11C.SplitControlPowerManagerC.PowerManager*/PowerManagerP__0__stopTask);
#line 56

#line 56
  return result;
#line 56
}
#line 56
# 97 "/home/tinyos/local/src/tinyos-2.x/tos/lib/power/PowerManagerP.nc"
static inline void /*HplSensirionSht11C.SplitControlPowerManagerC.PowerManager*/PowerManagerP__0__ResourceDefaultOwner__granted(void )
#line 97
{
  /* atomic removed: atomic calls only */
#line 98
  /*HplSensirionSht11C.SplitControlPowerManagerC.PowerManager*/PowerManagerP__0__stopping = TRUE;
  /*HplSensirionSht11C.SplitControlPowerManagerC.PowerManager*/PowerManagerP__0__stopTask__postTask();
}

# 46 "/home/tinyos/local/src/tinyos-2.x/tos/interfaces/ResourceDefaultOwner.nc"
inline static void /*HplSensirionSht11C.Arbiter.Arbiter*/ArbiterP__1__ResourceDefaultOwner__granted(void ){
#line 46
  /*HplSensirionSht11C.SplitControlPowerManagerC.PowerManager*/PowerManagerP__0__ResourceDefaultOwner__granted();
#line 46
}
#line 46
# 102 "/home/tinyos/local/src/tinyos-2.x/tos/lib/power/PowerManagerP.nc"
static inline void /*HplSensirionSht11C.SplitControlPowerManagerC.PowerManager*/PowerManagerP__0__SplitControl__stopDone(error_t error)
#line 102
{
  if (/*HplSensirionSht11C.SplitControlPowerManagerC.PowerManager*/PowerManagerP__0__requested == TRUE) {
      /*HplSensirionSht11C.SplitControlPowerManagerC.PowerManager*/PowerManagerP__0__StdControl__start();
      /*HplSensirionSht11C.SplitControlPowerManagerC.PowerManager*/PowerManagerP__0__SplitControl__start();
    }
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 107
    {
      /*HplSensirionSht11C.SplitControlPowerManagerC.PowerManager*/PowerManagerP__0__requested = FALSE;
      /*HplSensirionSht11C.SplitControlPowerManagerC.PowerManager*/PowerManagerP__0__stopping = FALSE;
    }
#line 110
    __nesc_atomic_end(__nesc_atomic); }
}

# 117 "/home/tinyos/local/src/tinyos-2.x/tos/interfaces/SplitControl.nc"
inline static void HplSensirionSht11P__SplitControl__stopDone(error_t error){
#line 117
  /*HplSensirionSht11C.SplitControlPowerManagerC.PowerManager*/PowerManagerP__0__SplitControl__stopDone(error);
#line 117
}
#line 117
# 73 "/home/tinyos/local/src/tinyos-2.x/tos/platforms/telosa/chips/sht11/HplSensirionSht11P.nc"
static inline void HplSensirionSht11P__stopTask__runTask(void )
#line 73
{
  HplSensirionSht11P__SplitControl__stopDone(SUCCESS);
}

# 99 "/home/tinyos/local/src/tinyos-2.x/tos/chips/sht11/SensirionSht11ReaderP.nc"
static inline void /*TCPEchoC.Sht11C.SensirionSht11ReaderP*/SensirionSht11ReaderP__0__Sht11Temp__measureHumidityDone(error_t result, uint16_t val)
#line 99
{
}

#line 93
static inline void /*TCPEchoC.Sht11C.SensirionSht11ReaderP*/SensirionSht11ReaderP__0__Sht11Hum__measureHumidityDone(error_t result, uint16_t val)
#line 93
{
  /*TCPEchoC.Sht11C.SensirionSht11ReaderP*/SensirionSht11ReaderP__0__HumResource__release();
  /*TCPEchoC.Sht11C.SensirionSht11ReaderP*/SensirionSht11ReaderP__0__Humidity__readDone(result, val);
}

# 408 "/home/tinyos/local/src/tinyos-2.x/tos/chips/sht11/SensirionSht11LogicP.nc"
static inline void /*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP__0__SensirionSht11__default__measureHumidityDone(uint8_t client, error_t result, uint16_t val)
#line 408
{
}

# 84 "/home/tinyos/local/src/tinyos-2.x/tos/chips/sht11/SensirionSht11.nc"
inline static void /*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP__0__SensirionSht11__measureHumidityDone(uint8_t arg_0x4150f970, error_t result, uint16_t val){
#line 84
  switch (arg_0x4150f970) {
#line 84
    case /*TCPEchoC.Sht11C*/SensirionSht11C__0__TEMP_KEY:
#line 84
      /*TCPEchoC.Sht11C.SensirionSht11ReaderP*/SensirionSht11ReaderP__0__Sht11Temp__measureHumidityDone(result, val);
#line 84
      break;
#line 84
    case /*TCPEchoC.Sht11C*/SensirionSht11C__0__HUM_KEY:
#line 84
      /*TCPEchoC.Sht11C.SensirionSht11ReaderP*/SensirionSht11ReaderP__0__Sht11Hum__measureHumidityDone(result, val);
#line 84
      break;
#line 84
    default:
#line 84
      /*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP__0__SensirionSht11__default__measureHumidityDone(arg_0x4150f970, result, val);
#line 84
      break;
#line 84
    }
#line 84
}
#line 84
# 104 "/home/tinyos/local/src/tinyos-2.x/tos/chips/sht11/SensirionSht11ReaderP.nc"
static inline void /*TCPEchoC.Sht11C.SensirionSht11ReaderP*/SensirionSht11ReaderP__0__Sht11Hum__measureTemperatureDone(error_t result, uint16_t val)
#line 104
{
}

# 407 "/home/tinyos/local/src/tinyos-2.x/tos/chips/sht11/SensirionSht11LogicP.nc"
static inline void /*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP__0__SensirionSht11__default__measureTemperatureDone(uint8_t client, error_t result, uint16_t val)
#line 407
{
}

# 69 "/home/tinyos/local/src/tinyos-2.x/tos/chips/sht11/SensirionSht11.nc"
inline static void /*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP__0__SensirionSht11__measureTemperatureDone(uint8_t arg_0x4150f970, error_t result, uint16_t val){
#line 69
  switch (arg_0x4150f970) {
#line 69
    case /*TCPEchoC.Sht11C*/SensirionSht11C__0__TEMP_KEY:
#line 69
      /*TCPEchoC.Sht11C.SensirionSht11ReaderP*/SensirionSht11ReaderP__0__Sht11Temp__measureTemperatureDone(result, val);
#line 69
      break;
#line 69
    case /*TCPEchoC.Sht11C*/SensirionSht11C__0__HUM_KEY:
#line 69
      /*TCPEchoC.Sht11C.SensirionSht11ReaderP*/SensirionSht11ReaderP__0__Sht11Hum__measureTemperatureDone(result, val);
#line 69
      break;
#line 69
    default:
#line 69
      /*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP__0__SensirionSht11__default__measureTemperatureDone(arg_0x4150f970, result, val);
#line 69
      break;
#line 69
    }
#line 69
}
#line 69
# 153 "/home/tinyos/local/src/tinyos-2.x/tos/lib/timer/VirtualizeTimerC.nc"
static inline void /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__Timer__stop(uint8_t num)
{
  /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__m_timers[num].isrunning = FALSE;
}

# 67 "/home/tinyos/local/src/tinyos-2.x/tos/lib/timer/Timer.nc"
inline static void /*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP__0__Timer__stop(void ){
#line 67
  /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__Timer__stop(11U);
#line 67
}
#line 67
# 320 "/home/tinyos/local/src/tinyos-2.x/tos/chips/sht11/SensirionSht11LogicP.nc"
static inline void /*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP__0__readSensor__runTask(void )
#line 320
{
  uint16_t data = 0;
  uint8_t crc = 0;

  if (/*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP__0__busy == FALSE) {


      return;
    }

  /*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP__0__Timer__stop();

  data = /*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP__0__readByte() << 8;
  data |= /*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP__0__readByte();

  crc = /*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP__0__readByte();

  /*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP__0__endTransmission();

  switch (/*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP__0__cmd) {
      case /*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP__0__CMD_MEASURE_TEMPERATURE: 
        /*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP__0__busy = FALSE;
      /*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP__0__SensirionSht11__measureTemperatureDone(/*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP__0__currentClient, SUCCESS, data);
      break;

      case /*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP__0__CMD_MEASURE_HUMIDITY: 
        /*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP__0__busy = FALSE;
      /*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP__0__SensirionSht11__measureHumidityDone(/*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP__0__currentClient, SUCCESS, data);
      break;

      default: 
        break;
    }
}

# 101 "/home/tinyos/local/src/tinyos-2.x/tos/chips/sht11/SensirionSht11ReaderP.nc"
static inline void /*TCPEchoC.Sht11C.SensirionSht11ReaderP*/SensirionSht11ReaderP__0__Sht11Temp__writeStatusRegDone(error_t result)
#line 101
{
}



static inline void /*TCPEchoC.Sht11C.SensirionSht11ReaderP*/SensirionSht11ReaderP__0__Sht11Hum__writeStatusRegDone(error_t result)
#line 106
{
}

# 410 "/home/tinyos/local/src/tinyos-2.x/tos/chips/sht11/SensirionSht11LogicP.nc"
static inline void /*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP__0__SensirionSht11__default__writeStatusRegDone(uint8_t client, error_t result)
#line 410
{
}

# 116 "/home/tinyos/local/src/tinyos-2.x/tos/chips/sht11/SensirionSht11.nc"
inline static void /*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP__0__SensirionSht11__writeStatusRegDone(uint8_t arg_0x4150f970, error_t result){
#line 116
  switch (arg_0x4150f970) {
#line 116
    case /*TCPEchoC.Sht11C*/SensirionSht11C__0__TEMP_KEY:
#line 116
      /*TCPEchoC.Sht11C.SensirionSht11ReaderP*/SensirionSht11ReaderP__0__Sht11Temp__writeStatusRegDone(result);
#line 116
      break;
#line 116
    case /*TCPEchoC.Sht11C*/SensirionSht11C__0__HUM_KEY:
#line 116
      /*TCPEchoC.Sht11C.SensirionSht11ReaderP*/SensirionSht11ReaderP__0__Sht11Hum__writeStatusRegDone(result);
#line 116
      break;
#line 116
    default:
#line 116
      /*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP__0__SensirionSht11__default__writeStatusRegDone(arg_0x4150f970, result);
#line 116
      break;
#line 116
    }
#line 116
}
#line 116
# 100 "/home/tinyos/local/src/tinyos-2.x/tos/chips/sht11/SensirionSht11ReaderP.nc"
static inline void /*TCPEchoC.Sht11C.SensirionSht11ReaderP*/SensirionSht11ReaderP__0__Sht11Temp__readStatusRegDone(error_t result, uint8_t val)
#line 100
{
}



static inline void /*TCPEchoC.Sht11C.SensirionSht11ReaderP*/SensirionSht11ReaderP__0__Sht11Hum__readStatusRegDone(error_t result, uint8_t val)
#line 105
{
}

# 409 "/home/tinyos/local/src/tinyos-2.x/tos/chips/sht11/SensirionSht11LogicP.nc"
static inline void /*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP__0__SensirionSht11__default__readStatusRegDone(uint8_t client, error_t result, uint8_t val)
#line 409
{
}

# 100 "/home/tinyos/local/src/tinyos-2.x/tos/chips/sht11/SensirionSht11.nc"
inline static void /*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP__0__SensirionSht11__readStatusRegDone(uint8_t arg_0x4150f970, error_t result, uint8_t val){
#line 100
  switch (arg_0x4150f970) {
#line 100
    case /*TCPEchoC.Sht11C*/SensirionSht11C__0__TEMP_KEY:
#line 100
      /*TCPEchoC.Sht11C.SensirionSht11ReaderP*/SensirionSht11ReaderP__0__Sht11Temp__readStatusRegDone(result, val);
#line 100
      break;
#line 100
    case /*TCPEchoC.Sht11C*/SensirionSht11C__0__HUM_KEY:
#line 100
      /*TCPEchoC.Sht11C.SensirionSht11ReaderP*/SensirionSht11ReaderP__0__Sht11Hum__readStatusRegDone(result, val);
#line 100
      break;
#line 100
    default:
#line 100
      /*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP__0__SensirionSht11__default__readStatusRegDone(arg_0x4150f970, result, val);
#line 100
      break;
#line 100
    }
#line 100
}
#line 100
# 388 "/home/tinyos/local/src/tinyos-2.x/tos/chips/sht11/SensirionSht11LogicP.nc"
static inline void /*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP__0__signalStatusDone__runTask(void )
#line 388
{
  bool _writeFail = /*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP__0__writeFail;

#line 390
  switch (/*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP__0__cmd) {
      case /*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP__0__CMD_READ_STATUS: 
        /*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP__0__busy = FALSE;
      /*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP__0__SensirionSht11__readStatusRegDone(/*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP__0__currentClient, SUCCESS, /*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP__0__status);
      break;
      case /*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP__0__CMD_WRITE_STATUS: 
        /*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP__0__busy = FALSE;
      /*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP__0__writeFail = FALSE;
      /*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP__0__SensirionSht11__writeStatusRegDone(/*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP__0__currentClient, _writeFail ? FAIL : SUCCESS);
      break;
      default: 

        break;
    }
}

# 240 "/usr/lib/ncc/nesc_nx.h"
static __inline  uint8_t __nesc_hton_uint8(void * target, uint8_t value)
#line 240
{
  uint8_t *base = target;

#line 242
  base[0] = value;
  return value;
}

# 81 "/home/tinyos/local/src/tinyos-2.x/tos/interfaces/Queue.nc"
inline static IPDispatchP__SendQueue__t  IPDispatchP__SendQueue__dequeue(void ){
#line 81
  struct __nesc_unnamed4306 *result;
#line 81

#line 81
  result = /*IPDispatchC.QueueC*/QueueC__0__Queue__dequeue();
#line 81

#line 81
  return result;
#line 81
}
#line 81
# 89 "/home/tinyos/local/src/tinyos-2.x/tos/interfaces/Pool.nc"
inline static error_t IPDispatchP__SendEntryPool__put(IPDispatchP__SendEntryPool__t * newVal){
#line 89
  unsigned char result;
#line 89

#line 89
  result = /*IPDispatchC.SendEntryPool.PoolP*/PoolP__1__Pool__put(newVal);
#line 89

#line 89
  return result;
#line 89
}
#line 89
inline static error_t IPDispatchP__FragPool__put(IPDispatchP__FragPool__t * newVal){
#line 89
  unsigned char result;
#line 89

#line 89
  result = /*IPDispatchC.FragPool.PoolP*/PoolP__0__Pool__put(newVal);
#line 89

#line 89
  return result;
#line 89
}
#line 89
inline static error_t IPDispatchP__SendInfoPool__put(IPDispatchP__SendInfoPool__t * newVal){
#line 89
  unsigned char result;
#line 89

#line 89
  result = /*IPDispatchC.SendInfoPool.PoolP*/PoolP__2__Pool__put(newVal);
#line 89

#line 89
  return result;
#line 89
}
#line 89
# 56 "/home/tinyos/local/src/tinyos-2.x/tos/interfaces/TaskBasic.nc"
inline static error_t IPDispatchP__sendTask__postTask(void ){
#line 56
  unsigned char result;
#line 56

#line 56
  result = SchedulerBasicP__TaskBasic__postTask(IPDispatchP__sendTask);
#line 56

#line 56
  return result;
#line 56
}
#line 56
# 65 "/home/tinyos/local/src/tinyos-2.x/tos/system/QueueC.nc"
static inline /*IPDispatchC.QueueC*/QueueC__0__queue_t /*IPDispatchC.QueueC*/QueueC__0__Queue__head(void )
#line 65
{
  return /*IPDispatchC.QueueC*/QueueC__0__queue[/*IPDispatchC.QueueC*/QueueC__0__head];
}

# 73 "/home/tinyos/local/src/tinyos-2.x/tos/interfaces/Queue.nc"
inline static IPDispatchP__SendQueue__t  IPDispatchP__SendQueue__head(void ){
#line 73
  struct __nesc_unnamed4306 *result;
#line 73

#line 73
  result = /*IPDispatchC.QueueC*/QueueC__0__Queue__head();
#line 73

#line 73
  return result;
#line 73
}
#line 73
# 53 "/home/tinyos/local/src/tinyos-2.x/tos/system/QueueC.nc"
static inline bool /*IPDispatchC.QueueC*/QueueC__0__Queue__empty(void )
#line 53
{
  return /*IPDispatchC.QueueC*/QueueC__0__size == 0;
}

# 50 "/home/tinyos/local/src/tinyos-2.x/tos/interfaces/Queue.nc"
inline static bool IPDispatchP__SendQueue__empty(void ){
#line 50
  unsigned char result;
#line 50

#line 50
  result = /*IPDispatchC.QueueC*/QueueC__0__Queue__empty();
#line 50

#line 50
  return result;
#line 50
}
#line 50
# 42 "/home/tinyos/local/src/tinyos-2.x/tos/chips/cc2420/interfaces/CC2420PacketBody.nc"
inline static cc2420_header_t * CC2420Ieee154MessageP__CC2420PacketBody__getHeader(message_t * msg){
#line 42
  nx_struct cc2420_header_t *result;
#line 42

#line 42
  result = CC2420PacketP__CC2420PacketBody__getHeader(msg);
#line 42

#line 42
  return result;
#line 42
}
#line 42
# 146 "/home/tinyos/local/src/tinyos-2.x/tos/chips/cc2420/CC2420Ieee154MessageP.nc"
static inline uint8_t CC2420Ieee154MessageP__Packet__payloadLength(message_t *msg)
#line 146
{
  return __nesc_ntoh_leuint8(CC2420Ieee154MessageP__CC2420PacketBody__getHeader(msg)->length.data) - CC2420_SIZE + AM_OVERHEAD;
}

# 67 "/home/tinyos/local/src/tinyos-2.x/tos/interfaces/Packet.nc"
inline static uint8_t IPDispatchP__Packet__payloadLength(message_t * msg){
#line 67
  unsigned char result;
#line 67

#line 67
  result = CC2420Ieee154MessageP__Packet__payloadLength(msg);
#line 67

#line 67
  return result;
#line 67
}
#line 67
# 28 "/home/tinyos/local/src/tinyos-2.x/tos/interfaces/Ieee154Packet.nc"
inline static ieee154_saddr_t IPDispatchP__Ieee154Packet__destination(message_t *msg){
#line 28
  unsigned int result;
#line 28

#line 28
  result = CC2420Ieee154MessageP__Ieee154Packet__destination(msg);
#line 28

#line 28
  return result;
#line 28
}
#line 28
# 54 "/home/tinyos/local/src/tinyos-2.x/tos/system/FcfsResourceQueueC.nc"
static inline bool /*CC2420TinyosNetworkC.FcfsResourceQueueC*/FcfsResourceQueueC__0__FcfsQueue__isEnqueued(resource_client_id_t id)
#line 54
{
  /* atomic removed: atomic calls only */
#line 55
  {
    unsigned char __nesc_temp = 
#line 55
    /*CC2420TinyosNetworkC.FcfsResourceQueueC*/FcfsResourceQueueC__0__resQ[id] != /*CC2420TinyosNetworkC.FcfsResourceQueueC*/FcfsResourceQueueC__0__NO_ENTRY || /*CC2420TinyosNetworkC.FcfsResourceQueueC*/FcfsResourceQueueC__0__qTail == id;

#line 55
    return __nesc_temp;
  }
}

#line 72
static inline error_t /*CC2420TinyosNetworkC.FcfsResourceQueueC*/FcfsResourceQueueC__0__FcfsQueue__enqueue(resource_client_id_t id)
#line 72
{
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 73
    {
      if (!/*CC2420TinyosNetworkC.FcfsResourceQueueC*/FcfsResourceQueueC__0__FcfsQueue__isEnqueued(id)) {
          if (/*CC2420TinyosNetworkC.FcfsResourceQueueC*/FcfsResourceQueueC__0__qHead == /*CC2420TinyosNetworkC.FcfsResourceQueueC*/FcfsResourceQueueC__0__NO_ENTRY) {
            /*CC2420TinyosNetworkC.FcfsResourceQueueC*/FcfsResourceQueueC__0__qHead = id;
            }
          else {
#line 78
            /*CC2420TinyosNetworkC.FcfsResourceQueueC*/FcfsResourceQueueC__0__resQ[/*CC2420TinyosNetworkC.FcfsResourceQueueC*/FcfsResourceQueueC__0__qTail] = id;
            }
#line 79
          /*CC2420TinyosNetworkC.FcfsResourceQueueC*/FcfsResourceQueueC__0__qTail = id;
          {
            unsigned char __nesc_temp = 
#line 80
            SUCCESS;

            {
#line 80
              __nesc_atomic_end(__nesc_atomic); 
#line 80
              return __nesc_temp;
            }
          }
        }
#line 82
      {
        unsigned char __nesc_temp = 
#line 82
        EBUSY;

        {
#line 82
          __nesc_atomic_end(__nesc_atomic); 
#line 82
          return __nesc_temp;
        }
      }
    }
#line 85
    __nesc_atomic_end(__nesc_atomic); }
}

# 69 "/home/tinyos/local/src/tinyos-2.x/tos/interfaces/ResourceQueue.nc"
inline static error_t CC2420TinyosNetworkP__Queue__enqueue(resource_client_id_t id){
#line 69
  unsigned char result;
#line 69

#line 69
  result = /*CC2420TinyosNetworkC.FcfsResourceQueueC*/FcfsResourceQueueC__0__FcfsQueue__enqueue(id);
#line 69

#line 69
  return result;
#line 69
}
#line 69
# 56 "/home/tinyos/local/src/tinyos-2.x/tos/interfaces/TaskBasic.nc"
inline static error_t CC2420TinyosNetworkP__grantTask__postTask(void ){
#line 56
  unsigned char result;
#line 56

#line 56
  result = SchedulerBasicP__TaskBasic__postTask(CC2420TinyosNetworkP__grantTask);
#line 56

#line 56
  return result;
#line 56
}
#line 56
# 167 "/home/tinyos/local/src/tinyos-2.x/tos/chips/cc2420/lowpan/CC2420TinyosNetworkP.nc"
static inline error_t CC2420TinyosNetworkP__Resource__request(uint8_t id)
#line 167
{

  CC2420TinyosNetworkP__grantTask__postTask();

  if (CC2420TinyosNetworkP__TINYOS_N_NETWORKS > 1) {

      return CC2420TinyosNetworkP__Queue__enqueue(id);
    }
  else 
#line 174
    {
      if (id == CC2420TinyosNetworkP__resource_owner) {
          return EALREADY;
        }
      else 
#line 177
        {
          CC2420TinyosNetworkP__next_owner = id;
          return SUCCESS;
        }
    }
}

# 78 "/home/tinyos/local/src/tinyos-2.x/tos/interfaces/Resource.nc"
inline static error_t ResourceSendP__Resource__request(void ){
#line 78
  unsigned char result;
#line 78

#line 78
  result = CC2420TinyosNetworkP__Resource__request(0U);
#line 78

#line 78
  return result;
#line 78
}
#line 78
# 15 "/home/tinyos/local/src/tinyos-2.x/tos/lib/net/blip/ResourceSendP.nc"
static inline error_t ResourceSendP__Ieee154Send__send(ieee154_saddr_t addr, 
message_t *msg, 
uint8_t len)
#line 17
{
  if (ResourceSendP__m_msg != (void *)0) {
#line 18
    return EBUSY;
    }
  ResourceSendP__m_addr = addr;
  ResourceSendP__m_msg = msg;
  ResourceSendP__m_len = len;

  ResourceSendP__Resource__request();
  return SUCCESS;
}

# 56 "/home/tinyos/local/src/tinyos-2.x/tos/interfaces/Ieee154Send.nc"
inline static error_t IPDispatchP__Ieee154Send__send(ieee154_saddr_t addr, message_t *msg, uint8_t len){
#line 56
  unsigned char result;
#line 56

#line 56
  result = ResourceSendP__Ieee154Send__send(addr, msg, len);
#line 56

#line 56
  return result;
#line 56
}
#line 56
# 269 "/usr/lib/ncc/nesc_nx.h"
static __inline  uint16_t __nesc_hton_uint16(void * target, uint16_t value)
#line 269
{
  uint8_t *base = target;

#line 271
  base[1] = value;
  base[0] = value >> 8;
  return value;
}

# 47 "/home/tinyos/local/src/tinyos-2.x/tos/chips/cc2420/interfaces/CC2420PacketBody.nc"
inline static cc2420_metadata_t * PacketLinkP__CC2420PacketBody__getMetadata(message_t * msg){
#line 47
  nx_struct cc2420_metadata_t *result;
#line 47

#line 47
  result = CC2420PacketP__CC2420PacketBody__getMetadata(msg);
#line 47

#line 47
  return result;
#line 47
}
#line 47
# 97 "/home/tinyos/local/src/tinyos-2.x/tos/chips/cc2420/link/PacketLinkP.nc"
static inline void PacketLinkP__PacketLink__setRetryDelay(message_t *msg, uint16_t retryDelay)
#line 97
{
  __nesc_hton_uint16(PacketLinkP__CC2420PacketBody__getMetadata(msg)->retryDelay.data, retryDelay);
}

# 53 "/home/tinyos/local/src/tinyos-2.x/tos/interfaces/PacketLink.nc"
inline static void IPDispatchP__PacketLink__setRetryDelay(message_t *msg, uint16_t retryDelay){
#line 53
  PacketLinkP__PacketLink__setRetryDelay(msg, retryDelay);
#line 53
}
#line 53
# 88 "/home/tinyos/local/src/tinyos-2.x/tos/chips/cc2420/link/PacketLinkP.nc"
static inline void PacketLinkP__PacketLink__setRetries(message_t *msg, uint16_t maxRetries)
#line 88
{
  __nesc_hton_uint16(PacketLinkP__CC2420PacketBody__getMetadata(msg)->maxRetries.data, maxRetries);
}

# 46 "/home/tinyos/local/src/tinyos-2.x/tos/interfaces/PacketLink.nc"
inline static void IPDispatchP__PacketLink__setRetries(message_t * msg, uint16_t maxRetries){
#line 46
  PacketLinkP__PacketLink__setRetries(msg, maxRetries);
#line 46
}
#line 46
# 281 "/usr/lib/ncc/nesc_nx.h"
static __inline  uint16_t __nesc_hton_leuint16(void * target, uint16_t value)
#line 281
{
  uint8_t *base = target;

#line 283
  base[0] = value;
  base[1] = value >> 8;
  return value;
}

# 111 "/home/tinyos/local/src/tinyos-2.x/tos/chips/cc2420/CC2420Ieee154MessageP.nc"
static inline void CC2420Ieee154MessageP__Ieee154Packet__setDestination(message_t *msg, ieee154_saddr_t addr)
#line 111
{
  cc2420_header_t *header = CC2420Ieee154MessageP__CC2420PacketBody__getHeader(msg);

#line 113
  __nesc_hton_leuint16(header->dest.data, addr);
}

# 32 "/home/tinyos/local/src/tinyos-2.x/tos/interfaces/Ieee154Packet.nc"
inline static void IPDispatchP__Ieee154Packet__setDestination(message_t *msg, ieee154_saddr_t addr){
#line 32
  CC2420Ieee154MessageP__Ieee154Packet__setDestination(msg, addr);
#line 32
}
#line 32
# 781 "/home/tinyos/local/src/tinyos-2.x/tos/lib/net/blip/IPDispatchP.nc"
static inline void IPDispatchP__sendTask__runTask(void )
#line 781
{
  unsigned char __nesc_temp63;
  unsigned char *__nesc_temp62;
#line 782
  send_entry_t *s_entry;

#line 783
  if (IPDispatchP__radioBusy || IPDispatchP__state != IPDispatchP__S_RUNNING) {
#line 783
    return;
    }
#line 784
  if (IPDispatchP__SendQueue__empty()) {
#line 784
    return;
    }
  s_entry = IPDispatchP__SendQueue__head();


  IPDispatchP__Ieee154Packet__setDestination(s_entry->msg, 
  s_entry->info->policy.dest[s_entry->info->policy.current]);
  IPDispatchP__PacketLink__setRetries(s_entry->msg, s_entry->info->policy.retries);
  IPDispatchP__PacketLink__setRetryDelay(s_entry->msg, s_entry->info->policy.delay);





  ;



  if (s_entry->info->failed) {
      ;
      goto fail;
    }



  if (
#line 807
  IPDispatchP__Ieee154Send__send(IPDispatchP__Ieee154Packet__destination(s_entry->msg), 
  s_entry->msg, 
  IPDispatchP__Packet__payloadLength(s_entry->msg)) != SUCCESS) {
      ;
      goto fail;
    }
  IPDispatchP__radioBusy = TRUE;
  if (IPDispatchP__SendQueue__empty()) {
#line 814
    return;
    }
  s_entry = IPDispatchP__SendQueue__head();

  return;
  fail: 
    IPDispatchP__sendTask__postTask();
  (__nesc_temp62 = IPDispatchP__stats.tx_drop.data, __nesc_hton_uint8(__nesc_temp62, (__nesc_temp63 = __nesc_ntoh_uint8(__nesc_temp62)) + 1), __nesc_temp63);



  s_entry->info->failed = TRUE;
  if (-- s_entry->info->refcount == 0) {
#line 826
    IPDispatchP__SendInfoPool__put(s_entry->info);
    }
#line 827
  IPDispatchP__FragPool__put(s_entry->msg);
  IPDispatchP__SendEntryPool__put(s_entry);
  IPDispatchP__SendQueue__dequeue();
}

# 69 "/home/tinyos/local/src/tinyos-2.x/tos/system/QueueC.nc"
static inline void /*IPDispatchC.QueueC*/QueueC__0__printQueue(void )
#line 69
{
}

# 56 "/home/tinyos/local/src/tinyos-2.x/tos/interfaces/TaskBasic.nc"
inline static error_t PacketLinkP__send__postTask(void ){
#line 56
  unsigned char result;
#line 56

#line 56
  result = SchedulerBasicP__TaskBasic__postTask(PacketLinkP__send);
#line 56

#line 56
  return result;
#line 56
}
#line 56
# 64 "/home/tinyos/local/src/tinyos-2.x/tos/interfaces/Send.nc"
inline static error_t PacketLinkP__SubSend__send(message_t * msg, uint8_t len){
#line 64
  unsigned char result;
#line 64

#line 64
  result = CC2420CsmaP__Send__send(msg, len);
#line 64

#line 64
  return result;
#line 64
}
#line 64
# 48 "/home/tinyos/local/src/tinyos-2.x/tos/interfaces/PacketAcknowledgements.nc"
inline static error_t PacketLinkP__PacketAcknowledgements__requestAck(message_t * msg){
#line 48
  unsigned char result;
#line 48

#line 48
  result = CC2420PacketP__Acks__requestAck(msg);
#line 48

#line 48
  return result;
#line 48
}
#line 48
# 264 "/usr/lib/ncc/nesc_nx.h"
static __inline  uint16_t __nesc_ntoh_uint16(const void * source)
#line 264
{
  const uint8_t *base = source;

#line 266
  return ((uint16_t )base[0] << 8) | base[1];
}

# 104 "/home/tinyos/local/src/tinyos-2.x/tos/chips/cc2420/link/PacketLinkP.nc"
static inline uint16_t PacketLinkP__PacketLink__getRetries(message_t *msg)
#line 104
{
  return __nesc_ntoh_uint16(PacketLinkP__CC2420PacketBody__getMetadata(msg)->maxRetries.data);
}

#line 209
static inline void PacketLinkP__send__runTask(void )
#line 209
{
  if (PacketLinkP__PacketLink__getRetries(PacketLinkP__currentSendMsg) > 0) {
      PacketLinkP__PacketAcknowledgements__requestAck(PacketLinkP__currentSendMsg);
    }

  if (PacketLinkP__SubSend__send(PacketLinkP__currentSendMsg, PacketLinkP__currentSendLen) != SUCCESS) {
      PacketLinkP__send__postTask();
    }
}

# 42 "/home/tinyos/local/src/tinyos-2.x/tos/chips/cc2420/interfaces/CC2420PacketBody.nc"
inline static cc2420_header_t * CC2420CsmaP__CC2420PacketBody__getHeader(message_t * msg){
#line 42
  nx_struct cc2420_header_t *result;
#line 42

#line 42
  result = CC2420PacketP__CC2420PacketBody__getHeader(msg);
#line 42

#line 42
  return result;
#line 42
}
#line 42





inline static cc2420_metadata_t * CC2420CsmaP__CC2420PacketBody__getMetadata(message_t * msg){
#line 47
  nx_struct cc2420_metadata_t *result;
#line 47

#line 47
  result = CC2420PacketP__CC2420PacketBody__getMetadata(msg);
#line 47

#line 47
  return result;
#line 47
}
#line 47
# 291 "/home/tinyos/local/src/tinyos-2.x/tos/chips/cc2420/csma/CC2420CsmaP.nc"
static inline void CC2420CsmaP__RadioBackoff__default__requestCca(message_t *msg)
#line 291
{
}

# 95 "/home/tinyos/local/src/tinyos-2.x/tos/chips/cc2420/interfaces/RadioBackoff.nc"
inline static void CC2420CsmaP__RadioBackoff__requestCca(message_t * msg){
#line 95
  CC2420CsmaP__RadioBackoff__default__requestCca(msg);
#line 95
}
#line 95
# 545 "/home/tinyos/local/src/tinyos-2.x/tos/chips/cc2420/transmit/CC2420TransmitP.nc"
static inline error_t CC2420TransmitP__send(message_t * p_msg, bool cca)
#line 545
{
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 546
    {
      if (CC2420TransmitP__m_state == CC2420TransmitP__S_CANCEL) {
          {
            unsigned char __nesc_temp = 
#line 548
            ECANCEL;

            {
#line 548
              __nesc_atomic_end(__nesc_atomic); 
#line 548
              return __nesc_temp;
            }
          }
        }
#line 551
      if (CC2420TransmitP__m_state != CC2420TransmitP__S_STARTED) {
          {
            unsigned char __nesc_temp = 
#line 552
            FAIL;

            {
#line 552
              __nesc_atomic_end(__nesc_atomic); 
#line 552
              return __nesc_temp;
            }
          }
        }


      CC2420TransmitP__m_state = CC2420TransmitP__S_LOAD;
      CC2420TransmitP__m_cca = cca;
      CC2420TransmitP__m_msg = p_msg;
      CC2420TransmitP__totalCcaChecks = 0;
    }
#line 562
    __nesc_atomic_end(__nesc_atomic); }

  if (CC2420TransmitP__acquireSpiResource() == SUCCESS) {
      CC2420TransmitP__loadTXFIFO();
    }

  return SUCCESS;
}

#line 192
static inline error_t CC2420TransmitP__Send__send(message_t * p_msg, bool useCca)
#line 192
{
  return CC2420TransmitP__send(p_msg, useCca);
}

# 51 "/home/tinyos/local/src/tinyos-2.x/tos/chips/cc2420/interfaces/CC2420Transmit.nc"
inline static error_t CC2420CsmaP__CC2420Transmit__send(message_t * p_msg, bool useCca){
#line 51
  unsigned char result;
#line 51

#line 51
  result = CC2420TransmitP__Send__send(p_msg, useCca);
#line 51

#line 51
  return result;
#line 51
}
#line 51
# 55 "/home/tinyos/local/src/tinyos-2.x/tos/chips/cc2420/interfaces/CC2420Register.nc"
inline static cc2420_status_t CC2420TransmitP__TXCTRL__write(uint16_t data){
#line 55
  unsigned char result;
#line 55

#line 55
  result = CC2420SpiP__Reg__write(CC2420_TXCTRL, data);
#line 55

#line 55
  return result;
#line 55
}
#line 55
# 59 "/home/tinyos/local/src/tinyos-2.x/tos/interfaces/SpiPacket.nc"
inline static error_t CC2420SpiP__SpiPacket__send(uint8_t * txBuf, uint8_t * rxBuf, uint16_t len){
#line 59
  unsigned char result;
#line 59

#line 59
  result = /*Msp430SpiDma0P.SpiP*/Msp430SpiDmaP__0__SpiPacket__send(/*CC2420SpiWireC.HplCC2420SpiC.SpiC*/Msp430Spi0C__0__CLIENT_ID, txBuf, rxBuf, len);
#line 59

#line 59
  return result;
#line 59
}
#line 59
# 34 "/home/tinyos/local/src/tinyos-2.x/tos/interfaces/SpiByte.nc"
inline static uint8_t CC2420SpiP__SpiByte__write(uint8_t tx){
#line 34
  unsigned char result;
#line 34

#line 34
  result = /*Msp430SpiDma0P.SpiP*/Msp430SpiDmaP__0__SpiByte__write(tx);
#line 34

#line 34
  return result;
#line 34
}
#line 34
# 126 "/home/tinyos/local/src/tinyos-2.x/tos/system/StateImplP.nc"
static inline bool StateImplP__State__isIdle(uint8_t id)
#line 126
{
  return StateImplP__State__isState(id, StateImplP__S_IDLE);
}

# 61 "/home/tinyos/local/src/tinyos-2.x/tos/interfaces/State.nc"
inline static bool CC2420SpiP__WorkingState__isIdle(void ){
#line 61
  unsigned char result;
#line 61

#line 61
  result = StateImplP__State__isIdle(0U);
#line 61

#line 61
  return result;
#line 61
}
#line 61
# 214 "/home/tinyos/local/src/tinyos-2.x/tos/chips/cc2420/spi/CC2420SpiP.nc"
static inline cc2420_status_t CC2420SpiP__Fifo__write(uint8_t addr, uint8_t *data, 
uint8_t len)
#line 215
{

  uint8_t status = 0;

  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 219
    {
      if (CC2420SpiP__WorkingState__isIdle()) {
          {
            unsigned char __nesc_temp = 
#line 221
            status;

            {
#line 221
              __nesc_atomic_end(__nesc_atomic); 
#line 221
              return __nesc_temp;
            }
          }
        }
    }
#line 225
    __nesc_atomic_end(__nesc_atomic); }
#line 225
  CC2420SpiP__m_addr = addr;

  status = CC2420SpiP__SpiByte__write(CC2420SpiP__m_addr);
  CC2420SpiP__SpiPacket__send(data, (void *)0, len);

  return status;
}

# 82 "/home/tinyos/local/src/tinyos-2.x/tos/chips/cc2420/interfaces/CC2420Fifo.nc"
inline static cc2420_status_t CC2420TransmitP__TXFIFO__write(uint8_t * data, uint8_t length){
#line 82
  unsigned char result;
#line 82

#line 82
  result = CC2420SpiP__Fifo__write(CC2420_TXFIFO, data, length);
#line 82

#line 82
  return result;
#line 82
}
#line 82
# 91 "/home/tinyos/local/src/tinyos-2.x/tos/chips/msp430/dma/HplMsp430DmaXP.nc"
static inline error_t /*HplMsp430DmaC.Dma1*/HplMsp430DmaXP__1__DMA__setTrigger(dma_trigger_t trigger)
#line 91
{

  if (* (volatile uint16_t *)488U & 0x0010) {
    return FAIL;
    }
  /*HplMsp430DmaC.Dma1*/HplMsp430DmaXP__1__DMACTL0 = (/*HplMsp430DmaC.Dma1*/HplMsp430DmaXP__1__DMACTL0 & ~240) | ((
  trigger << 4) & 240);

  return SUCCESS;
}

#line 243
static inline void /*HplMsp430DmaC.Dma1*/HplMsp430DmaXP__1__DMA__setStateRaw(uint16_t s, uint16_t t, 
void *src, void *dest, 
uint16_t size)
#line 245
{
  * (volatile uint16_t *)490U = (uint16_t )src;
  * (volatile uint16_t *)492U = (uint16_t )dest;
  * (volatile uint16_t *)494U = size;
  /*HplMsp430DmaC.Dma1*/HplMsp430DmaXP__1__DMA__setTrigger((dma_trigger_t )t);
  * (volatile uint16_t *)488U = s;
}

#line 235
static inline void /*HplMsp430DmaC.Dma1*/HplMsp430DmaXP__1__DMA__setState(dma_channel_state_t s, 
dma_channel_trigger_t t, 
void *src, void *dest, 
uint16_t size)
#line 238
{
  /*HplMsp430DmaC.Dma1*/HplMsp430DmaXP__1__DMA__setStateRaw(* (uint16_t *)&s, * (uint16_t *)&t, 
  src, dest, size);
}

# 64 "/home/tinyos/local/src/tinyos-2.x/tos/chips/msp430/dma/HplMsp430DmaChannel.nc"
inline static void /*Msp430DmaC.Channel1P*/Msp430DmaChannelP__1__HplChannel__setState(dma_channel_state_t s, dma_channel_trigger_t t, void *src, void *dest, uint16_t size){
#line 64
  /*HplMsp430DmaC.Dma1*/HplMsp430DmaXP__1__DMA__setState(s, t, src, dest, size);
#line 64
}
#line 64
# 80 "/home/tinyos/local/src/tinyos-2.x/tos/chips/msp430/dma/Msp430DmaChannelP.nc"
static inline error_t /*Msp430DmaC.Channel1P*/Msp430DmaChannelP__1__Channel__setupTransfer(dma_transfer_mode_t transfer_mode, 
dma_trigger_t trigger, 
dma_level_t level, 
void *src_addr, 
void *dst_addr, 
uint16_t size, 
dma_byte_t src_byte, 
dma_byte_t dst_byte, 
dma_incr_t src_incr, 
dma_incr_t dst_incr)
#line 89
{

  /*Msp430DmaC.Channel1P*/Msp430DmaChannelP__1__gChannelState.request = 0;
  /*Msp430DmaC.Channel1P*/Msp430DmaChannelP__1__gChannelState.abort = 0;
  /*Msp430DmaC.Channel1P*/Msp430DmaChannelP__1__gChannelState.interruptEnable = 1;
  /*Msp430DmaC.Channel1P*/Msp430DmaChannelP__1__gChannelState.interruptFlag = 0;
  /*Msp430DmaC.Channel1P*/Msp430DmaChannelP__1__gChannelState.enable = 0;
  /*Msp430DmaC.Channel1P*/Msp430DmaChannelP__1__gChannelState.level = level;
  /*Msp430DmaC.Channel1P*/Msp430DmaChannelP__1__gChannelState.srcByte = src_byte;
  /*Msp430DmaC.Channel1P*/Msp430DmaChannelP__1__gChannelState.dstByte = dst_byte;
  /*Msp430DmaC.Channel1P*/Msp430DmaChannelP__1__gChannelState.srcIncrement = src_incr;
  /*Msp430DmaC.Channel1P*/Msp430DmaChannelP__1__gChannelState.dstIncrement = dst_incr;
  /*Msp430DmaC.Channel1P*/Msp430DmaChannelP__1__gChannelState.transferMode = transfer_mode;

  /*Msp430DmaC.Channel1P*/Msp430DmaChannelP__1__gChannelTrigger.trigger = trigger;

  /*Msp430DmaC.Channel1P*/Msp430DmaChannelP__1__HplChannel__setState(/*Msp430DmaC.Channel1P*/Msp430DmaChannelP__1__gChannelState, /*Msp430DmaC.Channel1P*/Msp430DmaChannelP__1__gChannelTrigger, 
  src_addr, dst_addr, size);

  return SUCCESS;
}

# 38 "/home/tinyos/local/src/tinyos-2.x/tos/chips/msp430/dma/Msp430DmaChannel.nc"
inline static error_t /*Msp430SpiDma0P.SpiP*/Msp430SpiDmaP__0__DmaChannel1__setupTransfer(dma_transfer_mode_t transfer_mode, dma_trigger_t trigger, dma_level_t level, void *src_addr, void *dst_addr, uint16_t size, dma_byte_t src_byte, dma_byte_t dst_byte, dma_incr_t src_incr, dma_incr_t dst_incr){
#line 38
  unsigned char result;
#line 38

#line 38
  result = /*Msp430DmaC.Channel1P*/Msp430DmaChannelP__1__Channel__setupTransfer(transfer_mode, trigger, level, src_addr, dst_addr, size, src_byte, dst_byte, src_incr, dst_incr);
#line 38

#line 38
  return result;
#line 38
}
#line 38
# 195 "/home/tinyos/local/src/tinyos-2.x/tos/chips/msp430/dma/HplMsp430DmaXP.nc"
static inline void /*HplMsp430DmaC.Dma1*/HplMsp430DmaXP__1__DMA__enableDMA(void )
#line 195
{
  * (volatile uint16_t *)488U |= 0x0010;
}

# 49 "/home/tinyos/local/src/tinyos-2.x/tos/chips/msp430/dma/HplMsp430DmaChannel.nc"
inline static void /*Msp430DmaC.Channel1P*/Msp430DmaChannelP__1__HplChannel__enableDMA(void ){
#line 49
  /*HplMsp430DmaC.Dma1*/HplMsp430DmaXP__1__DMA__enableDMA();
#line 49
}
#line 49
# 112 "/home/tinyos/local/src/tinyos-2.x/tos/chips/msp430/dma/Msp430DmaChannelP.nc"
static inline error_t /*Msp430DmaC.Channel1P*/Msp430DmaChannelP__1__Channel__startTransfer(void )
#line 112
{
  /*Msp430DmaC.Channel1P*/Msp430DmaChannelP__1__HplChannel__enableDMA();
  return SUCCESS;
}

# 73 "/home/tinyos/local/src/tinyos-2.x/tos/chips/msp430/dma/Msp430DmaChannel.nc"
inline static error_t /*Msp430SpiDma0P.SpiP*/Msp430SpiDmaP__0__DmaChannel1__startTransfer(void ){
#line 73
  unsigned char result;
#line 73

#line 73
  result = /*Msp430DmaC.Channel1P*/Msp430DmaChannelP__1__Channel__startTransfer();
#line 73

#line 73
  return result;
#line 73
}
#line 73
# 91 "/home/tinyos/local/src/tinyos-2.x/tos/chips/msp430/dma/HplMsp430DmaXP.nc"
static inline error_t /*HplMsp430DmaC.Dma2*/HplMsp430DmaXP__2__DMA__setTrigger(dma_trigger_t trigger)
#line 91
{

  if (* (volatile uint16_t *)496U & 0x0010) {
    return FAIL;
    }
  /*HplMsp430DmaC.Dma2*/HplMsp430DmaXP__2__DMACTL0 = (/*HplMsp430DmaC.Dma2*/HplMsp430DmaXP__2__DMACTL0 & ~3840) | ((
  trigger << 8) & 3840);

  return SUCCESS;
}

#line 243
static inline void /*HplMsp430DmaC.Dma2*/HplMsp430DmaXP__2__DMA__setStateRaw(uint16_t s, uint16_t t, 
void *src, void *dest, 
uint16_t size)
#line 245
{
  * (volatile uint16_t *)498U = (uint16_t )src;
  * (volatile uint16_t *)500U = (uint16_t )dest;
  * (volatile uint16_t *)502U = size;
  /*HplMsp430DmaC.Dma2*/HplMsp430DmaXP__2__DMA__setTrigger((dma_trigger_t )t);
  * (volatile uint16_t *)496U = s;
}

#line 235
static inline void /*HplMsp430DmaC.Dma2*/HplMsp430DmaXP__2__DMA__setState(dma_channel_state_t s, 
dma_channel_trigger_t t, 
void *src, void *dest, 
uint16_t size)
#line 238
{
  /*HplMsp430DmaC.Dma2*/HplMsp430DmaXP__2__DMA__setStateRaw(* (uint16_t *)&s, * (uint16_t *)&t, 
  src, dest, size);
}

# 64 "/home/tinyos/local/src/tinyos-2.x/tos/chips/msp430/dma/HplMsp430DmaChannel.nc"
inline static void /*Msp430DmaC.Channel2P*/Msp430DmaChannelP__2__HplChannel__setState(dma_channel_state_t s, dma_channel_trigger_t t, void *src, void *dest, uint16_t size){
#line 64
  /*HplMsp430DmaC.Dma2*/HplMsp430DmaXP__2__DMA__setState(s, t, src, dest, size);
#line 64
}
#line 64
# 80 "/home/tinyos/local/src/tinyos-2.x/tos/chips/msp430/dma/Msp430DmaChannelP.nc"
static inline error_t /*Msp430DmaC.Channel2P*/Msp430DmaChannelP__2__Channel__setupTransfer(dma_transfer_mode_t transfer_mode, 
dma_trigger_t trigger, 
dma_level_t level, 
void *src_addr, 
void *dst_addr, 
uint16_t size, 
dma_byte_t src_byte, 
dma_byte_t dst_byte, 
dma_incr_t src_incr, 
dma_incr_t dst_incr)
#line 89
{

  /*Msp430DmaC.Channel2P*/Msp430DmaChannelP__2__gChannelState.request = 0;
  /*Msp430DmaC.Channel2P*/Msp430DmaChannelP__2__gChannelState.abort = 0;
  /*Msp430DmaC.Channel2P*/Msp430DmaChannelP__2__gChannelState.interruptEnable = 1;
  /*Msp430DmaC.Channel2P*/Msp430DmaChannelP__2__gChannelState.interruptFlag = 0;
  /*Msp430DmaC.Channel2P*/Msp430DmaChannelP__2__gChannelState.enable = 0;
  /*Msp430DmaC.Channel2P*/Msp430DmaChannelP__2__gChannelState.level = level;
  /*Msp430DmaC.Channel2P*/Msp430DmaChannelP__2__gChannelState.srcByte = src_byte;
  /*Msp430DmaC.Channel2P*/Msp430DmaChannelP__2__gChannelState.dstByte = dst_byte;
  /*Msp430DmaC.Channel2P*/Msp430DmaChannelP__2__gChannelState.srcIncrement = src_incr;
  /*Msp430DmaC.Channel2P*/Msp430DmaChannelP__2__gChannelState.dstIncrement = dst_incr;
  /*Msp430DmaC.Channel2P*/Msp430DmaChannelP__2__gChannelState.transferMode = transfer_mode;

  /*Msp430DmaC.Channel2P*/Msp430DmaChannelP__2__gChannelTrigger.trigger = trigger;

  /*Msp430DmaC.Channel2P*/Msp430DmaChannelP__2__HplChannel__setState(/*Msp430DmaC.Channel2P*/Msp430DmaChannelP__2__gChannelState, /*Msp430DmaC.Channel2P*/Msp430DmaChannelP__2__gChannelTrigger, 
  src_addr, dst_addr, size);

  return SUCCESS;
}

# 38 "/home/tinyos/local/src/tinyos-2.x/tos/chips/msp430/dma/Msp430DmaChannel.nc"
inline static error_t /*Msp430SpiDma0P.SpiP*/Msp430SpiDmaP__0__DmaChannel2__setupTransfer(dma_transfer_mode_t transfer_mode, dma_trigger_t trigger, dma_level_t level, void *src_addr, void *dst_addr, uint16_t size, dma_byte_t src_byte, dma_byte_t dst_byte, dma_incr_t src_incr, dma_incr_t dst_incr){
#line 38
  unsigned char result;
#line 38

#line 38
  result = /*Msp430DmaC.Channel2P*/Msp430DmaChannelP__2__Channel__setupTransfer(transfer_mode, trigger, level, src_addr, dst_addr, size, src_byte, dst_byte, src_incr, dst_incr);
#line 38

#line 38
  return result;
#line 38
}
#line 38
# 195 "/home/tinyos/local/src/tinyos-2.x/tos/chips/msp430/dma/HplMsp430DmaXP.nc"
static inline void /*HplMsp430DmaC.Dma2*/HplMsp430DmaXP__2__DMA__enableDMA(void )
#line 195
{
  * (volatile uint16_t *)496U |= 0x0010;
}

# 49 "/home/tinyos/local/src/tinyos-2.x/tos/chips/msp430/dma/HplMsp430DmaChannel.nc"
inline static void /*Msp430DmaC.Channel2P*/Msp430DmaChannelP__2__HplChannel__enableDMA(void ){
#line 49
  /*HplMsp430DmaC.Dma2*/HplMsp430DmaXP__2__DMA__enableDMA();
#line 49
}
#line 49
# 112 "/home/tinyos/local/src/tinyos-2.x/tos/chips/msp430/dma/Msp430DmaChannelP.nc"
static inline error_t /*Msp430DmaC.Channel2P*/Msp430DmaChannelP__2__Channel__startTransfer(void )
#line 112
{
  /*Msp430DmaC.Channel2P*/Msp430DmaChannelP__2__HplChannel__enableDMA();
  return SUCCESS;
}

# 73 "/home/tinyos/local/src/tinyos-2.x/tos/chips/msp430/dma/Msp430DmaChannel.nc"
inline static error_t /*Msp430SpiDma0P.SpiP*/Msp430SpiDmaP__0__DmaChannel2__startTransfer(void ){
#line 73
  unsigned char result;
#line 73

#line 73
  result = /*Msp430DmaC.Channel2P*/Msp430DmaChannelP__2__Channel__startTransfer();
#line 73

#line 73
  return result;
#line 73
}
#line 73
# 56 "/home/tinyos/local/src/tinyos-2.x/tos/interfaces/TaskBasic.nc"
inline static error_t /*Msp430SpiDma0P.SpiP*/Msp430SpiDmaP__0__signalDone_task__postTask(void ){
#line 56
  unsigned char result;
#line 56

#line 56
  result = SchedulerBasicP__TaskBasic__postTask(/*Msp430SpiDma0P.SpiP*/Msp430SpiDmaP__0__signalDone_task);
#line 56

#line 56
  return result;
#line 56
}
#line 56
# 198 "/home/tinyos/local/src/tinyos-2.x/tos/chips/cc2420/lowpan/CC2420TinyosNetworkP.nc"
static inline error_t CC2420TinyosNetworkP__Resource__release(uint8_t id)
#line 198
{
  if (CC2420TinyosNetworkP__TINYOS_N_NETWORKS > 1) {
      CC2420TinyosNetworkP__grantTask__postTask();
    }
  CC2420TinyosNetworkP__resource_owner = CC2420TinyosNetworkP__OWNER_NONE;
  return SUCCESS;
}

#line 222
static inline void CC2420TinyosNetworkP__Resource__default__granted(uint8_t client)
#line 222
{
  CC2420TinyosNetworkP__Resource__release(client);
}

# 92 "/home/tinyos/local/src/tinyos-2.x/tos/interfaces/Resource.nc"
inline static void CC2420TinyosNetworkP__Resource__granted(uint8_t arg_0x41096628){
#line 92
  switch (arg_0x41096628) {
#line 92
    case 0U:
#line 92
      ResourceSendP__Resource__granted();
#line 92
      break;
#line 92
    default:
#line 92
      CC2420TinyosNetworkP__Resource__default__granted(arg_0x41096628);
#line 92
      break;
#line 92
    }
#line 92
}
#line 92
# 58 "/home/tinyos/local/src/tinyos-2.x/tos/system/FcfsResourceQueueC.nc"
static inline resource_client_id_t /*CC2420TinyosNetworkC.FcfsResourceQueueC*/FcfsResourceQueueC__0__FcfsQueue__dequeue(void )
#line 58
{
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 59
    {
      if (/*CC2420TinyosNetworkC.FcfsResourceQueueC*/FcfsResourceQueueC__0__qHead != /*CC2420TinyosNetworkC.FcfsResourceQueueC*/FcfsResourceQueueC__0__NO_ENTRY) {
          uint8_t id = /*CC2420TinyosNetworkC.FcfsResourceQueueC*/FcfsResourceQueueC__0__qHead;

#line 62
          /*CC2420TinyosNetworkC.FcfsResourceQueueC*/FcfsResourceQueueC__0__qHead = /*CC2420TinyosNetworkC.FcfsResourceQueueC*/FcfsResourceQueueC__0__resQ[/*CC2420TinyosNetworkC.FcfsResourceQueueC*/FcfsResourceQueueC__0__qHead];
          if (/*CC2420TinyosNetworkC.FcfsResourceQueueC*/FcfsResourceQueueC__0__qHead == /*CC2420TinyosNetworkC.FcfsResourceQueueC*/FcfsResourceQueueC__0__NO_ENTRY) {
            /*CC2420TinyosNetworkC.FcfsResourceQueueC*/FcfsResourceQueueC__0__qTail = /*CC2420TinyosNetworkC.FcfsResourceQueueC*/FcfsResourceQueueC__0__NO_ENTRY;
            }
#line 65
          /*CC2420TinyosNetworkC.FcfsResourceQueueC*/FcfsResourceQueueC__0__resQ[id] = /*CC2420TinyosNetworkC.FcfsResourceQueueC*/FcfsResourceQueueC__0__NO_ENTRY;
          {
            unsigned char __nesc_temp = 
#line 66
            id;

            {
#line 66
              __nesc_atomic_end(__nesc_atomic); 
#line 66
              return __nesc_temp;
            }
          }
        }
#line 68
      {
        unsigned char __nesc_temp = 
#line 68
        /*CC2420TinyosNetworkC.FcfsResourceQueueC*/FcfsResourceQueueC__0__NO_ENTRY;

        {
#line 68
          __nesc_atomic_end(__nesc_atomic); 
#line 68
          return __nesc_temp;
        }
      }
    }
#line 71
    __nesc_atomic_end(__nesc_atomic); }
}

# 60 "/home/tinyos/local/src/tinyos-2.x/tos/interfaces/ResourceQueue.nc"
inline static resource_client_id_t CC2420TinyosNetworkP__Queue__dequeue(void ){
#line 60
  unsigned char result;
#line 60

#line 60
  result = /*CC2420TinyosNetworkC.FcfsResourceQueueC*/FcfsResourceQueueC__0__FcfsQueue__dequeue();
#line 60

#line 60
  return result;
#line 60
}
#line 60
# 50 "/home/tinyos/local/src/tinyos-2.x/tos/system/FcfsResourceQueueC.nc"
static inline bool /*CC2420TinyosNetworkC.FcfsResourceQueueC*/FcfsResourceQueueC__0__FcfsQueue__isEmpty(void )
#line 50
{
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 51
    {
      unsigned char __nesc_temp = 
#line 51
      /*CC2420TinyosNetworkC.FcfsResourceQueueC*/FcfsResourceQueueC__0__qHead == /*CC2420TinyosNetworkC.FcfsResourceQueueC*/FcfsResourceQueueC__0__NO_ENTRY;

      {
#line 51
        __nesc_atomic_end(__nesc_atomic); 
#line 51
        return __nesc_temp;
      }
    }
#line 53
    __nesc_atomic_end(__nesc_atomic); }
}

# 43 "/home/tinyos/local/src/tinyos-2.x/tos/interfaces/ResourceQueue.nc"
inline static bool CC2420TinyosNetworkP__Queue__isEmpty(void ){
#line 43
  unsigned char result;
#line 43

#line 43
  result = /*CC2420TinyosNetworkC.FcfsResourceQueueC*/FcfsResourceQueueC__0__FcfsQueue__isEmpty();
#line 43

#line 43
  return result;
#line 43
}
#line 43
# 148 "/home/tinyos/local/src/tinyos-2.x/tos/chips/cc2420/lowpan/CC2420TinyosNetworkP.nc"
static inline void CC2420TinyosNetworkP__grantTask__runTask(void )
#line 148
{


  if (CC2420TinyosNetworkP__TINYOS_N_NETWORKS > 1) {
      if (CC2420TinyosNetworkP__resource_owner == CC2420TinyosNetworkP__OWNER_NONE && !CC2420TinyosNetworkP__Queue__isEmpty()) {
          CC2420TinyosNetworkP__resource_owner = CC2420TinyosNetworkP__Queue__dequeue();

          if (CC2420TinyosNetworkP__resource_owner != CC2420TinyosNetworkP__OWNER_NONE) {
              CC2420TinyosNetworkP__Resource__granted(CC2420TinyosNetworkP__resource_owner);
            }
        }
    }
  else 
#line 159
    {
      if (CC2420TinyosNetworkP__next_owner != CC2420TinyosNetworkP__resource_owner) {
          CC2420TinyosNetworkP__resource_owner = CC2420TinyosNetworkP__next_owner;
          CC2420TinyosNetworkP__Resource__granted(CC2420TinyosNetworkP__resource_owner);
        }
    }
}

# 251 "/usr/lib/ncc/nesc_nx.h"
static __inline  uint8_t __nesc_hton_leuint8(void * target, uint8_t value)
#line 251
{
  uint8_t *base = target;

#line 253
  base[0] = value;
  return value;
}

# 45 "/home/tinyos/local/src/tinyos-2.x/tos/interfaces/State.nc"
inline static error_t PacketLinkP__SendState__requestState(uint8_t reqState){
#line 45
  unsigned char result;
#line 45

#line 45
  result = StateImplP__State__requestState(4U, reqState);
#line 45

#line 45
  return result;
#line 45
}
#line 45
# 130 "/home/tinyos/local/src/tinyos-2.x/tos/chips/cc2420/link/PacketLinkP.nc"
static inline error_t PacketLinkP__Send__send(message_t *msg, uint8_t len)
#line 130
{
  error_t error;

#line 132
  if (PacketLinkP__SendState__requestState(PacketLinkP__S_SENDING) == SUCCESS) {

      PacketLinkP__currentSendMsg = msg;
      PacketLinkP__currentSendLen = len;
      PacketLinkP__totalRetries = 0;

      if (PacketLinkP__PacketLink__getRetries(msg) > 0) {
          PacketLinkP__PacketAcknowledgements__requestAck(msg);
        }

      if ((error = PacketLinkP__SubSend__send(msg, len)) != SUCCESS) {
          PacketLinkP__SendState__toIdle();
        }

      return error;
    }
  return EBUSY;
}

# 64 "/home/tinyos/local/src/tinyos-2.x/tos/interfaces/Send.nc"
inline static error_t UniqueSendP__SubSend__send(message_t * msg, uint8_t len){
#line 64
  unsigned char result;
#line 64

#line 64
  result = PacketLinkP__Send__send(msg, len);
#line 64

#line 64
  return result;
#line 64
}
#line 64
# 42 "/home/tinyos/local/src/tinyos-2.x/tos/chips/cc2420/interfaces/CC2420PacketBody.nc"
inline static cc2420_header_t * UniqueSendP__CC2420PacketBody__getHeader(message_t * msg){
#line 42
  nx_struct cc2420_header_t *result;
#line 42

#line 42
  result = CC2420PacketP__CC2420PacketBody__getHeader(msg);
#line 42

#line 42
  return result;
#line 42
}
#line 42
# 45 "/home/tinyos/local/src/tinyos-2.x/tos/interfaces/State.nc"
inline static error_t UniqueSendP__State__requestState(uint8_t reqState){
#line 45
  unsigned char result;
#line 45

#line 45
  result = StateImplP__State__requestState(2U, reqState);
#line 45

#line 45
  return result;
#line 45
}
#line 45
# 75 "/home/tinyos/local/src/tinyos-2.x/tos/chips/cc2420/unique/UniqueSendP.nc"
static inline error_t UniqueSendP__Send__send(message_t *msg, uint8_t len)
#line 75
{
  error_t error;

#line 77
  if (UniqueSendP__State__requestState(UniqueSendP__S_SENDING) == SUCCESS) {
      __nesc_hton_leuint8(UniqueSendP__CC2420PacketBody__getHeader(msg)->dsn.data, UniqueSendP__localSendId++);

      if ((error = UniqueSendP__SubSend__send(msg, len)) != SUCCESS) {
          UniqueSendP__State__toIdle();
        }

      return error;
    }

  return EBUSY;
}

# 64 "/home/tinyos/local/src/tinyos-2.x/tos/interfaces/Send.nc"
inline static error_t CC2420TinyosNetworkP__SubSend__send(message_t * msg, uint8_t len){
#line 64
  unsigned char result;
#line 64

#line 64
  result = UniqueSendP__Send__send(msg, len);
#line 64

#line 64
  return result;
#line 64
}
#line 64
# 95 "/home/tinyos/local/src/tinyos-2.x/tos/chips/cc2420/lowpan/CC2420TinyosNetworkP.nc"
static inline error_t CC2420TinyosNetworkP__BareSend__send(message_t *msg, uint8_t len)
#line 95
{
  return CC2420TinyosNetworkP__SubSend__send(msg, len - AM_OVERHEAD);
}

# 64 "/home/tinyos/local/src/tinyos-2.x/tos/interfaces/Send.nc"
inline static error_t CC2420Ieee154MessageP__SubSend__send(message_t * msg, uint8_t len){
#line 64
  unsigned char result;
#line 64

#line 64
  result = CC2420TinyosNetworkP__BareSend__send(msg, len);
#line 64

#line 64
  return result;
#line 64
}
#line 64
# 64 "/home/tinyos/local/src/tinyos-2.x/tos/chips/cc2420/interfaces/CC2420Config.nc"
inline static uint16_t CC2420Ieee154MessageP__CC2420Config__getShortAddr(void ){
#line 64
  unsigned int result;
#line 64

#line 64
  result = CC2420ControlP__CC2420Config__getShortAddr();
#line 64

#line 64
  return result;
#line 64
}
#line 64
# 287 "/home/tinyos/local/src/tinyos-2.x/tos/chips/cc2420/control/CC2420ControlP.nc"
static inline uint16_t CC2420ControlP__CC2420Config__getPanAddr(void )
#line 287
{
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 288
    {
      unsigned int __nesc_temp = 
#line 288
      CC2420ControlP__m_pan;

      {
#line 288
        __nesc_atomic_end(__nesc_atomic); 
#line 288
        return __nesc_temp;
      }
    }
#line 290
    __nesc_atomic_end(__nesc_atomic); }
}

# 70 "/home/tinyos/local/src/tinyos-2.x/tos/chips/cc2420/interfaces/CC2420Config.nc"
inline static uint16_t CC2420Ieee154MessageP__CC2420Config__getPanAddr(void ){
#line 70
  unsigned int result;
#line 70

#line 70
  result = CC2420ControlP__CC2420Config__getPanAddr();
#line 70

#line 70
  return result;
#line 70
}
#line 70
# 73 "/home/tinyos/local/src/tinyos-2.x/tos/chips/cc2420/CC2420Ieee154MessageP.nc"
static inline error_t CC2420Ieee154MessageP__Ieee154Send__send(ieee154_saddr_t addr, 
message_t *msg, 
uint8_t len)
#line 75
{
  cc2420_header_t *header = CC2420Ieee154MessageP__CC2420PacketBody__getHeader(msg);

#line 77
  __nesc_hton_leuint16(header->dest.data, addr);
  __nesc_hton_leuint16(header->destpan.data, CC2420Ieee154MessageP__CC2420Config__getPanAddr());
  __nesc_hton_leuint16(header->src.data, CC2420Ieee154MessageP__CC2420Config__getShortAddr());

  return CC2420Ieee154MessageP__SubSend__send(msg, len);
}

# 56 "/home/tinyos/local/src/tinyos-2.x/tos/interfaces/Ieee154Send.nc"
inline static error_t ResourceSendP__SubSend__send(ieee154_saddr_t addr, message_t *msg, uint8_t len){
#line 56
  unsigned char result;
#line 56

#line 56
  result = CC2420Ieee154MessageP__Ieee154Send__send(addr, msg, len);
#line 56

#line 56
  return result;
#line 56
}
#line 56
# 66 "/home/tinyos/local/src/tinyos-2.x/tos/interfaces/State.nc"
inline static bool CC2420CsmaP__SplitControlState__isState(uint8_t myState){
#line 66
  unsigned char result;
#line 66

#line 66
  result = StateImplP__State__isState(1U, myState);
#line 66

#line 66
  return result;
#line 66
}
#line 66
# 111 "/home/tinyos/local/src/tinyos-2.x/tos/system/StateImplP.nc"
static inline void StateImplP__State__forceState(uint8_t id, uint8_t reqState)
#line 111
{
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 112
    StateImplP__state[id] = reqState;
#line 112
    __nesc_atomic_end(__nesc_atomic); }
}

# 51 "/home/tinyos/local/src/tinyos-2.x/tos/interfaces/State.nc"
inline static void CC2420CsmaP__SplitControlState__forceState(uint8_t reqState){
#line 51
  StateImplP__State__forceState(1U, reqState);
#line 51
}
#line 51
# 56 "/home/tinyos/local/src/tinyos-2.x/tos/interfaces/TaskBasic.nc"
inline static error_t CC2420CsmaP__stopDone_task__postTask(void ){
#line 56
  unsigned char result;
#line 56

#line 56
  result = SchedulerBasicP__TaskBasic__postTask(CC2420CsmaP__stopDone_task);
#line 56

#line 56
  return result;
#line 56
}
#line 56
# 63 "/home/tinyos/local/src/tinyos-2.x/tos/chips/cc2420/interfaces/CC2420Power.nc"
inline static error_t CC2420CsmaP__CC2420Power__stopVReg(void ){
#line 63
  unsigned char result;
#line 63

#line 63
  result = CC2420ControlP__CC2420Power__stopVReg();
#line 63

#line 63
  return result;
#line 63
}
#line 63
# 58 "/home/tinyos/local/src/tinyos-2.x/tos/types/TinyError.h"
static inline  error_t ecombine(error_t r1, error_t r2)




{
  return r1 == r2 ? r1 : FAIL;
}

# 84 "/home/tinyos/local/src/tinyos-2.x/tos/interfaces/StdControl.nc"
inline static error_t CC2420CsmaP__SubControl__stop(void ){
#line 84
  unsigned char result;
#line 84

#line 84
  result = CC2420TransmitP__StdControl__stop();
#line 84
  result = ecombine(result, CC2420ReceiveP__StdControl__stop());
#line 84

#line 84
  return result;
#line 84
}
#line 84
# 272 "/home/tinyos/local/src/tinyos-2.x/tos/chips/cc2420/csma/CC2420CsmaP.nc"
static inline void CC2420CsmaP__shutdown(void )
#line 272
{
  CC2420CsmaP__SubControl__stop();
  CC2420CsmaP__CC2420Power__stopVReg();
  CC2420CsmaP__stopDone_task__postTask();
}

#line 96
static inline error_t CC2420CsmaP__SplitControl__stop(void )
#line 96
{
  if (CC2420CsmaP__SplitControlState__isState(CC2420CsmaP__S_STARTED)) {
      CC2420CsmaP__SplitControlState__forceState(CC2420CsmaP__S_STOPPING);
      CC2420CsmaP__shutdown();
      return SUCCESS;
    }
  else {
#line 102
    if (CC2420CsmaP__SplitControlState__isState(CC2420CsmaP__S_STOPPED)) {
        return EALREADY;
      }
    else {
#line 105
      if (CC2420CsmaP__SplitControlState__isState(CC2420CsmaP__S_TRANSMITTING)) {
          CC2420CsmaP__SplitControlState__forceState(CC2420CsmaP__S_STOPPING);

          return SUCCESS;
        }
      else {
#line 110
        if (CC2420CsmaP__SplitControlState__isState(CC2420CsmaP__S_STOPPING)) {
            return SUCCESS;
          }
        }
      }
    }
#line 114
  return EBUSY;
}

# 109 "/home/tinyos/local/src/tinyos-2.x/tos/interfaces/SplitControl.nc"
inline static error_t IPDispatchP__RadioControl__stop(void ){
#line 109
  unsigned char result;
#line 109

#line 109
  result = CC2420CsmaP__SplitControl__stop();
#line 109

#line 109
  return result;
#line 109
}
#line 109
# 56 "/home/tinyos/local/src/tinyos-2.x/tos/chips/msp430/pins/HplMsp430GeneralIOP.nc"
static inline void /*HplMsp430GeneralIOC.P41*/HplMsp430GeneralIOP__25__IO__selectIOFunc(void )
#line 56
{
  /* atomic removed: atomic calls only */
#line 56
  * (volatile uint8_t * )31U &= ~(0x01 << 1);
}

# 85 "/home/tinyos/local/src/tinyos-2.x/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
inline static void /*HplCC2420InterruptsC.CaptureSFDC*/GpioCaptureC__0__GeneralIO__selectIOFunc(void ){
#line 85
  /*HplMsp430GeneralIOC.P41*/HplMsp430GeneralIOP__25__IO__selectIOFunc();
#line 85
}
#line 85
# 124 "/home/tinyos/local/src/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static inline void /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__Control__disableEvents(void )
{
  * (volatile uint16_t * )388U &= ~0x0010;
}

# 47 "/home/tinyos/local/src/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerControl.nc"
inline static void /*HplCC2420InterruptsC.CaptureSFDC*/GpioCaptureC__0__Msp430TimerControl__disableEvents(void ){
#line 47
  /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__Control__disableEvents();
#line 47
}
#line 47
# 58 "/home/tinyos/local/src/tinyos-2.x/tos/chips/msp430/timer/GpioCaptureC.nc"
static inline void /*HplCC2420InterruptsC.CaptureSFDC*/GpioCaptureC__0__Capture__disable(void )
#line 58
{
  /* atomic removed: atomic calls only */
#line 59
  {
    /*HplCC2420InterruptsC.CaptureSFDC*/GpioCaptureC__0__Msp430TimerControl__disableEvents();
    /*HplCC2420InterruptsC.CaptureSFDC*/GpioCaptureC__0__GeneralIO__selectIOFunc();
  }
}

# 55 "/home/tinyos/local/src/tinyos-2.x/tos/interfaces/GpioCapture.nc"
inline static void CC2420TransmitP__CaptureSFD__disable(void ){
#line 55
  /*HplCC2420InterruptsC.CaptureSFDC*/GpioCaptureC__0__Capture__disable();
#line 55
}
#line 55
# 91 "/home/tinyos/local/src/tinyos-2.x/tos/chips/msp430/pins/HplMsp430InterruptP.nc"
static inline void HplMsp430InterruptP__Port10__clear(void )
#line 91
{
#line 91
  P1IFG &= ~(1 << 0);
}

# 41 "/home/tinyos/local/src/tinyos-2.x/tos/chips/msp430/pins/HplMsp430Interrupt.nc"
inline static void /*HplCC2420InterruptsC.InterruptFIFOPC*/Msp430InterruptC__1__HplInterrupt__clear(void ){
#line 41
  HplMsp430InterruptP__Port10__clear();
#line 41
}
#line 41
# 83 "/home/tinyos/local/src/tinyos-2.x/tos/chips/msp430/pins/HplMsp430InterruptP.nc"
static inline void HplMsp430InterruptP__Port10__disable(void )
#line 83
{
#line 83
  P1IE &= ~(1 << 0);
}

# 36 "/home/tinyos/local/src/tinyos-2.x/tos/chips/msp430/pins/HplMsp430Interrupt.nc"
inline static void /*HplCC2420InterruptsC.InterruptFIFOPC*/Msp430InterruptC__1__HplInterrupt__disable(void ){
#line 36
  HplMsp430InterruptP__Port10__disable();
#line 36
}
#line 36
# 58 "/home/tinyos/local/src/tinyos-2.x/tos/chips/msp430/pins/Msp430InterruptC.nc"
static inline error_t /*HplCC2420InterruptsC.InterruptFIFOPC*/Msp430InterruptC__1__Interrupt__disable(void )
#line 58
{
  /* atomic removed: atomic calls only */
#line 59
  {
    /*HplCC2420InterruptsC.InterruptFIFOPC*/Msp430InterruptC__1__HplInterrupt__disable();
    /*HplCC2420InterruptsC.InterruptFIFOPC*/Msp430InterruptC__1__HplInterrupt__clear();
  }
  return SUCCESS;
}

# 50 "/home/tinyos/local/src/tinyos-2.x/tos/interfaces/GpioInterrupt.nc"
inline static error_t CC2420ReceiveP__InterruptFIFOP__disable(void ){
#line 50
  unsigned char result;
#line 50

#line 50
  result = /*HplCC2420InterruptsC.InterruptFIFOPC*/Msp430InterruptC__1__Interrupt__disable();
#line 50

#line 50
  return result;
#line 50
}
#line 50
# 46 "/home/tinyos/local/src/tinyos-2.x/tos/chips/msp430/pins/HplMsp430GeneralIOP.nc"
static inline void /*HplMsp430GeneralIOC.P45*/HplMsp430GeneralIOP__29__IO__clr(void )
#line 46
{
#line 46
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 46
    * (volatile uint8_t * )29U &= ~(0x01 << 5);
#line 46
    __nesc_atomic_end(__nesc_atomic); }
}

# 39 "/home/tinyos/local/src/tinyos-2.x/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
inline static void /*HplCC2420PinsC.VRENM*/Msp430GpioC__9__HplGeneralIO__clr(void ){
#line 39
  /*HplMsp430GeneralIOC.P45*/HplMsp430GeneralIOP__29__IO__clr();
#line 39
}
#line 39
# 38 "/home/tinyos/local/src/tinyos-2.x/tos/chips/msp430/pins/Msp430GpioC.nc"
static inline void /*HplCC2420PinsC.VRENM*/Msp430GpioC__9__GeneralIO__clr(void )
#line 38
{
#line 38
  /*HplCC2420PinsC.VRENM*/Msp430GpioC__9__HplGeneralIO__clr();
}

# 30 "/home/tinyos/local/src/tinyos-2.x/tos/interfaces/GeneralIO.nc"
inline static void CC2420ControlP__VREN__clr(void ){
#line 30
  /*HplCC2420PinsC.VRENM*/Msp430GpioC__9__GeneralIO__clr();
#line 30
}
#line 30
# 75 "/home/tinyos/local/src/tinyos-2.x/tos/chips/cc2420/packet/CC2420PacketP.nc"
static inline bool CC2420PacketP__Acks__wasAcked(message_t *p_msg)
#line 75
{
  return __nesc_ntoh_int8(CC2420PacketP__CC2420PacketBody__getMetadata(p_msg)->ack.data);
}

# 74 "/home/tinyos/local/src/tinyos-2.x/tos/interfaces/PacketAcknowledgements.nc"
inline static bool PacketLinkP__PacketAcknowledgements__wasAcked(message_t * msg){
#line 74
  unsigned char result;
#line 74

#line 74
  result = CC2420PacketP__Acks__wasAcked(msg);
#line 74

#line 74
  return result;
#line 74
}
#line 74
# 118 "/home/tinyos/local/src/tinyos-2.x/tos/chips/cc2420/link/PacketLinkP.nc"
static inline bool PacketLinkP__PacketLink__wasDelivered(message_t *msg)
#line 118
{
  return PacketLinkP__PacketAcknowledgements__wasAcked(msg);
}

# 71 "/home/tinyos/local/src/tinyos-2.x/tos/interfaces/PacketLink.nc"
inline static bool IPDispatchP__PacketLink__wasDelivered(message_t * msg){
#line 71
  unsigned char result;
#line 71

#line 71
  result = PacketLinkP__PacketLink__wasDelivered(msg);
#line 71

#line 71
  return result;
#line 71
}
#line 71
#line 59
inline static uint16_t IPDispatchP__PacketLink__getRetries(message_t * msg){
#line 59
  unsigned int result;
#line 59

#line 59
  result = PacketLinkP__PacketLink__getRetries(msg);
#line 59

#line 59
  return result;
#line 59
}
#line 59
# 1019 "/home/tinyos/local/src/tinyos-2.x/tos/lib/net/blip/IPRoutingP.nc"
static inline void IPRoutingP__IPExtensions__reportTransmission(uint8_t label, send_policy_t *policy)
#line 1019
{
  int i;
  struct neigh_entry *e = (void *)0;









  if (policy->dest[0] != IEEE154_BROADCAST_ADDR) {

      ;



      for (i = 0; i < policy->current; i++) {
          e = IPRoutingP__getNeighEntry(policy->dest[i]);
          if (e != (void *)0) {

              e->stats[IPRoutingP__SHORT_EPOCH].total += policy->retries;

              if (e == IPRoutingP__default_route) {
                  IPRoutingP__default_route_failures++;
                }

              ;
            }
        }



      if (IPRoutingP__default_route_failures > MAX_CONSEC_FAILURES) {
          ;
          IPRoutingP__chooseNewRandomDefault(TRUE);
        }


      e = IPRoutingP__getNeighEntry(policy->dest[policy->current]);
      if (policy->current < policy->nchoices && e != (void *)0) {
          e->stats[IPRoutingP__SHORT_EPOCH].success += 1;
          e->stats[IPRoutingP__SHORT_EPOCH].total += policy->actRetries;

          ;

          ;

          if (e == IPRoutingP__default_route) {
            IPRoutingP__default_route_failures++;
            }

          if (e != &IPRoutingP__neigh_table[0] && ((

          IPRoutingP__getConfidence(e) > CONF_PROM_THRESHOLD && 
          IPRoutingP__checkThresh(IPRoutingP__getMetric(e), IPRoutingP__getMetric(e - 1), PATH_COST_DIFF_THRESH) == BELOW_THRESH) || (

          IPRoutingP__checkThresh(IPRoutingP__getMetric(e), IPRoutingP__getMetric(e - 1), PATH_COST_DIFF_THRESH) == WITHIN_THRESH && 
          IPRoutingP__getConfidence(e) > CONF_PROM_THRESHOLD))) {

              ;
              IPRoutingP__swapNodes(e - 1, e);
            }
        }
      else 

        {
          ;
        }
    }
}

# 17 "/home/tinyos/local/src/tinyos-2.x/tos/lib/net/blip/interfaces/IPExtensions.nc"
inline static void IPDispatchP__IPExtensions__reportTransmission(uint8_t label, send_policy_t *send){
#line 17
  IPRoutingP__IPExtensions__reportTransmission(label, send);
#line 17
}
#line 17
# 646 "/home/tinyos/local/src/tinyos-2.x/tos/lib/net/blip/IPRoutingP.nc"
static inline uint16_t IPRoutingP__getSuccess(struct neigh_entry *neigh)
#line 646
{

  uint16_t succ = 0;

#line 649
  if (neigh != (void *)0 && (neigh->flags & T_VALID_MASK) == T_VALID_MASK) {



      succ += neigh->stats[IPRoutingP__LONG_EPOCH].success;
    }
  return succ;
}

# 257 "/usr/lib/ncc/nesc_nx.h"
static __inline  int8_t __nesc_hton_int8(void * target, int8_t value)
#line 257
{
#line 257
  __nesc_hton_uint8(target, value);
#line 257
  return value;
}

# 90 "/home/tinyos/local/src/tinyos-2.x/tos/interfaces/Queue.nc"
inline static error_t IPDispatchP__SendQueue__enqueue(IPDispatchP__SendQueue__t  newVal){
#line 90
  unsigned char result;
#line 90

#line 90
  result = /*IPDispatchC.QueueC*/QueueC__0__Queue__enqueue(newVal);
#line 90

#line 90
  return result;
#line 90
}
#line 90
# 97 "/home/tinyos/local/src/tinyos-2.x/tos/interfaces/Pool.nc"
inline static IPDispatchP__SendEntryPool__t * IPDispatchP__SendEntryPool__get(void ){
#line 97
  struct __nesc_unnamed4306 *result;
#line 97

#line 97
  result = /*IPDispatchC.SendEntryPool.PoolP*/PoolP__1__Pool__get();
#line 97

#line 97
  return result;
#line 97
}
#line 97
inline static IPDispatchP__FragPool__t * IPDispatchP__FragPool__get(void ){
#line 97
  nx_struct message_t *result;
#line 97

#line 97
  result = /*IPDispatchC.FragPool.PoolP*/PoolP__0__Pool__get();
#line 97

#line 97
  return result;
#line 97
}
#line 97
# 281 "/home/tinyos/local/src/tinyos-2.x/tos/lib/net/blip/IPDispatchP.nc"
static inline void IPDispatchP__signalDone(reconstruct_t *recon)
#line 281
{
  struct ip6_hdr *iph = (struct ip6_hdr *)recon->buf;

  IPDispatchP__IP__recv(recon->nxt_hdr, iph, recon->transport_hdr, & recon->metadata);
  ip_free(recon->buf);
  recon->timeout = T_UNUSED;
  recon->buf = (void *)0;
}

#line 199
static inline int IPDispatchP__forward_lookup(void *ent)
#line 199
{
  forward_entry_t *fwd = (forward_entry_t *)ent;

  if (
#line 201
  fwd->timeout > T_UNUSED && 
  fwd->l2_src == IPDispatchP__forward_lookup_src && 
  fwd->old_tag == IPDispatchP__forward_lookup_tag) {
      fwd->timeout = T_ACTIVE;
      return 1;
    }
  return 0;
}

# 30 "/home/tinyos/local/src/tinyos-2.x/tos/interfaces/Ieee154Packet.nc"
inline static ieee154_saddr_t IPDispatchP__Ieee154Packet__source(message_t *msg){
#line 30
  unsigned int result;
#line 30

#line 30
  result = CC2420Ieee154MessageP__Ieee154Packet__source(msg);
#line 30

#line 30
  return result;
#line 30
}
#line 30
# 190 "/home/tinyos/local/src/tinyos-2.x/tos/lib/net/blip/IPDispatchP.nc"
static inline int IPDispatchP__forward_unused(void *ent)
#line 190
{
  forward_entry_t *fwd = (forward_entry_t *)ent;

#line 192
  if (fwd->timeout == T_UNUSED) {
    return 1;
    }
#line 194
  return 0;
}

# 48 "/home/tinyos/local/src/tinyos-2.x/tos/lib/net/blip/interfaces/IPRouting.nc"
inline static error_t IPDispatchP__IPRouting__getNextHop(struct ip6_hdr *hdr, struct ip6_route *routing_hdr, ieee154_saddr_t prev_hop, send_policy_t *ret){
#line 48
  unsigned char result;
#line 48

#line 48
  result = IPRoutingP__IPRouting__getNextHop(hdr, routing_hdr, prev_hop, ret);
#line 48

#line 48
  return result;
#line 48
}
#line 48
# 15 "/home/tinyos/local/src/tinyos-2.x/tos/lib/net/blip/interfaces/IP.nc"
inline static error_t ICMPResponderP__IP__send(struct split_ip_msg *msg){
#line 15
  unsigned char result;
#line 15

#line 15
  result = IPDispatchP__IP__send(IANA_ICMP, msg);
#line 15

#line 15
  return result;
#line 15
}
#line 15
# 66 "/home/tinyos/local/src/tinyos-2.x/tos/lib/net/blip/ICMPResponderP.nc"
static inline uint16_t ICMPResponderP__ICMP__cksum(struct split_ip_msg *msg, uint8_t nxt_hdr)
#line 66
{
  return msg_cksum(msg, nxt_hdr);
}

# 30 "/home/tinyos/local/src/tinyos-2.x/tos/lib/net/blip/interfaces/IPAddress.nc"
inline static void ICMPResponderP__IPAddress__getIPAddr(struct in6_addr *addr){
#line 30
  IPAddressP__IPAddress__getIPAddr(addr);
#line 30
}
#line 30
# 87 "/home/tinyos/local/src/tinyos-2.x/tos/lib/net/blip/ICMPResponderP.nc"
static inline void ICMPResponderP__ICMP__sendTimeExceeded(struct ip6_hdr *hdr, unpack_info_t *u_info, uint16_t amount_here)
#line 87
{
  uint8_t i_hdr_buf[sizeof(struct icmp6_hdr ) + 4];
  struct split_ip_msg *msg = (struct split_ip_msg *)ip_malloc(sizeof(struct split_ip_msg ));
  struct generic_header g_hdr[3];
  struct icmp6_hdr *i_hdr = (struct icmp6_hdr *)i_hdr_buf;

  if (msg == (void *)0) {
#line 93
    return;
    }
  ;

  msg->headers = (void *)0;
  msg->data = u_info->payload_start;
  msg->data_len = amount_here;


  if (u_info->nxt_hdr == IANA_UDP) {
      g_hdr[2].hdr.udp = (struct udp_hdr *)u_info->transport_ptr;
      g_hdr[2].len = sizeof(struct udp_hdr );
      g_hdr[2].next = (void *)0;




      hdr->plen = (((uint16_t )(((((uint16_t )hdr->plen >> 8) | ((uint16_t )hdr->plen << 8)) & 0xffff) + sizeof(struct udp_hdr )) << 8) | ((uint16_t )(((((uint16_t )hdr->plen >> 8) | ((uint16_t )hdr->plen << 8)) & 0xffff) + sizeof(struct udp_hdr )) >> 8)) & 0xffff;
      msg->headers = &g_hdr[2];
    }



  hdr->nxt_hdr = u_info->nxt_hdr;
  hdr->plen = (((uint16_t )(((((uint16_t )hdr->plen >> 8) | ((uint16_t )hdr->plen << 8)) & 0xffff) - u_info->payload_offset) << 8) | ((uint16_t )(((((uint16_t )hdr->plen >> 8) | ((uint16_t )hdr->plen << 8)) & 0xffff) - u_info->payload_offset) >> 8)) & 0xffff;


  g_hdr[1].hdr.data = (void *)hdr;
  g_hdr[1].len = sizeof(struct ip6_hdr );
  g_hdr[1].next = msg->headers;
  msg->headers = &g_hdr[1];


  g_hdr[0].hdr.data = (void *)i_hdr;
  g_hdr[0].len = sizeof(struct icmp6_hdr ) + 4;
  g_hdr[0].next = msg->headers;
  msg->headers = &g_hdr[0];

  ip_memcpy(& msg->hdr.ip6_dst, & hdr->ip6_src, 16);
  ICMPResponderP__IPAddress__getIPAddr(& msg->hdr.ip6_src);

  i_hdr->type = ICMP_TYPE_ECHO_TIME_EXCEEDED;
  i_hdr->code = ICMP_CODE_HOPLIMIT_EXCEEDED;
  i_hdr->cksum = 0;
  ip_memclr((void *)(i_hdr + 1), 4);

  msg->hdr.nxt_hdr = IANA_ICMP;

  i_hdr->cksum = (((uint16_t )ICMPResponderP__ICMP__cksum(msg, IANA_ICMP) << 8) | ((uint16_t )ICMPResponderP__ICMP__cksum(msg, IANA_ICMP) >> 8)) & 0xffff;

  ICMPResponderP__IP__send(msg);

  ip_free(msg);
}

# 35 "/home/tinyos/local/src/tinyos-2.x/tos/lib/net/blip/interfaces/ICMP.nc"
inline static void IPDispatchP__ICMP__sendTimeExceeded(struct ip6_hdr *hdr, unpack_info_t *u_info, uint16_t amount_here){
#line 35
  ICMPResponderP__ICMP__sendTimeExceeded(hdr, u_info, amount_here);
#line 35
}
#line 35
# 48 "/home/tinyos/local/src/tinyos-2.x/tos/system/NoLedsC.nc"
static inline void NoLedsC__Leds__led1Toggle(void )
#line 48
{
}

# 72 "/home/tinyos/local/src/tinyos-2.x/tos/interfaces/Leds.nc"
inline static void IPDispatchP__Leds__led1Toggle(void ){
#line 72
  NoLedsC__Leds__led1Toggle();
#line 72
}
#line 72
# 94 "/home/tinyos/local/src/tinyos-2.x/tos/chips/cc2420/packet/CC2420PacketP.nc"
static inline uint8_t CC2420PacketP__CC2420Packet__getLqi(message_t *p_msg)
#line 94
{
  return __nesc_ntoh_uint8(CC2420PacketP__CC2420PacketBody__getMetadata(p_msg)->lqi.data);
}

# 72 "/home/tinyos/local/src/tinyos-2.x/tos/chips/cc2420/interfaces/CC2420Packet.nc"
inline static uint8_t CC2420ReadLqiC__CC2420Packet__getLqi(message_t *p_msg){
#line 72
  unsigned char result;
#line 72

#line 72
  result = CC2420PacketP__CC2420Packet__getLqi(p_msg);
#line 72

#line 72
  return result;
#line 72
}
#line 72
# 12 "/home/tinyos/local/src/tinyos-2.x/tos/lib/net/blip/platform/CC2420ReadLqiC.nc"
static inline uint8_t CC2420ReadLqiC__ReadLqi__read(message_t *msg)
#line 12
{
  return CC2420ReadLqiC__CC2420Packet__getLqi(msg);
}

# 3 "/home/tinyos/local/src/tinyos-2.x/tos/lib/net/blip/interfaces/ReadLqi.nc"
inline static uint8_t IPDispatchP__ReadLqi__read(message_t *msg){
#line 3
  unsigned char result;
#line 3

#line 3
  result = CC2420ReadLqiC__ReadLqi__read(msg);
#line 3

#line 3
  return result;
#line 3
}
#line 3
# 28 "/home/tinyos/local/src/tinyos-2.x/tos/lib/net/blip/interfaces/IPAddress.nc"
inline static struct in6_addr *IPRoutingP__IPAddress__getPublicAddr(void ){
#line 28
  struct in6_addr *result;
#line 28

#line 28
  result = IPAddressP__IPAddress__getPublicAddr();
#line 28

#line 28
  return result;
#line 28
}
#line 28
# 197 "/home/tinyos/local/src/tinyos-2.x/tos/lib/net/blip/IPRoutingP.nc"
static inline bool IPRoutingP__IPRouting__isForMe(struct ip6_hdr *hdr)
#line 197
{



  struct in6_addr *my_address = IPRoutingP__IPAddress__getPublicAddr();

#line 202
  return ((cmpPfx(my_address->in6_u.u6_addr8, hdr->ip6_dst.in6_u.u6_addr8) || 
  cmpPfx(linklocal_prefix, hdr->ip6_dst.in6_u.u6_addr8)) && 
  cmpPfx(&my_address->in6_u.u6_addr8[8], &hdr->ip6_dst.in6_u.u6_addr8[8])) || (
  hdr->ip6_dst.in6_u.u6_addr8[0] == 0xff && (
  hdr->ip6_dst.in6_u.u6_addr8[1] & 0x0f) <= 3);
}

# 40 "/home/tinyos/local/src/tinyos-2.x/tos/lib/net/blip/interfaces/IPRouting.nc"
inline static bool IPDispatchP__IPRouting__isForMe(struct ip6_hdr *a){
#line 40
  unsigned char result;
#line 40

#line 40
  result = IPRoutingP__IPRouting__isForMe(a);
#line 40

#line 40
  return result;
#line 40
}
#line 40
# 513 "/home/tinyos/local/src/tinyos-2.x/tos/lib/net/blip/IPRoutingP.nc"
static inline void IPRoutingP__IPExtensions__handleExtensions(uint8_t label, 
struct ip6_hdr *iph, 
struct ip6_ext *hop, 
struct ip6_ext *dst, 
struct ip6_route *route, 
uint8_t nxt_hdr)
#line 518
{
}

# 6 "/home/tinyos/local/src/tinyos-2.x/tos/lib/net/blip/interfaces/IPExtensions.nc"
inline static void IPDispatchP__IPExtensions__handleExtensions(uint8_t label, struct ip6_hdr *iph, struct ip6_ext *hop, struct ip6_ext *dst, struct ip6_route *route, uint8_t nxt_hdr){
#line 6
  IPRoutingP__IPExtensions__handleExtensions(label, iph, hop, dst, route, nxt_hdr);
#line 6
}
#line 6
# 40 "/home/tinyos/local/src/tinyos-2.x/tos/lib/net/blip/IPAddressP.nc"
static inline ieee154_saddr_t IPAddressP__IPAddress__getShortAddr(void )
#line 40
{
  return TOS_NODE_ID;
}

# 25 "/home/tinyos/local/src/tinyos-2.x/tos/lib/net/blip/interfaces/IPAddress.nc"
inline static ieee154_saddr_t IPDispatchP__IPAddress__getShortAddr(void ){
#line 25
  unsigned int result;
#line 25

#line 25
  result = IPAddressP__IPAddress__getShortAddr();
#line 25

#line 25
  return result;
#line 25
}
#line 25
# 412 "/home/tinyos/local/src/tinyos-2.x/tos/lib/net/blip/IPDispatchP.nc"
static inline void IPDispatchP__updateSourceRoute(ieee154_saddr_t prev_hop, struct ip6_route *sh)
#line 412
{
  uint16_t my_address = IPDispatchP__IPAddress__getShortAddr();
  uint16_t target_hop = sh->hops[(sh->len - sizeof(struct ip6_route )) / sizeof(uint16_t ) - sh->segs_remain];

#line 415
  if ((sh->type & ~IP6ROUTE_FLAG_MASK) == IP6ROUTE_TYPE_INVAL || sh->segs_remain == 0) {
#line 415
    return;
    }
  if (target_hop != ((((uint16_t )my_address << 8) | ((uint16_t )my_address >> 8)) & 0xffff)) {
      ;

      if ((sh->len - sizeof(struct ip6_route )) / sizeof(uint16_t ) >= 2) {
          sh->hops[0] = (((uint16_t )prev_hop << 8) | ((uint16_t )prev_hop >> 8)) & 0xffff;
          sh->hops[1] = target_hop;
        }
      sh->type = (sh->type & IP6ROUTE_FLAG_MASK) | IP6ROUTE_TYPE_INVAL;
    }
  else 
#line 425
    {
      sh->hops[(sh->len - sizeof(struct ip6_route )) / sizeof(uint16_t ) - sh->segs_remain] = (((uint16_t )prev_hop << 8) | ((uint16_t )prev_hop >> 8)) & 0xffff;
      sh->segs_remain--;
      ;
    }
}


static inline message_t *IPDispatchP__handle1stFrag(message_t *msg, packed_lowmsg_t *lowmsg)
#line 433
{
  unsigned int __nesc_temp53;
  unsigned char *__nesc_temp52;
  unsigned char __nesc_temp51;
  unsigned char *__nesc_temp50;
#line 434
  uint8_t *unpack_buf;
  struct ip6_hdr *ip;

  uint16_t real_payload_length;

  unpack_info_t u_info;

  unpack_buf = ip_malloc(LIB6LOWPAN_MAX_LEN + LOWPAN_LINK_MTU);
  if (unpack_buf == (void *)0) {
#line 442
    return msg;
    }



  ip_memclr(unpack_buf, LIB6LOWPAN_MAX_LEN + LOWPAN_LINK_MTU);

  if (
#line 448
  unpackHeaders(lowmsg, &u_info, 
  unpack_buf, LIB6LOWPAN_MAX_LEN) == (void *)0) {
      ip_free(unpack_buf);
      return msg;
    }

  ip = (struct ip6_hdr *)unpack_buf;


  if (u_info.hdr_route != (void *)0) {


      IPDispatchP__updateSourceRoute(IPDispatchP__Ieee154Packet__source(msg), 
      u_info.hdr_route);
    }


  IPDispatchP__IPExtensions__handleExtensions(IPDispatchP__current_local_label++, 
  ip, 
  u_info.hdr_hop, 
  u_info.hdr_dest, 
  u_info.hdr_route, 
  u_info.nxt_hdr);


  if (IPDispatchP__IPRouting__isForMe(ip)) {
      struct ip_metadata metadata;

#line 475
      ;





      metadata.sender = IPDispatchP__Ieee154Packet__source(msg);
      metadata.lqi = IPDispatchP__ReadLqi__read(msg);

      real_payload_length = (((uint16_t )ip->plen >> 8) | ((uint16_t )ip->plen << 8)) & 0xffff;
      adjustPlen(ip, &u_info);

      if (!hasFrag1Header(lowmsg)) {
          uint16_t amount_here = lowmsg->len - (u_info.payload_start - lowmsg->data);

#line 506
          ip_memcpy(u_info.header_end, u_info.payload_start, amount_here);

          ;
          IPDispatchP__IP__recv(u_info.nxt_hdr, ip, u_info.transport_ptr, &metadata);
        }
      else 
#line 510
        {



          reconstruct_t *recon;
          uint16_t tag;
#line 515
          uint16_t amount_here = lowmsg->len - (u_info.payload_start - lowmsg->data);
          void *rcv_buf;

          if (getFragDgramTag(lowmsg, &tag)) {
#line 518
            goto fail;
            }
          ;
          recon = IPDispatchP__get_reconstruct(lowmsg->src, tag);


          if (recon == (void *)0) {
              goto fail;
            }


          rcv_buf = ip_malloc(real_payload_length + sizeof(struct ip6_hdr ));

          recon->metadata.sender = lowmsg->src;
          recon->tag = tag;
          recon->size = real_payload_length + sizeof(struct ip6_hdr );
          recon->buf = rcv_buf;
          recon->nxt_hdr = u_info.nxt_hdr;
          recon->transport_hdr = (uint8_t *)rcv_buf + (u_info.transport_ptr - unpack_buf);
          recon->bytes_rcvd = u_info.payload_offset + amount_here + sizeof(struct ip6_hdr );
          recon->timeout = T_ACTIVE;

          if (rcv_buf == (void *)0) {

              recon->timeout = T_FAILED1;
              recon->size = 0;
              goto fail;
            }
          if (amount_here > recon->size - sizeof(struct ip6_hdr )) {
              IPDispatchP__Leds__led1Toggle();
              recon->timeout = T_FAILED1;
              recon->size = 0;
              ip_free(rcv_buf);
              recon->buf = (void *)0;
              goto fail;
            }

          ip_memcpy(rcv_buf, unpack_buf, u_info.payload_offset + sizeof(struct ip6_hdr ));
          ip_memcpy(rcv_buf + u_info.payload_offset + sizeof(struct ip6_hdr ), 
          u_info.payload_start, amount_here);
          ip_memcpy(& recon->metadata, &metadata, sizeof(struct ip_metadata ));

          goto done;
        }
    }
  else {


      send_info_t *s_info;
      send_entry_t *s_entry;
      forward_entry_t *fwd;
      message_t *msg_replacement;


      * u_info.hlim = * u_info.hlim - 1;
      if (* u_info.hlim == 0) {

          uint16_t amount_here = lowmsg->len - (u_info.payload_start - lowmsg->data);

#line 576
          IPDispatchP__ICMP__sendTimeExceeded(ip, &u_info, amount_here);





          ip_free(unpack_buf);
          return msg;
        }
      s_info = IPDispatchP__getSendInfo();
      s_entry = IPDispatchP__SendEntryPool__get();
      msg_replacement = IPDispatchP__FragPool__get();
      if ((s_info == (void *)0 || s_entry == (void *)0) || msg_replacement == (void *)0) {
          if (s_info != (void *)0) {
            if (-- s_info->refcount == 0) {
#line 590
              IPDispatchP__SendInfoPool__put(s_info);
              }
            }
#line 591
          if (s_entry != (void *)0) {
            IPDispatchP__SendEntryPool__put(s_entry);
            }
#line 593
          if (msg_replacement != (void *)0) {
            IPDispatchP__FragPool__put(msg_replacement);
            }
#line 595
          goto fail;
        }


      if (
#line 598
      IPDispatchP__IPRouting__getNextHop(ip, u_info.hdr_route, 
      lowmsg->src, & s_info->policy) != SUCCESS) {
        goto fwd_fail;
        }
      ;

      if (hasFrag1Header(lowmsg)) {
          fwd = table_search(&IPDispatchP__forward_cache, IPDispatchP__forward_unused);
          if (fwd == (void *)0) {
              goto fwd_fail;
            }

          fwd->timeout = T_ACTIVE;
          fwd->l2_src = IPDispatchP__Ieee154Packet__source(msg);
          getFragDgramTag(lowmsg, & fwd->old_tag);
          fwd->new_tag = ++lib6lowpan_frag_tag;

          s_info->refcount++;
          fwd->s_info = s_info;
          setFragDgramTag(lowmsg, lib6lowpan_frag_tag);
        }


      s_info->refcount++;
      s_info->local_flow_label = IPDispatchP__current_local_label - 1;
      s_entry->msg = msg;
      s_entry->info = s_info;

      if (IPDispatchP__SendQueue__enqueue(s_entry) != SUCCESS) {
        (__nesc_temp50 = IPDispatchP__stats.encfail.data, __nesc_hton_uint8(__nesc_temp50, (__nesc_temp51 = __nesc_ntoh_uint8(__nesc_temp50)) + 1), __nesc_temp51);
        }
#line 628
      IPDispatchP__sendTask__postTask();

      (__nesc_temp52 = IPDispatchP__stats.forwarded.data, __nesc_hton_uint16(__nesc_temp52, (__nesc_temp53 = __nesc_ntoh_uint16(__nesc_temp52)) + 1), __nesc_temp53);


      if (-- s_info->refcount == 0) {
#line 633
        IPDispatchP__SendInfoPool__put(s_info);
        }
#line 634
      ip_free(unpack_buf);
      return msg_replacement;

      fwd_fail: 
        IPDispatchP__FragPool__put(msg_replacement);
      IPDispatchP__SendInfoPool__put(s_info);
      IPDispatchP__SendEntryPool__put(s_entry);
    }



  fail: 
    done: 
      ip_free(unpack_buf);
  return msg;
}

# 1001 "/home/tinyos/local/src/tinyos-2.x/tos/lib/net/blip/IPRoutingP.nc"
static inline void IPRoutingP__IPRouting__reportReception(ieee154_saddr_t neigh, uint8_t lqi)
#line 1001
{
  struct neigh_entry *e = IPRoutingP__getNeighEntry(neigh);

#line 1003
  ;

  if (e != (void *)0) {
      e->linkEstimate = adjustLQI(lqi);
    }
}

# 74 "/home/tinyos/local/src/tinyos-2.x/tos/lib/net/blip/interfaces/IPRouting.nc"
inline static void IPDispatchP__IPRouting__reportReception(ieee154_saddr_t neigh, uint8_t lqi){
#line 74
  IPRoutingP__IPRouting__reportReception(neigh, lqi);
#line 74
}
#line 74
# 651 "/home/tinyos/local/src/tinyos-2.x/tos/lib/net/blip/IPDispatchP.nc"
static inline message_t *IPDispatchP__Ieee154Receive__receive(message_t *msg, void *msg_payload, uint8_t len)
#line 651
{
  unsigned char __nesc_temp61;
  unsigned char *__nesc_temp60;
  unsigned char __nesc_temp59;
  unsigned char *__nesc_temp58;
  unsigned char __nesc_temp57;
  unsigned char *__nesc_temp56;
  unsigned char __nesc_temp55;
  unsigned char *__nesc_temp54;
#line 652
  packed_lowmsg_t lowmsg;

  ;

  lowmsg.data = msg_payload;
  lowmsg.len = len;
  lowmsg.src = IPDispatchP__Ieee154Packet__source(msg);
  lowmsg.dst = IPDispatchP__Ieee154Packet__destination(msg);

  ;

  (__nesc_temp54 = IPDispatchP__stats.rx_total.data, __nesc_hton_uint8(__nesc_temp54, (__nesc_temp55 = __nesc_ntoh_uint8(__nesc_temp54)) + 1), __nesc_temp55);

  IPDispatchP__IPRouting__reportReception(IPDispatchP__Ieee154Packet__source(msg), 
  IPDispatchP__ReadLqi__read(msg));

  lowmsg.headers = getHeaderBitmap(&lowmsg);
  if (lowmsg.headers == LOWPAN_NALP_PATTERN) {
      goto fail;
    }


  if (!hasFragNHeader(&lowmsg)) {


      msg = IPDispatchP__handle1stFrag(msg, &lowmsg);
      goto done;
    }
  else 
#line 679
    {


      forward_entry_t *fwd;
      reconstruct_t *recon;
      uint8_t offset_cmpr;
      uint16_t offset;
#line 685
      uint16_t amount_here;
#line 685
      uint16_t tag;
      uint8_t *payload;

      if (getFragDgramTag(&lowmsg, &tag)) {
#line 688
        goto fail;
        }
#line 689
      if (getFragDgramOffset(&lowmsg, &offset_cmpr)) {
#line 689
        goto fail;
        }
      IPDispatchP__forward_lookup_tag = tag;
      IPDispatchP__forward_lookup_src = IPDispatchP__Ieee154Packet__source(msg);

      fwd = table_search(&IPDispatchP__forward_cache, IPDispatchP__forward_lookup);
      payload = getLowpanPayload(&lowmsg);

      recon = IPDispatchP__get_reconstruct(lowmsg.src, tag);
      if (recon != (void *)0 && recon->timeout > T_UNUSED && recon->buf != (void *)0) {


          offset = offset_cmpr * 8;
          amount_here = lowmsg.len - (payload - lowmsg.data);

          if (offset + amount_here > recon->size) {
#line 704
            goto fail;
            }
#line 705
          ip_memcpy(recon->buf + offset, payload, amount_here);

          recon->bytes_rcvd += amount_here;

          ;
          if (recon->size == recon->bytes_rcvd) {

              IPDispatchP__signalDone(recon);
            }
        }
      else {
#line 714
        if (fwd != (void *)0 && fwd->timeout > T_UNUSED) {


            message_t *replacement = IPDispatchP__FragPool__get();
            send_entry_t *s_entry = IPDispatchP__SendEntryPool__get();
            uint16_t lowpan_size;
            uint8_t lowpan_offset;

            if (replacement == (void *)0 || s_entry == (void *)0) {


                if (replacement != (void *)0) {
                  IPDispatchP__FragPool__put(replacement);
                  }
#line 727
                if (s_entry != (void *)0) {
                  IPDispatchP__SendEntryPool__put(s_entry);
                  }
                (__nesc_temp56 = IPDispatchP__stats.fw_drop.data, __nesc_hton_uint8(__nesc_temp56, (__nesc_temp57 = __nesc_ntoh_uint8(__nesc_temp56)) + 1), __nesc_temp57);
                fwd->timeout = T_FAILED1;
                goto fail;
              }


            fwd->s_info->refcount++;

            getFragDgramOffset(&lowmsg, &lowpan_offset);
            getFragDgramSize(&lowmsg, &lowpan_size);
            if (lowpan_offset * 8 + (lowmsg.len - (payload - lowmsg.data)) == lowpan_size) {



                if (-- fwd->s_info->refcount == 0) {
#line 744
                  IPDispatchP__SendInfoPool__put(fwd->s_info);
                  }
#line 745
                fwd->timeout = T_UNUSED;
              }

            setFragDgramTag(&lowmsg, fwd->new_tag);

            s_entry->msg = msg;
            s_entry->info = fwd->s_info;

            ;


            if (IPDispatchP__SendQueue__enqueue(s_entry) != SUCCESS) {
                (__nesc_temp58 = IPDispatchP__stats.encfail.data, __nesc_hton_uint8(__nesc_temp58, (__nesc_temp59 = __nesc_ntoh_uint8(__nesc_temp58)) + 1), __nesc_temp59);
                ;
              }
            IPDispatchP__sendTask__postTask();
            return replacement;
          }
        else {
#line 763
          goto fail;
          }
        }
#line 764
      goto done;
    }

  fail: 
    ;
#line 768
  ;
  (__nesc_temp60 = IPDispatchP__stats.rx_drop.data, __nesc_hton_uint8(__nesc_temp60, (__nesc_temp61 = __nesc_ntoh_uint8(__nesc_temp60)) + 1), __nesc_temp61);
  done: 
    return msg;
}

# 67 "/home/tinyos/local/src/tinyos-2.x/tos/interfaces/Receive.nc"
inline static message_t * CC2420TinyosNetworkP__BareReceive__receive(message_t * msg, void * payload, uint8_t len){
#line 67
  nx_struct message_t *result;
#line 67

#line 67
  result = IPDispatchP__Ieee154Receive__receive(msg, payload, len);
#line 67

#line 67
  return result;
#line 67
}
#line 67
# 42 "/home/tinyos/local/src/tinyos-2.x/tos/chips/cc2420/interfaces/CC2420PacketBody.nc"
inline static cc2420_header_t * CC2420TinyosNetworkP__CC2420PacketBody__getHeader(message_t * msg){
#line 42
  nx_struct cc2420_header_t *result;
#line 42

#line 42
  result = CC2420PacketP__CC2420PacketBody__getHeader(msg);
#line 42

#line 42
  return result;
#line 42
}
#line 42
# 216 "/home/tinyos/local/src/tinyos-2.x/tos/chips/cc2420/lowpan/CC2420TinyosNetworkP.nc"
static inline message_t *CC2420TinyosNetworkP__ActiveReceive__default__receive(message_t *msg, void *payload, uint8_t len)
#line 216
{
  return msg;
}

# 67 "/home/tinyos/local/src/tinyos-2.x/tos/interfaces/Receive.nc"
inline static message_t * CC2420TinyosNetworkP__ActiveReceive__receive(message_t * msg, void * payload, uint8_t len){
#line 67
  nx_struct message_t *result;
#line 67

#line 67
  result = CC2420TinyosNetworkP__ActiveReceive__default__receive(msg, payload, len);
#line 67

#line 67
  return result;
#line 67
}
#line 67
# 75 "/home/tinyos/local/src/tinyos-2.x/tos/chips/cc2420/interfaces/CC2420Packet.nc"
inline static uint8_t CC2420TinyosNetworkP__CC2420Packet__getNetwork(message_t *p_msg){
#line 75
  unsigned char result;
#line 75

#line 75
  result = CC2420PacketP__CC2420Packet__getNetwork(p_msg);
#line 75

#line 75
  return result;
#line 75
}
#line 75
# 47 "/home/tinyos/local/src/tinyos-2.x/tos/chips/cc2420/interfaces/CC2420PacketBody.nc"
inline static cc2420_metadata_t * CC2420TinyosNetworkP__CC2420PacketBody__getMetadata(message_t * msg){
#line 47
  nx_struct cc2420_metadata_t *result;
#line 47

#line 47
  result = CC2420PacketP__CC2420PacketBody__getMetadata(msg);
#line 47

#line 47
  return result;
#line 47
}
#line 47
# 127 "/home/tinyos/local/src/tinyos-2.x/tos/chips/cc2420/lowpan/CC2420TinyosNetworkP.nc"
static inline message_t *CC2420TinyosNetworkP__SubReceive__receive(message_t *msg, void *payload, uint8_t len)
#line 127
{

  if (! __nesc_ntoh_int8(CC2420TinyosNetworkP__CC2420PacketBody__getMetadata(msg)->crc.data)) {
      return msg;
    }

  if (CC2420TinyosNetworkP__CC2420Packet__getNetwork(msg) == 0x3f) {
      return CC2420TinyosNetworkP__ActiveReceive__receive(msg, payload, len);
    }
  else 
#line 135
    {
      cc2420_header_t *hdr = CC2420TinyosNetworkP__CC2420PacketBody__getHeader(msg);

#line 137
      return CC2420TinyosNetworkP__BareReceive__receive(msg, & hdr->network, len + AM_OVERHEAD);
    }
}

# 67 "/home/tinyos/local/src/tinyos-2.x/tos/interfaces/Receive.nc"
inline static message_t * UniqueReceiveP__Receive__receive(message_t * msg, void * payload, uint8_t len){
#line 67
  nx_struct message_t *result;
#line 67

#line 67
  result = CC2420TinyosNetworkP__SubReceive__receive(msg, payload, len);
#line 67

#line 67
  return result;
#line 67
}
#line 67
# 137 "/home/tinyos/local/src/tinyos-2.x/tos/chips/cc2420/unique/UniqueReceiveP.nc"
static inline void UniqueReceiveP__insert(uint16_t msgSource, uint8_t msgDsn)
#line 137
{
  uint8_t element = UniqueReceiveP__recycleSourceElement;
  bool increment = FALSE;

  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 141
    {
      if (element == UniqueReceiveP__INVALID_ELEMENT || UniqueReceiveP__writeIndex == element) {

          element = UniqueReceiveP__writeIndex;
          increment = TRUE;
        }

      UniqueReceiveP__receivedMessages[element].source = msgSource;
      UniqueReceiveP__receivedMessages[element].dsn = msgDsn;
      if (increment) {
          UniqueReceiveP__writeIndex++;
          UniqueReceiveP__writeIndex %= 4;
        }
    }
#line 154
    __nesc_atomic_end(__nesc_atomic); }
}


static inline message_t *UniqueReceiveP__DuplicateReceive__default__receive(message_t *msg, void *payload, uint8_t len)
#line 158
{
  return msg;
}

# 67 "/home/tinyos/local/src/tinyos-2.x/tos/interfaces/Receive.nc"
inline static message_t * UniqueReceiveP__DuplicateReceive__receive(message_t * msg, void * payload, uint8_t len){
#line 67
  nx_struct message_t *result;
#line 67

#line 67
  result = UniqueReceiveP__DuplicateReceive__default__receive(msg, payload, len);
#line 67

#line 67
  return result;
#line 67
}
#line 67
# 111 "/home/tinyos/local/src/tinyos-2.x/tos/chips/cc2420/unique/UniqueReceiveP.nc"
static inline bool UniqueReceiveP__hasSeen(uint16_t msgSource, uint8_t msgDsn)
#line 111
{
  int i;

#line 113
  UniqueReceiveP__recycleSourceElement = UniqueReceiveP__INVALID_ELEMENT;

  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 115
    {
      for (i = 0; i < 4; i++) {
          if (UniqueReceiveP__receivedMessages[i].source == msgSource) {
              if (UniqueReceiveP__receivedMessages[i].dsn == msgDsn) {

                  {
                    unsigned char __nesc_temp = 
#line 120
                    TRUE;

                    {
#line 120
                      __nesc_atomic_end(__nesc_atomic); 
#line 120
                      return __nesc_temp;
                    }
                  }
                }
#line 123
              UniqueReceiveP__recycleSourceElement = i;
            }
        }
    }
#line 126
    __nesc_atomic_end(__nesc_atomic); }

  return FALSE;
}

# 42 "/home/tinyos/local/src/tinyos-2.x/tos/chips/cc2420/interfaces/CC2420PacketBody.nc"
inline static cc2420_header_t * UniqueReceiveP__CC2420PacketBody__getHeader(message_t * msg){
#line 42
  nx_struct cc2420_header_t *result;
#line 42

#line 42
  result = CC2420PacketP__CC2420PacketBody__getHeader(msg);
#line 42

#line 42
  return result;
#line 42
}
#line 42
# 85 "/home/tinyos/local/src/tinyos-2.x/tos/chips/cc2420/unique/UniqueReceiveP.nc"
static inline message_t *UniqueReceiveP__SubReceive__receive(message_t *msg, void *payload, 
uint8_t len)
#line 86
{
  uint16_t msgSource = __nesc_ntoh_leuint16(UniqueReceiveP__CC2420PacketBody__getHeader(msg)->src.data);
  uint8_t msgDsn = __nesc_ntoh_leuint8(UniqueReceiveP__CC2420PacketBody__getHeader(msg)->dsn.data);

  if (UniqueReceiveP__hasSeen(msgSource, msgDsn)) {
      return UniqueReceiveP__DuplicateReceive__receive(msg, payload, len);
    }
  else {
      UniqueReceiveP__insert(msgSource, msgDsn);
      return UniqueReceiveP__Receive__receive(msg, payload, len);
    }
}

# 67 "/home/tinyos/local/src/tinyos-2.x/tos/interfaces/Receive.nc"
inline static message_t * CC2420ReceiveP__Receive__receive(message_t * msg, void * payload, uint8_t len){
#line 67
  nx_struct message_t *result;
#line 67

#line 67
  result = UniqueReceiveP__SubReceive__receive(msg, payload, len);
#line 67

#line 67
  return result;
#line 67
}
#line 67
# 64 "/home/tinyos/local/src/tinyos-2.x/tos/chips/cc2420/interfaces/CC2420Config.nc"
inline static uint16_t CC2420ReceiveP__CC2420Config__getShortAddr(void ){
#line 64
  unsigned int result;
#line 64

#line 64
  result = CC2420ControlP__CC2420Config__getShortAddr();
#line 64

#line 64
  return result;
#line 64
}
#line 64
# 332 "/home/tinyos/local/src/tinyos-2.x/tos/chips/cc2420/control/CC2420ControlP.nc"
static inline bool CC2420ControlP__CC2420Config__isAddressRecognitionEnabled(void )
#line 332
{
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 333
    {
      unsigned char __nesc_temp = 
#line 333
      CC2420ControlP__addressRecognition;

      {
#line 333
        __nesc_atomic_end(__nesc_atomic); 
#line 333
        return __nesc_temp;
      }
    }
#line 335
    __nesc_atomic_end(__nesc_atomic); }
}

# 86 "/home/tinyos/local/src/tinyos-2.x/tos/chips/cc2420/interfaces/CC2420Config.nc"
inline static bool CC2420ReceiveP__CC2420Config__isAddressRecognitionEnabled(void ){
#line 86
  unsigned char result;
#line 86

#line 86
  result = CC2420ControlP__CC2420Config__isAddressRecognitionEnabled();
#line 86

#line 86
  return result;
#line 86
}
#line 86
# 42 "/home/tinyos/local/src/tinyos-2.x/tos/chips/cc2420/interfaces/CC2420PacketBody.nc"
inline static cc2420_header_t * CC2420ReceiveP__CC2420PacketBody__getHeader(message_t * msg){
#line 42
  nx_struct cc2420_header_t *result;
#line 42

#line 42
  result = CC2420PacketP__CC2420PacketBody__getHeader(msg);
#line 42

#line 42
  return result;
#line 42
}
#line 42
# 819 "/home/tinyos/local/src/tinyos-2.x/tos/chips/cc2420/receive/CC2420ReceiveP.nc"
static inline bool CC2420ReceiveP__passesAddressCheck(message_t *msg)
#line 819
{
  cc2420_header_t *header = CC2420ReceiveP__CC2420PacketBody__getHeader(msg);

  if (!CC2420ReceiveP__CC2420Config__isAddressRecognitionEnabled()) {
      return TRUE;
    }

  return __nesc_ntoh_leuint16(header->dest.data) == CC2420ReceiveP__CC2420Config__getShortAddr()
   || __nesc_ntoh_leuint16(header->dest.data) == AM_BROADCAST_ADDR;
}

# 47 "/home/tinyos/local/src/tinyos-2.x/tos/chips/cc2420/interfaces/CC2420PacketBody.nc"
inline static cc2420_metadata_t * CC2420ReceiveP__CC2420PacketBody__getMetadata(message_t * msg){
#line 47
  nx_struct cc2420_metadata_t *result;
#line 47

#line 47
  result = CC2420PacketP__CC2420PacketBody__getMetadata(msg);
#line 47

#line 47
  return result;
#line 47
}
#line 47
# 671 "/home/tinyos/local/src/tinyos-2.x/tos/chips/cc2420/receive/CC2420ReceiveP.nc"
static inline void CC2420ReceiveP__receiveDone_task__runTask(void )
#line 671
{
  cc2420_metadata_t *metadata = CC2420ReceiveP__CC2420PacketBody__getMetadata(CC2420ReceiveP__m_p_rx_buf);
  cc2420_header_t *header = CC2420ReceiveP__CC2420PacketBody__getHeader(CC2420ReceiveP__m_p_rx_buf);
  uint8_t length = __nesc_ntoh_leuint8(header->length.data);
  uint8_t tmpLen __attribute((unused))  = sizeof(message_t ) - ((size_t )& ((message_t *)0)->data - sizeof(cc2420_header_t ));
  uint8_t * buf = (uint8_t * )header;

  __nesc_hton_int8(metadata->crc.data, buf[length] >> 7);
  __nesc_hton_uint8(metadata->lqi.data, buf[length] & 0x7f);
  __nesc_hton_uint8(metadata->rssi.data, buf[length - 1]);

  if (CC2420ReceiveP__passesAddressCheck(CC2420ReceiveP__m_p_rx_buf) && length >= CC2420_SIZE) {
#line 696
      CC2420ReceiveP__m_p_rx_buf = CC2420ReceiveP__Receive__receive(CC2420ReceiveP__m_p_rx_buf, CC2420ReceiveP__m_p_rx_buf->data, 
      length - CC2420_SIZE);
    }
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 699
    CC2420ReceiveP__receivingPacket = FALSE;
#line 699
    __nesc_atomic_end(__nesc_atomic); }
  CC2420ReceiveP__waitForNextPacket();
}

# 44 "/home/tinyos/local/src/tinyos-2.x/tos/lib/net/blip/TcpP.nc"
static inline void TcpP__tcplib_extern_closed(struct TcpP__tcplib_sock *sock)
#line 44
{
  TcpP__tcplib_close(sock);
}

# 44 "/home/tinyos/local/src/tinyos-2.x/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
inline static void /*PlatformLedsC.Led2Impl*/Msp430GpioC__2__HplGeneralIO__toggle(void ){
#line 44
  /*HplMsp430GeneralIOC.P56*/HplMsp430GeneralIOP__38__IO__toggle();
#line 44
}
#line 44
# 39 "/home/tinyos/local/src/tinyos-2.x/tos/chips/msp430/pins/Msp430GpioC.nc"
static inline void /*PlatformLedsC.Led2Impl*/Msp430GpioC__2__GeneralIO__toggle(void )
#line 39
{
#line 39
  /*PlatformLedsC.Led2Impl*/Msp430GpioC__2__HplGeneralIO__toggle();
}

# 31 "/home/tinyos/local/src/tinyos-2.x/tos/interfaces/GeneralIO.nc"
inline static void LedsP__Led2__toggle(void ){
#line 31
  /*PlatformLedsC.Led2Impl*/Msp430GpioC__2__GeneralIO__toggle();
#line 31
}
#line 31
# 103 "/home/tinyos/local/src/tinyos-2.x/tos/system/LedsP.nc"
static inline void LedsP__Leds__led2Toggle(void )
#line 103
{
  LedsP__Led2__toggle();
  ;
#line 105
  ;
}

# 89 "/home/tinyos/local/src/tinyos-2.x/tos/interfaces/Leds.nc"
inline static void TCPEchoP__Leds__led2Toggle(void ){
#line 89
  LedsP__Leds__led2Toggle();
#line 89
}
#line 89
# 32 "/home/tinyos/local/src/tinyos-2.x/tos/lib/net/blip/interfaces/Tcp.nc"
inline static error_t TCPEchoP__TcpEcho__send(void *payload, uint16_t len){
#line 32
  unsigned char result;
#line 32

#line 32
  result = TcpP__Tcp__send(0U, payload, len);
#line 32

#line 32
  return result;
#line 32
}
#line 32
# 151 "TCPEchoP.nc"
static inline void TCPEchoP__TcpEcho__recv(void *payload, uint16_t len)
#line 151
{
  if (TCPEchoP__TcpEcho__send(payload, len) != SUCCESS) {
    TCPEchoP__Leds__led2Toggle();
    }
}

# 39 "/home/tinyos/local/src/tinyos-2.x/tos/lib/net/blip/interfaces/Tcp.nc"
inline static error_t HttpdP__Tcp__close(void ){
#line 39
  unsigned char result;
#line 39

#line 39
  result = TcpP__Tcp__close(1U);
#line 39

#line 39
  return result;
#line 39
}
#line 39
#line 32
inline static error_t HttpdP__Tcp__send(void *payload, uint16_t len){
#line 32
  unsigned char result;
#line 32

#line 32
  result = TcpP__Tcp__send(1U, payload, len);
#line 32

#line 32
  return result;
#line 32
}
#line 32
# 89 "/home/tinyos/local/src/tinyos-2.x/tos/interfaces/Leds.nc"
inline static void HttpdP__Leds__led2Toggle(void ){
#line 89
  LedsP__Leds__led2Toggle();
#line 89
}
#line 89
# 47 "/home/tinyos/local/src/tinyos-2.x/tos/chips/msp430/pins/HplMsp430GeneralIOP.nc"
static inline void /*HplMsp430GeneralIOC.P55*/HplMsp430GeneralIOP__37__IO__toggle(void )
#line 47
{
#line 47
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 47
    * (volatile uint8_t * )49U ^= 0x01 << 5;
#line 47
    __nesc_atomic_end(__nesc_atomic); }
}

# 44 "/home/tinyos/local/src/tinyos-2.x/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
inline static void /*PlatformLedsC.Led1Impl*/Msp430GpioC__1__HplGeneralIO__toggle(void ){
#line 44
  /*HplMsp430GeneralIOC.P55*/HplMsp430GeneralIOP__37__IO__toggle();
#line 44
}
#line 44
# 39 "/home/tinyos/local/src/tinyos-2.x/tos/chips/msp430/pins/Msp430GpioC.nc"
static inline void /*PlatformLedsC.Led1Impl*/Msp430GpioC__1__GeneralIO__toggle(void )
#line 39
{
#line 39
  /*PlatformLedsC.Led1Impl*/Msp430GpioC__1__HplGeneralIO__toggle();
}

# 31 "/home/tinyos/local/src/tinyos-2.x/tos/interfaces/GeneralIO.nc"
inline static void LedsP__Led1__toggle(void ){
#line 31
  /*PlatformLedsC.Led1Impl*/Msp430GpioC__1__GeneralIO__toggle();
#line 31
}
#line 31
# 88 "/home/tinyos/local/src/tinyos-2.x/tos/system/LedsP.nc"
static inline void LedsP__Leds__led1Toggle(void )
#line 88
{
  LedsP__Led1__toggle();
  ;
#line 90
  ;
}

# 72 "/home/tinyos/local/src/tinyos-2.x/tos/interfaces/Leds.nc"
inline static void HttpdP__Leds__led1Toggle(void ){
#line 72
  LedsP__Leds__led1Toggle();
#line 72
}
#line 72
# 44 "/home/tinyos/local/src/tinyos-2.x/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
inline static void /*PlatformLedsC.Led0Impl*/Msp430GpioC__0__HplGeneralIO__toggle(void ){
#line 44
  /*HplMsp430GeneralIOC.P54*/HplMsp430GeneralIOP__36__IO__toggle();
#line 44
}
#line 44
# 39 "/home/tinyos/local/src/tinyos-2.x/tos/chips/msp430/pins/Msp430GpioC.nc"
static inline void /*PlatformLedsC.Led0Impl*/Msp430GpioC__0__GeneralIO__toggle(void )
#line 39
{
#line 39
  /*PlatformLedsC.Led0Impl*/Msp430GpioC__0__HplGeneralIO__toggle();
}

# 31 "/home/tinyos/local/src/tinyos-2.x/tos/interfaces/GeneralIO.nc"
inline static void LedsP__Led0__toggle(void ){
#line 31
  /*PlatformLedsC.Led0Impl*/Msp430GpioC__0__GeneralIO__toggle();
#line 31
}
#line 31
# 73 "/home/tinyos/local/src/tinyos-2.x/tos/system/LedsP.nc"
static inline void LedsP__Leds__led0Toggle(void )
#line 73
{
  LedsP__Led0__toggle();
  ;
#line 75
  ;
}

# 56 "/home/tinyos/local/src/tinyos-2.x/tos/interfaces/Leds.nc"
inline static void HttpdP__Leds__led0Toggle(void ){
#line 56
  LedsP__Leds__led0Toggle();
#line 56
}
#line 56
# 48 "/home/tinyos/local/src/tinyos-2.x/tos/chips/msp430/pins/HplMsp430GeneralIOP.nc"
static inline uint8_t /*HplMsp430GeneralIOC.P56*/HplMsp430GeneralIOP__38__IO__getRaw(void )
#line 48
{
#line 48
  return * (volatile uint8_t * )48U & (0x01 << 6);
}

#line 49
static inline bool /*HplMsp430GeneralIOC.P56*/HplMsp430GeneralIOP__38__IO__get(void )
#line 49
{
#line 49
  return /*HplMsp430GeneralIOC.P56*/HplMsp430GeneralIOP__38__IO__getRaw() != 0;
}

# 59 "/home/tinyos/local/src/tinyos-2.x/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
inline static bool /*PlatformLedsC.Led2Impl*/Msp430GpioC__2__HplGeneralIO__get(void ){
#line 59
  unsigned char result;
#line 59

#line 59
  result = /*HplMsp430GeneralIOC.P56*/HplMsp430GeneralIOP__38__IO__get();
#line 59

#line 59
  return result;
#line 59
}
#line 59
# 40 "/home/tinyos/local/src/tinyos-2.x/tos/chips/msp430/pins/Msp430GpioC.nc"
static inline bool /*PlatformLedsC.Led2Impl*/Msp430GpioC__2__GeneralIO__get(void )
#line 40
{
#line 40
  return /*PlatformLedsC.Led2Impl*/Msp430GpioC__2__HplGeneralIO__get();
}

# 32 "/home/tinyos/local/src/tinyos-2.x/tos/interfaces/GeneralIO.nc"
inline static bool LedsP__Led2__get(void ){
#line 32
  unsigned char result;
#line 32

#line 32
  result = /*PlatformLedsC.Led2Impl*/Msp430GpioC__2__GeneralIO__get();
#line 32

#line 32
  return result;
#line 32
}
#line 32
# 48 "/home/tinyos/local/src/tinyos-2.x/tos/chips/msp430/pins/HplMsp430GeneralIOP.nc"
static inline uint8_t /*HplMsp430GeneralIOC.P55*/HplMsp430GeneralIOP__37__IO__getRaw(void )
#line 48
{
#line 48
  return * (volatile uint8_t * )48U & (0x01 << 5);
}

#line 49
static inline bool /*HplMsp430GeneralIOC.P55*/HplMsp430GeneralIOP__37__IO__get(void )
#line 49
{
#line 49
  return /*HplMsp430GeneralIOC.P55*/HplMsp430GeneralIOP__37__IO__getRaw() != 0;
}

# 59 "/home/tinyos/local/src/tinyos-2.x/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
inline static bool /*PlatformLedsC.Led1Impl*/Msp430GpioC__1__HplGeneralIO__get(void ){
#line 59
  unsigned char result;
#line 59

#line 59
  result = /*HplMsp430GeneralIOC.P55*/HplMsp430GeneralIOP__37__IO__get();
#line 59

#line 59
  return result;
#line 59
}
#line 59
# 40 "/home/tinyos/local/src/tinyos-2.x/tos/chips/msp430/pins/Msp430GpioC.nc"
static inline bool /*PlatformLedsC.Led1Impl*/Msp430GpioC__1__GeneralIO__get(void )
#line 40
{
#line 40
  return /*PlatformLedsC.Led1Impl*/Msp430GpioC__1__HplGeneralIO__get();
}

# 32 "/home/tinyos/local/src/tinyos-2.x/tos/interfaces/GeneralIO.nc"
inline static bool LedsP__Led1__get(void ){
#line 32
  unsigned char result;
#line 32

#line 32
  result = /*PlatformLedsC.Led1Impl*/Msp430GpioC__1__GeneralIO__get();
#line 32

#line 32
  return result;
#line 32
}
#line 32
# 48 "/home/tinyos/local/src/tinyos-2.x/tos/chips/msp430/pins/HplMsp430GeneralIOP.nc"
static inline uint8_t /*HplMsp430GeneralIOC.P54*/HplMsp430GeneralIOP__36__IO__getRaw(void )
#line 48
{
#line 48
  return * (volatile uint8_t * )48U & (0x01 << 4);
}

#line 49
static inline bool /*HplMsp430GeneralIOC.P54*/HplMsp430GeneralIOP__36__IO__get(void )
#line 49
{
#line 49
  return /*HplMsp430GeneralIOC.P54*/HplMsp430GeneralIOP__36__IO__getRaw() != 0;
}

# 59 "/home/tinyos/local/src/tinyos-2.x/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
inline static bool /*PlatformLedsC.Led0Impl*/Msp430GpioC__0__HplGeneralIO__get(void ){
#line 59
  unsigned char result;
#line 59

#line 59
  result = /*HplMsp430GeneralIOC.P54*/HplMsp430GeneralIOP__36__IO__get();
#line 59

#line 59
  return result;
#line 59
}
#line 59
# 40 "/home/tinyos/local/src/tinyos-2.x/tos/chips/msp430/pins/Msp430GpioC.nc"
static inline bool /*PlatformLedsC.Led0Impl*/Msp430GpioC__0__GeneralIO__get(void )
#line 40
{
#line 40
  return /*PlatformLedsC.Led0Impl*/Msp430GpioC__0__HplGeneralIO__get();
}

# 32 "/home/tinyos/local/src/tinyos-2.x/tos/interfaces/GeneralIO.nc"
inline static bool LedsP__Led0__get(void ){
#line 32
  unsigned char result;
#line 32

#line 32
  result = /*PlatformLedsC.Led0Impl*/Msp430GpioC__0__GeneralIO__get();
#line 32

#line 32
  return result;
#line 32
}
#line 32
# 108 "/home/tinyos/local/src/tinyos-2.x/tos/system/LedsP.nc"
static inline uint8_t LedsP__Leds__get(void )
#line 108
{
  uint8_t rval;

#line 110
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 110
    {
      rval = 0;
      if (!LedsP__Led0__get()) {
          rval |= LEDS_LED0;
        }
      if (!LedsP__Led1__get()) {
          rval |= LEDS_LED1;
        }
      if (!LedsP__Led2__get()) {
          rval |= LEDS_LED2;
        }
    }
#line 121
    __nesc_atomic_end(__nesc_atomic); }
  return rval;
}

# 106 "/home/tinyos/local/src/tinyos-2.x/tos/interfaces/Leds.nc"
inline static uint8_t HttpdP__Leds__get(void ){
#line 106
  unsigned char result;
#line 106

#line 106
  result = LedsP__Leds__get();
#line 106

#line 106
  return result;
#line 106
}
#line 106
# 31 "HttpdP.nc"
static inline void HttpdP__process_request(int verb, char *request, int len)
#line 31
{
  char reply[24];

  ;






  if (
#line 36
  len >= 10 && 
  request[0] == '/' && 
  request[1] == 'r' && 
  request[2] == 'e' && 
  request[3] == 'a' && 
  request[4] == 'd' && 
  request[5] == '/') {


      if (
#line 43
      request[6] == 'l' && 
      request[7] == 'e' && 
      request[8] == 'd' && 
      request[9] == 's') {
          uint8_t bitmap = HttpdP__Leds__get();

#line 48
          ip_memcpy(reply, "led0: 0 led1: 0 led2: 0\n", 24);

          HttpdP__Tcp__send(HttpdP__http_okay, HttpdP__http_okay_len);
          if (bitmap & 1) {
#line 51
            reply[6] = '1';
            }
#line 52
          if (bitmap & 2) {
#line 52
            reply[14] = '1';
            }
#line 53
          if (bitmap & 4) {
#line 53
            reply[22] = '1';
            }
#line 54
          HttpdP__Tcp__send(reply, 24);
        }
      else {
        if (
#line 55
        request[6] == 't' && 
        request[7] == 'e' && 
        request[8] == 'm' && 
        request[9] == 'p') {
            HttpdP__Tcp__send(HttpdP__http_okay, HttpdP__http_okay_len);
            ip_memcpy(reply, "temp:                  \n", 24);

            reply[7] = HttpdP__temp % 10000 / 1000 + 48;
            reply[8] = HttpdP__temp % 1000 / 100 + 48;
            reply[9] = HttpdP__temp % 100 / 10 + 48;
            reply[10] = HttpdP__temp % 10 + 48;
            HttpdP__Tcp__send(reply, 24);
          }
        }
    }
  else {
#line 70
    if (
#line 68
    len == 4 && 
    request[0] == '/' && 
    request[1] == 's' && 
    request[2] == '/') {

        HttpdP__Tcp__send(HttpdP__http_okay, HttpdP__http_okay_len);

        if (request[3] == '0') {
#line 75
          HttpdP__Leds__led0Toggle();
          }
#line 76
        if (request[3] == '1') {
#line 76
          HttpdP__Leds__led1Toggle();
          }
#line 77
        if (request[3] == '2') {
#line 77
          HttpdP__Leds__led2Toggle();
          }
        ip_memcpy(reply, "led set.               \n", 24);
        HttpdP__Tcp__send(reply, 24);
      }
    }
#line 82
  HttpdP__Tcp__close();
}

#line 118
static inline void HttpdP__Tcp__recv(void *payload, uint16_t len)
#line 118
{
  static int crlf_pos;
  char *msg = payload;

#line 121
  switch (HttpdP__http_state) {
      case HttpdP__S_CONNECTED: 
        crlf_pos = 0;
      HttpdP__request = HttpdP__request_buf;
      if (len < 3) {
          HttpdP__Tcp__close();
          return;
        }
      if (msg[0] == 'G') {
          HttpdP__req_verb = HttpdP__HTTP_GET;
          msg += 3;
          len -= 3;
        }
      HttpdP__http_state = HttpdP__S_REQUEST_PRE;
      case HttpdP__S_REQUEST_PRE: 
        while (len > 0 && *msg == ' ') {
            len--;
#line 137
            msg++;
          }
      if (len == 0) {
#line 139
        break;
        }
#line 140
      HttpdP__http_state = HttpdP__S_REQUEST;
      case HttpdP__S_REQUEST: 
        while (len > 0 && *msg != ' ') {
            * HttpdP__request++ = * msg++;
            len--;
          }
      if (len == 0) {
#line 146
        break;
        }
#line 147
      * HttpdP__request++ = '\0';
      HttpdP__http_state = HttpdP__S_HEADER;
      case HttpdP__S_HEADER: 
        while (len > 0) {
            switch (crlf_pos) {
                case 0: 
                  case 2: 
                    if (*msg == '\r') {
#line 154
                      crlf_pos++;
                      }
                    else {
#line 155
                      if (*msg == '\n') {
#line 155
                        crlf_pos += 2;
                        }
                      else {
#line 156
                        crlf_pos = 0;
                        }
                      }
#line 157
                break;
                case 1: 
                  case 3: 
                    if (*msg == '\n') {
#line 160
                      crlf_pos++;
                      }
                    else {
#line 161
                      crlf_pos = 0;
                      }
#line 162
                break;
              }
            len--;
#line 164
            msg++;

            if (crlf_pos == 4) {
                HttpdP__http_state = HttpdP__S_BODY;
                HttpdP__process_request(HttpdP__req_verb, HttpdP__request_buf, HttpdP__request - HttpdP__request_buf - 1);
                break;
              }
          }
      if (crlf_pos < 4) {
#line 172
        break;
        }
      case HttpdP__S_BODY: 

        default: 
          HttpdP__Tcp__close();
    }
}

# 150 "/home/tinyos/local/src/tinyos-2.x/tos/lib/net/blip/TcpP.nc"
static inline void TcpP__Tcp__default__recv(uint8_t cid, void *payload, uint16_t len)
#line 150
{
}

# 34 "/home/tinyos/local/src/tinyos-2.x/tos/lib/net/blip/interfaces/Tcp.nc"
inline static void TcpP__Tcp__recv(uint8_t arg_0x413d8b78, void *payload, uint16_t len){
#line 34
  switch (arg_0x413d8b78) {
#line 34
    case 0U:
#line 34
      TCPEchoP__TcpEcho__recv(payload, len);
#line 34
      break;
#line 34
    case 1U:
#line 34
      HttpdP__Tcp__recv(payload, len);
#line 34
      break;
#line 34
    default:
#line 34
      TcpP__Tcp__default__recv(arg_0x413d8b78, payload, len);
#line 34
      break;
#line 34
    }
#line 34
}
#line 34
# 38 "/home/tinyos/local/src/tinyos-2.x/tos/lib/net/blip/TcpP.nc"
static inline void TcpP__tcplib_extern_recv(struct TcpP__tcplib_sock *sock, void *data, int len)
#line 38
{
  int cid = TcpP__find_client(sock);

#line 40
  if (cid < TcpP__N_CLIENTS) {
    TcpP__Tcp__recv(cid, data, len);
    }
}

# 21 "/home/tinyos/local/src/tinyos-2.x/support/sdk/c/blip/lib6lowpan/nwbyte.h"
static __inline uint32_t __attribute((unused)) ntoh32(uint32_t i)
#line 21
{
  uint16_t lo = (uint16_t )i;
  uint16_t hi = (uint16_t )(i >> 16);

#line 24
  lo = (lo << 8) | (lo >> 8);
  hi = (hi << 8) | (hi >> 8);
  return ((uint32_t )lo << 16) | (uint32_t )hi;
}

# 259 "/home/tinyos/local/src/tinyos-2.x/support/sdk/c/blip/libtcp/tcplib.c"
inline static void TcpP__receive_data(struct TcpP__tcplib_sock *sock, struct tcp_hdr *tcph, int len)
#line 259
{
  uint8_t *ptr;
  int payload_len;

  ptr = (uint8_t *)tcph + tcph->offset / 4;
  payload_len = len - tcph->offset / 4;
  sock->ackno = ntoh32(tcph->seqno) + payload_len;

  if (payload_len > 0) {
      TcpP__tcplib_extern_recv(sock, ptr, payload_len);
    }
}

# 48 "/home/tinyos/local/src/tinyos-2.x/support/sdk/c/blip/libtcp/circ.c"
static inline uint32_t TcpP__circ_get_seqno(void *buf)
#line 48
{
  struct TcpP__circ_buf *b = (struct TcpP__circ_buf *)buf;

#line 50
  return b->head_seqno;
}

# 159 "TCPEchoP.nc"
static inline void TCPEchoP__TcpEcho__acked(void )
#line 159
{
}

# 188 "HttpdP.nc"
static inline void HttpdP__Tcp__acked(void )
#line 188
{
}

# 152 "/home/tinyos/local/src/tinyos-2.x/tos/lib/net/blip/TcpP.nc"
static inline void TcpP__Tcp__default__acked(uint8_t cid)
#line 152
{
}

# 52 "/home/tinyos/local/src/tinyos-2.x/tos/lib/net/blip/interfaces/Tcp.nc"
inline static void TcpP__Tcp__acked(uint8_t arg_0x413d8b78){
#line 52
  switch (arg_0x413d8b78) {
#line 52
    case 0U:
#line 52
      TCPEchoP__TcpEcho__acked();
#line 52
      break;
#line 52
    case 1U:
#line 52
      HttpdP__Tcp__acked();
#line 52
      break;
#line 52
    default:
#line 52
      TcpP__Tcp__default__acked(arg_0x413d8b78);
#line 52
      break;
#line 52
    }
#line 52
}
#line 52
# 55 "/home/tinyos/local/src/tinyos-2.x/tos/lib/net/blip/TcpP.nc"
static inline void TcpP__tcplib_extern_acked(struct TcpP__tcplib_sock *sock)
#line 55
{
  int cid = TcpP__find_client(sock);

#line 57
  if (cid < TcpP__N_CLIENTS) {
    TcpP__Tcp__acked(cid);
    }
}

# 74 "/home/tinyos/local/src/tinyos-2.x/support/sdk/c/blip/libtcp/circ.c"
static inline int TcpP__circ_shorten_head(void *buf, uint32_t seqno)
#line 74
{
  struct TcpP__circ_buf *b = (struct TcpP__circ_buf *)buf;
  int offset = seqno - b->head_seqno;

  b->head_seqno = seqno;
  b->data_head += offset;

  while (b->data_head > b->data_start + b->data_len) 
    b->data_head -= b->data_len;

  return 0;
}

#line 36
static inline int TcpP__circ_buf_init(void *data, int len, uint32_t seqno)
#line 36
{
  struct TcpP__circ_buf *b = (struct TcpP__circ_buf *)data;

  if (len < sizeof(struct TcpP__circ_buf )) {
    return -1;
    }
  b->data_head = b->data_start = (uint8_t *)(b + 1);
  b->data_len = len - sizeof(struct TcpP__circ_buf );
  b->head_seqno = seqno;
  return 0;
}

# 47 "/home/tinyos/local/src/tinyos-2.x/support/sdk/c/blip/libtcp/tcplib.c"
static __inline void TcpP__conn_add_once(struct TcpP__tcplib_sock *sock)
#line 47
{
  struct TcpP__tcplib_sock *iter;

  for (iter = TcpP__conns; iter != (void *)0; iter = iter->next) {
      if (iter == sock) {
#line 51
        break;
        }
    }
#line 53
  if (iter == (void *)0) {
      sock->next = TcpP__conns;
      TcpP__conns = sock;
    }
}

# 142 "TCPEchoP.nc"
static inline bool TCPEchoP__TcpEcho__accept(struct sockaddr_in6 *from, 
void **tx_buf, int *tx_buf_len)
#line 143
{
  *tx_buf = TCPEchoP__tcp_buf;
  *tx_buf_len = 150;
  return TRUE;
}

# 104 "HttpdP.nc"
static inline bool HttpdP__Tcp__accept(struct sockaddr_in6 *from, 
void **tx_buf, int *tx_buf_len)
#line 105
{
  if (HttpdP__http_state == HttpdP__S_IDLE) {
      HttpdP__http_state = HttpdP__S_CONNECTED;
      *tx_buf = HttpdP__tcp_buf;
      *tx_buf_len = 100;
      return TRUE;
    }
  ;
  return FALSE;
}

# 144 "/home/tinyos/local/src/tinyos-2.x/tos/lib/net/blip/TcpP.nc"
static inline bool TcpP__Tcp__default__accept(uint8_t cid, struct sockaddr_in6 *from, 
void **tx_buf, int *tx_buf_len)
#line 145
{
  return FALSE;
}

# 16 "/home/tinyos/local/src/tinyos-2.x/tos/lib/net/blip/interfaces/Tcp.nc"
inline static bool TcpP__Tcp__accept(uint8_t arg_0x413d8b78, struct sockaddr_in6 *from, void **tx_buf, int *tx_buf_len){
#line 16
  unsigned char result;
#line 16

#line 16
  switch (arg_0x413d8b78) {
#line 16
    case 0U:
#line 16
      result = TCPEchoP__TcpEcho__accept(from, tx_buf, tx_buf_len);
#line 16
      break;
#line 16
    case 1U:
#line 16
      result = HttpdP__Tcp__accept(from, tx_buf, tx_buf_len);
#line 16
      break;
#line 16
    default:
#line 16
      result = TcpP__Tcp__default__accept(arg_0x413d8b78, from, tx_buf, tx_buf_len);
#line 16
      break;
#line 16
    }
#line 16

#line 16
  return result;
#line 16
}
#line 16
# 65 "/home/tinyos/local/src/tinyos-2.x/tos/lib/net/blip/TcpP.nc"
static inline struct TcpP__tcplib_sock *TcpP__tcplib_accept(struct TcpP__tcplib_sock *conn, 
struct sockaddr_in6 *from)
#line 66
{
  int cid = TcpP__find_client(conn);
  int buf_len;

  ;

  if (cid == TcpP__N_CLIENTS) {
#line 72
    return (void *)0;
    }
#line 73
  if (TcpP__Tcp__accept(cid, from, & conn->tx_buf, &buf_len)) {
      if (conn->tx_buf == (void *)0) {
#line 74
        return (void *)0;
        }
#line 75
      conn->tx_buf_len = buf_len;
      return conn;
    }
  return (void *)0;
}

# 59 "/home/tinyos/local/src/tinyos-2.x/support/sdk/c/blip/libtcp/tcplib.c"
inline static int TcpP__isInaddrAny(struct in6_addr *addr)
#line 59
{
  int i;

#line 61
  for (i = 0; i < 8; i++) 
    if (addr->in6_u.u6_addr16[i] != 0) {
#line 62
      break;
      }
#line 63
  if (i != 8) {
#line 63
    return 0;
    }
#line 64
  return 1;
}

#line 90
inline static struct TcpP__tcplib_sock *TcpP__conn_lookup(struct ip6_hdr *iph, 
struct tcp_hdr *tcph)
#line 91
{
  struct TcpP__tcplib_sock *iter;


  for (iter = TcpP__conns; iter != (void *)0; iter = iter->next) {

      ;


      if ((
#line 98
      memcmp(iph->ip6_dst.in6_u.u6_addr8, iter->l_ep.sin6_addr.in6_u.u6_addr8, 16) == 0 || 
      TcpP__isInaddrAny(& iter->l_ep.sin6_addr)) && 
      tcph->dstport == iter->l_ep.sin6_port && (
      iter->r_ep.sin6_port == 0 || (
      memcmp(& iph->ip6_src, & iter->r_ep.sin6_addr, 16) == 0 && 
      tcph->srcport == iter->r_ep.sin6_port))) {
        return iter;
        }
    }
#line 106
  return (void *)0;
}

#line 279
static inline int TcpP__tcplib_process(struct ip6_hdr *iph, void *payload)
#line 279
{
  int rc = 0;
  struct tcp_hdr *tcph;
  struct TcpP__tcplib_sock *this_conn;

  int len = ((((uint16_t )iph->plen >> 8) | ((uint16_t )iph->plen << 8)) & 0xffff) + sizeof(struct ip6_hdr );
  int payload_len;
  uint32_t hdr_seqno;
#line 286
  uint32_t hdr_ackno;

  tcph = (struct tcp_hdr *)payload;
  payload_len = len - sizeof(struct ip6_hdr ) - tcph->offset / 4;


  this_conn = TcpP__conn_lookup(iph, tcph);

  if (this_conn != (void *)0) {
      hdr_seqno = ntoh32(tcph->seqno);
      hdr_ackno = ntoh32(tcph->ackno);

      if (tcph->flags & TCP_FLAG_RST) {


          ;

          TcpP__tcplib_extern_closedone(this_conn);

          return 0;
        }



      this_conn->r_wind = (((uint16_t )tcph->window >> 8) | ((uint16_t )tcph->window << 8)) & 0xffff;
      ;

      switch (this_conn->state) {
          case TcpP__TCP_LAST_ACK: 
            if (tcph->flags & TCP_FLAG_ACK && 
            hdr_ackno == this_conn->seqno + 1) {

                this_conn->state = TcpP__TCP_CLOSED;
                TcpP__tcplib_extern_closedone(this_conn);
                break;
              }
          case TcpP__TCP_FIN_WAIT_1: 
            ;
          if (tcph->flags & TCP_FLAG_ACK && 
          hdr_ackno == this_conn->seqno + 1) {
              if (tcph->flags & TCP_FLAG_FIN) {
                  this_conn->seqno++;
                  this_conn->state = TcpP__TCP_TIME_WAIT;



                  this_conn->timer.retx = TcpP__TCPLIB_TIMEWAIT_LEN;
                }
              else 
#line 333
                {
                  this_conn->timer.retx = TcpP__TCPLIB_2MSL;
                  this_conn->state = TcpP__TCP_FIN_WAIT_2;
                }
            }

          goto ESTABLISHED;
          case TcpP__TCP_FIN_WAIT_2: 
            if (tcph->flags & TCP_FLAG_FIN) {
                this_conn->seqno++;
                this_conn->state = TcpP__TCP_TIME_WAIT;

                this_conn->timer.retx = TcpP__TCPLIB_TIMEWAIT_LEN;
                TcpP__tcplib_send_ack(this_conn, 0, TCP_FLAG_ACK);
              }
          break;

          case TcpP__TCP_SYN_SENT: 
            if (tcph->flags & (TCP_FLAG_SYN | TCP_FLAG_ACK)) {


                this_conn->state = TcpP__TCP_ESTABLISHED;
                this_conn->ackno = hdr_seqno + 1;


                goto ESTABLISHED;
              }
            else {
#line 359
              if (this_conn->flags & TCP_FLAG_SYN) {

                  this_conn->state = TcpP__TCP_SYN_RCVD;
                }
              else 
#line 362
                {
                  ;

                  TcpP__tcplib_send_rst(iph, tcph);
                  break;
                }
              }
#line 368
          case TcpP__TCP_SYN_RCVD: 
            case TcpP__TCP_LISTEN: 

              if (tcph->flags & TCP_FLAG_SYN) {
                  struct TcpP__tcplib_sock *new_sock;

                  if (this_conn->state == TcpP__TCP_LISTEN) {
                      ip_memcpy(& this_conn->r_ep.sin6_addr, & iph->ip6_src, 16);
                      this_conn->r_ep.sin6_port = tcph->srcport;
                      new_sock = TcpP__tcplib_accept(this_conn, & this_conn->r_ep);
                      if (new_sock != this_conn) {
                          memset(this_conn->r_ep.sin6_addr.in6_u.u6_addr8, 0, 16);
                          this_conn->r_ep.sin6_port = 0;
                          if (new_sock != (void *)0) {
                              ip_memcpy(& new_sock->r_ep.sin6_addr, & iph->ip6_src, 16);
                              new_sock->r_ep.sin6_port = tcph->srcport;
                              TcpP__conn_add_once(new_sock);
                            }
                        }
                      if (new_sock == (void *)0) {
                          TcpP__tcplib_send_rst(iph, tcph);
                          break;
                        }
                      ip_memcpy(& new_sock->l_ep.sin6_addr, & iph->ip6_dst, 16);
                      new_sock->l_ep.sin6_port = tcph->dstport;

                      new_sock->ackno = hdr_seqno + 1;
                      TcpP__circ_buf_init(new_sock->tx_buf, new_sock->tx_buf_len, 
                      0xcafebabe + 1);
                    }
                  else 
#line 397
                    {

                      new_sock = this_conn;
                    }

                  if (new_sock != (void *)0) {
                      new_sock->seqno = 0xcafebabe + 1;
                      new_sock->state = TcpP__TCP_SYN_RCVD;
                      TcpP__tcplib_send_ack(new_sock, 0, TCP_FLAG_ACK | TCP_FLAG_SYN);
                      new_sock->seqno++;
                    }
                  else 
#line 407
                    {
                      memset(& this_conn->r_ep, 0, sizeof(struct sockaddr_in6 ));
                    }
                }
              else {
#line 410
                if (this_conn->state == TcpP__TCP_LISTEN) {
                    TcpP__tcplib_send_rst(iph, tcph);
                    break;
                  }
                }
          if (tcph->flags & TCP_FLAG_ACK) {
              this_conn->state = TcpP__TCP_ESTABLISHED;
            }



          case TcpP__TCP_CLOSE_WAIT: 
            case TcpP__TCP_ESTABLISHED: 
              ESTABLISHED: 


                if (this_conn->state == TcpP__TCP_ESTABLISHED || this_conn->state == TcpP__TCP_FIN_WAIT_1) {
                    if (payload_len > 0) {
                        if ((this_conn->flags & TcpP__TCP_ACKPENDING) == TcpP__TCP_ACKPENDING) {
                          }

                        this_conn->flags++;
                      }



                    ;



                    if (hdr_ackno > TcpP__circ_get_seqno(this_conn->tx_buf)) {


                        if (this_conn->cwnd <= this_conn->ssthresh) {

                            this_conn->cwnd += this_conn->mss;
                          }
                        else {

                            this_conn->cwnd += this_conn->mss * this_conn->mss / this_conn->cwnd;
                          }



                        this_conn->flags &= ~TcpP__TCP_DUPACKS;

                        TcpP__circ_shorten_head(this_conn->tx_buf, hdr_ackno);


                        if (this_conn->seqno == hdr_ackno) {
                            TcpP__tcplib_extern_acked(this_conn);
                          }
                      }
                    else {
#line 462
                      if (this_conn->seqno > TcpP__circ_get_seqno(this_conn->tx_buf)) {





                          this_conn->flags += 1 << TcpP__TCP_DUPACKS_OFF;



                          if ((this_conn->flags & TcpP__TCP_DUPACKS) >> TcpP__TCP_DUPACKS_OFF == 2) {
                              this_conn->flags &= ~TcpP__TCP_DUPACKS;
                              ;





                              TcpP__reset_ssthresh(this_conn);
                              TcpP__tcplib_output(this_conn, TcpP__circ_get_seqno(this_conn->tx_buf));
                              this_conn->timer.retx = 6;
                            }
                        }
                      }

                    if (hdr_seqno != this_conn->ackno) {
                        ;
                        if (hdr_seqno > this_conn->ackno + this_conn->my_wind || 
                        hdr_seqno < this_conn->ackno - this_conn->my_wind) {

                            TcpP__tcplib_send_rst(iph, tcph);
                          }
                        else 
#line 493
                          {
                            TcpP__tcplib_send_ack(this_conn, 0, TCP_FLAG_ACK);
                            this_conn->flags |= TcpP__TCP_ACKSENT;
                          }
                      }
                    else 
#line 497
                      {
                        ;
                        TcpP__receive_data(this_conn, tcph, len - sizeof(struct ip6_hdr ));

                        if (this_conn->flags & TcpP__TCP_ACKSENT) {
                            this_conn->flags &= ~TcpP__TCP_ACKSENT;
                            TcpP__tcplib_send_ack(this_conn, 0, TCP_FLAG_ACK);
                          }
                      }


                    if (this_conn->timer.retx == 0) {
                      this_conn->timer.retx = 6;
                      }
                  }
#line 511
          case TcpP__TCP_TIME_WAIT: 

            if ((
#line 512
            payload_len > 0 && (this_conn->flags & TcpP__TCP_ACKPENDING) >= 1)
             || tcph->flags & TCP_FLAG_FIN) {
                TcpP__tcplib_send_ack(this_conn, payload_len == 0 && tcph->flags & TCP_FLAG_FIN, TCP_FLAG_ACK);



                if (
#line 516
                this_conn->state == TcpP__TCP_ESTABLISHED
                 && tcph->flags & TCP_FLAG_FIN
                 && hdr_seqno == this_conn->ackno) {
                    this_conn->state = TcpP__TCP_CLOSE_WAIT;
                    TcpP__tcplib_extern_closed(this_conn);
                  }
              }
          break;
          case TcpP__TCP_CLOSED: 
            default: 
              rc = -1;
        }
    }
  else 
    {


      ;
      TcpP__tcplib_send_rst(iph, tcph);
    }

  return rc;
}

# 104 "/home/tinyos/local/src/tinyos-2.x/tos/lib/net/blip/TcpP.nc"
static inline void TcpP__IP__recv(struct ip6_hdr *iph, 
void *payload, 
struct ip_metadata *meta)
#line 106
{

  ;
  TcpP__tcplib_process(iph, payload);
}

# 9 "/home/tinyos/local/src/tinyos-2.x/tos/lib/net/blip/interfaces/Tcp.nc"
inline static error_t TCPEchoP__TcpEcho__bind(uint16_t port){
#line 9
  unsigned char result;
#line 9

#line 9
  result = TcpP__Tcp__bind(0U, port);
#line 9

#line 9
  return result;
#line 9
}
#line 9
# 56 "/home/tinyos/local/src/tinyos-2.x/tos/interfaces/Leds.nc"
inline static void TCPEchoP__Leds__led0Toggle(void ){
#line 56
  LedsP__Leds__led0Toggle();
#line 56
}
#line 56
# 155 "TCPEchoP.nc"
static inline void TCPEchoP__TcpEcho__closed(error_t e)
#line 155
{
  TCPEchoP__Leds__led0Toggle();
  TCPEchoP__TcpEcho__bind(7);
}

# 9 "/home/tinyos/local/src/tinyos-2.x/tos/lib/net/blip/interfaces/Tcp.nc"
inline static error_t HttpdP__Tcp__bind(uint16_t port){
#line 9
  unsigned char result;
#line 9

#line 9
  result = TcpP__Tcp__bind(1U, port);
#line 9

#line 9
  return result;
#line 9
}
#line 9
# 181 "HttpdP.nc"
static inline void HttpdP__Tcp__closed(error_t e)
#line 181
{
  HttpdP__Leds__led2Toggle();

  HttpdP__Tcp__bind(80);
  HttpdP__http_state = HttpdP__S_IDLE;
}

# 151 "/home/tinyos/local/src/tinyos-2.x/tos/lib/net/blip/TcpP.nc"
static inline void TcpP__Tcp__default__closed(uint8_t cid, error_t e)
#line 151
{
}

# 47 "/home/tinyos/local/src/tinyos-2.x/tos/lib/net/blip/interfaces/Tcp.nc"
inline static void TcpP__Tcp__closed(uint8_t arg_0x413d8b78, error_t e){
#line 47
  switch (arg_0x413d8b78) {
#line 47
    case 0U:
#line 47
      TCPEchoP__TcpEcho__closed(e);
#line 47
      break;
#line 47
    case 1U:
#line 47
      HttpdP__Tcp__closed(e);
#line 47
      break;
#line 47
    default:
#line 47
      TcpP__Tcp__default__closed(arg_0x413d8b78, e);
#line 47
      break;
#line 47
    }
#line 47
}
#line 47
# 109 "/home/tinyos/local/src/tinyos-2.x/support/sdk/c/blip/libtcp/tcplib.c"
inline static int TcpP__conn_checkport(uint16_t port)
#line 109
{
  struct TcpP__tcplib_sock *iter;

  for (iter = TcpP__conns; iter != (void *)0; iter = iter->next) {
      if (iter->l_ep.sin6_port == port) {
        return -1;
        }
    }
#line 116
  return 0;
}

#line 542
static inline int TcpP__tcplib_bind(struct TcpP__tcplib_sock *sock, 
struct sockaddr_in6 *addr)
#line 543
{


  if (TcpP__conn_checkport(addr->sin6_port)) {
    return -1;
    }
  ip_memcpy(& sock->l_ep, addr, sizeof(struct sockaddr_in6 ));

  sock->state = TcpP__TCP_LISTEN;
  return 0;
}

#line 119
static inline struct tcp_hdr *TcpP__find_tcp_hdr(struct split_ip_msg *msg)
#line 119
{
  if (msg->hdr.nxt_hdr == IANA_TCP) {
      return (struct tcp_hdr *)(msg->headers == (void *)0 ? msg->data : 
      msg->headers->hdr.data);
    }
  return (void *)0;
}

# 32 "/home/tinyos/local/src/tinyos-2.x/tos/lib/net/blip/interfaces/IPAddress.nc"
inline static void TcpP__IPAddress__setSource(struct ip6_hdr *hdr){
#line 32
  IPAddressP__IPAddress__setSource(hdr);
#line 32
}
#line 32
# 75 "/home/tinyos/local/src/tinyos-2.x/tos/lib/net/blip/IPAddressP.nc"
static inline bool IPAddressP__IPAddress__haveAddress(void )
#line 75
{
  return globalPrefix;
}

# 15 "/home/tinyos/local/src/tinyos-2.x/tos/lib/net/blip/interfaces/IP.nc"
inline static error_t TcpP__IP__send(struct split_ip_msg *msg){
#line 15
  unsigned char result;
#line 15

#line 15
  result = IPDispatchP__IP__send(IANA_TCP, msg);
#line 15

#line 15
  return result;
#line 15
}
#line 15
# 100 "/home/tinyos/local/src/tinyos-2.x/tos/lib/net/blip/IPExtensionP.nc"
static inline void IPExtensionP__InternalIPExtension__ip_free(void )
#line 100
{
  if (IPExtensionP__ext_dest != (void *)0) {
#line 101
    ip_free(IPExtensionP__ext_dest);
    }
#line 102
  if (IPExtensionP__ext_hop != (void *)0) {
#line 102
    ip_free(IPExtensionP__ext_hop);
    }
#line 103
  IPExtensionP__ext_dest = IPExtensionP__ext_hop = (void *)0;
}

# 6 "/home/tinyos/local/src/tinyos-2.x/tos/lib/net/blip/interfaces/InternalIPExtension.nc"
inline static void IPDispatchP__InternalIPExtension__ip_free(void ){
#line 6
  IPExtensionP__InternalIPExtension__ip_free();
#line 6
}
#line 6
# 150 "/home/tinyos/local/src/tinyos-2.x/tos/chips/cc2420/CC2420Ieee154MessageP.nc"
static inline void CC2420Ieee154MessageP__Packet__setPayloadLength(message_t *msg, uint8_t len)
#line 150
{
  __nesc_hton_leuint8(CC2420Ieee154MessageP__CC2420PacketBody__getHeader(msg)->length.data, len + CC2420_SIZE - AM_OVERHEAD);
}

# 83 "/home/tinyos/local/src/tinyos-2.x/tos/interfaces/Packet.nc"
inline static void IPDispatchP__Packet__setPayloadLength(message_t * msg, uint8_t len){
#line 83
  CC2420Ieee154MessageP__Packet__setPayloadLength(msg, len);
#line 83
}
#line 83
# 154 "/home/tinyos/local/src/tinyos-2.x/tos/chips/cc2420/CC2420Ieee154MessageP.nc"
static inline uint8_t CC2420Ieee154MessageP__Packet__maxPayloadLength(void )
#line 154
{
  return 102 + AM_OVERHEAD;
}

# 95 "/home/tinyos/local/src/tinyos-2.x/tos/interfaces/Packet.nc"
inline static uint8_t IPDispatchP__Packet__maxPayloadLength(void ){
#line 95
  unsigned char result;
#line 95

#line 95
  result = CC2420Ieee154MessageP__Packet__maxPayloadLength();
#line 95

#line 95
  return result;
#line 95
}
#line 95
# 107 "/home/tinyos/local/src/tinyos-2.x/tos/chips/cc2420/lowpan/CC2420TinyosNetworkP.nc"
static inline void *CC2420TinyosNetworkP__BareSend__getPayload(message_t *msg, uint8_t len)
#line 107
{

  cc2420_header_t *hdr = CC2420TinyosNetworkP__CC2420PacketBody__getHeader(msg);

#line 110
  return & hdr->network;
}

# 114 "/home/tinyos/local/src/tinyos-2.x/tos/interfaces/Send.nc"
inline static void * CC2420Ieee154MessageP__SubSend__getPayload(message_t * msg, uint8_t len){
#line 114
  void *result;
#line 114

#line 114
  result = CC2420TinyosNetworkP__BareSend__getPayload(msg, len);
#line 114

#line 114
  return result;
#line 114
}
#line 114
# 158 "/home/tinyos/local/src/tinyos-2.x/tos/chips/cc2420/CC2420Ieee154MessageP.nc"
static inline void *CC2420Ieee154MessageP__Packet__getPayload(message_t *msg, uint8_t len)
#line 158
{
  return CC2420Ieee154MessageP__SubSend__getPayload(msg, len);
}

# 115 "/home/tinyos/local/src/tinyos-2.x/tos/interfaces/Packet.nc"
inline static void * IPDispatchP__Packet__getPayload(message_t * msg, uint8_t len){
#line 115
  void *result;
#line 115

#line 115
  result = CC2420Ieee154MessageP__Packet__getPayload(msg, len);
#line 115

#line 115
  return result;
#line 115
}
#line 115
# 1221 "/home/tinyos/local/src/tinyos-2.x/tos/lib/net/blip/IPRoutingP.nc"
static inline struct ip6_route *IPRoutingP__IPRouting__insertRoutingHeader(struct split_ip_msg *msg)
#line 1221
{
#line 1240
  return (void *)0;
}

# 81 "/home/tinyos/local/src/tinyos-2.x/tos/lib/net/blip/interfaces/IPRouting.nc"
inline static struct ip6_route *IPDispatchP__IPRouting__insertRoutingHeader(struct split_ip_msg *msg){
#line 81
  struct ip6_route *result;
#line 81

#line 81
  result = IPRoutingP__IPRouting__insertRoutingHeader(msg);
#line 81

#line 81
  return result;
#line 81
}
#line 81
# 87 "/home/tinyos/local/src/tinyos-2.x/tos/lib/net/blip/IPExtensionP.nc"
static inline void IPExtensionP__InternalIPExtension__addHeaders(struct split_ip_msg *msg, 
uint8_t nxt_hdr, 
uint16_t label)
#line 89
{

  IPExtensionP__ext_dest = IPExtensionP__ext_hop = (void *)0;
  msg->hdr.nxt_hdr = nxt_hdr;
  IPExtensionP__ext_dest = IPExtensionP__buildTLVHdr(msg, 0, 1, nxt_hdr);
  if (IPExtensionP__ext_dest != (void *)0) {
#line 94
    msg->hdr.nxt_hdr = IPV6_DEST;
    }
  IPExtensionP__ext_hop = IPExtensionP__buildTLVHdr(msg, 1, 1, msg->hdr.nxt_hdr);
  if (IPExtensionP__ext_hop != (void *)0) {
#line 97
    msg->hdr.nxt_hdr = IPV6_HOP;
    }
}

# 4 "/home/tinyos/local/src/tinyos-2.x/tos/lib/net/blip/interfaces/InternalIPExtension.nc"
inline static void IPDispatchP__InternalIPExtension__addHeaders(struct split_ip_msg *msg, uint8_t nxt_hdr, uint16_t label){
#line 4
  IPExtensionP__InternalIPExtension__addHeaders(msg, nxt_hdr, label);
#line 4
}
#line 4
# 58 "/home/tinyos/local/src/tinyos-2.x/tos/lib/net/blip/interfaces/IPRouting.nc"
inline static uint8_t IPDispatchP__IPRouting__getHopLimit(void ){
#line 58
  unsigned char result;
#line 58

#line 58
  result = IPRoutingP__IPRouting__getHopLimit();
#line 58

#line 58
  return result;
#line 58
}
#line 58
# 852 "/home/tinyos/local/src/tinyos-2.x/tos/lib/net/blip/IPDispatchP.nc"
static inline error_t IPDispatchP__IP__bareSend(uint8_t prot, struct split_ip_msg *msg, 
struct ip6_route *route, 
int flags)
#line 854
{
  unsigned int __nesc_temp67;
  unsigned char *__nesc_temp66;
  unsigned char __nesc_temp65;
  unsigned char *__nesc_temp64;
#line 855
  uint16_t payload_length;

  if (IPDispatchP__state != IPDispatchP__S_RUNNING) {
      return EOFF;
    }

  if (msg->hdr.hlim != 0xff) {
    msg->hdr.hlim = IPDispatchP__IPRouting__getHopLimit();
    }
  ;

  ip_memclr(msg->hdr.vlfc, 4);
  msg->hdr.vlfc[0] = 0x6 << 4;

  IPDispatchP__current_local_label++;
  if (!(flags & IP_NOHEADERS)) {
      IPDispatchP__InternalIPExtension__addHeaders(msg, prot, IPDispatchP__current_local_label);

      if (route == (void *)0) {
        route = IPDispatchP__IPRouting__insertRoutingHeader(msg);
        }
    }
  payload_length = msg->data_len;
  {
    struct generic_header *cur = msg->headers;

#line 880
    while (cur != (void *)0) {
        payload_length += cur->len;
        cur = cur->next;
      }
  }

  msg->hdr.plen = (((uint16_t )payload_length << 8) | ((uint16_t )payload_length >> 8)) & 0xffff;
  ;





  {
    error_t rc = SUCCESS;
    send_info_t *s_info;
    send_entry_t *s_entry;
    uint8_t frag_len = 1;
    message_t *outgoing;
    fragment_t progress;
    struct source_header *sh;

#line 901
    progress.offset = 0;

    s_info = IPDispatchP__getSendInfo();
    if (s_info == (void *)0) {
        rc = ERETRY;
        goto cleanup_outer;
      }
    s_info->local_flow_label = IPDispatchP__current_local_label;


    sh = msg->headers != (void *)0 ? (struct source_header *)msg->headers->hdr.ext : (void *)0;

    if (
#line 912
    IPDispatchP__IPRouting__getNextHop(& msg->hdr, route, 0x0, 
    & s_info->policy) != SUCCESS) {
        ;
        goto done;
      }


    while (frag_len > 0) {
        s_entry = IPDispatchP__SendEntryPool__get();
        outgoing = IPDispatchP__FragPool__get();

        if (s_entry == (void *)0 || outgoing == (void *)0) {
            if (s_entry != (void *)0) {
              IPDispatchP__SendEntryPool__put(s_entry);
              }
#line 926
            if (outgoing != (void *)0) {
              IPDispatchP__FragPool__put(outgoing);
              }

            s_info->failed = TRUE;
            ;
            goto done;
          }




        frag_len = getNextFrag(msg, &progress, 
        IPDispatchP__Packet__getPayload(outgoing, IPDispatchP__Packet__maxPayloadLength()), 
        IPDispatchP__Packet__maxPayloadLength());

        if (frag_len == 0) {
            IPDispatchP__FragPool__put(outgoing);
            IPDispatchP__SendEntryPool__put(s_entry);
            goto done;
          }
        IPDispatchP__Packet__setPayloadLength(outgoing, frag_len);

        s_entry->msg = outgoing;
        s_entry->info = s_info;

        if (IPDispatchP__SendQueue__enqueue(s_entry) != SUCCESS) {
            (__nesc_temp64 = IPDispatchP__stats.encfail.data, __nesc_hton_uint8(__nesc_temp64, (__nesc_temp65 = __nesc_ntoh_uint8(__nesc_temp64)) + 1), __nesc_temp65);
            ;
            goto done;
          }

        s_info->refcount++;
      }


    done: 
      (__nesc_temp66 = IPDispatchP__stats.sent.data, __nesc_hton_uint16(__nesc_temp66, (__nesc_temp67 = __nesc_ntoh_uint16(__nesc_temp66)) + 1), __nesc_temp67);
    if (-- s_info->refcount == 0) {
#line 964
      IPDispatchP__SendInfoPool__put(s_info);
      }
#line 965
    IPDispatchP__sendTask__postTask();
    cleanup_outer: 
      IPDispatchP__InternalIPExtension__ip_free();

    return rc;
  }
}

# 1156 "/home/tinyos/local/src/tinyos-2.x/tos/lib/net/blip/IPRoutingP.nc"
static inline struct tlv_hdr *IPRoutingP__DestinationExt__getHeader(int label, int nxt_hdr, 
struct ip6_hdr *iph)
#line 1157
{
  static uint8_t sh_buf
  [
#line 1158
  sizeof(struct tlv_hdr ) + 
  sizeof(struct topology_header ) + 
  sizeof(struct topology_entry ) * N_NEIGH];
  struct tlv_hdr *tlv = (struct tlv_hdr *)sh_buf;
  struct topology_header *th = (struct topology_header *)(tlv + 1);

  tlv->len = sizeof(struct tlv_hdr ) + sizeof(struct topology_header );
  tlv->type = TLV_TYPE_TOPOLOGY;

  if (iph->ip6_dst.in6_u.u6_addr8[0] == 0xff && (
  iph->ip6_dst.in6_u.u6_addr8[1] & 0xf) <= 3) {
      return (void *)0;
    }

  ;









  if (iph->nxt_hdr == IANA_UDP || 
  iph->nxt_hdr == IPV6_NONEXT) {
      int i;
#line 1184
      int j = 0;

#line 1185
      if (iph->ip6_dst.in6_u.u6_addr16[0] == ((((uint16_t )0xff02 << 8) | ((uint16_t )0xff02 >> 8)) & 0xffff)) {
#line 1185
        return (void *)0;
        }
#line 1186
      if (IPRoutingP__traffic_sent) {
#line 1186
        return (void *)0;
        }
      IPRoutingP__traffic_sent = TRUE;





      th->seqno = IPRoutingP__reportSeqno++;
      th->seqno = (((uint16_t )th->seqno << 8) | ((uint16_t )th->seqno >> 8)) & 0xffff;


      for (i = 0; i < N_NEIGH; i++) {
          if (((&IPRoutingP__neigh_table[i])->flags & T_VALID_MASK) == T_VALID_MASK && j < 4 && ((
          (&IPRoutingP__neigh_table[i])->flags & T_MATURE_MASK) == T_MATURE_MASK || IPRoutingP__default_route == &IPRoutingP__neigh_table[i])) {
              th->topo[j].etx = IPRoutingP__getLinkCost(&IPRoutingP__neigh_table[i]) > 0xff ? 0xff : IPRoutingP__getLinkCost(&IPRoutingP__neigh_table[i]);
              th->topo[j].conf = IPRoutingP__getConfidence(&IPRoutingP__neigh_table[i]) > 0xff ? 0xff : IPRoutingP__getConfidence(&IPRoutingP__neigh_table[i]);
              th->topo[j].hwaddr = (((uint16_t )IPRoutingP__neigh_table[i].neighbor << 8) | ((uint16_t )IPRoutingP__neigh_table[i].neighbor >> 8)) & 0xffff;
              j++;
              tlv->len += sizeof(struct topology_entry );
              ;
            }
        }

      if (j > 0) {
          return tlv;
        }
    }
  return (void *)0;
}

# 132 "/home/tinyos/local/src/tinyos-2.x/tos/lib/net/blip/IPExtensionP.nc"
static inline struct tlv_hdr *IPExtensionP__DestinationExt__default__getHeader(uint8_t i, int label, int nxt_hdr, 
struct ip6_hdr *msg)
#line 133
{
  ;
  return (void *)0;
}

# 5 "/home/tinyos/local/src/tinyos-2.x/tos/lib/net/blip/interfaces/TLVHeader.nc"
inline static struct tlv_hdr *IPExtensionP__DestinationExt__getHeader(uint8_t arg_0x413a5f20, int label, int nxt_hdr, struct ip6_hdr *msg){
#line 5
  struct tlv_hdr *result;
#line 5

#line 5
  switch (arg_0x413a5f20) {
#line 5
    case 0:
#line 5
      result = IPRoutingP__DestinationExt__getHeader(label, nxt_hdr, msg);
#line 5
      break;
#line 5
    default:
#line 5
      result = IPExtensionP__DestinationExt__default__getHeader(arg_0x413a5f20, label, nxt_hdr, msg);
#line 5
      break;
#line 5
    }
#line 5

#line 5
  return result;
#line 5
}
#line 5
# 138 "/home/tinyos/local/src/tinyos-2.x/tos/lib/net/blip/IPExtensionP.nc"
static inline struct tlv_hdr *IPExtensionP__HopByHopExt__default__getHeader(uint8_t i, int label, int nxt_hdr, 
struct ip6_hdr *msg)
#line 139
{
  return (void *)0;
}

# 5 "/home/tinyos/local/src/tinyos-2.x/tos/lib/net/blip/interfaces/TLVHeader.nc"
inline static struct tlv_hdr *IPExtensionP__HopByHopExt__getHeader(uint8_t arg_0x413a5710, int label, int nxt_hdr, struct ip6_hdr *msg){
#line 5
  struct tlv_hdr *result;
#line 5

#line 5
    result = IPExtensionP__HopByHopExt__default__getHeader(arg_0x413a5710, label, nxt_hdr, msg);
#line 5

#line 5
  return result;
#line 5
}
#line 5
# 88 "/home/tinyos/local/src/tinyos-2.x/tos/system/PoolP.nc"
static inline /*IPDispatchC.SendInfoPool.PoolP*/PoolP__2__pool_t */*IPDispatchC.SendInfoPool.PoolP*/PoolP__2__Pool__get(void )
#line 88
{
  if (/*IPDispatchC.SendInfoPool.PoolP*/PoolP__2__free) {
      /*IPDispatchC.SendInfoPool.PoolP*/PoolP__2__pool_t *rval = /*IPDispatchC.SendInfoPool.PoolP*/PoolP__2__queue[/*IPDispatchC.SendInfoPool.PoolP*/PoolP__2__index];

#line 91
      /*IPDispatchC.SendInfoPool.PoolP*/PoolP__2__queue[/*IPDispatchC.SendInfoPool.PoolP*/PoolP__2__index] = (void *)0;
      /*IPDispatchC.SendInfoPool.PoolP*/PoolP__2__free--;
      /*IPDispatchC.SendInfoPool.PoolP*/PoolP__2__index++;
      if (/*IPDispatchC.SendInfoPool.PoolP*/PoolP__2__index == 14) {
          /*IPDispatchC.SendInfoPool.PoolP*/PoolP__2__index = 0;
        }
      ;
      return rval;
    }
  return (void *)0;
}

# 97 "/home/tinyos/local/src/tinyos-2.x/tos/interfaces/Pool.nc"
inline static IPDispatchP__SendInfoPool__t * IPDispatchP__SendInfoPool__get(void ){
#line 97
  struct __nesc_unnamed4305 *result;
#line 97

#line 97
  result = /*IPDispatchC.SendInfoPool.PoolP*/PoolP__2__Pool__get();
#line 97

#line 97
  return result;
#line 97
}
#line 97
# 57 "/home/tinyos/local/src/tinyos-2.x/tos/system/QueueC.nc"
static inline uint8_t /*IPDispatchC.QueueC*/QueueC__0__Queue__size(void )
#line 57
{
  return /*IPDispatchC.QueueC*/QueueC__0__size;
}

static inline uint8_t /*IPDispatchC.QueueC*/QueueC__0__Queue__maxSize(void )
#line 61
{
  return 14;
}

# 87 "/home/tinyos/local/src/tinyos-2.x/support/sdk/c/blip/libtcp/circ.c"
static inline int TcpP__circ_buf_read(void *buf, uint32_t sseqno, 
uint8_t *data, int len)
#line 88
{
  struct TcpP__circ_buf *b = (struct TcpP__circ_buf *)buf;
  uint8_t *readptr;
  int r_len;
#line 91
  int rc = 0;

  TcpP__get_ptr_off_1(b, sseqno, len, &readptr, &r_len);
  ip_memcpy(data, readptr, r_len);
  data += r_len;
  rc += r_len;

  if (r_len != len) {
      readptr = b->data_start;
      r_len = len - r_len > b->data_head - b->data_start ? b->data_head - b->data_start : len - r_len;
      ip_memcpy(data, readptr, r_len);
      rc += r_len;
    }
  return rc;
}

static inline int TcpP__circ_buf_write(char *buf, uint32_t sseqno, 
uint8_t *data, int len)
#line 108
{
  struct TcpP__circ_buf *b = (struct TcpP__circ_buf *)buf;
  uint8_t *writeptr;
  int w_len;


  if (sseqno > b->head_seqno + b->data_len) {
    return -1;
    }
#line 116
  if (len == 0) {
#line 116
    return 0;
    }
  TcpP__get_ptr_off_1(b, sseqno, len, &writeptr, &w_len);

  ip_memcpy(writeptr, data, w_len);
  data += w_len;

  if (w_len != len) {
      writeptr = b->data_start;
      w_len = len - w_len > b->data_head - b->data_start ? b->data_head - b->data_start : len - w_len;
      ip_memcpy(writeptr, data, w_len);
    }

  return 0;
}

# 588 "/home/tinyos/local/src/tinyos-2.x/support/sdk/c/blip/libtcp/tcplib.c"
static inline int TcpP__tcplib_send(struct TcpP__tcplib_sock *sock, void *data, int len)
#line 588
{

  if (sock->state != TcpP__TCP_ESTABLISHED) {
    return -1;
    }
#line 592
  if (sock->seqno - TcpP__circ_get_seqno(sock->tx_buf) + len > sock->tx_buf_len) {
    return -1;
    }
#line 594
  if (TcpP__circ_buf_write(sock->tx_buf, sock->seqno, data, len) < 0) {
    return -1;
    }
  sock->seqno += len;

  TcpP__tcplib_output(sock, sock->seqno - len);


  if (sock->timer.retx == 0) {
    sock->timer.retx = 6;
    }
  return 0;
}

# 16 "/home/tinyos/local/src/tinyos-2.x/tos/lib/net/blip/interfaces/UDP.nc"
inline static error_t TCPEchoP__Echo__sendto(struct sockaddr_in6 *dest, void *payload, uint16_t len){
#line 16
  unsigned char result;
#line 16

#line 16
  result = UdpP__UDP__sendto(0U, dest, payload, len);
#line 16

#line 16
  return result;
#line 16
}
#line 16
# 104 "TCPEchoP.nc"
static inline void TCPEchoP__Echo__recvfrom(struct sockaddr_in6 *from, void *data, 
uint16_t len, struct ip_metadata *meta)
#line 105
{
  ;
  TCPEchoP__Echo__sendto(from, data, len);
}

#line 99
static inline void TCPEchoP__Status__recvfrom(struct sockaddr_in6 *from, void *data, 
uint16_t len, struct ip_metadata *meta)
#line 100
{
}

# 16 "/home/tinyos/local/src/tinyos-2.x/tos/lib/net/blip/interfaces/UDP.nc"
inline static error_t UDPShellP__UDP__sendto(struct sockaddr_in6 *dest, void *payload, uint16_t len){
#line 16
  unsigned char result;
#line 16

#line 16
  result = UdpP__UDP__sendto(2U, dest, payload, len);
#line 16

#line 16
  return result;
#line 16
}
#line 16
# 285 "/home/tinyos/local/src/tinyos-2.x/tos/lib/net/blip/shell/UDPShellP.nc"
static inline char *UDPShellP__ShellCommand__default__eval(uint8_t cmd_id, int argc, char **argv)
#line 285
{
  return (void *)0;
}

# 11 "/home/tinyos/local/src/tinyos-2.x/tos/lib/net/blip/shell/ShellCommand.nc"
inline static char *UDPShellP__ShellCommand__eval(uint8_t arg_0x415a4698, int argc, char **argv){
#line 11
  char *result;
#line 11

#line 11
    result = UDPShellP__ShellCommand__default__eval(arg_0x415a4698, argc, argv);
#line 11

#line 11
  return result;
#line 11
}
#line 11
# 207 "/home/tinyos/local/src/tinyos-2.x/tos/lib/net/blip/shell/UDPShellP.nc"
static inline void UDPShellP__init_argv(char *cmd, uint16_t len, char **argv, int *argc)
#line 207
{
  int inArg = 0;

#line 209
  *argc = 0;
  while (len > 0 && *argc < UDPShellP__N_ARGS) {
      if ((((*cmd == ' ' || *cmd == '\n') || *cmd == '\t') || *cmd == '\0') || len == 1) {
          if (inArg) {
              *argc = *argc + 1;
              inArg = 0;
              *cmd = '\0';
            }
        }
      else {
#line 217
        if (!inArg) {
            argv[*argc] = cmd;
            inArg = 1;
          }
        }
#line 221
      cmd++;
      len--;
    }
}











static inline void UDPShellP__UDP__recvfrom(struct sockaddr_in6 *from, void *data, 
uint16_t len, struct ip_metadata *meta)
#line 237
{
  char *argv[UDPShellP__N_ARGS];
  int argc;
#line 239
  int cmd;

  ip_memcpy(&UDPShellP__session_endpoint, from, sizeof(struct sockaddr_in6 ));
  UDPShellP__init_argv((char *)data, len, argv, &argc);

  if (argc > 0) {
      cmd = UDPShellP__lookup_cmd(argv[0], UDPShellP__N_BUILTINS, UDPShellP__builtins);
      if (cmd != UDPShellP__CMD_NO_CMD) {
          UDPShellP__builtin_actions[cmd].action(argc, argv);
          return;
        }
      cmd = UDPShellP__lookup_cmd(argv[0], UDPShellP__N_EXTERNAL, UDPShellP__externals);
      if (cmd != UDPShellP__CMD_NO_CMD) {
          char *reply = UDPShellP__ShellCommand__eval(cmd, argc, argv);

#line 253
          if (reply != (void *)0) {
            UDPShellP__UDP__sendto(&UDPShellP__session_endpoint, reply, strlen(reply));
            }
#line 255
          return;
        }
      cmd = snprintf(UDPShellP__reply_buf, MAX_REPLY_LEN, "sdsh: %s: command not found\n", argv[0]);
      UDPShellP__UDP__sendto(&UDPShellP__session_endpoint, UDPShellP__reply_buf, cmd);
    }
}

# 195 "/home/tinyos/local/src/tinyos-2.x/tos/lib/net/blip/UdpP.nc"
static inline void UdpP__UDP__default__recvfrom(uint8_t clnt, struct sockaddr_in6 *from, void *payload, 
uint16_t len, struct ip_metadata *meta)
#line 196
{
}

# 24 "/home/tinyos/local/src/tinyos-2.x/tos/lib/net/blip/interfaces/UDP.nc"
inline static void UdpP__UDP__recvfrom(uint8_t arg_0x413c2e40, struct sockaddr_in6 *src, void *payload, uint16_t len, struct ip_metadata *meta){
#line 24
  switch (arg_0x413c2e40) {
#line 24
    case 0U:
#line 24
      TCPEchoP__Echo__recvfrom(src, payload, len, meta);
#line 24
      break;
#line 24
    case 1U:
#line 24
      TCPEchoP__Status__recvfrom(src, payload, len, meta);
#line 24
      break;
#line 24
    case 2U:
#line 24
      UDPShellP__UDP__recvfrom(src, payload, len, meta);
#line 24
      break;
#line 24
    default:
#line 24
      UdpP__UDP__default__recvfrom(arg_0x413c2e40, src, payload, len, meta);
#line 24
      break;
#line 24
    }
#line 24
}
#line 24
# 68 "/home/tinyos/local/src/tinyos-2.x/tos/lib/net/blip/UdpP.nc"
static inline void UdpP__IP__recv(struct ip6_hdr *iph, 
void *payload, 
struct ip_metadata *meta)
#line 70
{
  unsigned int __nesc_temp89;
  unsigned char *__nesc_temp88;
  unsigned int __nesc_temp87;
  unsigned char *__nesc_temp86;
#line 71
  int i;
  struct sockaddr_in6 addr;
  struct udp_hdr *udph = (struct udp_hdr *)payload;

  ;


  for (i = 0; i < UdpP__N_CLIENTS; i++) 
    if (UdpP__local_ports[i] == udph->dstport) {
      break;
      }
  if (i == UdpP__N_CLIENTS) {

      return;
    }
  ip_memcpy(& addr.sin6_addr, & iph->ip6_src, 16);
  addr.sin6_port = udph->srcport;


  {
    uint16_t rx_cksum = (((uint16_t )udph->chksum >> 8) | ((uint16_t )udph->chksum << 8)) & 0xffff;
#line 91
    uint16_t my_cksum;
    vec_t cksum_vec[4];
    uint32_t hdr[2];

    udph->chksum = 0;

    cksum_vec[0].ptr = (uint8_t *)iph->ip6_src.in6_u.u6_addr8;
    cksum_vec[0].len = 16;
    cksum_vec[1].ptr = (uint8_t *)iph->ip6_dst.in6_u.u6_addr8;
    cksum_vec[1].len = 16;
    cksum_vec[2].ptr = (uint8_t *)hdr;
    cksum_vec[2].len = 8;
    hdr[0] = iph->plen;
    hdr[1] = ntoh32(IANA_UDP);
    cksum_vec[3].ptr = payload;
    cksum_vec[3].len = (((uint16_t )iph->plen >> 8) | ((uint16_t )iph->plen << 8)) & 0xffff;

    my_cksum = in_cksum(cksum_vec, 4);
    ;
    if (rx_cksum != my_cksum) {
        (__nesc_temp86 = UdpP__stats.cksum.data, __nesc_hton_uint16(__nesc_temp86, (__nesc_temp87 = __nesc_ntoh_uint16(__nesc_temp86)) + 1), __nesc_temp87);
      }
  }


  (__nesc_temp88 = UdpP__stats.rcvd.data, __nesc_hton_uint16(__nesc_temp88, (__nesc_temp89 = __nesc_ntoh_uint16(__nesc_temp88)) + 1), __nesc_temp89);
  UdpP__UDP__recvfrom(i, &addr, (void *)(udph + 1), ((((uint16_t )iph->plen >> 8) | ((uint16_t )iph->plen << 8)) & 0xffff) - sizeof(struct udp_hdr ), meta);
}

# 32 "/home/tinyos/local/src/tinyos-2.x/tos/lib/net/blip/interfaces/IPAddress.nc"
inline static void UdpP__IPAddress__setSource(struct ip6_hdr *hdr){
#line 32
  IPAddressP__IPAddress__setSource(hdr);
#line 32
}
#line 32
# 32 "/home/tinyos/local/src/tinyos-2.x/tos/lib/net/blip/UdpP.nc"
static inline uint16_t UdpP__alloc_lport(uint8_t clnt)
#line 32
{
  int i;
#line 33
  int done = 0;
  uint16_t compare = (((uint16_t )UdpP__last_localport << 8) | ((uint16_t )UdpP__last_localport >> 8)) & 0xffff;

#line 35
  UdpP__last_localport = UdpP__last_localport < UdpP__LOCAL_PORT_START ? UdpP__last_localport + 1 : UdpP__LOCAL_PORT_START;
  while (!done) {
      done = 1;
      for (i = 0; i < UdpP__N_CLIENTS; i++) {
          if (UdpP__local_ports[i] == compare) {
              UdpP__last_localport = UdpP__last_localport < UdpP__LOCAL_PORT_START ? UdpP__last_localport + 1 : UdpP__LOCAL_PORT_START;
              compare = (((uint16_t )UdpP__last_localport << 8) | ((uint16_t )UdpP__last_localport >> 8)) & 0xffff;
              done = 0;
              break;
            }
        }
    }
  return UdpP__last_localport;
}

# 15 "/home/tinyos/local/src/tinyos-2.x/tos/lib/net/blip/interfaces/IP.nc"
inline static error_t UdpP__IP__send(struct split_ip_msg *msg){
#line 15
  unsigned char result;
#line 15

#line 15
  result = IPDispatchP__IP__send(IANA_UDP, msg);
#line 15

#line 15
  return result;
#line 15
}
#line 15
# 32 "/home/tinyos/local/src/tinyos-2.x/tos/lib/net/blip/interfaces/IPAddress.nc"
inline static void ICMPResponderP__IPAddress__setSource(struct ip6_hdr *hdr){
#line 32
  IPAddressP__IPAddress__setSource(hdr);
#line 32
}
#line 32
# 262 "/home/tinyos/local/src/tinyos-2.x/tos/lib/net/blip/shell/UDPShellP.nc"
static inline void UDPShellP__ICMPPing__pingReply(struct in6_addr *source, struct icmp_stats *stats)
#line 262
{
  int len;

#line 264
  len = inet_ntop6(source, UDPShellP__reply_buf, MAX_REPLY_LEN);
  if (len > 0) {
      len += snprintf(UDPShellP__reply_buf + len - 1, MAX_REPLY_LEN - len + 1, UDPShellP__ping_fmt, 
      stats->seq, stats->ttl, stats->rtt);
      UDPShellP__UDP__sendto(&UDPShellP__session_endpoint, UDPShellP__reply_buf, len);
    }
}

# 438 "/home/tinyos/local/src/tinyos-2.x/tos/lib/net/blip/ICMPResponderP.nc"
static inline void ICMPResponderP__ICMPPing__default__pingReply(uint16_t client, struct in6_addr *source, 
struct icmp_stats *ping_stats)
#line 439
{
}

# 8 "/home/tinyos/local/src/tinyos-2.x/tos/lib/net/blip/interfaces/ICMPPing.nc"
inline static void ICMPResponderP__ICMPPing__pingReply(uint16_t arg_0x412fc2e0, struct in6_addr *source, struct icmp_stats *stats){
#line 8
  switch (arg_0x412fc2e0) {
#line 8
    case 0U:
#line 8
      UDPShellP__ICMPPing__pingReply(source, stats);
#line 8
      break;
#line 8
    default:
#line 8
      ICMPResponderP__ICMPPing__default__pingReply(arg_0x412fc2e0, source, stats);
#line 8
      break;
#line 8
    }
#line 8
}
#line 8
# 53 "/home/tinyos/local/src/tinyos-2.x/tos/lib/timer/Counter.nc"
inline static /*HilTimerMilliC.CounterToLocalTimeC*/CounterToLocalTimeC__0__Counter__size_type /*HilTimerMilliC.CounterToLocalTimeC*/CounterToLocalTimeC__0__Counter__get(void ){
#line 53
  unsigned long result;
#line 53

#line 53
  result = /*CounterMilli32C.Transform*/TransformCounterC__0__Counter__get();
#line 53

#line 53
  return result;
#line 53
}
#line 53
# 42 "/home/tinyos/local/src/tinyos-2.x/tos/lib/timer/CounterToLocalTimeC.nc"
static inline uint32_t /*HilTimerMilliC.CounterToLocalTimeC*/CounterToLocalTimeC__0__LocalTime__get(void )
{
  return /*HilTimerMilliC.CounterToLocalTimeC*/CounterToLocalTimeC__0__Counter__get();
}

# 50 "/home/tinyos/local/src/tinyos-2.x/tos/lib/timer/LocalTime.nc"
inline static uint32_t ICMPResponderP__LocalTime__get(void ){
#line 50
  unsigned long result;
#line 50

#line 50
  result = /*HilTimerMilliC.CounterToLocalTimeC*/CounterToLocalTimeC__0__LocalTime__get();
#line 50

#line 50
  return result;
#line 50
}
#line 50
# 1095 "/home/tinyos/local/src/tinyos-2.x/tos/lib/net/blip/IPRoutingP.nc"
static inline bool IPRoutingP__IPRouting__hasRoute(void )
#line 1095
{
  return ((&IPRoutingP__neigh_table[0])->flags & T_VALID_MASK) == T_VALID_MASK;
}

# 79 "/home/tinyos/local/src/tinyos-2.x/tos/lib/net/blip/interfaces/IPRouting.nc"
inline static bool ICMPResponderP__IPRouting__hasRoute(void ){
#line 79
  unsigned char result;
#line 79

#line 79
  result = IPRoutingP__IPRouting__hasRoute();
#line 79

#line 79
  return result;
#line 79
}
#line 79
# 69 "/home/tinyos/local/src/tinyos-2.x/tos/lib/net/blip/IPAddressP.nc"
static inline void IPAddressP__IPAddress__setPrefix(uint8_t *pfx)
#line 69
{
  ip_memclr(__my_address.in6_u.u6_addr8, sizeof(struct in6_addr ));
  ip_memcpy(__my_address.in6_u.u6_addr8, pfx, 8);
  globalPrefix = 1;
}

# 34 "/home/tinyos/local/src/tinyos-2.x/tos/lib/net/blip/interfaces/IPAddress.nc"
inline static void ICMPResponderP__IPAddress__setPrefix(uint8_t *prefix){
#line 34
  IPAddressP__IPAddress__setPrefix(prefix);
#line 34
}
#line 34
# 668 "/home/tinyos/local/src/tinyos-2.x/tos/lib/net/blip/IPRoutingP.nc"
static inline void IPRoutingP__printTable(void )
#line 668
{
}

#line 911
static inline void IPRoutingP__IPRouting__reportAdvertisement(ieee154_saddr_t neigh, uint8_t hops, 
uint8_t lqi, uint16_t cost)
#line 912
{




  struct neigh_entry *neigh_slot = (void *)0;

#line 918
  ;
  ;


  if ((neigh_slot = IPRoutingP__getNeighEntry(neigh)) == (void *)0) {
      ;
      if (adjustLQI(lqi) > LQI_ADMIT_THRESH || cost == 0xffff) {
          ;
          return;
        }

      if (!(((&IPRoutingP__neigh_table[N_NEIGH - 1])->flags & T_VALID_MASK) == T_VALID_MASK)) {

          ;
          for (neigh_slot = &IPRoutingP__neigh_table[N_NEIGH - 1]; 
          neigh_slot > &IPRoutingP__neigh_table[0]; neigh_slot--) {



              if (((
#line 936
              neigh_slot - 1)->flags & T_VALID_MASK) == T_VALID_MASK && 
              IPRoutingP__getConfidence(neigh_slot - 1) == 0 && (
              (struct neigh_entry *)(neigh_slot - 1))->costEstimate > cost) {
                  IPRoutingP__swapNodes(neigh_slot - 1, neigh_slot);
                }
              else {
#line 940
                if (((neigh_slot - 1)->flags & T_VALID_MASK) == T_VALID_MASK) {




                    break;
                  }
                }
            }
#line 948
          ip_memclr((void *)neigh_slot, sizeof(struct neigh_entry ));
        }
      else 
#line 949
        {

          ;

          if (((&IPRoutingP__neigh_table[N_NEIGH - 1])->flags & T_MATURE_MASK) == T_MATURE_MASK || 
          hops <= IPRoutingP__neigh_table[N_NEIGH - 1].hops) {
              ;


              if (
#line 957
              IPRoutingP__checkThresh(IPRoutingP__neigh_table[N_NEIGH - 1].costEstimate, cost, 
              PATH_COST_DIFF_THRESH) == BELOW_THRESH || (

              IPRoutingP__checkThresh(IPRoutingP__neigh_table[N_NEIGH - 1].costEstimate, cost, 
              PATH_COST_DIFF_THRESH) == WITHIN_THRESH && 
              IPRoutingP__checkThresh(IPRoutingP__neigh_table[N_NEIGH - 1].linkEstimate, adjustLQI(lqi), 
              LQI_DIFF_THRESH) == BELOW_THRESH)) {
                  ;



                  IPRoutingP__evictNeighbor(&IPRoutingP__neigh_table[N_NEIGH - 1]);
                  neigh_slot = &IPRoutingP__neigh_table[N_NEIGH - 1];
                }
            }
        }
    }
  else 
#line 973
    {
      if (cost == 0xffff) {
          ;
          IPRoutingP__evictNeighbor(neigh_slot);
          return;
        }

      neigh_slot->stats[IPRoutingP__SHORT_EPOCH].receptions--;
    }

  if (neigh_slot != (void *)0) {
      neigh_slot->flags |= T_VALID_MASK;
      neigh_slot->neighbor = neigh;
      neigh_slot->hops = hops;
      neigh_slot->costEstimate = cost;
      neigh_slot->linkEstimate = adjustLQI(lqi);
      neigh_slot->stats[IPRoutingP__SHORT_EPOCH].receptions++;
      ;
    }

  IPRoutingP__printTable();
}

# 66 "/home/tinyos/local/src/tinyos-2.x/tos/lib/net/blip/interfaces/IPRouting.nc"
inline static void ICMPResponderP__IPRouting__reportAdvertisement(ieee154_saddr_t neigh, uint8_t hops, uint8_t lqi, uint16_t cost){
#line 66
  IPRoutingP__IPRouting__reportAdvertisement(neigh, hops, lqi, cost);
#line 66
}
#line 66
#line 83
inline static void ICMPResponderP__IPRouting__reset(void ){
#line 83
  IPRoutingP__IPRouting__reset();
#line 83
}
#line 83
# 214 "/home/tinyos/local/src/tinyos-2.x/tos/lib/net/blip/ICMPResponderP.nc"
static inline void ICMPResponderP__handleRouterAdv(void *payload, uint16_t len, struct ip_metadata *meta)
#line 214
{

  radv_t *r = (radv_t *)payload;
  pfx_t *pfx = (pfx_t *)r->options;
  rqual_t *beacon = (rqual_t *)(pfx + 1);

  if (len > sizeof(radv_t ) + sizeof(pfx_t ) && __nesc_ntoh_uint8(
  beacon->type.data) == ICMP_EXT_TYPE_BEACON) {

      ;


      if ((__nesc_ntoh_uint16(
#line 225
      beacon->seqno.data) > ICMPResponderP__nd_seqno || (
      ICMPResponderP__nd_seqno > 0 && __nesc_ntoh_uint16(beacon->seqno.data) == 0)) || 
      !ICMPResponderP__IPRouting__hasRoute()) {
          ICMPResponderP__IPRouting__reset();
          ICMPResponderP__nd_seqno = __nesc_ntoh_uint16(beacon->seqno.data);
        }

      if (__nesc_ntoh_uint16(beacon->seqno.data) == ICMPResponderP__nd_seqno) {
          ICMPResponderP__IPRouting__reportAdvertisement(meta->sender, __nesc_ntoh_uint8(r->hlim.data), 
          meta->lqi, __nesc_ntoh_uint16(beacon->metric.data));




          if (__nesc_ntoh_uint8(pfx->type.data) != ICMP_EXT_TYPE_PREFIX) {
#line 239
            return;
            }
          ICMPResponderP__IPAddress__setPrefix((uint8_t *)pfx->prefix);
        }


      ;
    }
  else 
#line 246
    {
      ;
    }
}

#line 321
static inline void ICMPResponderP__IP__recv(struct ip6_hdr *iph, 
void *payload, 
struct ip_metadata *meta)
#line 323
{
  unsigned char __nesc_temp85;
  unsigned char *__nesc_temp84;
  unsigned char __nesc_temp83;
  unsigned char *__nesc_temp82;
  unsigned char __nesc_temp81;
  unsigned char *__nesc_temp80;
  unsigned char __nesc_temp79;
  unsigned char *__nesc_temp78;
  unsigned char __nesc_temp77;
  unsigned char *__nesc_temp76;
  unsigned int __nesc_temp75;
  unsigned char *__nesc_temp74;
#line 324
  icmp_echo_hdr_t *req = (icmp_echo_hdr_t *)payload;
  uint16_t len = (((uint16_t )iph->plen >> 8) | ((uint16_t )iph->plen << 8)) & 0xffff;

#line 326
  (__nesc_temp74 = ICMPResponderP__stats.rx.data, __nesc_hton_uint16(__nesc_temp74, (__nesc_temp75 = __nesc_ntoh_uint16(__nesc_temp74)) + 1), __nesc_temp75);


  ;


  switch (__nesc_ntoh_uint8(req->type.data)) {
      case ICMP_TYPE_ROUTER_ADV: 
        ICMPResponderP__handleRouterAdv(payload, len, meta);
      (__nesc_temp76 = ICMPResponderP__stats.adv_rx.data, __nesc_hton_uint8(__nesc_temp76, (__nesc_temp77 = __nesc_ntoh_uint8(__nesc_temp76)) + 1), __nesc_temp77);
      break;
      case ICMP_TYPE_ROUTER_SOL: 

        if (ICMPResponderP__IPRouting__hasRoute()) {
            ICMPResponderP__ICMP__sendAdvertisements();
          }
      (__nesc_temp78 = ICMPResponderP__stats.sol_rx.data, __nesc_hton_uint8(__nesc_temp78, (__nesc_temp79 = __nesc_ntoh_uint8(__nesc_temp78)) + 1), __nesc_temp79);
      break;
      case ICMP_TYPE_ECHO_REPLY: 
        {
          nx_uint32_t *sendTime = (nx_uint32_t *)(req + 1);
          struct icmp_stats p_stat;

#line 348
          p_stat.seq = __nesc_ntoh_uint16(req->seqno.data);
          p_stat.ttl = iph->hlim;
          p_stat.rtt = ICMPResponderP__LocalTime__get() - __nesc_ntoh_uint32((*sendTime).data);
          ICMPResponderP__ICMPPing__pingReply(__nesc_ntoh_uint16(req->ident.data), & iph->ip6_src, &p_stat);
          ICMPResponderP__ping_rcv++;
          (__nesc_temp80 = ICMPResponderP__stats.echo_rx.data, __nesc_hton_uint8(__nesc_temp80, (__nesc_temp81 = __nesc_ntoh_uint8(__nesc_temp80)) + 1), __nesc_temp81);
        }
      break;
      case ICMP_TYPE_ECHO_REQUEST: 
        {

          struct split_ip_msg msg;

#line 360
          msg.headers = (void *)0;
          msg.data = payload;
          msg.data_len = len;

          ip_memcpy(& msg.hdr.ip6_dst, & iph->ip6_src, 16);
          ICMPResponderP__IPAddress__setSource(& msg.hdr);

          __nesc_hton_uint8(req->type.data, ICMP_TYPE_ECHO_REPLY);
          __nesc_hton_uint8(req->code.data, 0);
          __nesc_hton_uint16(req->cksum.data, 0);
          __nesc_hton_uint16(req->cksum.data, ICMPResponderP__ICMP__cksum(&msg, IANA_ICMP));


          ICMPResponderP__IP__send(&msg);
          (__nesc_temp82 = ICMPResponderP__stats.echo_tx.data, __nesc_hton_uint8(__nesc_temp82, (__nesc_temp83 = __nesc_ntoh_uint8(__nesc_temp82)) + 1), __nesc_temp83);
          break;
        }
      default: 
        (__nesc_temp84 = ICMPResponderP__stats.unk_rx.data, __nesc_hton_uint8(__nesc_temp84, (__nesc_temp85 = __nesc_ntoh_uint8(__nesc_temp84)) + 1), __nesc_temp85);
    }
}

# 88 "/home/tinyos/local/src/tinyos-2.x/tos/lib/net/blip/IPRoutingP.nc"
static inline void IPRoutingP__clearStats(struct neigh_entry *r)
#line 88
{
  ip_memclr((uint8_t *)r->stats, sizeof(struct epoch_stats ) * N_EPOCHS);
}

# 41 "/home/tinyos/local/src/tinyos-2.x/tos/interfaces/Random.nc"
inline static uint16_t ICMPResponderP__Random__rand16(void ){
#line 41
  unsigned int result;
#line 41

#line 41
  result = RandomMlcgC__Random__rand16();
#line 41

#line 41
  return result;
#line 41
}
#line 41
# 158 "/home/tinyos/local/src/tinyos-2.x/tos/lib/timer/VirtualizeTimerC.nc"
static inline bool /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__Timer__isRunning(uint8_t num)
{
  return /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__m_timers[num].isrunning;
}

# 81 "/home/tinyos/local/src/tinyos-2.x/tos/lib/timer/Timer.nc"
inline static bool ICMPResponderP__Solicitation__isRunning(void ){
#line 81
  unsigned char result;
#line 81

#line 81
  result = /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__Timer__isRunning(4U);
#line 81

#line 81
  return result;
#line 81
}
#line 81
inline static bool IPRoutingP__TrafficGenTimer__isRunning(void ){
#line 81
  unsigned char result;
#line 81

#line 81
  result = /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__Timer__isRunning(7U);
#line 81

#line 81
  return result;
#line 81
}
#line 81
#line 67
inline static void IPRoutingP__TrafficGenTimer__stop(void ){
#line 67
  /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__Timer__stop(7U);
#line 67
}
#line 67
#line 81
inline static bool ICMPResponderP__Advertisement__isRunning(void ){
#line 81
  unsigned char result;
#line 81

#line 81
  result = /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__Timer__isRunning(5U);
#line 81

#line 81
  return result;
#line 81
}
#line 81
# 150 "/home/tinyos/local/src/tinyos-2.x/tos/lib/net/blip/IPRoutingP.nc"
static inline void IPRoutingP__TGenSend__recv(struct ip6_hdr *iph, 
void *payload, 
struct ip_metadata *meta)
#line 152
{
}

# 178 "/home/tinyos/local/src/tinyos-2.x/tos/chips/cc2420/spi/CC2420SpiP.nc"
static inline uint8_t CC2420SpiP__Resource__isOwner(uint8_t id)
#line 178
{
  /* atomic removed: atomic calls only */
#line 179
  {
    unsigned char __nesc_temp = 
#line 179
    CC2420SpiP__m_holder == id;

#line 179
    return __nesc_temp;
  }
}

# 118 "/home/tinyos/local/src/tinyos-2.x/tos/interfaces/Resource.nc"
inline static bool CC2420ReceiveP__SpiResource__isOwner(void ){
#line 118
  unsigned char result;
#line 118

#line 118
  result = CC2420SpiP__Resource__isOwner(/*CC2420ReceiveC.Spi*/CC2420SpiC__4__CLIENT_ID);
#line 118

#line 118
  return result;
#line 118
}
#line 118
#line 87
inline static error_t CC2420ReceiveP__SpiResource__immediateRequest(void ){
#line 87
  unsigned char result;
#line 87

#line 87
  result = CC2420SpiP__Resource__immediateRequest(/*CC2420ReceiveC.Spi*/CC2420SpiC__4__CLIENT_ID);
#line 87

#line 87
  return result;
#line 87
}
#line 87
#line 78
inline static error_t CC2420ReceiveP__SpiResource__request(void ){
#line 78
  unsigned char result;
#line 78

#line 78
  result = CC2420SpiP__Resource__request(/*CC2420ReceiveC.Spi*/CC2420SpiC__4__CLIENT_ID);
#line 78

#line 78
  return result;
#line 78
}
#line 78
# 56 "/home/tinyos/local/src/tinyos-2.x/tos/interfaces/TaskBasic.nc"
inline static error_t CC2420SpiP__grant__postTask(void ){
#line 56
  unsigned char result;
#line 56

#line 56
  result = SchedulerBasicP__TaskBasic__postTask(CC2420SpiP__grant);
#line 56

#line 56
  return result;
#line 56
}
#line 56
# 184 "/home/tinyos/local/src/tinyos-2.x/tos/chips/cc2420/spi/CC2420SpiP.nc"
static inline void CC2420SpiP__SpiResource__granted(void )
#line 184
{
  CC2420SpiP__grant__postTask();
}

# 114 "/home/tinyos/local/src/tinyos-2.x/tos/chips/msp430/usart/Msp430SpiDmaP.nc"
static inline void /*Msp430SpiDma0P.SpiP*/Msp430SpiDmaP__0__Resource__default__granted(uint8_t id)
#line 114
{
}

# 92 "/home/tinyos/local/src/tinyos-2.x/tos/interfaces/Resource.nc"
inline static void /*Msp430SpiDma0P.SpiP*/Msp430SpiDmaP__0__Resource__granted(uint8_t arg_0x40c701b8){
#line 92
  switch (arg_0x40c701b8) {
#line 92
    case /*CC2420SpiWireC.HplCC2420SpiC.SpiC*/Msp430Spi0C__0__CLIENT_ID:
#line 92
      CC2420SpiP__SpiResource__granted();
#line 92
      break;
#line 92
    default:
#line 92
      /*Msp430SpiDma0P.SpiP*/Msp430SpiDmaP__0__Resource__default__granted(arg_0x40c701b8);
#line 92
      break;
#line 92
    }
#line 92
}
#line 92
# 98 "/home/tinyos/local/src/tinyos-2.x/tos/chips/msp430/usart/Msp430SpiDmaP.nc"
static inline void /*Msp430SpiDma0P.SpiP*/Msp430SpiDmaP__0__UsartResource__granted(uint8_t id)
#line 98
{
  /*Msp430SpiDma0P.SpiP*/Msp430SpiDmaP__0__Resource__granted(id);
}

# 199 "/home/tinyos/local/src/tinyos-2.x/tos/system/ArbiterP.nc"
static inline void /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0__Resource__default__granted(uint8_t id)
#line 199
{
}

# 92 "/home/tinyos/local/src/tinyos-2.x/tos/interfaces/Resource.nc"
inline static void /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0__Resource__granted(uint8_t arg_0x40e33310){
#line 92
  switch (arg_0x40e33310) {
#line 92
    case /*CC2420SpiWireC.HplCC2420SpiC.SpiC.UsartC*/Msp430Usart0C__0__CLIENT_ID:
#line 92
      /*Msp430SpiDma0P.SpiP*/Msp430SpiDmaP__0__UsartResource__granted(/*CC2420SpiWireC.HplCC2420SpiC.SpiC*/Msp430Spi0C__0__CLIENT_ID);
#line 92
      break;
#line 92
    default:
#line 92
      /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0__Resource__default__granted(arg_0x40e33310);
#line 92
      break;
#line 92
    }
#line 92
}
#line 92
# 187 "/home/tinyos/local/src/tinyos-2.x/tos/system/ArbiterP.nc"
static inline void /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0__grantedTask__runTask(void )
#line 187
{
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 188
    {
      /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0__resId = /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0__reqResId;
      /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0__state = /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0__RES_BUSY;
    }
#line 191
    __nesc_atomic_end(__nesc_atomic); }
  /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0__ResourceConfigure__configure(/*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0__resId);
  /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0__Resource__granted(/*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0__resId);
}

# 197 "/home/tinyos/local/src/tinyos-2.x/tos/chips/msp430/usart/Msp430SpiDmaP.nc"
static inline void /*Msp430SpiDma0P.SpiP*/Msp430SpiDmaP__0__SpiPacket__default__sendDone(uint8_t id, uint8_t *tx_buf, uint8_t *rx_buf, uint16_t len, error_t error)
#line 197
{
}

# 71 "/home/tinyos/local/src/tinyos-2.x/tos/interfaces/SpiPacket.nc"
inline static void /*Msp430SpiDma0P.SpiP*/Msp430SpiDmaP__0__SpiPacket__sendDone(uint8_t arg_0x40c6e740, uint8_t * txBuf, uint8_t * rxBuf, uint16_t len, error_t error){
#line 71
  switch (arg_0x40c6e740) {
#line 71
    case /*CC2420SpiWireC.HplCC2420SpiC.SpiC*/Msp430Spi0C__0__CLIENT_ID:
#line 71
      CC2420SpiP__SpiPacket__sendDone(txBuf, rxBuf, len, error);
#line 71
      break;
#line 71
    default:
#line 71
      /*Msp430SpiDma0P.SpiP*/Msp430SpiDmaP__0__SpiPacket__default__sendDone(arg_0x40c6e740, txBuf, rxBuf, len, error);
#line 71
      break;
#line 71
    }
#line 71
}
#line 71
# 190 "/home/tinyos/local/src/tinyos-2.x/tos/chips/msp430/usart/Msp430SpiDmaP.nc"
static inline void /*Msp430SpiDma0P.SpiP*/Msp430SpiDmaP__0__signalDone(error_t error)
#line 190
{
  /*Msp430SpiDma0P.SpiP*/Msp430SpiDmaP__0__SpiPacket__sendDone(/*Msp430SpiDma0P.SpiP*/Msp430SpiDmaP__0__m_client, /*Msp430SpiDma0P.SpiP*/Msp430SpiDmaP__0__m_tx_buf, /*Msp430SpiDma0P.SpiP*/Msp430SpiDmaP__0__m_rx_buf, /*Msp430SpiDma0P.SpiP*/Msp430SpiDmaP__0__m_len, error);
}

#line 180
static inline void /*Msp430SpiDma0P.SpiP*/Msp430SpiDmaP__0__signalDone_task__runTask(void )
#line 180
{
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 181
    /*Msp430SpiDma0P.SpiP*/Msp430SpiDmaP__0__signalDone(SUCCESS);
#line 181
    __nesc_atomic_end(__nesc_atomic); }
}

# 484 "/home/tinyos/local/src/tinyos-2.x/tos/chips/cc2420/transmit/CC2420TransmitP.nc"
static inline void CC2420TransmitP__TXFIFO__readDone(uint8_t *tx_buf, uint8_t tx_len, 
error_t error)
#line 485
{
}

# 110 "/home/tinyos/local/src/tinyos-2.x/tos/interfaces/Resource.nc"
inline static error_t CC2420ReceiveP__SpiResource__release(void ){
#line 110
  unsigned char result;
#line 110

#line 110
  result = CC2420SpiP__Resource__release(/*CC2420ReceiveC.Spi*/CC2420SpiC__4__CLIENT_ID);
#line 110

#line 110
  return result;
#line 110
}
#line 110
# 29 "/home/tinyos/local/src/tinyos-2.x/tos/interfaces/GeneralIO.nc"
inline static void CC2420ReceiveP__CSN__set(void ){
#line 29
  /*HplCC2420PinsC.CSNM*/Msp430GpioC__4__GeneralIO__set();
#line 29
}
#line 29
# 56 "/home/tinyos/local/src/tinyos-2.x/tos/interfaces/TaskBasic.nc"
inline static error_t CC2420ReceiveP__receiveDone_task__postTask(void ){
#line 56
  unsigned char result;
#line 56

#line 56
  result = SchedulerBasicP__TaskBasic__postTask(CC2420ReceiveP__receiveDone_task);
#line 56

#line 56
  return result;
#line 56
}
#line 56
# 47 "/home/tinyos/local/src/tinyos-2.x/tos/chips/cc2420/interfaces/CC2420PacketBody.nc"
inline static cc2420_metadata_t * CC2420TransmitP__CC2420PacketBody__getMetadata(message_t * msg){
#line 47
  nx_struct cc2420_metadata_t *result;
#line 47

#line 47
  result = CC2420PacketP__CC2420PacketBody__getMetadata(msg);
#line 47

#line 47
  return result;
#line 47
}
#line 47
# 387 "/home/tinyos/local/src/tinyos-2.x/tos/chips/cc2420/transmit/CC2420TransmitP.nc"
static inline void CC2420TransmitP__CC2420Receive__receive(uint8_t type, message_t *ack_msg)
#line 387
{
  cc2420_header_t *ack_header;
  cc2420_header_t *msg_header;
  cc2420_metadata_t *msg_metadata;
  uint8_t *ack_buf;
  uint8_t length;

  if (type == IEEE154_TYPE_ACK && CC2420TransmitP__m_msg) {
      ack_header = CC2420TransmitP__CC2420PacketBody__getHeader(ack_msg);
      msg_header = CC2420TransmitP__CC2420PacketBody__getHeader(CC2420TransmitP__m_msg);

      if (CC2420TransmitP__m_state == CC2420TransmitP__S_ACK_WAIT && __nesc_ntoh_leuint8(msg_header->dsn.data) == __nesc_ntoh_leuint8(ack_header->dsn.data)) {
          CC2420TransmitP__BackoffTimer__stop();

          msg_metadata = CC2420TransmitP__CC2420PacketBody__getMetadata(CC2420TransmitP__m_msg);
          ack_buf = (uint8_t *)ack_header;
          length = __nesc_ntoh_leuint8(ack_header->length.data);

          __nesc_hton_int8(msg_metadata->ack.data, TRUE);
          __nesc_hton_uint8(msg_metadata->rssi.data, ack_buf[length - 1]);
          __nesc_hton_uint8(msg_metadata->lqi.data, ack_buf[length] & 0x7f);
          CC2420TransmitP__signalDone(SUCCESS);
        }
    }
}

# 63 "/home/tinyos/local/src/tinyos-2.x/tos/chips/cc2420/interfaces/CC2420Receive.nc"
inline static void CC2420ReceiveP__CC2420Receive__receive(uint8_t type, message_t * message){
#line 63
  CC2420TransmitP__CC2420Receive__receive(type, message);
#line 63
}
#line 63
# 59 "/home/tinyos/local/src/tinyos-2.x/tos/interfaces/PacketTimeStamp.nc"
inline static void CC2420ReceiveP__PacketTimeStamp__clear(message_t * msg){
#line 59
  CC2420PacketP__PacketTimeStamp32khz__clear(msg);
#line 59
}
#line 59








inline static void CC2420ReceiveP__PacketTimeStamp__set(message_t * msg, CC2420ReceiveP__PacketTimeStamp__size_type value){
#line 67
  CC2420PacketP__PacketTimeStamp32khz__set(msg, value);
#line 67
}
#line 67
# 48 "/home/tinyos/local/src/tinyos-2.x/tos/chips/msp430/pins/HplMsp430GeneralIOP.nc"
static inline uint8_t /*HplMsp430GeneralIOC.P10*/HplMsp430GeneralIOP__0__IO__getRaw(void )
#line 48
{
#line 48
  return * (volatile uint8_t * )32U & (0x01 << 0);
}

#line 49
static inline bool /*HplMsp430GeneralIOC.P10*/HplMsp430GeneralIOP__0__IO__get(void )
#line 49
{
#line 49
  return /*HplMsp430GeneralIOC.P10*/HplMsp430GeneralIOP__0__IO__getRaw() != 0;
}

# 59 "/home/tinyos/local/src/tinyos-2.x/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
inline static bool /*HplCC2420PinsC.FIFOPM*/Msp430GpioC__6__HplGeneralIO__get(void ){
#line 59
  unsigned char result;
#line 59

#line 59
  result = /*HplMsp430GeneralIOC.P10*/HplMsp430GeneralIOP__0__IO__get();
#line 59

#line 59
  return result;
#line 59
}
#line 59
# 40 "/home/tinyos/local/src/tinyos-2.x/tos/chips/msp430/pins/Msp430GpioC.nc"
static inline bool /*HplCC2420PinsC.FIFOPM*/Msp430GpioC__6__GeneralIO__get(void )
#line 40
{
#line 40
  return /*HplCC2420PinsC.FIFOPM*/Msp430GpioC__6__HplGeneralIO__get();
}

# 32 "/home/tinyos/local/src/tinyos-2.x/tos/interfaces/GeneralIO.nc"
inline static bool CC2420ReceiveP__FIFOP__get(void ){
#line 32
  unsigned char result;
#line 32

#line 32
  result = /*HplCC2420PinsC.FIFOPM*/Msp430GpioC__6__GeneralIO__get();
#line 32

#line 32
  return result;
#line 32
}
#line 32
# 48 "/home/tinyos/local/src/tinyos-2.x/tos/chips/msp430/pins/HplMsp430GeneralIOP.nc"
static inline uint8_t /*HplMsp430GeneralIOC.P13*/HplMsp430GeneralIOP__3__IO__getRaw(void )
#line 48
{
#line 48
  return * (volatile uint8_t * )32U & (0x01 << 3);
}

#line 49
static inline bool /*HplMsp430GeneralIOC.P13*/HplMsp430GeneralIOP__3__IO__get(void )
#line 49
{
#line 49
  return /*HplMsp430GeneralIOC.P13*/HplMsp430GeneralIOP__3__IO__getRaw() != 0;
}

# 59 "/home/tinyos/local/src/tinyos-2.x/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
inline static bool /*HplCC2420PinsC.FIFOM*/Msp430GpioC__5__HplGeneralIO__get(void ){
#line 59
  unsigned char result;
#line 59

#line 59
  result = /*HplMsp430GeneralIOC.P13*/HplMsp430GeneralIOP__3__IO__get();
#line 59

#line 59
  return result;
#line 59
}
#line 59
# 40 "/home/tinyos/local/src/tinyos-2.x/tos/chips/msp430/pins/Msp430GpioC.nc"
static inline bool /*HplCC2420PinsC.FIFOM*/Msp430GpioC__5__GeneralIO__get(void )
#line 40
{
#line 40
  return /*HplCC2420PinsC.FIFOM*/Msp430GpioC__5__HplGeneralIO__get();
}

# 32 "/home/tinyos/local/src/tinyos-2.x/tos/interfaces/GeneralIO.nc"
inline static bool CC2420ReceiveP__FIFO__get(void ){
#line 32
  unsigned char result;
#line 32

#line 32
  result = /*HplCC2420PinsC.FIFOM*/Msp430GpioC__5__GeneralIO__get();
#line 32

#line 32
  return result;
#line 32
}
#line 32
# 209 "/home/tinyos/local/src/tinyos-2.x/tos/chips/cc2420/spi/CC2420SpiP.nc"
static inline error_t CC2420SpiP__Fifo__continueRead(uint8_t addr, uint8_t *data, 
uint8_t len)
#line 210
{
  return CC2420SpiP__SpiPacket__send((void *)0, data, len);
}

# 62 "/home/tinyos/local/src/tinyos-2.x/tos/chips/cc2420/interfaces/CC2420Fifo.nc"
inline static error_t CC2420ReceiveP__RXFIFO__continueRead(uint8_t * data, uint8_t length){
#line 62
  unsigned char result;
#line 62

#line 62
  result = CC2420SpiP__Fifo__continueRead(CC2420_RXFIFO, data, length);
#line 62

#line 62
  return result;
#line 62
}
#line 62
#line 51
inline static cc2420_status_t CC2420ReceiveP__RXFIFO__beginRead(uint8_t * data, uint8_t length){
#line 51
  unsigned char result;
#line 51

#line 51
  result = CC2420SpiP__Fifo__beginRead(CC2420_RXFIFO, data, length);
#line 51

#line 51
  return result;
#line 51
}
#line 51
# 30 "/home/tinyos/local/src/tinyos-2.x/tos/interfaces/GeneralIO.nc"
inline static void CC2420ReceiveP__CSN__clr(void ){
#line 30
  /*HplCC2420PinsC.CSNM*/Msp430GpioC__4__GeneralIO__clr();
#line 30
}
#line 30
# 45 "/home/tinyos/local/src/tinyos-2.x/tos/chips/cc2420/interfaces/CC2420Strobe.nc"
inline static cc2420_status_t CC2420ReceiveP__SACK__strobe(void ){
#line 45
  unsigned char result;
#line 45

#line 45
  result = CC2420SpiP__Strobe__strobe(CC2420_SACK);
#line 45

#line 45
  return result;
#line 45
}
#line 45
# 359 "/home/tinyos/local/src/tinyos-2.x/tos/chips/cc2420/control/CC2420ControlP.nc"
static inline bool CC2420ControlP__CC2420Config__isHwAutoAckDefault(void )
#line 359
{
  /* atomic removed: atomic calls only */
#line 360
  {
    unsigned char __nesc_temp = 
#line 360
    CC2420ControlP__hwAutoAckDefault;

#line 360
    return __nesc_temp;
  }
}

# 105 "/home/tinyos/local/src/tinyos-2.x/tos/chips/cc2420/interfaces/CC2420Config.nc"
inline static bool CC2420ReceiveP__CC2420Config__isHwAutoAckDefault(void ){
#line 105
  unsigned char result;
#line 105

#line 105
  result = CC2420ControlP__CC2420Config__isHwAutoAckDefault();
#line 105

#line 105
  return result;
#line 105
}
#line 105
# 366 "/home/tinyos/local/src/tinyos-2.x/tos/chips/cc2420/control/CC2420ControlP.nc"
static inline bool CC2420ControlP__CC2420Config__isAutoAckEnabled(void )
#line 366
{
  /* atomic removed: atomic calls only */
#line 367
  {
    unsigned char __nesc_temp = 
#line 367
    CC2420ControlP__autoAckEnabled;

#line 367
    return __nesc_temp;
  }
}

# 110 "/home/tinyos/local/src/tinyos-2.x/tos/chips/cc2420/interfaces/CC2420Config.nc"
inline static bool CC2420ReceiveP__CC2420Config__isAutoAckEnabled(void ){
#line 110
  unsigned char result;
#line 110

#line 110
  result = CC2420ControlP__CC2420Config__isAutoAckEnabled();
#line 110

#line 110
  return result;
#line 110
}
#line 110
# 525 "/home/tinyos/local/src/tinyos-2.x/tos/chips/cc2420/receive/CC2420ReceiveP.nc"
static inline void CC2420ReceiveP__RXFIFO__readDone(uint8_t *rx_buf, uint8_t rx_len, 
error_t error)
#line 526
{
  cc2420_header_t *header = CC2420ReceiveP__CC2420PacketBody__getHeader(CC2420ReceiveP__m_p_rx_buf);
  uint8_t tmpLen __attribute((unused))  = sizeof(message_t ) - ((size_t )& ((message_t *)0)->data - sizeof(cc2420_header_t ));
  uint8_t * buf = (uint8_t * )header;

#line 530
  CC2420ReceiveP__rxFrameLength = buf[0];

  switch (CC2420ReceiveP__m_state) {

      case CC2420ReceiveP__S_RX_LENGTH: 
        CC2420ReceiveP__m_state = CC2420ReceiveP__S_RX_FCF;



      if (CC2420ReceiveP__rxFrameLength + 1 > CC2420ReceiveP__m_bytes_left) 



        {

          CC2420ReceiveP__flush();
        }
      else {
          if (!CC2420ReceiveP__FIFO__get() && !CC2420ReceiveP__FIFOP__get()) {
              CC2420ReceiveP__m_bytes_left -= CC2420ReceiveP__rxFrameLength + 1;
            }

          if (CC2420ReceiveP__rxFrameLength <= MAC_PACKET_SIZE) {
              if (CC2420ReceiveP__rxFrameLength > 0) {
                  if (CC2420ReceiveP__rxFrameLength > CC2420ReceiveP__SACK_HEADER_LENGTH) {

                      CC2420ReceiveP__RXFIFO__continueRead(buf + 1, CC2420ReceiveP__SACK_HEADER_LENGTH);
                    }
                  else {

                      CC2420ReceiveP__m_state = CC2420ReceiveP__S_RX_PAYLOAD;
                      CC2420ReceiveP__RXFIFO__continueRead(buf + 1, CC2420ReceiveP__rxFrameLength);
                    }
                }
              else {
                  /* atomic removed: atomic calls only */
                  CC2420ReceiveP__receivingPacket = FALSE;
                  CC2420ReceiveP__CSN__set();
                  CC2420ReceiveP__SpiResource__release();
                  CC2420ReceiveP__waitForNextPacket();
                }
            }
          else {

              CC2420ReceiveP__flush();
            }
        }
      break;

      case CC2420ReceiveP__S_RX_FCF: 
        CC2420ReceiveP__m_state = CC2420ReceiveP__S_RX_PAYLOAD;










      if (CC2420ReceiveP__CC2420Config__isAutoAckEnabled() && !CC2420ReceiveP__CC2420Config__isHwAutoAckDefault()) {



          if (((__nesc_ntoh_leuint16(
#line 592
          header->fcf.data) >> IEEE154_FCF_ACK_REQ) & 0x01) == 1
           && (__nesc_ntoh_leuint16(header->dest.data) == CC2420ReceiveP__CC2420Config__getShortAddr()
           || __nesc_ntoh_leuint16(header->dest.data) == AM_BROADCAST_ADDR)
           && ((__nesc_ntoh_leuint16(header->fcf.data) >> IEEE154_FCF_FRAME_TYPE) & 7) == IEEE154_TYPE_DATA) {

              CC2420ReceiveP__CSN__set();
              CC2420ReceiveP__CSN__clr();
              CC2420ReceiveP__SACK__strobe();
              CC2420ReceiveP__CSN__set();
              CC2420ReceiveP__CSN__clr();
              CC2420ReceiveP__RXFIFO__beginRead(buf + 1 + CC2420ReceiveP__SACK_HEADER_LENGTH, 
              CC2420ReceiveP__rxFrameLength - CC2420ReceiveP__SACK_HEADER_LENGTH);
              return;
            }
        }

      CC2420ReceiveP__RXFIFO__continueRead(buf + 1 + CC2420ReceiveP__SACK_HEADER_LENGTH, 
      CC2420ReceiveP__rxFrameLength - CC2420ReceiveP__SACK_HEADER_LENGTH);
      break;

      case CC2420ReceiveP__S_RX_PAYLOAD: 

        CC2420ReceiveP__CSN__set();
      if (!CC2420ReceiveP__m_missed_packets) {

          CC2420ReceiveP__SpiResource__release();
        }




      if ((((
#line 621
      CC2420ReceiveP__m_missed_packets && CC2420ReceiveP__FIFO__get()) || !CC2420ReceiveP__FIFOP__get())
       || !CC2420ReceiveP__m_timestamp_size)
       || CC2420ReceiveP__rxFrameLength <= 10) {
          CC2420ReceiveP__PacketTimeStamp__clear(CC2420ReceiveP__m_p_rx_buf);
        }
      else {
          if (CC2420ReceiveP__m_timestamp_size == 1) {
            CC2420ReceiveP__PacketTimeStamp__set(CC2420ReceiveP__m_p_rx_buf, CC2420ReceiveP__m_timestamp_queue[CC2420ReceiveP__m_timestamp_head]);
            }
#line 629
          CC2420ReceiveP__m_timestamp_head = (CC2420ReceiveP__m_timestamp_head + 1) % CC2420ReceiveP__TIMESTAMP_QUEUE_SIZE;
          CC2420ReceiveP__m_timestamp_size--;

          if (CC2420ReceiveP__m_timestamp_size > 0) {
              CC2420ReceiveP__PacketTimeStamp__clear(CC2420ReceiveP__m_p_rx_buf);
              CC2420ReceiveP__m_timestamp_head = 0;
              CC2420ReceiveP__m_timestamp_size = 0;
            }
        }



      if (buf[CC2420ReceiveP__rxFrameLength] >> 7 && rx_buf) {
          uint8_t type = (__nesc_ntoh_leuint16(header->fcf.data) >> IEEE154_FCF_FRAME_TYPE) & 7;

#line 643
          CC2420ReceiveP__CC2420Receive__receive(type, CC2420ReceiveP__m_p_rx_buf);
          if (type == IEEE154_TYPE_DATA) {
              CC2420ReceiveP__receiveDone_task__postTask();
              return;
            }
        }

      CC2420ReceiveP__waitForNextPacket();
      break;

      default: /* atomic removed: atomic calls only */
        CC2420ReceiveP__receivingPacket = FALSE;
      CC2420ReceiveP__CSN__set();
      CC2420ReceiveP__SpiResource__release();
      break;
    }
}

# 370 "/home/tinyos/local/src/tinyos-2.x/tos/chips/cc2420/spi/CC2420SpiP.nc"
static inline void CC2420SpiP__Fifo__default__readDone(uint8_t addr, uint8_t *rx_buf, uint8_t rx_len, error_t error)
#line 370
{
}

# 71 "/home/tinyos/local/src/tinyos-2.x/tos/chips/cc2420/interfaces/CC2420Fifo.nc"
inline static void CC2420SpiP__Fifo__readDone(uint8_t arg_0x40bbb010, uint8_t * data, uint8_t length, error_t error){
#line 71
  switch (arg_0x40bbb010) {
#line 71
    case CC2420_TXFIFO:
#line 71
      CC2420TransmitP__TXFIFO__readDone(data, length, error);
#line 71
      break;
#line 71
    case CC2420_RXFIFO:
#line 71
      CC2420ReceiveP__RXFIFO__readDone(data, length, error);
#line 71
      break;
#line 71
    default:
#line 71
      CC2420SpiP__Fifo__default__readDone(arg_0x40bbb010, data, length, error);
#line 71
      break;
#line 71
    }
#line 71
}
#line 71
# 45 "/home/tinyos/local/src/tinyos-2.x/tos/chips/cc2420/interfaces/CC2420Strobe.nc"
inline static cc2420_status_t CC2420ReceiveP__SFLUSHRX__strobe(void ){
#line 45
  unsigned char result;
#line 45

#line 45
  result = CC2420SpiP__Strobe__strobe(CC2420_SFLUSHRX);
#line 45

#line 45
  return result;
#line 45
}
#line 45
# 285 "/home/tinyos/local/src/tinyos-2.x/tos/chips/cc2420/csma/CC2420CsmaP.nc"
static inline void CC2420CsmaP__RadioBackoff__default__requestInitialBackoff(message_t *msg)
#line 285
{
}

# 81 "/home/tinyos/local/src/tinyos-2.x/tos/chips/cc2420/interfaces/RadioBackoff.nc"
inline static void CC2420CsmaP__RadioBackoff__requestInitialBackoff(message_t * msg){
#line 81
  CC2420CsmaP__RadioBackoff__default__requestInitialBackoff(msg);
#line 81
}
#line 81
# 243 "/home/tinyos/local/src/tinyos-2.x/tos/chips/cc2420/transmit/CC2420TransmitP.nc"
static inline void CC2420TransmitP__RadioBackoff__setInitialBackoff(uint16_t backoffTime)
#line 243
{
  CC2420TransmitP__myInitialBackoff = backoffTime + 1;
}

# 60 "/home/tinyos/local/src/tinyos-2.x/tos/chips/cc2420/interfaces/RadioBackoff.nc"
inline static void CC2420CsmaP__SubBackoff__setInitialBackoff(uint16_t backoffTime){
#line 60
  CC2420TransmitP__RadioBackoff__setInitialBackoff(backoffTime);
#line 60
}
#line 60
# 220 "/home/tinyos/local/src/tinyos-2.x/tos/chips/cc2420/csma/CC2420CsmaP.nc"
static inline void CC2420CsmaP__SubBackoff__requestInitialBackoff(message_t *msg)
#line 220
{
  CC2420CsmaP__SubBackoff__setInitialBackoff(CC2420CsmaP__Random__rand16()
   % (0x1F * CC2420_BACKOFF_PERIOD) + CC2420_MIN_BACKOFF);

  CC2420CsmaP__RadioBackoff__requestInitialBackoff(msg);
}

# 81 "/home/tinyos/local/src/tinyos-2.x/tos/chips/cc2420/interfaces/RadioBackoff.nc"
inline static void CC2420TransmitP__RadioBackoff__requestInitialBackoff(message_t * msg){
#line 81
  CC2420CsmaP__SubBackoff__requestInitialBackoff(msg);
#line 81
}
#line 81
# 56 "/home/tinyos/local/src/tinyos-2.x/tos/interfaces/TaskBasic.nc"
inline static error_t CC2420CsmaP__sendDone_task__postTask(void ){
#line 56
  unsigned char result;
#line 56

#line 56
  result = SchedulerBasicP__TaskBasic__postTask(CC2420CsmaP__sendDone_task);
#line 56

#line 56
  return result;
#line 56
}
#line 56
# 202 "/home/tinyos/local/src/tinyos-2.x/tos/chips/cc2420/csma/CC2420CsmaP.nc"
static inline void CC2420CsmaP__CC2420Transmit__sendDone(message_t *p_msg, error_t err)
#line 202
{
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 203
    CC2420CsmaP__sendErr = err;
#line 203
    __nesc_atomic_end(__nesc_atomic); }
  CC2420CsmaP__sendDone_task__postTask();
}

# 73 "/home/tinyos/local/src/tinyos-2.x/tos/chips/cc2420/interfaces/CC2420Transmit.nc"
inline static void CC2420TransmitP__Send__sendDone(message_t * p_msg, error_t error){
#line 73
  CC2420CsmaP__CC2420Transmit__sendDone(p_msg, error);
#line 73
}
#line 73
# 452 "/home/tinyos/local/src/tinyos-2.x/tos/chips/cc2420/transmit/CC2420TransmitP.nc"
static inline void CC2420TransmitP__TXFIFO__writeDone(uint8_t *tx_buf, uint8_t tx_len, 
error_t error)
#line 453
{

  CC2420TransmitP__CSN__set();
  if (CC2420TransmitP__m_state == CC2420TransmitP__S_CANCEL) {
      /* atomic removed: atomic calls only */
#line 457
      {
        CC2420TransmitP__CSN__clr();
        CC2420TransmitP__SFLUSHTX__strobe();
        CC2420TransmitP__CSN__set();
      }
      CC2420TransmitP__releaseSpiResource();
      CC2420TransmitP__m_state = CC2420TransmitP__S_STARTED;
      CC2420TransmitP__Send__sendDone(CC2420TransmitP__m_msg, ECANCEL);
    }
  else {
#line 466
    if (!CC2420TransmitP__m_cca) {
        /* atomic removed: atomic calls only */
#line 467
        {
          CC2420TransmitP__m_state = CC2420TransmitP__S_BEGIN_TRANSMIT;
        }
        CC2420TransmitP__attemptSend();
      }
    else {
        CC2420TransmitP__releaseSpiResource();
        /* atomic removed: atomic calls only */
#line 474
        {
          CC2420TransmitP__m_state = CC2420TransmitP__S_SAMPLE_CCA;
        }

        CC2420TransmitP__RadioBackoff__requestInitialBackoff(CC2420TransmitP__m_msg);
        CC2420TransmitP__BackoffTimer__start(CC2420TransmitP__myInitialBackoff);
      }
    }
}

# 663 "/home/tinyos/local/src/tinyos-2.x/tos/chips/cc2420/receive/CC2420ReceiveP.nc"
static inline void CC2420ReceiveP__RXFIFO__writeDone(uint8_t *tx_buf, uint8_t tx_len, error_t error)
#line 663
{
}

# 373 "/home/tinyos/local/src/tinyos-2.x/tos/chips/cc2420/spi/CC2420SpiP.nc"
static inline void CC2420SpiP__Fifo__default__writeDone(uint8_t addr, uint8_t *tx_buf, uint8_t tx_len, error_t error)
#line 373
{
}

# 91 "/home/tinyos/local/src/tinyos-2.x/tos/chips/cc2420/interfaces/CC2420Fifo.nc"
inline static void CC2420SpiP__Fifo__writeDone(uint8_t arg_0x40bbb010, uint8_t * data, uint8_t length, error_t error){
#line 91
  switch (arg_0x40bbb010) {
#line 91
    case CC2420_TXFIFO:
#line 91
      CC2420TransmitP__TXFIFO__writeDone(data, length, error);
#line 91
      break;
#line 91
    case CC2420_RXFIFO:
#line 91
      CC2420ReceiveP__RXFIFO__writeDone(data, length, error);
#line 91
      break;
#line 91
    default:
#line 91
      CC2420SpiP__Fifo__default__writeDone(arg_0x40bbb010, data, length, error);
#line 91
      break;
#line 91
    }
#line 91
}
#line 91
# 55 "/home/tinyos/local/src/tinyos-2.x/tos/chips/cc2420/interfaces/CC2420Register.nc"
inline static cc2420_status_t CC2420ControlP__RXCTRL1__write(uint16_t data){
#line 55
  unsigned char result;
#line 55

#line 55
  result = CC2420SpiP__Reg__write(CC2420_RXCTRL1, data);
#line 55

#line 55
  return result;
#line 55
}
#line 55
inline static cc2420_status_t CC2420ControlP__IOCFG0__write(uint16_t data){
#line 55
  unsigned char result;
#line 55

#line 55
  result = CC2420SpiP__Reg__write(CC2420_IOCFG0, data);
#line 55

#line 55
  return result;
#line 55
}
#line 55
# 45 "/home/tinyos/local/src/tinyos-2.x/tos/chips/cc2420/interfaces/CC2420Strobe.nc"
inline static cc2420_status_t CC2420ControlP__SXOSCON__strobe(void ){
#line 45
  unsigned char result;
#line 45

#line 45
  result = CC2420SpiP__Strobe__strobe(CC2420_SXOSCON);
#line 45

#line 45
  return result;
#line 45
}
#line 45
# 79 "/home/tinyos/local/src/tinyos-2.x/tos/chips/msp430/pins/HplMsp430InterruptP.nc"
static inline void HplMsp430InterruptP__Port14__enable(void )
#line 79
{
#line 79
  P1IE |= 1 << 4;
}

# 31 "/home/tinyos/local/src/tinyos-2.x/tos/chips/msp430/pins/HplMsp430Interrupt.nc"
inline static void /*HplCC2420InterruptsC.InterruptCCAC*/Msp430InterruptC__0__HplInterrupt__enable(void ){
#line 31
  HplMsp430InterruptP__Port14__enable();
#line 31
}
#line 31
# 131 "/home/tinyos/local/src/tinyos-2.x/tos/chips/msp430/pins/HplMsp430InterruptP.nc"
static inline void HplMsp430InterruptP__Port14__edge(bool l2h)
#line 131
{
  /* atomic removed: atomic calls only */
#line 132
  {
    if (l2h) {
#line 133
      P1IES &= ~(1 << 4);
      }
    else {
#line 134
      P1IES |= 1 << 4;
      }
  }
}

# 56 "/home/tinyos/local/src/tinyos-2.x/tos/chips/msp430/pins/HplMsp430Interrupt.nc"
inline static void /*HplCC2420InterruptsC.InterruptCCAC*/Msp430InterruptC__0__HplInterrupt__edge(bool low_to_high){
#line 56
  HplMsp430InterruptP__Port14__edge(low_to_high);
#line 56
}
#line 56
# 95 "/home/tinyos/local/src/tinyos-2.x/tos/chips/msp430/pins/HplMsp430InterruptP.nc"
static inline void HplMsp430InterruptP__Port14__clear(void )
#line 95
{
#line 95
  P1IFG &= ~(1 << 4);
}

# 41 "/home/tinyos/local/src/tinyos-2.x/tos/chips/msp430/pins/HplMsp430Interrupt.nc"
inline static void /*HplCC2420InterruptsC.InterruptCCAC*/Msp430InterruptC__0__HplInterrupt__clear(void ){
#line 41
  HplMsp430InterruptP__Port14__clear();
#line 41
}
#line 41
# 87 "/home/tinyos/local/src/tinyos-2.x/tos/chips/msp430/pins/HplMsp430InterruptP.nc"
static inline void HplMsp430InterruptP__Port14__disable(void )
#line 87
{
#line 87
  P1IE &= ~(1 << 4);
}

# 36 "/home/tinyos/local/src/tinyos-2.x/tos/chips/msp430/pins/HplMsp430Interrupt.nc"
inline static void /*HplCC2420InterruptsC.InterruptCCAC*/Msp430InterruptC__0__HplInterrupt__disable(void ){
#line 36
  HplMsp430InterruptP__Port14__disable();
#line 36
}
#line 36
# 58 "/home/tinyos/local/src/tinyos-2.x/tos/chips/msp430/pins/Msp430InterruptC.nc"
static inline error_t /*HplCC2420InterruptsC.InterruptCCAC*/Msp430InterruptC__0__Interrupt__disable(void )
#line 58
{
  /* atomic removed: atomic calls only */
#line 59
  {
    /*HplCC2420InterruptsC.InterruptCCAC*/Msp430InterruptC__0__HplInterrupt__disable();
    /*HplCC2420InterruptsC.InterruptCCAC*/Msp430InterruptC__0__HplInterrupt__clear();
  }
  return SUCCESS;
}

#line 41
static inline error_t /*HplCC2420InterruptsC.InterruptCCAC*/Msp430InterruptC__0__enable(bool rising)
#line 41
{
  /* atomic removed: atomic calls only */
#line 42
  {
    /*HplCC2420InterruptsC.InterruptCCAC*/Msp430InterruptC__0__Interrupt__disable();
    /*HplCC2420InterruptsC.InterruptCCAC*/Msp430InterruptC__0__HplInterrupt__edge(rising);
    /*HplCC2420InterruptsC.InterruptCCAC*/Msp430InterruptC__0__HplInterrupt__enable();
  }
  return SUCCESS;
}

static inline error_t /*HplCC2420InterruptsC.InterruptCCAC*/Msp430InterruptC__0__Interrupt__enableRisingEdge(void )
#line 50
{
  return /*HplCC2420InterruptsC.InterruptCCAC*/Msp430InterruptC__0__enable(TRUE);
}

# 42 "/home/tinyos/local/src/tinyos-2.x/tos/interfaces/GpioInterrupt.nc"
inline static error_t CC2420ControlP__InterruptCCA__enableRisingEdge(void ){
#line 42
  unsigned char result;
#line 42

#line 42
  result = /*HplCC2420InterruptsC.InterruptCCAC*/Msp430InterruptC__0__Interrupt__enableRisingEdge();
#line 42

#line 42
  return result;
#line 42
}
#line 42
# 55 "/home/tinyos/local/src/tinyos-2.x/tos/chips/cc2420/interfaces/CC2420Register.nc"
inline static cc2420_status_t CC2420ControlP__IOCFG1__write(uint16_t data){
#line 55
  unsigned char result;
#line 55

#line 55
  result = CC2420SpiP__Reg__write(CC2420_IOCFG1, data);
#line 55

#line 55
  return result;
#line 55
}
#line 55
# 207 "/home/tinyos/local/src/tinyos-2.x/tos/chips/cc2420/control/CC2420ControlP.nc"
static inline error_t CC2420ControlP__CC2420Power__startOscillator(void )
#line 207
{
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 208
    {
      if (CC2420ControlP__m_state != CC2420ControlP__S_VREG_STARTED) {
          {
            unsigned char __nesc_temp = 
#line 210
            FAIL;

            {
#line 210
              __nesc_atomic_end(__nesc_atomic); 
#line 210
              return __nesc_temp;
            }
          }
        }
#line 213
      CC2420ControlP__m_state = CC2420ControlP__S_XOSC_STARTING;
      CC2420ControlP__IOCFG1__write(CC2420_SFDMUX_XOSC16M_STABLE << 
      CC2420_IOCFG1_CCAMUX);

      CC2420ControlP__InterruptCCA__enableRisingEdge();
      CC2420ControlP__SXOSCON__strobe();

      CC2420ControlP__IOCFG0__write((1 << CC2420_IOCFG0_FIFOP_POLARITY) | (
      127 << CC2420_IOCFG0_FIFOP_THR));

      CC2420ControlP__writeFsctrl();
      CC2420ControlP__writeMdmctrl0();

      CC2420ControlP__RXCTRL1__write(((((((1 << CC2420_RXCTRL1_RXBPF_LOCUR) | (
      1 << CC2420_RXCTRL1_LOW_LOWGAIN)) | (
      1 << CC2420_RXCTRL1_HIGH_HGM)) | (
      1 << CC2420_RXCTRL1_LNA_CAP_ARRAY)) | (
      1 << CC2420_RXCTRL1_RXMIX_TAIL)) | (
      1 << CC2420_RXCTRL1_RXMIX_VCM)) | (
      2 << CC2420_RXCTRL1_RXMIX_CURRENT));
    }
#line 233
    __nesc_atomic_end(__nesc_atomic); }
  return SUCCESS;
}

# 71 "/home/tinyos/local/src/tinyos-2.x/tos/chips/cc2420/interfaces/CC2420Power.nc"
inline static error_t CC2420CsmaP__CC2420Power__startOscillator(void ){
#line 71
  unsigned char result;
#line 71

#line 71
  result = CC2420ControlP__CC2420Power__startOscillator();
#line 71

#line 71
  return result;
#line 71
}
#line 71
# 211 "/home/tinyos/local/src/tinyos-2.x/tos/chips/cc2420/csma/CC2420CsmaP.nc"
static inline void CC2420CsmaP__Resource__granted(void )
#line 211
{
  CC2420CsmaP__CC2420Power__startOscillator();
}

# 92 "/home/tinyos/local/src/tinyos-2.x/tos/interfaces/Resource.nc"
inline static void CC2420ControlP__Resource__granted(void ){
#line 92
  CC2420CsmaP__Resource__granted();
#line 92
}
#line 92
# 30 "/home/tinyos/local/src/tinyos-2.x/tos/interfaces/GeneralIO.nc"
inline static void CC2420ControlP__CSN__clr(void ){
#line 30
  /*HplCC2420PinsC.CSNM*/Msp430GpioC__4__GeneralIO__clr();
#line 30
}
#line 30
# 390 "/home/tinyos/local/src/tinyos-2.x/tos/chips/cc2420/control/CC2420ControlP.nc"
static inline void CC2420ControlP__SpiResource__granted(void )
#line 390
{
  CC2420ControlP__CSN__clr();
  CC2420ControlP__Resource__granted();
}

# 56 "/home/tinyos/local/src/tinyos-2.x/tos/interfaces/TaskBasic.nc"
inline static error_t CC2420ControlP__syncDone__postTask(void ){
#line 56
  unsigned char result;
#line 56

#line 56
  result = SchedulerBasicP__TaskBasic__postTask(CC2420ControlP__syncDone);
#line 56

#line 56
  return result;
#line 56
}
#line 56
# 110 "/home/tinyos/local/src/tinyos-2.x/tos/interfaces/Resource.nc"
inline static error_t CC2420ControlP__SyncResource__release(void ){
#line 110
  unsigned char result;
#line 110

#line 110
  result = CC2420SpiP__Resource__release(/*CC2420ControlC.SyncSpiC*/CC2420SpiC__1__CLIENT_ID);
#line 110

#line 110
  return result;
#line 110
}
#line 110
# 29 "/home/tinyos/local/src/tinyos-2.x/tos/interfaces/GeneralIO.nc"
inline static void CC2420ControlP__CSN__set(void ){
#line 29
  /*HplCC2420PinsC.CSNM*/Msp430GpioC__4__GeneralIO__set();
#line 29
}
#line 29
# 45 "/home/tinyos/local/src/tinyos-2.x/tos/chips/cc2420/interfaces/CC2420Strobe.nc"
inline static cc2420_status_t CC2420ControlP__SRXON__strobe(void ){
#line 45
  unsigned char result;
#line 45

#line 45
  result = CC2420SpiP__Strobe__strobe(CC2420_SRXON);
#line 45

#line 45
  return result;
#line 45
}
#line 45
inline static cc2420_status_t CC2420ControlP__SRFOFF__strobe(void ){
#line 45
  unsigned char result;
#line 45

#line 45
  result = CC2420SpiP__Strobe__strobe(CC2420_SRFOFF);
#line 45

#line 45
  return result;
#line 45
}
#line 45
# 376 "/home/tinyos/local/src/tinyos-2.x/tos/chips/cc2420/control/CC2420ControlP.nc"
static inline void CC2420ControlP__SyncResource__granted(void )
#line 376
{
  CC2420ControlP__CSN__clr();
  CC2420ControlP__SRFOFF__strobe();
  CC2420ControlP__writeFsctrl();
  CC2420ControlP__writeMdmctrl0();
  CC2420ControlP__writeId();
  CC2420ControlP__CSN__set();
  CC2420ControlP__CSN__clr();
  CC2420ControlP__SRXON__strobe();
  CC2420ControlP__CSN__set();
  CC2420ControlP__SyncResource__release();
  CC2420ControlP__syncDone__postTask();
}

#line 509
static inline void CC2420ControlP__ReadRssi__default__readDone(error_t error, uint16_t data)
#line 509
{
}

# 63 "/home/tinyos/local/src/tinyos-2.x/tos/interfaces/Read.nc"
inline static void CC2420ControlP__ReadRssi__readDone(error_t result, CC2420ControlP__ReadRssi__val_t val){
#line 63
  CC2420ControlP__ReadRssi__default__readDone(result, val);
#line 63
}
#line 63
# 110 "/home/tinyos/local/src/tinyos-2.x/tos/interfaces/Resource.nc"
inline static error_t CC2420ControlP__RssiResource__release(void ){
#line 110
  unsigned char result;
#line 110

#line 110
  result = CC2420SpiP__Resource__release(/*CC2420ControlC.RssiResource*/CC2420SpiC__2__CLIENT_ID);
#line 110

#line 110
  return result;
#line 110
}
#line 110
# 287 "/home/tinyos/local/src/tinyos-2.x/tos/chips/cc2420/spi/CC2420SpiP.nc"
static inline cc2420_status_t CC2420SpiP__Reg__read(uint8_t addr, uint16_t *data)
#line 287
{

  cc2420_status_t status = 0;

  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 291
    {
      if (CC2420SpiP__WorkingState__isIdle()) {
          {
            unsigned char __nesc_temp = 
#line 293
            status;

            {
#line 293
              __nesc_atomic_end(__nesc_atomic); 
#line 293
              return __nesc_temp;
            }
          }
        }
    }
#line 297
    __nesc_atomic_end(__nesc_atomic); }
#line 297
  status = CC2420SpiP__SpiByte__write(addr | 0x40);
  *data = (uint16_t )CC2420SpiP__SpiByte__write(0) << 8;
  *data |= CC2420SpiP__SpiByte__write(0);

  return status;
}

# 47 "/home/tinyos/local/src/tinyos-2.x/tos/chips/cc2420/interfaces/CC2420Register.nc"
inline static cc2420_status_t CC2420ControlP__RSSI__read(uint16_t *data){
#line 47
  unsigned char result;
#line 47

#line 47
  result = CC2420SpiP__Reg__read(CC2420_RSSI, data);
#line 47

#line 47
  return result;
#line 47
}
#line 47
# 395 "/home/tinyos/local/src/tinyos-2.x/tos/chips/cc2420/control/CC2420ControlP.nc"
static inline void CC2420ControlP__RssiResource__granted(void )
#line 395
{
  uint16_t data = 0;

#line 397
  CC2420ControlP__CSN__clr();
  CC2420ControlP__RSSI__read(&data);
  CC2420ControlP__CSN__set();

  CC2420ControlP__RssiResource__release();
  data += 0x7f;
  data &= 0x00ff;
  CC2420ControlP__ReadRssi__readDone(SUCCESS, data);
}

# 414 "/home/tinyos/local/src/tinyos-2.x/tos/chips/cc2420/transmit/CC2420TransmitP.nc"
static inline void CC2420TransmitP__SpiResource__granted(void )
#line 414
{
  uint8_t cur_state;

  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 417
    {
      cur_state = CC2420TransmitP__m_state;
    }
#line 419
    __nesc_atomic_end(__nesc_atomic); }

  switch (cur_state) {
      case CC2420TransmitP__S_LOAD: 
        CC2420TransmitP__loadTXFIFO();
      break;

      case CC2420TransmitP__S_BEGIN_TRANSMIT: 
        CC2420TransmitP__attemptSend();
      break;

      case CC2420TransmitP__S_CANCEL: 
        CC2420TransmitP__CSN__clr();
      CC2420TransmitP__SFLUSHTX__strobe();
      CC2420TransmitP__CSN__set();
      CC2420TransmitP__releaseSpiResource();
      { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 435
        {
          CC2420TransmitP__m_state = CC2420TransmitP__S_STARTED;
        }
#line 437
        __nesc_atomic_end(__nesc_atomic); }
      CC2420TransmitP__Send__sendDone(CC2420TransmitP__m_msg, ECANCEL);
      break;

      default: 
        CC2420TransmitP__releaseSpiResource();
      break;
    }
}

# 508 "/home/tinyos/local/src/tinyos-2.x/tos/chips/cc2420/receive/CC2420ReceiveP.nc"
static inline void CC2420ReceiveP__SpiResource__granted(void )
#line 508
{







  CC2420ReceiveP__receive();
}

# 367 "/home/tinyos/local/src/tinyos-2.x/tos/chips/cc2420/spi/CC2420SpiP.nc"
static inline void CC2420SpiP__Resource__default__granted(uint8_t id)
#line 367
{
}

# 92 "/home/tinyos/local/src/tinyos-2.x/tos/interfaces/Resource.nc"
inline static void CC2420SpiP__Resource__granted(uint8_t arg_0x40bbc5c8){
#line 92
  switch (arg_0x40bbc5c8) {
#line 92
    case /*CC2420ControlC.Spi*/CC2420SpiC__0__CLIENT_ID:
#line 92
      CC2420ControlP__SpiResource__granted();
#line 92
      break;
#line 92
    case /*CC2420ControlC.SyncSpiC*/CC2420SpiC__1__CLIENT_ID:
#line 92
      CC2420ControlP__SyncResource__granted();
#line 92
      break;
#line 92
    case /*CC2420ControlC.RssiResource*/CC2420SpiC__2__CLIENT_ID:
#line 92
      CC2420ControlP__RssiResource__granted();
#line 92
      break;
#line 92
    case /*CC2420TransmitC.Spi*/CC2420SpiC__3__CLIENT_ID:
#line 92
      CC2420TransmitP__SpiResource__granted();
#line 92
      break;
#line 92
    case /*CC2420ReceiveC.Spi*/CC2420SpiC__4__CLIENT_ID:
#line 92
      CC2420ReceiveP__SpiResource__granted();
#line 92
      break;
#line 92
    default:
#line 92
      CC2420SpiP__Resource__default__granted(arg_0x40bbc5c8);
#line 92
      break;
#line 92
    }
#line 92
}
#line 92
# 358 "/home/tinyos/local/src/tinyos-2.x/tos/chips/cc2420/spi/CC2420SpiP.nc"
static inline void CC2420SpiP__grant__runTask(void )
#line 358
{
  uint8_t holder;

#line 360
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 360
    {
      holder = CC2420SpiP__m_holder;
    }
#line 362
    __nesc_atomic_end(__nesc_atomic); }
  CC2420SpiP__Resource__granted(holder);
}

# 55 "/home/tinyos/local/src/tinyos-2.x/tos/chips/cc2420/interfaces/CC2420Register.nc"
inline static cc2420_status_t CC2420ControlP__FSCTRL__write(uint16_t data){
#line 55
  unsigned char result;
#line 55

#line 55
  result = CC2420SpiP__Reg__write(CC2420_FSCTRL, data);
#line 55

#line 55
  return result;
#line 55
}
#line 55
inline static cc2420_status_t CC2420ControlP__MDMCTRL0__write(uint16_t data){
#line 55
  unsigned char result;
#line 55

#line 55
  result = CC2420SpiP__Reg__write(CC2420_MDMCTRL0, data);
#line 55

#line 55
  return result;
#line 55
}
#line 55
# 63 "/home/tinyos/local/src/tinyos-2.x/tos/chips/cc2420/interfaces/CC2420Ram.nc"
inline static cc2420_status_t CC2420ControlP__PANID__write(uint8_t offset, uint8_t * data, uint8_t length){
#line 63
  unsigned char result;
#line 63

#line 63
  result = CC2420SpiP__Ram__write(CC2420_RAM_PANID, offset, data, length);
#line 63

#line 63
  return result;
#line 63
}
#line 63
# 169 "/home/tinyos/local/src/tinyos-2.x/tos/chips/cc2420/CC2420Ieee154MessageP.nc"
static inline void CC2420Ieee154MessageP__CC2420Config__syncDone(error_t error)
#line 169
{
}

# 704 "/home/tinyos/local/src/tinyos-2.x/tos/chips/cc2420/receive/CC2420ReceiveP.nc"
static inline void CC2420ReceiveP__CC2420Config__syncDone(error_t error)
#line 704
{
}

# 53 "/home/tinyos/local/src/tinyos-2.x/tos/chips/cc2420/interfaces/CC2420Config.nc"
inline static void CC2420ControlP__CC2420Config__syncDone(error_t error){
#line 53
  CC2420ReceiveP__CC2420Config__syncDone(error);
#line 53
  CC2420Ieee154MessageP__CC2420Config__syncDone(error);
#line 53
}
#line 53
# 446 "/home/tinyos/local/src/tinyos-2.x/tos/chips/cc2420/control/CC2420ControlP.nc"
static inline void CC2420ControlP__syncDone__runTask(void )
#line 446
{
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 447
    CC2420ControlP__m_sync_busy = FALSE;
#line 447
    __nesc_atomic_end(__nesc_atomic); }
  CC2420ControlP__CC2420Config__syncDone(SUCCESS);
}

# 78 "/home/tinyos/local/src/tinyos-2.x/tos/interfaces/Resource.nc"
inline static error_t CC2420ControlP__SyncResource__request(void ){
#line 78
  unsigned char result;
#line 78

#line 78
  result = CC2420SpiP__Resource__request(/*CC2420ControlC.SyncSpiC*/CC2420SpiC__1__CLIENT_ID);
#line 78

#line 78
  return result;
#line 78
}
#line 78
# 300 "/home/tinyos/local/src/tinyos-2.x/tos/chips/cc2420/control/CC2420ControlP.nc"
static inline error_t CC2420ControlP__CC2420Config__sync(void )
#line 300
{
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 301
    {
      if (CC2420ControlP__m_sync_busy) {
          {
            unsigned char __nesc_temp = 
#line 303
            FAIL;

            {
#line 303
              __nesc_atomic_end(__nesc_atomic); 
#line 303
              return __nesc_temp;
            }
          }
        }
#line 306
      CC2420ControlP__m_sync_busy = TRUE;
      if (CC2420ControlP__m_state == CC2420ControlP__S_XOSC_STARTED) {
          CC2420ControlP__SyncResource__request();
        }
      else 
#line 309
        {
          CC2420ControlP__syncDone__postTask();
        }
    }
#line 312
    __nesc_atomic_end(__nesc_atomic); }
  return SUCCESS;
}

#line 442
static inline void CC2420ControlP__sync__runTask(void )
#line 442
{
  CC2420ControlP__CC2420Config__sync();
}

# 111 "/home/tinyos/local/src/tinyos-2.x/tos/chips/cc2420/link/PacketLinkP.nc"
static inline uint16_t PacketLinkP__PacketLink__getRetryDelay(message_t *msg)
#line 111
{
  return __nesc_ntoh_uint16(PacketLinkP__CC2420PacketBody__getMetadata(msg)->retryDelay.data);
}

# 62 "/home/tinyos/local/src/tinyos-2.x/tos/lib/timer/Timer.nc"
inline static void PacketLinkP__DelayTimer__startOneShot(uint32_t dt){
#line 62
  /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__Timer__startOneShot(2U, dt);
#line 62
}
#line 62
# 71 "/home/tinyos/local/src/tinyos-2.x/tos/interfaces/State.nc"
inline static uint8_t PacketLinkP__SendState__getState(void ){
#line 71
  unsigned char result;
#line 71

#line 71
  result = StateImplP__State__getState(4U);
#line 71

#line 71
  return result;
#line 71
}
#line 71
# 171 "/home/tinyos/local/src/tinyos-2.x/tos/chips/cc2420/link/PacketLinkP.nc"
static inline void PacketLinkP__SubSend__sendDone(message_t *msg, error_t error)
#line 171
{
  if (PacketLinkP__SendState__getState() == PacketLinkP__S_SENDING) {
      PacketLinkP__totalRetries++;
      if (PacketLinkP__PacketAcknowledgements__wasAcked(msg)) {
          PacketLinkP__signalDone(SUCCESS);
          return;
        }
      else {
#line 178
        if (PacketLinkP__totalRetries < PacketLinkP__PacketLink__getRetries(PacketLinkP__currentSendMsg)) {

            if (PacketLinkP__PacketLink__getRetryDelay(PacketLinkP__currentSendMsg) > 0) {

                PacketLinkP__DelayTimer__startOneShot(PacketLinkP__PacketLink__getRetryDelay(PacketLinkP__currentSendMsg));
              }
            else {

                PacketLinkP__send__postTask();
              }

            return;
          }
        }
    }
  PacketLinkP__signalDone(error);
}

# 89 "/home/tinyos/local/src/tinyos-2.x/tos/interfaces/Send.nc"
inline static void CC2420CsmaP__Send__sendDone(message_t * msg, error_t error){
#line 89
  PacketLinkP__SubSend__sendDone(msg, error);
#line 89
}
#line 89
# 241 "/home/tinyos/local/src/tinyos-2.x/tos/chips/cc2420/csma/CC2420CsmaP.nc"
static inline void CC2420CsmaP__sendDone_task__runTask(void )
#line 241
{
  error_t packetErr;

#line 243
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 243
    packetErr = CC2420CsmaP__sendErr;
#line 243
    __nesc_atomic_end(__nesc_atomic); }
  if (CC2420CsmaP__SplitControlState__isState(CC2420CsmaP__S_STOPPING)) {
      CC2420CsmaP__shutdown();
    }
  else {
      CC2420CsmaP__SplitControlState__forceState(CC2420CsmaP__S_STARTED);
    }

  CC2420CsmaP__Send__sendDone(CC2420CsmaP__m_msg, packetErr);
}

# 67 "/home/tinyos/local/src/tinyos-2.x/tos/lib/timer/Timer.nc"
inline static void PacketLinkP__DelayTimer__stop(void ){
#line 67
  /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__Timer__stop(2U);
#line 67
}
#line 67
# 86 "/home/tinyos/local/src/tinyos-2.x/tos/interfaces/Ieee154Send.nc"
inline static void ResourceSendP__Ieee154Send__sendDone(message_t *msg, error_t error){
#line 86
  IPDispatchP__Ieee154Send__sendDone(msg, error);
#line 86
}
#line 86
# 110 "/home/tinyos/local/src/tinyos-2.x/tos/interfaces/Resource.nc"
inline static error_t ResourceSendP__Resource__release(void ){
#line 110
  unsigned char result;
#line 110

#line 110
  result = CC2420TinyosNetworkP__Resource__release(0U);
#line 110

#line 110
  return result;
#line 110
}
#line 110
# 28 "/home/tinyos/local/src/tinyos-2.x/tos/lib/net/blip/ResourceSendP.nc"
static inline void ResourceSendP__SubSend__sendDone(message_t *msg, error_t result)
#line 28
{
  ResourceSendP__Resource__release();
  ResourceSendP__Ieee154Send__sendDone(msg, result);
  ResourceSendP__m_msg = (void *)0;
}

# 86 "/home/tinyos/local/src/tinyos-2.x/tos/interfaces/Ieee154Send.nc"
inline static void CC2420Ieee154MessageP__Ieee154Send__sendDone(message_t *msg, error_t error){
#line 86
  ResourceSendP__SubSend__sendDone(msg, error);
#line 86
}
#line 86
# 164 "/home/tinyos/local/src/tinyos-2.x/tos/chips/cc2420/CC2420Ieee154MessageP.nc"
static inline void CC2420Ieee154MessageP__SubSend__sendDone(message_t *msg, error_t result)
#line 164
{
  CC2420Ieee154MessageP__Ieee154Send__sendDone(msg, result);
}

# 89 "/home/tinyos/local/src/tinyos-2.x/tos/interfaces/Send.nc"
inline static void CC2420TinyosNetworkP__BareSend__sendDone(message_t * msg, error_t error){
#line 89
  CC2420Ieee154MessageP__SubSend__sendDone(msg, error);
#line 89
}
#line 89
# 219 "/home/tinyos/local/src/tinyos-2.x/tos/chips/cc2420/lowpan/CC2420TinyosNetworkP.nc"
static inline void CC2420TinyosNetworkP__ActiveSend__default__sendDone(message_t *msg, error_t error)
#line 219
{
}

# 89 "/home/tinyos/local/src/tinyos-2.x/tos/interfaces/Send.nc"
inline static void CC2420TinyosNetworkP__ActiveSend__sendDone(message_t * msg, error_t error){
#line 89
  CC2420TinyosNetworkP__ActiveSend__default__sendDone(msg, error);
#line 89
}
#line 89
# 118 "/home/tinyos/local/src/tinyos-2.x/tos/chips/cc2420/lowpan/CC2420TinyosNetworkP.nc"
static inline void CC2420TinyosNetworkP__SubSend__sendDone(message_t *msg, error_t error)
#line 118
{
  if (CC2420TinyosNetworkP__CC2420Packet__getNetwork(msg) == 0x3f) {
      CC2420TinyosNetworkP__ActiveSend__sendDone(msg, error);
    }
  else 
#line 121
    {
      CC2420TinyosNetworkP__BareSend__sendDone(msg, error);
    }
}

# 89 "/home/tinyos/local/src/tinyos-2.x/tos/interfaces/Send.nc"
inline static void UniqueSendP__Send__sendDone(message_t * msg, error_t error){
#line 89
  CC2420TinyosNetworkP__SubSend__sendDone(msg, error);
#line 89
}
#line 89
# 104 "/home/tinyos/local/src/tinyos-2.x/tos/chips/cc2420/unique/UniqueSendP.nc"
static inline void UniqueSendP__SubSend__sendDone(message_t *msg, error_t error)
#line 104
{
  UniqueSendP__State__toIdle();
  UniqueSendP__Send__sendDone(msg, error);
}

# 89 "/home/tinyos/local/src/tinyos-2.x/tos/interfaces/Send.nc"
inline static void PacketLinkP__Send__sendDone(message_t * msg, error_t error){
#line 89
  UniqueSendP__SubSend__sendDone(msg, error);
#line 89
}
#line 89
# 95 "TCPEchoP.nc"
static inline void TCPEchoP__RadioControl__stopDone(error_t e)
#line 95
{
}

# 117 "/home/tinyos/local/src/tinyos-2.x/tos/interfaces/SplitControl.nc"
inline static void IPDispatchP__SplitControl__stopDone(error_t error){
#line 117
  TCPEchoP__RadioControl__stopDone(error);
#line 117
}
#line 117
# 249 "/home/tinyos/local/src/tinyos-2.x/tos/lib/net/blip/IPDispatchP.nc"
static inline void IPDispatchP__RadioControl__stopDone(error_t error)
#line 249
{
  IPDispatchP__SplitControl__stopDone(error);
}

# 117 "/home/tinyos/local/src/tinyos-2.x/tos/interfaces/SplitControl.nc"
inline static void CC2420CsmaP__SplitControl__stopDone(error_t error){
#line 117
  IPDispatchP__RadioControl__stopDone(error);
#line 117
}
#line 117
# 262 "/home/tinyos/local/src/tinyos-2.x/tos/chips/cc2420/csma/CC2420CsmaP.nc"
static inline void CC2420CsmaP__stopDone_task__runTask(void )
#line 262
{
  CC2420CsmaP__SplitControlState__forceState(CC2420CsmaP__S_STOPPED);
  CC2420CsmaP__SplitControl__stopDone(SUCCESS);
}

# 91 "TCPEchoP.nc"
static inline void TCPEchoP__RadioControl__startDone(error_t e)
#line 91
{
}

# 92 "/home/tinyos/local/src/tinyos-2.x/tos/interfaces/SplitControl.nc"
inline static void IPDispatchP__SplitControl__startDone(error_t error){
#line 92
  TCPEchoP__RadioControl__startDone(error);
#line 92
}
#line 92
# 28 "/home/tinyos/local/src/tinyos-2.x/tos/lib/net/blip/interfaces/ICMP.nc"
inline static void IPDispatchP__ICMP__sendSolicitations(void ){
#line 28
  ICMPResponderP__ICMP__sendSolicitations();
#line 28
}
#line 28
# 238 "/home/tinyos/local/src/tinyos-2.x/tos/lib/net/blip/IPDispatchP.nc"
static inline void IPDispatchP__RadioControl__startDone(error_t error)
#line 238
{



  if (error == SUCCESS) {
      IPDispatchP__ICMP__sendSolicitations();
      IPDispatchP__state = IPDispatchP__S_RUNNING;
    }
  IPDispatchP__SplitControl__startDone(error);
}

# 92 "/home/tinyos/local/src/tinyos-2.x/tos/interfaces/SplitControl.nc"
inline static void CC2420CsmaP__SplitControl__startDone(error_t error){
#line 92
  IPDispatchP__RadioControl__startDone(error);
#line 92
}
#line 92
# 110 "/home/tinyos/local/src/tinyos-2.x/tos/interfaces/Resource.nc"
inline static error_t CC2420ControlP__SpiResource__release(void ){
#line 110
  unsigned char result;
#line 110

#line 110
  result = CC2420SpiP__Resource__release(/*CC2420ControlC.Spi*/CC2420SpiC__0__CLIENT_ID);
#line 110

#line 110
  return result;
#line 110
}
#line 110
# 179 "/home/tinyos/local/src/tinyos-2.x/tos/chips/cc2420/control/CC2420ControlP.nc"
static inline error_t CC2420ControlP__Resource__release(void )
#line 179
{
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 180
    {
      CC2420ControlP__CSN__set();
      {
        unsigned char __nesc_temp = 
#line 182
        CC2420ControlP__SpiResource__release();

        {
#line 182
          __nesc_atomic_end(__nesc_atomic); 
#line 182
          return __nesc_temp;
        }
      }
    }
#line 185
    __nesc_atomic_end(__nesc_atomic); }
}

# 110 "/home/tinyos/local/src/tinyos-2.x/tos/interfaces/Resource.nc"
inline static error_t CC2420CsmaP__Resource__release(void ){
#line 110
  unsigned char result;
#line 110

#line 110
  result = CC2420ControlP__Resource__release();
#line 110

#line 110
  return result;
#line 110
}
#line 110
# 249 "/home/tinyos/local/src/tinyos-2.x/tos/chips/cc2420/control/CC2420ControlP.nc"
static inline error_t CC2420ControlP__CC2420Power__rxOn(void )
#line 249
{
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 250
    {
      if (CC2420ControlP__m_state != CC2420ControlP__S_XOSC_STARTED) {
          {
            unsigned char __nesc_temp = 
#line 252
            FAIL;

            {
#line 252
              __nesc_atomic_end(__nesc_atomic); 
#line 252
              return __nesc_temp;
            }
          }
        }
#line 254
      CC2420ControlP__SRXON__strobe();
    }
#line 255
    __nesc_atomic_end(__nesc_atomic); }
  return SUCCESS;
}

# 90 "/home/tinyos/local/src/tinyos-2.x/tos/chips/cc2420/interfaces/CC2420Power.nc"
inline static error_t CC2420CsmaP__CC2420Power__rxOn(void ){
#line 90
  unsigned char result;
#line 90

#line 90
  result = CC2420ControlP__CC2420Power__rxOn();
#line 90

#line 90
  return result;
#line 90
}
#line 90
# 75 "/home/tinyos/local/src/tinyos-2.x/tos/chips/msp430/pins/HplMsp430InterruptP.nc"
static inline void HplMsp430InterruptP__Port10__enable(void )
#line 75
{
#line 75
  P1IE |= 1 << 0;
}

# 31 "/home/tinyos/local/src/tinyos-2.x/tos/chips/msp430/pins/HplMsp430Interrupt.nc"
inline static void /*HplCC2420InterruptsC.InterruptFIFOPC*/Msp430InterruptC__1__HplInterrupt__enable(void ){
#line 31
  HplMsp430InterruptP__Port10__enable();
#line 31
}
#line 31
# 107 "/home/tinyos/local/src/tinyos-2.x/tos/chips/msp430/pins/HplMsp430InterruptP.nc"
static inline void HplMsp430InterruptP__Port10__edge(bool l2h)
#line 107
{
  /* atomic removed: atomic calls only */
#line 108
  {
    if (l2h) {
#line 109
      P1IES &= ~(1 << 0);
      }
    else {
#line 110
      P1IES |= 1 << 0;
      }
  }
}

# 56 "/home/tinyos/local/src/tinyos-2.x/tos/chips/msp430/pins/HplMsp430Interrupt.nc"
inline static void /*HplCC2420InterruptsC.InterruptFIFOPC*/Msp430InterruptC__1__HplInterrupt__edge(bool low_to_high){
#line 56
  HplMsp430InterruptP__Port10__edge(low_to_high);
#line 56
}
#line 56
# 41 "/home/tinyos/local/src/tinyos-2.x/tos/chips/msp430/pins/Msp430InterruptC.nc"
static inline error_t /*HplCC2420InterruptsC.InterruptFIFOPC*/Msp430InterruptC__1__enable(bool rising)
#line 41
{
  /* atomic removed: atomic calls only */
#line 42
  {
    /*HplCC2420InterruptsC.InterruptFIFOPC*/Msp430InterruptC__1__Interrupt__disable();
    /*HplCC2420InterruptsC.InterruptFIFOPC*/Msp430InterruptC__1__HplInterrupt__edge(rising);
    /*HplCC2420InterruptsC.InterruptFIFOPC*/Msp430InterruptC__1__HplInterrupt__enable();
  }
  return SUCCESS;
}





static inline error_t /*HplCC2420InterruptsC.InterruptFIFOPC*/Msp430InterruptC__1__Interrupt__enableFallingEdge(void )
#line 54
{
  return /*HplCC2420InterruptsC.InterruptFIFOPC*/Msp430InterruptC__1__enable(FALSE);
}

# 43 "/home/tinyos/local/src/tinyos-2.x/tos/interfaces/GpioInterrupt.nc"
inline static error_t CC2420ReceiveP__InterruptFIFOP__enableFallingEdge(void ){
#line 43
  unsigned char result;
#line 43

#line 43
  result = /*HplCC2420InterruptsC.InterruptFIFOPC*/Msp430InterruptC__1__Interrupt__enableFallingEdge();
#line 43

#line 43
  return result;
#line 43
}
#line 43
# 157 "/home/tinyos/local/src/tinyos-2.x/tos/chips/cc2420/receive/CC2420ReceiveP.nc"
static inline error_t CC2420ReceiveP__StdControl__start(void )
#line 157
{
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 158
    {
      CC2420ReceiveP__reset_state();
      CC2420ReceiveP__m_state = CC2420ReceiveP__S_STARTED;
      CC2420ReceiveP__receivingPacket = FALSE;




      CC2420ReceiveP__InterruptFIFOP__enableFallingEdge();
    }
#line 167
    __nesc_atomic_end(__nesc_atomic); }
  return SUCCESS;
}

# 168 "/home/tinyos/local/src/tinyos-2.x/tos/chips/cc2420/transmit/CC2420TransmitP.nc"
static inline error_t CC2420TransmitP__StdControl__start(void )
#line 168
{
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 169
    {
      CC2420TransmitP__CaptureSFD__captureRisingEdge();
      CC2420TransmitP__m_state = CC2420TransmitP__S_STARTED;
      CC2420TransmitP__m_receiving = FALSE;
      CC2420TransmitP__abortSpiRelease = FALSE;
      CC2420TransmitP__m_tx_power = 0;
    }
#line 175
    __nesc_atomic_end(__nesc_atomic); }
  return SUCCESS;
}

# 74 "/home/tinyos/local/src/tinyos-2.x/tos/interfaces/StdControl.nc"
inline static error_t CC2420CsmaP__SubControl__start(void ){
#line 74
  unsigned char result;
#line 74

#line 74
  result = CC2420TransmitP__StdControl__start();
#line 74
  result = ecombine(result, CC2420ReceiveP__StdControl__start());
#line 74

#line 74
  return result;
#line 74
}
#line 74
# 254 "/home/tinyos/local/src/tinyos-2.x/tos/chips/cc2420/csma/CC2420CsmaP.nc"
static inline void CC2420CsmaP__startDone_task__runTask(void )
#line 254
{
  CC2420CsmaP__SubControl__start();
  CC2420CsmaP__CC2420Power__rxOn();
  CC2420CsmaP__Resource__release();
  CC2420CsmaP__SplitControlState__forceState(CC2420CsmaP__S_STARTED);
  CC2420CsmaP__SplitControl__startDone(SUCCESS);
}

# 92 "/home/tinyos/local/src/tinyos-2.x/tos/lib/timer/Alarm.nc"
inline static void /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__Alarm__startAt(/*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__Alarm__size_type t0, /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__Alarm__size_type dt){
#line 92
  /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__Alarm__startAt(t0, dt);
#line 92
}
#line 92
# 47 "/home/tinyos/local/src/tinyos-2.x/tos/lib/timer/AlarmToTimerC.nc"
static inline void /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__start(uint32_t t0, uint32_t dt, bool oneshot)
{
  /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__m_dt = dt;
  /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__m_oneshot = oneshot;
  /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__Alarm__startAt(t0, dt);
}

#line 82
static inline void /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__Timer__startOneShotAt(uint32_t t0, uint32_t dt)
{
#line 83
  /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__start(t0, dt, TRUE);
}

# 118 "/home/tinyos/local/src/tinyos-2.x/tos/lib/timer/Timer.nc"
inline static void /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__TimerFrom__startOneShotAt(uint32_t t0, uint32_t dt){
#line 118
  /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__Timer__startOneShotAt(t0, dt);
#line 118
}
#line 118
# 54 "/home/tinyos/local/src/tinyos-2.x/tos/chips/msp430/timer/Msp430AlarmC.nc"
static inline void /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__0__Alarm__stop(void )
{
  /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__0__Msp430TimerControl__disableEvents();
}

# 62 "/home/tinyos/local/src/tinyos-2.x/tos/lib/timer/Alarm.nc"
inline static void /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__AlarmFrom__stop(void ){
#line 62
  /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__0__Alarm__stop();
#line 62
}
#line 62
# 91 "/home/tinyos/local/src/tinyos-2.x/tos/lib/timer/TransformAlarmC.nc"
static inline void /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__Alarm__stop(void )
{
  /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__AlarmFrom__stop();
}

# 62 "/home/tinyos/local/src/tinyos-2.x/tos/lib/timer/Alarm.nc"
inline static void /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__Alarm__stop(void ){
#line 62
  /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__Alarm__stop();
#line 62
}
#line 62
# 60 "/home/tinyos/local/src/tinyos-2.x/tos/lib/timer/AlarmToTimerC.nc"
static inline void /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__Timer__stop(void )
{
#line 61
  /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__Alarm__stop();
}

# 67 "/home/tinyos/local/src/tinyos-2.x/tos/lib/timer/Timer.nc"
inline static void /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__TimerFrom__stop(void ){
#line 67
  /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__Timer__stop();
#line 67
}
#line 67
# 89 "/home/tinyos/local/src/tinyos-2.x/tos/lib/timer/VirtualizeTimerC.nc"
static inline void /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__updateFromTimer__runTask(void )
{




  uint32_t now = /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__TimerFrom__getNow();
  int32_t min_remaining = (1UL << 31) - 1;
  bool min_remaining_isset = FALSE;
  uint8_t num;

  /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__TimerFrom__stop();

  for (num = 0; num < /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__NUM_TIMERS; num++) 
    {
      /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__Timer_t *timer = &/*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__m_timers[num];

      if (timer->isrunning) 
        {
          uint32_t elapsed = now - timer->t0;
          int32_t remaining = timer->dt - elapsed;

          if (remaining < min_remaining) 
            {
              min_remaining = remaining;
              min_remaining_isset = TRUE;
            }
        }
    }

  if (min_remaining_isset) 
    {
      if (min_remaining <= 0) {
        /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__fireTimers(now);
        }
      else {
#line 124
        /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__TimerFrom__startOneShotAt(now, min_remaining);
        }
    }
}

# 16 "/home/tinyos/local/src/tinyos-2.x/tos/lib/net/blip/interfaces/UDP.nc"
inline static error_t TCPEchoP__Status__sendto(struct sockaddr_in6 *dest, void *payload, uint16_t len){
#line 16
  unsigned char result;
#line 16

#line 16
  result = UdpP__UDP__sendto(1U, dest, payload, len);
#line 16

#line 16
  return result;
#line 16
}
#line 16
# 189 "/home/tinyos/local/src/tinyos-2.x/tos/lib/net/blip/UdpP.nc"
static inline void UdpP__Statistics__get(udp_statistics_t *buf)
#line 189
{

  ip_memcpy(buf, &UdpP__stats, sizeof(udp_statistics_t ));
}

# 29 "/home/tinyos/local/src/tinyos-2.x/tos/lib/net/blip/interfaces/Statistics.nc"
inline static void TCPEchoP__UDPStats__get(TCPEchoP__UDPStats__stat_str *stats){
#line 29
  UdpP__Statistics__get(stats);
#line 29
}
#line 29
# 430 "/home/tinyos/local/src/tinyos-2.x/tos/lib/net/blip/ICMPResponderP.nc"
static inline void ICMPResponderP__Statistics__get(icmp_statistics_t *statistics)
#line 430
{
  ip_memcpy(statistics, &ICMPResponderP__stats, sizeof(icmp_statistics_t ));
}

# 29 "/home/tinyos/local/src/tinyos-2.x/tos/lib/net/blip/interfaces/Statistics.nc"
inline static void TCPEchoP__ICMPStats__get(TCPEchoP__ICMPStats__stat_str *stats){
#line 29
  ICMPResponderP__Statistics__get(stats);
#line 29
}
#line 29
# 1295 "/home/tinyos/local/src/tinyos-2.x/tos/lib/net/blip/IPRoutingP.nc"
static inline void IPRoutingP__Statistics__get(route_statistics_t *statistics)
#line 1295
{







  __nesc_hton_uint8(statistics->hop_limit.data, IPRoutingP__IPRouting__getHopLimit());
  __nesc_hton_uint16(statistics->parent.data, (uint16_t )IPRoutingP__default_route->neighbor);
  __nesc_hton_uint16(statistics->parent_metric.data, IPRoutingP__IPRouting__getQuality());
  __nesc_hton_uint16(statistics->parent_etx.data, IPRoutingP__getMetric(IPRoutingP__default_route));
}

# 29 "/home/tinyos/local/src/tinyos-2.x/tos/lib/net/blip/interfaces/Statistics.nc"
inline static void TCPEchoP__RouteStats__get(TCPEchoP__RouteStats__stat_str *stats){
#line 29
  IPRoutingP__Statistics__get(stats);
#line 29
}
#line 29
# 1047 "/home/tinyos/local/src/tinyos-2.x/tos/lib/net/blip/IPDispatchP.nc"
static inline void IPDispatchP__Statistics__get(ip_statistics_t *statistics)
#line 1047
{
#line 1061
  ip_memcpy(statistics, &IPDispatchP__stats, sizeof(ip_statistics_t ));
}

# 29 "/home/tinyos/local/src/tinyos-2.x/tos/lib/net/blip/interfaces/Statistics.nc"
inline static void TCPEchoP__IPStats__get(TCPEchoP__IPStats__stat_str *stats){
#line 29
  IPDispatchP__Statistics__get(stats);
#line 29
}
#line 29
# 53 "/home/tinyos/local/src/tinyos-2.x/tos/lib/timer/Timer.nc"
inline static void TCPEchoP__StatusTimer__startPeriodic(uint32_t dt){
#line 53
  /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__Timer__startPeriodic(0U, dt);
#line 53
}
#line 53
# 117 "TCPEchoP.nc"
static inline void TCPEchoP__StatusTimer__fired(void )
#line 117
{
  unsigned int __nesc_temp43;
  unsigned char *__nesc_temp42;

#line 119
  if (!TCPEchoP__timerStarted) {
      TCPEchoP__StatusTimer__startPeriodic(1024 * 75L);
      TCPEchoP__timerStarted = TRUE;
    }

  (__nesc_temp42 = TCPEchoP__stats.seqno.data, __nesc_hton_uint16(__nesc_temp42, (__nesc_temp43 = __nesc_ntoh_uint16(__nesc_temp42)) + 1), __nesc_temp43);
  __nesc_hton_uint16(TCPEchoP__stats.sender.data, TOS_NODE_ID);

  TCPEchoP__IPStats__get(& TCPEchoP__stats.ip);
  TCPEchoP__RouteStats__get(& TCPEchoP__stats.route);
  TCPEchoP__ICMPStats__get(& TCPEchoP__stats.icmp);
  TCPEchoP__UDPStats__get(& TCPEchoP__stats.udp);

  TCPEchoP__Status__sendto(&TCPEchoP__route_dest, &TCPEchoP__stats, sizeof TCPEchoP__stats);
}

# 202 "/home/tinyos/local/src/tinyos-2.x/tos/chips/cc2420/link/PacketLinkP.nc"
static inline void PacketLinkP__DelayTimer__fired(void )
#line 202
{
  if (PacketLinkP__SendState__getState() == PacketLinkP__S_SENDING) {
      PacketLinkP__send__postTask();
    }
}

# 347 "/home/tinyos/local/src/tinyos-2.x/tos/lib/net/blip/IPDispatchP.nc"
static inline void IPDispatchP__ip_print_heap(void )
#line 347
{
}

#line 331
static inline void IPDispatchP__forward_age(void *elt)
#line 331
{
  forward_entry_t *fwd = (forward_entry_t *)elt;

#line 333
  switch (fwd->timeout) {
      case T_ACTIVE: 
        fwd->timeout = T_ZOMBIE;
#line 335
      break;
      case T_FAILED1: 
        fwd->timeout = T_FAILED2;
#line 337
      break;
      case T_ZOMBIE: 
        case T_FAILED2: 
          fwd->s_info->failed = TRUE;
      if (-- fwd->s_info->refcount == 0) {
#line 341
        IPDispatchP__SendInfoPool__put(fwd->s_info);
        }
#line 342
      fwd->timeout = T_UNUSED;
      break;
    }
}

#line 312
static inline void IPDispatchP__reconstruct_age(void *elt)
#line 312
{
  reconstruct_t *recon = (reconstruct_t *)elt;

#line 314
  switch (recon->timeout) {
      case T_ACTIVE: 
        recon->timeout = T_ZOMBIE;
#line 316
      break;
      case T_FAILED1: 
        recon->timeout = T_FAILED2;
#line 318
      break;
      case T_ZOMBIE: 
        case T_FAILED2: 

          if (recon->buf != (void *)0) {
              ip_free(recon->buf);
            }
      recon->timeout = T_UNUSED;
      recon->buf = (void *)0;
      break;
    }
}

#line 358
static inline void IPDispatchP__ExpireTimer__fired(void )
#line 358
{
  table_map(&IPDispatchP__recon_cache, IPDispatchP__reconstruct_age);
  table_map(&IPDispatchP__forward_cache, IPDispatchP__forward_age);







  IPDispatchP__ip_print_heap();
}

# 28 "/home/tinyos/local/src/tinyos-2.x/tos/lib/net/blip/interfaces/ICMP.nc"
inline static void IPRoutingP__ICMP__sendSolicitations(void ){
#line 28
  ICMPResponderP__ICMP__sendSolicitations();
#line 28
}
#line 28
# 1282 "/home/tinyos/local/src/tinyos-2.x/tos/lib/net/blip/IPRoutingP.nc"
static inline void IPRoutingP__ICMP__solicitationDone(void )
#line 1282
{


  ;

  IPRoutingP__soliciting = FALSE;

  if (!IPRoutingP__IPRouting__hasRoute()) {
      IPRoutingP__ICMP__sendSolicitations();
      IPRoutingP__soliciting = TRUE;
    }
}

# 1040 "/home/tinyos/local/src/tinyos-2.x/tos/lib/net/blip/IPDispatchP.nc"
static inline void IPDispatchP__ICMP__solicitationDone(void )
#line 1040
{
}

# 31 "/home/tinyos/local/src/tinyos-2.x/tos/lib/net/blip/interfaces/ICMP.nc"
inline static void ICMPResponderP__ICMP__solicitationDone(void ){
#line 31
  IPDispatchP__ICMP__solicitationDone();
#line 31
  IPRoutingP__ICMP__solicitationDone();
#line 31
}
#line 31
# 29 "/home/tinyos/local/src/tinyos-2.x/tos/lib/net/blip/interfaces/IPAddress.nc"
inline static void ICMPResponderP__IPAddress__getLLAddr(struct in6_addr *addr){
#line 29
  IPAddressP__IPAddress__getLLAddr(addr);
#line 29
}
#line 29
# 150 "/home/tinyos/local/src/tinyos-2.x/tos/lib/net/blip/ICMPResponderP.nc"
static inline void ICMPResponderP__sendSolicitation(void )
#line 150
{
  unsigned char __nesc_temp71;
  unsigned char *__nesc_temp70;
#line 151
  struct split_ip_msg *ipmsg = (struct split_ip_msg *)ip_malloc(sizeof(struct split_ip_msg ) + sizeof(rsol_t ));
  rsol_t *msg = (rsol_t *)(ipmsg + 1);

  if (ipmsg == (void *)0) {
#line 154
    return;
    }
  (__nesc_temp70 = ICMPResponderP__stats.sol_tx.data, __nesc_hton_uint8(__nesc_temp70, (__nesc_temp71 = __nesc_ntoh_uint8(__nesc_temp70)) + 1), __nesc_temp71);

  __nesc_hton_uint8(msg->type.data, ICMP_TYPE_ROUTER_SOL);
  __nesc_hton_uint8(msg->code.data, 0);
  __nesc_hton_uint16(msg->cksum.data, 0);
  __nesc_hton_uint32(msg->reserved.data, 0);

  ipmsg->headers = (void *)0;
  ipmsg->data = (void *)msg;
  ipmsg->data_len = sizeof(rsol_t );


  ipmsg->hdr.hlim = 0xff;


  ICMPResponderP__IPAddress__getLLAddr(& ipmsg->hdr.ip6_src);
  ip_memclr((uint8_t *)& ipmsg->hdr.ip6_dst, 16);
  ipmsg->hdr.ip6_dst.in6_u.u6_addr16[0] = (((uint16_t )0xff02 << 8) | ((uint16_t )0xff02 >> 8)) & 0xffff;
  ipmsg->hdr.ip6_dst.in6_u.u6_addr16[7] = (((uint16_t )2 << 8) | ((uint16_t )2 >> 8)) & 0xffff;

  __nesc_hton_uint16(msg->cksum.data, ICMPResponderP__ICMP__cksum(ipmsg, IANA_ICMP));

  ICMPResponderP__IP__send(ipmsg);

  ip_free(ipmsg);
}

#line 383
static inline void ICMPResponderP__Solicitation__fired(void )
#line 383
{
  ICMPResponderP__sendSolicitation();
  ;
  ICMPResponderP__solicitation_period <<= 1;
  if (ICMPResponderP__solicitation_period < TRICKLE_MAX) {
      ICMPResponderP__Solicitation__startOneShot(ICMPResponderP__solicitation_period);
    }
  else 
#line 389
    {
      ICMPResponderP__ICMP__solicitationDone();
    }
}

# 60 "/home/tinyos/local/src/tinyos-2.x/tos/lib/net/blip/interfaces/IPRouting.nc"
inline static uint16_t ICMPResponderP__IPRouting__getQuality(void ){
#line 60
  unsigned int result;
#line 60

#line 60
  result = IPRoutingP__IPRouting__getQuality();
#line 60

#line 60
  return result;
#line 60
}
#line 60
# 28 "/home/tinyos/local/src/tinyos-2.x/tos/lib/net/blip/interfaces/IPAddress.nc"
inline static struct in6_addr *ICMPResponderP__IPAddress__getPublicAddr(void ){
#line 28
  struct in6_addr *result;
#line 28

#line 28
  result = IPAddressP__IPAddress__getPublicAddr();
#line 28

#line 28
  return result;
#line 28
}
#line 28
# 58 "/home/tinyos/local/src/tinyos-2.x/tos/lib/net/blip/interfaces/IPRouting.nc"
inline static uint8_t ICMPResponderP__IPRouting__getHopLimit(void ){
#line 58
  unsigned char result;
#line 58

#line 58
  result = IPRoutingP__IPRouting__getHopLimit();
#line 58

#line 58
  return result;
#line 58
}
#line 58
# 254 "/home/tinyos/local/src/tinyos-2.x/tos/lib/net/blip/ICMPResponderP.nc"
static inline void ICMPResponderP__sendAdvertisement(void )
#line 254
{
  unsigned char __nesc_temp73;
  unsigned char *__nesc_temp72;
#line 255
  struct split_ip_msg *ipmsg = (struct split_ip_msg *)ip_malloc(sizeof(struct split_ip_msg ) + 
  sizeof(radv_t ) + 
  sizeof(pfx_t ) + 
  sizeof(rqual_t ));
  uint16_t len = sizeof(radv_t );
  radv_t *r = (radv_t *)(ipmsg + 1);
  pfx_t *p = (pfx_t *)r->options;
  rqual_t *q = (rqual_t *)(p + 1);

  if (ipmsg == (void *)0) {
#line 264
    return;
    }
  if (!ICMPResponderP__IPRouting__hasRoute()) {
      ip_free(ipmsg);
      return;
    }
  (__nesc_temp72 = ICMPResponderP__stats.adv_tx.data, __nesc_hton_uint8(__nesc_temp72, (__nesc_temp73 = __nesc_ntoh_uint8(__nesc_temp72)) + 1), __nesc_temp73);

  __nesc_hton_uint8(r->type.data, ICMP_TYPE_ROUTER_ADV);
  __nesc_hton_uint8(r->code.data, 0);
  __nesc_hton_uint8(r->hlim.data, ICMPResponderP__IPRouting__getHopLimit());
  __nesc_hton_uint8(r->flags.data, 0);
  __nesc_hton_uint16(r->lifetime.data, 1);
  __nesc_hton_uint32(r->reachable_time.data, 0);
  __nesc_hton_uint32(r->retrans_time.data, 0);

  ipmsg->hdr.hlim = 0xff;

  if (globalPrefix) {
      len += sizeof(pfx_t );
      __nesc_hton_uint8(p->type.data, ICMP_EXT_TYPE_PREFIX);
      __nesc_hton_uint8(p->length.data, sizeof(pfx_t ) >> 3);
      __nesc_hton_uint8(p->pfx_len.data, 64);
      ip_memcpy(p->prefix, ICMPResponderP__IPAddress__getPublicAddr(), 8);
    }

  len += sizeof(rqual_t );
  __nesc_hton_uint8(q->type.data, ICMP_EXT_TYPE_BEACON);
  __nesc_hton_uint8(q->length.data, sizeof(rqual_t ) >> 3);
#line 292
  ;
  __nesc_hton_uint16(q->metric.data, ICMPResponderP__IPRouting__getQuality());
  __nesc_hton_uint16(q->seqno.data, ICMPResponderP__nd_seqno);

  ICMPResponderP__IPAddress__getLLAddr(& ipmsg->hdr.ip6_src);
  ip_memclr((uint8_t *)& ipmsg->hdr.ip6_dst, 16);
  ipmsg->hdr.ip6_dst.in6_u.u6_addr16[0] = (((uint16_t )0xff02 << 8) | ((uint16_t )0xff02 >> 8)) & 0xffff;
  ipmsg->hdr.ip6_dst.in6_u.u6_addr16[7] = (((uint16_t )1 << 8) | ((uint16_t )1 >> 8)) & 0xffff;


  ;

  if (__nesc_ntoh_uint8(r->hlim.data) >= 0xf0) {
      ip_free(ipmsg);
      return;
    }

  ipmsg->data = (void *)r;
  ipmsg->data_len = len;
  ipmsg->headers = (void *)0;

  __nesc_hton_uint16(r->cksum.data, 0);
  __nesc_hton_uint16(r->cksum.data, ICMPResponderP__ICMP__cksum(ipmsg, IANA_ICMP));

  ICMPResponderP__IP__send(ipmsg);
  ip_free(ipmsg);
}

#line 394
static inline void ICMPResponderP__Advertisement__fired(void )
#line 394
{
  ;
  ICMPResponderP__sendAdvertisement();
  ICMPResponderP__advertisement_period <<= 1;
  if (ICMPResponderP__advertisement_period < TRICKLE_MAX) {
      ICMPResponderP__Advertisement__startOneShot(ICMPResponderP__advertisement_period);
    }
}

#line 183
static inline void ICMPResponderP__sendPing(struct in6_addr *dest, uint16_t seqno)
#line 183
{
  struct split_ip_msg *ipmsg = (struct split_ip_msg *)ip_malloc(sizeof(struct split_ip_msg ) + 
  sizeof(icmp_echo_hdr_t ) + 
  sizeof(nx_uint32_t ));
  icmp_echo_hdr_t *e_hdr = (icmp_echo_hdr_t *)ipmsg->next;
  nx_uint32_t *sendTime = (nx_uint32_t *)(e_hdr + 1);

  if (ipmsg == (void *)0) {
#line 190
    return;
    }
#line 191
  ipmsg->headers = (void *)0;
  ipmsg->data = (void *)e_hdr;
  ipmsg->data_len = sizeof(icmp_echo_hdr_t ) + sizeof(nx_uint32_t );

  __nesc_hton_uint8(e_hdr->type.data, ICMP_TYPE_ECHO_REQUEST);
  __nesc_hton_uint8(e_hdr->code.data, 0);
  __nesc_hton_uint16(e_hdr->cksum.data, 0);
  __nesc_hton_uint16(e_hdr->ident.data, ICMPResponderP__ping_ident);
  __nesc_hton_uint16(e_hdr->seqno.data, seqno);
  __nesc_hton_uint32((*sendTime).data, ICMPResponderP__LocalTime__get());

  ip_memcpy(& ipmsg->hdr.ip6_dst, dest->in6_u.u6_addr8, 16);
  ICMPResponderP__IPAddress__getIPAddr(& ipmsg->hdr.ip6_src);

  __nesc_hton_uint16(e_hdr->cksum.data, ICMPResponderP__ICMP__cksum(ipmsg, IANA_ICMP));

  ICMPResponderP__IP__send(ipmsg);
  ip_free(ipmsg);
}

# 67 "/home/tinyos/local/src/tinyos-2.x/tos/lib/timer/Timer.nc"
inline static void ICMPResponderP__PingTimer__stop(void ){
#line 67
  /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__Timer__stop(6U);
#line 67
}
#line 67
# 272 "/home/tinyos/local/src/tinyos-2.x/tos/lib/net/blip/shell/UDPShellP.nc"
static inline void UDPShellP__ICMPPing__pingDone(uint16_t ping_rcv, uint16_t ping_n)
#line 272
{
  int len;

#line 274
  len = snprintf(UDPShellP__reply_buf, MAX_REPLY_LEN, UDPShellP__ping_summary, ping_n, ping_rcv);
  UDPShellP__UDP__sendto(&UDPShellP__session_endpoint, UDPShellP__reply_buf, len);
}

# 442 "/home/tinyos/local/src/tinyos-2.x/tos/lib/net/blip/ICMPResponderP.nc"
static inline void ICMPResponderP__ICMPPing__default__pingDone(uint16_t client, uint16_t n, uint16_t m)
#line 442
{
}

# 10 "/home/tinyos/local/src/tinyos-2.x/tos/lib/net/blip/interfaces/ICMPPing.nc"
inline static void ICMPResponderP__ICMPPing__pingDone(uint16_t arg_0x412fc2e0, uint16_t ping_rcv, uint16_t ping_n){
#line 10
  switch (arg_0x412fc2e0) {
#line 10
    case 0U:
#line 10
      UDPShellP__ICMPPing__pingDone(ping_rcv, ping_n);
#line 10
      break;
#line 10
    default:
#line 10
      ICMPResponderP__ICMPPing__default__pingDone(arg_0x412fc2e0, ping_rcv, ping_n);
#line 10
      break;
#line 10
    }
#line 10
}
#line 10
# 417 "/home/tinyos/local/src/tinyos-2.x/tos/lib/net/blip/ICMPResponderP.nc"
static inline void ICMPResponderP__PingTimer__fired(void )
#line 417
{

  if (ICMPResponderP__ping_seq == ICMPResponderP__ping_n) {
      ICMPResponderP__ICMPPing__pingDone(ICMPResponderP__ping_ident, ICMPResponderP__ping_rcv, ICMPResponderP__ping_n);
      ICMPResponderP__PingTimer__stop();
      return;
    }
  ICMPResponderP__sendPing(&ICMPResponderP__ping_dest, ICMPResponderP__ping_seq);
  ICMPResponderP__ping_seq++;
}

# 15 "/home/tinyos/local/src/tinyos-2.x/tos/lib/net/blip/interfaces/IP.nc"
inline static error_t IPRoutingP__TGenSend__send(struct split_ip_msg *msg){
#line 15
  unsigned char result;
#line 15

#line 15
  result = IPDispatchP__IP__send(IPV6_NONEXT, msg);
#line 15

#line 15
  return result;
#line 15
}
#line 15
# 30 "/home/tinyos/local/src/tinyos-2.x/tos/lib/net/blip/interfaces/IPAddress.nc"
inline static void IPRoutingP__IPAddress__getIPAddr(struct in6_addr *addr){
#line 30
  IPAddressP__IPAddress__getIPAddr(addr);
#line 30
}
#line 30
# 120 "/home/tinyos/local/src/tinyos-2.x/tos/lib/net/blip/IPRoutingP.nc"
static inline void IPRoutingP__TrafficGenTimer__fired(void )
#line 120
{
  struct split_ip_msg *msg;

#line 122
  if (IPRoutingP__traffic_sent) {
#line 122
    goto done;
    }
#line 123
  msg = (struct split_ip_msg *)ip_malloc(sizeof(struct split_ip_msg ));
  if (msg == (void *)0) {
      ;
      goto done;
    }
  IPRoutingP__traffic_sent = FALSE;

  ip_memclr((uint8_t *)& msg->hdr, sizeof(struct ip6_hdr ));
  inet_pton6("ff05::1", & msg->hdr.ip6_dst);
  IPRoutingP__IPAddress__getIPAddr(& msg->hdr.ip6_src);
  msg->data = (void *)0;
  msg->data_len = 0;
  msg->headers = (void *)0;

  ;
  IPRoutingP__TGenSend__send(msg);
  ip_free(msg);
  done: 

    ;
  IPRoutingP__traffic_sent = FALSE;
  IPRoutingP__traffic_interval *= 2;
  if (IPRoutingP__traffic_interval > TGEN_MAX_INTERVAL) {
    IPRoutingP__traffic_interval = TGEN_MAX_INTERVAL;
    }
#line 147
  IPRoutingP__TrafficGenTimer__startOneShot(IPRoutingP__traffic_interval);
}

# 41 "/home/tinyos/local/src/tinyos-2.x/tos/interfaces/Random.nc"
inline static uint16_t IPRoutingP__Random__rand16(void ){
#line 41
  unsigned int result;
#line 41

#line 41
  result = RandomMlcgC__Random__rand16();
#line 41

#line 41
  return result;
#line 41
}
#line 41
# 1344 "/home/tinyos/local/src/tinyos-2.x/tos/lib/net/blip/IPRoutingP.nc"
static inline void IPRoutingP__updateRankings(void )
#line 1344
{
  uint8_t i;
  bool evicted = FALSE;

  for (i = 0; i < N_NEIGH; i++) {
      IPRoutingP__neigh_table[i].flags &= ~T_EVICT_MASK;
      if (!(((&IPRoutingP__neigh_table[i])->flags & T_VALID_MASK) == T_VALID_MASK)) {
#line 1350
        continue;
        }
#line 1351
      IPRoutingP__neigh_table[i].stats[IPRoutingP__LONG_EPOCH].total += IPRoutingP__neigh_table[i].stats[IPRoutingP__SHORT_EPOCH].total;
      IPRoutingP__neigh_table[i].stats[IPRoutingP__LONG_EPOCH].receptions += IPRoutingP__neigh_table[i].stats[IPRoutingP__SHORT_EPOCH].receptions;
      IPRoutingP__neigh_table[i].stats[IPRoutingP__LONG_EPOCH].success += IPRoutingP__neigh_table[i].stats[IPRoutingP__SHORT_EPOCH].success;

      if (IPRoutingP__neigh_table[i].stats[IPRoutingP__LONG_EPOCH].total & 0xf000) {


          IPRoutingP__neigh_table[i].stats[IPRoutingP__LONG_EPOCH].total >>= 1;
          IPRoutingP__neigh_table[i].stats[IPRoutingP__LONG_EPOCH].success >>= 1;
        }

      if (IPRoutingP__neigh_table[i].stats[IPRoutingP__LONG_EPOCH].total > CONF_EVICT_THRESHOLD) {
        (&IPRoutingP__neigh_table[i])->flags |= T_MATURE_MASK;
        }
      if (((&IPRoutingP__neigh_table[i])->flags & T_MATURE_MASK) == T_MATURE_MASK) {
          uint16_t cost;

          if (IPRoutingP__neigh_table[i].stats[IPRoutingP__SHORT_EPOCH].total == 0) {
#line 1368
            goto done_iter;
            }
#line 1369
          if (IPRoutingP__neigh_table[i].stats[IPRoutingP__SHORT_EPOCH].success == 0) {
              cost = 0xff;
            }
          else 
#line 1371
            {
              cost = 10 * IPRoutingP__neigh_table[i].stats[IPRoutingP__SHORT_EPOCH].total / 
              IPRoutingP__neigh_table[i].stats[IPRoutingP__SHORT_EPOCH].success;
            }
          if (cost > LINK_EVICT_THRESH) {
              ;
              IPRoutingP__neigh_table[i].flags |= T_EVICT_MASK;
            }
        }
      done_iter: 
        IPRoutingP__neigh_table[i].stats[IPRoutingP__SHORT_EPOCH].total = 0;
      IPRoutingP__neigh_table[i].stats[IPRoutingP__SHORT_EPOCH].receptions = 0;
      IPRoutingP__neigh_table[i].stats[IPRoutingP__SHORT_EPOCH].success = 0;
    }
  for (i = 0; i < N_NEIGH; i++) {
      if (((&IPRoutingP__neigh_table[i])->flags & T_VALID_MASK) == T_VALID_MASK && 
      IPRoutingP__neigh_table[i].flags & T_EVICT_MASK) {





          ;
          IPRoutingP__evictNeighbor(&IPRoutingP__neigh_table[i]);
          i--;

          evicted = TRUE;
        }
    }
  if (evicted) {
    IPRoutingP__ICMP__sendSolicitations();
    }
}

# 33 "/home/tinyos/local/src/tinyos-2.x/tos/lib/net/blip/interfaces/ICMP.nc"
inline static void IPRoutingP__ICMP__sendAdvertisements(void ){
#line 33
  ICMPResponderP__ICMP__sendAdvertisements();
#line 33
}
#line 33
# 1247 "/home/tinyos/local/src/tinyos-2.x/tos/lib/net/blip/IPRoutingP.nc"
static inline void IPRoutingP__SortTimer__fired(void )
#line 1247
{
  ;
  IPRoutingP__printTable();

  if (!IPRoutingP__IPRouting__hasRoute() && !IPRoutingP__soliciting) {
      IPRoutingP__ICMP__sendSolicitations();
      IPRoutingP__soliciting = TRUE;
    }

  if (IPRoutingP__checkThresh(IPRoutingP__IPRouting__getQuality(), IPRoutingP__last_qual, 5) != WITHIN_THRESH || 
  IPRoutingP__last_hops != IPRoutingP__IPRouting__getHopLimit()) {
      IPRoutingP__ICMP__sendAdvertisements();
      IPRoutingP__last_qual = IPRoutingP__IPRouting__getQuality();
      IPRoutingP__last_hops = IPRoutingP__IPRouting__getHopLimit();
    }

  IPRoutingP__updateRankings();

  if (IPRoutingP__Random__rand16() % 32 < 8) {
      ;
      IPRoutingP__chooseNewRandomDefault(FALSE);
    }
  else 
#line 1268
    {

      IPRoutingP__default_route_failures = 0;
    }
}

# 608 "/home/tinyos/local/src/tinyos-2.x/support/sdk/c/blip/libtcp/tcplib.c"
static inline void TcpP__tcplib_retx_expire(struct TcpP__tcplib_sock *sock)
#line 608
{

  sock->retxcnt++;
  switch (sock->state) {
      case TcpP__TCP_ESTABLISHED: 
        if (TcpP__circ_get_seqno(sock->tx_buf) != sock->seqno) {
            ;

            TcpP__reset_ssthresh(sock);

            sock->cwnd = sock->mss;

            TcpP__tcplib_output(sock, TcpP__circ_get_seqno(sock->tx_buf));
            sock->timer.retx = 6;
          }
        else 
#line 622
          {
            sock->retxcnt--;
          }
      break;
      case TcpP__TCP_SYN_SENT: 
        TcpP__tcplib_send_ack(sock, 0, TCP_FLAG_SYN);
      sock->timer.retx = 6;
      break;
      case TcpP__TCP_LAST_ACK: 
        case TcpP__TCP_FIN_WAIT_1: 
          TcpP__tcplib_send_ack(sock, 1, TCP_FLAG_ACK | TCP_FLAG_FIN);
      sock->timer.retx = TcpP__TCPLIB_2MSL;
      break;
      case TcpP__TCP_FIN_WAIT_2: 
        case TcpP__TCP_TIME_WAIT: 
          sock->state = TcpP__TCP_CLOSED;

      TcpP__tcplib_extern_closedone(sock);
      break;
      default: 
        break;
    }










  if (sock->retxcnt > TcpP__TCPLIB_GIVEUP) {
      sock->state = TcpP__TCP_TIME_WAIT;
      sock->timer.retx = TcpP__TCPLIB_TIMEWAIT_LEN;
    }
}

#line 702
static inline int TcpP__tcplib_timer_process(void )
#line 702
{
  struct TcpP__tcplib_sock *iter;

#line 704
  for (iter = TcpP__conns; iter != (void *)0; iter = iter->next) {
      if (iter->timer.retx > 0 && -- iter->timer.retx == 0) {
        TcpP__tcplib_retx_expire(iter);
        }
#line 707
      if ((iter->flags & TcpP__TCP_ACKPENDING) >= 2) {
          TcpP__tcplib_send_ack(iter, 0, TCP_FLAG_ACK);
        }
    }
  return 0;
}

# 100 "/home/tinyos/local/src/tinyos-2.x/tos/lib/net/blip/TcpP.nc"
static inline void TcpP__Timer__fired(void )
#line 100
{
  TcpP__tcplib_timer_process();
}

# 56 "/home/tinyos/local/src/tinyos-2.x/tos/interfaces/TaskBasic.nc"
inline static error_t /*HplSensirionSht11C.Arbiter.Arbiter*/ArbiterP__1__grantedTask__postTask(void ){
#line 56
  unsigned char result;
#line 56

#line 56
  result = SchedulerBasicP__TaskBasic__postTask(/*HplSensirionSht11C.Arbiter.Arbiter*/ArbiterP__1__grantedTask);
#line 56

#line 56
  return result;
#line 56
}
#line 56
# 130 "/home/tinyos/local/src/tinyos-2.x/tos/system/ArbiterP.nc"
static inline error_t /*HplSensirionSht11C.Arbiter.Arbiter*/ArbiterP__1__ResourceDefaultOwner__release(void )
#line 130
{
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 131
    {
      if (/*HplSensirionSht11C.Arbiter.Arbiter*/ArbiterP__1__resId == /*HplSensirionSht11C.Arbiter.Arbiter*/ArbiterP__1__default_owner_id) {
          if (/*HplSensirionSht11C.Arbiter.Arbiter*/ArbiterP__1__state == /*HplSensirionSht11C.Arbiter.Arbiter*/ArbiterP__1__RES_GRANTING) {
              /*HplSensirionSht11C.Arbiter.Arbiter*/ArbiterP__1__grantedTask__postTask();
              {
                unsigned char __nesc_temp = 
#line 135
                SUCCESS;

                {
#line 135
                  __nesc_atomic_end(__nesc_atomic); 
#line 135
                  return __nesc_temp;
                }
              }
            }
          else {
#line 137
            if (/*HplSensirionSht11C.Arbiter.Arbiter*/ArbiterP__1__state == /*HplSensirionSht11C.Arbiter.Arbiter*/ArbiterP__1__RES_IMM_GRANTING) {
                /*HplSensirionSht11C.Arbiter.Arbiter*/ArbiterP__1__resId = /*HplSensirionSht11C.Arbiter.Arbiter*/ArbiterP__1__reqResId;
                /*HplSensirionSht11C.Arbiter.Arbiter*/ArbiterP__1__state = /*HplSensirionSht11C.Arbiter.Arbiter*/ArbiterP__1__RES_BUSY;
                {
                  unsigned char __nesc_temp = 
#line 140
                  SUCCESS;

                  {
#line 140
                    __nesc_atomic_end(__nesc_atomic); 
#line 140
                    return __nesc_temp;
                  }
                }
              }
            }
        }
    }
#line 146
    __nesc_atomic_end(__nesc_atomic); }
#line 144
  return FAIL;
}

# 56 "/home/tinyos/local/src/tinyos-2.x/tos/interfaces/ResourceDefaultOwner.nc"
inline static error_t /*HplSensirionSht11C.SplitControlPowerManagerC.PowerManager*/PowerManagerP__0__ResourceDefaultOwner__release(void ){
#line 56
  unsigned char result;
#line 56

#line 56
  result = /*HplSensirionSht11C.Arbiter.Arbiter*/ArbiterP__1__ResourceDefaultOwner__release();
#line 56

#line 56
  return result;
#line 56
}
#line 56
# 181 "/home/tinyos/local/src/tinyos-2.x/tos/system/ArbiterP.nc"
static inline uint8_t /*HplSensirionSht11C.Arbiter.Arbiter*/ArbiterP__1__ResourceDefaultOwner__isOwner(void )
#line 181
{
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 182
    {
      unsigned char __nesc_temp = 
#line 182
      /*HplSensirionSht11C.Arbiter.Arbiter*/ArbiterP__1__state == /*HplSensirionSht11C.Arbiter.Arbiter*/ArbiterP__1__RES_CONTROLLED
       || (/*HplSensirionSht11C.Arbiter.Arbiter*/ArbiterP__1__resId == /*HplSensirionSht11C.Arbiter.Arbiter*/ArbiterP__1__default_owner_id
       && (/*HplSensirionSht11C.Arbiter.Arbiter*/ArbiterP__1__state == /*HplSensirionSht11C.Arbiter.Arbiter*/ArbiterP__1__RES_GRANTING || /*HplSensirionSht11C.Arbiter.Arbiter*/ArbiterP__1__state == /*HplSensirionSht11C.Arbiter.Arbiter*/ArbiterP__1__RES_IMM_GRANTING));

      {
#line 182
        __nesc_atomic_end(__nesc_atomic); 
#line 182
        return __nesc_temp;
      }
    }
#line 184
    __nesc_atomic_end(__nesc_atomic); }
}

# 65 "/home/tinyos/local/src/tinyos-2.x/tos/interfaces/ResourceDefaultOwner.nc"
inline static bool /*HplSensirionSht11C.SplitControlPowerManagerC.PowerManager*/PowerManagerP__0__ResourceDefaultOwner__isOwner(void ){
#line 65
  unsigned char result;
#line 65

#line 65
  result = /*HplSensirionSht11C.Arbiter.Arbiter*/ArbiterP__1__ResourceDefaultOwner__isOwner();
#line 65

#line 65
  return result;
#line 65
}
#line 65
# 92 "/home/tinyos/local/src/tinyos-2.x/tos/lib/power/PowerManagerP.nc"
static inline void /*HplSensirionSht11C.SplitControlPowerManagerC.PowerManager*/PowerManagerP__0__SplitControl__startDone(error_t error)
#line 92
{
  if (/*HplSensirionSht11C.SplitControlPowerManagerC.PowerManager*/PowerManagerP__0__ResourceDefaultOwner__isOwner()) {
    /*HplSensirionSht11C.SplitControlPowerManagerC.PowerManager*/PowerManagerP__0__ResourceDefaultOwner__release();
    }
}

# 92 "/home/tinyos/local/src/tinyos-2.x/tos/interfaces/SplitControl.nc"
inline static void HplSensirionSht11P__SplitControl__startDone(error_t error){
#line 92
  /*HplSensirionSht11C.SplitControlPowerManagerC.PowerManager*/PowerManagerP__0__SplitControl__startDone(error);
#line 92
}
#line 92
# 59 "/home/tinyos/local/src/tinyos-2.x/tos/platforms/telosa/chips/sht11/HplSensirionSht11P.nc"
static inline void HplSensirionSht11P__Timer__fired(void )
#line 59
{
  HplSensirionSht11P__SplitControl__startDone(SUCCESS);
}

# 98 "/home/tinyos/local/src/tinyos-2.x/tos/chips/sht11/SensirionSht11ReaderP.nc"
static inline void /*TCPEchoC.Sht11C.SensirionSht11ReaderP*/SensirionSht11ReaderP__0__Sht11Temp__resetDone(error_t result)
#line 98
{
}



static inline void /*TCPEchoC.Sht11C.SensirionSht11ReaderP*/SensirionSht11ReaderP__0__Sht11Hum__resetDone(error_t result)
#line 103
{
}

# 406 "/home/tinyos/local/src/tinyos-2.x/tos/chips/sht11/SensirionSht11LogicP.nc"
static inline void /*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP__0__SensirionSht11__default__resetDone(uint8_t client, error_t result)
#line 406
{
}

# 54 "/home/tinyos/local/src/tinyos-2.x/tos/chips/sht11/SensirionSht11.nc"
inline static void /*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP__0__SensirionSht11__resetDone(uint8_t arg_0x4150f970, error_t result){
#line 54
  switch (arg_0x4150f970) {
#line 54
    case /*TCPEchoC.Sht11C*/SensirionSht11C__0__TEMP_KEY:
#line 54
      /*TCPEchoC.Sht11C.SensirionSht11ReaderP*/SensirionSht11ReaderP__0__Sht11Temp__resetDone(result);
#line 54
      break;
#line 54
    case /*TCPEchoC.Sht11C*/SensirionSht11C__0__HUM_KEY:
#line 54
      /*TCPEchoC.Sht11C.SensirionSht11ReaderP*/SensirionSht11ReaderP__0__Sht11Hum__resetDone(result);
#line 54
      break;
#line 54
    default:
#line 54
      /*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP__0__SensirionSht11__default__resetDone(arg_0x4150f970, result);
#line 54
      break;
#line 54
    }
#line 54
}
#line 54
# 287 "/home/tinyos/local/src/tinyos-2.x/tos/chips/sht11/SensirionSht11LogicP.nc"
static inline void /*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP__0__Timer__fired(void )
#line 287
{

  switch (/*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP__0__cmd) {

      case /*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP__0__CMD_SOFT_RESET: 

        /*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP__0__busy = FALSE;
      /*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP__0__SensirionSht11__resetDone(/*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP__0__currentClient, SUCCESS);
      break;

      case /*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP__0__CMD_MEASURE_TEMPERATURE: 

        /*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP__0__busy = FALSE;
      /*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP__0__SensirionSht11__measureTemperatureDone(/*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP__0__currentClient, FAIL, 0);
      break;

      case /*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP__0__CMD_MEASURE_HUMIDITY: 

        /*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP__0__busy = FALSE;
      /*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP__0__SensirionSht11__measureHumidityDone(/*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP__0__currentClient, FAIL, 0);
      break;

      default: 

        break;
    }
}

# 56 "/home/tinyos/local/src/tinyos-2.x/tos/interfaces/TaskBasic.nc"
inline static error_t /*HplSensirionSht11C.SplitControlPowerManagerC.PowerManager*/PowerManagerP__0__startTask__postTask(void ){
#line 56
  unsigned char result;
#line 56

#line 56
  result = SchedulerBasicP__TaskBasic__postTask(/*HplSensirionSht11C.SplitControlPowerManagerC.PowerManager*/PowerManagerP__0__startTask);
#line 56

#line 56
  return result;
#line 56
}
#line 56
# 74 "/home/tinyos/local/src/tinyos-2.x/tos/lib/power/PowerManagerP.nc"
static inline void /*HplSensirionSht11C.SplitControlPowerManagerC.PowerManager*/PowerManagerP__0__ResourceDefaultOwner__requested(void )
#line 74
{
  if (/*HplSensirionSht11C.SplitControlPowerManagerC.PowerManager*/PowerManagerP__0__stopping == FALSE) {
      /*HplSensirionSht11C.SplitControlPowerManagerC.PowerManager*/PowerManagerP__0__startTask__postTask();
    }
  else {
#line 78
    /*HplSensirionSht11C.SplitControlPowerManagerC.PowerManager*/PowerManagerP__0__requested = TRUE;
    }
}

# 73 "/home/tinyos/local/src/tinyos-2.x/tos/interfaces/ResourceDefaultOwner.nc"
inline static void /*HplSensirionSht11C.Arbiter.Arbiter*/ArbiterP__1__ResourceDefaultOwner__requested(void ){
#line 73
  /*HplSensirionSht11C.SplitControlPowerManagerC.PowerManager*/PowerManagerP__0__ResourceDefaultOwner__requested();
#line 73
}
#line 73
# 54 "/home/tinyos/local/src/tinyos-2.x/tos/system/FcfsResourceQueueC.nc"
static inline bool /*HplSensirionSht11C.Arbiter.Queue*/FcfsResourceQueueC__2__FcfsQueue__isEnqueued(resource_client_id_t id)
#line 54
{
  /* atomic removed: atomic calls only */
#line 55
  {
    unsigned char __nesc_temp = 
#line 55
    /*HplSensirionSht11C.Arbiter.Queue*/FcfsResourceQueueC__2__resQ[id] != /*HplSensirionSht11C.Arbiter.Queue*/FcfsResourceQueueC__2__NO_ENTRY || /*HplSensirionSht11C.Arbiter.Queue*/FcfsResourceQueueC__2__qTail == id;

#line 55
    return __nesc_temp;
  }
}

#line 72
static inline error_t /*HplSensirionSht11C.Arbiter.Queue*/FcfsResourceQueueC__2__FcfsQueue__enqueue(resource_client_id_t id)
#line 72
{
  /* atomic removed: atomic calls only */
#line 73
  {
    if (!/*HplSensirionSht11C.Arbiter.Queue*/FcfsResourceQueueC__2__FcfsQueue__isEnqueued(id)) {
        if (/*HplSensirionSht11C.Arbiter.Queue*/FcfsResourceQueueC__2__qHead == /*HplSensirionSht11C.Arbiter.Queue*/FcfsResourceQueueC__2__NO_ENTRY) {
          /*HplSensirionSht11C.Arbiter.Queue*/FcfsResourceQueueC__2__qHead = id;
          }
        else {
#line 78
          /*HplSensirionSht11C.Arbiter.Queue*/FcfsResourceQueueC__2__resQ[/*HplSensirionSht11C.Arbiter.Queue*/FcfsResourceQueueC__2__qTail] = id;
          }
#line 79
        /*HplSensirionSht11C.Arbiter.Queue*/FcfsResourceQueueC__2__qTail = id;
        {
          unsigned char __nesc_temp = 
#line 80
          SUCCESS;

#line 80
          return __nesc_temp;
        }
      }
#line 82
    {
      unsigned char __nesc_temp = 
#line 82
      EBUSY;

#line 82
      return __nesc_temp;
    }
  }
}

# 69 "/home/tinyos/local/src/tinyos-2.x/tos/interfaces/ResourceQueue.nc"
inline static error_t /*HplSensirionSht11C.Arbiter.Arbiter*/ArbiterP__1__Queue__enqueue(resource_client_id_t id){
#line 69
  unsigned char result;
#line 69

#line 69
  result = /*HplSensirionSht11C.Arbiter.Queue*/FcfsResourceQueueC__2__FcfsQueue__enqueue(id);
#line 69

#line 69
  return result;
#line 69
}
#line 69
# 201 "/home/tinyos/local/src/tinyos-2.x/tos/system/ArbiterP.nc"
static inline void /*HplSensirionSht11C.Arbiter.Arbiter*/ArbiterP__1__ResourceRequested__default__requested(uint8_t id)
#line 201
{
}

# 43 "/home/tinyos/local/src/tinyos-2.x/tos/interfaces/ResourceRequested.nc"
inline static void /*HplSensirionSht11C.Arbiter.Arbiter*/ArbiterP__1__ResourceRequested__requested(uint8_t arg_0x40e33d38){
#line 43
    /*HplSensirionSht11C.Arbiter.Arbiter*/ArbiterP__1__ResourceRequested__default__requested(arg_0x40e33d38);
#line 43
}
#line 43
# 77 "/home/tinyos/local/src/tinyos-2.x/tos/system/ArbiterP.nc"
static inline error_t /*HplSensirionSht11C.Arbiter.Arbiter*/ArbiterP__1__Resource__request(uint8_t id)
#line 77
{
  /*HplSensirionSht11C.Arbiter.Arbiter*/ArbiterP__1__ResourceRequested__requested(/*HplSensirionSht11C.Arbiter.Arbiter*/ArbiterP__1__resId);
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 79
    {
      if (/*HplSensirionSht11C.Arbiter.Arbiter*/ArbiterP__1__state == /*HplSensirionSht11C.Arbiter.Arbiter*/ArbiterP__1__RES_CONTROLLED) {
          /*HplSensirionSht11C.Arbiter.Arbiter*/ArbiterP__1__state = /*HplSensirionSht11C.Arbiter.Arbiter*/ArbiterP__1__RES_GRANTING;
          /*HplSensirionSht11C.Arbiter.Arbiter*/ArbiterP__1__reqResId = id;
        }
      else {
          unsigned char __nesc_temp = 
#line 84
          /*HplSensirionSht11C.Arbiter.Arbiter*/ArbiterP__1__Queue__enqueue(id);

          {
#line 84
            __nesc_atomic_end(__nesc_atomic); 
#line 84
            return __nesc_temp;
          }
        }
    }
#line 87
    __nesc_atomic_end(__nesc_atomic); }
#line 86
  /*HplSensirionSht11C.Arbiter.Arbiter*/ArbiterP__1__ResourceDefaultOwner__requested();
  return SUCCESS;
}

# 78 "/home/tinyos/local/src/tinyos-2.x/tos/interfaces/Resource.nc"
inline static error_t /*TCPEchoC.Sht11C.SensirionSht11ReaderP*/SensirionSht11ReaderP__0__TempResource__request(void ){
#line 78
  unsigned char result;
#line 78

#line 78
  result = /*HplSensirionSht11C.Arbiter.Arbiter*/ArbiterP__1__Resource__request(/*TCPEchoC.Sht11C*/SensirionSht11C__0__TEMP_KEY);
#line 78

#line 78
  return result;
#line 78
}
#line 78
# 60 "/home/tinyos/local/src/tinyos-2.x/tos/chips/sht11/SensirionSht11ReaderP.nc"
static inline error_t /*TCPEchoC.Sht11C.SensirionSht11ReaderP*/SensirionSht11ReaderP__0__Temperature__read(void )
#line 60
{
  /*TCPEchoC.Sht11C.SensirionSht11ReaderP*/SensirionSht11ReaderP__0__TempResource__request();
  return SUCCESS;
}

# 55 "/home/tinyos/local/src/tinyos-2.x/tos/interfaces/Read.nc"
inline static error_t HttpdP__ReadTemp__read(void ){
#line 55
  unsigned char result;
#line 55

#line 55
  result = /*TCPEchoC.Sht11C.SensirionSht11ReaderP*/SensirionSht11ReaderP__0__Temperature__read();
#line 55

#line 55
  return result;
#line 55
}
#line 55
# 96 "HttpdP.nc"
static inline void HttpdP__TimerTemp__fired(void )
#line 96
{
  HttpdP__ReadTemp__read();
}

# 193 "/home/tinyos/local/src/tinyos-2.x/tos/lib/timer/VirtualizeTimerC.nc"
static inline void /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__Timer__default__fired(uint8_t num)
{
}

# 72 "/home/tinyos/local/src/tinyos-2.x/tos/lib/timer/Timer.nc"
inline static void /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__Timer__fired(uint8_t arg_0x409729f0){
#line 72
  switch (arg_0x409729f0) {
#line 72
    case 0U:
#line 72
      TCPEchoP__StatusTimer__fired();
#line 72
      break;
#line 72
    case 2U:
#line 72
      PacketLinkP__DelayTimer__fired();
#line 72
      break;
#line 72
    case 3U:
#line 72
      IPDispatchP__ExpireTimer__fired();
#line 72
      break;
#line 72
    case 4U:
#line 72
      ICMPResponderP__Solicitation__fired();
#line 72
      break;
#line 72
    case 5U:
#line 72
      ICMPResponderP__Advertisement__fired();
#line 72
      break;
#line 72
    case 6U:
#line 72
      ICMPResponderP__PingTimer__fired();
#line 72
      break;
#line 72
    case 7U:
#line 72
      IPRoutingP__TrafficGenTimer__fired();
#line 72
      break;
#line 72
    case 8U:
#line 72
      IPRoutingP__SortTimer__fired();
#line 72
      break;
#line 72
    case 9U:
#line 72
      TcpP__Timer__fired();
#line 72
      break;
#line 72
    case 10U:
#line 72
      HplSensirionSht11P__Timer__fired();
#line 72
      break;
#line 72
    case 11U:
#line 72
      /*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP__0__Timer__fired();
#line 72
      break;
#line 72
    case 12U:
#line 72
      HttpdP__TimerTemp__fired();
#line 72
      break;
#line 72
    default:
#line 72
      /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__Timer__default__fired(arg_0x409729f0);
#line 72
      break;
#line 72
    }
#line 72
}
#line 72
# 128 "/home/tinyos/local/src/tinyos-2.x/tos/lib/timer/VirtualizeTimerC.nc"
static inline void /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__TimerFrom__fired(void )
{
  /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__fireTimers(/*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__TimerFrom__getNow());
}

# 72 "/home/tinyos/local/src/tinyos-2.x/tos/lib/timer/Timer.nc"
inline static void /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__Timer__fired(void ){
#line 72
  /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__TimerFrom__fired();
#line 72
}
#line 72
# 80 "/home/tinyos/local/src/tinyos-2.x/tos/lib/timer/TransformAlarmC.nc"
static inline /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__to_size_type /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__Alarm__getAlarm(void )
{
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 82
    {
      /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__to_size_type __nesc_temp = 
#line 82
      /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__m_t0 + /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__m_dt;

      {
#line 82
        __nesc_atomic_end(__nesc_atomic); 
#line 82
        return __nesc_temp;
      }
    }
#line 84
    __nesc_atomic_end(__nesc_atomic); }
}

# 105 "/home/tinyos/local/src/tinyos-2.x/tos/lib/timer/Alarm.nc"
inline static /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__Alarm__size_type /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__Alarm__getAlarm(void ){
#line 105
  unsigned long result;
#line 105

#line 105
  result = /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__Alarm__getAlarm();
#line 105

#line 105
  return result;
#line 105
}
#line 105
# 63 "/home/tinyos/local/src/tinyos-2.x/tos/lib/timer/AlarmToTimerC.nc"
static inline void /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__fired__runTask(void )
{
  if (/*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__m_oneshot == FALSE) {
    /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__start(/*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__Alarm__getAlarm(), /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__m_dt, FALSE);
    }
#line 67
  /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__Timer__fired();
}

# 46 "/home/tinyos/local/src/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static inline  uint16_t /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__CC2int(/*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__cc_t x)
#line 46
{
#line 46
  union /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3____nesc_unnamed4457 {
#line 46
    /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__cc_t f;
#line 46
    uint16_t t;
  } 
#line 46
  c = { .f = x };

#line 46
  return c.t;
}

static inline uint16_t /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__compareControl(void )
{
  /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__cc_t x = { 
  .cm = 1, 
  .ccis = 0, 
  .clld = 0, 
  .cap = 0, 
  .ccie = 0 };

  return /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__CC2int(x);
}

#line 94
static inline void /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Control__setControlAsCompare(void )
{
  * (volatile uint16_t * )386U = /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__compareControl();
}

# 36 "/home/tinyos/local/src/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerControl.nc"
inline static void /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__0__Msp430TimerControl__setControlAsCompare(void ){
#line 36
  /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Control__setControlAsCompare();
#line 36
}
#line 36
# 42 "/home/tinyos/local/src/tinyos-2.x/tos/chips/msp430/timer/Msp430AlarmC.nc"
static inline error_t /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__0__Init__init(void )
{
  /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__0__Msp430TimerControl__disableEvents();
  /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__0__Msp430TimerControl__setControlAsCompare();
  return SUCCESS;
}

# 24 "/home/tinyos/local/src/tinyos-2.x/tos/lib/net/blip/IPExtensionP.nc"
static inline error_t IPExtensionP__Init__init(void )
#line 24
{
  IPExtensionP__ext_hop = IPExtensionP__ext_dest = (void *)0;
  return SUCCESS;
}

# 82 "/home/tinyos/local/src/tinyos-2.x/tos/system/ActiveMessageAddressC.nc"
static inline am_group_t ActiveMessageAddressC__ActiveMessageAddress__amGroup(void )
#line 82
{
  am_group_t myGroup;

  /* atomic removed: atomic calls only */
#line 84
  myGroup = ActiveMessageAddressC__group;
  return myGroup;
}

# 55 "/home/tinyos/local/src/tinyos-2.x/tos/interfaces/ActiveMessageAddress.nc"
inline static am_group_t CC2420ControlP__ActiveMessageAddress__amGroup(void ){
#line 55
  unsigned char result;
#line 55

#line 55
  result = ActiveMessageAddressC__ActiveMessageAddress__amGroup();
#line 55

#line 55
  return result;
#line 55
}
#line 55
# 95 "/home/tinyos/local/src/tinyos-2.x/tos/system/ActiveMessageAddressC.nc"
static inline am_addr_t ActiveMessageAddressC__amAddress(void )
#line 95
{
  am_addr_t myAddr;

  /* atomic removed: atomic calls only */
#line 97
  myAddr = ActiveMessageAddressC__addr;
  return myAddr;
}

#line 61
static inline am_addr_t ActiveMessageAddressC__ActiveMessageAddress__amAddress(void )
#line 61
{
  return ActiveMessageAddressC__amAddress();
}

# 50 "/home/tinyos/local/src/tinyos-2.x/tos/interfaces/ActiveMessageAddress.nc"
inline static am_addr_t CC2420ControlP__ActiveMessageAddress__amAddress(void ){
#line 50
  unsigned int result;
#line 50

#line 50
  result = ActiveMessageAddressC__ActiveMessageAddress__amAddress();
#line 50

#line 50
  return result;
#line 50
}
#line 50
# 52 "/home/tinyos/local/src/tinyos-2.x/tos/chips/msp430/pins/HplMsp430GeneralIOP.nc"
static inline void /*HplMsp430GeneralIOC.P45*/HplMsp430GeneralIOP__29__IO__makeOutput(void )
#line 52
{
  /* atomic removed: atomic calls only */
#line 52
  * (volatile uint8_t * )30U |= 0x01 << 5;
}

# 71 "/home/tinyos/local/src/tinyos-2.x/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
inline static void /*HplCC2420PinsC.VRENM*/Msp430GpioC__9__HplGeneralIO__makeOutput(void ){
#line 71
  /*HplMsp430GeneralIOC.P45*/HplMsp430GeneralIOP__29__IO__makeOutput();
#line 71
}
#line 71
# 43 "/home/tinyos/local/src/tinyos-2.x/tos/chips/msp430/pins/Msp430GpioC.nc"
static inline void /*HplCC2420PinsC.VRENM*/Msp430GpioC__9__GeneralIO__makeOutput(void )
#line 43
{
#line 43
  /*HplCC2420PinsC.VRENM*/Msp430GpioC__9__HplGeneralIO__makeOutput();
}

# 35 "/home/tinyos/local/src/tinyos-2.x/tos/interfaces/GeneralIO.nc"
inline static void CC2420ControlP__VREN__makeOutput(void ){
#line 35
  /*HplCC2420PinsC.VRENM*/Msp430GpioC__9__GeneralIO__makeOutput();
#line 35
}
#line 35
# 52 "/home/tinyos/local/src/tinyos-2.x/tos/chips/msp430/pins/HplMsp430GeneralIOP.nc"
static inline void /*HplMsp430GeneralIOC.P46*/HplMsp430GeneralIOP__30__IO__makeOutput(void )
#line 52
{
  /* atomic removed: atomic calls only */
#line 52
  * (volatile uint8_t * )30U |= 0x01 << 6;
}

# 71 "/home/tinyos/local/src/tinyos-2.x/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
inline static void /*HplCC2420PinsC.RSTNM*/Msp430GpioC__7__HplGeneralIO__makeOutput(void ){
#line 71
  /*HplMsp430GeneralIOC.P46*/HplMsp430GeneralIOP__30__IO__makeOutput();
#line 71
}
#line 71
# 43 "/home/tinyos/local/src/tinyos-2.x/tos/chips/msp430/pins/Msp430GpioC.nc"
static inline void /*HplCC2420PinsC.RSTNM*/Msp430GpioC__7__GeneralIO__makeOutput(void )
#line 43
{
#line 43
  /*HplCC2420PinsC.RSTNM*/Msp430GpioC__7__HplGeneralIO__makeOutput();
}

# 35 "/home/tinyos/local/src/tinyos-2.x/tos/interfaces/GeneralIO.nc"
inline static void CC2420ControlP__RSTN__makeOutput(void ){
#line 35
  /*HplCC2420PinsC.RSTNM*/Msp430GpioC__7__GeneralIO__makeOutput();
#line 35
}
#line 35
# 52 "/home/tinyos/local/src/tinyos-2.x/tos/chips/msp430/pins/HplMsp430GeneralIOP.nc"
static inline void /*HplMsp430GeneralIOC.P42*/HplMsp430GeneralIOP__26__IO__makeOutput(void )
#line 52
{
  /* atomic removed: atomic calls only */
#line 52
  * (volatile uint8_t * )30U |= 0x01 << 2;
}

# 71 "/home/tinyos/local/src/tinyos-2.x/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
inline static void /*HplCC2420PinsC.CSNM*/Msp430GpioC__4__HplGeneralIO__makeOutput(void ){
#line 71
  /*HplMsp430GeneralIOC.P42*/HplMsp430GeneralIOP__26__IO__makeOutput();
#line 71
}
#line 71
# 43 "/home/tinyos/local/src/tinyos-2.x/tos/chips/msp430/pins/Msp430GpioC.nc"
static inline void /*HplCC2420PinsC.CSNM*/Msp430GpioC__4__GeneralIO__makeOutput(void )
#line 43
{
#line 43
  /*HplCC2420PinsC.CSNM*/Msp430GpioC__4__HplGeneralIO__makeOutput();
}

# 35 "/home/tinyos/local/src/tinyos-2.x/tos/interfaces/GeneralIO.nc"
inline static void CC2420ControlP__CSN__makeOutput(void ){
#line 35
  /*HplCC2420PinsC.CSNM*/Msp430GpioC__4__GeneralIO__makeOutput();
#line 35
}
#line 35
# 121 "/home/tinyos/local/src/tinyos-2.x/tos/chips/cc2420/control/CC2420ControlP.nc"
static inline error_t CC2420ControlP__Init__init(void )
#line 121
{
  CC2420ControlP__CSN__makeOutput();
  CC2420ControlP__RSTN__makeOutput();
  CC2420ControlP__VREN__makeOutput();

  CC2420ControlP__m_short_addr = CC2420ControlP__ActiveMessageAddress__amAddress();
  CC2420ControlP__m_pan = CC2420ControlP__ActiveMessageAddress__amGroup();
  CC2420ControlP__m_tx_power = 31;
  CC2420ControlP__m_channel = 17;





  CC2420ControlP__addressRecognition = TRUE;





  CC2420ControlP__hwAddressRecognition = FALSE;






  CC2420ControlP__autoAckEnabled = TRUE;



  CC2420ControlP__hwAutoAckDefault = TRUE;
  CC2420ControlP__hwAddressRecognition = TRUE;





  return SUCCESS;
}

# 81 "/home/tinyos/local/src/tinyos-2.x/tos/system/StateImplP.nc"
static inline error_t StateImplP__Init__init(void )
#line 81
{
  int i;

#line 83
  for (i = 0; i < 5U; i++) {
      StateImplP__state[i] = StateImplP__S_IDLE;
    }
  return SUCCESS;
}

# 45 "/home/tinyos/local/src/tinyos-2.x/tos/system/FcfsResourceQueueC.nc"
static inline error_t /*Msp430UsartShare0P.ArbiterC.Queue*/FcfsResourceQueueC__1__Init__init(void )
#line 45
{
  memset(/*Msp430UsartShare0P.ArbiterC.Queue*/FcfsResourceQueueC__1__resQ, /*Msp430UsartShare0P.ArbiterC.Queue*/FcfsResourceQueueC__1__NO_ENTRY, sizeof /*Msp430UsartShare0P.ArbiterC.Queue*/FcfsResourceQueueC__1__resQ);
  return SUCCESS;
}

# 46 "/home/tinyos/local/src/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static inline  uint16_t /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__CC2int(/*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__cc_t x)
#line 46
{
#line 46
  union /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5____nesc_unnamed4458 {
#line 46
    /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__cc_t f;
#line 46
    uint16_t t;
  } 
#line 46
  c = { .f = x };

#line 46
  return c.t;
}

static inline uint16_t /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__compareControl(void )
{
  /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__cc_t x = { 
  .cm = 1, 
  .ccis = 0, 
  .clld = 0, 
  .cap = 0, 
  .ccie = 0 };

  return /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__CC2int(x);
}

#line 94
static inline void /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__Control__setControlAsCompare(void )
{
  * (volatile uint16_t * )390U = /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__compareControl();
}

# 36 "/home/tinyos/local/src/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerControl.nc"
inline static void /*AlarmMultiplexC.Alarm.Alarm32khz32C.AlarmC.Msp430Alarm*/Msp430AlarmC__1__Msp430TimerControl__setControlAsCompare(void ){
#line 36
  /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__Control__setControlAsCompare();
#line 36
}
#line 36
# 42 "/home/tinyos/local/src/tinyos-2.x/tos/chips/msp430/timer/Msp430AlarmC.nc"
static inline error_t /*AlarmMultiplexC.Alarm.Alarm32khz32C.AlarmC.Msp430Alarm*/Msp430AlarmC__1__Init__init(void )
{
  /*AlarmMultiplexC.Alarm.Alarm32khz32C.AlarmC.Msp430Alarm*/Msp430AlarmC__1__Msp430TimerControl__disableEvents();
  /*AlarmMultiplexC.Alarm.Alarm32khz32C.AlarmC.Msp430Alarm*/Msp430AlarmC__1__Msp430TimerControl__setControlAsCompare();
  return SUCCESS;
}

# 50 "/home/tinyos/local/src/tinyos-2.x/tos/chips/msp430/pins/HplMsp430GeneralIOP.nc"
static inline void /*HplMsp430GeneralIOC.P41*/HplMsp430GeneralIOP__25__IO__makeInput(void )
#line 50
{
  /* atomic removed: atomic calls only */
#line 50
  * (volatile uint8_t * )30U &= ~(0x01 << 1);
}

# 64 "/home/tinyos/local/src/tinyos-2.x/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
inline static void /*HplCC2420PinsC.SFDM*/Msp430GpioC__8__HplGeneralIO__makeInput(void ){
#line 64
  /*HplMsp430GeneralIOC.P41*/HplMsp430GeneralIOP__25__IO__makeInput();
#line 64
}
#line 64
# 41 "/home/tinyos/local/src/tinyos-2.x/tos/chips/msp430/pins/Msp430GpioC.nc"
static inline void /*HplCC2420PinsC.SFDM*/Msp430GpioC__8__GeneralIO__makeInput(void )
#line 41
{
#line 41
  /*HplCC2420PinsC.SFDM*/Msp430GpioC__8__HplGeneralIO__makeInput();
}

# 33 "/home/tinyos/local/src/tinyos-2.x/tos/interfaces/GeneralIO.nc"
inline static void CC2420TransmitP__SFD__makeInput(void ){
#line 33
  /*HplCC2420PinsC.SFDM*/Msp430GpioC__8__GeneralIO__makeInput();
#line 33
}
#line 33


inline static void CC2420TransmitP__CSN__makeOutput(void ){
#line 35
  /*HplCC2420PinsC.CSNM*/Msp430GpioC__4__GeneralIO__makeOutput();
#line 35
}
#line 35
# 50 "/home/tinyos/local/src/tinyos-2.x/tos/chips/msp430/pins/HplMsp430GeneralIOP.nc"
static inline void /*HplMsp430GeneralIOC.P14*/HplMsp430GeneralIOP__4__IO__makeInput(void )
#line 50
{
  /* atomic removed: atomic calls only */
#line 50
  * (volatile uint8_t * )34U &= ~(0x01 << 4);
}

# 64 "/home/tinyos/local/src/tinyos-2.x/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
inline static void /*HplCC2420PinsC.CCAM*/Msp430GpioC__3__HplGeneralIO__makeInput(void ){
#line 64
  /*HplMsp430GeneralIOC.P14*/HplMsp430GeneralIOP__4__IO__makeInput();
#line 64
}
#line 64
# 41 "/home/tinyos/local/src/tinyos-2.x/tos/chips/msp430/pins/Msp430GpioC.nc"
static inline void /*HplCC2420PinsC.CCAM*/Msp430GpioC__3__GeneralIO__makeInput(void )
#line 41
{
#line 41
  /*HplCC2420PinsC.CCAM*/Msp430GpioC__3__HplGeneralIO__makeInput();
}

# 33 "/home/tinyos/local/src/tinyos-2.x/tos/interfaces/GeneralIO.nc"
inline static void CC2420TransmitP__CCA__makeInput(void ){
#line 33
  /*HplCC2420PinsC.CCAM*/Msp430GpioC__3__GeneralIO__makeInput();
#line 33
}
#line 33
# 160 "/home/tinyos/local/src/tinyos-2.x/tos/chips/cc2420/transmit/CC2420TransmitP.nc"
static inline error_t CC2420TransmitP__Init__init(void )
#line 160
{
  CC2420TransmitP__CCA__makeInput();
  CC2420TransmitP__CSN__makeOutput();
  CC2420TransmitP__SFD__makeInput();
  return SUCCESS;
}

# 151 "/home/tinyos/local/src/tinyos-2.x/tos/chips/cc2420/receive/CC2420ReceiveP.nc"
static inline error_t CC2420ReceiveP__Init__init(void )
#line 151
{
  CC2420ReceiveP__m_p_rx_buf = &CC2420ReceiveP__m_rx_buf;
  return SUCCESS;
}

# 44 "/home/tinyos/local/src/tinyos-2.x/tos/system/RandomMlcgC.nc"
static inline error_t RandomMlcgC__Init__init(void )
#line 44
{
  /* atomic removed: atomic calls only */
#line 45
  RandomMlcgC__seed = (uint32_t )(TOS_NODE_ID + 1);

  return SUCCESS;
}

# 41 "/home/tinyos/local/src/tinyos-2.x/tos/interfaces/Random.nc"
inline static uint16_t UniqueSendP__Random__rand16(void ){
#line 41
  unsigned int result;
#line 41

#line 41
  result = RandomMlcgC__Random__rand16();
#line 41

#line 41
  return result;
#line 41
}
#line 41
# 62 "/home/tinyos/local/src/tinyos-2.x/tos/chips/cc2420/unique/UniqueSendP.nc"
static inline error_t UniqueSendP__Init__init(void )
#line 62
{
  UniqueSendP__localSendId = UniqueSendP__Random__rand16();
  return SUCCESS;
}

# 71 "/home/tinyos/local/src/tinyos-2.x/tos/chips/cc2420/unique/UniqueReceiveP.nc"
static inline error_t UniqueReceiveP__Init__init(void )
#line 71
{
  int i;

#line 73
  for (i = 0; i < 4; i++) {
      UniqueReceiveP__receivedMessages[i].source = (am_addr_t )0xFFFF;
      UniqueReceiveP__receivedMessages[i].dsn = 0;
    }
  return SUCCESS;
}

# 45 "/home/tinyos/local/src/tinyos-2.x/tos/system/FcfsResourceQueueC.nc"
static inline error_t /*CC2420TinyosNetworkC.FcfsResourceQueueC*/FcfsResourceQueueC__0__Init__init(void )
#line 45
{
  memset(/*CC2420TinyosNetworkC.FcfsResourceQueueC*/FcfsResourceQueueC__0__resQ, /*CC2420TinyosNetworkC.FcfsResourceQueueC*/FcfsResourceQueueC__0__NO_ENTRY, sizeof /*CC2420TinyosNetworkC.FcfsResourceQueueC*/FcfsResourceQueueC__0__resQ);
  return SUCCESS;
}

# 65 "/home/tinyos/local/src/tinyos-2.x/tos/system/PoolP.nc"
static inline error_t /*IPDispatchC.FragPool.PoolP*/PoolP__0__Init__init(void )
#line 65
{
  int i;

#line 67
  for (i = 0; i < 14; i++) {
      /*IPDispatchC.FragPool.PoolP*/PoolP__0__queue[i] = &/*IPDispatchC.FragPool.PoolP*/PoolP__0__pool[i];
    }
  /*IPDispatchC.FragPool.PoolP*/PoolP__0__free = 14;
  /*IPDispatchC.FragPool.PoolP*/PoolP__0__index = 0;
  return SUCCESS;
}

#line 65
static inline error_t /*IPDispatchC.SendEntryPool.PoolP*/PoolP__1__Init__init(void )
#line 65
{
  int i;

#line 67
  for (i = 0; i < 14; i++) {
      /*IPDispatchC.SendEntryPool.PoolP*/PoolP__1__queue[i] = &/*IPDispatchC.SendEntryPool.PoolP*/PoolP__1__pool[i];
    }
  /*IPDispatchC.SendEntryPool.PoolP*/PoolP__1__free = 14;
  /*IPDispatchC.SendEntryPool.PoolP*/PoolP__1__index = 0;
  return SUCCESS;
}

#line 65
static inline error_t /*IPDispatchC.SendInfoPool.PoolP*/PoolP__2__Init__init(void )
#line 65
{
  int i;

#line 67
  for (i = 0; i < 14; i++) {
      /*IPDispatchC.SendInfoPool.PoolP*/PoolP__2__queue[i] = &/*IPDispatchC.SendInfoPool.PoolP*/PoolP__2__pool[i];
    }
  /*IPDispatchC.SendInfoPool.PoolP*/PoolP__2__free = 14;
  /*IPDispatchC.SendInfoPool.PoolP*/PoolP__2__index = 0;
  return SUCCESS;
}

# 183 "/home/tinyos/local/src/tinyos-2.x/tos/lib/net/blip/UdpP.nc"
static inline void UdpP__Statistics__clear(void )
#line 183
{

  ip_memclr((uint8_t *)&UdpP__stats, sizeof(udp_statistics_t ));
}

#line 50
static inline error_t UdpP__Init__init(void )
#line 50
{
  UdpP__Statistics__clear();
  ip_memclr((uint8_t *)UdpP__local_ports, sizeof(uint16_t ) * UdpP__N_CLIENTS);
  return SUCCESS;
}

# 88 "/home/tinyos/local/src/tinyos-2.x/tos/lib/net/blip/TcpP.nc"
static inline error_t TcpP__Init__init(void )
#line 88
{
  int i;

#line 90
  for (i = 0; i < 2U; i++) {
      TcpP__tcplib_init_sock(&TcpP__socks[i]);
    }
  return SUCCESS;
}

# 45 "/home/tinyos/local/src/tinyos-2.x/tos/system/FcfsResourceQueueC.nc"
static inline error_t /*HplSensirionSht11C.Arbiter.Queue*/FcfsResourceQueueC__2__Init__init(void )
#line 45
{
  memset(/*HplSensirionSht11C.Arbiter.Queue*/FcfsResourceQueueC__2__resQ, /*HplSensirionSht11C.Arbiter.Queue*/FcfsResourceQueueC__2__NO_ENTRY, sizeof /*HplSensirionSht11C.Arbiter.Queue*/FcfsResourceQueueC__2__resQ);
  return SUCCESS;
}

# 51 "/home/tinyos/local/src/tinyos-2.x/tos/interfaces/Init.nc"
inline static error_t RealMainP__SoftwareInit__init(void ){
#line 51
  unsigned char result;
#line 51

#line 51
  result = /*HplSensirionSht11C.Arbiter.Queue*/FcfsResourceQueueC__2__Init__init();
#line 51
  result = ecombine(result, TcpP__Init__init());
#line 51
  result = ecombine(result, UdpP__Init__init());
#line 51
  result = ecombine(result, /*IPDispatchC.SendInfoPool.PoolP*/PoolP__2__Init__init());
#line 51
  result = ecombine(result, /*IPDispatchC.SendEntryPool.PoolP*/PoolP__1__Init__init());
#line 51
  result = ecombine(result, /*IPDispatchC.FragPool.PoolP*/PoolP__0__Init__init());
#line 51
  result = ecombine(result, /*CC2420TinyosNetworkC.FcfsResourceQueueC*/FcfsResourceQueueC__0__Init__init());
#line 51
  result = ecombine(result, UniqueReceiveP__Init__init());
#line 51
  result = ecombine(result, UniqueSendP__Init__init());
#line 51
  result = ecombine(result, RandomMlcgC__Init__init());
#line 51
  result = ecombine(result, CC2420ReceiveP__Init__init());
#line 51
  result = ecombine(result, CC2420TransmitP__Init__init());
#line 51
  result = ecombine(result, /*AlarmMultiplexC.Alarm.Alarm32khz32C.AlarmC.Msp430Alarm*/Msp430AlarmC__1__Init__init());
#line 51
  result = ecombine(result, /*Msp430UsartShare0P.ArbiterC.Queue*/FcfsResourceQueueC__1__Init__init());
#line 51
  result = ecombine(result, StateImplP__Init__init());
#line 51
  result = ecombine(result, CC2420ControlP__Init__init());
#line 51
  result = ecombine(result, IPExtensionP__Init__init());
#line 51
  result = ecombine(result, /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC__0__Init__init());
#line 51

#line 51
  return result;
#line 51
}
#line 51
# 53 "/home/tinyos/local/src/tinyos-2.x/tos/lib/timer/Timer.nc"
inline static void HttpdP__TimerTemp__startPeriodic(uint32_t dt){
#line 53
  /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__Timer__startPeriodic(12U, dt);
#line 53
}
#line 53
# 90 "HttpdP.nc"
static inline void HttpdP__Boot__booted(void )
#line 90
{
  HttpdP__http_state = HttpdP__S_IDLE;
  HttpdP__Tcp__bind(80);
  HttpdP__TimerTemp__startPeriodic(500);
}

# 10 "/home/tinyos/local/src/tinyos-2.x/tos/lib/net/blip/interfaces/UDP.nc"
inline static error_t TCPEchoP__Status__bind(uint16_t port){
#line 10
  unsigned char result;
#line 10

#line 10
  result = UdpP__UDP__bind(1U, port);
#line 10

#line 10
  return result;
#line 10
}
#line 10
inline static error_t TCPEchoP__Echo__bind(uint16_t port){
#line 10
  unsigned char result;
#line 10

#line 10
  result = UdpP__UDP__bind(0U, port);
#line 10

#line 10
  return result;
#line 10
}
#line 10
# 41 "/home/tinyos/local/src/tinyos-2.x/tos/interfaces/Random.nc"
inline static uint16_t TCPEchoP__Random__rand16(void ){
#line 41
  unsigned int result;
#line 41

#line 41
  result = RandomMlcgC__Random__rand16();
#line 41

#line 41
  return result;
#line 41
}
#line 41
# 62 "/home/tinyos/local/src/tinyos-2.x/tos/lib/timer/Timer.nc"
inline static void TCPEchoP__StatusTimer__startOneShot(uint32_t dt){
#line 62
  /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__Timer__startOneShot(0U, dt);
#line 62
}
#line 62
# 118 "/home/tinyos/local/src/tinyos-2.x/tos/lib/net/blip/PrintfUART.h"
static inline void printfUART_init()
#line 118
{
}

# 434 "/home/tinyos/local/src/tinyos-2.x/tos/lib/net/blip/ICMPResponderP.nc"
static inline void ICMPResponderP__Statistics__clear(void )
#line 434
{
  ip_memclr((uint8_t *)&ICMPResponderP__stats, sizeof(icmp_statistics_t ));
}

# 34 "/home/tinyos/local/src/tinyos-2.x/tos/lib/net/blip/interfaces/Statistics.nc"
inline static void TCPEchoP__ICMPStats__clear(void ){
#line 34
  ICMPResponderP__Statistics__clear();
#line 34
}
#line 34
# 1309 "/home/tinyos/local/src/tinyos-2.x/tos/lib/net/blip/IPRoutingP.nc"
static inline void IPRoutingP__Statistics__clear(void )
#line 1309
{
}

# 34 "/home/tinyos/local/src/tinyos-2.x/tos/lib/net/blip/interfaces/Statistics.nc"
inline static void TCPEchoP__RouteStats__clear(void ){
#line 34
  IPRoutingP__Statistics__clear();
#line 34
}
#line 34
# 1065 "/home/tinyos/local/src/tinyos-2.x/tos/lib/net/blip/IPDispatchP.nc"
static inline void IPDispatchP__Statistics__clear(void )
#line 1065
{
  ip_memclr((uint8_t *)&IPDispatchP__stats, sizeof(ip_statistics_t ));
}

# 34 "/home/tinyos/local/src/tinyos-2.x/tos/lib/net/blip/interfaces/Statistics.nc"
inline static void TCPEchoP__IPStats__clear(void ){
#line 34
  IPDispatchP__Statistics__clear();
#line 34
}
#line 34
# 83 "/home/tinyos/local/src/tinyos-2.x/tos/interfaces/SplitControl.nc"
inline static error_t IPDispatchP__RadioControl__start(void ){
#line 83
  unsigned char result;
#line 83

#line 83
  result = CC2420CsmaP__SplitControl__start();
#line 83

#line 83
  return result;
#line 83
}
#line 83
# 222 "/home/tinyos/local/src/tinyos-2.x/tos/lib/net/blip/IPDispatchP.nc"
static inline error_t IPDispatchP__SplitControl__start(void )
#line 222
{
  return IPDispatchP__RadioControl__start();
}

# 83 "/home/tinyos/local/src/tinyos-2.x/tos/interfaces/SplitControl.nc"
inline static error_t TCPEchoP__RadioControl__start(void ){
#line 83
  unsigned char result;
#line 83

#line 83
  result = IPDispatchP__SplitControl__start();
#line 83

#line 83
  return result;
#line 83
}
#line 83
# 68 "TCPEchoP.nc"
static inline void TCPEchoP__Boot__booted(void )
#line 68
{
  ;
  TCPEchoP__RadioControl__start();
  TCPEchoP__timerStarted = FALSE;

  TCPEchoP__IPStats__clear();
  TCPEchoP__RouteStats__clear();
  TCPEchoP__ICMPStats__clear();
  printfUART_init();



  TCPEchoP__route_dest.sin6_port = (((uint16_t )7000 << 8) | ((uint16_t )7000 >> 8)) & 0xffff;
  inet_pton6("fec0::64", & TCPEchoP__route_dest.sin6_addr);
  TCPEchoP__StatusTimer__startOneShot(TCPEchoP__Random__rand16() % (1024 * 75L));


  ;
  TCPEchoP__Echo__bind(7);
  TCPEchoP__TcpEcho__bind(7);
  TCPEchoP__Status__bind(7001);
}

# 53 "/home/tinyos/local/src/tinyos-2.x/tos/lib/timer/Timer.nc"
inline static void IPRoutingP__SortTimer__startPeriodic(uint32_t dt){
#line 53
  /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__Timer__startPeriodic(8U, dt);
#line 53
}
#line 53
# 188 "/home/tinyos/local/src/tinyos-2.x/tos/lib/net/blip/IPRoutingP.nc"
static inline void IPRoutingP__Boot__booted(void )
#line 188
{
  IPRoutingP__IPRouting__reset();
  IPRoutingP__reportSeqno = IPRoutingP__Random__rand16();

  IPRoutingP__Statistics__clear();
  IPRoutingP__SortTimer__startPeriodic(1024L * 60);
}

# 53 "/home/tinyos/local/src/tinyos-2.x/tos/lib/timer/Timer.nc"
inline static void IPDispatchP__ExpireTimer__startPeriodic(uint32_t dt){
#line 53
  /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__Timer__startPeriodic(3U, dt);
#line 53
}
#line 53
# 185 "/home/tinyos/local/src/tinyos-2.x/tos/lib/net/blip/IPDispatchP.nc"
static inline void IPDispatchP__forward_clear(void *ent)
#line 185
{
  forward_entry_t *fwd = (forward_entry_t *)ent;

#line 187
  fwd->timeout = T_UNUSED;
}

#line 178
static inline void IPDispatchP__reconstruct_clear(void *ent)
#line 178
{
  reconstruct_t *recon = (reconstruct_t *)ent;

#line 180
  ip_memclr((uint8_t *)& recon->metadata, sizeof(struct ip_metadata ));
  recon->timeout = T_UNUSED;
  recon->buf = (void *)0;
}

#line 253
static inline void IPDispatchP__Boot__booted(void )
#line 253
{
  IPDispatchP__Statistics__clear();

  ip_malloc_init();

  table_init(&IPDispatchP__recon_cache, IPDispatchP__recon_data, sizeof(reconstruct_t ), N_RECONSTRUCTIONS);
  table_init(&IPDispatchP__forward_cache, IPDispatchP__forward_data, sizeof(forward_entry_t ), N_FORWARD_ENT);

  table_map(&IPDispatchP__recon_cache, IPDispatchP__reconstruct_clear);
  table_map(&IPDispatchP__forward_cache, IPDispatchP__forward_clear);

  IPDispatchP__radioBusy = FALSE;

  IPDispatchP__ExpireTimer__startPeriodic(FRAG_EXPIRE_TIME);

  IPDispatchP__SplitControl__start();
  return;
}

# 53 "/home/tinyos/local/src/tinyos-2.x/tos/lib/timer/Timer.nc"
inline static void TcpP__Timer__startPeriodic(uint32_t dt){
#line 53
  /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__Timer__startPeriodic(9U, dt);
#line 53
}
#line 53
# 96 "/home/tinyos/local/src/tinyos-2.x/tos/lib/net/blip/TcpP.nc"
static inline void TcpP__Boot__booted(void )
#line 96
{
  TcpP__Timer__startPeriodic(512);
}

# 10 "/home/tinyos/local/src/tinyos-2.x/tos/lib/net/blip/interfaces/UDP.nc"
inline static error_t UDPShellP__UDP__bind(uint16_t port){
#line 10
  unsigned char result;
#line 10

#line 10
  result = UdpP__UDP__bind(2U, port);
#line 10

#line 10
  return result;
#line 10
}
#line 10
# 288 "/home/tinyos/local/src/tinyos-2.x/tos/lib/net/blip/shell/UDPShellP.nc"
static inline char *UDPShellP__RegisterShellCommand__default__getCommandName(uint8_t cmd_id)
#line 288
{
  return (void *)0;
}

# 3 "/home/tinyos/local/src/tinyos-2.x/tos/lib/net/blip/shell/RegisterShellCommand.nc"
inline static char *UDPShellP__RegisterShellCommand__getCommandName(uint8_t arg_0x415a3650){
#line 3
  char *result;
#line 3

#line 3
    result = UDPShellP__RegisterShellCommand__default__getCommandName(arg_0x415a3650);
#line 3

#line 3
  return result;
#line 3
}
#line 3
# 53 "/home/tinyos/local/src/tinyos-2.x/tos/lib/timer/Counter.nc"
inline static UDPShellP__Uptime__size_type UDPShellP__Uptime__get(void ){
#line 53
  unsigned long result;
#line 53

#line 53
  result = /*CounterMilli32C.Transform*/TransformCounterC__0__Counter__get();
#line 53

#line 53
  return result;
#line 53
}
#line 53
# 81 "/home/tinyos/local/src/tinyos-2.x/tos/lib/net/blip/shell/UDPShellP.nc"
static inline void UDPShellP__Boot__booted(void )
#line 81
{
  int i;

#line 83
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 83
    {
      UDPShellP__uptime = 0;

      UDPShellP__boot_time = UDPShellP__Uptime__get();
    }
#line 87
    __nesc_atomic_end(__nesc_atomic); }

  for (i = 0; i < UDPShellP__N_EXTERNAL; i++) {
      UDPShellP__externals[i].c_name[UDPShellP__CMDNAMSIZ - 1] = '\0';
      strncpy(UDPShellP__externals[i].c_name, UDPShellP__RegisterShellCommand__getCommandName(i), UDPShellP__CMDNAMSIZ);
      UDPShellP__externals[i].c_len = strlen(UDPShellP__externals[i].c_name);
    }
  UDPShellP__UDP__bind(2000);
}

# 49 "/home/tinyos/local/src/tinyos-2.x/tos/interfaces/Boot.nc"
inline static void RealMainP__Boot__booted(void ){
#line 49
  UDPShellP__Boot__booted();
#line 49
  TcpP__Boot__booted();
#line 49
  IPDispatchP__Boot__booted();
#line 49
  IPRoutingP__Boot__booted();
#line 49
  TCPEchoP__Boot__booted();
#line 49
  HttpdP__Boot__booted();
#line 49
}
#line 49
# 45 "/home/tinyos/local/src/tinyos-2.x/tos/interfaces/State.nc"
inline static error_t CC2420CsmaP__SplitControlState__requestState(uint8_t reqState){
#line 45
  unsigned char result;
#line 45

#line 45
  result = StateImplP__State__requestState(1U, reqState);
#line 45

#line 45
  return result;
#line 45
}
#line 45
# 55 "/home/tinyos/local/src/tinyos-2.x/tos/lib/timer/Alarm.nc"
inline static void CC2420ControlP__StartupTimer__start(CC2420ControlP__StartupTimer__size_type dt){
#line 55
  /*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC__1__Alarm__start(dt);
#line 55
}
#line 55
# 45 "/home/tinyos/local/src/tinyos-2.x/tos/chips/msp430/pins/HplMsp430GeneralIOP.nc"
static inline void /*HplMsp430GeneralIOC.P45*/HplMsp430GeneralIOP__29__IO__set(void )
#line 45
{
#line 45
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 45
    * (volatile uint8_t * )29U |= 0x01 << 5;
#line 45
    __nesc_atomic_end(__nesc_atomic); }
}

# 34 "/home/tinyos/local/src/tinyos-2.x/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
inline static void /*HplCC2420PinsC.VRENM*/Msp430GpioC__9__HplGeneralIO__set(void ){
#line 34
  /*HplMsp430GeneralIOC.P45*/HplMsp430GeneralIOP__29__IO__set();
#line 34
}
#line 34
# 37 "/home/tinyos/local/src/tinyos-2.x/tos/chips/msp430/pins/Msp430GpioC.nc"
static inline void /*HplCC2420PinsC.VRENM*/Msp430GpioC__9__GeneralIO__set(void )
#line 37
{
#line 37
  /*HplCC2420PinsC.VRENM*/Msp430GpioC__9__HplGeneralIO__set();
}

# 29 "/home/tinyos/local/src/tinyos-2.x/tos/interfaces/GeneralIO.nc"
inline static void CC2420ControlP__VREN__set(void ){
#line 29
  /*HplCC2420PinsC.VRENM*/Msp430GpioC__9__GeneralIO__set();
#line 29
}
#line 29
# 187 "/home/tinyos/local/src/tinyos-2.x/tos/chips/cc2420/control/CC2420ControlP.nc"
static inline error_t CC2420ControlP__CC2420Power__startVReg(void )
#line 187
{
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 188
    {
      if (CC2420ControlP__m_state != CC2420ControlP__S_VREG_STOPPED) {
          {
            unsigned char __nesc_temp = 
#line 190
            FAIL;

            {
#line 190
              __nesc_atomic_end(__nesc_atomic); 
#line 190
              return __nesc_temp;
            }
          }
        }
#line 192
      CC2420ControlP__m_state = CC2420ControlP__S_VREG_STARTING;
    }
#line 193
    __nesc_atomic_end(__nesc_atomic); }
  CC2420ControlP__VREN__set();
  CC2420ControlP__StartupTimer__start(CC2420_TIME_VREN);
  return SUCCESS;
}

# 51 "/home/tinyos/local/src/tinyos-2.x/tos/chips/cc2420/interfaces/CC2420Power.nc"
inline static error_t CC2420CsmaP__CC2420Power__startVReg(void ){
#line 51
  unsigned char result;
#line 51

#line 51
  result = CC2420ControlP__CC2420Power__startVReg();
#line 51

#line 51
  return result;
#line 51
}
#line 51
# 206 "/home/tinyos/local/src/tinyos-2.x/tos/chips/msp430/msp430hardware.h"
static inline  void __nesc_disable_interrupt(void )
{
   __asm volatile ("dint");
   __asm volatile ("nop");}

# 52 "/home/tinyos/local/src/tinyos-2.x/tos/chips/msp430/timer/Msp430ClockP.nc"
static inline mcu_power_t Msp430ClockP__McuPowerOverride__lowestState(void )
#line 52
{
  return MSP430_POWER_LPM3;
}

# 54 "/home/tinyos/local/src/tinyos-2.x/tos/interfaces/McuPowerOverride.nc"
inline static mcu_power_t McuSleepC__McuPowerOverride__lowestState(void ){
#line 54
  unsigned char result;
#line 54

#line 54
  result = Msp430ClockP__McuPowerOverride__lowestState();
#line 54

#line 54
  return result;
#line 54
}
#line 54
# 66 "/home/tinyos/local/src/tinyos-2.x/tos/chips/msp430/McuSleepC.nc"
static inline mcu_power_t McuSleepC__getPowerState(void )
#line 66
{
  mcu_power_t pState = MSP430_POWER_LPM4;









  if ((((((
#line 69
  TA0CCTL0 & 0x0010 || 
  TA0CCTL1 & 0x0010) || 
  TA0CCTL2 & 0x0010) && (
  TA0CTL & (3 << 8)) == 2 << 8) || (
  ME1 & ((1 << 7) | (1 << 6)) && U0TCTL & 0x20)) || (
  ME2 & ((1 << 5) | (1 << 4)) && U1TCTL & 0x20))


   || (U0CTLnr & 0x01 && I2CTCTLnr & 0x20 && 
  I2CDCTLnr & 0x20 && U0CTLnr & 0x04 && U0CTLnr & 0x20)) {


    pState = MSP430_POWER_LPM1;
    }


  if (ADC12CTL0 & 0x0010) {
      if (ADC12CTL1 & (2 << 3)) {

          if (ADC12CTL1 & (1 << 3)) {
            pState = MSP430_POWER_LPM1;
            }
          else {
#line 91
            pState = MSP430_POWER_ACTIVE;
            }
        }
      else {
#line 92
        if (ADC12CTL1 & 0x0400 && (TA0CTL & (3 << 8)) == 2 << 8) {



            pState = MSP430_POWER_LPM1;
          }
        }
    }

  return pState;
}

# 194 "/home/tinyos/local/src/tinyos-2.x/tos/chips/msp430/msp430hardware.h"
static inline  mcu_power_t mcombine(mcu_power_t m1, mcu_power_t m2)
#line 194
{
  return m1 < m2 ? m1 : m2;
}

# 104 "/home/tinyos/local/src/tinyos-2.x/tos/chips/msp430/McuSleepC.nc"
static inline void McuSleepC__computePowerState(void )
#line 104
{
  McuSleepC__powerState = mcombine(McuSleepC__getPowerState(), 
  McuSleepC__McuPowerOverride__lowestState());
}

static inline void McuSleepC__McuSleep__sleep(void )
#line 109
{
  uint16_t temp;

#line 111
  if (McuSleepC__dirty) {
      McuSleepC__computePowerState();
    }

  temp = McuSleepC__msp430PowerBits[McuSleepC__powerState] | 0x0008;
   __asm volatile ("bis  %0, r2" :  : "m"(temp));

   __asm volatile ("" :  :  : "memory");
  __nesc_disable_interrupt();
}

# 59 "/home/tinyos/local/src/tinyos-2.x/tos/interfaces/McuSleep.nc"
inline static void SchedulerBasicP__McuSleep__sleep(void ){
#line 59
  McuSleepC__McuSleep__sleep();
#line 59
}
#line 59
# 67 "/home/tinyos/local/src/tinyos-2.x/tos/system/SchedulerBasicP.nc"
static __inline uint8_t SchedulerBasicP__popTask(void )
{
  if (SchedulerBasicP__m_head != SchedulerBasicP__NO_TASK) 
    {
      uint8_t id = SchedulerBasicP__m_head;

#line 72
      SchedulerBasicP__m_head = SchedulerBasicP__m_next[SchedulerBasicP__m_head];
      if (SchedulerBasicP__m_head == SchedulerBasicP__NO_TASK) 
        {
          SchedulerBasicP__m_tail = SchedulerBasicP__NO_TASK;
        }
      SchedulerBasicP__m_next[id] = SchedulerBasicP__NO_TASK;
      return id;
    }
  else 
    {
      return SchedulerBasicP__NO_TASK;
    }
}

#line 138
static inline void SchedulerBasicP__Scheduler__taskLoop(void )
{
  for (; ; ) 
    {
      uint8_t nextTask;

      { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
        {
          while ((nextTask = SchedulerBasicP__popTask()) == SchedulerBasicP__NO_TASK) 
            {
              SchedulerBasicP__McuSleep__sleep();
            }
        }
#line 150
        __nesc_atomic_end(__nesc_atomic); }
      SchedulerBasicP__TaskBasic__runTask(nextTask);
    }
}

# 61 "/home/tinyos/local/src/tinyos-2.x/tos/interfaces/Scheduler.nc"
inline static void RealMainP__Scheduler__taskLoop(void ){
#line 61
  SchedulerBasicP__Scheduler__taskLoop();
#line 61
}
#line 61
# 212 "/home/tinyos/local/src/tinyos-2.x/tos/chips/cc2420/receive/CC2420ReceiveP.nc"
static inline void CC2420ReceiveP__InterruptFIFOP__fired(void )
#line 212
{
  if (CC2420ReceiveP__m_state == CC2420ReceiveP__S_STARTED) {

      CC2420ReceiveP__m_state = CC2420ReceiveP__S_RX_LENGTH;
      CC2420ReceiveP__beginReceive();
    }
  else 



    {
      CC2420ReceiveP__m_missed_packets++;
    }
}

# 57 "/home/tinyos/local/src/tinyos-2.x/tos/interfaces/GpioInterrupt.nc"
inline static void /*HplCC2420InterruptsC.InterruptFIFOPC*/Msp430InterruptC__1__Interrupt__fired(void ){
#line 57
  CC2420ReceiveP__InterruptFIFOP__fired();
#line 57
}
#line 57
# 66 "/home/tinyos/local/src/tinyos-2.x/tos/chips/msp430/pins/Msp430InterruptC.nc"
static inline void /*HplCC2420InterruptsC.InterruptFIFOPC*/Msp430InterruptC__1__HplInterrupt__fired(void )
#line 66
{
  /*HplCC2420InterruptsC.InterruptFIFOPC*/Msp430InterruptC__1__HplInterrupt__clear();
  /*HplCC2420InterruptsC.InterruptFIFOPC*/Msp430InterruptC__1__Interrupt__fired();
}

# 61 "/home/tinyos/local/src/tinyos-2.x/tos/chips/msp430/pins/HplMsp430Interrupt.nc"
inline static void HplMsp430InterruptP__Port10__fired(void ){
#line 61
  /*HplCC2420InterruptsC.InterruptFIFOPC*/Msp430InterruptC__1__HplInterrupt__fired();
#line 61
}
#line 61
# 92 "/home/tinyos/local/src/tinyos-2.x/tos/chips/msp430/pins/HplMsp430InterruptP.nc"
static inline void HplMsp430InterruptP__Port11__clear(void )
#line 92
{
#line 92
  P1IFG &= ~(1 << 1);
}

#line 68
static inline void HplMsp430InterruptP__Port11__default__fired(void )
#line 68
{
#line 68
  HplMsp430InterruptP__Port11__clear();
}

# 61 "/home/tinyos/local/src/tinyos-2.x/tos/chips/msp430/pins/HplMsp430Interrupt.nc"
inline static void HplMsp430InterruptP__Port11__fired(void ){
#line 61
  HplMsp430InterruptP__Port11__default__fired();
#line 61
}
#line 61
# 93 "/home/tinyos/local/src/tinyos-2.x/tos/chips/msp430/pins/HplMsp430InterruptP.nc"
static inline void HplMsp430InterruptP__Port12__clear(void )
#line 93
{
#line 93
  P1IFG &= ~(1 << 2);
}

#line 69
static inline void HplMsp430InterruptP__Port12__default__fired(void )
#line 69
{
#line 69
  HplMsp430InterruptP__Port12__clear();
}

# 61 "/home/tinyos/local/src/tinyos-2.x/tos/chips/msp430/pins/HplMsp430Interrupt.nc"
inline static void HplMsp430InterruptP__Port12__fired(void ){
#line 61
  HplMsp430InterruptP__Port12__default__fired();
#line 61
}
#line 61
# 94 "/home/tinyos/local/src/tinyos-2.x/tos/chips/msp430/pins/HplMsp430InterruptP.nc"
static inline void HplMsp430InterruptP__Port13__clear(void )
#line 94
{
#line 94
  P1IFG &= ~(1 << 3);
}

#line 70
static inline void HplMsp430InterruptP__Port13__default__fired(void )
#line 70
{
#line 70
  HplMsp430InterruptP__Port13__clear();
}

# 61 "/home/tinyos/local/src/tinyos-2.x/tos/chips/msp430/pins/HplMsp430Interrupt.nc"
inline static void HplMsp430InterruptP__Port13__fired(void ){
#line 61
  HplMsp430InterruptP__Port13__default__fired();
#line 61
}
#line 61
# 56 "/home/tinyos/local/src/tinyos-2.x/tos/interfaces/TaskBasic.nc"
inline static error_t CC2420CsmaP__startDone_task__postTask(void ){
#line 56
  unsigned char result;
#line 56

#line 56
  result = SchedulerBasicP__TaskBasic__postTask(CC2420CsmaP__startDone_task);
#line 56

#line 56
  return result;
#line 56
}
#line 56
# 215 "/home/tinyos/local/src/tinyos-2.x/tos/chips/cc2420/csma/CC2420CsmaP.nc"
static inline void CC2420CsmaP__CC2420Power__startOscillatorDone(void )
#line 215
{
  CC2420CsmaP__startDone_task__postTask();
}

# 76 "/home/tinyos/local/src/tinyos-2.x/tos/chips/cc2420/interfaces/CC2420Power.nc"
inline static void CC2420ControlP__CC2420Power__startOscillatorDone(void ){
#line 76
  CC2420CsmaP__CC2420Power__startOscillatorDone();
#line 76
}
#line 76
# 50 "/home/tinyos/local/src/tinyos-2.x/tos/interfaces/GpioInterrupt.nc"
inline static error_t CC2420ControlP__InterruptCCA__disable(void ){
#line 50
  unsigned char result;
#line 50

#line 50
  result = /*HplCC2420InterruptsC.InterruptCCAC*/Msp430InterruptC__0__Interrupt__disable();
#line 50

#line 50
  return result;
#line 50
}
#line 50
# 418 "/home/tinyos/local/src/tinyos-2.x/tos/chips/cc2420/control/CC2420ControlP.nc"
static inline void CC2420ControlP__InterruptCCA__fired(void )
#line 418
{
  CC2420ControlP__m_state = CC2420ControlP__S_XOSC_STARTED;
  CC2420ControlP__InterruptCCA__disable();
  CC2420ControlP__IOCFG1__write(0);
  CC2420ControlP__writeId();
  CC2420ControlP__CSN__set();
  CC2420ControlP__CSN__clr();
  CC2420ControlP__CC2420Power__startOscillatorDone();
}

# 57 "/home/tinyos/local/src/tinyos-2.x/tos/interfaces/GpioInterrupt.nc"
inline static void /*HplCC2420InterruptsC.InterruptCCAC*/Msp430InterruptC__0__Interrupt__fired(void ){
#line 57
  CC2420ControlP__InterruptCCA__fired();
#line 57
}
#line 57
# 66 "/home/tinyos/local/src/tinyos-2.x/tos/chips/msp430/pins/Msp430InterruptC.nc"
static inline void /*HplCC2420InterruptsC.InterruptCCAC*/Msp430InterruptC__0__HplInterrupt__fired(void )
#line 66
{
  /*HplCC2420InterruptsC.InterruptCCAC*/Msp430InterruptC__0__HplInterrupt__clear();
  /*HplCC2420InterruptsC.InterruptCCAC*/Msp430InterruptC__0__Interrupt__fired();
}

# 61 "/home/tinyos/local/src/tinyos-2.x/tos/chips/msp430/pins/HplMsp430Interrupt.nc"
inline static void HplMsp430InterruptP__Port14__fired(void ){
#line 61
  /*HplCC2420InterruptsC.InterruptCCAC*/Msp430InterruptC__0__HplInterrupt__fired();
#line 61
}
#line 61
# 56 "/home/tinyos/local/src/tinyos-2.x/tos/interfaces/TaskBasic.nc"
inline static error_t /*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP__0__readSensor__postTask(void ){
#line 56
  unsigned char result;
#line 56

#line 56
  result = SchedulerBasicP__TaskBasic__postTask(/*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP__0__readSensor);
#line 56

#line 56
  return result;
#line 56
}
#line 56
# 315 "/home/tinyos/local/src/tinyos-2.x/tos/chips/sht11/SensirionSht11LogicP.nc"
static inline void /*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP__0__InterruptDATA__fired(void )
#line 315
{
  /*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP__0__InterruptDATA__disable();
  /*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP__0__readSensor__postTask();
}

# 57 "/home/tinyos/local/src/tinyos-2.x/tos/interfaces/GpioInterrupt.nc"
inline static void /*HplSensirionSht11C.InterruptDATAC*/Msp430InterruptC__2__Interrupt__fired(void ){
#line 57
  /*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP__0__InterruptDATA__fired();
#line 57
}
#line 57
# 96 "/home/tinyos/local/src/tinyos-2.x/tos/chips/msp430/pins/HplMsp430InterruptP.nc"
static inline void HplMsp430InterruptP__Port15__clear(void )
#line 96
{
#line 96
  P1IFG &= ~(1 << 5);
}

# 41 "/home/tinyos/local/src/tinyos-2.x/tos/chips/msp430/pins/HplMsp430Interrupt.nc"
inline static void /*HplSensirionSht11C.InterruptDATAC*/Msp430InterruptC__2__HplInterrupt__clear(void ){
#line 41
  HplMsp430InterruptP__Port15__clear();
#line 41
}
#line 41
# 66 "/home/tinyos/local/src/tinyos-2.x/tos/chips/msp430/pins/Msp430InterruptC.nc"
static inline void /*HplSensirionSht11C.InterruptDATAC*/Msp430InterruptC__2__HplInterrupt__fired(void )
#line 66
{
  /*HplSensirionSht11C.InterruptDATAC*/Msp430InterruptC__2__HplInterrupt__clear();
  /*HplSensirionSht11C.InterruptDATAC*/Msp430InterruptC__2__Interrupt__fired();
}

# 61 "/home/tinyos/local/src/tinyos-2.x/tos/chips/msp430/pins/HplMsp430Interrupt.nc"
inline static void HplMsp430InterruptP__Port15__fired(void ){
#line 61
  /*HplSensirionSht11C.InterruptDATAC*/Msp430InterruptC__2__HplInterrupt__fired();
#line 61
}
#line 61
# 97 "/home/tinyos/local/src/tinyos-2.x/tos/chips/msp430/pins/HplMsp430InterruptP.nc"
static inline void HplMsp430InterruptP__Port16__clear(void )
#line 97
{
#line 97
  P1IFG &= ~(1 << 6);
}

#line 73
static inline void HplMsp430InterruptP__Port16__default__fired(void )
#line 73
{
#line 73
  HplMsp430InterruptP__Port16__clear();
}

# 61 "/home/tinyos/local/src/tinyos-2.x/tos/chips/msp430/pins/HplMsp430Interrupt.nc"
inline static void HplMsp430InterruptP__Port16__fired(void ){
#line 61
  HplMsp430InterruptP__Port16__default__fired();
#line 61
}
#line 61
# 98 "/home/tinyos/local/src/tinyos-2.x/tos/chips/msp430/pins/HplMsp430InterruptP.nc"
static inline void HplMsp430InterruptP__Port17__clear(void )
#line 98
{
#line 98
  P1IFG &= ~(1 << 7);
}

#line 74
static inline void HplMsp430InterruptP__Port17__default__fired(void )
#line 74
{
#line 74
  HplMsp430InterruptP__Port17__clear();
}

# 61 "/home/tinyos/local/src/tinyos-2.x/tos/chips/msp430/pins/HplMsp430Interrupt.nc"
inline static void HplMsp430InterruptP__Port17__fired(void ){
#line 61
  HplMsp430InterruptP__Port17__default__fired();
#line 61
}
#line 61
# 195 "/home/tinyos/local/src/tinyos-2.x/tos/chips/msp430/pins/HplMsp430InterruptP.nc"
static inline void HplMsp430InterruptP__Port20__clear(void )
#line 195
{
#line 195
  P2IFG &= ~(1 << 0);
}

#line 171
static inline void HplMsp430InterruptP__Port20__default__fired(void )
#line 171
{
#line 171
  HplMsp430InterruptP__Port20__clear();
}

# 61 "/home/tinyos/local/src/tinyos-2.x/tos/chips/msp430/pins/HplMsp430Interrupt.nc"
inline static void HplMsp430InterruptP__Port20__fired(void ){
#line 61
  HplMsp430InterruptP__Port20__default__fired();
#line 61
}
#line 61
# 196 "/home/tinyos/local/src/tinyos-2.x/tos/chips/msp430/pins/HplMsp430InterruptP.nc"
static inline void HplMsp430InterruptP__Port21__clear(void )
#line 196
{
#line 196
  P2IFG &= ~(1 << 1);
}

#line 172
static inline void HplMsp430InterruptP__Port21__default__fired(void )
#line 172
{
#line 172
  HplMsp430InterruptP__Port21__clear();
}

# 61 "/home/tinyos/local/src/tinyos-2.x/tos/chips/msp430/pins/HplMsp430Interrupt.nc"
inline static void HplMsp430InterruptP__Port21__fired(void ){
#line 61
  HplMsp430InterruptP__Port21__default__fired();
#line 61
}
#line 61
# 197 "/home/tinyos/local/src/tinyos-2.x/tos/chips/msp430/pins/HplMsp430InterruptP.nc"
static inline void HplMsp430InterruptP__Port22__clear(void )
#line 197
{
#line 197
  P2IFG &= ~(1 << 2);
}

#line 173
static inline void HplMsp430InterruptP__Port22__default__fired(void )
#line 173
{
#line 173
  HplMsp430InterruptP__Port22__clear();
}

# 61 "/home/tinyos/local/src/tinyos-2.x/tos/chips/msp430/pins/HplMsp430Interrupt.nc"
inline static void HplMsp430InterruptP__Port22__fired(void ){
#line 61
  HplMsp430InterruptP__Port22__default__fired();
#line 61
}
#line 61
# 198 "/home/tinyos/local/src/tinyos-2.x/tos/chips/msp430/pins/HplMsp430InterruptP.nc"
static inline void HplMsp430InterruptP__Port23__clear(void )
#line 198
{
#line 198
  P2IFG &= ~(1 << 3);
}

#line 174
static inline void HplMsp430InterruptP__Port23__default__fired(void )
#line 174
{
#line 174
  HplMsp430InterruptP__Port23__clear();
}

# 61 "/home/tinyos/local/src/tinyos-2.x/tos/chips/msp430/pins/HplMsp430Interrupt.nc"
inline static void HplMsp430InterruptP__Port23__fired(void ){
#line 61
  HplMsp430InterruptP__Port23__default__fired();
#line 61
}
#line 61
# 199 "/home/tinyos/local/src/tinyos-2.x/tos/chips/msp430/pins/HplMsp430InterruptP.nc"
static inline void HplMsp430InterruptP__Port24__clear(void )
#line 199
{
#line 199
  P2IFG &= ~(1 << 4);
}

#line 175
static inline void HplMsp430InterruptP__Port24__default__fired(void )
#line 175
{
#line 175
  HplMsp430InterruptP__Port24__clear();
}

# 61 "/home/tinyos/local/src/tinyos-2.x/tos/chips/msp430/pins/HplMsp430Interrupt.nc"
inline static void HplMsp430InterruptP__Port24__fired(void ){
#line 61
  HplMsp430InterruptP__Port24__default__fired();
#line 61
}
#line 61
# 200 "/home/tinyos/local/src/tinyos-2.x/tos/chips/msp430/pins/HplMsp430InterruptP.nc"
static inline void HplMsp430InterruptP__Port25__clear(void )
#line 200
{
#line 200
  P2IFG &= ~(1 << 5);
}

#line 176
static inline void HplMsp430InterruptP__Port25__default__fired(void )
#line 176
{
#line 176
  HplMsp430InterruptP__Port25__clear();
}

# 61 "/home/tinyos/local/src/tinyos-2.x/tos/chips/msp430/pins/HplMsp430Interrupt.nc"
inline static void HplMsp430InterruptP__Port25__fired(void ){
#line 61
  HplMsp430InterruptP__Port25__default__fired();
#line 61
}
#line 61
# 201 "/home/tinyos/local/src/tinyos-2.x/tos/chips/msp430/pins/HplMsp430InterruptP.nc"
static inline void HplMsp430InterruptP__Port26__clear(void )
#line 201
{
#line 201
  P2IFG &= ~(1 << 6);
}

#line 177
static inline void HplMsp430InterruptP__Port26__default__fired(void )
#line 177
{
#line 177
  HplMsp430InterruptP__Port26__clear();
}

# 61 "/home/tinyos/local/src/tinyos-2.x/tos/chips/msp430/pins/HplMsp430Interrupt.nc"
inline static void HplMsp430InterruptP__Port26__fired(void ){
#line 61
  HplMsp430InterruptP__Port26__default__fired();
#line 61
}
#line 61
# 202 "/home/tinyos/local/src/tinyos-2.x/tos/chips/msp430/pins/HplMsp430InterruptP.nc"
static inline void HplMsp430InterruptP__Port27__clear(void )
#line 202
{
#line 202
  P2IFG &= ~(1 << 7);
}

#line 178
static inline void HplMsp430InterruptP__Port27__default__fired(void )
#line 178
{
#line 178
  HplMsp430InterruptP__Port27__clear();
}

# 61 "/home/tinyos/local/src/tinyos-2.x/tos/chips/msp430/pins/HplMsp430Interrupt.nc"
inline static void HplMsp430InterruptP__Port27__fired(void ){
#line 61
  HplMsp430InterruptP__Port27__default__fired();
#line 61
}
#line 61
# 88 "/home/tinyos/local/src/tinyos-2.x/tos/interfaces/ArbiterInfo.nc"
inline static uint8_t /*Msp430UsartShare0P.UsartShareP*/Msp430UsartShareP__0__ArbiterInfo__userId(void ){
#line 88
  unsigned char result;
#line 88

#line 88
  result = /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0__ArbiterInfo__userId();
#line 88

#line 88
  return result;
#line 88
}
#line 88
# 195 "/home/tinyos/local/src/tinyos-2.x/tos/chips/msp430/usart/Msp430SpiDmaP.nc"
static inline void /*Msp430SpiDma0P.SpiP*/Msp430SpiDmaP__0__UsartInterrupts__rxDone(uint8_t data)
#line 195
{
}

# 65 "/home/tinyos/local/src/tinyos-2.x/tos/chips/msp430/usart/Msp430UsartShareP.nc"
static inline void /*Msp430UsartShare0P.UsartShareP*/Msp430UsartShareP__0__Interrupts__default__rxDone(uint8_t id, uint8_t data)
#line 65
{
}

# 54 "/home/tinyos/local/src/tinyos-2.x/tos/chips/msp430/usart/HplMsp430UsartInterrupts.nc"
inline static void /*Msp430UsartShare0P.UsartShareP*/Msp430UsartShareP__0__Interrupts__rxDone(uint8_t arg_0x40df4cf8, uint8_t data){
#line 54
  switch (arg_0x40df4cf8) {
#line 54
    case /*CC2420SpiWireC.HplCC2420SpiC.SpiC.UsartC*/Msp430Usart0C__0__CLIENT_ID:
#line 54
      /*Msp430SpiDma0P.SpiP*/Msp430SpiDmaP__0__UsartInterrupts__rxDone(data);
#line 54
      break;
#line 54
    default:
#line 54
      /*Msp430UsartShare0P.UsartShareP*/Msp430UsartShareP__0__Interrupts__default__rxDone(arg_0x40df4cf8, data);
#line 54
      break;
#line 54
    }
#line 54
}
#line 54
# 80 "/home/tinyos/local/src/tinyos-2.x/tos/interfaces/ArbiterInfo.nc"
inline static bool /*Msp430UsartShare0P.UsartShareP*/Msp430UsartShareP__0__ArbiterInfo__inUse(void ){
#line 80
  unsigned char result;
#line 80

#line 80
  result = /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0__ArbiterInfo__inUse();
#line 80

#line 80
  return result;
#line 80
}
#line 80
# 54 "/home/tinyos/local/src/tinyos-2.x/tos/chips/msp430/usart/Msp430UsartShareP.nc"
static inline void /*Msp430UsartShare0P.UsartShareP*/Msp430UsartShareP__0__RawInterrupts__rxDone(uint8_t data)
#line 54
{
  if (/*Msp430UsartShare0P.UsartShareP*/Msp430UsartShareP__0__ArbiterInfo__inUse()) {
    /*Msp430UsartShare0P.UsartShareP*/Msp430UsartShareP__0__Interrupts__rxDone(/*Msp430UsartShare0P.UsartShareP*/Msp430UsartShareP__0__ArbiterInfo__userId(), data);
    }
}

# 54 "/home/tinyos/local/src/tinyos-2.x/tos/chips/msp430/usart/HplMsp430UsartInterrupts.nc"
inline static void HplMsp430Usart0P__Interrupts__rxDone(uint8_t data){
#line 54
  /*Msp430UsartShare0P.UsartShareP*/Msp430UsartShareP__0__RawInterrupts__rxDone(data);
#line 54
}
#line 54
# 55 "/home/tinyos/local/src/tinyos-2.x/tos/chips/msp430/usart/HplMsp430I2C0P.nc"
static inline bool HplMsp430I2C0P__HplI2C__isI2C(void )
#line 55
{
  /* atomic removed: atomic calls only */
#line 56
  {
    unsigned char __nesc_temp = 
#line 56
    HplMsp430I2C0P__U0CTL & 0x20 && HplMsp430I2C0P__U0CTL & 0x04 && HplMsp430I2C0P__U0CTL & 0x01;

#line 56
    return __nesc_temp;
  }
}

# 6 "/home/tinyos/local/src/tinyos-2.x/tos/chips/msp430/usart/HplMsp430I2C.nc"
inline static bool HplMsp430Usart0P__HplI2C__isI2C(void ){
#line 6
  unsigned char result;
#line 6

#line 6
  result = HplMsp430I2C0P__HplI2C__isI2C();
#line 6

#line 6
  return result;
#line 6
}
#line 6
# 66 "/home/tinyos/local/src/tinyos-2.x/tos/chips/msp430/usart/Msp430UsartShareP.nc"
static inline void /*Msp430UsartShare0P.UsartShareP*/Msp430UsartShareP__0__I2CInterrupts__default__fired(uint8_t id)
#line 66
{
}

# 39 "/home/tinyos/local/src/tinyos-2.x/tos/chips/msp430/usart/HplMsp430I2CInterrupts.nc"
inline static void /*Msp430UsartShare0P.UsartShareP*/Msp430UsartShareP__0__I2CInterrupts__fired(uint8_t arg_0x40e245c8){
#line 39
    /*Msp430UsartShare0P.UsartShareP*/Msp430UsartShareP__0__I2CInterrupts__default__fired(arg_0x40e245c8);
#line 39
}
#line 39
# 59 "/home/tinyos/local/src/tinyos-2.x/tos/chips/msp430/usart/Msp430UsartShareP.nc"
static inline void /*Msp430UsartShare0P.UsartShareP*/Msp430UsartShareP__0__RawI2CInterrupts__fired(void )
#line 59
{
  if (/*Msp430UsartShare0P.UsartShareP*/Msp430UsartShareP__0__ArbiterInfo__inUse()) {
    /*Msp430UsartShare0P.UsartShareP*/Msp430UsartShareP__0__I2CInterrupts__fired(/*Msp430UsartShare0P.UsartShareP*/Msp430UsartShareP__0__ArbiterInfo__userId());
    }
}

# 39 "/home/tinyos/local/src/tinyos-2.x/tos/chips/msp430/usart/HplMsp430I2CInterrupts.nc"
inline static void HplMsp430Usart0P__I2CInterrupts__fired(void ){
#line 39
  /*Msp430UsartShare0P.UsartShareP*/Msp430UsartShareP__0__RawI2CInterrupts__fired();
#line 39
}
#line 39
# 194 "/home/tinyos/local/src/tinyos-2.x/tos/chips/msp430/usart/Msp430SpiDmaP.nc"
static inline void /*Msp430SpiDma0P.SpiP*/Msp430SpiDmaP__0__UsartInterrupts__txDone(void )
#line 194
{
}

# 64 "/home/tinyos/local/src/tinyos-2.x/tos/chips/msp430/usart/Msp430UsartShareP.nc"
static inline void /*Msp430UsartShare0P.UsartShareP*/Msp430UsartShareP__0__Interrupts__default__txDone(uint8_t id)
#line 64
{
}

# 49 "/home/tinyos/local/src/tinyos-2.x/tos/chips/msp430/usart/HplMsp430UsartInterrupts.nc"
inline static void /*Msp430UsartShare0P.UsartShareP*/Msp430UsartShareP__0__Interrupts__txDone(uint8_t arg_0x40df4cf8){
#line 49
  switch (arg_0x40df4cf8) {
#line 49
    case /*CC2420SpiWireC.HplCC2420SpiC.SpiC.UsartC*/Msp430Usart0C__0__CLIENT_ID:
#line 49
      /*Msp430SpiDma0P.SpiP*/Msp430SpiDmaP__0__UsartInterrupts__txDone();
#line 49
      break;
#line 49
    default:
#line 49
      /*Msp430UsartShare0P.UsartShareP*/Msp430UsartShareP__0__Interrupts__default__txDone(arg_0x40df4cf8);
#line 49
      break;
#line 49
    }
#line 49
}
#line 49
# 49 "/home/tinyos/local/src/tinyos-2.x/tos/chips/msp430/usart/Msp430UsartShareP.nc"
static inline void /*Msp430UsartShare0P.UsartShareP*/Msp430UsartShareP__0__RawInterrupts__txDone(void )
#line 49
{
  if (/*Msp430UsartShare0P.UsartShareP*/Msp430UsartShareP__0__ArbiterInfo__inUse()) {
    /*Msp430UsartShare0P.UsartShareP*/Msp430UsartShareP__0__Interrupts__txDone(/*Msp430UsartShare0P.UsartShareP*/Msp430UsartShareP__0__ArbiterInfo__userId());
    }
}

# 49 "/home/tinyos/local/src/tinyos-2.x/tos/chips/msp430/usart/HplMsp430UsartInterrupts.nc"
inline static void HplMsp430Usart0P__Interrupts__txDone(void ){
#line 49
  /*Msp430UsartShare0P.UsartShareP*/Msp430UsartShareP__0__RawInterrupts__txDone();
#line 49
}
#line 49
# 98 "/home/tinyos/local/src/tinyos-2.x/tos/chips/msp430/dma/Msp430DmaControlP.nc"
static inline void Msp430DmaControlP__HplChannel2__transferDone(error_t error)
#line 98
{
}

# 188 "/home/tinyos/local/src/tinyos-2.x/tos/chips/msp430/usart/Msp430SpiDmaP.nc"
static inline void /*Msp430SpiDma0P.SpiP*/Msp430SpiDmaP__0__DmaChannel2__transferDone(error_t error)
#line 188
{
}

# 96 "/home/tinyos/local/src/tinyos-2.x/tos/chips/msp430/dma/Msp430DmaChannel.nc"
inline static void /*Msp430DmaC.Channel2P*/Msp430DmaChannelP__2__Channel__transferDone(error_t success){
#line 96
  /*Msp430SpiDma0P.SpiP*/Msp430SpiDmaP__0__DmaChannel2__transferDone(success);
#line 96
}
#line 96
# 143 "/home/tinyos/local/src/tinyos-2.x/tos/chips/msp430/dma/Msp430DmaChannelP.nc"
static inline void /*Msp430DmaC.Channel2P*/Msp430DmaChannelP__2__HplChannel__transferDone(error_t error)
#line 143
{
  /*Msp430DmaC.Channel2P*/Msp430DmaChannelP__2__Channel__transferDone(error);
}

# 74 "/home/tinyos/local/src/tinyos-2.x/tos/chips/msp430/dma/HplMsp430DmaChannel.nc"
inline static void /*HplMsp430DmaC.Dma2*/HplMsp430DmaXP__2__DMA__transferDone(error_t success){
#line 74
  /*Msp430DmaC.Channel2P*/Msp430DmaChannelP__2__HplChannel__transferDone(success);
#line 74
  Msp430DmaControlP__HplChannel2__transferDone(success);
#line 74
}
#line 74
# 82 "/home/tinyos/local/src/tinyos-2.x/tos/chips/msp430/dma/HplMsp430DmaXP.nc"
static inline void /*HplMsp430DmaC.Dma2*/HplMsp430DmaXP__2__Interrupt__fired(void )
#line 82
{
  error_t error = * (volatile uint16_t *)496U & 0x0002 ? FAIL : SUCCESS;

#line 84
  if (* (volatile uint16_t *)496U & 0x0008) {
      * (volatile uint16_t *)496U &= ~0x0008;
      * (volatile uint16_t *)496U &= ~0x0002;
      /*HplMsp430DmaC.Dma2*/HplMsp430DmaXP__2__DMA__transferDone(error);
    }
}

# 97 "/home/tinyos/local/src/tinyos-2.x/tos/chips/msp430/dma/Msp430DmaControlP.nc"
static inline void Msp430DmaControlP__HplChannel1__transferDone(error_t error)
#line 97
{
}

# 184 "/home/tinyos/local/src/tinyos-2.x/tos/chips/msp430/usart/Msp430SpiDmaP.nc"
static inline void /*Msp430SpiDma0P.SpiP*/Msp430SpiDmaP__0__DmaChannel1__transferDone(error_t error)
#line 184
{
  /*Msp430SpiDma0P.SpiP*/Msp430SpiDmaP__0__signalDone(error);
}

# 96 "/home/tinyos/local/src/tinyos-2.x/tos/chips/msp430/dma/Msp430DmaChannel.nc"
inline static void /*Msp430DmaC.Channel1P*/Msp430DmaChannelP__1__Channel__transferDone(error_t success){
#line 96
  /*Msp430SpiDma0P.SpiP*/Msp430SpiDmaP__0__DmaChannel1__transferDone(success);
#line 96
}
#line 96
# 143 "/home/tinyos/local/src/tinyos-2.x/tos/chips/msp430/dma/Msp430DmaChannelP.nc"
static inline void /*Msp430DmaC.Channel1P*/Msp430DmaChannelP__1__HplChannel__transferDone(error_t error)
#line 143
{
  /*Msp430DmaC.Channel1P*/Msp430DmaChannelP__1__Channel__transferDone(error);
}

# 74 "/home/tinyos/local/src/tinyos-2.x/tos/chips/msp430/dma/HplMsp430DmaChannel.nc"
inline static void /*HplMsp430DmaC.Dma1*/HplMsp430DmaXP__1__DMA__transferDone(error_t success){
#line 74
  /*Msp430DmaC.Channel1P*/Msp430DmaChannelP__1__HplChannel__transferDone(success);
#line 74
  Msp430DmaControlP__HplChannel1__transferDone(success);
#line 74
}
#line 74
# 82 "/home/tinyos/local/src/tinyos-2.x/tos/chips/msp430/dma/HplMsp430DmaXP.nc"
static inline void /*HplMsp430DmaC.Dma1*/HplMsp430DmaXP__1__Interrupt__fired(void )
#line 82
{
  error_t error = * (volatile uint16_t *)488U & 0x0002 ? FAIL : SUCCESS;

#line 84
  if (* (volatile uint16_t *)488U & 0x0008) {
      * (volatile uint16_t *)488U &= ~0x0008;
      * (volatile uint16_t *)488U &= ~0x0002;
      /*HplMsp430DmaC.Dma1*/HplMsp430DmaXP__1__DMA__transferDone(error);
    }
}

# 96 "/home/tinyos/local/src/tinyos-2.x/tos/chips/msp430/dma/Msp430DmaControlP.nc"
static inline void Msp430DmaControlP__HplChannel0__transferDone(error_t error)
#line 96
{
}

# 147 "/home/tinyos/local/src/tinyos-2.x/tos/chips/msp430/dma/Msp430DmaChannelP.nc"
static inline void /*Msp430DmaC.Channel0P*/Msp430DmaChannelP__0__Channel__default__transferDone(error_t error)
#line 147
{
}

# 96 "/home/tinyos/local/src/tinyos-2.x/tos/chips/msp430/dma/Msp430DmaChannel.nc"
inline static void /*Msp430DmaC.Channel0P*/Msp430DmaChannelP__0__Channel__transferDone(error_t success){
#line 96
  /*Msp430DmaC.Channel0P*/Msp430DmaChannelP__0__Channel__default__transferDone(success);
#line 96
}
#line 96
# 143 "/home/tinyos/local/src/tinyos-2.x/tos/chips/msp430/dma/Msp430DmaChannelP.nc"
static inline void /*Msp430DmaC.Channel0P*/Msp430DmaChannelP__0__HplChannel__transferDone(error_t error)
#line 143
{
  /*Msp430DmaC.Channel0P*/Msp430DmaChannelP__0__Channel__transferDone(error);
}

# 74 "/home/tinyos/local/src/tinyos-2.x/tos/chips/msp430/dma/HplMsp430DmaChannel.nc"
inline static void /*HplMsp430DmaC.Dma0*/HplMsp430DmaXP__0__DMA__transferDone(error_t success){
#line 74
  /*Msp430DmaC.Channel0P*/Msp430DmaChannelP__0__HplChannel__transferDone(success);
#line 74
  Msp430DmaControlP__HplChannel0__transferDone(success);
#line 74
}
#line 74
# 82 "/home/tinyos/local/src/tinyos-2.x/tos/chips/msp430/dma/HplMsp430DmaXP.nc"
static inline void /*HplMsp430DmaC.Dma0*/HplMsp430DmaXP__0__Interrupt__fired(void )
#line 82
{
  error_t error = * (volatile uint16_t *)480U & 0x0002 ? FAIL : SUCCESS;

#line 84
  if (* (volatile uint16_t *)480U & 0x0008) {
      * (volatile uint16_t *)480U &= ~0x0008;
      * (volatile uint16_t *)480U &= ~0x0002;
      /*HplMsp430DmaC.Dma0*/HplMsp430DmaXP__0__DMA__transferDone(error);
    }
}

# 28 "/home/tinyos/local/src/tinyos-2.x/tos/chips/msp430/dma/HplMsp430DmaInterrupt.nc"
inline static void HplMsp430DmaP__Interrupt__fired(void ){
#line 28
  /*HplMsp430DmaC.Dma0*/HplMsp430DmaXP__0__Interrupt__fired();
#line 28
  /*HplMsp430DmaC.Dma1*/HplMsp430DmaXP__1__Interrupt__fired();
#line 28
  /*HplMsp430DmaC.Dma2*/HplMsp430DmaXP__2__Interrupt__fired();
#line 28
}
#line 28
# 109 "/home/tinyos/local/src/tinyos-2.x/tos/lib/net/blip/shell/UDPShellP.nc"
static inline void UDPShellP__action_help(int argc, char **argv)
#line 109
{
  int i = 0;
  char *pos = UDPShellP__reply_buf;

#line 112
  UDPShellP__UDP__sendto(&UDPShellP__session_endpoint, UDPShellP__help_str, strlen(UDPShellP__help_str));
  if (UDPShellP__N_EXTERNAL > 0) {
      strcpy(pos, "\t\t[");
      pos += 3;
      for (i = 0; i < UDPShellP__N_EXTERNAL; i++) {
          if (UDPShellP__externals[i].c_len + 4 < MAX_REPLY_LEN - (pos - UDPShellP__reply_buf)) {
              ip_memcpy(pos, UDPShellP__externals[i].c_name, UDPShellP__externals[i].c_len);
              pos += UDPShellP__externals[i].c_len;
              if (i < UDPShellP__N_EXTERNAL - 1) {
                  pos[0] = ',';
                  pos[1] = ' ';
                  pos += 2;
                }
            }
          else 
#line 125
            {
              pos[0] = '.';
              pos[1] = '.';
              pos[2] = '.';
              pos += 3;
              break;
            }
        }
      * pos++ = ']';
      * pos++ = '\n';
      UDPShellP__UDP__sendto(&UDPShellP__session_endpoint, UDPShellP__reply_buf, pos - UDPShellP__reply_buf);
    }
}

# 53 "/home/tinyos/local/src/tinyos-2.x/tos/lib/timer/Timer.nc"
inline static void ICMPResponderP__PingTimer__startPeriodic(uint32_t dt){
#line 53
  /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__Timer__startPeriodic(6U, dt);
#line 53
}
#line 53
#line 81
inline static bool ICMPResponderP__PingTimer__isRunning(void ){
#line 81
  unsigned char result;
#line 81

#line 81
  result = /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__Timer__isRunning(6U);
#line 81

#line 81
  return result;
#line 81
}
#line 81
# 405 "/home/tinyos/local/src/tinyos-2.x/tos/lib/net/blip/ICMPResponderP.nc"
static inline error_t ICMPResponderP__ICMPPing__ping(uint16_t client, struct in6_addr *target, uint16_t period, uint16_t n)
#line 405
{
  if (ICMPResponderP__PingTimer__isRunning()) {
#line 406
    return ERETRY;
    }
#line 407
  ICMPResponderP__PingTimer__startPeriodic(period);

  ip_memcpy(&ICMPResponderP__ping_dest, target, 16);
  ICMPResponderP__ping_n = n;
  ICMPResponderP__ping_seq = 0;
  ICMPResponderP__ping_rcv = 0;
  ICMPResponderP__ping_ident = client;
  return SUCCESS;
}

# 6 "/home/tinyos/local/src/tinyos-2.x/tos/lib/net/blip/interfaces/ICMPPing.nc"
inline static error_t UDPShellP__ICMPPing__ping(struct in6_addr *target, uint16_t period, uint16_t n){
#line 6
  unsigned char result;
#line 6

#line 6
  result = ICMPResponderP__ICMPPing__ping(0U, target, period, n);
#line 6

#line 6
  return result;
#line 6
}
#line 6
# 188 "/home/tinyos/local/src/tinyos-2.x/tos/lib/net/blip/shell/UDPShellP.nc"
static inline void UDPShellP__action_ident(int argc, char **argv)
#line 188
{
  UDPShellP__UDP__sendto(&UDPShellP__session_endpoint, UDPShellP__ident_string, strlen(UDPShellP__ident_string));
}

# 226 "/home/tinyos/local/src/tinyos-2.x/tos/chips/msp430/msp430hardware.h"
  __nesc_atomic_t __nesc_atomic_start(void )
{
  __nesc_atomic_t result = (({
#line 228
    uint16_t __x;

#line 228
     __asm volatile ("mov	r2, %0" : "=r"((uint16_t )__x));__x;
  }
  )
#line 228
   & 0x0008) != 0;

#line 229
  __nesc_disable_interrupt();
   __asm volatile ("" :  :  : "memory");
  return result;
}

  void __nesc_atomic_end(__nesc_atomic_t reenable_interrupts)
{
   __asm volatile ("" :  :  : "memory");
  if (reenable_interrupts) {
    __nesc_enable_interrupt();
    }
}

# 11 "/home/tinyos/local/src/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerCommonP.nc"
__attribute((wakeup)) __attribute((interrupt(12)))  void sig_TIMERA0_VECTOR(void )
#line 11
{
#line 11
  Msp430TimerCommonP__VectorTimerA0__fired();
}

# 169 "/home/tinyos/local/src/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static void /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0__Event__fired(void )
{
  if (/*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0__Control__getControl().cap) {
    /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0__Capture__captured(/*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0__Capture__getEvent());
    }
  else {
#line 174
    /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP__0__Compare__fired();
    }
}

#line 169
static void /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP__1__Event__fired(void )
{
  if (/*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP__1__Control__getControl().cap) {
    /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP__1__Capture__captured(/*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP__1__Capture__getEvent());
    }
  else {
#line 174
    /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP__1__Compare__fired();
    }
}

#line 169
static void /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP__2__Event__fired(void )
{
  if (/*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP__2__Control__getControl().cap) {
    /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP__2__Capture__captured(/*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP__2__Capture__getEvent());
    }
  else {
#line 174
    /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP__2__Compare__fired();
    }
}

# 12 "/home/tinyos/local/src/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerCommonP.nc"
__attribute((wakeup)) __attribute((interrupt(10)))  void sig_TIMERA1_VECTOR(void )
#line 12
{
#line 12
  Msp430TimerCommonP__VectorTimerA1__fired();
}

#line 13
__attribute((wakeup)) __attribute((interrupt(26)))  void sig_TIMERB0_VECTOR(void )
#line 13
{
#line 13
  Msp430TimerCommonP__VectorTimerB0__fired();
}

# 135 "/home/tinyos/local/src/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerP.nc"
static void /*Msp430TimerC.Msp430TimerB*/Msp430TimerP__1__Event__default__fired(uint8_t n)
{
}

# 28 "/home/tinyos/local/src/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerEvent.nc"
static void /*Msp430TimerC.Msp430TimerB*/Msp430TimerP__1__Event__fired(uint8_t arg_0x406403c8){
#line 28
  switch (arg_0x406403c8) {
#line 28
    case 0:
#line 28
      /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP__3__Event__fired();
#line 28
      break;
#line 28
    case 1:
#line 28
      /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP__4__Event__fired();
#line 28
      break;
#line 28
    case 2:
#line 28
      /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP__5__Event__fired();
#line 28
      break;
#line 28
    case 3:
#line 28
      /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP__6__Event__fired();
#line 28
      break;
#line 28
    case 4:
#line 28
      /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP__7__Event__fired();
#line 28
      break;
#line 28
    case 5:
#line 28
      /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP__8__Event__fired();
#line 28
      break;
#line 28
    case 6:
#line 28
      /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP__9__Event__fired();
#line 28
      break;
#line 28
    case 7:
#line 28
      /*Msp430TimerC.Msp430TimerB*/Msp430TimerP__1__Overflow__fired();
#line 28
      break;
#line 28
    default:
#line 28
      /*Msp430TimerC.Msp430TimerB*/Msp430TimerP__1__Event__default__fired(arg_0x406403c8);
#line 28
      break;
#line 28
    }
#line 28
}
#line 28
# 159 "/home/tinyos/local/src/tinyos-2.x/tos/system/SchedulerBasicP.nc"
static error_t SchedulerBasicP__TaskBasic__postTask(uint8_t id)
{
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 161
    {
#line 161
      {
        unsigned char __nesc_temp = 
#line 161
        SchedulerBasicP__pushTask(id) ? SUCCESS : EBUSY;

        {
#line 161
          __nesc_atomic_end(__nesc_atomic); 
#line 161
          return __nesc_temp;
        }
      }
    }
#line 164
    __nesc_atomic_end(__nesc_atomic); }
}

# 96 "/home/tinyos/local/src/tinyos-2.x/tos/lib/timer/TransformAlarmC.nc"
static void /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__set_alarm(void )
{
  /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__to_size_type now = /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__Counter__get();
#line 98
  /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__to_size_type expires;
#line 98
  /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__to_size_type remaining;




  expires = /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__m_t0 + /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__m_dt;


  remaining = (/*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__to_size_type )(expires - now);


  if (/*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__m_t0 <= now) 
    {
      if (expires >= /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__m_t0 && 
      expires <= now) {
        remaining = 0;
        }
    }
  else {
      if (expires >= /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__m_t0 || 
      expires <= now) {
        remaining = 0;
        }
    }
#line 121
  if (remaining > /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__MAX_DELAY) 
    {
      /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__m_t0 = now + /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__MAX_DELAY;
      /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__m_dt = remaining - /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__MAX_DELAY;
      remaining = /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__MAX_DELAY;
    }
  else 
    {
      /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__m_t0 += /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__m_dt;
      /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__m_dt = 0;
    }
  /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__AlarmFrom__startAt((/*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__from_size_type )now << 5, 
  (/*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__from_size_type )remaining << 5);
}

# 69 "/home/tinyos/local/src/tinyos-2.x/tos/lib/timer/TransformCounterC.nc"
static /*CounterMilli32C.Transform*/TransformCounterC__0__to_size_type /*CounterMilli32C.Transform*/TransformCounterC__0__Counter__get(void )
{
  /*CounterMilli32C.Transform*/TransformCounterC__0__to_size_type rv = 0;

#line 72
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
    {
      /*CounterMilli32C.Transform*/TransformCounterC__0__upper_count_type high = /*CounterMilli32C.Transform*/TransformCounterC__0__m_upper;
      /*CounterMilli32C.Transform*/TransformCounterC__0__from_size_type low = /*CounterMilli32C.Transform*/TransformCounterC__0__CounterFrom__get();

#line 76
      if (/*CounterMilli32C.Transform*/TransformCounterC__0__CounterFrom__isOverflowPending()) 
        {






          high++;
          low = /*CounterMilli32C.Transform*/TransformCounterC__0__CounterFrom__get();
        }
      {
        /*CounterMilli32C.Transform*/TransformCounterC__0__to_size_type high_to = high;
        /*CounterMilli32C.Transform*/TransformCounterC__0__to_size_type low_to = low >> /*CounterMilli32C.Transform*/TransformCounterC__0__LOW_SHIFT_RIGHT;

#line 90
        rv = (high_to << /*CounterMilli32C.Transform*/TransformCounterC__0__HIGH_SHIFT_LEFT) | low_to;
      }
    }
#line 92
    __nesc_atomic_end(__nesc_atomic); }
  return rv;
}

# 51 "/home/tinyos/local/src/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerP.nc"
static uint16_t /*Msp430TimerC.Msp430TimerB*/Msp430TimerP__1__Timer__get(void )
{




  if (1) {
      /* atomic removed: atomic calls only */
#line 58
      {
        uint16_t t0;
        uint16_t t1 = * (volatile uint16_t * )400U;

#line 61
        do {
#line 61
            t0 = t1;
#line 61
            t1 = * (volatile uint16_t * )400U;
          }
        while (
#line 61
        t0 != t1);
        {
          unsigned int __nesc_temp = 
#line 62
          t1;

#line 62
          return __nesc_temp;
        }
      }
    }
  else 
#line 65
    {
      return * (volatile uint16_t * )400U;
    }
}

# 69 "/home/tinyos/local/src/tinyos-2.x/tos/lib/timer/TransformCounterC.nc"
static /*Counter32khz32C.Transform*/TransformCounterC__1__to_size_type /*Counter32khz32C.Transform*/TransformCounterC__1__Counter__get(void )
{
  /*Counter32khz32C.Transform*/TransformCounterC__1__to_size_type rv = 0;

#line 72
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
    {
      /*Counter32khz32C.Transform*/TransformCounterC__1__upper_count_type high = /*Counter32khz32C.Transform*/TransformCounterC__1__m_upper;
      /*Counter32khz32C.Transform*/TransformCounterC__1__from_size_type low = /*Counter32khz32C.Transform*/TransformCounterC__1__CounterFrom__get();

#line 76
      if (/*Counter32khz32C.Transform*/TransformCounterC__1__CounterFrom__isOverflowPending()) 
        {






          high++;
          low = /*Counter32khz32C.Transform*/TransformCounterC__1__CounterFrom__get();
        }
      {
        /*Counter32khz32C.Transform*/TransformCounterC__1__to_size_type high_to = high;
        /*Counter32khz32C.Transform*/TransformCounterC__1__to_size_type low_to = low >> /*Counter32khz32C.Transform*/TransformCounterC__1__LOW_SHIFT_RIGHT;

#line 90
        rv = (high_to << /*Counter32khz32C.Transform*/TransformCounterC__1__HIGH_SHIFT_LEFT) | low_to;
      }
    }
#line 92
    __nesc_atomic_end(__nesc_atomic); }
  return rv;
}

# 38 "/home/tinyos/local/src/tinyos-2.x/tos/chips/msp430/timer/GpioCaptureC.nc"
static error_t /*HplCC2420InterruptsC.CaptureSFDC*/GpioCaptureC__0__enableCapture(uint8_t mode)
#line 38
{
  /* atomic removed: atomic calls only */
#line 39
  {
    /*HplCC2420InterruptsC.CaptureSFDC*/GpioCaptureC__0__Msp430TimerControl__disableEvents();
    /*HplCC2420InterruptsC.CaptureSFDC*/GpioCaptureC__0__GeneralIO__selectModuleFunc();
    /*HplCC2420InterruptsC.CaptureSFDC*/GpioCaptureC__0__Msp430TimerControl__clearPendingInterrupt();
    /*HplCC2420InterruptsC.CaptureSFDC*/GpioCaptureC__0__Msp430Capture__clearOverflow();
    /*HplCC2420InterruptsC.CaptureSFDC*/GpioCaptureC__0__Msp430TimerControl__setControlAsCapture(mode);
    /*HplCC2420InterruptsC.CaptureSFDC*/GpioCaptureC__0__Msp430TimerControl__enableEvents();
  }
  return SUCCESS;
}

# 46 "/home/tinyos/local/src/tinyos-2.x/tos/chips/msp430/pins/HplMsp430GeneralIOP.nc"
static void /*HplMsp430GeneralIOC.P42*/HplMsp430GeneralIOP__26__IO__clr(void )
#line 46
{
#line 46
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 46
    * (volatile uint8_t * )29U &= ~(0x01 << 2);
#line 46
    __nesc_atomic_end(__nesc_atomic); }
}

# 260 "/home/tinyos/local/src/tinyos-2.x/tos/chips/cc2420/spi/CC2420SpiP.nc"
static cc2420_status_t CC2420SpiP__Ram__write(uint16_t addr, uint8_t offset, 
uint8_t *data, 
uint8_t len)
#line 262
{

  cc2420_status_t status = 0;
  uint8_t tmpLen = len;
  uint8_t * tmpData = (uint8_t * )data;

  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 268
    {
      if (CC2420SpiP__WorkingState__isIdle()) {
          {
            unsigned char __nesc_temp = 
#line 270
            status;

            {
#line 270
              __nesc_atomic_end(__nesc_atomic); 
#line 270
              return __nesc_temp;
            }
          }
        }
    }
#line 274
    __nesc_atomic_end(__nesc_atomic); }
#line 274
  addr += offset;

  status = CC2420SpiP__SpiByte__write(addr | 0x80);
  CC2420SpiP__SpiByte__write((addr >> 1) & 0xc0);
  for (; len; len--) {
      CC2420SpiP__SpiByte__write(tmpData[tmpLen - len]);
    }

  return status;
}

# 133 "/home/tinyos/local/src/tinyos-2.x/tos/system/StateImplP.nc"
static bool StateImplP__State__isState(uint8_t id, uint8_t myState)
#line 133
{
  bool isState;

#line 135
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 135
    isState = StateImplP__state[id] == myState;
#line 135
    __nesc_atomic_end(__nesc_atomic); }
  return isState;
}

# 116 "/home/tinyos/local/src/tinyos-2.x/tos/chips/msp430/usart/Msp430SpiDmaP.nc"
static uint8_t /*Msp430SpiDma0P.SpiP*/Msp430SpiDmaP__0__SpiByte__write(uint8_t tx)
#line 116
{

  /*Msp430SpiDma0P.SpiP*/Msp430SpiDmaP__0__Usart__tx(tx);
  while (!/*Msp430SpiDma0P.SpiP*/Msp430SpiDmaP__0__Usart__isRxIntrPending()) ;
  /*Msp430SpiDma0P.SpiP*/Msp430SpiDmaP__0__Usart__clrRxIntr();
  return /*Msp430SpiDma0P.SpiP*/Msp430SpiDmaP__0__Usart__rx();
}

# 45 "/home/tinyos/local/src/tinyos-2.x/tos/chips/msp430/pins/HplMsp430GeneralIOP.nc"
static void /*HplMsp430GeneralIOC.P42*/HplMsp430GeneralIOP__26__IO__set(void )
#line 45
{
#line 45
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 45
    * (volatile uint8_t * )29U |= 0x01 << 2;
#line 45
    __nesc_atomic_end(__nesc_atomic); }
}

# 149 "/home/tinyos/local/src/tinyos-2.x/tos/chips/cc2420/spi/CC2420SpiP.nc"
static error_t CC2420SpiP__Resource__release(uint8_t id)
#line 149
{
  uint8_t i;

#line 151
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 151
    {
      if (CC2420SpiP__m_holder != id) {
          {
            unsigned char __nesc_temp = 
#line 153
            FAIL;

            {
#line 153
              __nesc_atomic_end(__nesc_atomic); 
#line 153
              return __nesc_temp;
            }
          }
        }
#line 156
      CC2420SpiP__m_holder = CC2420SpiP__NO_HOLDER;
      if (!CC2420SpiP__m_requests) {
          CC2420SpiP__WorkingState__toIdle();
          CC2420SpiP__attemptRelease();
        }
      else {
          for (i = CC2420SpiP__m_holder + 1; ; i++) {
              i %= CC2420SpiP__RESOURCE_COUNT;

              if (CC2420SpiP__m_requests & (1 << i)) {
                  CC2420SpiP__m_holder = i;
                  CC2420SpiP__m_requests &= ~(1 << i);
                  CC2420SpiP__grant__postTask();
                  {
                    unsigned char __nesc_temp = 
#line 169
                    SUCCESS;

                    {
#line 169
                      __nesc_atomic_end(__nesc_atomic); 
#line 169
                      return __nesc_temp;
                    }
                  }
                }
            }
        }
    }
#line 175
    __nesc_atomic_end(__nesc_atomic); }
#line 175
  return SUCCESS;
}

#line 339
static error_t CC2420SpiP__attemptRelease(void )
#line 339
{


  if ((
#line 340
  CC2420SpiP__m_requests > 0
   || CC2420SpiP__m_holder != CC2420SpiP__NO_HOLDER)
   || !CC2420SpiP__WorkingState__isIdle()) {
      return FAIL;
    }
  /* atomic removed: atomic calls only */
  CC2420SpiP__release = TRUE;
  CC2420SpiP__ChipSpiResource__releasing();
  /* atomic removed: atomic calls only */
#line 348
  {
    if (CC2420SpiP__release) {
        CC2420SpiP__SpiResource__release();
        {
          unsigned char __nesc_temp = 
#line 351
          SUCCESS;

#line 351
          return __nesc_temp;
        }
      }
  }
  return EBUSY;
}

# 247 "/home/tinyos/local/src/tinyos-2.x/tos/chips/msp430/usart/HplMsp430Usart0P.nc"
static void HplMsp430Usart0P__Usart__disableSpi(void )
#line 247
{
  /* atomic removed: atomic calls only */
#line 248
  {
    HplMsp430Usart0P__ME1 &= ~(1 << 6);
    HplMsp430Usart0P__SIMO__selectIOFunc();
    HplMsp430Usart0P__SOMI__selectIOFunc();
    HplMsp430Usart0P__UCLK__selectIOFunc();
  }
}

# 136 "/home/tinyos/local/src/tinyos-2.x/tos/lib/timer/TransformAlarmC.nc"
static void /*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC__1__Alarm__startAt(/*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC__1__to_size_type t0, /*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC__1__to_size_type dt)
{
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
    {
      /*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC__1__m_t0 = t0;
      /*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC__1__m_dt = dt;
      /*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC__1__set_alarm();
    }
#line 143
    __nesc_atomic_end(__nesc_atomic); }
}

#line 96
static void /*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC__1__set_alarm(void )
{
  /*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC__1__to_size_type now = /*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC__1__Counter__get();
#line 98
  /*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC__1__to_size_type expires;
#line 98
  /*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC__1__to_size_type remaining;




  expires = /*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC__1__m_t0 + /*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC__1__m_dt;


  remaining = (/*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC__1__to_size_type )(expires - now);


  if (/*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC__1__m_t0 <= now) 
    {
      if (expires >= /*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC__1__m_t0 && 
      expires <= now) {
        remaining = 0;
        }
    }
  else {
      if (expires >= /*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC__1__m_t0 || 
      expires <= now) {
        remaining = 0;
        }
    }
#line 121
  if (remaining > /*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC__1__MAX_DELAY) 
    {
      /*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC__1__m_t0 = now + /*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC__1__MAX_DELAY;
      /*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC__1__m_dt = remaining - /*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC__1__MAX_DELAY;
      remaining = /*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC__1__MAX_DELAY;
    }
  else 
    {
      /*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC__1__m_t0 += /*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC__1__m_dt;
      /*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC__1__m_dt = 0;
    }
  /*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC__1__AlarmFrom__startAt((/*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC__1__from_size_type )now << 0, 
  (/*AlarmMultiplexC.Alarm.Alarm32khz32C.Transform*/TransformAlarmC__1__from_size_type )remaining << 0);
}

# 842 "/home/tinyos/local/src/tinyos-2.x/tos/chips/cc2420/transmit/CC2420TransmitP.nc"
static void CC2420TransmitP__signalDone(error_t err)
#line 842
{
  /* atomic removed: atomic calls only */
#line 843
  CC2420TransmitP__m_state = CC2420TransmitP__S_STARTED;
  CC2420TransmitP__abortSpiRelease = FALSE;
  CC2420TransmitP__ChipSpiResource__attemptRelease();
  CC2420TransmitP__Send__sendDone(CC2420TransmitP__m_msg, err);
}

# 137 "/home/tinyos/local/src/tinyos-2.x/tos/chips/cc2420/packet/CC2420PacketP.nc"
static void CC2420PacketP__PacketTimeStamp32khz__clear(message_t *msg)
{
  __nesc_hton_int8(CC2420PacketP__CC2420PacketBody__getMetadata(msg)->timesync.data, FALSE);
  __nesc_hton_uint32(CC2420PacketP__CC2420PacketBody__getMetadata(msg)->timestamp.data, CC2420_INVALID_TIMESTAMP);
}

# 780 "/home/tinyos/local/src/tinyos-2.x/tos/chips/cc2420/transmit/CC2420TransmitP.nc"
static void CC2420TransmitP__congestionBackoff(void )
#line 780
{
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 781
    {
      CC2420TransmitP__RadioBackoff__requestCongestionBackoff(CC2420TransmitP__m_msg);
      CC2420TransmitP__BackoffTimer__start(CC2420TransmitP__myCongestionBackoff);
    }
#line 784
    __nesc_atomic_end(__nesc_atomic); }
}

# 58 "/home/tinyos/local/src/tinyos-2.x/tos/system/RandomMlcgC.nc"
static uint32_t RandomMlcgC__Random__rand32(void )
#line 58
{
  uint32_t mlcg;
#line 59
  uint32_t p;
#line 59
  uint32_t q;
  uint64_t tmpseed;

#line 61
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
    {
      tmpseed = (uint64_t )33614U * (uint64_t )RandomMlcgC__seed;
      q = tmpseed;
      q = q >> 1;
      p = tmpseed >> 32;
      mlcg = p + q;
      if (mlcg & 0x80000000) {
          mlcg = mlcg & 0x7FFFFFFF;
          mlcg++;
        }
      RandomMlcgC__seed = mlcg;
    }
#line 73
    __nesc_atomic_end(__nesc_atomic); }
  return mlcg;
}

# 787 "/home/tinyos/local/src/tinyos-2.x/tos/chips/cc2420/transmit/CC2420TransmitP.nc"
static error_t CC2420TransmitP__acquireSpiResource(void )
#line 787
{
  error_t error = CC2420TransmitP__SpiResource__immediateRequest();

#line 789
  if (error != SUCCESS) {
      CC2420TransmitP__SpiResource__request();
    }
  return error;
}

# 126 "/home/tinyos/local/src/tinyos-2.x/tos/chips/cc2420/spi/CC2420SpiP.nc"
static error_t CC2420SpiP__Resource__immediateRequest(uint8_t id)
#line 126
{
  error_t error;

  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 129
    {
      if (CC2420SpiP__WorkingState__requestState(CC2420SpiP__S_BUSY) != SUCCESS) {
          {
            unsigned char __nesc_temp = 
#line 131
            EBUSY;

            {
#line 131
              __nesc_atomic_end(__nesc_atomic); 
#line 131
              return __nesc_temp;
            }
          }
        }
      if (CC2420SpiP__SpiResource__isOwner()) {
          CC2420SpiP__m_holder = id;
          error = SUCCESS;
        }
      else {
#line 139
        if ((error = CC2420SpiP__SpiResource__immediateRequest()) == SUCCESS) {
            CC2420SpiP__m_holder = id;
          }
        else {
            CC2420SpiP__WorkingState__toIdle();
          }
        }
    }
#line 146
    __nesc_atomic_end(__nesc_atomic); }
#line 146
  return error;
}

# 96 "/home/tinyos/local/src/tinyos-2.x/tos/system/StateImplP.nc"
static error_t StateImplP__State__requestState(uint8_t id, uint8_t reqState)
#line 96
{
  error_t returnVal = FAIL;

#line 98
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 98
    {
      if (reqState == StateImplP__S_IDLE || StateImplP__state[id] == StateImplP__S_IDLE) {
          StateImplP__state[id] = reqState;
          returnVal = SUCCESS;
        }
    }
#line 103
    __nesc_atomic_end(__nesc_atomic); }
  return returnVal;
}

# 174 "/home/tinyos/local/src/tinyos-2.x/tos/system/ArbiterP.nc"
static uint8_t /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0__Resource__isOwner(uint8_t id)
#line 174
{
  /* atomic removed: atomic calls only */
#line 175
  {
    if (/*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0__resId == id && /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0__state == /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0__RES_BUSY) {
        unsigned char __nesc_temp = 
#line 176
        TRUE;

#line 176
        return __nesc_temp;
      }
    else 
#line 177
      {
        unsigned char __nesc_temp = 
#line 177
        FALSE;

#line 177
        return __nesc_temp;
      }
  }
}

#line 130
static error_t /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0__ResourceDefaultOwner__release(void )
#line 130
{
  /* atomic removed: atomic calls only */
#line 131
  {
    if (/*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0__resId == /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0__default_owner_id) {
        if (/*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0__state == /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0__RES_GRANTING) {
            /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0__grantedTask__postTask();
            {
              unsigned char __nesc_temp = 
#line 135
              SUCCESS;

#line 135
              return __nesc_temp;
            }
          }
        else {
#line 137
          if (/*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0__state == /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0__RES_IMM_GRANTING) {
              /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0__resId = /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0__reqResId;
              /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0__state = /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0__RES_BUSY;
              {
                unsigned char __nesc_temp = 
#line 140
                SUCCESS;

#line 140
                return __nesc_temp;
              }
            }
          }
      }
  }
#line 144
  return FAIL;
}

# 265 "/home/tinyos/local/src/tinyos-2.x/tos/chips/msp430/usart/HplMsp430Usart0P.nc"
static void HplMsp430Usart0P__Usart__setModeSpi(msp430_spi_union_config_t *config)
#line 265
{

  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 267
    {
      HplMsp430Usart0P__Usart__resetUsart(TRUE);
      HplMsp430Usart0P__HplI2C__clearModeI2C();
      HplMsp430Usart0P__Usart__disableUart();
      HplMsp430Usart0P__configSpi(config);
      HplMsp430Usart0P__Usart__enableSpi();
      HplMsp430Usart0P__Usart__resetUsart(FALSE);
      HplMsp430Usart0P__Usart__clrIntr();
      HplMsp430Usart0P__Usart__disableIntr();
    }
#line 276
    __nesc_atomic_end(__nesc_atomic); }
  return;
}

# 107 "/home/tinyos/local/src/tinyos-2.x/tos/chips/cc2420/spi/CC2420SpiP.nc"
static error_t CC2420SpiP__Resource__request(uint8_t id)
#line 107
{

  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 109
    {
      if (CC2420SpiP__WorkingState__requestState(CC2420SpiP__S_BUSY) == SUCCESS) {
          CC2420SpiP__m_holder = id;
          if (CC2420SpiP__SpiResource__isOwner()) {
              CC2420SpiP__grant__postTask();
            }
          else {
              CC2420SpiP__SpiResource__request();
            }
        }
      else {
          CC2420SpiP__m_requests |= 1 << id;
        }
    }
#line 122
    __nesc_atomic_end(__nesc_atomic); }
  return SUCCESS;
}

# 735 "/home/tinyos/local/src/tinyos-2.x/tos/chips/cc2420/transmit/CC2420TransmitP.nc"
static void CC2420TransmitP__attemptSend(void )
#line 735
{
  uint8_t status;
  bool congestion = TRUE;

  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 739
    {
      if (CC2420TransmitP__m_state == CC2420TransmitP__S_CANCEL) {
          CC2420TransmitP__SFLUSHTX__strobe();
          CC2420TransmitP__releaseSpiResource();
          CC2420TransmitP__CSN__set();
          CC2420TransmitP__m_state = CC2420TransmitP__S_STARTED;
          CC2420TransmitP__Send__sendDone(CC2420TransmitP__m_msg, ECANCEL);
          {
#line 746
            __nesc_atomic_end(__nesc_atomic); 
#line 746
            return;
          }
        }





      CC2420TransmitP__CSN__clr();
      status = CC2420TransmitP__m_cca ? CC2420TransmitP__STXONCCA__strobe() : CC2420TransmitP__STXON__strobe();
      if (!(status & CC2420_STATUS_TX_ACTIVE)) {
          status = CC2420TransmitP__SNOP__strobe();
          if (status & CC2420_STATUS_TX_ACTIVE) {
              congestion = FALSE;
            }
        }

      CC2420TransmitP__m_state = congestion ? CC2420TransmitP__S_SAMPLE_CCA : CC2420TransmitP__S_SFD;
      CC2420TransmitP__CSN__set();
    }
#line 765
    __nesc_atomic_end(__nesc_atomic); }

  if (congestion) {
      CC2420TransmitP__totalCcaChecks = 0;
      CC2420TransmitP__releaseSpiResource();
      CC2420TransmitP__congestionBackoff();
    }
  else 
#line 771
    {
      CC2420TransmitP__BackoffTimer__start(CC2420TransmitP__CC2420_ABORT_PERIOD);
    }
}

# 318 "/home/tinyos/local/src/tinyos-2.x/tos/chips/cc2420/spi/CC2420SpiP.nc"
static cc2420_status_t CC2420SpiP__Strobe__strobe(uint8_t addr)
#line 318
{
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 319
    {
      if (CC2420SpiP__WorkingState__isIdle()) {
          {
            unsigned char __nesc_temp = 
#line 321
            0;

            {
#line 321
              __nesc_atomic_end(__nesc_atomic); 
#line 321
              return __nesc_temp;
            }
          }
        }
    }
#line 325
    __nesc_atomic_end(__nesc_atomic); }
#line 325
  return CC2420SpiP__SpiByte__write(addr);
}

# 46 "/home/tinyos/local/src/tinyos-2.x/tos/chips/msp430/pins/HplMsp430GeneralIOP.nc"
static void /*HplMsp430GeneralIOC.P46*/HplMsp430GeneralIOP__30__IO__clr(void )
#line 46
{
#line 46
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 46
    * (volatile uint8_t * )29U &= ~(0x01 << 6);
#line 46
    __nesc_atomic_end(__nesc_atomic); }
}

#line 45
static void /*HplMsp430GeneralIOC.P46*/HplMsp430GeneralIOP__30__IO__set(void )
#line 45
{
#line 45
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 45
    * (volatile uint8_t * )29U |= 0x01 << 6;
#line 45
    __nesc_atomic_end(__nesc_atomic); }
}

# 14 "/home/tinyos/local/src/tinyos-2.x/tos/chips/msp430/timer/Msp430TimerCommonP.nc"
__attribute((wakeup)) __attribute((interrupt(24)))  void sig_TIMERB1_VECTOR(void )
#line 14
{
#line 14
  Msp430TimerCommonP__VectorTimerB1__fired();
}

# 52 "/home/tinyos/local/src/tinyos-2.x/tos/system/RealMainP.nc"
  int main(void )
#line 52
{
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
    {





      {
      }
#line 60
      ;

      RealMainP__Scheduler__init();





      RealMainP__PlatformInit__init();
      while (RealMainP__Scheduler__runNextTask()) ;





      RealMainP__SoftwareInit__init();
      while (RealMainP__Scheduler__runNextTask()) ;
    }
#line 77
    __nesc_atomic_end(__nesc_atomic); }


  __nesc_enable_interrupt();

  RealMainP__Boot__booted();


  RealMainP__Scheduler__taskLoop();




  return -1;
}

# 164 "/home/tinyos/local/src/tinyos-2.x/tos/chips/msp430/timer/Msp430ClockP.nc"
static void Msp430ClockP__set_dco_calib(int calib)
{
  BCSCTL1 = (BCSCTL1 & ~0x07) | ((calib >> 8) & 0x07);
  DCOCTL = calib & 0xff;
}

# 16 "/home/tinyos/local/src/tinyos-2.x/tos/platforms/telosb/MotePlatformC.nc"
static void MotePlatformC__TOSH_FLASH_M25P_DP_bit(bool set)
#line 16
{
  if (set) {
    TOSH_SET_SIMO0_PIN();
    }
  else {
#line 20
    TOSH_CLR_SIMO0_PIN();
    }
#line 21
  TOSH_SET_UCLK0_PIN();
  TOSH_CLR_UCLK0_PIN();
}

# 123 "/home/tinyos/local/src/tinyos-2.x/tos/system/SchedulerBasicP.nc"
static bool SchedulerBasicP__Scheduler__runNextTask(void )
{
  uint8_t nextTask;

  /* atomic removed: atomic calls only */
#line 127
  {
    nextTask = SchedulerBasicP__popTask();
    if (nextTask == SchedulerBasicP__NO_TASK) 
      {
        {
          unsigned char __nesc_temp = 
#line 131
          FALSE;

#line 131
          return __nesc_temp;
        }
      }
  }
#line 134
  SchedulerBasicP__TaskBasic__runTask(nextTask);
  return TRUE;
}

#line 164
static void SchedulerBasicP__TaskBasic__default__runTask(uint8_t id)
{
}

# 64 "/home/tinyos/local/src/tinyos-2.x/tos/interfaces/TaskBasic.nc"
static void SchedulerBasicP__TaskBasic__runTask(uint8_t arg_0x4059ab40){
#line 64
  switch (arg_0x4059ab40) {
#line 64
    case /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__fired:
#line 64
      /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC__0__fired__runTask();
#line 64
      break;
#line 64
    case /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__updateFromTimer:
#line 64
      /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__updateFromTimer__runTask();
#line 64
      break;
#line 64
    case CC2420CsmaP__startDone_task:
#line 64
      CC2420CsmaP__startDone_task__runTask();
#line 64
      break;
#line 64
    case CC2420CsmaP__stopDone_task:
#line 64
      CC2420CsmaP__stopDone_task__runTask();
#line 64
      break;
#line 64
    case CC2420CsmaP__sendDone_task:
#line 64
      CC2420CsmaP__sendDone_task__runTask();
#line 64
      break;
#line 64
    case CC2420ControlP__sync:
#line 64
      CC2420ControlP__sync__runTask();
#line 64
      break;
#line 64
    case CC2420ControlP__syncDone:
#line 64
      CC2420ControlP__syncDone__runTask();
#line 64
      break;
#line 64
    case CC2420SpiP__grant:
#line 64
      CC2420SpiP__grant__runTask();
#line 64
      break;
#line 64
    case /*Msp430SpiDma0P.SpiP*/Msp430SpiDmaP__0__signalDone_task:
#line 64
      /*Msp430SpiDma0P.SpiP*/Msp430SpiDmaP__0__signalDone_task__runTask();
#line 64
      break;
#line 64
    case /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0__grantedTask:
#line 64
      /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0__grantedTask__runTask();
#line 64
      break;
#line 64
    case CC2420ReceiveP__receiveDone_task:
#line 64
      CC2420ReceiveP__receiveDone_task__runTask();
#line 64
      break;
#line 64
    case CC2420TinyosNetworkP__grantTask:
#line 64
      CC2420TinyosNetworkP__grantTask__runTask();
#line 64
      break;
#line 64
    case PacketLinkP__send:
#line 64
      PacketLinkP__send__runTask();
#line 64
      break;
#line 64
    case IPDispatchP__sendTask:
#line 64
      IPDispatchP__sendTask__runTask();
#line 64
      break;
#line 64
    case /*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP__0__readSensor:
#line 64
      /*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP__0__readSensor__runTask();
#line 64
      break;
#line 64
    case /*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP__0__signalStatusDone:
#line 64
      /*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP__0__signalStatusDone__runTask();
#line 64
      break;
#line 64
    case HplSensirionSht11P__stopTask:
#line 64
      HplSensirionSht11P__stopTask__runTask();
#line 64
      break;
#line 64
    case /*HplSensirionSht11C.Arbiter.Arbiter*/ArbiterP__1__grantedTask:
#line 64
      /*HplSensirionSht11C.Arbiter.Arbiter*/ArbiterP__1__grantedTask__runTask();
#line 64
      break;
#line 64
    case /*HplSensirionSht11C.SplitControlPowerManagerC.PowerManager*/PowerManagerP__0__startTask:
#line 64
      /*HplSensirionSht11C.SplitControlPowerManagerC.PowerManager*/PowerManagerP__0__startTask__runTask();
#line 64
      break;
#line 64
    case /*HplSensirionSht11C.SplitControlPowerManagerC.PowerManager*/PowerManagerP__0__stopTask:
#line 64
      /*HplSensirionSht11C.SplitControlPowerManagerC.PowerManager*/PowerManagerP__0__stopTask__runTask();
#line 64
      break;
#line 64
    default:
#line 64
      SchedulerBasicP__TaskBasic__default__runTask(arg_0x4059ab40);
#line 64
      break;
#line 64
    }
#line 64
}
#line 64
# 52 "/home/tinyos/local/src/tinyos-2.x/tos/platforms/telosa/chips/sht11/HplSensirionSht11P.nc"
static error_t HplSensirionSht11P__SplitControl__start(void )
#line 52
{
  HplSensirionSht11P__PWR__makeOutput();
  HplSensirionSht11P__PWR__set();
  HplSensirionSht11P__Timer__startOneShot(11);
  return SUCCESS;
}

# 133 "/home/tinyos/local/src/tinyos-2.x/tos/lib/timer/VirtualizeTimerC.nc"
static void /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__startTimer(uint8_t num, uint32_t t0, uint32_t dt, bool isoneshot)
{
  /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__Timer_t *timer = &/*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__m_timers[num];

#line 136
  timer->t0 = t0;
  timer->dt = dt;
  timer->isoneshot = isoneshot;
  timer->isrunning = TRUE;
  /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__updateFromTimer__postTask();
}

# 46 "/home/tinyos/local/src/tinyos-2.x/tos/chips/msp430/pins/HplMsp430GeneralIOP.nc"
static void /*HplMsp430GeneralIOC.P16*/HplMsp430GeneralIOP__6__IO__clr(void )
#line 46
{
#line 46
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 46
    * (volatile uint8_t * )33U &= ~(0x01 << 6);
#line 46
    __nesc_atomic_end(__nesc_atomic); }
}


static void /*HplMsp430GeneralIOC.P15*/HplMsp430GeneralIOP__5__IO__makeInput(void )
#line 50
{
#line 50
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 50
    * (volatile uint8_t * )34U &= ~(0x01 << 5);
#line 50
    __nesc_atomic_end(__nesc_atomic); }
}

#line 46
static void /*HplMsp430GeneralIOC.P15*/HplMsp430GeneralIOP__5__IO__clr(void )
#line 46
{
#line 46
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 46
    * (volatile uint8_t * )33U &= ~(0x01 << 5);
#line 46
    __nesc_atomic_end(__nesc_atomic); }
}

# 149 "/home/tinyos/local/src/tinyos-2.x/tos/chips/sht11/SensirionSht11LogicP.nc"
static error_t /*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP__0__performCommand(void )
#line 149
{

  /*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP__0__initPins();
  /*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP__0__resetDevice();
  /*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP__0__transmissionStart();
  /*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP__0__cmd &= 0x1F;
  /*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP__0__sendCommand(/*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP__0__cmd);

  if (/*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP__0__waitForResponse() != SUCCESS) {
      /*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP__0__busy = FALSE;
      return FAIL;
    }

  switch (/*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP__0__cmd) {

      case /*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP__0__CMD_SOFT_RESET: 
        /*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP__0__Timer__startOneShot(/*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP__0__TIMEOUT_RESET);
      break;

      case /*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP__0__CMD_MEASURE_TEMPERATURE: 
        /*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP__0__enableInterrupt();

      if (/*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP__0__status & SHT11_STATUS_LOW_RES_BIT) {
          /*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP__0__Timer__startOneShot(/*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP__0__TIMEOUT_12BIT);
        }
      else 
#line 173
        {
          /*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP__0__Timer__startOneShot(/*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP__0__TIMEOUT_14BIT);
        }

      break;

      case /*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP__0__CMD_MEASURE_HUMIDITY: 
        /*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP__0__enableInterrupt();

      if (/*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP__0__status & SHT11_STATUS_LOW_RES_BIT) {
          /*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP__0__Timer__startOneShot(/*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP__0__TIMEOUT_8BIT);
        }
      else 
#line 184
        {
          /*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP__0__Timer__startOneShot(/*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP__0__TIMEOUT_12BIT);
        }

      break;

      case /*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP__0__CMD_READ_STATUS: 
        {
          uint8_t tempStatus;
          uint8_t crc;

          tempStatus = /*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP__0__readByte();
          crc = /*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP__0__readByte();
          /*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP__0__endTransmission();

          /*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP__0__status = tempStatus;

          /*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP__0__signalStatusDone__postTask();
        }

      case /*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP__0__CMD_WRITE_STATUS: 
        /*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP__0__writeByte(/*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP__0__newStatus);

      if (/*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP__0__waitForResponse() != SUCCESS) {
          /*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP__0__writeFail = TRUE;
        }
      else 
#line 209
        {
          /*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP__0__status = /*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP__0__newStatus;
        }

      /*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP__0__signalStatusDone__postTask();
    }


  return SUCCESS;
}

# 45 "/home/tinyos/local/src/tinyos-2.x/tos/chips/msp430/pins/HplMsp430GeneralIOP.nc"
static void /*HplMsp430GeneralIOC.P15*/HplMsp430GeneralIOP__5__IO__set(void )
#line 45
{
#line 45
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 45
    * (volatile uint8_t * )33U |= 0x01 << 5;
#line 45
    __nesc_atomic_end(__nesc_atomic); }
}

# 58 "/home/tinyos/local/src/tinyos-2.x/tos/chips/msp430/pins/Msp430InterruptC.nc"
static error_t /*HplSensirionSht11C.InterruptDATAC*/Msp430InterruptC__2__Interrupt__disable(void )
#line 58
{
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 59
    {
      /*HplSensirionSht11C.InterruptDATAC*/Msp430InterruptC__2__HplInterrupt__disable();
      /*HplSensirionSht11C.InterruptDATAC*/Msp430InterruptC__2__HplInterrupt__clear();
    }
#line 62
    __nesc_atomic_end(__nesc_atomic); }
  return SUCCESS;
}

# 52 "/home/tinyos/local/src/tinyos-2.x/tos/chips/msp430/pins/HplMsp430GeneralIOP.nc"
static void /*HplMsp430GeneralIOC.P15*/HplMsp430GeneralIOP__5__IO__makeOutput(void )
#line 52
{
#line 52
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 52
    * (volatile uint8_t * )34U |= 0x01 << 5;
#line 52
    __nesc_atomic_end(__nesc_atomic); }
}

#line 45
static void /*HplMsp430GeneralIOC.P16*/HplMsp430GeneralIOP__6__IO__set(void )
#line 45
{
#line 45
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 45
    * (volatile uint8_t * )33U |= 0x01 << 6;
#line 45
    __nesc_atomic_end(__nesc_atomic); }
}

# 255 "/home/tinyos/local/src/tinyos-2.x/tos/chips/sht11/SensirionSht11LogicP.nc"
static void /*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP__0__writeByte(uint8_t byte)
#line 255
{
  uint8_t i;

#line 257
  for (i = 0; i < 8; i++) {
      if (byte & 0x80) {
        /*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP__0__DATA__set();
        }
      else {
#line 261
        /*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP__0__DATA__clr();
        }
#line 262
      byte = byte << 1;
      /*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP__0__CLOCK__set();
      /*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP__0__CLOCK__clr();
    }
}

static error_t /*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP__0__waitForResponse(void )
#line 268
{
  /*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP__0__DATA__makeInput();
  /*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP__0__DATA__set();
  /*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP__0__CLOCK__set();
  if (/*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP__0__DATA__get()) {


      return FAIL;
    }
  /*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP__0__CLOCK__clr();
  return SUCCESS;
}

# 62 "/home/tinyos/local/src/tinyos-2.x/tos/lib/timer/Timer.nc"
static void /*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP__0__Timer__startOneShot(uint32_t dt){
#line 62
  /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__Timer__startOneShot(11U, dt);
#line 62
}
#line 62
# 281 "/home/tinyos/local/src/tinyos-2.x/tos/chips/sht11/SensirionSht11LogicP.nc"
static void /*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP__0__enableInterrupt(void )
#line 281
{
  /*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP__0__DATA__makeInput();
  /*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP__0__DATA__set();
  /*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP__0__InterruptDATA__enableFallingEdge();
}

#line 355
static uint8_t /*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP__0__readByte(void )
#line 355
{
  uint8_t byte = 0;
  uint8_t i;

  for (i = 0; i < 8; i++) {
      /*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP__0__CLOCK__set();
      if (/*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP__0__DATA__get()) {
        byte |= 1;
        }
#line 363
      if (i != 7) {
        byte = byte << 1;
        }
#line 365
      /*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP__0__CLOCK__clr();
    }

  /*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP__0__ack();
  return byte;
}










static void /*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP__0__endTransmission(void )
#line 381
{
  /*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP__0__DATA__makeOutput();
  /*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP__0__DATA__set();
  /*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP__0__CLOCK__set();
  /*HalSensirionSht11C.SensirionSht11LogicP*/SensirionSht11LogicP__0__CLOCK__clr();
}

# 108 "/home/tinyos/local/src/tinyos-2.x/tos/system/ArbiterP.nc"
static error_t /*HplSensirionSht11C.Arbiter.Arbiter*/ArbiterP__1__Resource__release(uint8_t id)
#line 108
{
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 109
    {
      if (/*HplSensirionSht11C.Arbiter.Arbiter*/ArbiterP__1__state == /*HplSensirionSht11C.Arbiter.Arbiter*/ArbiterP__1__RES_BUSY && /*HplSensirionSht11C.Arbiter.Arbiter*/ArbiterP__1__resId == id) {
          if (/*HplSensirionSht11C.Arbiter.Arbiter*/ArbiterP__1__Queue__isEmpty() == FALSE) {
              /*HplSensirionSht11C.Arbiter.Arbiter*/ArbiterP__1__reqResId = /*HplSensirionSht11C.Arbiter.Arbiter*/ArbiterP__1__Queue__dequeue();
              /*HplSensirionSht11C.Arbiter.Arbiter*/ArbiterP__1__resId = /*HplSensirionSht11C.Arbiter.Arbiter*/ArbiterP__1__NO_RES;
              /*HplSensirionSht11C.Arbiter.Arbiter*/ArbiterP__1__state = /*HplSensirionSht11C.Arbiter.Arbiter*/ArbiterP__1__RES_GRANTING;
              /*HplSensirionSht11C.Arbiter.Arbiter*/ArbiterP__1__grantedTask__postTask();
              /*HplSensirionSht11C.Arbiter.Arbiter*/ArbiterP__1__ResourceConfigure__unconfigure(id);
            }
          else {
              /*HplSensirionSht11C.Arbiter.Arbiter*/ArbiterP__1__resId = /*HplSensirionSht11C.Arbiter.Arbiter*/ArbiterP__1__default_owner_id;
              /*HplSensirionSht11C.Arbiter.Arbiter*/ArbiterP__1__state = /*HplSensirionSht11C.Arbiter.Arbiter*/ArbiterP__1__RES_CONTROLLED;
              /*HplSensirionSht11C.Arbiter.Arbiter*/ArbiterP__1__ResourceConfigure__unconfigure(id);
              /*HplSensirionSht11C.Arbiter.Arbiter*/ArbiterP__1__ResourceDefaultOwner__granted();
            }
          {
            unsigned char __nesc_temp = 
#line 124
            SUCCESS;

            {
#line 124
              __nesc_atomic_end(__nesc_atomic); 
#line 124
              return __nesc_temp;
            }
          }
        }
    }
#line 128
    __nesc_atomic_end(__nesc_atomic); }
#line 127
  return FAIL;
}

# 73 "/home/tinyos/local/src/tinyos-2.x/tos/chips/sht11/SensirionSht11ReaderP.nc"
static void /*TCPEchoC.Sht11C.SensirionSht11ReaderP*/SensirionSht11ReaderP__0__Sht11Temp__measureTemperatureDone(error_t result, uint16_t val)
#line 73
{
  /*TCPEchoC.Sht11C.SensirionSht11ReaderP*/SensirionSht11ReaderP__0__TempResource__release();
  /*TCPEchoC.Sht11C.SensirionSht11ReaderP*/SensirionSht11ReaderP__0__Temperature__readDone(result, val);
}

# 101 "/home/tinyos/local/src/tinyos-2.x/tos/chips/cc2420/CC2420Ieee154MessageP.nc"
static ieee154_saddr_t CC2420Ieee154MessageP__Ieee154Packet__destination(message_t *msg)
#line 101
{
  cc2420_header_t *header = CC2420Ieee154MessageP__CC2420PacketBody__getHeader(msg);

#line 103
  return __nesc_ntoh_leuint16(header->dest.data);
}

# 103 "/home/tinyos/local/src/tinyos-2.x/tos/system/PoolP.nc"
static error_t /*IPDispatchC.SendInfoPool.PoolP*/PoolP__2__Pool__put(/*IPDispatchC.SendInfoPool.PoolP*/PoolP__2__pool_t *newVal)
#line 103
{
  if (/*IPDispatchC.SendInfoPool.PoolP*/PoolP__2__free >= 14) {
      return FAIL;
    }
  else {
      uint16_t emptyIndex = /*IPDispatchC.SendInfoPool.PoolP*/PoolP__2__index + /*IPDispatchC.SendInfoPool.PoolP*/PoolP__2__free;

#line 109
      if (emptyIndex >= 14) {
          emptyIndex -= 14;
        }
      /*IPDispatchC.SendInfoPool.PoolP*/PoolP__2__queue[emptyIndex] = newVal;
      /*IPDispatchC.SendInfoPool.PoolP*/PoolP__2__free++;
      ;
      return SUCCESS;
    }
}

#line 103
static error_t /*IPDispatchC.FragPool.PoolP*/PoolP__0__Pool__put(/*IPDispatchC.FragPool.PoolP*/PoolP__0__pool_t *newVal)
#line 103
{
  if (/*IPDispatchC.FragPool.PoolP*/PoolP__0__free >= 14) {
      return FAIL;
    }
  else {
      uint16_t emptyIndex = /*IPDispatchC.FragPool.PoolP*/PoolP__0__index + /*IPDispatchC.FragPool.PoolP*/PoolP__0__free;

#line 109
      if (emptyIndex >= 14) {
          emptyIndex -= 14;
        }
      /*IPDispatchC.FragPool.PoolP*/PoolP__0__queue[emptyIndex] = newVal;
      /*IPDispatchC.FragPool.PoolP*/PoolP__0__free++;
      ;
      return SUCCESS;
    }
}

#line 103
static error_t /*IPDispatchC.SendEntryPool.PoolP*/PoolP__1__Pool__put(/*IPDispatchC.SendEntryPool.PoolP*/PoolP__1__pool_t *newVal)
#line 103
{
  if (/*IPDispatchC.SendEntryPool.PoolP*/PoolP__1__free >= 14) {
      return FAIL;
    }
  else {
      uint16_t emptyIndex = /*IPDispatchC.SendEntryPool.PoolP*/PoolP__1__index + /*IPDispatchC.SendEntryPool.PoolP*/PoolP__1__free;

#line 109
      if (emptyIndex >= 14) {
          emptyIndex -= 14;
        }
      /*IPDispatchC.SendEntryPool.PoolP*/PoolP__1__queue[emptyIndex] = newVal;
      /*IPDispatchC.SendEntryPool.PoolP*/PoolP__1__free++;
      ;
      return SUCCESS;
    }
}

# 85 "/home/tinyos/local/src/tinyos-2.x/tos/system/QueueC.nc"
static /*IPDispatchC.QueueC*/QueueC__0__queue_t /*IPDispatchC.QueueC*/QueueC__0__Queue__dequeue(void )
#line 85
{
  /*IPDispatchC.QueueC*/QueueC__0__queue_t t = /*IPDispatchC.QueueC*/QueueC__0__Queue__head();

#line 87
  ;
  if (!/*IPDispatchC.QueueC*/QueueC__0__Queue__empty()) {
      /*IPDispatchC.QueueC*/QueueC__0__head++;
      if (/*IPDispatchC.QueueC*/QueueC__0__head == 14) {
#line 90
        /*IPDispatchC.QueueC*/QueueC__0__head = 0;
        }
#line 91
      /*IPDispatchC.QueueC*/QueueC__0__size--;
      /*IPDispatchC.QueueC*/QueueC__0__printQueue();
    }
  return t;
}

# 65 "/home/tinyos/local/src/tinyos-2.x/tos/chips/cc2420/packet/CC2420PacketP.nc"
static error_t CC2420PacketP__Acks__requestAck(message_t *p_msg)
#line 65
{
  unsigned char *__nesc_temp48;

#line 66
  (__nesc_temp48 = CC2420PacketP__CC2420PacketBody__getHeader(p_msg)->fcf.data, __nesc_hton_leuint16(__nesc_temp48, __nesc_ntoh_leuint16(__nesc_temp48) | (1 << IEEE154_FCF_ACK_REQ)));
  return SUCCESS;
}

# 122 "/home/tinyos/local/src/tinyos-2.x/tos/chips/cc2420/csma/CC2420CsmaP.nc"
static error_t CC2420CsmaP__Send__send(message_t *p_msg, uint8_t len)
#line 122
{
  unsigned char *__nesc_temp45;
  unsigned char *__nesc_temp44;
#line 124
  cc2420_header_t *header = CC2420CsmaP__CC2420PacketBody__getHeader(p_msg);
  cc2420_metadata_t *metadata = CC2420CsmaP__CC2420PacketBody__getMetadata(p_msg);

  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 127
    {
      if (!CC2420CsmaP__SplitControlState__isState(CC2420CsmaP__S_STARTED)) {
          {
            unsigned char __nesc_temp = 
#line 129
            FAIL;

            {
#line 129
              __nesc_atomic_end(__nesc_atomic); 
#line 129
              return __nesc_temp;
            }
          }
        }
#line 132
      CC2420CsmaP__SplitControlState__forceState(CC2420CsmaP__S_TRANSMITTING);
      CC2420CsmaP__m_msg = p_msg;
    }
#line 134
    __nesc_atomic_end(__nesc_atomic); }

  __nesc_hton_leuint8(header->length.data, len + CC2420_SIZE);



  (__nesc_temp44 = header->fcf.data, __nesc_hton_leuint16(__nesc_temp44, __nesc_ntoh_leuint16(__nesc_temp44) & (1 << IEEE154_FCF_ACK_REQ)));

  (__nesc_temp45 = header->fcf.data, __nesc_hton_leuint16(__nesc_temp45, __nesc_ntoh_leuint16(__nesc_temp45) | ((((IEEE154_TYPE_DATA << IEEE154_FCF_FRAME_TYPE) | (
  1 << IEEE154_FCF_INTRAPAN)) | (
  IEEE154_ADDR_SHORT << IEEE154_FCF_DEST_ADDR_MODE)) | (
  IEEE154_ADDR_SHORT << IEEE154_FCF_SRC_ADDR_MODE))));

  __nesc_hton_int8(metadata->ack.data, FALSE);
  __nesc_hton_uint8(metadata->rssi.data, 0);
  __nesc_hton_uint8(metadata->lqi.data, 0);

  __nesc_hton_uint32(metadata->timestamp.data, CC2420_INVALID_TIMESTAMP);

  CC2420CsmaP__ccaOn = TRUE;
  CC2420CsmaP__RadioBackoff__requestCca(CC2420CsmaP__m_msg);

  CC2420CsmaP__CC2420Transmit__send(CC2420CsmaP__m_msg, CC2420CsmaP__ccaOn);
  return SUCCESS;
}

# 817 "/home/tinyos/local/src/tinyos-2.x/tos/chips/cc2420/transmit/CC2420TransmitP.nc"
static void CC2420TransmitP__loadTXFIFO(void )
#line 817
{
  cc2420_header_t *header = CC2420TransmitP__CC2420PacketBody__getHeader(CC2420TransmitP__m_msg);
  uint8_t tx_power = __nesc_ntoh_uint8(CC2420TransmitP__CC2420PacketBody__getMetadata(CC2420TransmitP__m_msg)->tx_power.data);

  if (!tx_power) {
      tx_power = 31;
    }

  CC2420TransmitP__CSN__clr();

  if (CC2420TransmitP__m_tx_power != tx_power) {
      CC2420TransmitP__TXCTRL__write((((2 << CC2420_TXCTRL_TXMIXBUF_CUR) | (
      3 << CC2420_TXCTRL_PA_CURRENT)) | (
      1 << CC2420_TXCTRL_RESERVED)) | ((
      tx_power & 0x1F) << CC2420_TXCTRL_PA_LEVEL));
    }

  CC2420TransmitP__m_tx_power = tx_power;

  {
    uint8_t tmpLen __attribute((unused))  = __nesc_ntoh_leuint8(header->length.data) - 1;

#line 838
    CC2420TransmitP__TXFIFO__write((uint8_t * )header, __nesc_ntoh_leuint8(header->length.data) - 1);
  }
}

# 305 "/home/tinyos/local/src/tinyos-2.x/tos/chips/cc2420/spi/CC2420SpiP.nc"
static cc2420_status_t CC2420SpiP__Reg__write(uint8_t addr, uint16_t data)
#line 305
{
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 306
    {
      if (CC2420SpiP__WorkingState__isIdle()) {
          {
            unsigned char __nesc_temp = 
#line 308
            0;

            {
#line 308
              __nesc_atomic_end(__nesc_atomic); 
#line 308
              return __nesc_temp;
            }
          }
        }
    }
#line 312
    __nesc_atomic_end(__nesc_atomic); }
#line 311
  CC2420SpiP__SpiByte__write(addr);
  CC2420SpiP__SpiByte__write(data >> 8);
  return CC2420SpiP__SpiByte__write(data & 0xff);
}

# 125 "/home/tinyos/local/src/tinyos-2.x/tos/chips/msp430/usart/Msp430SpiDmaP.nc"
static error_t /*Msp430SpiDma0P.SpiP*/Msp430SpiDmaP__0__SpiPacket__send(uint8_t id, uint8_t *tx_buf, 
uint8_t *rx_buf, 
uint16_t len)
#line 127
{

  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 129
    {
      /*Msp430SpiDma0P.SpiP*/Msp430SpiDmaP__0__m_client = id;
      /*Msp430SpiDma0P.SpiP*/Msp430SpiDmaP__0__m_tx_buf = tx_buf;
      /*Msp430SpiDma0P.SpiP*/Msp430SpiDmaP__0__m_rx_buf = rx_buf;
      /*Msp430SpiDma0P.SpiP*/Msp430SpiDmaP__0__m_len = len;
    }
#line 134
    __nesc_atomic_end(__nesc_atomic); }

  if (len) {

      * (volatile uint8_t *)2U &= ~(128 | 64);


      /*Msp430SpiDma0P.SpiP*/Msp430SpiDmaP__0__DmaChannel1__setupTransfer(DMA_SINGLE_TRANSFER, 
      3U, 
      DMA_EDGE_SENSITIVE, 
      (void *)118U, 
      rx_buf ? rx_buf : &/*Msp430SpiDma0P.SpiP*/Msp430SpiDmaP__0__m_dump, 
      len, 
      DMA_BYTE, 
      DMA_BYTE, 
      DMA_ADDRESS_UNCHANGED, 
      rx_buf ? 
      DMA_ADDRESS_INCREMENTED : 
      DMA_ADDRESS_UNCHANGED);

      /*Msp430SpiDma0P.SpiP*/Msp430SpiDmaP__0__DmaChannel1__startTransfer();


      /*Msp430SpiDma0P.SpiP*/Msp430SpiDmaP__0__DmaChannel2__setupTransfer(DMA_SINGLE_TRANSFER, 
      4U, 
      DMA_EDGE_SENSITIVE, 
      tx_buf, 
      (void *)119U, 
      len, 
      DMA_BYTE, 
      DMA_BYTE, 
      DMA_ADDRESS_INCREMENTED, 
      DMA_ADDRESS_UNCHANGED);

      /*Msp430SpiDma0P.SpiP*/Msp430SpiDmaP__0__DmaChannel2__startTransfer();


      * (volatile uint8_t *)2U |= 128;
    }
  else 
#line 172
    {
      /*Msp430SpiDma0P.SpiP*/Msp430SpiDmaP__0__signalDone_task__postTask();
    }

  return SUCCESS;
}

# 34 "/home/tinyos/local/src/tinyos-2.x/tos/lib/net/blip/ResourceSendP.nc"
static void ResourceSendP__Resource__granted(void )
#line 34
{
  error_t rc;

#line 36
  if ((rc = ResourceSendP__SubSend__send(ResourceSendP__m_addr, ResourceSendP__m_msg, ResourceSendP__m_len)) != SUCCESS) {
      ResourceSendP__Ieee154Send__sendDone(ResourceSendP__m_msg, rc);
      ResourceSendP__m_msg = (void *)0;
      ResourceSendP__Resource__release();
    }
}

# 279 "/home/tinyos/local/src/tinyos-2.x/tos/chips/cc2420/control/CC2420ControlP.nc"
static uint16_t CC2420ControlP__CC2420Config__getShortAddr(void )
#line 279
{
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 280
    {
      unsigned int __nesc_temp = 
#line 280
      CC2420ControlP__m_short_addr;

      {
#line 280
        __nesc_atomic_end(__nesc_atomic); 
#line 280
        return __nesc_temp;
      }
    }
#line 282
    __nesc_atomic_end(__nesc_atomic); }
}

# 56 "/home/tinyos/local/src/tinyos-2.x/tos/interfaces/State.nc"
static void PacketLinkP__SendState__toIdle(void ){
#line 56
  StateImplP__State__toIdle(4U);
#line 56
}
#line 56
static void UniqueSendP__State__toIdle(void ){
#line 56
  StateImplP__State__toIdle(2U);
#line 56
}
#line 56
# 974 "/home/tinyos/local/src/tinyos-2.x/tos/lib/net/blip/IPDispatchP.nc"
static void IPDispatchP__Ieee154Send__sendDone(message_t *msg, error_t error)
#line 974
{
  unsigned char __nesc_temp69;
  unsigned char *__nesc_temp68;
#line 975
  send_entry_t *s_entry = IPDispatchP__SendQueue__head();

  IPDispatchP__radioBusy = FALSE;

  if (IPDispatchP__state == IPDispatchP__S_STOPPING) {
      IPDispatchP__RadioControl__stop();
      IPDispatchP__state = IPDispatchP__S_STOPPED;
      goto fail;
    }


  if (!IPDispatchP__PacketLink__wasDelivered(msg)) {


      if (s_entry->info->frags_sent == 0) {


          s_entry->info->policy.current++;
          if (s_entry->info->policy.current < s_entry->info->policy.nchoices) {

              IPDispatchP__sendTask__postTask();
              return;
            }
        }



      goto fail;
    }
  else 
#line 1003
    {

      s_entry->info->frags_sent++;
      goto done;
    }
  goto done;

  fail: 
    s_entry->info->failed = TRUE;
  if (s_entry->info->policy.dest[0] != 0xffff) {
    ;
    }
#line 1014
  (__nesc_temp68 = IPDispatchP__stats.tx_drop.data, __nesc_hton_uint8(__nesc_temp68, (__nesc_temp69 = __nesc_ntoh_uint8(__nesc_temp68)) + 1), __nesc_temp69);

  done: 
    s_entry->info->policy.actRetries = IPDispatchP__PacketLink__getRetries(msg);
  IPDispatchP__IPExtensions__reportTransmission(s_entry->info->local_flow_label, & s_entry->info->policy);

  if (-- s_entry->info->refcount == 0) {
#line 1020
    IPDispatchP__SendInfoPool__put(s_entry->info);
    }
#line 1021
  IPDispatchP__FragPool__put(s_entry->msg);
  IPDispatchP__SendEntryPool__put(s_entry);
  IPDispatchP__SendQueue__dequeue();

  IPDispatchP__sendTask__postTask();
}

# 179 "/home/tinyos/local/src/tinyos-2.x/tos/chips/cc2420/transmit/CC2420TransmitP.nc"
static error_t CC2420TransmitP__StdControl__stop(void )
#line 179
{
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 180
    {
      CC2420TransmitP__m_state = CC2420TransmitP__S_STOPPED;
      CC2420TransmitP__BackoffTimer__stop();
      CC2420TransmitP__CaptureSFD__disable();
      CC2420TransmitP__SpiResource__release();
      CC2420TransmitP__CSN__set();
    }
#line 186
    __nesc_atomic_end(__nesc_atomic); }
  return SUCCESS;
}

# 171 "/home/tinyos/local/src/tinyos-2.x/tos/chips/cc2420/receive/CC2420ReceiveP.nc"
static error_t CC2420ReceiveP__StdControl__stop(void )
#line 171
{
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 172
    {
      CC2420ReceiveP__m_state = CC2420ReceiveP__S_STOPPED;
      CC2420ReceiveP__reset_state();
      CC2420ReceiveP__CSN__set();
      CC2420ReceiveP__InterruptFIFOP__disable();
    }
#line 177
    __nesc_atomic_end(__nesc_atomic); }
  return SUCCESS;
}

#line 808
static void CC2420ReceiveP__reset_state(void )
#line 808
{
  CC2420ReceiveP__m_bytes_left = CC2420ReceiveP__RXFIFO_SIZE;
  /* atomic removed: atomic calls only */
#line 810
  CC2420ReceiveP__receivingPacket = FALSE;
  CC2420ReceiveP__m_timestamp_head = 0;
  CC2420ReceiveP__m_timestamp_size = 0;
  CC2420ReceiveP__m_missed_packets = 0;
}

# 199 "/home/tinyos/local/src/tinyos-2.x/tos/chips/cc2420/control/CC2420ControlP.nc"
static error_t CC2420ControlP__CC2420Power__stopVReg(void )
#line 199
{
  CC2420ControlP__m_state = CC2420ControlP__S_VREG_STOPPED;
  CC2420ControlP__RSTN__clr();
  CC2420ControlP__VREN__clr();
  CC2420ControlP__RSTN__set();
  return SUCCESS;
}

# 287 "/home/tinyos/local/src/tinyos-2.x/tos/lib/net/blip/IPRoutingP.nc"
static struct neigh_entry *IPRoutingP__getNeighEntry(cmpr_ip6_addr_t a)
#line 287
{
  int i;

#line 289
  for (i = 0; i < N_NEIGH; i++) {
      if (IPRoutingP__neigh_table[i].neighbor == a) {
        return &IPRoutingP__neigh_table[i];
        }
    }
#line 293
  return (void *)0;
}

#line 722
static void IPRoutingP__chooseNewRandomDefault(bool force)
#line 722
{
  uint8_t i;
  uint8_t numNeigh = 0;
  uint8_t chosenNeigh;
  bool useHops = TRUE;

  ;
  retry: 
    for (i = 1; i < N_NEIGH; i++) {
        if (!(((&IPRoutingP__neigh_table[i])->flags & T_VALID_MASK) == T_VALID_MASK)) {
#line 731
          break;
          }
#line 732
        if (&IPRoutingP__neigh_table[i] == IPRoutingP__default_route) {
#line 732
          continue;
          }
#line 733
        if ((useHops && IPRoutingP__neigh_table[i].hops < IPRoutingP__neigh_table[0].hops) || (
        !useHops && IPRoutingP__neigh_table[i].costEstimate < IPRoutingP__neigh_table[0].costEstimate)) {
            numNeigh++;
          }
      }


  if (numNeigh) {
      chosenNeigh = IPRoutingP__Random__rand16() % numNeigh;
      for (i = 1; i < N_NEIGH; i++) {
          if (&IPRoutingP__neigh_table[i] == IPRoutingP__default_route) {
#line 743
            continue;
            }
          if ((
#line 744
          useHops && IPRoutingP__neigh_table[i].hops < IPRoutingP__neigh_table[0].hops)
           || (!useHops && IPRoutingP__neigh_table[i].costEstimate < IPRoutingP__neigh_table[0].costEstimate)) {
              if (chosenNeigh) {
                  chosenNeigh--;
                }
              else 
#line 748
                {
                  IPRoutingP__default_route = &IPRoutingP__neigh_table[i];
                  IPRoutingP__default_route_failures = 0;
                  return;
                }
            }
        }
    }

  if (!force || !useHops) {
#line 757
    goto done;
    }
#line 758
  numNeigh = 0;
  useHops = FALSE;
  goto retry;

  done: 
    ;
  IPRoutingP__default_route = &IPRoutingP__neigh_table[0];
  IPRoutingP__default_route_failures = 0;
}

#line 622
static uint16_t IPRoutingP__getConfidence(struct neigh_entry *neigh)
#line 622
{

  uint16_t conf = 0;

#line 625
  if (neigh != (void *)0 && (neigh->flags & T_VALID_MASK) == T_VALID_MASK) {



      conf = neigh->stats[IPRoutingP__LONG_EPOCH].total;
    }
  return conf;
}

#line 1415
static uint8_t IPRoutingP__checkThresh(uint32_t firstVal, uint32_t secondVal, uint16_t thresh)
#line 1415
{
  if ((firstVal > secondVal && firstVal - secondVal <= thresh) || (
  secondVal >= firstVal && secondVal - firstVal <= thresh)) {
#line 1417
    return WITHIN_THRESH;
    }
#line 1418
  if (firstVal > secondVal && firstVal - secondVal > thresh) {
#line 1418
    return ABOVE_THRESH;
    }
#line 1419
  return BELOW_THRESH;
}

#line 716
static uint16_t IPRoutingP__getMetric(struct neigh_entry *r)
#line 716
{
  return r == (void *)0 || !((r->flags & T_VALID_MASK) == T_VALID_MASK) ? 
  0xffff : r->costEstimate + IPRoutingP__getLinkCost(r);
}

#line 658
static uint16_t IPRoutingP__getLinkCost(struct neigh_entry *neigh)
#line 658
{
  uint16_t conf;
#line 659
  uint16_t succ;

#line 660
  conf = IPRoutingP__getConfidence(neigh);
  succ = IPRoutingP__getSuccess(neigh);

  if (succ == 0 || conf == 0) {
#line 663
    return 0xff;
    }
#line 664
  return conf * 10 / succ;
}

#line 1404
static void IPRoutingP__swapNodes(struct neigh_entry *highNode, struct neigh_entry *lowNode)
#line 1404
{
  struct neigh_entry tempNode;

#line 1406
  if (highNode == (void *)0 || lowNode == (void *)0) {
#line 1406
    return;
    }
#line 1407
  ip_memcpy(&tempNode, highNode, sizeof(struct neigh_entry ));
  ip_memcpy(highNode, lowNode, sizeof(struct neigh_entry ));
  ip_memcpy(lowNode, &tempNode, sizeof(struct neigh_entry ));

  if (highNode == IPRoutingP__default_route) {
#line 1411
    IPRoutingP__default_route = lowNode;
    }
  else {
#line 1412
    if (lowNode == IPRoutingP__default_route) {
#line 1412
      IPRoutingP__default_route = highNode;
      }
    }
}

# 98 "/home/tinyos/local/src/tinyos-2.x/tos/chips/cc2420/packet/CC2420PacketP.nc"
static uint8_t CC2420PacketP__CC2420Packet__getNetwork(message_t *p_msg)
#line 98
{



  return __nesc_ntoh_leuint8(CC2420PacketP__CC2420PacketBody__getHeader(p_msg)->network.data);
}

# 106 "/home/tinyos/local/src/tinyos-2.x/tos/chips/cc2420/CC2420Ieee154MessageP.nc"
static ieee154_saddr_t CC2420Ieee154MessageP__Ieee154Packet__source(message_t *msg)
#line 106
{
  cc2420_header_t *header = CC2420Ieee154MessageP__CC2420PacketBody__getHeader(msg);

#line 108
  return __nesc_ntoh_leuint16(header->src.data);
}

# 2 "/home/tinyos/local/src/tinyos-2.x/tos/lib/net/blip/platform/CC2420ReadLqiC.nc"
static uint16_t adjustLQI(uint8_t val)
#line 2
{
  uint16_t result = 80 - (val - 50);

#line 4
  result = (result * result >> 3) * result >> 3;
  return result;
}

# 64 "/home/tinyos/local/src/tinyos-2.x/tos/lib/net/blip/IPAddressP.nc"
static struct in6_addr *IPAddressP__IPAddress__getPublicAddr(void )
#line 64
{
  __my_address.in6_u.u6_addr16[7] = (((uint16_t )TOS_NODE_ID << 8) | ((uint16_t )TOS_NODE_ID >> 8)) & 0xffff;
  return &__my_address;
}

# 1069 "/home/tinyos/local/src/tinyos-2.x/tos/lib/net/blip/IPDispatchP.nc"
static void IPDispatchP__IP__default__recv(uint8_t nxt_hdr, struct ip6_hdr *iph, 
void *payload, 
struct ip_metadata *meta)
#line 1071
{
}

# 25 "/home/tinyos/local/src/tinyos-2.x/tos/lib/net/blip/interfaces/IP.nc"
static void IPDispatchP__IP__recv(uint8_t arg_0x41134178, struct ip6_hdr *iph, void *payload, struct ip_metadata *meta){
#line 25
  switch (arg_0x41134178) {
#line 25
    case IANA_TCP:
#line 25
      TcpP__IP__recv(iph, payload, meta);
#line 25
      break;
#line 25
    case IANA_UDP:
#line 25
      UdpP__IP__recv(iph, payload, meta);
#line 25
      break;
#line 25
    case IANA_ICMP:
#line 25
      ICMPResponderP__IP__recv(iph, payload, meta);
#line 25
      break;
#line 25
    case IPV6_NONEXT:
#line 25
      IPRoutingP__TGenSend__recv(iph, payload, meta);
#line 25
      break;
#line 25
    default:
#line 25
      IPDispatchP__IP__default__recv(arg_0x41134178, iph, payload, meta);
#line 25
      break;
#line 25
    }
#line 25
}
#line 25
# 48 "/home/tinyos/local/src/tinyos-2.x/tos/lib/net/blip/TcpP.nc"
static void TcpP__tcplib_extern_closedone(struct TcpP__tcplib_sock *sock)
#line 48
{
  int cid = TcpP__find_client(sock);

#line 50
  TcpP__tcplib_init_sock(sock);
  if (cid < TcpP__N_CLIENTS) {
    TcpP__Tcp__closed(cid, 0);
    }
}

#line 24
static int TcpP__find_client(struct TcpP__tcplib_sock *conn)
#line 24
{
  int i;

#line 26
  for (i = 0; i < TcpP__N_CLIENTS; i++) 
    if (&TcpP__socks[i] == conn) {
#line 27
      break;
      }
  return i;
}

# 245 "/home/tinyos/local/src/tinyos-2.x/support/sdk/c/blip/libtcp/tcplib.c"
static int TcpP__tcplib_init_sock(struct TcpP__tcplib_sock *sock)
#line 245
{
  memset(sock, 0, sizeof(struct TcpP__tcplib_sock ) - sizeof(struct TcpP__tcplib_sock *));
  sock->mss = 200;
  sock->my_wind = 200;
  sock->cwnd = sock->mss;
  sock->ssthresh = 0xffff;
  TcpP__conn_add_once(sock);
  return 0;
}

# 47 "/home/tinyos/local/src/tinyos-2.x/tos/chips/msp430/pins/HplMsp430GeneralIOP.nc"
static void /*HplMsp430GeneralIOC.P56*/HplMsp430GeneralIOP__38__IO__toggle(void )
#line 47
{
#line 47
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 47
    * (volatile uint8_t * )49U ^= 0x01 << 6;
#line 47
    __nesc_atomic_end(__nesc_atomic); }
}

# 113 "/home/tinyos/local/src/tinyos-2.x/tos/lib/net/blip/TcpP.nc"
static error_t TcpP__Tcp__bind(uint8_t client, uint16_t port)
#line 113
{
  struct sockaddr_in6 addr;

#line 115
  ip_memclr(addr.sin6_addr.in6_u.u6_addr8, 16);
  addr.sin6_port = (((uint16_t )port << 8) | ((uint16_t )port >> 8)) & 0xffff;
  TcpP__tcplib_bind(&TcpP__socks[client], &addr);
  return SUCCESS;
}

# 47 "/home/tinyos/local/src/tinyos-2.x/tos/chips/msp430/pins/HplMsp430GeneralIOP.nc"
static void /*HplMsp430GeneralIOC.P54*/HplMsp430GeneralIOP__36__IO__toggle(void )
#line 47
{
#line 47
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 47
    * (volatile uint8_t * )49U ^= 0x01 << 4;
#line 47
    __nesc_atomic_end(__nesc_atomic); }
}

# 161 "/home/tinyos/local/src/tinyos-2.x/support/sdk/c/blip/libtcp/tcplib.c"
static void TcpP__tcplib_send_ack(struct TcpP__tcplib_sock *sock, int fin_seqno, uint8_t flags)
#line 161
{
  struct split_ip_msg *msg = TcpP__get_ipmsg(0);

  if (msg != (void *)0) {
      struct tcp_hdr *tcp_rep = (struct tcp_hdr *)(msg + 1);

#line 166
      tcp_rep->flags = flags;


      tcp_rep->seqno = ntoh32(sock->seqno);
      tcp_rep->ackno = ntoh32(sock->ackno + (fin_seqno ? 1 : 0));


      TcpP____tcplib_send(sock, msg);
      ip_free(msg);
    }
  else 
#line 175
    {
      ;
    }
}

#line 127
static struct split_ip_msg *TcpP__get_ipmsg(int plen)
#line 127
{
  struct split_ip_msg *msg = 
  (struct split_ip_msg *)ip_malloc(sizeof(struct split_ip_msg ) + sizeof(struct tcp_hdr ) + plen);

#line 130
  if (msg == (void *)0) {
#line 130
    return (void *)0;
    }
#line 131
  memset(msg, 0, sizeof(struct split_ip_msg ) + sizeof(struct tcp_hdr ));
  msg->hdr.nxt_hdr = IANA_TCP;
  msg->hdr.plen = (((uint16_t )(sizeof(struct tcp_hdr ) + plen) << 8) | ((uint16_t )(sizeof(struct tcp_hdr ) + plen) >> 8)) & 0xffff;

  msg->headers = (void *)0;
  msg->data = (void *)(msg + 1);
  msg->data_len = sizeof(struct tcp_hdr ) + plen;

  return msg;
}

static void TcpP____tcplib_send(struct TcpP__tcplib_sock *sock, 
struct split_ip_msg *msg)
#line 143
{
  struct tcp_hdr *tcph = TcpP__find_tcp_hdr(msg);

#line 145
  if (tcph == (void *)0) {
#line 145
    return;
    }
#line 146
  ip_memcpy(& msg->hdr.ip6_dst, & sock->r_ep.sin6_addr, 16);

  sock->flags &= ~TcpP__TCP_ACKPENDING;


  tcph->srcport = sock->l_ep.sin6_port;
  tcph->dstport = sock->r_ep.sin6_port;
  tcph->offset = sizeof(struct tcp_hdr ) * 4;
  tcph->window = (((uint16_t )sock->my_wind << 8) | ((uint16_t )sock->my_wind >> 8)) & 0xffff;
  tcph->chksum = 0;
  tcph->urgent = 0;

  TcpP__tcplib_send_out(msg, tcph);
}

# 81 "/home/tinyos/local/src/tinyos-2.x/tos/lib/net/blip/TcpP.nc"
static void TcpP__tcplib_send_out(struct split_ip_msg *msg, struct tcp_hdr *tcph)
#line 81
{
  ;
  TcpP__IPAddress__setSource(& msg->hdr);
  tcph->chksum = (((uint16_t )msg_cksum(msg, IANA_TCP) << 8) | ((uint16_t )msg_cksum(msg, IANA_TCP) >> 8)) & 0xffff;
  TcpP__IP__send(msg);
}

# 79 "/home/tinyos/local/src/tinyos-2.x/tos/lib/net/blip/IPAddressP.nc"
static void IPAddressP__IPAddress__setSource(struct ip6_hdr *hdr)
#line 79
{
  enum __nesc_unnamed4459 {
#line 80
    LOCAL, GLOBAL
  } 
#line 80
  type = GLOBAL;

  if (hdr->ip6_dst.in6_u.u6_addr8[0] == 0xff) {

      if ((hdr->ip6_dst.in6_u.u6_addr8[1] & 0x0f) <= 0x2) {
          type = LOCAL;
        }
    }
  else {
#line 87
    if (hdr->ip6_dst.in6_u.u6_addr8[0] == 0xfe) {

        if ((hdr->ip6_dst.in6_u.u6_addr8[1] & 0xf0) <= 0x80) {
            type = LOCAL;
          }
      }
    }
  if (type == GLOBAL && IPAddressP__IPAddress__haveAddress()) {
      IPAddressP__IPAddress__getIPAddr(& hdr->ip6_src);
    }
  else 
#line 96
    {
      IPAddressP__IPAddress__getLLAddr(& hdr->ip6_src);
    }
}

#line 59
static void IPAddressP__IPAddress__getIPAddr(struct in6_addr *addr)
#line 59
{
  __my_address.in6_u.u6_addr16[7] = (((uint16_t )TOS_NODE_ID << 8) | ((uint16_t )TOS_NODE_ID >> 8)) & 0xffff;
  ip_memcpy(addr, &__my_address, 16);
}

#line 53
static void IPAddressP__IPAddress__getLLAddr(struct in6_addr *addr)
#line 53
{
  __my_address.in6_u.u6_addr16[7] = (((uint16_t )TOS_NODE_ID << 8) | ((uint16_t )TOS_NODE_ID >> 8)) & 0xffff;
  ip_memcpy(addr->in6_u.u6_addr8, linklocal_prefix, 8);
  ip_memcpy(&addr->in6_u.u6_addr8[8], &__my_address.in6_u.u6_addr8[8], 8);
}

# 847 "/home/tinyos/local/src/tinyos-2.x/tos/lib/net/blip/IPDispatchP.nc"
static error_t IPDispatchP__IP__send(uint8_t prot, struct split_ip_msg *msg)
#line 847
{
  msg->hdr.nxt_hdr = prot;
  return IPDispatchP__IP__bareSend(prot, msg, (void *)0, 0);
}

# 878 "/home/tinyos/local/src/tinyos-2.x/tos/lib/net/blip/IPRoutingP.nc"
static uint8_t IPRoutingP__IPRouting__getHopLimit(void )
#line 878
{

  if (((&IPRoutingP__neigh_table[0])->flags & T_VALID_MASK) == T_VALID_MASK) {
    return IPRoutingP__neigh_table[0].hops + 1;
    }
  else {
#line 882
    return 0xf0;
    }
}

# 39 "/home/tinyos/local/src/tinyos-2.x/tos/lib/net/blip/IPExtensionP.nc"
static struct generic_header *IPExtensionP__buildTLVHdr(struct split_ip_msg *msg, 
int which, 
int n, int nxt_hdr)
#line 41
{


  int i;
  uint8_t *buf = ip_malloc(sizeof(struct ip6_ext ) + sizeof(struct generic_header ) * (n + 1));
  struct ip6_ext *real_hdr;
  struct generic_header *ghdrs;

#line 48
  if (buf == (void *)0) {
#line 48
    return (void *)0;
    }
#line 49
  ghdrs = (struct generic_header *)buf;
  real_hdr = (struct ip6_ext *)(ghdrs + (n + 1));


  real_hdr->len = sizeof(struct ip6_ext );

  ghdrs[0].len = sizeof(struct ip6_ext );
  ghdrs[0].hdr.data = (uint8_t *)real_hdr;
  ghdrs[0].next = msg->headers;

  for (i = 0; i < n; i++) {
      struct tlv_hdr *this_hdr;

#line 61
      if (which == 0) {
          ;
          this_hdr = IPExtensionP__DestinationExt__getHeader(i, 0, nxt_hdr, & msg->hdr);
        }
      else 
#line 64
        {
          this_hdr = IPExtensionP__HopByHopExt__getHeader(i, 0, nxt_hdr, & msg->hdr);
        }

      ;
      if (this_hdr == (void *)0) {
#line 69
        continue;
        }
      real_hdr->len += this_hdr->len;
      ghdrs[i + 1].len = this_hdr->len;
      ghdrs[i + 1].hdr.data = (uint8_t *)this_hdr;
      ghdrs[i].next = &ghdrs[i + 1];
      ghdrs[i + 1].next = msg->headers;
    }
  if (real_hdr->len == sizeof(struct ip6_ext )) {
      ip_free(buf);
      return (void *)0;
    }
  else 
#line 80
    {
      real_hdr->nxt_hdr = msg->hdr.nxt_hdr;
      msg->headers = ghdrs;
      return ghdrs;
    }
}

# 211 "/home/tinyos/local/src/tinyos-2.x/tos/lib/net/blip/IPDispatchP.nc"
static send_info_t *IPDispatchP__getSendInfo(void )
#line 211
{
  send_info_t *ret = IPDispatchP__SendInfoPool__get();

#line 213
  if (ret == (void *)0) {
#line 213
    return ret;
    }
#line 214
  ret->refcount = 1;
  ret->failed = FALSE;
  ret->frags_sent = 0;
  return ret;
}

# 774 "/home/tinyos/local/src/tinyos-2.x/tos/lib/net/blip/IPRoutingP.nc"
static error_t IPRoutingP__IPRouting__getNextHop(struct ip6_hdr *hdr, 
struct ip6_route *sh, 
ieee154_saddr_t prev_hop, 
send_policy_t *ret)
#line 777
{

  int i;



  prev_hop = 0;
  ret->retries = 5;
  ret->delay = 15 % IPRoutingP__Random__rand16() + 15;
  ret->current = 0;
  ret->nchoices = 0;




  if (sh != (void *)0) {
      ;
    }





  if (sh != (void *)0 && (sh->type & ~IP6ROUTE_FLAG_MASK) == IP6ROUTE_TYPE_SOURCE) {



      if (sh->segs_remain == 0) {
#line 804
        return FAIL;
        }
      ret->dest[0] = (((uint16_t )sh->hops[(sh->len - sizeof(struct ip6_route )) / sizeof(uint16_t ) - sh->segs_remain] >> 8) | ((uint16_t )sh->hops[(sh->len - sizeof(struct ip6_route )) / sizeof(uint16_t ) - sh->segs_remain] << 8)) & 0xffff;
      ret->nchoices = 1;
    }
  else {
#line 809
    if (hdr->ip6_dst.in6_u.u6_addr8[0] == 0xff && (
    hdr->ip6_dst.in6_u.u6_addr8[1] & 0xf) <= 0x03) {


        ret->dest[0] = 0xffff;
        ret->nchoices = 1;
        ret->retries = 0;
        ret->delay = 0;
        return SUCCESS;
      }
    else {
#line 818
      if (cmpPfx(hdr->ip6_dst.in6_u.u6_addr8, linklocal_prefix)) {
          ret->dest[0] = (((uint16_t )hdr->ip6_dst.in6_u.u6_addr16[7] >> 8) | ((uint16_t )hdr->ip6_dst.in6_u.u6_addr16[7] << 8)) & 0xffff;
          ret->nchoices = 1;
          return SUCCESS;
        }
      }
    }
#line 824
  if (IPRoutingP__getNeighEntry((((uint16_t )hdr->ip6_dst.in6_u.u6_addr16[7] >> 8) | ((uint16_t )hdr->ip6_dst.in6_u.u6_addr16[7] << 8)) & 0xffff) != (void *)0) {
      ;
      ret->dest[ret->nchoices++] = (((uint16_t )hdr->ip6_dst.in6_u.u6_addr16[7] >> 8) | ((uint16_t )hdr->ip6_dst.in6_u.u6_addr16[7] << 8)) & 0xffff;
    }
#line 849
  if ((IPRoutingP__default_route->flags & T_VALID_MASK) == T_VALID_MASK && prev_hop != IPRoutingP__default_route->neighbor) {
      ret->dest[ret->nchoices++] = IPRoutingP__default_route->neighbor;
    }
  else 
#line 851
    {
      ;




      IPRoutingP__traffic_sent = FALSE;
      return FAIL;
    }
  i = 0;
  while (ret->nchoices < N_PARENT_CHOICES && i < N_NEIGH) {

      if ((
#line 862
      (&IPRoutingP__neigh_table[i])->flags & T_VALID_MASK) == T_VALID_MASK && 
      &IPRoutingP__neigh_table[i] != IPRoutingP__default_route && 
      IPRoutingP__neigh_table[i].neighbor != prev_hop) {
          ret->dest[ret->nchoices++] = IPRoutingP__neigh_table[i].neighbor;
        }
      i++;
    }

  if (ret->nchoices == 0) {
    return FAIL;
    }
  ;

  return SUCCESS;
}

# 88 "/home/tinyos/local/src/tinyos-2.x/tos/system/PoolP.nc"
static /*IPDispatchC.SendEntryPool.PoolP*/PoolP__1__pool_t */*IPDispatchC.SendEntryPool.PoolP*/PoolP__1__Pool__get(void )
#line 88
{
  if (/*IPDispatchC.SendEntryPool.PoolP*/PoolP__1__free) {
      /*IPDispatchC.SendEntryPool.PoolP*/PoolP__1__pool_t *rval = /*IPDispatchC.SendEntryPool.PoolP*/PoolP__1__queue[/*IPDispatchC.SendEntryPool.PoolP*/PoolP__1__index];

#line 91
      /*IPDispatchC.SendEntryPool.PoolP*/PoolP__1__queue[/*IPDispatchC.SendEntryPool.PoolP*/PoolP__1__index] = (void *)0;
      /*IPDispatchC.SendEntryPool.PoolP*/PoolP__1__free--;
      /*IPDispatchC.SendEntryPool.PoolP*/PoolP__1__index++;
      if (/*IPDispatchC.SendEntryPool.PoolP*/PoolP__1__index == 14) {
          /*IPDispatchC.SendEntryPool.PoolP*/PoolP__1__index = 0;
        }
      ;
      return rval;
    }
  return (void *)0;
}

#line 88
static /*IPDispatchC.FragPool.PoolP*/PoolP__0__pool_t */*IPDispatchC.FragPool.PoolP*/PoolP__0__Pool__get(void )
#line 88
{
  if (/*IPDispatchC.FragPool.PoolP*/PoolP__0__free) {
      /*IPDispatchC.FragPool.PoolP*/PoolP__0__pool_t *rval = /*IPDispatchC.FragPool.PoolP*/PoolP__0__queue[/*IPDispatchC.FragPool.PoolP*/PoolP__0__index];

#line 91
      /*IPDispatchC.FragPool.PoolP*/PoolP__0__queue[/*IPDispatchC.FragPool.PoolP*/PoolP__0__index] = (void *)0;
      /*IPDispatchC.FragPool.PoolP*/PoolP__0__free--;
      /*IPDispatchC.FragPool.PoolP*/PoolP__0__index++;
      if (/*IPDispatchC.FragPool.PoolP*/PoolP__0__index == 14) {
          /*IPDispatchC.FragPool.PoolP*/PoolP__0__index = 0;
        }
      ;
      return rval;
    }
  return (void *)0;
}

# 97 "/home/tinyos/local/src/tinyos-2.x/tos/system/QueueC.nc"
static error_t /*IPDispatchC.QueueC*/QueueC__0__Queue__enqueue(/*IPDispatchC.QueueC*/QueueC__0__queue_t newVal)
#line 97
{
  if (/*IPDispatchC.QueueC*/QueueC__0__Queue__size() < /*IPDispatchC.QueueC*/QueueC__0__Queue__maxSize()) {
      ;
      /*IPDispatchC.QueueC*/QueueC__0__queue[/*IPDispatchC.QueueC*/QueueC__0__tail] = newVal;
      /*IPDispatchC.QueueC*/QueueC__0__tail++;
      if (/*IPDispatchC.QueueC*/QueueC__0__tail == 14) {
#line 102
        /*IPDispatchC.QueueC*/QueueC__0__tail = 0;
        }
#line 103
      /*IPDispatchC.QueueC*/QueueC__0__size++;
      /*IPDispatchC.QueueC*/QueueC__0__printQueue();
      return SUCCESS;
    }
  else {
      return FAIL;
    }
}

# 180 "/home/tinyos/local/src/tinyos-2.x/support/sdk/c/blip/libtcp/tcplib.c"
static void TcpP__tcplib_send_rst(struct ip6_hdr *iph, struct tcp_hdr *tcph)
#line 180
{
  struct split_ip_msg *msg = TcpP__get_ipmsg(0);
  struct tcp_hdr *tcp_rep;

  if (msg == (void *)0) {
      return;
    }

  tcp_rep = (struct tcp_hdr *)(msg + 1);

  tcp_rep->flags = TCP_FLAG_RST | TCP_FLAG_ACK;

  tcp_rep->ackno = ntoh32(ntoh32(tcph->seqno) + 1);
  tcp_rep->seqno = tcph->ackno;
#line 193
  ;

  tcp_rep->srcport = tcph->dstport;
  tcp_rep->dstport = tcph->srcport;
  tcp_rep->offset = sizeof(struct tcp_hdr ) * 4;
  tcp_rep->window = 0;
  tcp_rep->chksum = 0;
  tcp_rep->urgent = 0;

  ip_memcpy(& msg->hdr.ip6_dst, & iph->ip6_src, 16);

  TcpP__tcplib_send_out(msg, tcp_rep);
  ip_free(msg);
}

#line 272
static void TcpP__reset_ssthresh(struct TcpP__tcplib_sock *conn)
#line 272
{
  uint16_t new_ssthresh = (conn->cwnd > conn->r_wind ? conn->r_wind : conn->cwnd) / 2;

#line 274
  if (new_ssthresh < 2 * conn->mss) {
    new_ssthresh = 2 * conn->mss;
    }
#line 276
  conn->ssthresh = new_ssthresh;
}

#line 209
static int TcpP__tcplib_output(struct TcpP__tcplib_sock *sock, uint32_t sseqno)
#line 209
{



  int seg_size = sock->seqno - sseqno > sock->r_wind ? sock->r_wind : sock->seqno - sseqno;

#line 214
  ;
  seg_size = seg_size > sock->cwnd ? sock->cwnd : seg_size;
  while (seg_size > 0 && sock->seqno > sseqno) {

      struct split_ip_msg *msg = TcpP__get_ipmsg(seg_size);
      struct tcp_hdr *tcph;
      uint8_t *data;

#line 221
      if (msg == (void *)0) {
#line 221
        return -1;
        }
#line 222
      tcph = (struct tcp_hdr *)(msg + 1);
      data = (uint8_t *)(tcph + 1);

      tcph->flags = TCP_FLAG_ACK;
      tcph->seqno = ntoh32(sseqno);
      tcph->ackno = ntoh32(sock->ackno);

      ;



      if (seg_size != TcpP__circ_buf_read(sock->tx_buf, sseqno, data, seg_size)) {
          ;
        }
      TcpP____tcplib_send(sock, msg);
      ip_free(msg);

      sseqno += seg_size;
      seg_size = sock->seqno - sseqno > sock->mss ? sock->mss : sock->seqno - sseqno;
    }
  return 0;
}

# 53 "/home/tinyos/local/src/tinyos-2.x/support/sdk/c/blip/libtcp/circ.c"
static void TcpP__get_ptr_off_1(struct TcpP__circ_buf *b, uint32_t sseqno, int len, 
uint8_t **writeptr, int *w_len)
#line 54
{
  uint8_t *endptr = b->data_start + b->data_len;
  int offset;

  *writeptr = (void *)0;
  *w_len = len;


  offset = sseqno - b->head_seqno;
  if (b->data_head + offset < endptr) {
      *writeptr = b->data_head + offset;
    }
  else 
#line 65
    {
      offset -= endptr - b->data_head;
      *writeptr = b->data_start + offset;
    }
  if (*writeptr + *w_len > endptr) {
      *w_len = endptr - *writeptr;
    }
}

# 133 "/home/tinyos/local/src/tinyos-2.x/tos/lib/net/blip/TcpP.nc"
static error_t TcpP__Tcp__close(uint8_t client)
#line 133
{
  if (!TcpP__tcplib_close(&TcpP__socks[client])) {
    return SUCCESS;
    }
#line 136
  return FAIL;
}

# 675 "/home/tinyos/local/src/tinyos-2.x/support/sdk/c/blip/libtcp/tcplib.c"
static int TcpP__tcplib_close(struct TcpP__tcplib_sock *sock)
#line 675
{
  int rc = 0;

  switch (sock->state) {

      case TcpP__TCP_CLOSE_WAIT: 
        TcpP__tcplib_send_ack(sock, 1, TCP_FLAG_ACK | TCP_FLAG_FIN);
      sock->timer.retx = 6;
      sock->state = TcpP__TCP_LAST_ACK;
      break;

      case TcpP__TCP_ESTABLISHED: 

        TcpP__tcplib_send_ack(sock, 0, TCP_FLAG_ACK | TCP_FLAG_FIN);
      sock->timer.retx = TcpP__TCPLIB_2MSL;
      sock->state = TcpP__TCP_FIN_WAIT_1;
      break;
      case TcpP__TCP_SYN_SENT: 
        sock->state = TcpP__TCP_CLOSED;
      break;
      default: 

        rc = -1;
    }
  return rc;
}

# 128 "/home/tinyos/local/src/tinyos-2.x/tos/lib/net/blip/TcpP.nc"
static error_t TcpP__Tcp__send(uint8_t client, void *payload, uint16_t len)
#line 128
{
  if (TcpP__tcplib_send(&TcpP__socks[client], payload, len) < 0) {
#line 129
    return FAIL;
    }
#line 130
  return SUCCESS;
}

# 226 "/home/tinyos/local/src/tinyos-2.x/tos/lib/net/blip/shell/UDPShellP.nc"
static int UDPShellP__lookup_cmd(char *cmd, int dbsize, struct UDPShellP__cmd_name *db)
#line 226
{
  int i;

#line 228
  for (i = 0; i < dbsize; i++) {

      if (
#line 229
      memcmp(cmd, db[i].c_name, db[i].c_len) == 0
       && cmd[db[i].c_len] == '\0') {
        return i;
        }
    }
#line 233
  return UDPShellP__CMD_NO_CMD;
}

# 128 "/home/tinyos/local/src/tinyos-2.x/tos/lib/net/blip/UdpP.nc"
static error_t UdpP__UDP__sendto(uint8_t clnt, struct sockaddr_in6 *dest, void *payload, 
uint16_t len)
#line 129
{
  unsigned int __nesc_temp91;
  unsigned char *__nesc_temp90;
#line 130
  struct split_ip_msg *msg;
  struct udp_hdr *udp;
  struct generic_header *g_udp;
  error_t rc;



  msg = (struct split_ip_msg *)ip_malloc(sizeof(struct split_ip_msg ) + 
  sizeof(struct udp_hdr ) + 
  sizeof(struct generic_header ));

  if (msg == (void *)0) {
      ;
      return ERETRY;
    }
  udp = (struct udp_hdr *)(msg + 1);
  g_udp = (struct generic_header *)(udp + 1);


  ip_memclr((uint8_t *)msg, sizeof(struct split_ip_msg ));
  ip_memclr((uint8_t *)udp, sizeof(struct udp_hdr ));

  ip_memcpy(& msg->hdr.ip6_dst, dest->sin6_addr.in6_u.u6_addr8, 16);
  UdpP__IPAddress__setSource(& msg->hdr);

  if (UdpP__local_ports[clnt] == 0 && (UdpP__local_ports[clnt] = UdpP__alloc_lport(clnt)) == 0) {
      ip_free(msg);
      return FAIL;
    }
  udp->srcport = UdpP__local_ports[clnt];
  udp->dstport = dest->sin6_port;
  udp->len = (((uint16_t )(len + sizeof(struct udp_hdr )) << 8) | ((uint16_t )(len + sizeof(struct udp_hdr )) >> 8)) & 0xffff;
  udp->chksum = 0;


  g_udp->len = sizeof(struct udp_hdr );
  g_udp->hdr.udp = udp;
  g_udp->next = (void *)0;
  msg->headers = g_udp;
  msg->data_len = len;
  msg->data = payload;
  msg->hdr.plen = udp->len;

  udp->chksum = (((uint16_t )msg_cksum(msg, IANA_UDP) << 8) | ((uint16_t )msg_cksum(msg, IANA_UDP) >> 8)) & 0xffff;

  rc = UdpP__IP__send(msg);
  (__nesc_temp90 = UdpP__stats.sent.data, __nesc_hton_uint16(__nesc_temp90, (__nesc_temp91 = __nesc_ntoh_uint16(__nesc_temp90)) + 1), __nesc_temp91);

  ip_free(msg);
  return rc;
}

# 156 "/home/tinyos/local/src/tinyos-2.x/tos/lib/net/blip/IPRoutingP.nc"
static void IPRoutingP__IPRouting__reset(void )
#line 156
{
  int i;

  for (i = 0; i < N_NEIGH; i++) {
      IPRoutingP__neigh_table[i].flags = 0;
      IPRoutingP__clearStats(&IPRoutingP__neigh_table[i]);
    }









  if (!IPRoutingP__soliciting) {
      IPRoutingP__ICMP__sendSolicitations();
      IPRoutingP__soliciting = TRUE;
    }

  IPRoutingP__default_route_failures = 0;
  IPRoutingP__default_route = &IPRoutingP__neigh_table[0];


  IPRoutingP__last_qual = 0xffff;
  IPRoutingP__last_hops = 0xff;

  IPRoutingP__traffic_sent = FALSE;
  IPRoutingP__restartTrafficGen();
}

# 71 "/home/tinyos/local/src/tinyos-2.x/tos/lib/net/blip/ICMPResponderP.nc"
static void ICMPResponderP__ICMP__sendSolicitations(void )
#line 71
{
  uint16_t jitter = ICMPResponderP__Random__rand16() % TRICKLE_JITTER;

#line 73
  if (ICMPResponderP__Solicitation__isRunning()) {
#line 73
    return;
    }
#line 74
  ICMPResponderP__solicitation_period = TRICKLE_PERIOD;
  ICMPResponderP__Solicitation__startOneShot(jitter);
}

# 62 "/home/tinyos/local/src/tinyos-2.x/tos/lib/timer/Timer.nc"
static void ICMPResponderP__Solicitation__startOneShot(uint32_t dt){
#line 62
  /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__Timer__startOneShot(4U, dt);
#line 62
}
#line 62
# 109 "/home/tinyos/local/src/tinyos-2.x/tos/lib/net/blip/IPRoutingP.nc"
static void IPRoutingP__restartTrafficGen(void )
#line 109
{
  IPRoutingP__traffic_interval = TGEN_BASE_TIME;

  IPRoutingP__traffic_interval += IPRoutingP__Random__rand16() % TGEN_BASE_TIME;
  if (IPRoutingP__TrafficGenTimer__isRunning()) {
    IPRoutingP__TrafficGenTimer__stop();
    }
#line 115
  IPRoutingP__traffic_sent = FALSE;

  IPRoutingP__TrafficGenTimer__startOneShot(IPRoutingP__traffic_interval);
}

# 62 "/home/tinyos/local/src/tinyos-2.x/tos/lib/timer/Timer.nc"
static void IPRoutingP__TrafficGenTimer__startOneShot(uint32_t dt){
#line 62
  /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__Timer__startOneShot(7U, dt);
#line 62
}
#line 62
# 1313 "/home/tinyos/local/src/tinyos-2.x/tos/lib/net/blip/IPRoutingP.nc"
static void IPRoutingP__evictNeighbor(struct neigh_entry *neigh)
#line 1313
{
  struct neigh_entry *iterator;
  bool reset_default = FALSE;

  ;
  ;

  neigh->flags &= ~T_VALID_MASK;

  if (neigh == IPRoutingP__default_route) {
      reset_default = TRUE;
    }

  ip_memclr((uint8_t *)neigh, sizeof(struct neigh_entry ));
  for (iterator = neigh; iterator < &IPRoutingP__neigh_table[N_NEIGH - 1]; iterator++) {
      if (!(((iterator + 1)->flags & T_VALID_MASK) == T_VALID_MASK)) {
#line 1328
        break;
        }
#line 1329
      IPRoutingP__swapNodes(iterator, iterator + 1);
    }

  if (reset_default) {


      IPRoutingP__restartTrafficGen();
      IPRoutingP__default_route = &IPRoutingP__neigh_table[0];
      IPRoutingP__default_route_failures = 0;
    }

  IPRoutingP__printTable();
}

# 78 "/home/tinyos/local/src/tinyos-2.x/tos/lib/net/blip/ICMPResponderP.nc"
static void ICMPResponderP__ICMP__sendAdvertisements(void )
#line 78
{


  uint16_t jitter = ICMPResponderP__Random__rand16() % TRICKLE_JITTER;

#line 82
  if (ICMPResponderP__Advertisement__isRunning()) {
#line 82
    return;
    }
#line 83
  ICMPResponderP__advertisement_period = TRICKLE_PERIOD;
  ICMPResponderP__Advertisement__startOneShot(jitter);
}

# 62 "/home/tinyos/local/src/tinyos-2.x/tos/lib/timer/Timer.nc"
static void ICMPResponderP__Advertisement__startOneShot(uint32_t dt){
#line 62
  /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__Timer__startOneShot(5U, dt);
#line 62
}
#line 62
# 375 "/home/tinyos/local/src/tinyos-2.x/tos/lib/net/blip/IPDispatchP.nc"
static reconstruct_t *IPDispatchP__get_reconstruct(ieee154_saddr_t src, uint16_t tag)
#line 375
{
  reconstruct_t *ret = (void *)0;
  int i;

#line 378
  for (i = 0; i < N_RECONSTRUCTIONS; i++) {
      reconstruct_t *recon = (reconstruct_t *)&IPDispatchP__recon_data[i];

#line 380
      ;

      if (recon->tag == tag && 
      recon->metadata.sender == src) {

          if (recon->timeout > T_UNUSED) {

              recon->timeout = T_ACTIVE;
              return recon;
            }
          else {
#line 390
            if (recon->timeout < T_UNUSED) {


                return (void *)0;
              }
            }
        }
#line 396
      if (recon->timeout == T_UNUSED) {
        ret = recon;
        }
    }
#line 399
  return ret;
}

# 764 "/home/tinyos/local/src/tinyos-2.x/tos/chips/cc2420/receive/CC2420ReceiveP.nc"
static void CC2420ReceiveP__waitForNextPacket(void )
#line 764
{
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 765
    {
      if (CC2420ReceiveP__m_state == CC2420ReceiveP__S_STOPPED) {
          CC2420ReceiveP__SpiResource__release();
          {
#line 768
            __nesc_atomic_end(__nesc_atomic); 
#line 768
            return;
          }
        }
      CC2420ReceiveP__receivingPacket = FALSE;
#line 783
      if ((CC2420ReceiveP__m_missed_packets && CC2420ReceiveP__FIFO__get()) || !CC2420ReceiveP__FIFOP__get()) {

          if (CC2420ReceiveP__m_missed_packets) {
              CC2420ReceiveP__m_missed_packets--;
            }





          CC2420ReceiveP__beginReceive();
        }
      else 
        {

          CC2420ReceiveP__m_state = CC2420ReceiveP__S_STARTED;
          CC2420ReceiveP__m_missed_packets = 0;
          CC2420ReceiveP__SpiResource__release();
        }
    }
#line 802
    __nesc_atomic_end(__nesc_atomic); }
}

#line 711
static void CC2420ReceiveP__beginReceive(void )
#line 711
{
  CC2420ReceiveP__m_state = CC2420ReceiveP__S_RX_LENGTH;
  /* atomic removed: atomic calls only */
#line 713
  CC2420ReceiveP__receivingPacket = TRUE;
  if (CC2420ReceiveP__SpiResource__isOwner()) {
      CC2420ReceiveP__receive();
    }
  else {
#line 717
    if (CC2420ReceiveP__SpiResource__immediateRequest() == SUCCESS) {
        CC2420ReceiveP__receive();
      }
    else {
        CC2420ReceiveP__SpiResource__request();
      }
    }
}

#line 754
static void CC2420ReceiveP__receive(void )
#line 754
{
  CC2420ReceiveP__CSN__clr();
  CC2420ReceiveP__RXFIFO__beginRead((uint8_t *)CC2420ReceiveP__CC2420PacketBody__getHeader(CC2420ReceiveP__m_p_rx_buf), 1);
}

# 189 "/home/tinyos/local/src/tinyos-2.x/tos/chips/cc2420/spi/CC2420SpiP.nc"
static cc2420_status_t CC2420SpiP__Fifo__beginRead(uint8_t addr, uint8_t *data, 
uint8_t len)
#line 190
{

  cc2420_status_t status = 0;

  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 194
    {
      if (CC2420SpiP__WorkingState__isIdle()) {
          {
            unsigned char __nesc_temp = 
#line 196
            status;

            {
#line 196
              __nesc_atomic_end(__nesc_atomic); 
#line 196
              return __nesc_temp;
            }
          }
        }
    }
#line 200
    __nesc_atomic_end(__nesc_atomic); }
#line 200
  CC2420SpiP__m_addr = addr | 0x40;

  status = CC2420SpiP__SpiByte__write(CC2420SpiP__m_addr);
  CC2420SpiP__Fifo__continueRead(addr, data, len);

  return status;
}

#line 329
static void CC2420SpiP__SpiPacket__sendDone(uint8_t *tx_buf, uint8_t *rx_buf, 
uint16_t len, error_t error)
#line 330
{
  if (CC2420SpiP__m_addr & 0x40) {
      CC2420SpiP__Fifo__readDone(CC2420SpiP__m_addr & ~0x40, rx_buf, len, error);
    }
  else 
#line 333
    {
      CC2420SpiP__Fifo__writeDone(CC2420SpiP__m_addr, tx_buf, len, error);
    }
}

# 728 "/home/tinyos/local/src/tinyos-2.x/tos/chips/cc2420/receive/CC2420ReceiveP.nc"
static void CC2420ReceiveP__flush(void )
#line 728
{








  CC2420ReceiveP__reset_state();

  CC2420ReceiveP__CSN__set();
  CC2420ReceiveP__CSN__clr();
  CC2420ReceiveP__SFLUSHRX__strobe();
  CC2420ReceiveP__SFLUSHRX__strobe();
  CC2420ReceiveP__CSN__set();
  CC2420ReceiveP__SpiResource__release();
  CC2420ReceiveP__waitForNextPacket();
}

# 456 "/home/tinyos/local/src/tinyos-2.x/tos/chips/cc2420/control/CC2420ControlP.nc"
static void CC2420ControlP__writeFsctrl(void )
#line 456
{
  uint8_t channel;

  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 459
    {
      channel = CC2420ControlP__m_channel;
    }
#line 461
    __nesc_atomic_end(__nesc_atomic); }

  CC2420ControlP__FSCTRL__write((1 << CC2420_FSCTRL_LOCK_THR) | (((
  channel - 11) * 5 + 357) << CC2420_FSCTRL_FREQ));
}







static void CC2420ControlP__writeMdmctrl0(void )
#line 473
{
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 474
    {
      CC2420ControlP__MDMCTRL0__write((((((((1 << CC2420_MDMCTRL0_RESERVED_FRAME_MODE) | ((
      CC2420ControlP__addressRecognition && CC2420ControlP__hwAddressRecognition) << CC2420_MDMCTRL0_ADR_DECODE)) | (
      2 << CC2420_MDMCTRL0_CCA_HYST)) | (
      3 << CC2420_MDMCTRL0_CCA_MOD)) | (
      1 << CC2420_MDMCTRL0_AUTOCRC)) | ((
      CC2420ControlP__autoAckEnabled && CC2420ControlP__hwAutoAckDefault) << CC2420_MDMCTRL0_AUTOACK)) | (
      0 << CC2420_MDMCTRL0_AUTOACK)) | (
      2 << CC2420_MDMCTRL0_PREAMBLE_LENGTH));
    }
#line 483
    __nesc_atomic_end(__nesc_atomic); }
}







static void CC2420ControlP__writeId(void )
#line 492
{
  nxle_uint16_t id[2];

  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 495
    {
      __nesc_hton_leuint16(id[0].data, CC2420ControlP__m_pan);
      __nesc_hton_leuint16(id[1].data, CC2420ControlP__m_short_addr);
    }
#line 498
    __nesc_atomic_end(__nesc_atomic); }

  CC2420ControlP__PANID__write(0, (uint8_t *)&id, sizeof id);
}

# 143 "/home/tinyos/local/src/tinyos-2.x/tos/system/StateImplP.nc"
static uint8_t StateImplP__State__getState(uint8_t id)
#line 143
{
  uint8_t theState;

#line 145
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 145
    theState = StateImplP__state[id];
#line 145
    __nesc_atomic_end(__nesc_atomic); }
  return theState;
}

# 220 "/home/tinyos/local/src/tinyos-2.x/tos/chips/cc2420/link/PacketLinkP.nc"
static void PacketLinkP__signalDone(error_t error)
#line 220
{
  PacketLinkP__DelayTimer__stop();
  PacketLinkP__SendState__toIdle();
  __nesc_hton_uint16(PacketLinkP__CC2420PacketBody__getMetadata(PacketLinkP__currentSendMsg)->maxRetries.data, PacketLinkP__totalRetries);
  PacketLinkP__Send__sendDone(PacketLinkP__currentSendMsg, error);
}

# 62 "/home/tinyos/local/src/tinyos-2.x/tos/lib/timer/VirtualizeTimerC.nc"
static void /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__fireTimers(uint32_t now)
{
  uint8_t num;

  for (num = 0; num < /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__NUM_TIMERS; num++) 
    {
      /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__Timer_t *timer = &/*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__m_timers[num];

      if (timer->isrunning) 
        {
          uint32_t elapsed = now - timer->t0;

          if (elapsed >= timer->dt) 
            {
              if (timer->isoneshot) {
                timer->isrunning = FALSE;
                }
              else {
#line 79
                timer->t0 += timer->dt;
                }
              /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__Timer__fired(num);
              break;
            }
        }
    }
  /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__updateFromTimer__postTask();
}

# 885 "/home/tinyos/local/src/tinyos-2.x/tos/lib/net/blip/IPRoutingP.nc"
static uint16_t IPRoutingP__IPRouting__getQuality(void )
#line 885
{
  if (((&IPRoutingP__neigh_table[0])->flags & T_VALID_MASK) == T_VALID_MASK) {
    return IPRoutingP__getMetric(&IPRoutingP__neigh_table[0]);
    }
  else {
#line 888
    return 0xffff;
    }
}

# 143 "/home/tinyos/local/src/tinyos-2.x/tos/lib/timer/VirtualizeTimerC.nc"
static void /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__Timer__startPeriodic(uint8_t num, uint32_t dt)
{
  /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__startTimer(num, /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC__0__TimerFrom__getNow(), dt, FALSE);
}

# 136 "/home/tinyos/local/src/tinyos-2.x/tos/lib/timer/TransformAlarmC.nc"
static void /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__Alarm__startAt(/*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__to_size_type t0, /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__to_size_type dt)
{
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
    {
      /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__m_t0 = t0;
      /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__m_dt = dt;
      /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC__0__set_alarm();
    }
#line 143
    __nesc_atomic_end(__nesc_atomic); }
}

# 56 "/home/tinyos/local/src/tinyos-2.x/tos/lib/net/blip/UdpP.nc"
static error_t UdpP__UDP__bind(uint8_t clnt, uint16_t port)
#line 56
{
  int i;

#line 58
  port = (((uint16_t )port << 8) | ((uint16_t )port >> 8)) & 0xffff;
  if (port > 0) {
      for (i = 0; i < UdpP__N_CLIENTS; i++) 
        if (i != clnt && UdpP__local_ports[i] == port) {
          return FAIL;
          }
    }
#line 64
  UdpP__local_ports[clnt] = port;
  return SUCCESS;
}

# 81 "/home/tinyos/local/src/tinyos-2.x/tos/chips/cc2420/csma/CC2420CsmaP.nc"
static error_t CC2420CsmaP__SplitControl__start(void )
#line 81
{
  if (CC2420CsmaP__SplitControlState__requestState(CC2420CsmaP__S_STARTING) == SUCCESS) {
      CC2420CsmaP__CC2420Power__startVReg();
      return SUCCESS;
    }
  else {
#line 86
    if (CC2420CsmaP__SplitControlState__isState(CC2420CsmaP__S_STARTED)) {
        return EALREADY;
      }
    else {
#line 89
      if (CC2420CsmaP__SplitControlState__isState(CC2420CsmaP__S_STARTING)) {
          return SUCCESS;
        }
      }
    }
#line 93
  return EBUSY;
}

# 53 "/home/tinyos/local/src/tinyos-2.x/tos/chips/msp430/pins/HplMsp430InterruptP.nc"
__attribute((wakeup)) __attribute((interrupt(8)))  void sig_PORT1_VECTOR(void )
{
  volatile int n = P1IFG & P1IE;

  if (n & (1 << 0)) {
#line 57
      HplMsp430InterruptP__Port10__fired();
#line 57
      return;
    }
#line 58
  if (n & (1 << 1)) {
#line 58
      HplMsp430InterruptP__Port11__fired();
#line 58
      return;
    }
#line 59
  if (n & (1 << 2)) {
#line 59
      HplMsp430InterruptP__Port12__fired();
#line 59
      return;
    }
#line 60
  if (n & (1 << 3)) {
#line 60
      HplMsp430InterruptP__Port13__fired();
#line 60
      return;
    }
#line 61
  if (n & (1 << 4)) {
#line 61
      HplMsp430InterruptP__Port14__fired();
#line 61
      return;
    }
#line 62
  if (n & (1 << 5)) {
#line 62
      HplMsp430InterruptP__Port15__fired();
#line 62
      return;
    }
#line 63
  if (n & (1 << 6)) {
#line 63
      HplMsp430InterruptP__Port16__fired();
#line 63
      return;
    }
#line 64
  if (n & (1 << 7)) {
#line 64
      HplMsp430InterruptP__Port17__fired();
#line 64
      return;
    }
}

#line 158
__attribute((wakeup)) __attribute((interrupt(2)))  void sig_PORT2_VECTOR(void )
{
  volatile int n = P2IFG & P2IE;

  if (n & (1 << 0)) {
#line 162
      HplMsp430InterruptP__Port20__fired();
#line 162
      return;
    }
#line 163
  if (n & (1 << 1)) {
#line 163
      HplMsp430InterruptP__Port21__fired();
#line 163
      return;
    }
#line 164
  if (n & (1 << 2)) {
#line 164
      HplMsp430InterruptP__Port22__fired();
#line 164
      return;
    }
#line 165
  if (n & (1 << 3)) {
#line 165
      HplMsp430InterruptP__Port23__fired();
#line 165
      return;
    }
#line 166
  if (n & (1 << 4)) {
#line 166
      HplMsp430InterruptP__Port24__fired();
#line 166
      return;
    }
#line 167
  if (n & (1 << 5)) {
#line 167
      HplMsp430InterruptP__Port25__fired();
#line 167
      return;
    }
#line 168
  if (n & (1 << 6)) {
#line 168
      HplMsp430InterruptP__Port26__fired();
#line 168
      return;
    }
#line 169
  if (n & (1 << 7)) {
#line 169
      HplMsp430InterruptP__Port27__fired();
#line 169
      return;
    }
}

# 96 "/home/tinyos/local/src/tinyos-2.x/tos/chips/msp430/usart/HplMsp430Usart0P.nc"
__attribute((wakeup)) __attribute((interrupt(18)))  void sig_UART0RX_VECTOR(void )
#line 96
{
  uint8_t temp = U0RXBUF;

#line 98
  HplMsp430Usart0P__Interrupts__rxDone(temp);
}

# 150 "/home/tinyos/local/src/tinyos-2.x/tos/system/ArbiterP.nc"
static bool /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0__ArbiterInfo__inUse(void )
#line 150
{
  /* atomic removed: atomic calls only */
#line 151
  {
    if (/*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0__state == /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0__RES_CONTROLLED) 
      {
        unsigned char __nesc_temp = 
#line 153
        FALSE;

#line 153
        return __nesc_temp;
      }
  }
#line 155
  return TRUE;
}






static uint8_t /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0__ArbiterInfo__userId(void )
#line 163
{
  /* atomic removed: atomic calls only */
#line 164
  {
    if (/*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0__state != /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0__RES_BUSY) 
      {
        unsigned char __nesc_temp = 
#line 166
        /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0__NO_RES;

#line 166
        return __nesc_temp;
      }
#line 167
    {
      unsigned char __nesc_temp = 
#line 167
      /*Msp430UsartShare0P.ArbiterC.Arbiter*/ArbiterP__0__resId;

#line 167
      return __nesc_temp;
    }
  }
}

# 101 "/home/tinyos/local/src/tinyos-2.x/tos/chips/msp430/usart/HplMsp430Usart0P.nc"
__attribute((wakeup)) __attribute((interrupt(16)))  void sig_UART0TX_VECTOR(void )
#line 101
{
  if (HplMsp430Usart0P__HplI2C__isI2C()) {
    HplMsp430Usart0P__I2CInterrupts__fired();
    }
  else {
#line 105
    HplMsp430Usart0P__Interrupts__txDone();
    }
}

# 72 "/home/tinyos/local/src/tinyos-2.x/tos/chips/msp430/dma/HplMsp430DmaP.nc"
__attribute((wakeup)) __attribute((interrupt(0)))  void sig_DACDMA_VECTOR(void )
#line 72
{
  HplMsp430DmaP__Interrupt__fired();
}

# 149 "/home/tinyos/local/src/tinyos-2.x/tos/lib/net/blip/shell/UDPShellP.nc"
static void UDPShellP__action_echo(int argc, char **argv)
#line 149
{
  int i;
#line 150
  int arg_len;
  char *payload = UDPShellP__reply_buf;

  if (argc < 2) {
#line 153
    return;
    }
#line 154
  for (i = 1; i < argc; i++) {
      arg_len = strlen(argv[i]);
      if (payload - UDPShellP__reply_buf + arg_len + 1 > MAX_REPLY_LEN) {
#line 156
        break;
        }
#line 157
      ip_memcpy(payload, argv[i], arg_len);
      payload += arg_len;
      *payload = ' ';
      payload++;
    }
  *(payload - 1) = '\n';

  UDPShellP__UDP__sendto(&UDPShellP__session_endpoint, UDPShellP__reply_buf, payload - UDPShellP__reply_buf);
}

static void UDPShellP__action_ping6(int argc, char **argv)
#line 167
{
  struct in6_addr dest;

  if (argc < 2) {
#line 170
    return;
    }
#line 171
  inet_pton6(argv[1], &dest);
  UDPShellP__ICMPPing__ping(&dest, 1024, 10);
}


static void UDPShellP__action_uptime(int argc, char **argv)
#line 176
{

  int len;
  uint64_t tval = UDPShellP__Uptime__get();

#line 180
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
    tval = (UDPShellP__uptime + tval - UDPShellP__boot_time) / 1024;
#line 181
    __nesc_atomic_end(__nesc_atomic); }
  len = snprintf(UDPShellP__reply_buf, MAX_REPLY_LEN, "up %li seconds\n", 
  (uint32_t )tval);
  UDPShellP__UDP__sendto(&UDPShellP__session_endpoint, UDPShellP__reply_buf, len);
}

