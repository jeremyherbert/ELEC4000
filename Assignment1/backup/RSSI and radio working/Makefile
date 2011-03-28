CFLAGS += -DCC2420_DEF_CHANNEL=17


#CFLAGS += -I$(TOSDIR)/lib/printf -DUSE_RF230_RADIO -DREPORT_DEST='"fec0::64"' -DBLIP_STATS
CFLAGS += -I$(TOSDIR)/lib/printf -DRF212_DEF_CHANNEL=12 -DRF230_DEF_CHANNEL=17 -DREPORT_DEST='"fec0::64"' -DBLIP_STATS






# -*- makefile -*-

SOURCES=$(LOWPAN_ROOT)/support/sdk/c/lib6lowpan/lib6lowpan.c  \
	$(LOWPAN_ROOT)/support/sdk/c/lib6lowpan/lib6lowpanIP.c \
	$(LOWPAN_ROOT)/support/sdk/c/lib6lowpan/lib6lowpanFrag.c \
	$(LOWPAN_ROOT)/support/sdk/c/lib6lowpan/in_cksum.c \
	$(LOWPAN_ROOT)/support/sdk/c/lib6lowpan/ip_malloc.c \
	$(LOWPAN_ROOT)/tos/lib/net/b6lowpan/table.c
OBJS=$(SOURCES:%.c=%.o)

remake: $(OBJS)
	make micaz sim-sf lowpan
	g++ -shared -fPIC  simbuild/micaz/pytossim.o simbuild/micaz/sim.o simbuild/micaz/tossim.o simbuild/micaz/c-support.o simbuild/micaz/c-sf.o simbuild/micaz/sf.o simbuild/micaz/throttle.o $(OBJS) -lstdc++  -lm -o _TOSSIMmodule.so
	make -f Makefile.local cppdriver

cppdriver:
	g++ -g -c -o sim/Driver.o sim/Driver.c -I$(TOSROOT)/tos/lib/tossim/
	g++ -o sim/Driver sim/Driver.o simbuild/micaz/tossim.o simbuild/micaz/sim.o simbuild/micaz/c-support.o simbuild/micaz/c-sf.o simbuild/micaz/sf.o simbuild/micaz/throttle.o $(OBJS)
# simbuild/micaz/c-sf.o       simbuild/micaz/pytossim.o  simbuild/micaz/sim.o       simbuild/micaz/tossim.o
# simbuild/micaz/c-support.o  simbuild/micaz/sf.o        simbuild/micaz/throttle.o



clean:
	rm $(OBJS)

%.o: %.c
	$(CC) -c -fPIC -o $@ $<

msg:	
	mig -DMIG -I$(LOWPAN_ROOT)/tos/lib/net/blip -I$(TOSROOT)/tos/chips/cc2420 -I$(LOWPAN_ROOT)/support/sdk/c/blip/include  python -python-classname=UdpReport UDPReport.h udp_report -o util/UdpReport.py












COMPONENT=RSSI_ForwarderC

include $(MAKERULES)
