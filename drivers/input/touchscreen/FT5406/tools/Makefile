CROSS_COMPILE = arm-linux-gnueabihf-

ifdef CROSS_COMPILE
CC = $(CROSS_COMPILE)gcc
SYSROOT=/opt/arm-dev/armhf
CFLAGS += -march=armv7-a -mfpu=neon -mhard-float -mfloat-abi=hard
STRIP = ${CROSS_COMPILE}strip
STRIP_OPT = --remove-section=.comment --remove-section=.note
else
CC = gcc
STRIP = strip
STRIP_OPT =
endif
ifdef SYSROOT
CFLAGS += --sysroot=${SYSROOT}
LDFLAGS += --sysroot=${SYSROOT}
endif
LDFLAGS += -static

PROGRAM = ftsfwloader
SRCS = fwloader.c hexdec.c
OBJS    = ${SRCS:%.c=%.o}

all: $(PROGRAM)

$(PROGRAM): $(OBJS) depend.inc
	$(CC) -o $@ $(OBJS) $(LDFLAGS)
	$(STRIP) $(STRIP_OPT) $@

%.o: %.c
	$(CC) -c $(CFLAGS) -o $@ $<

clean:
	-rm -rf $(OBJS) $(PROGRAM) depend.inc

depend.inc: $(SRCS)
	$(CC) -MM $(CFLAGS) $^ > depend.inc

-include depend.inc
