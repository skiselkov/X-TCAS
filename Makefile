all : xtcas.xpl

OBJS=list.o helpers.o htbl.o geom.o math.o \
    SL.o

DEPS=$(patsubst %.o, %.d, $(OBJS))
CFLAGS+=-W -Wall -Werror -O0 -g

# Silence GCC warnings about our CTASSERT
ifeq ($(findstring gcc, $(COMPILE.c)), gcc)
	CFLAGS += -Wno-unused-local-typedefs
endif

ifeq ($(debug),no)
	CFLAGS += -O3
else
	CFLAGS += -DDEBUG
endif

xtcas.xpl : $(OBJS)
	$(LINK.c) -shared -o $@ $(OBJS)

%.o : %.c
	$(COMPILE.c) -MMD -o $@ $<

clean :
	rm -f xtcas.xpl $(OBJS) $(DEPS)

-include $(DEPS)
