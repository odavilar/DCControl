objectscon := 2Vel.o VelControl.o
objectsmov := move.o

objects := $(objectscon:.o=) $(objectsmov:.o=)

IXPIO := -I/home/odavilar/workspace/ixpio/include/

DC_CPPFLAGS := ${IXPIO} -I/lib/modules/`uname -r`/build/include/xenomai/ -I/lib/modules/`uname -r`/build/include/xenomai/posix -I/usr/xenomai/include -D_GNU_SOURCE -D_REENTRANT -D__XENO__
DC_LDFLAGS := -lnative -L/usr/xenomai/lib -lxenomai -lpthread -lrt

CC := gcc

DEBUGFLAGS := -ggdb
NCURSES := -lncurses

all: mov control

mov: $(objectsmov)

control: $(objectscon)

2Vel.o: 
	$(CC) 2Vel.c ${DC_CPPFLAGS} ${DC_LDFLAGS} ${DEBUGFLAGS} -o 2Vel

VelControl.o:
	$(CC) VelControl.c ${DC_CPPFLAGS} ${DC_LDFLAGS} ${DEBUGFLAGS} -o VelControl

move.o:
	$(CC) move.c ${DEBUGFLAGS} ${IXPIO} ${NCURSES} -o move


clean:
	rm $(objects)
