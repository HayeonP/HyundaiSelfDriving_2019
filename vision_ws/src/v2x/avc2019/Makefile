CFLAGS = -O -Wall -std=gnu11 -I../parser/include/j2735
LIBS = ../parser/src/libj2735.a

DEC = dec
UDP = udp

all: $(DEC) $(UDP)

%.o: %.c %.h
	$(CC) -c -o $@ $< $(CFLAGS)

$(DEC): dec.o
	$(CC) -o $@ $^ $(CFLAGS) $(LIBS)

$(UDP): udp.o
	$(CC) -o $@ $^ $(CFLAGS) -lpcap

clean:
	-$(RM) $(DEC) $(UDP) *.o
