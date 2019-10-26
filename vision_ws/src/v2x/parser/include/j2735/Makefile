include Makefile.am.libasncodec

LIBS += -lm
CFLAGS += $(ASN_MODULE_CFLAGS) -DPDU=MessageFrame -I. -O -Wall -std=gnu11
# CFLAGS += -DASN_EMIT_DEBUG=1
CFLAGS += -Wno-missing-braces -Wno-unused-const-variable -Wno-parentheses-equality
ASN_LIBRARY = libj2735.a

all: $(ASN_LIBRARY)

$(ASN_LIBRARY): $(ASN_MODULE_SRCS:.c=.o)
	$(AR) rcs $@ $(ASN_MODULE_SRCS:.c=.o)

.SUFFIXES:
.SUFFIXES: .c .o

.c.o:
	$(CC) $(CFLAGS) -o $@ -c $<

clean:
	rm -f $(ASN_MODULE_SRCS:.c=.o) $(ASN_PROGRAM_SRCS:.c=.o) $(ASN_LIBRARY)

regen: regenerate-from-asn1-source

regenerate-from-asn1-source:
	rm -f *.c *.h
	asn1c -no-gen-example -fcompound-names -funnamed-unions -pdu=MessageFrame ../J2735_201603DA+ITSK4-0.4_fix.asn

