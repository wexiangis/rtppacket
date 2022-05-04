
GCC = g++

CFLAGS += -Wall
# CFLAGS += -g
CFLAGS += -fsanitize=address

all: h264_packet h264_unpacket aac_packet aac_unpacket pcm_packet

h264_packet:
	@$(GCC) -o out.$@ test_$@.c rtppacket.c $(CFLAGS)
h264_unpacket:
	@$(GCC) -o out.$@ test_$@.c rtppacket.c $(CFLAGS)

aac_packet:
	@$(GCC) -o out.$@ test_$@.c rtppacket.c $(CFLAGS)
aac_unpacket:
	@$(GCC) -o out.$@ test_$@.c rtppacket.c $(CFLAGS)

pcm_packet:
	@$(GCC) -o out.$@ test_$@.c rtppacket.c $(CFLAGS)

clean:
	@rm -rf out*
