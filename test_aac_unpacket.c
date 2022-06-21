#include "rtppacket.h"
#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/types.h>

// #define READ_FILE "./out.aac_2x44100.rtp"
#define READ_FILE "./data/aac_2x44100.rtp"
#define WRITE_FILE "./out.2x44100.aac"

int32_t ReadFile(uint8_t* buff, int32_t buffSize)
{
    static int fd = 0;
    static uint8_t ssrc[4] = {0};
    static int32_t cacheIndex = 0;
    static uint8_t cache[1024 * 1024] = {0};

    int32_t ret = 0;
    int32_t offset = 0;
    int32_t count = 0;

    // file close & open
    if (!buff)
    {
        if (fd > 0)
            close(fd);
        fd = -1;
        return -1;
    }
    if (fd == 0)
        fd = open(READ_FILE, O_RDONLY);
    if (fd < 0)
        return -1;
    
    // read
    ret = read(fd, &cache[cacheIndex], sizeof(cache) - cacheIndex);
    if (ret > 0)
        cacheIndex += ret;

    // seek
    ret = RtpSeek(cache, cacheIndex, &offset, ssrc);

    // copy out
    if (ret > 0)
        memcpy(buff, &cache[offset], ret < buffSize ? ret : buffSize);
    
    // cache move
    if (ret > 0)
    {
        for (count = 0, offset += ret; offset < cacheIndex; count++, offset++)
            cache[count] = cache[offset];
        cacheIndex = count;
    }
    else
        cacheIndex = 0;

    return ret;
}

void WriteFile(uint8_t* buff, int32_t buffSize)
{
    static int fd = 0;

    // file close & open
    if (!buff)
    {
        if (fd > 0)
            close(fd);
        fd = -1;
        return;
    }
    if (fd == 0)
        fd = open(WRITE_FILE, O_RDWR | O_CREAT | O_TRUNC, 0666);
    if (fd < 0)
        return;
    
    write(fd, buff, buffSize);
    fsync(fd);
}

int main()
{
    int32_t index = 0;

    int32_t frameSize = 0;
    uint8_t frame[1024*1024] = {0};

    int32_t rtpSize = 0;
    uint8_t rtp[1024*8] = {0};

    void* cache = RtpUnPacketCacheInit(128);

    do {
        rtpSize = ReadFile(rtp, sizeof(rtp));
        if (rtpSize > 14)
        {
            memset(frame, 0, sizeof(frame));
        
            frameSize = RtpUnPacket(
                cache,
                rtp, rtpSize,
                frame, sizeof(frame),
                RTP_PAYLOAD_TYPE_AAC, 2, 44100);

            printf("rtp: %02X %02X %02X %02X, "
                "ssrc: %02X%02X%02X%02X, "
                "payload: %02X %02X %02X %02X, "
                "size/%d, index/%d - "
                "pt/%d, %02X %02X %02X %02X %02X %02X %02X, "
                "frameSize/%d \r\n",
                rtp[0], rtp[1], rtp[2], rtp[3],
                rtp[8], rtp[9], rtp[10], rtp[11],
                rtp[12], rtp[13], rtp[14], rtp[15],
                rtpSize, index++,
                rtp[1] & 0x7F, 
                frame[0], frame[1], frame[2], frame[3], frame[4], frame[5], frame[6],
                frameSize);

            if (frameSize > 0)
                WriteFile(frame, frameSize);
        }
    } while (rtpSize > 14);

    ReadFile(NULL, 0);
    WriteFile(NULL, 0);
    RtpUnPacketCacheRelease(cache);

    return 0;
}
