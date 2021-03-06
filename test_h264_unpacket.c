#include "rtppacket.h"
#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/types.h>

// #define READ_FILE "./out.h264.rtp"
#define READ_FILE "./data/h264.rtp"
#define WRITE_FILE "./out.h264"

int32_t ReadFile(uint8_t* buff, int32_t buffSize)
{
    static FILE* fp = NULL;
    static uint8_t ssrc[4] = {0};
    static int32_t cacheIndex = 0;
    static uint8_t cache[1024 * 1024] = {0};

    int32_t ret = 0;
    int32_t offset = 0;
    int32_t count = 0;

    // file close & open
    if (!buff)
    {
        if (fp)
            fclose(fp);
        fp = NULL;
        return -1;
    }
    if (!fp)
        fp = fopen(READ_FILE, "r");
    if (!fp)
        return -1;
    
    // read
    ret = fread(&cache[cacheIndex], 1, sizeof(cache) - cacheIndex, fp);
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

int32_t MemFind(uint8_t* buff, int32_t buffSize)
{
    const int32_t tarSize = 4;
    const uint8_t tar[5] = {0, 0, 1, 0x2F};

    int32_t i = 0, j = 0;

    for (; i < buffSize && j < tarSize; i++)
    {
        if (buff[i] == tar[j])
            j++;
        else
            j = 0;
    }

    return j == tarSize ? (i - tarSize) : -1;
}

int main()
{
    int32_t index = 0;
    int32_t index2 = 0;

    int32_t frameSize = 0;
    uint8_t frame[1024*1024] = {0};

    int32_t rtpSize = 0;
    uint8_t rtp[1024*8] = {0};

    void* cache = RtpUnPacketCacheInit(512);

    do {
        rtpSize = ReadFile(rtp, sizeof(rtp));
        if (rtpSize > 14)
        {                
            memset(frame, 0, sizeof(frame));
        
            frameSize = RtpUnPacket(
                cache,
                rtp, rtpSize,
                frame, sizeof(frame),
                RTP_PAYLOAD_TYPE_H264, 0, 0);

            if (frameSize > 0)
                index2 += 1;

            printf("rtp: %02X %02X %02X %02X, "
                "ssrc: %02X%02X%02X%02X, "
                "tm: %02X%02X%02X%02X, "
                "payload: %02X %02X, size/%d, "
                "index/%d - "
                "pt/%d, %02X %02X %02X %02X %02X, "
                "frameSize/%d, index/%d\r\n",
                rtp[0], rtp[1], rtp[2], rtp[3],
                rtp[8], rtp[9], rtp[10], rtp[11],
                rtp[4], rtp[5], rtp[6], rtp[7],
                rtp[12], rtp[13],
                rtpSize, index++,
                rtp[1] & 0x7F, 
                frame[0], frame[1], frame[2], frame[3], frame[4],
                frameSize, index2);
            
            if (frameSize > 0)
                WriteFile(frame, frameSize);
        }
    } while (rtpSize >= 12);

    ReadFile(NULL, 0);
    WriteFile(NULL, 0);
    RtpUnPacketCacheRelease(cache);

    return 0;
}
