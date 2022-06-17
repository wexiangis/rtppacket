#include "rtppacket.h"
#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/types.h>

#define READ_FILE "./data/2x44100.aac"
#define WRITE_FILE "./out.aac_2x44100.rtp"

int32_t ReadFile(uint8_t* buff, int32_t buffSize, int32_t* chn, int32_t* freq, uint32_t* duration)
{
    static int fd = 0;
    static int32_t cacheIndex = 0;
    static uint8_t cache[1024 * 1024] = {0};

    int32_t ret = 0;
    int32_t count = 0;
    int32_t offset = -1;

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
    ret = AACSeek(cache, cacheIndex, &offset, chn, freq, duration);

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

void RtpCallback(void* priv, uint8_t* rtp, int32_t rtpSize, RTP_PAYLOAD_TYPE type)
{
    static int32_t index = 0;
    printf("rtp: %02X %02X %02X %02X, "
        "timestamp: %02X%02X%02X%02X, "
        "ssrc: %02X%02X%02X%02X, "
        "payload: %02X %02X %02X %02X %02X %02X, size/%d, index/%d\r\n",
        rtp[0], rtp[1], rtp[2], rtp[3],
        rtp[4], rtp[5], rtp[6], rtp[7],
        rtp[8], rtp[9], rtp[10], rtp[11],
        rtp[12], rtp[13], rtp[14], rtp[15], rtp[16], rtp[17],
        rtpSize, index++);
    WriteFile(rtp, rtpSize);
}

int main()
{
    int32_t index = 0;
    int32_t chn = 0;
    int32_t freq = 0;

    int32_t frameSize = 0;
    uint8_t frame[1024*1024] = {0};
    int32_t pktNum = 0;

    uint16_t seq = 0;
    uint32_t tm = 0;
    uint32_t duration = 0;
    uint32_t ssrc = 0x12345678;
    RTP_PAYLOAD_TYPE type = RTP_PAYLOAD_TYPE_AAC;

    do {
        frameSize = ReadFile(frame, sizeof(frame), &chn, &freq, &duration);
        // frameSize += ReadFile(frame + frameSize, sizeof(frame) - frameSize, &chn, &freq, &duration);
        // frameSize += ReadFile(frame + frameSize, sizeof(frame) - frameSize, &chn, &freq, &duration);

        if (frameSize > 7)
        {
            pktNum = RtpPacket(
                NULL,
                frame, frameSize,
                &seq, &tm, duration,
                ssrc, type,
                NULL, &RtpCallback);

            printf("frame: %02X %02X %02X %02X %02X, data: %02X %02X, "
                "size/%d, chn/%d, freq/%dHz, index/%d - pktNum/%d - %02d:%02d \r\n",
                frame[0], frame[1], frame[2], frame[3], frame[4],
                frame[7], frame[8],
                frameSize, chn, freq, index++,
                pktNum,
                (index * duration) / 1000000 / 60,
                (index * duration) / 1000000 % 60);
            
            usleep(duration);
        }
    } while (frameSize > 0);

    ReadFile(NULL, 0, NULL, NULL, NULL);
    WriteFile(NULL, 0);

    return 0;
}
