#include "rtppacket.h"
#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/types.h>

#define READ_FILE "./data/1x8000.pcm"
#define WRITE_FILE "./out.pcm_1x8000.rtp"

int32_t ReadFile(uint8_t* buff, int32_t buffSize)
{
    static int fd = 0;

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
    return read(fd, buff, 160);
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
        "payload: %02X %02X, size/%d, index/%d\r\n",
        rtp[0], rtp[1], rtp[2], rtp[3],
        rtp[4], rtp[5], rtp[6], rtp[7],
        rtp[8], rtp[9], rtp[10], rtp[11],
        rtp[12], rtp[13],
        rtpSize, index++);
}

int main()
{
    int32_t index = 0;

    int32_t frameSize = 0;
    uint8_t frame[1024*1024] = {0};
    int32_t pktNum = 0;

    uint16_t seq = 0;
    uint32_t tm = 0;
    uint32_t duration = 0;
    uint32_t ssrc = 0x12345678;
    RTP_PAYLOAD_TYPE type = RTP_PAYLOAD_TYPE_PCMA;

    do {
        frameSize = ReadFile(frame, sizeof(frame));
        // frameSize += ReadFile(frame + frameSize, sizeof(frame) - frameSize);
        // frameSize += ReadFile(frame + frameSize, sizeof(frame) - frameSize);

        duration = frameSize * 1000000 / (8000 * 2);

        if (frameSize > 4)
        {
            *((uint16_t*)frame) = (uint16_t)index;

            pktNum = RtpPacket(
                frame, frameSize,
                &seq, &tm, duration,
                ssrc, type,
                NULL, &RtpCallback);

            printf("frame: %02X %02X %02X %02X %02X, "
                "size/%d, index/%d - pktNum/%d - %02d:%02d \r\n",
                frame[0], frame[1], frame[2], frame[3], frame[4],
                frameSize, index++,
                pktNum,
                (index * duration) / 1000000 / 60,
                (index * duration) / 1000000 % 60);
            
            usleep(duration);
        }
    } while (frameSize > 0);

    ReadFile(NULL, 0);
    WriteFile(NULL, 0);

    return 0;
}
