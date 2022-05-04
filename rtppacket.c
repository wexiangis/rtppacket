#include "rtppacket.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define RTP_INFO(...) fprintf(stdout, "[RTP_INFO] %s(%d): ", __FUNCTION__, __LINE__),fprintf(stdout, __VA_ARGS__)

/* ========================= param define ========================= */

#define RTP_PACKET_PAYLOAD_SIZE 1500 // 收rtp时payload预留大小
#define RTP_PACKET_PAYLOAD_DATA_SIZE 1400 // 发rtp包时最大payload大小

#define RTP_PACKET_PAYLOAD_PCM_SZIE 160 // g711a & g711u

#define RTP_CIRCLE_FRAME_TIMEOUT_DIV 32 // 单帧等待时长不超过1/32循环缓冲区大小

// payload[0] & 0x1F
typedef enum {
    H26X_FRAME_NAL_MIN = 1,
    H26X_FRAME_PA = 2,
    H26X_FRAME_PB = 3,
    H26X_FRAME_PC = 4,
    H26X_FRAME_IDR = 5,
    H26X_FRAME_SEI = 6,
    H26X_FRAME_SPS = 7,
    H26X_FRAME_PPS = 8,
    H26X_FRAME_NAL_MAX = 23, // 0x17

    H26X_FRAME_STAP_A = 24, // 0x18
    H26X_FRAME_STAP_B = 25, // 0x19
    H26X_FRAME_MTAP16 = 26, // 0x1A
    H26X_FRAME_MTAP32 = 27, // 0x1B
    H26X_FRAME_FU_A = 28, // 0x1C
    H26X_FRAME_FU_B = 29, // 0x1D
} H26X_FRAME_TYPE;

// payload[1] & 0xE0
typedef enum {
    H26X_FU_A_BEGIN = 0x80,
    H26X_FU_A_MIDDLE = 0x00,
    H26X_FU_A_END = 0x40,
} H26X_FU_A_TYPE;

typedef enum {
    RTP_CIRCLE_FRAME_NONE = 0,
    RTP_CIRCLE_FRAME_SINGLE = 1,
    RTP_CIRCLE_FRAME_BEGIN = 2,
    RTP_CIRCLE_FRAME_MID = 3,
    RTP_CIRCLE_FRAME_END = 4,
} RTP_CIRCLE_FRAME_TYPE;

/* ========================= struct define ========================= */

/*
 *
 *    0                   1                   2                   3
 *    7 6 5 4 3 2 1 0|7 6 5 4 3 2 1 0|7 6 5 4 3 2 1 0|7 6 5 4 3 2 1 0
 *   +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
 *   |V=2|P|X|  CC   |M|     PT      |       sequence number         |
 *   +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
 *   |                           timestamp                           |
 *   +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
 *   |           synchronization source (SSRC) identifier            |
 *   +=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+
 *   |            contributing source (CSRC) identifiers             |
 *   :                             ....                              :
 *   +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
 *
 */
typedef struct
{
    /* 注意下面每字节中的参数低位bit排在前面 */

    /* byte 0 */
    uint8_t cc : 4; //csrc标识符个数,0~15
    uint8_t x : 1;  //扩展标志,为1时表示rtp报头后有1个扩展包
    uint8_t p : 1;  //填充标志,为1时标识数据尾部有无效填充
    uint8_t v : 2;  //版本号

    /* byte 1 */
    uint8_t pt : 7; //有小载荷类型
    uint8_t m : 1;  //载荷标记,视频为1帧结束,音频为会话开始

    /* bytes 2,3 */
    uint8_t seq[2]; //序列号,每帧+1,随机开始(注意高字节在前)

    /* bytes 4-7 */
    uint8_t tm[4]; //时间戳timestamp,us,自增(注意高字节在前)

    /* bytes 8-11 */
    uint8_t ssrc[4]; //同步信号源,信号来源唯一标识

    /* 由前面的cc决定这里长度 */
    // uint32_t csrc[cc]; //由混合器记录的各路参与者的ssrc的列表
} RtpHeader;

typedef struct
{
    RtpHeader header;
    uint8_t payload[RTP_PACKET_PAYLOAD_SIZE];
} RtpStruct;


//aac的头字段,共7字节,注意每字节中低位数据放在在前面(实际bit流高位在先)
typedef struct
{
    /* 注意下面每字节中的参数低位bit排在前面 */

    //byte 0
    uint8_t syncwordH : 8; //8 bit 同步字高位 0xFF,说明一个ADTS帧的开始
    //byte 1
    uint8_t protectionAbsent : 1; //1 bit 1表示没有校验,0表示有校验(header后面加2字节crc32)
    uint8_t layer : 2;            //2 bit 总是'00'
    uint8_t id : 1;               //1 bit MPEG 标示符, 0/MPEG-4 1/MPEG-2
    uint8_t syncwordL : 4;        //4 bit 同步字低位 0xF,说明一个ADTS帧的开始
    //byte 2
    uint8_t chnH : 1;              //1 bit 表示声道数(高位)
    uint8_t privateBit : 1;        //1 bit
    uint8_t samplingFreqIndex : 4; //4 bit 表示使用的采样频率
    uint8_t profile : 2;           //2 bit 表示使用哪个级别的AAC 0/main 1/LC 2/SSR
    //byte 3
    uint8_t aacFrameLengthH : 2;              //2 bit 帧长度高位(头+数据段的总长度)
    uint8_t copyrightIdentificationStart : 1; //1 bit
    uint8_t copyrightIdentificationBit : 1;   //1 bit
    uint8_t home : 1;                         //1 bit
    uint8_t originalCopy : 1;                 //1 bit
    uint8_t chnL : 2;                         //2 bit 表示声道数(低位)
    //byte 4
    uint8_t aacFrameLengthM : 8; //8 bit 帧长度中位(头+数据段的总长度)
    //byte 5
    uint8_t adtsBufferFullnessH : 5; //5 bit 0x7 说明是码率可变的码流高位
    uint8_t aacFrameLengthL : 3;     //3 bit 帧长度低位(头+数据段的总长度)
    //byte 6
    uint8_t numberOfRawDataBlockInFrame : 2; //2 bits 表示ADTS帧中有 该值+1个AAC原始帧,一般为0
    uint8_t adtsBufferFullnessL : 6;         //6 bit 0xFF 说明是码率可变的码流低位
} AACHeader;


typedef struct {
    RTP_CIRCLE_FRAME_TYPE type;
    uint32_t frameSize;
    uint8_t frame[RTP_PACKET_PAYLOAD_SIZE];
} RtpCirclePacket;

typedef struct {
    int32_t count; // 目前缓冲区中的元素个数
    int32_t seqMax; // 目前最大的序号
    int32_t seqMin; // 目前最小的序号
    int32_t seqWait; // 当前在等待的帧序号
    int32_t frameCount;
    int32_t frameTimeout;
    int32_t packetCount; // 缓存packet个数
    RtpCirclePacket* packets; // packet数组
} RtpCircleCache;


static const uint8_t gH264Header[4] = {0, 0, 0, 1};

static const int32_t gAACFreqList[15] = {
    96000, 88200, 64000, 48000, 44100, 32000, 24000,
    22050, 16000, 12000, 11025, 8000, 7350, 0, 0};

/* ========================= header packet/unpacket (static) ========================= */

static void RtpHeaderPacket(
    RtpHeader* rtp, RTP_PAYLOAD_TYPE type,
    uint16_t seq, uint32_t tm, uint32_t ssrc,
    uint8_t isEOF)
{
    rtp->cc = 0;
    rtp->x = 0;
    rtp->p = 0;
    rtp->v = 2;
    
    rtp->pt = type;
    rtp->m = isEOF;

    // 注意这里用的大端存储模式
    rtp->seq[0] = (uint8_t)((seq >> 8) & 0xFF);
    rtp->seq[1] = (uint8_t)(seq & 0xFF);

    rtp->tm[0] = (uint8_t)((tm >> 24) & 0xFF);
    rtp->tm[1] = (uint8_t)((tm >> 16) & 0xFF);
    rtp->tm[2] = (uint8_t)((tm >> 8) & 0xFF);
    rtp->tm[3] = (uint8_t)(tm & 0xFF);

    rtp->ssrc[0] = (uint8_t)((ssrc >> 24) & 0xFF);
    rtp->ssrc[1] = (uint8_t)((ssrc >> 16) & 0xFF);
    rtp->ssrc[2] = (uint8_t)((ssrc >> 8) & 0xFF);
    rtp->ssrc[3] = (uint8_t)(ssrc & 0xFF);
}

static int32_t AACHeaderPacket(
    AACHeader* header, uint8_t chn, uint16_t freq,
    uint16_t codeRate, uint16_t dataLen)
{
    dataLen += 7;
    //byte 1
    header->syncwordH = 0xFF;
    //byte 2
    header->syncwordL = 0xF;
    header->id = 0;
    header->layer = 0;
    header->protectionAbsent = 1;
    //byte 3
    header->profile = 1;
    if (freq <= gAACFreqList[12])
        header->samplingFreqIndex = 12;
    else if (freq <= gAACFreqList[11])
        header->samplingFreqIndex = 11;
    else if (freq <= gAACFreqList[10])
        header->samplingFreqIndex = 10;
    else if (freq <= gAACFreqList[9])
        header->samplingFreqIndex = 9;
    else if (freq <= gAACFreqList[8])
        header->samplingFreqIndex = 8;
    else if (freq <= gAACFreqList[7])
        header->samplingFreqIndex = 7;
    else if (freq <= gAACFreqList[6])
        header->samplingFreqIndex = 6;
    else if (freq <= gAACFreqList[5])
        header->samplingFreqIndex = 5;
    else if (freq <= gAACFreqList[4])
        header->samplingFreqIndex = 4;
    else if (freq <= gAACFreqList[3])
        header->samplingFreqIndex = 3;
    else if (freq <= gAACFreqList[2])
        header->samplingFreqIndex = 2;
    else if (freq <= gAACFreqList[1])
        header->samplingFreqIndex = 1;
    else
        header->samplingFreqIndex = 0;
    header->privateBit = 0;
    header->chnH = (chn >> 2) & 0x1;
    //byte 4
    header->chnL = chn & 0x3;
    header->originalCopy = 0;
    header->home = 0;
    header->copyrightIdentificationBit = 0;
    header->copyrightIdentificationStart = 0;
    header->aacFrameLengthH = (dataLen >> 11) & 0x3;
    //byte 5
    header->aacFrameLengthM = (dataLen >> 3) & 0xFF;
    //byte 6
    header->aacFrameLengthL = dataLen & 0x7;
    header->adtsBufferFullnessH = (codeRate >> 6) & 0x1F;
    //byte 7
    header->adtsBufferFullnessL = codeRate & 0x3F;
    header->numberOfRawDataBlockInFrame = 0;
    return dataLen;
}

static int32_t AACHeaderUnPacket(
    AACHeader* header, uint8_t* chn, uint16_t* freq, uint16_t* frameLen, uint16_t* pcmLen)
{
    if (header->syncwordH != 0xFF ||
        header->syncwordL != 0x0F ||
        header->samplingFreqIndex > 12)
        return -1;
    if (chn) {
        *chn = (header->chnH << 3) | header->chnL;
    }
    if (freq) {
        *freq = gAACFreqList[header->samplingFreqIndex];
    }
    if (frameLen) {
        *frameLen = (header->aacFrameLengthH << 11) |
                (header->aacFrameLengthM << 3) |
                header->aacFrameLengthL;
        if (*frameLen <= sizeof(AACHeader)) {
            return -2;
        }
    }
    if (pcmLen) {
        *pcmLen = (((header->adtsBufferFullnessL) & 0x3F) |
            ((uint16_t)header->adtsBufferFullnessH << 6)) + 1;
    }
#if 0
    RTP_INFO("adts:id  %d\n", header->id);
    RTP_INFO("adts:layer  %d\n", header->layer);
    RTP_INFO("adts:protection_absent  %d\n", header->protectionAbsent);
    RTP_INFO("adts:profile  %d\n", header->profile);
    RTP_INFO("adts:sf_index  %dHz\n", gAACFreqList[header->samplingFreqIndex]);
    RTP_INFO("adts:pritvate_bit  %d\n", header->privateBit);
    RTP_INFO("adts:channel_configuration  %d\n", (header->chnH << 3) | header->chnL);
    RTP_INFO("adts:original  %d\n", header->originalCopy);
    RTP_INFO("adts:home  %d\n", header->home);
    RTP_INFO("adts:copyright_identification_bit  %d\n", header->copyrightIdentificationBit);
    RTP_INFO("adts:copyright_identification_start  %d\n", header->copyrightIdentificationStart);
    RTP_INFO("adts:aac_frame_length  %d\n", (header->aacFrameLengthH << 11) | (header->aacFrameLengthM << 3) | header->aacFrameLengthL);
    RTP_INFO("adts:adts_buffer_fullness  %d\n", (((uint16_t)header->adtsBufferFullnessH << 6) | header->adtsBufferFullnessL) + 1);
    RTP_INFO("adts:no_raw_data_blocks_in_frame  %d\n", header->numberOfRawDataBlockInFrame);
    RTP_INFO("\n");
#endif
    return 0;
}

/* ========================= api seek ========================= */

int32_t RtpSeek(
    uint8_t* buff, int32_t buffSize, int32_t* offset,
    uint8_t ssrc[4])
{
    static uint8_t rtpFirstByte = 0;
    uint32_t u32Ssrc = *((uint32_t*)ssrc);

    int32_t rtpBegin = 0;
    int32_t rtpEnd = 0;

    *offset = 0;
    if (!buff || buffSize < 14)
        return -1;

    if (!u32Ssrc)
    {
        rtpFirstByte = buff[0];
        ssrc[0] = buff[8];
        ssrc[1] = buff[9];
        ssrc[2] = buff[10];
        ssrc[3] = buff[11];
    }

    for (; rtpBegin < buffSize - 12; rtpBegin++)
    {
        if (buff[rtpBegin] == rtpFirstByte &&
            buff[rtpBegin + 8] == ssrc[0] &&
            buff[rtpBegin + 9] == ssrc[1] &&
            buff[rtpBegin + 10] == ssrc[2] &&
            buff[rtpBegin + 11] == ssrc[3])
        {
            rtpEnd = rtpBegin + 12;
            *offset = rtpBegin;
            break;
        }
    }

    for (; rtpEnd < buffSize - 12; rtpEnd++)
    {
        if (buff[rtpEnd] == rtpFirstByte &&
            buff[rtpEnd + 8] == ssrc[0] &&
            buff[rtpEnd + 9] == ssrc[1] &&
            buff[rtpEnd + 10] == ssrc[2] &&
            buff[rtpEnd + 11] == ssrc[3])
        {
            return rtpEnd - rtpBegin;
        }
    }

    return -1;
}

int32_t H26XSeek(
    uint8_t* buff, int32_t buffSize, int32_t* offset,
    int32_t* width, int32_t* height, int32_t* fps)
{
    int32_t rtpBegin = 0;
    int32_t rtpEnd = 0;

    *offset = 0;
    if (!buff || buffSize < 6)
        return -1;

    for (; rtpBegin < buffSize - 4; rtpBegin++)
    {
        if (buff[rtpBegin] == 0 &&
            buff[rtpBegin + 1] == 0 &&
            buff[rtpBegin + 2] == 1)
        {
            rtpEnd = rtpBegin + 3;
            *offset = rtpBegin;
            break;
        }
        else if (buff[rtpBegin] == 0 &&
            buff[rtpBegin + 1] == 0 &&
            buff[rtpBegin + 2] == 0 &&
            buff[rtpBegin + 3] == 1)
        {
            rtpEnd = rtpBegin + 4;
            *offset = rtpBegin;
            break;
        }
    }

    for (; rtpEnd < buffSize - 4; rtpEnd++)
    {
        if (buff[rtpEnd] == 0 &&
            buff[rtpEnd + 1] == 0 &&
            buff[rtpEnd + 2] == 1)
        {
            return rtpEnd - rtpBegin;
        }
        else if (buff[rtpEnd] == 0 &&
            buff[rtpEnd + 1] == 0 &&
            buff[rtpEnd + 2] == 0 &&
            buff[rtpEnd + 3] == 1)
        {
            return rtpEnd - rtpBegin;
        }
    }

    return rtpEnd > 0 ? buffSize : (-1);
}

int32_t AACSeek(
    uint8_t* buff, int32_t buffSize, int32_t* offset,
    int32_t* chn, int32_t* freq, uint32_t* duration)
{
    uint8_t _chn = 0;
    uint16_t _freq = 0;
    uint16_t frameLen = 0;
    uint16_t pcmLen = 0;

    int32_t rtpBegin = 0;
    int32_t rtpEnd = 0;

    *offset = 0;
    if (!buff || buffSize < 8)
        return -1;

    for (; rtpBegin < buffSize - 7; rtpBegin++)
    {
        if (buff[rtpBegin] == 0xFF &&
            AACHeaderUnPacket((AACHeader*)&buff[rtpBegin], &_chn, &_freq, &frameLen, &pcmLen) == 0 &&
            _chn > 0)
        {
            rtpEnd = rtpBegin + 7;
            *offset = rtpBegin;

            if (chn)
                *chn = _chn;
            if (freq)
                *freq = _freq;
            if (duration)
                *duration = (uint32_t)pcmLen * 1000000 / (_freq * _chn);
            break;
        }
    }

    for (; rtpEnd < buffSize - 7; rtpEnd++)
    {
        if (buff[rtpEnd] == 0xFF &&
            AACHeaderUnPacket((AACHeader*)&buff[rtpEnd], &_chn, &_freq, NULL, NULL) == 0 &&
            _chn > 0)
        {
            break;
        }
    }

    return frameLen;
}

/* ========================= api circle cache (static) ========================= */

void* RtpUnPacketCacheInit(int32_t packetCount)
{
    RtpCircleCache* ret;

    if (packetCount < 0)
        return NULL;

    ret = (RtpCircleCache*)calloc(1, sizeof(RtpCircleCache));
    ret->frameTimeout = packetCount / RTP_CIRCLE_FRAME_TIMEOUT_DIV;
    ret->packetCount = packetCount;
    ret->packets = (RtpCirclePacket*)calloc(packetCount, sizeof(RtpCirclePacket));
    return ret;
}

void RtpUnPacketCacheRelease(void** cache)
{
    RtpCircleCache* rcc;
    if (cache)
    {
        if (*cache)
        {
            rcc = (RtpCircleCache*)(*cache);
            if (rcc->packets)
                free(rcc->packets);
            free(rcc);
        }
        *cache = NULL;
    }
}

static void RtpCircleCacheIn(
    uint8_t* frame, int32_t frameSize,
    RTP_CIRCLE_FRAME_TYPE type,
    uint16_t seq, RtpCircleCache* rcc)
{
    RtpCirclePacket* packet = &rcc->packets[seq % rcc->packetCount];

    // Test: lost frame
    // if (seq >= 0x2BEE && seq <= 0x2BF1)
    // if (seq % 63 == 0 || seq % 62 == 0)
    // {
    //     RTP_INFO("lost: %02X %02X %02X %02X %02X, size/%d, seq/%04X, type/%d\r\n",
    //         frame[0], frame[1], frame[2], frame[3], frame[4], frameSize, seq, type);
    //     return;
    // }

    // copy
    if (frameSize > RTP_PACKET_PAYLOAD_SIZE)
    {
        RTP_INFO("error frameSize(%d) > payloadSize(%d) \r\n",
            frameSize, RTP_PACKET_PAYLOAD_SIZE);
        memcpy(packet->frame, frame, RTP_PACKET_PAYLOAD_SIZE);
    }
    else
    {
        memcpy(packet->frame, frame, frameSize);
    }
    packet->type = type;
    packet->frameSize = frameSize;

    // refresh seqMax
    rcc->seqMax = (seq + 1) % rcc->packetCount;

    // count++
    rcc->count += 1;
}

static int32_t RtpCircleCacheOut(
    uint8_t* frame, int32_t frameSize,
    RtpCircleCache* rcc)
{
    int32_t retSize = 0;

    int32_t pktIndex = 0;
    int32_t seqIndex = 0;
    int32_t seqBegin = 0; // 分包起始位置缓存

    RTP_CIRCLE_FRAME_TYPE hitType = RTP_CIRCLE_FRAME_NONE;

    // 遍历从min到max
    for (seqIndex = rcc->seqMin;
        seqIndex != rcc->seqMax && pktIndex < rcc->packetCount;
        seqIndex = (seqIndex + 1) % rcc->packetCount, pktIndex++)
    {
        // 查找独立帧或者分包起始
        if (hitType == RTP_CIRCLE_FRAME_NONE)
        {
            if (rcc->packets[seqIndex].frameSize > 0)
            {
                if (rcc->packets[seqIndex].type == RTP_CIRCLE_FRAME_SINGLE)
                {
                    hitType = RTP_CIRCLE_FRAME_SINGLE;
                    break;
                }
                else if (rcc->packets[seqIndex].type == RTP_CIRCLE_FRAME_BEGIN)
                {
                    seqBegin = seqIndex;
                    hitType = RTP_CIRCLE_FRAME_BEGIN;
                }
                else if (rcc->frameCount++ < rcc->frameTimeout)
                {
                    // RTP_INFO("not found SINGLE or BEGIN frame \r\n");
                    break;
                }
            }
        }
        // 查找分包的结束包
        else if (hitType == RTP_CIRCLE_FRAME_BEGIN)
        {
            if (rcc->packets[seqIndex].frameSize > 0)
            {
                if (rcc->packets[seqIndex].type == RTP_CIRCLE_FRAME_END)
                {
                    hitType = RTP_CIRCLE_FRAME_END;
                    break;
                }
            }
            else if (rcc->frameCount++ < rcc->frameTimeout)
            {
                // RTP_INFO("not found MID or END frame \r\n");
                break;
            }
        }
    }

    // 这是独立包
    if (hitType == RTP_CIRCLE_FRAME_SINGLE)
    {
        // copy out
        memcpy(frame, rcc->packets[seqIndex].frame, rcc->packets[seqIndex].frameSize);
        retSize = rcc->packets[seqIndex].frameSize;
        // clean
        rcc->packets[seqIndex].frameSize = 0;
        rcc->packets[seqIndex].type = RTP_CIRCLE_FRAME_NONE;
    }
    // 这是分包,组合分包
    else if (hitType == RTP_CIRCLE_FRAME_END)
    {
        do
        {
            if (rcc->packets[seqBegin].type >= RTP_CIRCLE_FRAME_BEGIN &&
                rcc->packets[seqBegin].type <= RTP_CIRCLE_FRAME_END)
            {
                if ((uint32_t)frameSize < retSize + rcc->packets[seqBegin].frameSize)
                {
                    RTP_INFO("error not enough frameSize(%d) to copy data(%d) out \r\n",
                        frameSize, retSize + rcc->packets[seqBegin].frameSize);
                }
                else
                {
                    // copy out
                    memcpy(frame, rcc->packets[seqBegin].frame, rcc->packets[seqBegin].frameSize);
                    frame += rcc->packets[seqBegin].frameSize;
                    retSize += rcc->packets[seqBegin].frameSize;
                }
                // clean
                rcc->packets[seqBegin].frameSize = 0;
                rcc->packets[seqBegin].type = RTP_CIRCLE_FRAME_NONE;
            }

            if (seqBegin == seqIndex)
                break;
            
            seqBegin = (seqBegin + 1) % rcc->packetCount;
        } while (1);
    }

    // update flags
    if (retSize > 0)
    {
        rcc->seqMin = (seqIndex + 1) % rcc->packetCount;
        rcc->count -= 1;
        rcc->frameCount = 0;
    }

    if (rcc->seqWait != seqIndex)
    {
        rcc->seqWait = seqIndex;
        rcc->frameCount = 0;
    }

    return retSize;
}

/* ========================= api xxx packet (static) ========================= */

static int32_t RtpPCMPacket(
    uint8_t* frame, int32_t frameSize,
    uint16_t* seq, uint32_t* tm, uint32_t duration,
    uint32_t ssrc, RTP_PAYLOAD_TYPE type,
    void* priv,
    void(*callback)(void* priv, uint8_t* rtp, int32_t rtpSize, RTP_PAYLOAD_TYPE type))
{
    RtpStruct rtp = {0};
    int32_t rtpSize = sizeof(RtpHeader) + RTP_PACKET_PAYLOAD_PCM_SZIE;
    int32_t retPkt = 0;

    if (frameSize > RTP_PACKET_PAYLOAD_PCM_SZIE)
        duration /= frameSize / RTP_PACKET_PAYLOAD_PCM_SZIE;

    while (frameSize > 0)
    {
        RtpHeaderPacket(&rtp.header, type, *seq, *tm, ssrc, 1);
        *seq += 1;
        *tm += duration;

        if (callback)
        {
            memcpy(rtp.payload, frame,
                frameSize > RTP_PACKET_PAYLOAD_PCM_SZIE ?
                RTP_PACKET_PAYLOAD_PCM_SZIE : frameSize);
            
            callback(priv, (uint8_t*)&rtp, rtpSize, type);
        }

        frame += RTP_PACKET_PAYLOAD_PCM_SZIE;
        frameSize -= RTP_PACKET_PAYLOAD_PCM_SZIE;
        retPkt += 1;
    }

    return retPkt;
}

static int32_t RtpAACPacket(
    uint8_t* frame, int32_t frameSize,
    uint16_t* seq, uint32_t* tm,
    uint32_t ssrc, RTP_PAYLOAD_TYPE type,
    void* priv,
    void(*callback)(void* priv, uint8_t* rtp, int32_t rtpSize, RTP_PAYLOAD_TYPE type))
{
    RtpStruct rtp = {0};

    int32_t partSize = 0;
    int32_t retPkt = 0;

    uint8_t chn = 0;
    uint16_t freq;
    uint16_t frameLen = 0;
    uint16_t pcmLen = 0;
    int32_t duration = 0;

    while (frameSize > (int32_t)sizeof(AACHeader))
    {

        if (AACHeaderUnPacket((AACHeader*)frame, &chn, &freq, &frameLen, &pcmLen) != 0)
        {
            RTP_INFO("unknown aac frame type, %02X %02X %02X %02X %02X %02X %02X, size/%d\r\n",
                frame[0], frame[1], frame[2], frame[3], frame[4], frame[5], frame[6], frameSize);
            break;
        }

        frame += sizeof(AACHeader);
        duration = (int32_t)pcmLen * 1000000 / (freq * chn);
        frameSize -= frameLen;

        frameLen -= sizeof(AACHeader);
        while (frameLen > 0)
        {
            partSize = frameLen > RTP_PACKET_PAYLOAD_DATA_SIZE ?
                RTP_PACKET_PAYLOAD_DATA_SIZE : frameLen;

            RtpHeaderPacket(&rtp.header, type, *seq, *tm, ssrc, partSize == frameLen ? 1 : 0);

            if (callback)
            {
                // aac packet number (hight 12bit) = 1
                rtp.payload[0] = 0x00;
                rtp.payload[1] = 0x10;
                // aac packet len (hight 13bit) = partSize
                rtp.payload[2] = (uint8_t)((partSize >> 5) & 0xFF);
                rtp.payload[3] = (uint8_t)((partSize << 3) & 0xFF);
                // aac data
                memcpy(&rtp.payload[4], frame, partSize);
                
                callback(priv, (uint8_t*)&rtp, partSize + sizeof(RtpHeader) + 4, type);
            }

            frame += partSize;
            frameLen -= partSize;
            retPkt += 1;
            *seq += 1;
        }

        *tm += duration;
    }


    return retPkt;
}

static int32_t RtpH264Packet(
    uint8_t* frame, int32_t frameSize,
    uint16_t* seq, uint32_t* tm, uint32_t duration,
    uint32_t ssrc, RTP_PAYLOAD_TYPE type,
    void* priv,
    void(*callback)(void* priv, uint8_t* rtp, int32_t rtpSize, RTP_PAYLOAD_TYPE type))
{
    RtpStruct rtp = {0};

    int32_t partSize = 0;
    int32_t retPkt = 0;

    int32_t offset = 0;
    uint8_t frameBegin = 0;

    while (frameSize > 4)
    {
        // seek
        partSize = H26XSeek(frame, frameSize, &offset, NULL, NULL, NULL);
        if (partSize <  4)
        {
            partSize = frameSize;
            offset = 0;
            frameSize = 0;
        }
        else
        {
            frame += offset;
            frameSize -= offset + partSize;
        }

        // drop header
        if (frame[0] == 0 && frame[1] == 0 && frame[2] == 1)
        {
            frame += 3;
            partSize -= 3;
        }
        else if (frame[0] == 0 && frame[1] == 0 && frame[2] == 0 && frame[3] == 1)
        {
            frame += 4;
            partSize -= 4;
        }
        else
        {
            RTP_INFO("unknown h26x frame type, %02X %02X %02X %02X %02X, size/%d, ret/%d, offset/%d\r\n",
                frame[0], frame[1], frame[2], frame[3], frame[4], frameSize, partSize, offset);
            break;
        }

        // multi rtp frame
        if (partSize > RTP_PACKET_PAYLOAD_DATA_SIZE)
        {
            frameBegin = frame[0] & 0x1F;

            // H26X_FU_A_BEGIN 
            RtpHeaderPacket(&rtp.header, type, *seq, *tm, ssrc, 0);
            if (callback)
            {
                rtp.payload[0] = 0x7C;
                memcpy(&rtp.payload[1], frame, RTP_PACKET_PAYLOAD_DATA_SIZE);
                rtp.payload[1] = H26X_FU_A_BEGIN | frameBegin;

                callback(priv, (uint8_t*)&rtp, RTP_PACKET_PAYLOAD_DATA_SIZE + sizeof(RtpHeader) + 1, type);
            }

            frame += RTP_PACKET_PAYLOAD_DATA_SIZE;
            partSize -= RTP_PACKET_PAYLOAD_DATA_SIZE;

            retPkt += 1;
            *seq += 1;
            
            // H26X_FU_A_MIDDLE
            while (partSize > RTP_PACKET_PAYLOAD_DATA_SIZE)
            {
                RtpHeaderPacket(&rtp.header, type, *seq, *tm, ssrc, 0);
                if (callback)
                {
                    rtp.payload[0] = 0x7C;
                    rtp.payload[1] = H26X_FU_A_MIDDLE | frameBegin;
                    memcpy(&rtp.payload[2], frame, RTP_PACKET_PAYLOAD_DATA_SIZE);

                    callback(priv, (uint8_t*)&rtp, RTP_PACKET_PAYLOAD_DATA_SIZE + sizeof(RtpHeader) + 2, type);
                }

                frame += RTP_PACKET_PAYLOAD_DATA_SIZE;
                partSize -= RTP_PACKET_PAYLOAD_DATA_SIZE;

                retPkt += 1;
                *seq += 1;
            }

            // H26X_FU_A_END
            RtpHeaderPacket(&rtp.header, type, *seq, *tm, ssrc, 1);
            if (callback)
            {
                rtp.payload[0] = 0x7C;
                rtp.payload[1] = H26X_FU_A_END | frameBegin;
                memcpy(&rtp.payload[2], frame, partSize);

                callback(priv, (uint8_t*)&rtp, partSize + sizeof(RtpHeader) + 2, type);
            }
        }
        // single rtp frame
        else
        {
            RtpHeaderPacket(&rtp.header, type, *seq, *tm, ssrc, 1);
            if (callback)
            {
                memcpy(rtp.payload, frame, partSize);
                callback(priv, (uint8_t*)&rtp, partSize + sizeof(RtpHeader), type);
            }
        }

        frame += partSize;
        partSize = 0;
        
        retPkt += 1;
        *seq += 1;
        *tm += duration;
    }

    return retPkt;
}

/* ========================= api xxx unpacket (static) ========================= */

static int32_t RtpPCMUnPacket(
    uint8_t* payload, int32_t payloadSize,
    uint8_t* frame, int32_t frameSize,
    uint16_t seq, RtpCircleCache* rcc)
{
    if (payloadSize > 0)
        RtpCircleCacheIn(payload, payloadSize, RTP_CIRCLE_FRAME_SINGLE, seq, rcc);
    return RtpCircleCacheOut(frame, frameSize, rcc);
}

static int32_t RtpAACUnPacket(
    uint8_t* payload, int32_t payloadSize,
    uint8_t* frame, int32_t frameSize,
    uint16_t seq, RtpCircleCache* rcc,
    uint8_t chn, uint16_t freq, uint16_t bf)
{
    AACHeader header;
    uint8_t* pFrame = frame;

    int32_t frameCount = 0;
    int32_t dataSizeTmp = 0;
    int32_t dataSize = 0;
    int32_t offsetOfLen = 0;
    int32_t offsetOfData = 0;

    frameCount = ((int32_t)payload[offsetOfLen++] & 0xFF) << 4;
    frameCount |= ((int32_t)payload[offsetOfLen++] >> 4) & 0x0F;

    offsetOfLen = 2;
    offsetOfData = 2 + frameCount * 2;

    while (frameCount-- > 0 &&
        offsetOfLen < payloadSize &&
        offsetOfData < payloadSize)
    {
        dataSizeTmp = ((int32_t)payload[offsetOfLen++] & 0xFF) << 5;
        dataSizeTmp |= ((int32_t)payload[offsetOfLen++] >> 3) & 0x1F;

        if (dataSizeTmp > 0 && offsetOfData + dataSizeTmp <= payloadSize)
        {
            // copy header
            AACHeaderPacket(&header, chn, freq, bf, dataSizeTmp);
            memcpy(pFrame, &header, sizeof(AACHeader));
            pFrame += sizeof(AACHeader);
            dataSize += sizeof(AACHeader);
            // copy data
            memcpy(pFrame, &payload[offsetOfData], dataSizeTmp);
            pFrame += dataSizeTmp;
            dataSize += dataSizeTmp;
            offsetOfData += dataSizeTmp;
        }
    }

    if (dataSize > 0)
        RtpCircleCacheIn(frame, dataSize, RTP_CIRCLE_FRAME_SINGLE, seq, rcc);

    return RtpCircleCacheOut(frame, frameSize, rcc);
}

static int32_t RtpH264UnPacket(
    uint8_t* payload, int32_t payloadSize,
    uint8_t* frame, int32_t frameSize,
    uint16_t seq, RtpCircleCache* rcc)
{
    uint8_t* pFrame = frame;

    int32_t dataOffset = -1;
    int32_t dataSize = -1;
    int32_t dataSizeTmp = -1;

    if (payloadSize > 2 && frameSize >= payloadSize)
    {
        switch (payload[0] & 0x1F)
        {
            case H26X_FRAME_NAL_MIN:
            case H26X_FRAME_PA:
            case H26X_FRAME_PB:
            case H26X_FRAME_PC:
            case H26X_FRAME_IDR:
            case H26X_FRAME_SEI:
            case H26X_FRAME_SPS:
            case H26X_FRAME_PPS:
            case H26X_FRAME_NAL_MAX:
            {
                // frame header
                memcpy(pFrame, gH264Header, sizeof(gH264Header));
                pFrame += sizeof(gH264Header);
                // frame type
                *pFrame++ = (payload[0] & 0x1F) | 0x60;
                // frame data
                memcpy(pFrame, &payload[1], payloadSize - 1);
                dataSize = payloadSize + sizeof(gH264Header);
                // cache in
                RtpCircleCacheIn(frame, dataSize, RTP_CIRCLE_FRAME_SINGLE, seq, rcc);
            }
            break; // case H26X_FRAME_NAL_MAX:

            case H26X_FRAME_STAP_A:
            {
                dataSize = 0;
                dataOffset = 1;
                // loop: [len][data][len][data][..][..]
                while (dataOffset < payloadSize)
                {
                    dataSizeTmp = ((uint16_t)payload[dataOffset++]) << 8;
                    dataSizeTmp |= payload[dataOffset++];
                    if (dataSizeTmp > 0 && dataOffset + dataSizeTmp <= payloadSize)
                    {
                        // frame header
                        memcpy(pFrame, gH264Header, sizeof(gH264Header));
                        pFrame += sizeof(gH264Header);
                        // frame type
                        *pFrame = (payload[dataOffset] & 0x1F) | 0x60;
                        pFrame += 1;
                        dataOffset += 1;
                        // frame data
                        memcpy(pFrame, &payload[dataOffset], dataSizeTmp - 1);
                        pFrame += dataSizeTmp - 1;
                        dataOffset += dataSizeTmp - 1;
                        // cache in
                        dataSize += dataSizeTmp + sizeof(gH264Header);
                    }
                    else
                    {
                        break;
                    }
                }

                if (dataSize > 0)
                    RtpCircleCacheIn(frame, dataSize, RTP_CIRCLE_FRAME_SINGLE, seq, rcc);
            }
            break; // case H26X_FRAME_STAP_A:

            case H26X_FRAME_STAP_B:
            {
                RTP_INFO("H26X_FRAME_STAP_B not supported now \r\n");
                dataSize = 0;
            }
            break; // case H26X_FRAME_STAP_B:

            case H26X_FRAME_MTAP16:
            {
                RTP_INFO("H26X_FRAME_MTAP16 not supported now \r\n");
                dataSize = 0;
            }
            break; // case H26X_FRAME_MTAP16:

            case H26X_FRAME_MTAP32:
            {
                RTP_INFO("H26X_FRAME_MTAP32 not supported now \r\n");
                dataSize = 0;
            }
            break; // case H26X_FRAME_MTAP32:

            case H26X_FRAME_FU_A:
            {
                switch (payload[1] & 0xE0)
                {
                    case H26X_FU_A_BEGIN:
                    {
                        // frame header
                        memcpy(pFrame, gH264Header, sizeof(gH264Header));
                        pFrame += sizeof(gH264Header);
                        // frame type
                        *pFrame++ = (payload[1] & 0x1F) | 0x60;
                        // frame data
                        memcpy(pFrame, &payload[2], payloadSize - 2);
                        dataSize = sizeof(gH264Header) + payloadSize - 1;
                        // cache in
                        RtpCircleCacheIn(frame, dataSize, RTP_CIRCLE_FRAME_BEGIN, seq, rcc);
                    }
                    break; // case H26X_FU_A_BEGIN:

                    case H26X_FU_A_MIDDLE:
                    {
                        // frame data
                        memcpy(pFrame, &payload[2], payloadSize - 2);
                        // cache in
                        RtpCircleCacheIn(frame, payloadSize - 2, RTP_CIRCLE_FRAME_MID, seq, rcc);
                        // no copy out
                        dataSize = 0;
                    }
                    break; // case H26X_FU_A_MIDDLE:

                    case H26X_FU_A_END:
                    {
                        // frame data
                        memcpy(pFrame, &payload[2], payloadSize - 2);
                        dataSize = payloadSize - 2;
                        // cache in
                        RtpCircleCacheIn(frame, dataSize, RTP_CIRCLE_FRAME_END, seq, rcc);
                    }
                    break; // case H26X_FU_A_END:
                }
            }
            break; // case H26X_FRAME_FU_A:

            case H26X_FRAME_FU_B:
            {
                RTP_INFO("H26X_FRAME_FU_B not supported now \r\n");
                dataSize = 0;
            }
            break; // case H26X_FRAME_FU_B:
        }

        if (dataSize < 0)
        {
            RTP_INFO("unknown h26x frame type, payload[%02X %02X], payloadSize/%d\r\n",
                payload[0], payload[1], payloadSize);
            // return dataSize;
        }
    }

    return RtpCircleCacheOut(frame, frameSize, rcc);
}

/* ========================= api packet/unpacket ========================= */

int32_t RtpPacket(
    uint8_t* frame, int32_t frameSize,
    uint16_t* seq, uint32_t* tm, uint32_t duration,
    uint32_t ssrc, RTP_PAYLOAD_TYPE type,
    void* priv,
    void(*callback)(void* priv, uint8_t* rtp, int32_t rtpSize, RTP_PAYLOAD_TYPE type))
{
    int32_t ret = -1;

    switch (type)
    {
        case RTP_PAYLOAD_TYPE_PCMU:
        case RTP_PAYLOAD_TYPE_G723:
        case RTP_PAYLOAD_TYPE_PCMA:
        case RTP_PAYLOAD_TYPE_G722:
        case RTP_PAYLOAD_TYPE_G728:
        case RTP_PAYLOAD_TYPE_G729:
        {
            ret = RtpPCMPacket(
                frame, frameSize,
                seq, tm, duration,
                ssrc, type,
                priv, callback);
        }
        break;

        case RTP_PAYLOAD_TYPE_AAC:
        {
            ret = RtpAACPacket(
                frame, frameSize,
                seq, tm,
                ssrc, type,
                priv, callback);
        }
        break;

        case RTP_PAYLOAD_TYPE_H264:
        case RTP_PAYLOAD_TYPE_H264_30FPS:
        {
            ret = RtpH264Packet(
                frame, frameSize,
                seq, tm, duration,
                ssrc, type,
                priv, callback);
        }
        break;

        default:
        {
            RTP_INFO("not supported payload type %d \r\n", type);
        }
        break;
    }

    return ret;
}


int32_t RtpUnPacket(
    uint8_t* rtp, int32_t rtpSize,
    uint8_t* frame, int32_t frameSize,
    RTP_PAYLOAD_TYPE* type,
    void* cache,
    uint16_t chn, uint16_t freq)
{
    RtpStruct* rtpStruct = (RtpStruct*)rtp;
    uint16_t seq;
    int32_t retFrameSize = 0;

    if (!rtp || !frame || rtpSize < 14)
        return -1;

    *type = (RTP_PAYLOAD_TYPE)(rtpStruct->header.pt);
    seq = ((uint16_t)rtpStruct->header.seq[0] << 8) | rtpStruct->header.seq[1];

    // get payload
    switch (rtpStruct->header.pt)
    {
        case RTP_PAYLOAD_TYPE_PCMU:
        case RTP_PAYLOAD_TYPE_G723:
        case RTP_PAYLOAD_TYPE_PCMA:
        case RTP_PAYLOAD_TYPE_G722:
        case RTP_PAYLOAD_TYPE_G728:
        case RTP_PAYLOAD_TYPE_G729:
        {
            if (rtpSize - (int32_t)sizeof(RtpHeader) < RTP_PACKET_PAYLOAD_PCM_SZIE)
            {
                RTP_INFO("not enough RTP data(%ld) to include PCMX/G7XX frame(%d)\r\n",
                    rtpSize - sizeof(RtpHeader), RTP_PACKET_PAYLOAD_PCM_SZIE);
            }

            retFrameSize = RtpPCMUnPacket(
                &rtpStruct->payload[0],
                RTP_PACKET_PAYLOAD_PCM_SZIE,
                frame,
                frameSize,
                seq, (RtpCircleCache*)cache);
        }
        break;

        case RTP_PAYLOAD_TYPE_AAC:
        {
            retFrameSize = RtpAACUnPacket(
                &rtpStruct->payload[0],
                rtpSize - sizeof(RtpHeader),
                frame,
                frameSize,
                seq, (RtpCircleCache*)cache,
                (uint8_t)(chn & 0xFF),
                (uint16_t)(freq & 0xFFFF),
                0x7FF);
        }
        break;

        case RTP_PAYLOAD_TYPE_H264:
        case RTP_PAYLOAD_TYPE_H264_30FPS:
        {
            retFrameSize = RtpH264UnPacket(
                &rtpStruct->payload[0],
                rtpSize - sizeof(RtpHeader),
                frame,
                frameSize,
                seq, (RtpCircleCache*)cache);
        }
        break;

        default:
        {
            RTP_INFO("not supported rtp type %d \r\n", rtpStruct->header.pt);
            retFrameSize = 0;
        }
        break;
    }

    return retFrameSize;
}
