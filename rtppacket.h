#ifndef _RTP_PACKET_H_
#define _RTP_PACKET_H_

#include <stdint.h>

typedef enum
{
    RTP_PAYLOAD_TYPE_PCMU = 0,
    RTP_PAYLOAD_TYPE_GSM = 3,
    RTP_PAYLOAD_TYPE_G723 = 4,
    RTP_PAYLOAD_TYPE_PCMA = 8,
    RTP_PAYLOAD_TYPE_G722 = 9,
    RTP_PAYLOAD_TYPE_G728 = 15,
    RTP_PAYLOAD_TYPE_G729 = 18,

    RTP_PAYLOAD_TYPE_H261 = 31, // not supported now
    RTP_PAYLOAD_TYPE_MPV = 32, // not supported now
    RTP_PAYLOAD_TYPE_MP2T = 33, // not supported now
    RTP_PAYLOAD_TYPE_H263 = 34, // not supported now

    RTP_PAYLOAD_TYPE_H264 = 96,
    RTP_PAYLOAD_TYPE_AAC = 97,

    RTP_PAYLOAD_TYPE_H264_UNKNOWN1 = 114,
    RTP_PAYLOAD_TYPE_H264_UNKNOWN2 = 118,
    RTP_PAYLOAD_TYPE_H264_UNKNOWN3 = 123,
} RTP_PAYLOAD_TYPE;

/*
 *  rtp字符串截取(首包必须为rtp数据,否则无法确定ssrc)
 *  参数:
 *      buff, buffSize: 输入字符串
 *      offset: 返回rtp数据在buff的起始偏移量
 *      ssrc[4]: rtp流唯一标识,第一次调用传入{0,0,0,0}函数进行写入
 *  返回: 成功返回识别rtp长度,失败返回-1
 */
int32_t RtpSeek(
    uint8_t* buff, int32_t buffSize, int32_t* offset,
    uint8_t ssrc[4]);

int32_t H26XSeek(
    uint8_t* buff, int32_t buffSize, int32_t* offset,
    int32_t* width, int32_t* height, int32_t* fps);

int32_t AACSeek(
    uint8_t* buff, int32_t buffSize, int32_t* offset,
    int32_t* chn, int32_t* freq, uint32_t* duration);

/*
 *  音视频帧打包rtp
 *  参数:
 *      frame, frameSize: 输入音视频帧
 *      rtp, rtpSize: 输出缓冲区
 *      seq: 包序号,函数内增加
 *      tm: 时间戳,单位us,函数内增加
 *      duration: 该包数据播放时长 (aac打包时自动计算)
 *      ssrc[4]: rtp流唯一标识
 *      type: 数据类型,务必准确填写
 *      priv, callback: 成功打包时的回调,用于一帧多包情况,不需要则写NULL
 *  返回: 成功返回rtp发包数, 失败返回-1
 */
int32_t RtpPacket(
    void* cache,
    uint8_t* frame, int32_t frameSize,
    uint16_t* seq, uint32_t* tm, uint32_t duration,
    uint32_t ssrc, RTP_PAYLOAD_TYPE type,
    void* priv,
    void(*callback)(void* priv, uint8_t* rtp, int32_t rtpSize, RTP_PAYLOAD_TYPE type));

/*
 *  rtp解包为音视频帧
 *  参数:
 *      rtp, rtpSize: 输入rtp包(必须为完整的一包)
 *      frame, frameSize: 输出音视频缓冲区
 *      type: 返回数据类型
 *      cache: 缓冲区指针,用于解决网络抖动带来的收包顺序问题
 *      chn,freq: aac补全帧头部时需要,其它时候写0即可
 *  返回: 成功返回帧数据长度, 失败返回-1
 */
int32_t RtpUnPacket(
    void* cache,
    uint8_t* rtp, int32_t rtpSize,
    uint8_t* frame, int32_t frameSize,
    RTP_PAYLOAD_TYPE* type,
    uint16_t chn, uint16_t freq);

/*
 *  申请rtp解包用缓冲区的申请和释放
 *  参数:
 *      packetCount: 缓存包数量(每包最多1500字节),建议音频不小于128,视频不小于512
 */
void* RtpUnPacketCacheInit(int32_t packetCount);
void RtpUnPacketCacheRelease(void* cache);

void* RtpPacketCacheInit();
void RtpPacketCacheRelease(void* cache);

#endif // _RTP_PACKET_H_
