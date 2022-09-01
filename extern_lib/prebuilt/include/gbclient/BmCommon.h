#pragma once
#include <string>
#include <map>
#include <memory>

using namespace std;

#define DEF_BUF_SIZE                512 * 1024
#define MAX_PAYLOAD_SIZE            1400
#define RTP_VERSION                 2
#define STREAM_TYPE_VIDEO_H264      0x1b
#define STREAM_TYPE_VIDEO_HEVC      0x24

#define UPLOAD_FILE_PREFIX "/home/lisi/upload_files/"

#define AV_RB16(x)  ((((const uint8_t*)(x))[0] << 8) | ((const uint8_t*)(x))[1])
#define AV_RB32(x)  ((((const uint8_t*)(x))[0] << 24) | \
    (((const uint8_t*)(x))[1] << 16) | \
    (((const uint8_t*)(x))[2] <<  8) | \
    ((const uint8_t*)(x))[3])
#define RTP_FLAG_MARKER 0x2 ///< RTP marker bit was set for this packet

static const uint8_t start_sequence[] = { 0, 0, 0, 1 };

bool IsHeaderComplete(char* p);
void SkipToSpace(char*& p);
void SkipSpace(char*& p);
void ParseReqLine(char*& buf, string& method, string& uri, string& version);
int GetHeaderLine(char*& buf, string& line);
void ParseHeaderLine(char*& buf, map <string, string>& headerLines);
void SkipToLineEnd(char*& p);
void SkipLineEnd(char*& p);
void AddHeaderLine(string& rsp, const string& key, const string& value);
void BuildFirstLine(string& rsp, string& s1, string& s2, string& s3);
void GetHostIP(string& ip, string& host);
void GmtTimeToStr(int64_t t, string& str);

int avio_r8(uint8_t*& pb);
unsigned int avio_rb16(uint8_t*& s);
unsigned int avio_rb32(uint8_t*& s);

void avio_w8(uint8_t*& s, int b);
void avio_wb16(uint8_t*& s, unsigned int val);
void avio_wb32(uint8_t*& s, unsigned int val);

const uint8_t* ff_avc_find_startcode(const uint8_t* p, const uint8_t* end);
const uint8_t* ff_avc_mp4_find_startcode(const uint8_t* start, const uint8_t* end, int nal_length_size);

int64_t GetCurTick();
void RemoveFile(string& name);
void GbkToUtf8(string& src_str, const char* src);
time_t StrTimeToUnixTime(string& timeStamp);
int64_t GetRandom();
int64_t GetRandomEx();

class BmComHeader
{
public:
    BmComHeader(const char* key);

    void Dump(string& rsp);

    inline void SetValue(const string& value) { m_value = value; m_exist = true; }

    string m_key;
    string m_value;
    bool m_exist;
};

class BmComIntVal : public BmComHeader
{
public:
    BmComIntVal(const char* key) : BmComHeader(key) {}

    int64_t GetIntVal();
    void SetIntVal(int64_t len);
};

class BmComAuth : public BmComHeader
{
public:
    BmComAuth(const char* key) : BmComHeader(key) {}

    string GetAttr(const char* key);
};

class BmComMsg
{
public:
    BmComMsg();

    void SetBody(const char* body, int len);

    string m_method;
    string m_uri;
    string m_version;

    string m_status;
    string m_reason;

    BmComIntVal  m_contentLength;
    BmComHeader  m_contentType;

    const char* m_body;
    int   m_bodyLen;
};

class IOContext
{
public:
    IOContext();
    ~IOContext();

    bool feof();
    int r8();
    int r16();
    unsigned int r32();
    void seek(int64_t s);
    int64_t tell();
    void skip(int64_t s);
    void read(uint8_t* buf, int len);
    void append(const uint8_t* buf, int len);
    int64_t left();
    uint8_t* cur();
    void clear();
    int64_t cur_len();

private:
    uint8_t* m_buf;
    int64_t  m_len;
    int64_t  m_off;
    int      m_size;
};

void cvt_to_hex(unsigned char* in, unsigned char* out);


void SleepMs(int ms);