#pragma once
#include "os_config.h"

#include <memory>
#include <mutex>
#include <queue>
#include <condition_variable>

#define LOG_LEVEL_ERROR 0
#define LOG_LEVEL_WARN  1
#define LOG_LEVEL_INFO  2
#define LOG_LEVEL_DEBUG 3
#define LOG_LEVEL_VERBS 4

class BmLog
{
public:
    BmLog();
    ~BmLog();

    static BmLog* Instance();

    void Log(int level, const char* strlog, ...);

    void Run();
    void OnRun();
    void Exit();
    void SetLevel(int level);

private:
    char* GetLogBuf();
    void RelLogBuf(char* buf);
    void EnqueLog(char* log);

    int m_level;
    bool m_exit;
    queue<char*> m_logBuf;
    queue<char*> m_logQue;
    FILE* m_fp;
    int m_curHour;

    static unique_ptr<BmLog>  m_log;
    static mutex m_mutex;
    static condition_variable m_condiVar;
};

static const char* logLevel[] = { "ERROR","WARN","INFO","DEBUG", "VERBS" };

#define BM_LOG(LEVEL, FORMAT_STRING, ...) \
{\
    BmLog::Instance()->Log(LEVEL, "%s %s,%d: " FORMAT_STRING,  logLevel[LEVEL],  __FILE__,  __LINE__, ##__VA_ARGS__); \
}

#define BM_LOG_INFO(FORMAT_STRING, ...) BM_LOG(LOG_LEVEL_INFO, FORMAT_STRING, ##__VA_ARGS__)
#define BM_LOG_WARN(FORMAT_STRING, ...) BM_LOG(LOG_LEVEL_WARN, FORMAT_STRING, ##__VA_ARGS__)
#define BM_LOG_ERROR(FORMAT_STRING, ...) BM_LOG(LOG_LEVEL_ERROR, FORMAT_STRING, ##__VA_ARGS__)
#define BM_LOG_DEBUG(FORMAT_STRING, ...) BM_LOG(LOG_LEVEL_DEBUG, FORMAT_STRING, ##__VA_ARGS__)
#define BM_LOG_VERBS(FORMAT_STRING, ...) BM_LOG(LOG_LEVEL_VERBS, FORMAT_STRING, ##__VA_ARGS__)