#pragma once

#include "os_config.h"
#include <string>
#include <memory>
#include "BmReactor.h"
#include <mutex>
#include "BmMsg.h"
#include <condition_variable>


class BmTimer
{
public:
    BmTimer();

    static BmTimer* Instance();

    int AddTimer(const shared_ptr<BmReactor>& reactor,
        BmMsg& msg, int inter, bool repeat);
    void DelTimer(int id);
    void ResetTimer(int id);

    void Run();
    void OnRun();
    void Exit();

private:
    class BmTimerItem
    {
    public:
        BmTimerItem(const shared_ptr<BmReactor>& reactor,
            int id, BmMsg& msg, int inter, bool repeat);

        ~BmTimerItem();

        bool Expire();

        shared_ptr<BmReactor> m_reactor;
        BmMsg m_msg;
        int   m_inter;
        bool  m_repeat;
        int   m_curTick;
        int   m_timerID;
    };

    void DelTimer_i(int id);

    bool m_exit;
    map<int, shared_ptr<BmTimerItem>> m_timerItems;
    int m_timerID;

    static unique_ptr<BmTimer>  m_timerMgr;
    static mutex m_mutex;
    static condition_variable m_condiVar;
};

