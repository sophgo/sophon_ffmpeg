#pragma once
#include "BmEvent.h"
#include <map>
#include <queue>
#include <mutex>
#include "BmMsg.h"

class BmReactor : public enable_shared_from_this<BmReactor>
{
public:
    BmReactor(int type, int id);
    virtual ~BmReactor();

    inline int GetType() { return m_type; }
    inline int GetID() { return m_id; }

    int AddEvent(const shared_ptr<BmEvent>& evt);
    int DelEvent(const shared_ptr<BmEvent>& evt);

    int AddTimer(BmMsg& msg, int inter, bool repeat = false);
    void DelTimer(int id);
    void ResetTimer(int id);

    virtual void Run();
    virtual void Exit();
    virtual void HandleMsg(BmMsg& msg);
    virtual void KeepLive(){};
    int Wait();

    void EnqueMsg(BmMsg& msg);
    void ProcessMsgQue();

    int PostMsg(BmMsg& msg);
    void PostExit();

private:
    int m_type;
    int m_id;
    bool m_exit;

    BM_EVENT m_eventHandles[BM_MAX_EVENTS];
    int m_index;

#ifdef WIN32
    shared_ptr<BmEvent > m_events[BM_MAX_EVENTS];
    BM_EVENT m_eventfd;
#else
    map<int, shared_ptr<BmEvent>> m_events;
    int m_efd;
    int m_eventfd;
#endif

    class BmNotifyHandler : public BmEventHandler
    {
    public:
        BmNotifyHandler(const shared_ptr<BmReactor>& reactor);
        ~BmNotifyHandler();

        void HandleRead(const shared_ptr<BmEvent>& evt);
        void HandleClose(const shared_ptr<BmEvent>& evt) {};

    private:
        shared_ptr<BmReactor> m_reactor;
    };

    queue<BmMsg> m_msgQue;

protected:
    mutex m_mutex;
};

class BmReactorMgr
{
public:
    BmReactorMgr();

    void Regist(const shared_ptr<BmReactor>& reactor);
    void UnRegist(const shared_ptr<BmReactor>& reactor);

    int PostMsg(BmMsg& msg);
    shared_ptr<BmReactor> GetReactor(int type, int id);

    static BmReactorMgr* Instance();

private:
    map<int, map<int, shared_ptr<BmReactor>>> m_reactors;

    static unique_ptr<BmReactorMgr>  m_manager;
    static mutex m_mutex;
};