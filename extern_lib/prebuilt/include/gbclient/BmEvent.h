#pragma once
#include "BmSocket.h"
#include <memory>

class BmEvent;

class BmEventHandler
{
public:
    virtual void HandleRead(const shared_ptr<BmEvent>& evt) = 0;
    virtual void HandleClose(const shared_ptr<BmEvent>& evt) = 0;
    virtual void HandleWrite(const shared_ptr<BmEvent>& evt) {}
};

class BmEvent : public enable_shared_from_this<BmEvent>
{
public:
    BmEvent(shared_ptr<BmSocket>& sock, int eventMask,
        const shared_ptr<BmEventHandler>& handler);

#ifdef WIN32
    BmEvent(BM_EVENT evt, shared_ptr<BmEventHandler>& handler);
#endif

    ~BmEvent();

    BM_EVENT* GetEvent();
    BmSocket* GetSocket();
    shared_ptr<BmSocket> GetSharedSocket();

    void HandleEvent(uint32_t eventMask);

private:
    BM_EVENT m_event;
    int m_eventMask;
    shared_ptr<BmSocket> m_sock;
    shared_ptr<BmEventHandler> m_handler;
};



