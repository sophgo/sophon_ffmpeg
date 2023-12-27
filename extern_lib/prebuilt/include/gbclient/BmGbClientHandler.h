#pragma once
#include "BmEvent.h"

class BmIGbClient;

class BmGbClientHandler :
    public BmEventHandler
{
public:
    BmGbClientHandler(const shared_ptr<BmIGbClient>& client);
    ~BmGbClientHandler();

    void HandleRead(const shared_ptr<BmEvent>& evt);
    void HandleClose(const shared_ptr<BmEvent>& evt);

private:
    shared_ptr<BmIGbClient> m_client;

    char*      m_buf;
    int        m_bufSize;
    int        m_bufOff;
    BmInetAddr m_recvAddr;
};

