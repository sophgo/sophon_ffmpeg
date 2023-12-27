#pragma once
#include "BmReactor.h"
#include "BmSipMsg.h"

class BmIGbClient : public BmReactor
{
public:
    using BmReactor::BmReactor;

    virtual void HandleResponse(BmSipMsg& sipMsg, char* body, int len) = 0;
    virtual void HandleBye(BmSipMsg& sipMsg, BmSocket* sock, BmInetAddr& addr) = 0;
    virtual void HandleMessage(BmSipMsg& sipMsg, BmSocket* sock, BmInetAddr& addr, char* body, int len) = 0;
    virtual void HandleInvite(BmSipMsg& sipMsg, BmSocket* sock, BmInetAddr& addr, char* body, int len) = 0;
    virtual void HandleAck(BmSipMsg& sipMsg, BmSocket* sock, BmInetAddr& addr) = 0;
    virtual void HandleSubscribe(BmSipMsg& sipMsg, BmSocket* sock, BmInetAddr& addr, char* body, int len) = 0;
    //virtual void KeepLive() = 0;

};

