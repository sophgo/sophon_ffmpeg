#pragma once
#include "libavformat/gb28181.h"
#include "BmIGbClient.h"
#include <iostream>
#include <memory>
#include <map>
using namespace std;
class BmGbClient : public BmIGbClient
{
public:
    using BmIGbClient::BmIGbClient;

    void Run(RegistrationInfo *rt,int udp);
    void HandleMsg(ffBmMsg& msg);
    void KeepLive();
    void HandleResponse(BmSipMsg& sipMsg, char* body, int len);
    void HandleBye(BmSipMsg& sipMsg, BmSocket* sock, BmInetAddr& addr);
    void HandleMessage(BmSipMsg& sipMsg, BmSocket* sock, BmInetAddr& addr, char* body, int len);
    void HandleInvite(BmSipMsg& sipMsg, BmSocket* sock, BmInetAddr& addr, char* body, int len);
    void HandleAck(BmSipMsg& sipMsg, BmSocket* sock, BmInetAddr& addr);
    void HandleSubscribe(BmSipMsg& sipMsg, BmSocket* sock, BmInetAddr& addr, char* body, int len);
    int  getSeq();
    int  GetReisterStatus();
    int  GetAckStatus();
    int  NetStatus();
    void GetSipMediaUriInSDP(char* sdp, char* ip, int* port);
    void sipCilentExit();
private:
    void HandleInviteRsp(BmSipMsg& msg, int status, char* body, int len);
    void HandleRegisterRsp(BmSipMsg& msg, int status, char* body, int len);
    void InitInvite(ffBmMsg& msg);
    void InitBye();
    void InitMessage();
    void InitRegister();
    void InitPause();
    void InitPlay();

private:
    shared_ptr<BmSocket> m_sock;
    RegistrationInfo *rt;
    string m_localIP;
    string m_localID;
    int    m_localPort;
    string callID;
    string fromUri;
    string toUri;
    bool   authSuccess;
    bool SocketInitSucess;
    int m_seq;
    bool isExit;
    bool moudleExit;
    bool KeepLiveExit;
    bool sendAck;
    int  timeOut;

    class InviteCtx
    {
    public:
        BmSipMsg m_rsp;
        int m_type;
        int m_id;
    };

    map<string, shared_ptr<InviteCtx>> m_inviteCtx;
};

