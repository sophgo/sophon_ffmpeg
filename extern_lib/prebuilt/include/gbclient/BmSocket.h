#pragma once
#include "os_config.h"
#include "BmInetAddr.h"
#include <memory>

class BmSocket
{
public:
    BmSocket(BM_SOCKET s);
    BmSocket(int af, int type, int protocol);
    ~BmSocket();

    int Bind(const BmInetAddr& addr);
    int Connect(const BmInetAddr& addr);
    int Connect(string& ip, int port);
    int Listen();
    int Accept(shared_ptr<BmSocket>& rSock);
    int Recv(char* buf, int len);
    int Recvfrom(char* buf, int len, BmInetAddr& addr);
    int Send(const char* buf, int len);
    int Sendto(const char* buf, int len, BmInetAddr& addr);
    void SetNonBlock();
    void SetBlock();

    BM_SOCKET GetFd();

private:
    BM_SOCKET m_sock;
};

