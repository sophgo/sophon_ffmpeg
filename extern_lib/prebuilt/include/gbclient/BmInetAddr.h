#pragma once
#include "os_config.h"
#include <string>

class BmInetAddr
{
public:
    BmInetAddr();
    BmInetAddr(int af, const string& ip, int port);

    int GetAF() const;
    const char* GetIP() const;
    int GetPort() const;

    void SetAF(int af);
    void SetIP(const char* ip);
    void SetPort(int port);

private:
    int m_af;
    string m_ip;
    int m_port;
};

