#pragma once

#include "BmCommon.h"
#include "BmInetAddr.h"

class BmSocket;

class BmSipContact : public BmComHeader
{
public:
    BmSipContact(const char* key) : BmComHeader(key) {}

    string GetID();
    string GetIP();
    int GetPort();
};

class BmSipCSeq : public BmComHeader
{
public:
    BmSipCSeq(const char* key) : BmComHeader(key) {}

    int GetCSeq();
    string GetMethond();
};

class BmSipFrom : public BmComHeader
{
public:
    BmSipFrom(const char* key) : BmComHeader(key) {}

    string GetID();
    bool HasTag();
    void AppendTag(string tag);
};

class BmSipVia : public BmComHeader
{
public:
    BmSipVia(const char* key) : BmComHeader(key){}

    string GetTransport();
    string GetBranch();
    bool HasRport();
    void Rebuild(const string& transport, const string& branch, BmInetAddr& recvAddr);
};

class BmSipMsg : public BmComMsg
{
public:
    BmSipMsg();

    void Dump(string& rsp);
    void Parse(char*& p2);
    void CloneBasic(BmSipMsg& from);

    BmSipVia     m_via;
    BmSipFrom    m_from;
    BmSipFrom    m_to;
    BmComHeader  m_callID;
    BmSipCSeq    m_cseq;
    BmSipContact m_contact;
    BmComIntVal  m_maxForwards;
    BmComHeader  m_userAgent;
    BmComIntVal  m_expires;
    BmComHeader  m_subject;
    BmComHeader  m_wwwAuthenticate;
    BmComAuth    m_authorization;
    BmComHeader  m_date;
    BmComHeader  m_event;
    BmComHeader  m_xSource;
};

string GenNonce();

void ngx_gmtime(time_t t, struct tm* tp);
string GetTimeStr();
bool AuthValid(BmComAuth& sipAuth, string& method, string pass);

void BuildIP(string& uri, const string& ip, int port);
void BuildUri(string& uri, const string& id, const string& ip, int port);
void BuildUriNoId(string& uri, const string& ip, int port);
void BuildTo(BmSipFrom& from, const string& id, const string& ip, int port);
void BuildFrom(BmSipFrom& from, const string& id, const string& ip, int port);
void BuildVia(BmComHeader& via, const string& ip, int port);
void BuildCSeq(BmSipCSeq& cseq, int seq, const string& method);
void BuildContact(BmSipContact& contact, const string& id, const string& ip, int port);
void BuildSubject(BmComHeader& subject, const string& sender, const string& recver, bool live);
void BuildUriRegister(string& uri,const string& ip, int port);
//void BuildAuthorization(BmComAuth& auth, string& pszUserName, string& pszRealm, string& pszPassword, string& nonce, bool live);
void BuildAuthorization(BmComAuth& auth,  string &method, string& pszUserName, string& pszRealm, string& pszPassword, string& nonce, string& pszAlgorithm, string& toID, string& toIP, int toPort, bool live);
int SendSipMsg(BmSipMsg& msg, BmSocket* s, string ip, int port);
int SendSipMsg(BmSipMsg& msg, BmSocket* s, BmInetAddr& addr);
void BuildSipMsg(const string& fromIP, int fromPort, const string& fromID,
    const string& toIP, int toPort, const string& toID,
    int cseq, const string& method, BmSipMsg& sipMsg);

