#pragma once

#ifdef BM_WIN
#include <WinSock2.h>
#include <WS2tcpip.h>
#include <Shlwapi.h>
#include <stdint.h>

using BM_SOCKET = SOCKET;
using BM_EVENT = HANDLE;

#define BM_INVALID_SOCKET INVALID_SOCKET
#define BM_MAX_EVENTS MAXIMUM_WAIT_OBJECTS
#define bm_strcasestr StrStrIA
#define bm_strcasecmp _stricmp
#define BM_FD_READ FD_READ
#define BM_FD_CLOSE FD_CLOSE
#define BM_FD_ACCEPT FD_ACCEPT
#define BM_FD_CONNECT FD_CONNECT

#define BM_LAST_ERROR WSAGetLastError()

#else

#include <sys/epoll.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <fcntl.h>
#include <signal.h>
#include <sys/eventfd.h>
#include <strings.h>
#include <iconv.h>

using BM_SOCKET = int;
using BM_EVENT = epoll_event;

#define BM_INVALID_SOCKET -1
#define BM_MAX_EVENTS 1024
#define bm_strcasestr strcasestr
#define bm_strcasecmp strcasecmp
#define BM_FD_READ EPOLLIN
#define BM_FD_CLOSE EPOLLRDHUP
#define BM_FD_ACCEPT EPOLLIN
#define BM_FD_CONNECT EPOLLOUT

#define BM_LAST_ERROR errno

#endif

using namespace std;
