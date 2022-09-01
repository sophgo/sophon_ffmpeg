#pragma once
#include "libavformat/gb28181.h"
#include "Gb28181SipWrap.h"
#ifdef __cplusplus
extern "C" {
#endif
int init_sip_recv(RegistrationInfo *rt,char **ptr,int udp);
int send_invite(char **ptr,int udp, char *proto);
int send_register(char **ptr);
int send_message(char **ptr);
int send_bye(char **ptr);
int send_play(char **ptr);
int send_pause(char **ptr);
int close_gbclient(char **ptr);
int get_register_status(char **ptr);
int get_ack_status(char **ptr);
#ifdef __cplusplus
}
#endif
