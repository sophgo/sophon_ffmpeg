#pragma once
#include "libavformat/gb28181.h"
int init_sip_recv_wrap(RegistrationInfo *rt,char **ptr,int udp);
int send_register_wrap(char **ptr);
int send_invite_wrap(char **ptr,int udp, char *proto);
int send_message_wrap(char **ptr);
int send_bye_wrap(char **ptr);
int send_play_wrap(char **ptr);
int send_pause_wrap(char **ptr);
int close_gbclient_wrap(char **ptr);
int get_register_status_wrap(char **ptr);
int get_ack_status_wrap(char **ptr);
