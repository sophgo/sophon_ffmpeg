/*
 * gb28181 definitions
 *
 */
#include "gb28181.h"
#if HAVE_WINSOCK2_H
#else
#include <ifaddrs.h>
#include <arpa/inet.h>
#endif
#include <string.h>
#include <stdlib.h>

static int gb28181_uri_split(char *path,char *localip,char *localid,char *localsipport,char *localmediaport,
             char *deviceid,char *devicetype,char *begtime,char *endtime) {
    char psubstr[16][32] = {0};
    int  len_temp = 0;
    int  pos_temp = 0;

    if(path == NULL){
        return -1;
    }
    if(*path == '\?') path++;
    for(int i=0;i<strlen(path);i++){
        if(path[i] == '#'){
            len_temp++;
            pos_temp=0;
            continue;
        }
        psubstr[len_temp][pos_temp++] = path[i];
    }

    for(int i=0;i<16;i++){
        if(!strncmp(psubstr[i],"begtime",7)){
            strcpy(begtime,&psubstr[i][8]);
        }else if(!strncmp(psubstr[i],"endtime",7)){
            strcpy(endtime,&psubstr[i][8]);
        }else if(!strncmp(psubstr[i],"localip",7)){
            strcpy(localip,&psubstr[i][8]);
        }else if(!strncmp(psubstr[i],"localid",7)){
            strcpy(localid,&psubstr[i][8]);
        //}else if(!strncmp(psubstr[i],"localsipport",12)){
        //    strcpy(localsipport,&psubstr[i][13]);
        }else if(!strncmp(psubstr[i],"localmediaport",14)){
            strcpy(localmediaport,&psubstr[i][15]);
        }else if(!strncmp(psubstr[i],"deviceid",8)){
            strcpy(deviceid,&psubstr[i][9]);
        }else if(!strncmp(psubstr[i],"devicetype",10)){
            strcpy(devicetype,&psubstr[i][11]);
        }
    }

    return 0;
}



unsigned int ff_gb28181_gettickcurmill(void) {
#if HAVE_CLOCK_GETTIME && defined(CLOCK_MONOTONIC)
    struct timespec tp;
    clock_gettime(CLOCK_MONOTONIC,&tp);
    return tp.tv_sec*1000 + tp.tv_nsec/1000/1000;
#else
    return av_gettime() + 42 * 60 * 60 * INT64_C(1000000);
#endif
}

int getLocalIp(char *ip)
{
#if !HAVE_WINSOCK2_H
    struct sockaddr_in *sin = NULL;
    struct ifaddrs *ifa = NULL, *ifList;
    if (getifaddrs(&ifList) < 0)
    {
        return -1;
    }

    for (ifa = ifList; ifa != NULL; ifa = ifa->ifa_next)
    {
        if(ifa->ifa_addr->sa_family == AF_INET)
        {
            if(!strcmp(ifa->ifa_name,"eth0")) {
                sin = (struct sockaddr_in *)ifa->ifa_addr;
                strcpy(ip,inet_ntoa(sin->sin_addr));
                return 0;
            }
        }
    }

    freeifaddrs(ifList);
#endif
    return -1;
}

time_t convert_str_to_tm(char * str_time)
{
    struct tm tt;
    char temp[32] = {0};

    memset(&tt,0,sizeof(tt));
    memset(temp,0,32);
    memcpy(temp,str_time,4);
    tt.tm_year=atoi(temp)-1900;

    memset(temp,0,32);
    memcpy(temp,str_time+4,2);
    tt.tm_mon=atoi(temp)-1;

    memset(temp,0,32);
    memcpy(temp,str_time+6,2);
    tt.tm_mday=atoi(temp);

    memset(temp,0,32);
    memcpy(temp,str_time+8,2);
    tt.tm_hour=atoi(temp);

    memset(temp,0,32);
    memcpy(temp,str_time+10,2);
    tt.tm_min=atoi(temp);

    memset(temp,0,32);
    memcpy(temp,str_time+12,2);
    tt.tm_sec=atoi(temp);
    return mktime(&tt);
}



int ff_gb28181_parm_parse(AVFormatContext *s) {
    RegistrationInfo *rt = (RegistrationInfo *)s->priv_data;
    int ret = 0;
    char proto[128] = {0};
    char host[1024] = {0};
    char path[1024] = {0};
    char auth[128]  = {0};
    int auth_len = 0;
    int port;
    char *pauth_pwd = NULL;

    //char localpath[1024] = {0};
    char localid[32]  = {0};
    char localip[32]  = {0};
    char localportsip[32]   = {0};
    char localportmedia[32] = {0};
    char deviceid[32]   = {0};
    char devicetype[4]  = {0};
    char begtime[32]    = {0};
    char endtime[32]    = {0};
    char localipeth0[32] = {0};
    char *ls;

    av_url_split(proto, sizeof(proto), auth, sizeof(auth),
                 host, sizeof(host), &port, path, sizeof(path), s->url);

    strcpy(rt->proto,proto);
    strcpy(rt->serverip, host);
    if(port < 0) {
        rt->serverport = SIP_SERVER_DEFAULT_PORT;
    } else {
        rt->serverport = port;
    }
    pauth_pwd = &auth[0];
    while((*pauth_pwd != '\0') && (*pauth_pwd++ != ':')){
        auth_len++;
    }
    strncpy(rt->serverid,&auth[0],auth_len);
    strcpy(rt->serverpasswd,pauth_pwd);

    if (strlen(path) <= 0){
        av_log(s, AV_LOG_ERROR,"parm err, path is null).\n");
        return -1;
    }

    ls = strchr(path, '?');
    ret = gb28181_uri_split(ls,localip,localid,localportsip,localportmedia,deviceid,devicetype,begtime,endtime);
    if(ret < 0){
        av_log(s, AV_LOG_ERROR,"gb28181 url is err[%s].\n", s->url);
        return -1;
    }

    if(strlen(localip) > 0){
        strcpy(rt->localip,localip);
    }else if(getLocalIp(localipeth0) >= 0) {
        strcpy(rt->localip,localipeth0);
    }else{
        av_log(s, AV_LOG_ERROR,"parm localip err[%s]).\n",localip);
        return -1;
    }

    if(strlen(localid) != 20){
        unsigned short tick = ff_gb28181_gettickcurmill();
        sprintf(localid,"1000000000400%07d",tick);
    }
    strcpy(rt->localid, localid);

    if(strlen(deviceid) != 20){
        av_log(s, AV_LOG_ERROR,"parm deviceid err[%s]).\n",deviceid);
        return -1;
    }
    strcpy(rt->deviceid,deviceid);

    rt->localportsip = atoi(localportsip);
#if 0
    if(rt->localportsip < 1000){
        av_log(s, AV_LOG_ERROR,"parm port err[%d]).\n",rt->localportsip);
        return -1;
    }
#endif

    if(rt->transport_rtp_mask == GB28181_TRANSPORT_UDP){
        rt->localportrtp = atoi(localportmedia);
        rt->localportrtcp = rt->localportrtp+1;
        if((rt->localportrtp < 1000)){
            av_log(s, AV_LOG_ERROR,"parm port err[%d,%d]).\n",rt->localportrtp,rt->localportrtcp);
            return -1;
        }
    }

    if(!strcmp(proto,"gb28181_playback")){
        if((strlen(begtime) != 14) || (strlen(endtime) != 14)) {
            av_log(s, AV_LOG_ERROR,"parm begtime or endtime err[%ld,%ld]).\n",
                   strlen(begtime), strlen(endtime));
            return -1;
        }
        sprintf(rt->begtime, "%ld", convert_str_to_tm(begtime));
        sprintf(rt->endtime, "%ld", convert_str_to_tm(endtime));

        if(strlen(devicetype) >= 1){
            strcpy(rt->devicetype,devicetype);
        }else{
            av_log(s, AV_LOG_ERROR,"parm devicetype err[%s]).\n",devicetype);
            return -1;
        }
    }

    rt->expires   = 3600;
    rt->iheartbeat = 60;
    return 0;
}

