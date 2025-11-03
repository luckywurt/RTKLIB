#ifndef WIN32
#define _POSIX_C_SOURCE 2
#endif
#include <stdio.h>
#include <stdlib.h>
#include <stdarg.h>
#include <ctype.h>
#ifdef WIN32
#include <winsock2.h>
#else
#include <unistd.h>
#include <fcntl.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/time.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <arpa/inet.h>
#include <netdb.h>
#include <errno.h>
#include <termios.h>
#endif
#include "vt.h"

#define DEF_DEV     "/dev/tty"          /* 默认控制台设备 */

#define C_DEL       (char)0x7F          /* 删除 */
#define C_ESC       (char)0x1B          /* 退出 */
#define C_CTRC      (char)0x03          /* 中断 (ctrl-c) */
#define C_CTRD      (char)0x04          /* 注销 (ctrl-d) */
#define C_ECHO      (char)1             /* telnet 回显 */
#define C_SUPPGA    (char)3             /* telnet 禁止继续 */
#define C_BRK       (char)243           /* telnet 中断 */
#define C_IP        (char)244           /* telnet 中断 */
#define C_EC        (char)247           /* telnet 擦除字符 */
#define C_EL        (char)248           /* telnet 擦除行 */
#define C_WILL      (char)251           /* telnet 选项协商 */
#define C_WONT      (char)252           /* telnet 选项协商 */
#define C_DO        (char)253           /* telnet 选项协商 */
#define C_DONT      (char)254           /* telnet 选项协商 */
#define C_IAC       (char)255           /* telnet 解释为命令 */

/* 打开虚拟控制台 --------------------------------------------------------
* 打开虚拟控制台
* 参数   : vt_t   *vt       I   虚拟控制台
*          int    sock      I   socket (0:使用设备)
*          char   *dev      I   设备 ("": 标准 tty)
* 返回值 : 虚拟控制台 (NULL: 错误)
*-----------------------------------------------------------------------------*/
extern vt_t *vt_open(int sock, const char *dev)
{
    const char mode[]={C_IAC,C_WILL,C_SUPPGA,C_IAC,C_WILL,C_ECHO};
    struct termios tio={0};
    vt_t *vt;
    int i;
    
    trace(3,"vt_open: sock=%d dev=%s\n",sock,dev);
    
    if (!(vt=(vt_t *)malloc(sizeof(vt_t)))) {
        return NULL;
    }
    vt->type=vt->n=vt->nesc=vt->cur=vt->cur_h=vt->brk=vt->blind=0;
    vt->logfp=NULL;
    for (i=0;i<MAXHIST;i++) {
        vt->hist[i]=NULL;
    }
    if (!sock) {
        if ((vt->in=vt->out=open(*dev?dev:DEF_DEV,O_RDWR))<0) {
            free(vt);
            return 0;
        }
        /* 设置终端模式为关闭回显 */
        tcgetattr(vt->in,&vt->tio);
        tcsetattr(vt->in,TCSANOW,&tio);
    }
    else {
        vt->type=1;
        vt->in=vt->out=sock;
        
        /* 发送 telnet 字符模式 */
        if (write(sock,mode,6)!=6) {
            free(vt);
            return NULL;
        }
    }
    vt->state=1;
    return vt;
}
/* 关闭虚拟控制台 -------------------------------------------------------
* 关闭虚拟控制台
* 参数   : vt_t   *vt       I   虚拟控制台
* 返回值 : 无
*-----------------------------------------------------------------------------*/
extern void vt_close(vt_t *vt)
{
    int i;
    
    trace(3,"vt_close:\n");
    
    /* 恢复终端模式 */
    if (!vt->type) {
        tcsetattr(vt->in,TCSANOW,&vt->tio);
    }
    close(vt->in);
    if (vt->logfp) fclose(vt->logfp);
    for (i=0;i<MAXHIST;i++) {
        free(vt->hist[i]);
    }
    vt->state=0;
    free(vt);
}
/* 清除行缓冲区 ---------------------------------------------------------*/
static int clear_buff(vt_t *vt)
{
    char buff[MAXBUFF*3],*p=buff;
    int i,len=strlen(vt->buff);
    for (i=0;i<vt->cur;i++) *p++='\b';
    for (i=0;i<len;i++) *p++=' ';
    for (i=0;i<len;i++) *p++='\b';
    vt->n=vt->nesc=vt->cur=0;
    return write(vt->out,buff,p-buff)==p-buff;
}
/* 刷新行缓冲区 -------------------------------------------------------*/
static int ref_buff(vt_t *vt)
{
    char buff[MAXBUFF*3],*p=buff;
    int i;
    for (i=vt->cur;i<vt->n;i++) *p++=vt->buff[i];
    *p++=' ';
    for (;i>=vt->cur;i--) *p++='\b';
    return write(vt->out,buff,p-buff)==p-buff;
}
/* 光标右移 ---------------------------------------------------------*/
static int right_cur(vt_t *vt)
{
    if (vt->cur>=vt->n) return 1;
    if (write(vt->out,vt->buff+vt->cur,1)<1) return 0;
    vt->cur++;
    return 1;
}
/* 光标左移 ----------------------------------------------------------*/
static int left_cur(vt_t *vt)
{
    if (vt->cur<=0) return 1;
    vt->cur--;
    return write(vt->out,"\b",1)==1;
}
/* 删除光标前的字符 --------------------------------------------*/
static int del_cur(vt_t *vt)
{
    int i;
    if (vt->cur<=0) return 1;
    for (i=vt->cur;i<vt->n;i++) vt->buff[i-1]=vt->buff[i];
    vt->n--;
    return left_cur(vt)&&ref_buff(vt);
}
/* 在光标后插入字符 ---------------------------------------------*/
static int ins_cur(vt_t *vt, char c)
{
    int i;
    if (vt->n>=MAXBUFF) return 1;
    for (i=vt->n++;i>vt->cur;i--) vt->buff[i]=vt->buff[i-1];
    vt->buff[vt->cur++]=c;
    if (write(vt->out,vt->blind?"*":&c,1)<1) return 0;
    return ref_buff(vt);
}
/* 添加历史记录 ---------------------------------------------------------------*/
static int hist_add(vt_t *vt, const char *buff)
{
    int len=vt->n;
    if (len<=0) return 1;
    free(vt->hist[MAXHIST-1]);
    for (int i=MAXHIST-1;i>0;i--) vt->hist[i]=vt->hist[i-1];
    if (!(vt->hist[0]=(char *)malloc(len+1))) return 0;
    strcpy(vt->hist[0],buff);
    return 1;
}
/* 调用上一条历史记录 -----------------------------------------------------*/
static int hist_prev(vt_t *vt)
{
    char *p;
    if (vt->cur_h>=MAXHIST||!vt->hist[vt->cur_h]) return 1;
    if (!clear_buff(vt)) return 0;
    for (p=vt->hist[vt->cur_h++];*p;p++) if (!ins_cur(vt,*p)) return 0;
    return 1;
}
/* 调用下一条历史记录 ---------------------------------------------------------*/
static int hist_next(vt_t *vt)
{
    char *p;
    if (!clear_buff(vt)) return 0;
    if (vt->cur_h==0||!vt->hist[vt->cur_h-1]) return 1;
    for (p=vt->hist[--vt->cur_h];*p;p++) if (!ins_cur(vt,*p)) return 0;
    return 1;
}
/* 处理 telnet 序列 ----------------------------------------------------*/
static int seq_telnet(vt_t *vt)
{
    char msg[3]={C_IAC};
    
    if (vt->esc[1]==C_WILL) { /* 选项协商 */
        if (vt->nesc<3) return 1;
        msg[1]=vt->esc[2]==C_ECHO||vt->esc[2]==C_SUPPGA?C_DO:C_DONT;
        msg[2]=vt->esc[2];
        if (write(vt->out,msg,3)<3) return 0;
    }
    else if (vt->esc[1]==C_DO) { /* 选项协商 */
        if (vt->nesc<3) return 1;
        msg[1]=vt->esc[2]==C_ECHO||vt->esc[2]==C_SUPPGA?C_WILL:C_WONT;
        msg[2]=vt->esc[2];
        if (write(vt->out,msg,3)<3) return 0;
    }
    else if (vt->esc[1]==C_WONT||vt->esc[1]==C_DONT) { /* 选项协商 */
        if (vt->nesc<3) return 1;
        msg[1]=vt->esc[1]==C_WONT?C_DONT:C_WONT;
        msg[2]=vt->esc[2];
        if (write(vt->out,msg,3)<3) return 0;
    }
    else if (vt->esc[1]==C_BRK||vt->esc[1]==C_IP) { /* 中断或中断 */
        vt->brk=1;
    }
    else if (vt->esc[1]==C_EC) { /* 擦除字符 */
        del_cur(vt);
    }
    else if (vt->esc[1]==C_EL) { /* 擦除行 */
        clear_buff(vt);
    }
    vt->nesc=0;
    return 1;
}
/* 处理转义序列 ----------------------------------------------------*/
static int seq_esc(vt_t *vt)
{
    if (vt->nesc<3) return 1;
    vt->nesc=0;
    if (vt->esc[1]=='['||vt->esc[1]=='O') {
        if (vt->esc[2]=='A') return hist_prev(vt); /* 光标上移    */
        if (vt->esc[2]=='B') return hist_next(vt); /* 光标下移  */
        if (vt->esc[2]=='C') return right_cur(vt); /* 光标右移 */
        if (vt->esc[2]=='D') return left_cur (vt); /* 光标左移  */
    }
    return 1;
}
/* 从控制台获取字符 --------------------------------------------------
* 从虚拟控制台获取字符（带超时）
* 参数   : vt_t   *vt       I   虚拟控制台
*          char   *c        O   字符
* 返回值 : 状态 (1:成功,0:错误)
* 注释  : 如果无输入，返回成功且 *c='\0'
*-----------------------------------------------------------------------------*/
extern int vt_getc(vt_t *vt, char *c)
{
    struct timeval tv={0,1000}; /* 超时 (微秒) */
    fd_set rs;
    int stat;
    
    *c='\0';
    
    if (!vt||!vt->state) return 0;
    
    /* 带超时读取字符 */
    FD_ZERO(&rs);
    FD_SET(vt->in,&rs);
    if (!(stat=select(vt->in+1,&rs,NULL,NULL,&tv))) return 1; /* 无数据 */
    if (stat<0||read(vt->in,c,1)!=1) return 0; /* 错误 */
    
    if ((vt->type&&*c==C_IAC)||*c==C_ESC) { /* 转义或 telnet */
        vt->esc[0]=*c; *c='\0';
        vt->nesc=1;
    }
    else if (vt->nesc>0&&vt->esc[0]==C_IAC) { /* telnet 序列 */
        vt->esc[vt->nesc++]=*c; *c='\0';
        if (!seq_telnet(vt)) return 0;
    }
    else if (vt->nesc>0&&vt->esc[0]==C_ESC) { /* 转义序列 */
        vt->esc[vt->nesc++]=*c; *c='\0';
        if (!seq_esc(vt)) return 0;
    }
    else if (*c=='\b'||*c==C_DEL) { /* 退格或删除 */
        if (!del_cur(vt)) return 0;
    }
    else if (*c==C_CTRC) { /* 中断 (ctrl-c) */
        vt->brk=1;
        if (!vt_puts(vt,"^C\n")) return 0;
    }
    else if (isprint(*c)) { /* 可打印字符 */
        if (!ins_cur(vt,*c)) return 0;
    }
    return 1;
}
/* 从控制台获取行 -------------------------------------------------------
* 从虚拟控制台获取一行
* 参数   : vt_t   *vt       I   虚拟控制台
*          char   *buff     O   缓冲区
*          in     n         I   缓冲区大小
* 返回值 : 状态 (1:成功,0:无输入)
*-----------------------------------------------------------------------------*/
extern int vt_gets(vt_t *vt, char *buff, int n)
{
    char c;
    
    buff[0]='\0';
    
    if (!vt||!vt->state) return 0;
    
    vt->n=vt->cur=vt->nesc=vt->brk=0;
    
    while (vt->state) {
        if (!vt_getc(vt,&c)) return 0;
        
        if (c==C_CTRD&&vt->n==0) { /* 注销 */
            vt->state=0;
        }
        else if (vt->brk) { /* 中断 */
            return vt_puts(vt,"\n");
        }
        else if (c=='\r') { /* 行结束 */
            vt->buff[vt->n]='\0';
            strncpy(buff,vt->buff,n-1);
            buff[n-1]='\0';
            if (!vt->blind) hist_add(vt,buff);
            return vt_putc(vt,'\n');
        }
    }
    return 0;
}
/* 向控制台输出字符 --------------------------------------------------*/
static int vt_putchar(vt_t *vt, char c)
{
    if (!vt||!vt->state) return 0;
    if (vt->logfp) fwrite(&c,1,1,vt->logfp);
    return write(vt->out,&c,1)==1;
}
/* 向控制台输出字符 ----------------------------------------------------
* 向虚拟控制台输出一个字符
* 参数   : vt_t   *vt       I   虚拟控制台
*          char   c         I   字符
* 返回值 : 状态 (1:成功,0:错误)
*-----------------------------------------------------------------------------*/
extern int vt_putc(vt_t *vt, char c)
{
    if (c=='\n'&&!vt_putchar(vt,'\r')) return 0;
    return vt_putchar(vt,c);
}
/* 向控制台输出字符串 ------------------------------------------------------
* 向虚拟控制台输出字符串
* 参数   : vt_t   *vt       I   虚拟控制台
*          char   *buff     I   字符串
* 返回值 : 状态 (1:成功,0:错误)
*-----------------------------------------------------------------------------*/
extern int vt_puts(vt_t *vt, const char *buff)
{
    const char *p;
    for (p=buff;*p;p++) if (!vt_putc(vt,*p)) return 0;
    return 1;
}
/* 格式化向控制台输出 ------------------------------------------------------------
* 格式化向虚拟控制台输出
* 参数   : vt_t   *vt       I   虚拟控制台
*          char   *format   I   格式 (与 sfprintf 相同)
*          ...              I   可变参数
* 返回值 : 状态 (1:成功,0:错误)
*-----------------------------------------------------------------------------*/
extern int vt_printf(vt_t *vt, const char *format, ...)
{
    va_list ap;
    char buff[MAXBUFF+1];
    va_start(ap,format);
    vsprintf(buff,format,ap);
    va_end(ap);
    return vt_puts(vt,buff);
}
/* 检查控制台中断 ------------------------------------------------------
* 检查虚拟控制台中断
* 参数   : vt_t   *vt       I   虚拟控制台
* 返回值 : 状态 (1:中断,0:无中断)
*-----------------------------------------------------------------------------*/
extern int vt_chkbrk(vt_t *vt)
{
    char c;
    vt->brk=0;
    return !vt_getc(vt,&c)||vt->brk;
}
/* 打开控制台日志 ------------------------------------------------------------
* 为虚拟控制台打开控制台日志
* 参数   : vt_t   *vt       I   虚拟控制台
*          char   *file     I   日志文件路径
* 返回值 : 状态 (1:成功,0:错误)
*-----------------------------------------------------------------------------*/
extern int vt_openlog(vt_t *vt, const char *file)
{
    if (!vt||!vt->state||!(vt->logfp=fopen(file,"w"))) return 0;
    return 1;
}
/* 关闭控制台日志 -----------------------------------------------------------
* 为虚拟控制台关闭控制台日志
* 参数   : vt_t   *vt       I   虚拟控制台
* 返回值 : 无
*-----------------------------------------------------------------------------*/
extern void vt_closelog(vt_t *vt)
{
    if (!vt||!vt->state||!vt->logfp) return;
    fclose(vt->logfp);
    vt->logfp=NULL;
}