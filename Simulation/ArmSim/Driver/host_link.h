#ifndef HOST_LINK_H
#define HOST_LINK_H

#include <stdbool.h>
#include <stdint.h>

/* Driver 层：上位机物理链路。
 * 实机是 UART10（921600），仿真里换成一对伪终端 pty：仿真持有 master，
 * 上位机像打开普通串口一样打开 slave（路径见 path，另有一个固定名字的软链接）。
 * 读写都是非阻塞的，没人连接时发出的数据会被丢弃。仅支持 Linux / macOS。 */
typedef struct {
    int fd;
    char path[64];        /* slave 设备，如 /dev/pts/5 */
    char link_path[64];   /* 固定名字的软链接，退出时删除 */
} HostLink_t;

bool HostLink_Open(HostLink_t *link, const char *link_path);
/* 返回读到的字节数，没有数据返回 0 */
int HostLink_Read(HostLink_t *link, uint8_t *buf, int max);
/* 返回写出的字节数，缓冲满则丢弃并返回 0 */
int HostLink_Write(HostLink_t *link, const uint8_t *data, int length);
void HostLink_Close(HostLink_t *link);

#endif
