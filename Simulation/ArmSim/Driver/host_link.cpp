#include "host_link.h"

#include <stdio.h>
#include <string.h>

#ifdef _WIN32

bool HostLink_Open(HostLink_t *link, const char *) { memset(link, 0, sizeof(*link)); link->fd = -1; return false; }
int HostLink_Read(HostLink_t *, uint8_t *, int) { return 0; }
int HostLink_Write(HostLink_t *, const uint8_t *, int) { return 0; }
void HostLink_Close(HostLink_t *) {}

#else

#include <fcntl.h>
#include <stdlib.h>
#include <termios.h>
#include <unistd.h>

bool HostLink_Open(HostLink_t *link, const char *link_path)
{
    struct termios tio;

    memset(link, 0, sizeof(*link));
    link->fd = posix_openpt(O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (link->fd < 0 || grantpt(link->fd) != 0 || unlockpt(link->fd) != 0 ||
        ptsname_r(link->fd, link->path, sizeof(link->path)) != 0)
    {
        HostLink_Close(link);
        return false;
    }

    /* 原始模式：不回显、不做行编辑，二进制帧原样透传 */
    if (tcgetattr(link->fd, &tio) == 0)
    {
        cfmakeraw(&tio);
        tcsetattr(link->fd, TCSANOW, &tio);
    }

    if (link_path != NULL)
    {
        unlink(link_path);
        if (symlink(link->path, link_path) == 0)
        {
            snprintf(link->link_path, sizeof(link->link_path), "%s", link_path);
        }
    }
    return true;
}

int HostLink_Read(HostLink_t *link, uint8_t *buf, int max)
{
    /* 上位机还没打开 slave 或已关闭时 read 返回 EIO，等同于没有数据 */
    const ssize_t n = link->fd >= 0 ? read(link->fd, buf, (size_t)max) : -1;

    return n > 0 ? (int)n : 0;
}

int HostLink_Write(HostLink_t *link, const uint8_t *data, int length)
{
    const ssize_t n = link->fd >= 0 ? write(link->fd, data, (size_t)length) : -1;

    return n > 0 ? (int)n : 0;
}

void HostLink_Close(HostLink_t *link)
{
    if (link->link_path[0] != '\0')
    {
        unlink(link->link_path);
        link->link_path[0] = '\0';
    }
    if (link->fd >= 0)
    {
        close(link->fd);
        link->fd = -1;
    }
}

#endif
