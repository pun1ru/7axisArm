#include "host_link.h"

#include <stdexcept>

#ifdef _WIN32

HostLink::HostLink(const std::string &) { throw std::runtime_error("host link needs pty (Linux / macOS only)"); }
HostLink::~HostLink() = default;
int HostLink::Read(uint8_t *, size_t) { return 0; }
int HostLink::Write(const uint8_t *, size_t) { return 0; }

#else

#include <fcntl.h>
#include <stdlib.h>
#include <termios.h>
#include <unistd.h>

HostLink::HostLink(const std::string &link_path)
{
    char name[64];
    struct termios tio;

    fd_ = posix_openpt(O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (fd_ < 0 || grantpt(fd_) != 0 || unlockpt(fd_) != 0 || ptsname_r(fd_, name, sizeof(name)) != 0)
    {
        if (fd_ >= 0)
        {
            close(fd_);
        }
        throw std::runtime_error("open pty failed");
    }
    device_path_ = name;

    /* 原始模式：不回显、不做行编辑，二进制帧原样透传 */
    if (tcgetattr(fd_, &tio) == 0)
    {
        cfmakeraw(&tio);
        tcsetattr(fd_, TCSANOW, &tio);
    }

    if (!link_path.empty())
    {
        unlink(link_path.c_str());
        if (symlink(device_path_.c_str(), link_path.c_str()) == 0)
        {
            link_path_ = link_path;
        }
    }
}

HostLink::~HostLink()
{
    if (!link_path_.empty())
    {
        unlink(link_path_.c_str());
    }
    if (fd_ >= 0)
    {
        close(fd_);
    }
}

int HostLink::Read(uint8_t *buf, size_t max)
{
    /* 上位机还没打开 slave 或已关闭时 read 返回 EIO，等同于没有数据 */
    const ssize_t n = read(fd_, buf, max);

    return n > 0 ? static_cast<int>(n) : 0;
}

int HostLink::Write(const uint8_t *data, size_t length)
{
    const ssize_t n = write(fd_, data, length);

    return n > 0 ? static_cast<int>(n) : 0;
}

#endif
