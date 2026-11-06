#pragma once

#include <cstddef>
#include <cstdint>
#include <string>

/* 上位机物理链路。实机是 UART10（921600），仿真里换成一对伪终端 pty：
 * 仿真持有 master，上位机像打开普通串口一样打开 slave（路径见 device_path，
 * 另有一个固定名字的软链接）。读写都非阻塞，没人连接时发出的数据会被丢弃。
 * 仅支持 Linux / macOS，其他平台构造时抛 std::runtime_error。 */
class HostLink {
public:
    explicit HostLink(const std::string &link_path);
    ~HostLink();
    HostLink(const HostLink &) = delete;
    HostLink &operator=(const HostLink &) = delete;

    /* 返回读到的字节数，没有数据返回 0 */
    int Read(uint8_t *buf, size_t max);
    /* 返回写出的字节数，缓冲满则丢弃并返回 0 */
    int Write(const uint8_t *data, size_t length);

    const std::string &device_path() const { return device_path_; }
    const std::string &link_path() const { return link_path_; }

private:
    int fd_ = -1;
    std::string device_path_;   // slave 设备，如 /dev/pts/5
    std::string link_path_;     // 固定名字的软链接，析构时删除
};
