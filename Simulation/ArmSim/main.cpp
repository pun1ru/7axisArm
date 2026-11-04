#include <csignal>
#include <cstdio>
#include <cstdlib>
#include <cstring>

#include "sim_plant.h"
#include "system.h"
#include "system_define.h"

static void OnSignal(int)
{
    System_Stop();
}

int main(int argc, char **argv)
{
    const char *xml_path = ARMSIM_DEFAULT_XML;
    double seconds = SIM_DEFAULT_DURATION_S;
    bool realtime = true;
    bool host_mode = false;
    bool seconds_given = false;
    int positional = 0;

    for (int i = 1; i < argc; ++i)
    {
        if (strcmp(argv[i], "--fast") == 0)
        {
            realtime = false;
        }
        else if (strcmp(argv[i], "--host") == 0)
        {
            host_mode = true;
        }
        else if (strcmp(argv[i], "-h") == 0 || strcmp(argv[i], "--help") == 0 || positional >= 2)
        {
            printf("用法: %s [模型.xml] [秒数] [--fast] [--host]\n"
                   "  默认模型 %s\n"
                   "  --fast  不按实时节拍 sleep\n"
                   "  --host  开伪串口 %s 等 Host/armhost 连接，由上位机给目标，Ctrl-C 退出\n",
                   argv[0], ARMSIM_DEFAULT_XML, HOST_LINK_PATH);
            return positional >= 2 ? 1 : 0;
        }
        else if (positional++ == 0)
        {
            xml_path = argv[i];
        }
        else
        {
            seconds = atof(argv[i]);
            seconds_given = true;
        }
    }
    if (host_mode && !seconds_given)
    {
        seconds = 1.0e9;   /* 交互用，没指定时长就一直跑 */
    }

    std::signal(SIGINT, OnSignal);
    std::signal(SIGTERM, OnSignal);

    if (!System_Init(xml_path, host_mode))
    {
        fprintf(stderr, "初始化失败: %s\n", SimPlant_LastError());
        System_Deinit();
        return 1;
    }
    System_Run(seconds, realtime);
    System_Deinit();
    return 0;
}
