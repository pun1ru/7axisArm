#include <cstdio>
#include <cstdlib>
#include <cstring>

#include "sim_plant.h"
#include "system.h"
#include "system_define.h"

static void PrintUsage(const char *program)
{
    printf("用法: %s [模型.xml] [秒数] [--fast]\n", program);
    printf("  --xml <path>  指定 MJCF，默认 %s\n", ARMSIM_DEFAULT_XML);
    printf("  --fast        不按实时节拍 sleep，尽快跑完\n");
}

int main(int argc, char **argv)
{
    const char *xml_path = ARMSIM_DEFAULT_XML;
    double seconds = SIM_DEFAULT_DURATION_S;
    bool realtime = true;
    const char *positional[2] = {nullptr, nullptr};
    int positional_count = 0;

    for (int i = 1; i < argc; ++i)
    {
        if (strcmp(argv[i], "--fast") == 0)
        {
            realtime = false;
        }
        else if (strcmp(argv[i], "--xml") == 0 && i + 1 < argc)
        {
            xml_path = argv[++i];
        }
        else if (strcmp(argv[i], "-h") == 0 || strcmp(argv[i], "--help") == 0)
        {
            PrintUsage(argv[0]);
            return 0;
        }
        else if (positional_count < 2)
        {
            positional[positional_count++] = argv[i];
        }
        else
        {
            PrintUsage(argv[0]);
            return 1;
        }
    }

    if (positional_count > 0)
    {
        xml_path = positional[0];
    }
    if (positional_count > 1)
    {
        seconds = atof(positional[1]);
    }

    if (!System_Init(xml_path))
    {
        fprintf(stderr, "初始化失败: %s\n", SimPlant_LastError());
        return 1;
    }

    System_Run(seconds, realtime);
    System_Deinit();
    return 0;
}
