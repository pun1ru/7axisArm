#pragma once

#include <functional>
#include <memory>
#include <mutex>

#include "mujoco_world.h"

/* MuJoCo 官方 simulate 查看器（passive 模式）：窗口、视角操作、左右两侧 UI 面板
 * 都是 MuJoCo 自带的，和 `python -m mujoco.viewer` / `simulate` 一致。
 * 物理由 ArmSim 自己推进，查看器只显示拷贝出来的状态。
 *
 * 线程：窗口必须占主线程（macOS 的要求），所以仿真循环放到副线程。
 *   主线程   viewer.Run(sim_body)   阻塞到窗口关闭
 *   副线程   sim_body()             仿真循环，每步（或每隔几步）调 Sync()
 * 左侧面板里的 Pause/Reset/Keyframe/Speed 属于 simulate 自己的物理循环，passive 模式下不起作用；
 * 视角、渲染选项、Watch、鼠标拖拽加外力等都可用。 */
class Viewer {
public:
    /* 没有显示环境时抛 std::runtime_error（官方适配器遇到打不开窗口会直接退出进程，所以先检查） */
    explicit Viewer(MujocoWorld &world);
    ~Viewer();
    Viewer(const Viewer &) = delete;
    Viewer &operator=(const Viewer &) = delete;

    /* 在副线程里跑 sim_body，主线程渲染，直到窗口关闭或 sim_body 返回。
     * sim_body 抛的异常会在这里重新抛出 */
    void Run(const std::function<void()> &sim_body);

    /* 以下在 sim_body 里调用 */
    std::unique_lock<std::recursive_mutex> Lock();   // 推进物理时持有，渲染线程会等
    void Sync();                                      // 把状态拷给查看器，并施加鼠标拖拽的外力
    bool closed() const;                              // 窗口已关闭

private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};
