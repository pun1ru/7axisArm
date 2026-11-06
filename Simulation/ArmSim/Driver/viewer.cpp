#include "viewer.h"

#include <simulate/glfw_adapter.h>
#include <simulate/simulate.h>

#include <cstdlib>
#include <exception>
#include <stdexcept>
#include <thread>

struct Viewer::Impl {
    MujocoWorld *world = nullptr;
    mjvCamera cam;
    mjvOption opt;
    mjvPerturb pert;
    std::unique_ptr<mujoco::Simulate> sim;   // 必须在 cam/opt/pert 之后销毁
};

Viewer::Viewer(MujocoWorld &world) : impl_(new Impl)
{
#ifdef __linux__
    /* 官方适配器创建窗口失败时直接 mju_error 退出进程，所以先看有没有显示环境 */
    if (std::getenv("DISPLAY") == nullptr && std::getenv("WAYLAND_DISPLAY") == nullptr)
    {
        throw std::runtime_error("没有显示环境（DISPLAY / WAYLAND_DISPLAY 都没设）");
    }
#endif
    Impl &v = *impl_;

    v.world = &world;
    mjv_defaultCamera(&v.cam);
    mjv_defaultOption(&v.opt);
    mjv_defaultPerturb(&v.pert);
    v.sim = std::make_unique<mujoco::Simulate>(std::make_unique<mujoco::GlfwAdapter>(),
                                               &v.cam, &v.opt, &v.pert, /* is_passive = */ true);
}

Viewer::~Viewer() = default;

void Viewer::Run(const std::function<void()> &sim_body)
{
    Impl &v = *impl_;
    std::exception_ptr error;

    std::thread side([&] {
        try
        {
            v.sim->LoadMessage("");
            v.sim->Load(v.world->model(), v.world->data(), "");
            sim_body();
        }
        catch (...)
        {
            error = std::current_exception();
        }
        int expected = 0;   // 通知渲染循环退出；窗口先关的话已经是 2，不用管
        v.sim->exitrequest.compare_exchange_strong(expected, 1);
    });

    v.sim->RenderLoop();   // 阻塞到窗口关闭或 exitrequest
    side.join();
    if (error)
    {
        std::rethrow_exception(error);
    }
}

std::unique_lock<std::recursive_mutex> Viewer::Lock()
{
    return std::unique_lock<std::recursive_mutex>(impl_->sim->mtx);
}

void Viewer::Sync()
{
    impl_->sim->Sync();
}

bool Viewer::closed() const
{
    return impl_->sim->exitrequest.load() != 0;
}
