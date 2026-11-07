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
    bool camera_initialized = false;
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
    /* RMUC2027 的视觉网格约 350 万三角形。降低查看器刷新和同步频率，
     * 避免渲染线程长期占住 passive viewer 锁，导致物理线程与 UI 都卡顿。 */
    v.sim->refresh_rate = 30;
    v.sim->vsync = 0;
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
            v.sim->Load(v.world->view_model(), v.world->view_data(), "");
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
    /* 模型在运行期间不变，只需要同步积分状态（qpos/qvel 等），
     * 不必每个同步周期重新复制完整 mjModel。 */
    impl_->sim->Sync(true);
    if (!impl_->camera_initialized)
    {
        /* 首帧不要把 28 x 16 m 的场地全部塞进视锥。先看机械臂附近，
         * 用户仍可在窗口中自行缩放查看全场。 */
        const std::string root_name = impl_->world->model_prefix() + "link1";
        const int body = mj_name2id(impl_->world->view_model(), mjOBJ_BODY, root_name.c_str());
        if (body >= 0)
        {
            const mjtNum *position = impl_->world->view_data()->xpos + 3 * body;
            impl_->cam.type = mjCAMERA_FREE;
            impl_->cam.lookat[0] = position[0];
            impl_->cam.lookat[1] = position[1];
            impl_->cam.lookat[2] = position[2] + 0.6;
            impl_->cam.distance = 3.0;
            impl_->cam.azimuth = 135.0;
            impl_->cam.elevation = -25.0;
        }
        impl_->camera_initialized = true;
    }
}

bool Viewer::closed() const
{
    return impl_->sim->exitrequest.load() != 0;
}
