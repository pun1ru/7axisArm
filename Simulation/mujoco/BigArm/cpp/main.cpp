// Minimal MuJoCo C++ harness for the BigArm model.
//
//   ./bigarm_ctrl robot.xml [seconds]
//
// The physics / IO plumbing below stays fixed.  To run your own algorithm,
// edit your update() implementation (see PdGravityController for an example).
#include <mujoco/mujoco.h>

#include <array>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <string>
#include <thread>
#include <vector>

constexpr int kN = 8;              // joint1 .. joint8 (arm; gripper not included)

// Everything the controller is allowed to read.
struct RobotState {
  std::array<double, kN> q{};      // joint angle [rad]
  std::array<double, kN> dq{};     // angular velocity [rad/s]
  std::array<double, kN> ddq{};    // angular acceleration [rad/s^2]
  std::array<double, kN> bias{};   // C(q,dq) + g(q), i.e. torque needed to hold still
};

// ---------------------------------------------------------------------------
// Your algorithm goes here.  Return joint torque [N·m] for joint1..joint8.
// ---------------------------------------------------------------------------
class Controller {
 public:
  virtual ~Controller() = default;
  virtual void reset(const RobotState&) {}
  virtual std::array<double, kN> update(const RobotState& state, double t) = 0;
};

// Example: PD on a reference pose + MuJoCo's gravity/Coriolis feedforward.
//
// With <motor> actuators the ctrl is a raw torque integrated explicitly, so
// kp/kd must satisfy the 2 ms timestep.  kp = I*wn^2, kd = 2*zeta*I*wn with
// wn = 10 rad/s is stable for this model (wn above ~20 rad/s diverges).
class PdGravityController : public Controller {
 public:
  PdGravityController(std::array<double, kN> kp, std::array<double, kN> kd)
      : kp_(kp), kd_(kd) {}

  void reset(const RobotState& state) override { q_ref_ = state.q; }

  void setTarget(const std::array<double, kN>& q_ref) { q_ref_ = q_ref; }

  std::array<double, kN> update(const RobotState& s, double /*t*/) override {
    std::array<double, kN> tau{};
    for (int i = 0; i < kN; ++i) {
      tau[i] = kp_[i] * (q_ref_[i] - s.q[i]) - kd_[i] * s.dq[i] + s.bias[i];
    }
    return tau;
  }

 private:
  std::array<double, kN> q_ref_{};
  const std::array<double, kN> kp_;
  const std::array<double, kN> kd_;
};

// ---------------------------------------------------------------------------
int main(int argc, char** argv) {
  const char* xml = argc > 1 ? argv[1] : "robot.xml";
  const double seconds = argc > 2 ? std::atof(argv[2]) : 5.0;

  char err[1000] = {};
  mjModel* m = mj_loadXML(xml, nullptr, err, sizeof(err));
  if (!m) {
    std::fprintf(stderr, "load %s failed:\n%s\n", xml, err);
    return 1;
  }
  mjData* d = mj_makeData(m);

  // Resolve joints / actuators by name once, so renaming in the XML does not
  // silently break index-based code.
  std::array<int, kN> qadr{}, vadr{}, act{};
  for (int i = 0; i < kN; ++i) {
    const std::string jn = "joint" + std::to_string(i + 1);
    const std::string an = "act_" + jn;
    const int jid = mj_name2id(m, mjOBJ_JOINT, jn.c_str());
    act[i] = mj_name2id(m, mjOBJ_ACTUATOR, an.c_str());
    if (jid < 0 || act[i] < 0) {
      std::fprintf(stderr, "missing %s / %s in %s\n", jn.c_str(), an.c_str(), xml);
      mj_deleteData(d);
      mj_deleteModel(m);
      return 1;
    }
    qadr[i] = m->jnt_qposadr[jid];
    vadr[i] = m->jnt_dofadr[jid];
  }

  mj_forward(m, d);              // fill qacc / qfrc_bias at qpos0

  // PD gains from the model's diagonal inertia at the initial pose.
  //   kp = I * wn^2,   kd = 2 * zeta * I * wn
  // A light distal link then gets a small gain, which is exactly what keeps
  // the explicit torque loop stable.  Recompute after changing qpos0.
  std::vector<mjtNum> M(static_cast<size_t>(m->nv) * m->nv);
  mj_fullM(m, d, M.data());
  constexpr double kOmega = 10.0;    // closed-loop bandwidth [rad/s]
  constexpr double kZeta = 1.0;      // 1.0 = critically damped
  std::array<double, kN> kp{}, kd{};
  for (int i = 0; i < kN; ++i) {
    const double inertia = M[static_cast<size_t>(vadr[i]) * m->nv + vadr[i]];
    kp[i] = inertia * kOmega * kOmega;
    kd[i] = 2.0 * kZeta * inertia * kOmega;
  }

  PdGravityController controller(kp, kd);   // <-- replace with your own class
  RobotState state;
  for (int i = 0; i < kN; ++i) state.q[i] = d->qpos[qadr[i]];
  controller.reset(state);

  const double dt = m->opt.timestep;
  const auto wall_start = std::chrono::steady_clock::now();
  double sim_t = 0.0;
  double next_print = 0.0;

  while (sim_t < seconds) {
    // ---- 1. read state ---------------------------------------------------
    // qpos/qvel are the *current* state.  The jointpos/jointvel sensors hold
    // the values sampled by mj_step before it integrated, i.e. they are one
    // timestep behind; feeding those back adds a full step of delay and made
    // the same PD loop unstable.  qacc/qfrc_bias below are also from the
    // start of the last step; call mj_forward(m, d) first if you need them
    // aligned with qpos/qvel.
    for (int i = 0; i < kN; ++i) {
      state.q[i] = d->qpos[qadr[i]];
      state.dq[i] = d->qvel[vadr[i]];
      state.ddq[i] = d->qacc[vadr[i]];
      state.bias[i] = d->qfrc_bias[vadr[i]];
    }

    // ---- 2. your algorithm ----------------------------------------------
    const std::array<double, kN> tau = controller.update(state, sim_t);

    // ---- 3. apply torque and advance physics ----------------------------
    for (int i = 0; i < kN; ++i) {
      double u = tau[i];
      if (m->actuator_ctrllimited[act[i]]) {   // ±100 N·m in this model
        u = std::fmax(m->actuator_ctrlrange[2 * act[i]],
                      std::fmin(u, m->actuator_ctrlrange[2 * act[i] + 1]));
      }
      d->ctrl[act[i]] = u;
    }
    mj_step(m, d);
    sim_t += dt;

    // ---- 4. real-time pacing --------------------------------------------
    const auto target = wall_start + std::chrono::duration_cast<
                                           std::chrono::steady_clock::duration>(
                                           std::chrono::duration<double>(sim_t));
    std::this_thread::sleep_until(target);

    if (sim_t >= next_print) {
      std::printf("t=%5.2f  q1=%7.4f q2=%7.4f q3=%7.4f q4=%7.4f"
                  "  tau1=%8.3f tau2=%8.3f\n",
                  sim_t, state.q[0], state.q[1], state.q[2], state.q[3],
                  tau[0], tau[1]);
      next_print += 0.5;
    }
  }

  mj_deleteData(d);
  mj_deleteModel(m);
  return 0;
}
