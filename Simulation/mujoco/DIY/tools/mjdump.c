/* mjdump.c — 用 libmujoco 编译 robot.xml, 导出 MuJoCo 实际使用的质量/质心/惯量.
 *
 * 用途: 与 URDF 逐 body 对账 (tools/urdf_check.py --dump). 编译后的 body_inertia
 * 是主惯量, 配合 body_iquat 可还原体质心系张量, 用来独立验证 fullinertia 的
 * 分量顺序没有解析错.
 *
 * 编译: cc -O2 -o mjdump mjdump.c -I$MUJOCO/include -L$MUJOCO/lib -lmujoco \
 *          -Wl,-rpath,$MUJOCO/lib
 * 用法: ./mjdump path/to/robot.xml
 *
 * 输出: 每个 body 一行
 *   <name> <mass> <ipos xyz> I <principal xyz> Q <iquat wxyz> X <xipos xyz> R <xmat 9>
 *   xmat 为行主序 3x3.
 */
#include <stdio.h>
#include <mujoco/mujoco.h>

int main(int argc, char** argv) {
  if (argc < 2) {
    fprintf(stderr, "usage: %s model.xml\n", argv[0]);
    return 2;
  }
  char err[2000] = "";
  mjModel* m = mj_loadXML(argv[1], NULL, err, sizeof(err));
  if (!m) {
    fprintf(stderr, "编译失败: %s\n", err);
    return 1;
  }
  mjData* d = mj_makeData(m);
  mj_forward(m, d);

  for (int i = 1; i < m->nbody; i++) {
    const char* name = mj_id2name(m, mjOBJ_BODY, i);
    printf("%s %.12g", name ? name : "?", m->body_mass[i]);
    for (int k = 0; k < 3; k++) printf(" %.12g", m->body_ipos[3 * i + k]);
    printf(" I");
    for (int k = 0; k < 3; k++) printf(" %.12g", m->body_inertia[3 * i + k]);
    printf(" Q");
    for (int k = 0; k < 4; k++) printf(" %.12g", m->body_iquat[4 * i + k]);
    printf(" X");
    for (int k = 0; k < 3; k++) printf(" %.12g", d->xipos[3 * i + k]);
    printf(" R");
    for (int k = 0; k < 9; k++) printf(" %.12g", d->xmat[9 * i + k]);
    printf("\n");
  }

  mj_deleteData(d);
  mj_deleteModel(m);
  return 0;
}
