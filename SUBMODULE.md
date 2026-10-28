# 子模块操作说明

本文说明这个仓库的子模块结构、日常操作和推送顺序。不按顺序推送时踩到的坑,文末有恢复方法。

## 仓库结构

```
7axisArm                    github.com/pun1ru/7axisArm          分支 main
└── Real/DIYcontroller       github.com/pun1ru/dm_mcV610        分支 7axisarm
    └── LibSE                github.com/pun1ru/LibSE            默认分支 main
```

LibSE 是嵌套在 DIYcontroller 里面的,不能直接从主仓库更新。

## 父仓库记录的是什么

父仓库里子模块那一项只是一个 **40 位提交哈希**,不是分支,也不是"远程上的某个提交"。

- 子模块目录平时处于**游离 HEAD** 状态,`git status` 显示 `HEAD（非分支）`,这是正常的。
- 主仓库的 `.gitmodules` 里 `branch = 7axisarm` 只在执行 `git submodule update --remote` 时才用,不影响平时。
- 父仓库推送时,git **不会**检查这个哈希在子模块远程上是否存在。

## 推送顺序:先子模块,后父仓库

```bash
# 1. 先推子模块
cd Real/DIYcontroller
git switch -c 7axisarm          # 第一次需要建分支;已有就 git switch 7axisarm
git push -u origin 7axisarm

# 2. 再推父仓库
cd ../..
git add Real/DIYcontroller
git commit -m "update DIYcontroller"
git push
```

LibSE 有改动时同理,先推 LibSE,再在 DIYcontroller 里提交指针,最后才推主仓库。

**为什么不能反过来**

先在父仓库提交并推送新的子模块指针,别人拉取后执行 `git submodule update --init --recursive` 时,git 会去子模块远程请求那个哈希。拿不到就失败:

```
fatal: 远程错误：upload-pack: not our ref ffe46a60...
fatal: 获取了子模组路径 'Real/DIYcontroller/LibSE'，但是它没有包含 ffe46a60...
fatal: 无法递归进入子模组路径 'Real/DIYcontroller'
```

这不是丢数据:子模块的提交还在提交者的本地。但补推之前,所有拉取这个主仓库的人都卡在子模块初始化,CI 每次都会失败。

## 让 git 自动检查顺序

在两层的仓库里各设一次,只写进本机的 `.git/config`,不提交、不影响别人:

```bash
# 在仓库根目录执行
git -C Real/DIYcontroller config push.recurseSubmodules check
git config push.recurseSubmodules check
```

- `check`:推送前检查被引用的子模块提交是否已在远程,不在就拒绝推送。
- 想让它自动先推子模块,把 `check` 换成 `on-demand`。
- 两层各管一层:DIYcontroller 的那项保证 LibSE 先推,主仓库的那项保证 DIYcontroller 先推。

确认是否设上:

```bash
git -C Real/DIYcontroller config --get push.recurseSubmodules
git config --get push.recurseSubmodules
```

也可以只在单次推送时用,不改配置:

```bash
git push --recurse-submodules=check
```

## 日常操作

**新克隆仓库**

```bash
git clone --recurse-submodules https://github.com/pun1ru/7axisArm.git
```

已经克隆过的:

```bash
git submodule update --init --recursive
```

**拉取别人的更新**

```bash
git pull --ff-only
git submodule update --init --recursive
```

`git pull` 只更新子模块指针,不会自动把工作目录里的子模块切到新提交,所以后面那条不能省。

**只使用、不修改子模块**

到此为止就够了。子模块停在游离 HEAD 上不影响编译。

**要改子模块代码**

游离 HEAD 上直接提交,一旦切走就成悬空提交。先确认当前提交是否已经在远程上:

```bash
cd Real/DIYcontroller
git log --oneline origin/7axisarm..HEAD     # 有输出说明还没推
```

有输出就先按上面的顺序推上去,然后切回分支:

```bash
git switch 7axisarm
```

现在没有未推送的提交,切过去不会丢东西。

## LibSE 要不要设置跟踪分支

现在不需要。LibSE 的 `.gitmodules` 条目没有 `branch`,执行 `git submodule update --remote` 时默认用远程的默认分支,而 LibSE 的远程默认分支就是 `main`,效果和写 `branch = main` 一样。

只有想在配置里明确看出"跟哪个分支"时,才在 `Real/DIYcontroller/.gitmodules` 的 LibSE 段加上:

```
branch = main
```

加不加都不影响指针是否有效。指针是否有效,只取决于那个提交有没有推到远程。

## 出问题怎么恢复

看到 `not our ref` / `没有包含 <哈希>`:

1. 找到那个提交在谁的本机:`git branch --contains <哈希>`。
2. 在那台机器上把它推到远程:
   ```bash
   cd Real/DIYcontroller/LibSE
   git push origin HEAD:main      # 或它所在的分支
   ```
3. 这边重新执行:
   ```bash
   git submodule update --init --recursive
   ```

如果那个提交已经不在了(refork、reset 过),就让 DIYcontroller 重新指向 LibSE 上真实存在的提交,再提交指针并推送。

临时跳过子模块、先能用主仓库:

```bash
git -c submodule.Real/DIYcontroller.update=none submodule update --init
```

## 检查自己有没有制造悬空指针

推送前确认子模块没有未推送的提交:

```bash
git -C Real/DIYcontroller status -sb
git -C Real/DIYcontroller log --oneline origin/7axisarm..HEAD
```

第二条有输出,说明有提交还没推到远程,先推它。`LibSE` 同理,把路径换成 `Real/DIYcontroller/LibSE`,远程分支换成 `origin/main`。
