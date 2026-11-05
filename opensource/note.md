![alt text](image.png)

* 关节冗余自由度辨识
```
输入：6×7 的 J（7 个关节，6 维任务）
for 每个关节 i：
    M ← 删掉第 i 列后的 6×6 矩阵
    if det(M) ≡ 0：        // 对任意位形都为零
        关节 i 是非冗余关节
    else：
        关节 i 是候选冗余关节

```
![alt text](image-1.png)