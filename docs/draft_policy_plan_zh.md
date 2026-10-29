# 论文草稿计划（中文）

> 版本：v1（2026-10-27）  
> 主线：受限单阶段 DRL（MDP + PPO）；hybrid / 两阶段作为扩展或强基线。  
> 用途：把 `main.tex` 里的模板占位内容替换成可扩写的中文草稿。本文档不是最终论文，所有未定项用 `[待定]`，所有数值用 `[待填]`。  
> 关联：`main.tex`、`main_zh.md`、`docs/mdp.md`（PHY 仓库）。

---

## 0. 一页纸定位

### 0.1 一句话

在存在移动用户、静态目标和因果信息结构的多无人机毫米波 ISAC 网络中，学习每架无人机的位移与感知功率占比，使有限时域平均的通信—感知目标最大化。

### 0.2 核心叙事

本文不把波束成形当成在线优化变量，而是明确区分两类决策：

- **CSI 之前**：中央控制器决定 UAV 位移 \(\mathbf p_k[t]\) 与感知功率占比 \(\varsigma_k[t]\)；
- **CSI 之后**：固定规则根据瞬时信道生成 RZF 通信波束、等功率分配和感知导向波束。

因此，策略只学习 \(4K\) 维动作，推理时只需一次前向计算。这个受限信息结构是方法设计的一部分，不是实现细节。它的性能损失由凸优化/先知辅助上界来量化。

### 0.3 贡献草案（待文献确认后定稿）

1. **模型与信息结构**：建立多 UAV 毫米波 ISAC 的有限时域模型，显式区分 CSI 前的轨迹/感知功率决策与 CSI 后的固定波束规则；用户关联由大尺度信道确定性导出。
2. **方法**：在受限策略类内把联合轨迹—感知功率问题写成有限时域 MDP，设计不依赖当前 CSI 的观测、\(4K\) 维动作和固定低复杂度波束接口，并用 PPO 学习策略。
3. **评估**：通过数值标定确定工作点，使用强可达基线与先知辅助/松弛上界，量化受限信息结构和固定波束带来的性能差距。

> 暂不使用“first”“optimal”“theoretical guarantee”等措辞，除非文献和证明都准备好。

### 0.4 不声称的内容

- 不声称 PPO 收敛到全局最优或局部最优；
- 不声称受限策略类 \(\mathfrak P_{\rm res}\) 等于完整策略类 \(\mathfrak P\)；
- 不把 SCA/BCD 的解称为“上界”；
- 不在结果出来前写具体增益数字。

---

## 1. 题目、摘要、关键词


### 1.2 摘要草稿

> 本文研究多无人机毫米波通感一体化网络中的联合轨迹与感知功率分配问题。每架无人机在运动学约束和防碰撞约束下飞行，同时服务移动用户并照射静态目标；用户关联由大尺度信道确定，波束成形在瞬时信道已知后由低复杂度规则生成，而轨迹与感知功率必须在信道实现前决定。本文将这一因果决策过程建模为有限时域马尔可夫决策过程，在受限策略类内学习轨迹与感知功率，并采用近端策略优化求解。仿真结果表明，在 `[待填]` 的参数设置下，所提方法相对 `[最强可达基线]` 将目标函数提升 `[待填]`，与 `[上界/性能极限]` 的差距为 `[待填]`。

定稿时按 IEEE 摘要要求去掉数学符号；这里的“马尔可夫决策过程”“近端策略优化”可以保留为文字。

### 1.3 关键词

multi-UAV networks, millimeter-wave ISAC, trajectory design, sensing power allocation, deep reinforcement learning, Cramér–Rao bound.

---

## 2. Introduction

### 2.1 背景段（草稿）

> 低空经济、应急通信和区域监视等场景要求无人机同时具备通信覆盖与目标感知能力。把通信和雷达分置于不同平台会带来频谱、硬件和部署成本；通感一体化（ISAC）让同一发射波形同时承担信息传输和目标照射，是多无人机网络的自然选择。毫米波频段提供大带宽和窄波束，但也使链路对 UAV 位置、用户移动和波束指向更敏感。因此，轨迹、功率和波束的联合设计成为决定系统性能的关键。

第二段收窄到本文的具体问题：

> 现有多无人机 ISAC 研究通常假设中央控制器可以基于完美 CSI 和已知的未来用户轨迹做离线优化，或者只优化通信侧轨迹而把感知当成固定覆盖。实际系统里，轨迹和感知功率必须在当前信道实现之前决定，波束则可以在看到 CSI 之后调整；忽略这个时间顺序，会把不可实现的先知信息算进性能。本文关注这一因果约束下的联合决策。

### 2.2 相关工作（按方法组织，不按时间）

建议分四类，每类一段，最后一段收束到 gap：

1. **基于模型的多 UAV 轨迹/波束优化**：SCA、BCD、凸松弛；性能好，但通常需要未来信息或反复求解，且对 CSI 假设敏感。
2. **DRL 用于 UAV 轨迹/资源分配**：PPO/SAC/DDPG；能处理非凸和在线决策，但很多工作只优化通信速率，感知指标用覆盖或距离代替，缺少 CRB 和因果信息结构。
3. **多 UAV 协同感知与 CRB 优化**：合作回波、几何优化、功率分配；但常固定轨迹，或把波束当成连续优化变量。
4. **有限 CSI / 信息结构下的 ISAC**：鲁棒波束、两阶段决策；但很少与多 UAV 轨迹和感知功率同时出现。

Gap 段：

> 现有工作要么假设离线/完美信息做优化，要么用 DRL 但忽略感知功率的决策时刻和固定波束约束。一个在明确信息结构下可学习、可复现、并能与上界比较的联合轨迹—感知功率框架仍然缺失。

### 2.3 贡献列表（草稿）

- 我们将多 UAV 毫米波 ISAC 的联合轨迹与感知功率问题写成有限时域 MDP，显式区分 CSI 前决策与 CSI 后固定波束规则。
- 我们设计不依赖当前 CSI 的观测与 \(4K\) 维动作，使策略在满足运动/安全约束的同时输出轨迹和感知功率。
- 我们给出标定、强基线和上界的评估方案，量化受限信息结构与固定波束的性能代价。

> 第三条要等实验设计定下来后再写死；如果上界太松，改成“给出强优化基线与消融”。

### 2.4 组织结构

> 第 2 节介绍系统模型；第 3 节给出信息结构、性能指标和问题；第 4 节介绍 MDP、策略与训练；第 5 节报告标定、基线、上界和主要结果；第 6 节总结。

---

## 3. System Model（已有，补一致性）

这一节 `main.tex` 已经写得比较完整，不建议重写，只做一致性检查：

- 时间约定：slot \(t=[t\Delta t,(t+1)\Delta t)\)，\(t=0,\dots,T-1\)，\(\mathbf p[0]\) 给定，\(\mathbf p[T]\) 不存在。
- 用户速度：右端点更新，\(\Delta\mathbf u[t]=\mathbf u[t]-\mathbf u[t-1]=\mathbf v^{\rm u}[t]\Delta t\)（无反射时）。
- 关联：\(k(m)[t]=\arg\max_k \bar\beta_{k,m}[t]\)，是位置的确定性函数，不是动作。
- 功率：\(\varsigma_k\) 定义、\(\operatorname{tr}(\mathbf R_k)=P_{\max}\)、通信/感知功率预算。
- 感知：逐时隙 CRB、\(\Gamma_{\rm cap}\) 截断、\(N_s=BT_s\)。
- 图 1/图 2 的占位图要替换；图注里不要写未定义符号。

需要补的一小段：在进入 Problem Formulation 之前，明确说明“本文的波束成形由固定规则给出，优化变量只剩轨迹和 \(\varsigma\)”，避免读者以为后面还会联合优化波束。

---

## 4. Problem Formulation（已有，改一处措辞）

现有内容：

- 信息结构 \(\mathcal H_t\)、\(\mathcal X^{\rm I}\)、\(\mathcal X^{\rm II}\)；
- 受限策略类 \(\mathfrak P_{\rm res}\) 与固定波束式；
- \(\bar R\)、\(\bar C\)、问题 (opt:main)、C1–C4；
- Challenges。

需要改的地方：

1. 现在正文说“Section V optimizes (opt:main) over \(\mathfrak P_{\rm res}\)”。如果第 5 节是 PPO，应改成“learns a policy within \(\mathfrak P_{\rm res}\)”或“searches \(\mathfrak P_{\rm res}\) approximately”。
2. Challenges 目前是空的。建议列四条：
   - 非凸：CRB 对轨迹/波束非凸，关联离散；
   - 因果：轨迹和 \(\varsigma\) 在 CSI 前决定，不能把当前 \(\mathbf h[t]\) 放进策略输入；
   - 高维：\(T\) 个时隙、\(K\) 架 UAV、连续动作；
   - 结构限制：固定 RZF/等功率/感知方向带来性能上界差距。
3. 明确 \(\mathfrak P_{\rm res}\subset\mathfrak P\)，所以受限类的最优值是全类最优值的下界；这句话为后面的上界比较做铺垫。

---

## 5. Proposed Method（核心草稿）

> 这一节要整节替换 `main.tex` 现在的 “Iterative Update / Convergence and Complexity” 模板。现在的模板是凸优化/BCD 的写法，和 PPO 不匹配。以下按“受限策略 + MDP + PPO”写。

### 5.1 Overview

> 本文不直接求解问题 (opt:main)。原因有三：第一，CRB 对轨迹和波束非凸，且用户关联是离散的；第二，用户未来位置和信道实现未知，离线优化会引入不可实现的先知信息；第三，若每个时隙都在线求解波束优化，计算量难以支撑长时域闭环。为此，我们把闭环决策写成有限时域 MDP，在受限策略类 \(\mathfrak P_{\rm res}\) 内学习轨迹与感知功率；波束在 CSI 已知后由固定低复杂度规则生成。方法由三部分组成：MDP 建模、策略网络、PPO 训练。

> 与“直接学波束”的做法不同，本文只学习 \(4K\) 维动作：每架 UAV 的三维位移和感知功率占比。关联、RZF 方向、通信功率均分和感知导向都在环境内部完成。这样策略的观测严格满足因果信息结构，推理时也只需要一次前向计算。

### 5.2 MDP formulation

#### 5.2.1 State and observation

定义状态
\[
\mathbf s_t=\big(\mathbf p[t-1],\ \mathbf u[t],\ \Delta\mathbf u[t],\ \mathbf q,\ \boldsymbol\sigma_{\rm rcs},\ \boldsymbol\omega,\ \mu,\ t\big),
\]
其中 \(\mathbf p[t-1]\) 是动作前位置，\(\mathbf u[t]\) 是当前用户位置，\(\Delta\mathbf u[t]=\mathbf u[t]-\mathbf u[t-1]\) 是当前可观测位移，\(\mathbf q\) 是静态目标位置，\(\boldsymbol\sigma_{\rm rcs}\) 是逐目标 RCS，\(\boldsymbol\omega\) 是用户权重，\(\mu\) 是通感权衡系数，\(t\) 是时隙索引。\(t=0\) 时 \(\Delta\mathbf u[0]=\mathbf 0\)。

观测是状态的归一化映射
\[
\mathbf o_t=\phi(\mathbf s_t),
\]
不包含当前 \(\mathbf h[t]\)。这不是 POMDP：\(\mathbf h[t]\) 是块衰落、跨时隙独立，且不影响 \(\mathbf p[t+1]\) 或 \(\mathbf u[t+1]\)，只让奖励随机；对固定 \((\mathbf s_t,\mathbf a_t)\)，\(\mathbb E_{\mathbf h}[r_t]\) 是确定的，转移仍是 Markov 的。把 \(\mathbf h[t]\) 排除在观测外，是受限策略类的设计选择，不是状态缺失。

> 论文正文不必列出所有归一化公式；只给观测包含哪些量、为什么这样设计。归一化细节放实现或附录。

#### 5.2.2 Action

动作
\[
\mathbf a_t=\big(\mathbf a^{\rm pos}[t],\ \mathbf a^\varsigma[t]\big)\in[-1,1]^{4K},
\]
物理映射为
\[
\boldsymbol\delta_k[t]=\mathbf a^{\rm pos}_k[t]\,v_{\max}\Delta t,\qquad
\varsigma_k[t]=\frac{a^\varsigma_k[t]+1}{2}.
\]
环境用 \(\operatorname{Proj}(\cdot)\) 把任意位移投影到 C1–C3 的可行集：
\[
\mathbf p[t]=\operatorname{Proj}\big(\mathbf p[t-1]+\boldsymbol\delta[t]\big).
\]
因此策略不需要自己满足运动学/安全约束；投影损失可以作为观测特征。

\(t=0\) 时 \(\mathbf p[0]\) 给定，位移动作被忽略，但 \(\varsigma[0]\) 仍参与决策。正文要明确这一点，否则读者会问“为什么第一个动作无效”。

#### 5.2.3 Transition and reward

转移为
\[
\mathbf p[t]=\operatorname{Proj}\big(\mathbf p[t-1]+\boldsymbol\delta[t]\big),\qquad
k(m)[t]=\operatorname{associate}\big(\mathbf p[t],\mathbf u[t]\big),
\]
\[
\mathbf u[t+1]\sim \text{correlated random walk},\qquad
\mathbf h[t]\sim \text{Rician}\big(\mathbf p[t],\mathbf u[t]\big).
\]
单时隙奖励
\[
r_t=\bar R[t]-\mu\bar C[t],
\]
其中 \(\bar R\)、\(\bar C\) 的定义与第 3 节一致。目标为
\[
\max_{\pi}\ \mathbb E_{\pi}\left[\frac1T\sum_{t=0}^{T-1}r_t\right].
\]
这是有限时域平均，不是折扣和；训练时取 \(\gamma=1\)，终止处不 bootstrap。

#### 5.2.4 What is learned vs fixed

| 决策/模块 | 是否学习 | 说明 |
|---|---|---|
| UAV 位移 \(\boldsymbol\delta[t]\) | 是 | 3D，受 C1–C3 投影 |
| 感知功率占比 \(\varsigma[t]\) | 是 | 在 \(\mathbf h[t]\) 之前决定 |
| 用户关联 \(k(m)[t]\) | 否 | 由大尺度增益确定性导出 |
| 通信波束方向 | 否 | RZF，看到 \(\mathbf h[t]\) 后计算 |
| 感知波束方向 | 否 | 固定目标导向规则 |
| 通信功率分配 | 否 | 服务用户间均分 |
| 目标权重 \(c_{k,n}\) | `[待定]` | 先固定规则，标定后决定是否进动作 |

### 5.3 Policy network and training

> 策略网络采用 MLP，输入展平观测，输出高斯分布的均值与对数标准差；动作经 tanh 压缩到 \([-1,1]\)。\(\varsigma\) 头经 \((a+1)/2\) 映射到 \((0,1)\)。选择 tanh-squashed Gaussian 而不是“采样后 clip”，是为了避免边界处梯度被截断。

> 训练采用 PPO。与离线凸优化相比，PPO 不需要未来用户轨迹，推理时只做一次前向计算；与 DDPG/SAC 相比，on-policy PPO 在固定时域和并行环境里更稳定。本文不声称 PPO 的收敛性，只报告经验稳定性。

训练设置（正文可精简为一段，细节放表格/附录）：

- \(\gamma=1\)，GAE \(\lambda=0.95\)；
- reward scale \(1/(1+\mu)\)，评估用原始 \(r_t\)；
- 每个动作平均 \(L=2\sim4\) 个 fading 实现，降低奖励方差；
- 并行环境 8–16，独立 seed；
- 训练时 \(T=200\sim500\)，评估时 \(T=1000\)；
- 固定 held-out seeds，报告均值 \(\pm\) 标准误。

算法 1 草稿：

```text
Algorithm 1: PPO-based trajectory and sensing-power learning
1: 初始化策略 πθ、值函数 Vφ、并行环境数 N
2: for iteration = 1..N_iter do
3:   for each environment in parallel do
4:     reset environment; obtain o_0
5:     for t = 0..T-1 do
6:       a_t ~ πθ(·|o_t)
7:       δ_t, ς_t ← physical mapping of a_t
8:       p_t ← Proj(p_{t-1} + δ_t)          // t=0 时 δ 忽略
9:       k(m) ← associate(p_t, u_t)
10:      h_t ~ Rician; beams ← fixed rules
11:      r_t ← R̄_t − μ C̄_t
12:      store (o_t, a_t, r_t, o_{t+1}, done)
13:    end for
14:  end for
15:  compute GAE with γ=1
16:  update θ, φ by PPO clipped objective
17: end for
```

### 5.4 Complexity and implementation

> 推理阶段每个时隙只需一次策略前向计算和一次固定波束构造；不需要迭代求解凸问题。计算量主要来自信道生成、RZF 波束和 CRB，随时隙数 \(T\)、UAV 数 \(K\)、用户数 \(M\)、目标数 \(N\)、天线数 \(N_t\) 线性增长。训练成本主要来自并行 rollout 和 PPO 更新，离线完成。

> 本文不给出 PPO 的收敛保证。与基于 SCA/BCD 的方法相比，本文的优势是在线推理快、不需要未来信息；代价是需要离线训练，且性能依赖训练分布。

### 5.5 Extension: hybrid / two-stage（可选，不放进主线）

> 如果标定显示 \(\varsigma\) 在 CSI 前决策损失太大，可以把 \(\varsigma\) 放到 CSI 之后：环境先算 \(\mathbf h[t]\)，再对 \(\varsigma\) 做一维网格/坐标下降，或用两阶段策略。这个 hybrid 版本不作为主方法，而是作为强基线/上界，用来回答“自适应感知功率到底值多少”。

---

## 6. Simulation Results（计划）

### 6.1 Calibration gate（先做，再跑 RL）

标定不是调参，而是确认感知项是否有梯度、是否可区分。当前标定脚本有三个问题，使用数据前必须修：

1. `static / user_centroid / target_centroid` 在 \(t=0\) 没有 `skip_motion=True`，centroid 基线在 \(t=0\) 就移动了 \(\mathbf p[0]\)，与论文“\(\mathbf p[0]\) 给定”不一致。
2. `random_layout` 用 `skip_motion=True`，不消耗 C3 随机顺序；其他策略消耗。`evaluate_slot` 默认 `order="random"`，导致不同策略的 fading 流错位，共同随机数没有真正对齐。
3. 最后一个时隙后还调用了一次 `step_users()`，产生不参与计算的 \(\mathbf u[T]\)；对均值无害，但不符合论文语义。

修完之后，标定至少要做：

- \(\mu=5,10\) 的完整 \(\varsigma\) 扫描。论文现在写 \(\mu=10\)，但现有原始表是 \(\mu=1\)，不能直接引用。
- 感知方向对照：等权 / 最近目标 / 最大 CRB / 逆 CRB 加权；`null_space` 开/关。
- `target_centroid` 相对 `static` 的 \(\Delta\bar C\) 是否显著。现有结果约 \(1.7\sigma\)，不够；如果修完 RNG 仍然不显著，说明感知方向是瓶颈，RL 的感知故事不成立。
- 冻结 \(\mu\)、\(\Gamma_{\rm cap}\)、\(T_s\)、感知方向规则，以及 \(c_{k,n}\) 是否进动作。

**Go/no-go 判据**：在 \(\mu=10\) 附近，至少部分策略的网格最优 \(\varsigma^*\) 在内部（不是 0），且不同感知方向规则能拉开 \(\bar C\)。否则不要写“学习感知功率”作为主要贡献，改用 hybrid 或先改感知波束。

### 6.2 Baselines

| 类别 | 方案 | 信息 | 作用 |
|---|---|---|---|
| 无学习 | `static` | 无 | 下界/参照 |
| 无学习 | `user_centroid` | 当前用户位置 | 通信贪婪基线 |
| 无学习 | `target_centroid` | 当前目标位置 | 感知贪婪基线 |
| 优化 | SCA/BCD，因果信息 | 当前及预测用户 | 强可达基线 |
| 学习 | 本文 v1 | 因果观测 | 主方法 |
| 上界 | 先知辅助/松弛 | 未来用户、完美 CSI | 性能极限，不可实现 |

> 只和 static/centroid 比不够；至少需要一个可实现的强优化基线。`random_layout` 只作 sanity check，不作为主要基线。

### 6.3 Upper bound（措辞要小心）

- 如果是 **SCA/BCD**，得到的是可行解/局部解，对最大化问题是下界，**不是上界**。
- 如果是 **outer relaxation**，并且全局求解，才可能是上界；前提是松弛可行域包含原问题可行域。
- 如果是 **genie-aided offline**（知道未来用户轨迹、完美 CSI），它是性能极限/上界，但不是可执行策略。
- 如果只是“凸优化求出来的解”，叫 **optimization-based baseline**，不要叫 theoretical upper bound。

建议做两个 benchmark：

1. **强可达基线**：因果信息下的 SCA/BCD；
2. **性能上界**：离线/先知辅助 + 必要松弛，明确写“non-causal, not implementable”。

上界的作用是量化“固定 RZF/等功率/单阶段信息结构”损失了多少，不是替代基线。如果上界太松，只当 sanity check，不要拿它当主要卖点。

### 6.4 Metrics and protocol

- 主指标：\(\frac1T\sum_t(\bar R[t]-\mu\bar C[t])\)；
- 分项：平均 \(\bar R\)、平均 \(\bar C\)、\(\varsigma\) 分布、约束违反率；
- 统计：同 seed 配对，按 episode 计算均值 \(\pm\) 标准误；报告 95% CI；
- 效率：推理时间/时隙、训练时间；
- 泛化：不同 \(K,M,N\)、用户速度、\(\mu\)、目标 RCS。

### 6.5 Result artifacts（计划）

- 表 I：仿真参数（现有表，等标定后改）。
- 图 3：训练曲线（reward / \(\bar R\) / \(\bar C\)）。
- 图 4：典型轨迹快照（UAV 轨迹 + 用户/目标）。
- 图 5：\(\bar R\)–\(\bar C\) 权衡曲线（不同 \(\mu\) 或 \(\varsigma\)）。
- 表 II：方法对比（objective、\(\bar R\)、\(\bar C\)、相对上界 gap、推理时间）。
- 图 6：消融（去掉 `crb_prev`、固定 \(\varsigma\)、单阶段 vs hybrid）。
- 图 7：鲁棒性（用户速度/目标数/\(\mu\)）。

### 6.6 Claims（等结果）

- “在 \(\mu=10\) 下，所提方法相对最强可达基线提升 `[待填]`%，与上界差距 `[待填]`%。”
- “学习到的 \(\varsigma\) 随几何/用户分布变化，而不是固定常数。”
- “去掉感知历史特征或固定 \(\varsigma\) 会显著降低 \(\bar C\)。”

没有数据前不要写这些句子。

---

## 7. Conclusion（骨架）

> 本文研究了多无人机毫米波 ISAC 中考虑因果信息结构的联合轨迹与感知功率分配问题。我们把问题写成有限时域 MDP，在受限策略类内用 PPO 学习轨迹和感知功率，波束由 CSI 后的固定低复杂度规则生成。仿真结果表明 `[待填]`。未来工作包括：两阶段/ hybrid 策略、逐用户功率分配、跨规模泛化，以及更紧的理论上界。

---

## 8. 写作顺序与依赖

| 顺序 | 内容 | 现在能不能写 | 依赖 |
|---:|---|---|---|
| 1 | 第 5 节 MDP + 策略 + 训练 | 能写骨架 | 方法主线选定 |
| 2 | 第 3/4 节一致性修改 | 能改 | 无 |
| 3 | 第 2 节相关工作/贡献 | 能起草 | 文献调研、基线定义 |
| 4 | 第 6.1 标定 | 能跑 | 修 RNG/语义 |
| 5 | 第 6.2 基线 | 标定后实现 | 强基线选择 |
| 6 | 第 6.3 上界 | 基线后 | 松弛/先知模型定义 |
| 7 | 第 6.4–6.6 结果 | 训练后 | 实验 |
| 8 | 摘要/引言 headline/结论 | 最后 | 结果 |

**建议**：现在先写第 5 节，同时修标定；不要等所有实验结束才动笔。第 2 节可以后写，因为贡献和定位依赖基线/上界。

---

## 9. 待定决策

| # | 决策 | 选项 | 建议 | 何时定 |
|---:|---|---|---|---|
| 1 | 方法主线 | 单阶段 DRL / hybrid / 两阶段 | v1 单阶段，hybrid 作基线 | 现在 |
| 2 | \(\varsigma\) 决策时刻 | CSI 前 / CSI 后 | v1 CSI 前；hybrid 对照 | 现在 |
| 3 | \(c_{k,n}\) 是否进动作 | 固定规则 / 进动作 | 先固定，标定后定 | 标定后 |
| 4 | \(\mu\) | 固定 10 / 扫描 | 主结果 10，附敏感性 | 标定后 |
| 5 | 强基线 | SCA/BCD / 启发式 | 至少一个 SCA/BCD | 标定后 |
| 6 | 上界 | 凸松弛 / 先知辅助 | 先定义合法性，再实现 | 基线后 |
| 7 | 训练/评估 \(T\) | 200–500 / 1000 | 短训练、长评估 | 训练前 |
| 8 | 网络 | MLP / DeepSets | v1 MLP | 现在 |

---

## 10. 对 `main.tex` 的具体修改清单

1. **摘要**：替换占位段，等结果填数字。
2. **Introduction**：删除 38 行 `placeholder`；补相关工作与贡献。
3. **Sec. III-C**：把“Section V optimizes”改成“Section V learns a policy within”。
4. **Sec. V**：整节替换，不要保留 `Iterative Update` / `Convergence and Complexity` 的凸优化模板。
5. **Algorithm 1**：改成 PPO 训练流程，不写 block update。
6. **Sec. VI**：保留参数表，替换 “Placeholder” 图注和示例表；补基线/上界/消融。
7. **Conclusion**：等结果后写。
8. **表 I**：\(\mu=10\) 需要标定支撑；如果没跑，先标 `[待定]`。

---

## 11. 不要写进论文正文的内容

- `docs/mdp.md` 里的全部归一化公式、RNG 细节、`info` 字段、测试清单；
- 代码路径、函数名、`shapes`；
- 过长的超参数表（放附录或补充材料）；
- “PPO 收敛”“理论最优”等没有证明的措辞；
- 未标定的 \(\mu=10\) 数值结论。

---

## 12. 下一步（最小行动）

1. 修 `experiments/calibrate.py` 的 \(t=0\)、终止、common random numbers 三个问题。
2. 跑 \(\mu=10\) 的感知方向标定，确认 \(\bar C\) 是否有可学习的差异。
3. 按本文件第 5 节写 `main.tex` 的 Proposed Method 骨架，未定项标 `[待定]`。
4. 标定通过后，冻结方法选择，再开始训练和基线实现。
