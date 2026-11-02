# 多无人机毫米波 ISAC 网络（论文正文中文版）

> 说明：本文档由 `paper/main.tex` 中**已成稿**的正文翻译整理而来，公式保持原始 LaTeX 记号（可直接复制回 tex）。文中公式编号（如式 (17)、式 (34)、式 (49)）沿用 `main.tex` 当前编译出的编号，与 `main.pdf` 一致；本文档内的公式块未另行编号。
> 纯模板/占位内容**未翻译**，仅在相应位置标注，包括：标题与作者块、摘要段、关键词、Introduction 的模板说明文字与 37 处 `placeholder` 段、Algorithm 1 的模板伪代码、`Proposed Method` 的说明性文字、Simulation Results 中的示例表 `tab:comparison`、Conclusion 的模板段、Acknowledgment。另有被注释掉的 Assumptions 列表与一条时隙索引注释（`main.tex` 第 394–408 行、第 178 行），不属于正文，未纳入。

---

## 1. Introduction（引言，尚未撰写）

当前该节仅有模板说明文字：背景段（应用场景、驱动趋势、问题价值）、文献综述段与三点贡献列表、总结果表述段、`Related Work` 与 `Contributions and Organization` 两个小节；其后为 36 行 `placeholder`。正文尚未成稿，故不翻译。

---

## 2. System Model（系统模型）

**图 1（`fig:system-overview`）**：所考虑系统的概览。（图件为 `system-placeholder.png`）

**图 2（`fig:system`）**：多无人机毫米波 ISAC 网络的示意图。（图件为 `system-io.png`）

### 2.1 系统描述

我们考虑图 2 所示的多无人机毫米波 ISAC 网络。该网络包含 $K$ 架无人机（$\mathcal K=\{1,2,\ldots,K\}$）、$M$ 个地面用户（$\mathcal M$）、$N$ 个静态地面目标（$\mathcal N$），以及一个由领航无人机承载的中央控制器（CC）。

每架无人机配备一个 $N_t$ 元均匀线阵（ULA），并在设计的高度带内飞行。天线数目满足 $N_t\ge M+1$。所有地面用户按照 2.2.4 节的相关随机游走模型移动，且各配备单根天线；地面目标则是静态无源反射体，不携带任何收发信机。无人机与 CC 之间的回传链路被视为无差错且无时延。CC 收集全局状态信息，运行轨迹、功率与波束成形策略，并将得到的控制量回传给各无人机。

我们考虑时长为 $T\Delta t$ 秒的任务时域，它被划分为 $T$ 个等长、互不重叠的时隙，每时隙长度为 $\Delta t$。时隙 $t\in\mathcal T\triangleq\{0,1,\ldots,T-1\}$ 对应区间 $[t\Delta t,(t+1)\Delta t)$。$\Delta t$ 足够小；时隙内的几何关系与信道被视为保持不变，并以时隙起始时刻的位置来表征。在全文中，凡不产生歧义之处均省略时隙索引 $[t]$。在每个时隙中，每个用户 $m$ 恰好与一架无人机关联，记作 $k(m)\in\mathcal K$，于是无人机 $k$ 所服务的用户集合为 $\mathcal M_k=\{m:k(m)=k\}$；确定 $k(m)$ 的规则由式 (17) 给出。每架无人机向 $\mathcal M_k$ 中的每个用户发送一个通信流，并发送一个专用感知流，同时接收该发射信号的目标回波。

### 2.2 坐标系与运动学

#### 2.2.1 坐标系

所有位置均在统一的右手直角坐标系中表述，其中 $x$–$y$ 平面为水平面，$z$ 轴指向上方。水平任务区域为 $\mathcal A=[0,L_x]\times[0,L_y]$。地面用户与目标位于地平面，而无人机在高度带 $[z_{\min},z_{\max}]$ 内飞行。无人机 $k$、用户 $m$ 与目标 $n$ 的位置分别为

$$
\mathbf p_k[t]=\big(x_k[t],\,y_k[t],\,z_k[t]\big)^T,\qquad
\mathbf u_m[t]=\big(x_m[t],\,y_m[t],\,0\big)^T,\qquad
\mathbf q_n=\big(x_n,\,y_n,\,0\big)^T,
$$

其中 $\mathbf p_k[t]$、$\mathbf u_m[t]$ 表示时刻 $t\Delta t$（即时隙 $t$ 起始）的位置。目标是静止的，用户则按 2.2.4 节所述的相关随机游走移动。

#### 2.2.2 链路几何

对于从点 $\mathbf o_1$ 指向点 $\mathbf o_2$ 的链路，令 $\mathbf r=\mathbf o_2-\mathbf o_1=(r_x,r_y,r_z)^T$。链路距离、方位角与俯仰角分别为

$$
d=\|\mathbf r\|,\qquad
\theta=\operatorname{atan2}(r_y,r_x)\in(-\pi,\pi],\qquad
\phi=\operatorname{atan2}\!\Big(r_z,\sqrt{r_x^2+r_y^2}\Big)\in\Big[-\tfrac{\pi}{2},\tfrac{\pi}{2}\Big],
$$

其中 $\theta$ 自 $x$ 轴正向起量、朝向 $y$ 轴正向，$\phi$ 自水平面量起、向上为正。当链路严格垂直（$r_x=r_y=0$）时 $\theta$ 无定义；该事件测度为零，予以忽略。据此，$d_{k,m}=\|\mathbf p_k-\mathbf u_m\|$ 与 $d_{k,n}=\|\mathbf p_k-\mathbf q_n\|$ 分别表示无人机—用户与无人机—目标距离，$(\theta_{k,m},\phi_{k,m})$ 与 $(\theta_{k,n},\phi_{k,n})$ 表示在无人机 $k$ 处测得的相应离开角。由于无人机始终高于地面节点，故 $\phi_{k,m}<0$、$\phi_{k,n}<0$，且 $d_{k,m}\ge z_{\min}$。与之相对，2.3 节中的 LoS 概率取决于从地面节点观察无人机所得的俯仰角，记为 $\theta_{\rm el}$（单位为度）。对同一条链路，这两个俯仰角符号相反，即

$$
\theta_{\rm el}=-\phi_{k,m}\cdot\frac{180^\circ}{\pi}\in(0^\circ,90^\circ].
$$

#### 2.2.3 无人机运动学

每架无人机的轨迹受运动学与安全约束：

$$
\begin{aligned}
&\|\mathbf p_k[t+1]-\mathbf p_k[t]\|\le v_{\max}\Delta t,\quad t=0,\ldots,T-2,\\
&\mathbf p_k[t]\in\mathcal A\times[z_{\min},z_{\max}],\quad t=0,\ldots,T-1,\\
&\|\mathbf p_k[t]-\mathbf p_{k'}[t]\|\ge d_{\min},\quad\forall\,k\neq k',\ t=0,\ldots,T-1,
\end{aligned}
$$

其中 $\mathcal A$ 为水平任务区域，$d_{\min}$ 为无人机间防碰撞距离。

#### 2.2.4 地面用户移动性

地面用户遵循相关随机游走，该模型刻画了行人或车辆运动的朝向持续性。用户 $m$ 的水平速度 $\mathbf v^{\rm u}_m[t]\in\mathbb R^2$ 按一阶 Gauss–Markov 过程演化：

$$
\tilde{\mathbf v}^{\rm u}_m[t+1]=\zeta\,\mathbf v^{\rm u}_m[t]+\sqrt{1-\zeta^{2}}\,\sigma_{\rm u}\,\boldsymbol\epsilon_m[t],\qquad
\boldsymbol\epsilon_m[t]\sim\mathcal N(\mathbf 0,\mathbf I_2),
$$

其中 $\zeta=e^{-\Delta t/\tau_{\rm v}}\in(0,1)$ 是由速度相关时间 $\tau_{\rm v}$ 决定的记忆系数，$\sigma_{\rm u}$ 为平稳速度的单轴标准差。新息 $\boldsymbol\epsilon_m[t]$ 在用户之间与时隙之间相互独立。在极限 $\tau_{\rm v}\to0$ 下 $\zeta\to0$，速度退化为不相关随机游走；而 $\tau_{\rm v}\to\infty$ 给出 $\zeta\to1$，即直线运动。速度经重缩放被限制在 $v^{\rm u}_{\max}$ 以内：

$$
\mathbf v^{\rm u}_m[t+1]=\tilde{\mathbf v}^{\rm u}_m[t+1]\,\min\!\Big\{1,\ \frac{v^{\rm u}_{\max}}{\|\tilde{\mathbf v}^{\rm u}_m[t+1]\|}\Big\},
$$

而二维水平位置 $\tilde{\mathbf u}_m[t+1]\in\mathbb R^2$ 更新为

$$
\tilde{\mathbf u}_m[t+1]=\big(x_m[t],\,y_m[t]\big)^T+\mathbf v^{\rm u}_m[t+1]\,\Delta t.
$$

此处 $\mathbf v^{\rm u}_m[t]$ 为时刻 $t\Delta t$ 的速度状态。因此更新式 (11)–(13) 构成一种右端点（隐式）离散化：采用时刻 $(t+1)\Delta t$ 的速度 $\mathbf v^{\rm u}_m[t+1]$ 将 $\mathbf u_m[t]$ 推进到 $\mathbf u_m[t+1]$。

记 $\tilde x_m$、$\tilde y_m$ 为 $\tilde{\mathbf u}_m[t+1]$ 的两个分量。为使用户保持在 $\mathcal A$ 内，对每个坐标施加反射边界。对范围为 $[0,L_x]$ 的 $x$ 坐标：

$$
x_m[t+1]=
\begin{cases}
-\tilde x_m, & \tilde x_m<0,\\
2L_x-\tilde x_m, & \tilde x_m>L_x,\\
\tilde x_m, & \text{otherwise},
\end{cases}
$$

且在前两种情形下 $\mathbf v^{\rm u}_m[t+1]$ 的 $x$ 分量变号。$y$ 坐标遵循同一规则，只需将 $x$、$\tilde x_m$、$L_x$ 分别替换为 $y$、$\tilde y_m$、$L_y$。由于单时隙位移 $v^{\rm u}_{\max}\Delta t\ll\min\{L_x,L_y\}$，每个坐标至多反射一次即可。初始位置在 $\mathcal A$ 上均匀抽取，初始速度由 $\mathcal N(\mathbf 0,\sigma_{\rm u}^2\mathbf I_2)$ 抽取并按式 (12) 的同一重缩放截断；当 $v^{\rm u}_{\max}\gtrsim 4\sigma_{\rm u}$ 时，速度上限与边界反射对该分布仅有边际扰动，因而速度近似保持平稳。

假设用户远慢于无人机，即 $v^{\rm u}_{\max}\ll v_{\max}$，从而使无人机能够跟踪用户。与无人机轨迹不同，用户轨迹是外生的随机过程，不是决策变量。用户位置 $\{\mathbf u_m[t]\}$ 在时隙 $t$ 起始时为 CC 已知，而其未来位置未知，仅有式 (11)–(13) 中的统计量可用。

### 2.3 A2G 毫米波信道模型

A2G 信道按块衰落模型处理。LoS 分量的确定性多普勒被假定已补偿，而散射分量在每个时隙重新抽取。

令 $\theta_{\rm el}$ 表示从地面节点观察无人机所得的俯仰角（单位为度）。LoS 概率遵循 Al-Hourani 的 sigmoid 模型：

$$
P_{\rm LoS}(\theta_{\rm el})=\frac{1}{1+a\exp\!\big(-b(\theta_{\rm el}-a)\big)},
$$

其中 $a$、$b$ 为环境相关常数。

记无人机 $k$ 与用户 $m$ 之间的距离为 $d_{k,m}=\|\mathbf p_k-\mathbf u_m\|$。由于自由空间参考模型仅在参考距离 $d_0$ 之外有效，有效距离被下截断为 $\bar d_{k,m}=\max\{d_{k,m},d_0\}$，参考信道增益为 $\beta_0=(\lambda/4\pi d_0)^2$。于是 LoS/NLoS 加权平均路损增益为

$$
\bar\beta_{k,m}=P_{\rm LoS}\,\beta_0\Big(\frac{d_0}{\bar d_{k,m}}\Big)^{\eta_{\rm L}} +\big(1-P_{\rm LoS}\big)\beta_0\Big(\frac{d_0}{\bar d_{k,m}}\Big)^{\eta_{\rm N}},
$$

其中 $\eta_{\rm L}<\eta_{\rm N}$ 分别为 LoS 与 NLoS 链路的路径损耗指数。

大尺度增益同时确定了用户关联：在每个时隙，用户 $m$ 由平均增益最强的无人机服务，

$$
k(m)[t]=\arg\max_{k\in\mathcal K}\ \bar\beta_{k,m}[t],\qquad\forall m,\ t,
$$

其中平局按最小索引优先裁决。由于 $\bar\beta_{k,m}[t]$ 仅依赖该时隙的位置 $\mathbf p_k[t]$ 与 $\mathbf u_m[t]$，关联是时隙几何的确定性函数：位置一旦已知即被固定，无需小尺度衰落的任何信息，也不是网络的优化变量。

无人机 $k$ 到用户 $m$ 的信道服从 Rician 衰落模型：

$$
\mathbf h_{k,m}=\sqrt{\frac{\kappa\bar\beta_{k,m}}{\kappa+1}}\, \mathbf a(\theta_{k,m},\phi_{k,m}) +\sqrt{\frac{\bar\beta_{k,m}}{\kappa+1}}\,\mathbf g^{\rm s}_{k,m},
$$

其中 $\kappa$ 为 Rician $K$ 因子，$\mathbf g^{\rm s}_{k,m}\sim\mathcal{CN}(\mathbf 0,\mathbf I_{N_t})$ 表示 NLoS 散射分量。

ULA 沿 $x$ 轴布设，阵元间距 $d_a=\lambda/2$。因此导向矢量 $\mathbf a(\theta,\phi)$ 表示为

$$
[\mathbf a(\theta,\phi)]_i=\exp\!\big(j\pi\, i\cos\phi\cos\theta\big), \quad i=0,\dots,N_t-1.
$$

这里 $\theta_{k,m}$、$\phi_{k,m}$ 分别表示在无人机处测得的离开方位角与离开俯仰角。方位角自 $x$ 轴正向起量，而俯仰角对向下离开方向取负值。

### 2.4 发射信号与下行通信

无人机 $k$ 发射信号

$$
\mathbf x_k=\sum_{j\in\mathcal M_k}\mathbf w_{k,j}s_{k,j}+\mathbf w_{k,r}s_{k,r},
$$

其中 $\mathbf w_{k,j}\in\mathbb C^{N_t}$ 是面向其所服务用户 $j$ 的通信波束成形器，$\mathbf w_{k,r}\in\mathbb C^{N_t}$ 是专用感知波束成形器，$s_{k,j}$ 是用户 $j$ 的数据符号，$s_{k,r}$ 是已知探测波形。由于无人机之间不进行联合发射，非服务无人机向用户 $m$ 发送的流不会携带有用信号、只会造成干扰，因此不发送这类流。符号满足：对 $j,j'\in\mathcal M_k$，$\mathbb{E}[s_{k,j}s_{k,j'}^*]=\delta_{jj'}$；$\mathbb{E}[s_{k,j}s_{k,r}^*]=0$；$\mathbb{E}[|s_{k,r}|^2]=1$。此外，不同无人机的流彼此不相关（无联合发射），即对 $k\neq k'$、$\ell\in\mathcal M_k\cup\{r\}$、$\ell'\in\mathcal M_{k'}\cup\{r\}$，有 $\mathbb{E}[s_{k,\ell}s_{k',\ell'}^*]=0$。

在上述符号统计下，无人机 $k$ 的发射协方差矩阵为

$$
\mathbf R_k=\mathbb{E}\!\left[\mathbf x_k\mathbf x_k^H\right]
=\sum_{j\in\mathcal M_k}\mathbf w_{k,j}\mathbf w_{k,j}^H+\mathbf w_{k,r}\mathbf w_{k,r}^H\succeq\mathbf 0,
$$

其总发射功率为

$$
\operatorname{tr}(\mathbf R_k)=\sum_{j\in\mathcal M_k}\|\mathbf w_{k,j}\|^2+\|\mathbf w_{k,r}\|^2\le P_{\max},\quad \forall k\in\mathcal K.
$$

为刻画各无人机如何在两种功能间分配功率，定义**感知功率占比**

$$
\varsigma_k\triangleq\frac{\|\mathbf w_{k,r}\|^2}{\operatorname{tr}(\mathbf R_k)}\in[0,1], \quad \forall k\in\mathcal K\ \text{with}\ \operatorname{tr}(\mathbf R_k)>0.
$$

相应地，各无人机用于通信任务的功率预算为

$$
\sum_{j\in\mathcal M_k}\|\mathbf w_{k,j}\|^2=(1-\varsigma_k)\operatorname{tr}(\mathbf R_k),\quad \forall k\in\mathcal K,
$$

用于感知任务的功率预算为

$$
\|\mathbf w_{k,r}\|^2=\varsigma_k\operatorname{tr}(\mathbf R_k),\quad \forall k\in\mathcal K.
$$

因此 $\varsigma_k$ 越大，功率越从通信转向感知：$\varsigma_k=0$ 对应纯通信，而 $\varsigma_k=1$ 对应经由专用感知流的纯感知。

用户 $m$ 的期望信号功率为 $S_m=|\mathbf h_{k(m),m}^H\mathbf w_{k(m),m}|^2$。用户 $m$ 处的干扰由三部分组成：(i) 来自服务无人机其他通信流的多用户干扰（MUI）；(ii) 来自非服务无人机发往其自身用户的通信流的无人机间干扰；(iii) 来自所有无人机感知波束的感知致干扰，即

$$
\begin{aligned}
I_m={}&\sum_{k\in\mathcal K}\ \sum_{j\in\mathcal M_k\setminus\{m\}}\big|\mathbf h_{k,m}^H\mathbf w_{k,j}\big|^2\\
&+\sum_{k\in\mathcal K}\big|\mathbf h_{k,m}^H\mathbf w_{k,r}\big|^2.
\end{aligned}
$$

其中内层求和排除 $j=m$：由于 $m\in\mathcal M_{k(m)}$ 而 $k\neq k(m)$ 时 $m\notin\mathcal M_k$，这恰好把用户 $m$ 的期望流，即配对 $(k,j)=(k(m),m)$，从干扰求和中剔除。上式的最后一项即感知泄漏。在本地 CSI 下，无人机 $k$ 可将其感知波束投影到所服务用户信道子空间的正交补上，从而对每个所服务用户消除 $k=k(m)$ 这一项，代价是感知阵列增益下降；3.1 节将该投影作为一种固定的波束成形规则采用。用户 $m$ 的接收 SINR $\gamma_m$ 与可达速率 $R_m$ 为

$$
\gamma_m=\frac{S_m}{\sigma^2+I_m},\qquad R_m=B\log_2(1+\gamma_m),
$$

其中 $\sigma^2$ 为噪声功率，$B$ 为系统带宽。

### 2.5 感知模型

#### 2.5.1 回波与观测模型

各无人机由超高速回传链路互联并共享统一时钟与本振，因此其回波观测可相干融合。每个目标被建模为各向同性点散射体，其反射率不依赖观测方向。假定各目标在时延或角度上可相互分辨，故其回波互不重叠、每个 CRB 可独立处理；杂波被忽略，且用户不产生回波。

于是目标 $n$ 相对无人机 $k$ 的反射系数可分解为

$$
\alpha_{k,n}=\chi_n\,g_{k,n},\qquad
g_{k,n}=\frac{\lambda}{(4\pi)^{3/2}d_{k,n}^{2}}\,e^{-j4\pi d_{k,n}/\lambda},
$$

其中 $\chi_n=\sqrt{\sigma_{{\rm rcs},n}}\,e^{j\psi_n}$ 是目标 $n$ 的未知复反射率，$\sigma_{{\rm rcs},n}$ 为其 RCS、$\psi_n$ 为反射相位；$d_{k,n}=\|\mathbf q_n-\mathbf p_k\|$ 是无人机 $k$ 与位于 $\mathbf q_n$ 的目标 $n$ 之间的距离。因子 $g_{k,n}$ 是已知的：其幅度体现双程自由空间路损（功率上为 $d_{k,n}^{-4}$），其相位为往返传播相位，在几何给定下是确定的，并借助相干链路得以解出。因此 $|\alpha_{k,n}|^2=\lambda^2\sigma_{{\rm rcs},n}/((4\pi)^3d_{k,n}^4)$，所有无人机共享的唯一未知量是标量 $\chi_n$。与信道模型一致，此处同样施加 $d_0$ 处的距离下限，但由于 $d_{k,n}\ge z_{\min}\ge d_0$（$d_0$ 为米级）该下限不起作用。

每架无人机已知其发射符号，并将其用作匹配滤波参考。由发射流之间的互不相关性，这些符号序列（包括不同无人机发射的）在相干积累的 $N_s=BT_s$ 个样本上近似正交，其中 $T_s$ 为积累时间，故每个流经匹配滤波后被分离出来。仅处理单站回波，即无人机 $k$ 只使用自身发射的回波；其他无人机信号的双站回波被丢弃，这进一步使该界偏保守。于是按无人机 $k$ 的 $|\mathcal M_k|+1$ 个流堆叠的观测向量为

$$
\mathbf y_{k,n}=\sqrt{N_s}\,\alpha_{k,n}\mathbf W_k^H\mathbf a(\theta_{k,n},\phi_{k,n})+\mathbf z,
$$

其中 $\mathbf W_k=[\{\mathbf w_{k,j}\}_{j\in\mathcal M_k},\mathbf w_{k,r}]$，$\mathbf z\sim\mathcal{CN}(\mathbf 0,\sigma_r^2\mathbf I_{|\mathcal M_k|+1})$。回波 SNR 仅通过 $\mathbf R_k$ 依赖波束成形器：

$$
\rho_{k,n}=\frac{N_s|\alpha_{k,n}|^2\,\mathbf a_{k,n}^H\mathbf R_k\mathbf a_{k,n}}{\sigma_r^2},
$$

其中 $\mathbf a_{k,n}\triangleq\mathbf a(\theta_{k,n},\phi_{k,n})$。该式揭示了感知与通信之间的耦合：感知性能对功率在各流之间如何分配不敏感，只依赖总的空域协方差，因此通信流同样对感知有贡献。

#### 2.5.2 Fisher 信息矩阵

对每个目标，未知参数向量为 $\boldsymbol\xi_n=[x_n,\,y_n,\,\ln|\chi_n|,\,\arg\chi_n]^T$，其中前两项为待估参数，后两项为冗余参数。由于所有无人机观测同一目标，它们共享同一对冗余参数。采用 $\ln|\chi_n|$ 参数化仅为记号方便；冗余参数的任意可逆重参数化都不改变位置 CRB。

为使模型可解且偏保守，忽略 $\alpha_{k,n}$ 对目标位置的依赖，即在求导时将 $g_{k,n}$ 视为已知常数。特别地，载波相位项 $e^{-j4\pi d_{k,n}/\lambda}$ 不用于测距；定位只利用空域波束图样与时延。

定义 $\mathbf H_{k,n}\in\mathbb C^{N_t\times4}$ 为接收信号流形对 $\boldsymbol\xi_n$ 的 Jacobian：

$$
\mathbf H_{k,n}=\alpha_{k,n}\big[\partial_x\mathbf a_{k,n},\ \partial_y\mathbf a_{k,n},\ \mathbf a_{k,n},\ j\,\mathbf a_{k,n}\big].
$$

四列均含公共因子 $\alpha_{k,n}$，故其相位在 $\mathbf H_{k,n}^H\mathbf R_k\mathbf H_{k,n}$ 中相消，只剩 $|\alpha_{k,n}|^2$。角度域 FIM 为

$$
\mathbf J^{\rm ang}_n=\frac{2N_s}{\sigma_r^2}\sum_{k\in\mathcal K}\operatorname{Re}\big\{\mathbf H_{k,n}^H\mathbf R_k\mathbf H_{k,n}\big\}.
$$

窄带模型 (29) 中不含时延：时延由宽带探测波形获得，其估计方差为 $(8\pi^2B_{\rm rms}^2\rho_{k,n})^{-1}$，其中 $\rho_{k,n}$ 为式 (30) 的回波 SNR，且在 $B$ 上频谱平坦时 $B_{\rm rms}^2=B^2/12$。由此得到关于水平位置的信息为

$$
\mathbf J^{\rm del}_n=\sum_{k\in\mathcal K}8\pi^2B_{\rm rms}^2\Big(\frac{2}{c}\Big)^2\rho_{k,n}\,\mathbf e_{k,n}\mathbf e_{k,n}^T,
$$

其中 $\mathbf e_{k,n}=(\mathbf q_n-\mathbf p_k)_{xy}/d_{k,n}$ 是 $d_{k,n}$ 对目标水平位置的梯度，其范数 $\|\mathbf e_{k,n}\|=\cos\phi_{k,n}\le 1$。因此总 FIM 为

$$
\mathbf J_n=\mathbf J^{\rm ang}_n+\operatorname{blkdiag}\big(\mathbf J^{\rm del}_n,\ \mathbf 0_{2\times2}\big).
$$

式 (34) 中的两个分块按“同一次回波的时延估计与角度估计相互独立”相加。由于上面只保留了群时延，时延与角度导数在频带上是正交的，而由阵列孔径引起的残余耦合可忽略，因为 $(N_t-1)\lambda/2\ll c/(2B)$。

#### 2.5.3 克拉美–罗界（CRB）

按位置参数与冗余参数对 $\mathbf J_n$ 分块：

$$
\mathbf J_n=\begin{bmatrix}\mathbf J_{pp,n} & \mathbf J_{p\nu,n}\\ \mathbf J_{p\nu,n}^T & \mathbf J_{\nu\nu,n}\end{bmatrix},
$$

其中 $\mathbf J_{pp,n}\in\mathbb R^{2\times2}$ 为位置分块，且已包含时延项 $\mathbf J^{\rm del}_n$；$\mathbf J_{p\nu,n}$ 与 $\mathbf J_{\nu\nu,n}$ 对应冗余参数。经 Schur 补消去冗余参数后，目标 $n$ 的位置 CRB 为

$$
\mathrm{CRB}_n=\operatorname{tr}\!\Big[\big(\mathbf J_{pp,n}-\mathbf J_{p\nu,n}\mathbf J_{\nu\nu,n}^{-1}\mathbf J_{p\nu,n}^T\big)^{-1}\Big]\quad[\mathrm{m}^2].
$$

由于 $\alpha_{k,n}$ 的公共相位在各个分块中相消，CRB 与反射相位 $\psi_n$ 以及传播相位 $4\pi d_{k,n}/\lambda$ 无关。因此相干融合假设只通过共享的冗余参数对起作用。又因丢弃了 $\alpha_{k,n}$ 对位置的依赖，所得界相对于完整信号模型（后者还会利用载波相位与幅度变化）是保守的。

对单架无人机而言，当目标位于平面 $r_y=y_n-y_k=0$ 内（即目标与无人机具有相同的 $y$ 坐标）时，位置分块变为奇异：角度信息与时延信息此时都沿 $x$ 方向作用，故 $y$ 坐标不可观测，ULA 也无法区分其轴线两侧。这是物理限制而非数值问题；且只有当所有无人机都满足 $y_n=y_k$ 时，它才会在 $K>1$ 时持续存在。

除上述几何退化外，当目标仅被单个流照射时 $\mathbf J_n$ 也会降秩：秩一的 $\mathbf R_k$ 经式 (32) 至多贡献两个实维度、经式 (33) 贡献一个实维度，故 $\operatorname{rank}(\mathbf J_n)\le3$，无论几何如何 CRB 都无界；而单机两个流，或两机各一个流，即可恢复满秩。

在下文中，感知指标采用截断后的界 $\widetilde{\mathrm{CRB}}_n=\min\{\mathrm{CRB}_n,\Gamma_{\rm cap}\}$：如上述两种情形中 $\mathbf J_n$ 奇异或病态的目标被赋值为 $\Gamma_{\rm cap}$，以保证指标有限。

---

## 3. Problem Formulation（问题建模）

### 3.1 信息结构

时隙 $t$ 按如下顺序进行。

1. CC 观测用户位置 $\mathbf u_m[t]$ 并决定无人机位置 $\mathbf p_k[t]$。关联 $k(m)[t]$ 不是决策：它由位置经规则 (17) 确定性地给出。对 $t=0$，$\mathbf p_k[0]$ 已给定，故不存在第一阶段决策。
2. 衰落 $\mathbf g^{\rm s}_{k,m}[t]$ 实现，得到信道 $\mathbf h[t]$。
3. CC 观测 $\mathbf h[t]$ 并决定波束成形器 $\mathbf w_{k,j}[t]$ 与 $\mathbf w_{k,r}[t]$。
4. 评估该时隙的速率 $R_m[t]$ 与界 $\mathrm{CRB}_n[t]$。

记时隙 $t$ 起始可用的历史为

$$
\mathcal H_t=\Big\{\{\mathbf u_m[\tau]\}_{\tau\le t},\ \{\mathcal X^{\rm I}[\tau],\mathcal X^{\rm II}[\tau],\mathbf h[\tau]\}_{\tau<t}\Big\},
$$

即截至并包含时隙 $t$ 的用户位置，以及此前所有时隙的决策与信道。时隙 $t$ 的决策分为两组：

$$
\mathcal X^{\rm I}[t] = \big\{\mathbf p_k[t]\big\},\qquad
\mathcal X^{\rm II}[t] = \big\{\{\mathbf w_{k,j}[t]\}_{j\in\mathcal M_k[t]},\ \mathbf w_{k,r}[t]\big\},
$$

其中 $\mathcal X^{\rm I}[t]$ 是 $\mathcal H_t$ 的函数，而 $\mathcal X^{\rm II}[t]$ 是 $(\mathcal H_t,\mathcal X^{\rm I}[t],\mathbf h[t])$ 的函数。注意 $\mathcal M_k[t]$ 由式 (17) 的关联 $k(m)[t]$ 决定，而后者是位置 $\mathbf p_k[t]$ 与 $\mathbf u_m[t]$ 的确定性函数；因此一旦选定 $\mathcal X^{\rm I}[t]$ 它即被固定，特别地，在决定波束成形器时它已被定义。具体地，定义两个映射

$$
\mathcal{L}^{\rm I}_t : \mathcal H_t \mapsto \mathcal X^{\rm I}[t],\qquad
\mathcal{L}^{\rm II}_t : \big(\mathcal H_t,\,\mathcal X^{\rm I}[t],\,\mathbf h[t]\big) \mapsto \mathcal X^{\rm II}[t],
$$

它们给出

$$
\mathcal{L}^{\rm I}_t\!\big(\mathcal H_t\big) = \big\{\mathbf p_k[t]\big\},\qquad
\mathcal{L}^{\rm II}_t\!\big(\mathcal H_t,\,\mathcal X^{\rm I}[t],\,\mathbf h[t]\big) = \big\{\{\mathbf w_{k,j}[t]\}_{j\in\mathcal M_k[t]},\;\mathbf w_{k,r}[t]\big\}.
$$

**策略** $\mathcal P$ 是这些映射在所有时隙上的汇集，形式化为

$$
\mathcal P = \big\{\,\mathcal{L}^{\rm I}_t,\;\mathcal{L}^{\rm II}_t\big\}_{t=0}^{T-1}.
$$

波束成形器确定了第 2 节中的发射协方差 $\mathbf R_k[t]$。

**受限策略类。** 类 (44) 允许波束成形器自由依赖瞬时 CSI。为便于处理，我们转而只学习位移以及感知功率占比 $\varsigma_k[t]$——它们是 $(\mathcal H_t,\mathcal X^{\rm I}[t])$ 的函数，因而在 $\mathbf h[t]$ 实现之前即已确定；而一旦 $\mathbf h[t]$ 已知，$\mathcal X^{\rm II}[t]$ 按固定规则而非优化给出：在所服务用户间等功率分配、通信流采用正则化迫零，以及一种要么指向目标、要么投影到所服务用户信道正交补上的感知方向。以单位范数方向 $\hat{\mathbf d}_{k,j}[t]$ 与 $\hat{\mathbf d}_{k,r}[t]$ 表示，波束成形器为

$$
\begin{aligned}
\mathbf w_{k,j}[t]&=\sqrt{\tfrac{(1-\varsigma_k[t])P_{\max}}{|\mathcal M_k[t]|}}\,\hat{\mathbf d}_{k,j}[t],\quad j\in\mathcal M_k[t],\\
\mathbf w_{k,r}[t]&=\sqrt{\varsigma_k[t]P_{\max}}\,\hat{\mathbf d}_{k,r}[t],
\end{aligned}
$$

其中若 $\mathcal M_k[t]=\emptyset$ 则取 $\varsigma_k[t]=1$。于是恒有 $\operatorname{tr}(\mathbf R_k[t])=P_{\max}$，即 (C4) 取等号，且所学策略属于子类 $\mathfrak P_{\rm res}\subset\mathfrak P$。

式 (45) 中的感知方向由一个二元开关 $\iota\in\{0,1\}$ 选择。令 $\bar{\mathbf d}_{k,r}[t]\neq\mathbf 0$ 为目标导向矢量的固定叠加，并令 $\mathbf Q_k[t]\in\mathbb C^{N_t\times S_k}$（$S_k=|\mathcal M_k[t]|$）为 $\operatorname{span}\{\mathbf h_{k,j}[t]\}_{j\in\mathcal M_k[t]}$ 的标准正交基。则

$$
\hat{\mathbf d}^{(\iota)}_{k,r}[t]=
\frac{\big(\mathbf I-\iota\,\mathbf Q_k[t]\mathbf Q_k[t]^H\big)\bar{\mathbf d}_{k,r}[t]}
{\big\|\big(\mathbf I-\iota\,\mathbf Q_k[t]\mathbf Q_k[t]^H\big)\bar{\mathbf d}_{k,r}[t]\big\|},
$$

并在式 (45) 中取 $\hat{\mathbf d}_{k,r}[t]=\hat{\mathbf d}^{(\iota)}_{k,r}[t]$。当 $\iota=0$ 时，(46) 退化为纯导向，$\hat{\mathbf d}^{(0)}_{k,r}[t]=\bar{\mathbf d}_{k,r}[t]/\|\bar{\mathbf d}_{k,r}[t]\|$；当 $\iota=1$ 时，它从 $\bar{\mathbf d}_{k,r}[t]$ 中去除其在所服务用户信道子空间上的投影。若 (46) 分母中的范数低于 $\epsilon\|\bar{\mathbf d}_{k,r}[t]\|$（$\epsilon>0$ 为小容差），则投影在数值上降秩，此时回退到未投影方向。只要该回退未被触发，所得波束即满足对每个 $j\in\mathcal M_k[t]$ 有 $\mathbf h_{k,j}^H[t]\mathbf w_{k,r}[t]=0$，从而 $I_m$ 中感知致干扰的 $k=k(m)$ 项消失，代价是感知阵列增益下降。该投影不改变 $\varsigma_k[t]$ 或功率预算，也不保护由其他无人机服务的用户。由于 $\hat{\mathbf d}_{k,r}[t]$ 由 $\mathbf h[t]$ 计算，该开关属于 $\mathcal X^{\rm II}[t]$；$\iota$ 按实验固定，不是被学习的动作。

### 3.2 性能指标

**通信指标。** 在用户权重 $\omega_m\ge 0$ 下，时隙 $t$ 的归一化加权频谱效率为

$$
\bar R[t]=\frac{1}{B\sum_{m\in\mathcal M}\omega_m}\sum_{m\in\mathcal M}\omega_m R_m[t]\quad[\mathrm{bit/s/Hz}],
$$

其中 $R_m[t]$ 见 2.4 节。按 $\sum_m\omega_m$ 归一化使 $\bar R[t]$ 与 $M$ 以及 $\omega$ 的尺度无关。

**感知指标。** 截断后的界 $\widetilde{\mathrm{CRB}}_n[t]=\min\{\mathrm{CRB}_n[t],\Gamma_{\rm cap}\}$ 按 $\Gamma_{\rm cap}$ 归一化：

$$
\bar C[t]=\frac{1}{|\mathcal N|\,\Gamma_{\rm cap}}\sum_{n\in\mathcal N}\widetilde{\mathrm{CRB}}_n[t]\in(0,1],
$$

其中 $\widetilde{\mathrm{CRB}}_n[t]$ 是式 (36) 的截断界在时隙 $t$ 的几何与协方差下的取值。目标是静止的，且该界仅通过 $\mathbf R_k[t]$ 依赖波束成形器；由于波束成形器使用 $\mathbf h[t]$，$\bar C[t]$ 因衰落而随机。该界逐时隙评估，故对静止目标所获信息不跨时隙累积，这相对于跟踪滤波器的累积界是保守的。相反，截断并不保守：一旦 $\mathrm{CRB}_n[t]>\Gamma_{\rm cap}$，指标报告的是 $\Gamma_{\rm cap}$ 而非真实界，因此在该区间内 $\bar C[t]$ 是乐观的。

### 3.3 问题陈述

以权衡权重 $\mu\ge 0$，任务级问题为

$$
\begin{aligned}
\max_{\mathcal{P}\in\mathfrak P}\quad & \mathbb E_{\mathcal{P}}\Big[\frac1T\sum_{t=0}^{T-1}\Big(\bar R[t]-\mu\,\bar C[t]\Big)\Big] \\
\text{s.t.}\quad & \|\mathbf p_k[t+1]-\mathbf p_k[t]\|\le v_{\max}\Delta t, \ \forall k,\,t\le T-2, \tag{C1}\\
& \mathbf p_k[t]\in\mathcal A\times[z_{\min},z_{\max}], \ \forall k,\,t, \tag{C2}\\
& \|\mathbf p_k[t]-\mathbf p_{k'}[t]\|\ge d_{\min}, \ \forall k\neq k',\,t, \tag{C3}\\
& \operatorname{tr}(\mathbf R_k[t])\le P_{\max}, \ \forall k,\,t, \tag{C4}
\end{aligned}
$$

其中期望针对用户移动性与小尺度衰落，约束须几乎必然成立，且 $\mathbf p_k[0]$ 已给定。

目标 (49) 是时间平均加权频谱效率减去 $\mu$ 倍的归一化定位界：$\mu=0$ 退化为纯通信，$\mu$ 越大则资源越转向感知。由于 $\bar R$ 以 bit/s/Hz 计而 $\bar C\in(0,1]$，$\mu$ 的取值需要相对 $\bar R$ 的典型量级来标定。约束 (C1) 与 (C2) 描述无人机运动学与飞行体积，(C3) 为防碰撞要求。功率预算 (C4) 限制各无人机由通信流与感知流共享的总功率，且须对每种衰落实现都成立。关联不属于策略的一部分：它由基于位置的规则 (17) 固定，在该规则下用户 $m$ 从唯一无人机 $k(m)$ 获取期望信号，而其他无人机发往其自身用户的流以及所有感知流都成为干扰。第 4 节在 3.1 节的受限类 $\mathfrak P_{\rm res}\subset\mathfrak P$ 上优化 (49)，故其取值是 $\mathfrak P$ 上最优值的下界。

### 3.4 难点

（正文尚未撰写，原文仅有引出句。）

---

## 4. Proposed Method（提出的方法，占位）

该节仅有模板说明文字与模板伪代码 Algorithm 1（“Proposed Algorithm”：初始化、分块更新、直至目标函数收敛），以及 `Overview`、`Iterative Update`（含式 `eq:update` 的分块子问题模板）、`Convergence and Complexity` 三个小节的说明性段落。正文尚未成稿，故不翻译。

---

## 5. Simulation Results（仿真结果）

### 5.1 仿真设置

仿真器是一个基于 Gymnasium 与 NumPy 的 Python 环境，其中 2–3 节的每个时隙都被直接实现：A2G 信道在当前位置上抽取，关联按式 (17) 执行，发射协方差由 3.1 节的波束成形器产生，时隙指标 $\bar R[t]$ 与 $\bar C[t]$ 由接收 SINR 与位置 CRB 评估。每个回合开始时，用户与目标在服务区域内均匀随机投放，无人机从飞行体积内的可行位置出发，其后位置在 (C1)–(C3) 下演化。除另有说明外，采用表 1 的参数，$\mu=10$，零空间开关 $\iota=1$，每个报告值为在独立回合（不同随机种子）上的平均。纯导向情形 $\iota=0$ 作为消融基线；它只改变这一开关，而随机种子、轨迹与功率占比均相同。

在 $\mu=10$ 时，(49) 的两项量级相当，权衡处于活跃状态；更小的 $\mu$ 使工作点移向纯通信。

**表 1 仿真参数**

| 参数 | 取值 | 参数 | 取值 |
|---|---|---|---|
| $K$, $M$, $N$ | 4, 8, 4 | $N_t$, $d_a$ | 10, $\lambda/2$ |
| $\mathcal A$ | $1\times1$ km$^2$ | $z_{\min}$, $z_{\max}$ | 100, 120 m |
| $d_{\min}$ | 20 m | $\Delta t$ | 0.1 s |
| $T$ | 1000 | $v_{\max}$ | 30 m/s |
| $v^{\rm u}_{\max}$ | 5 m/s | $\sigma_{\rm u}$ | 1 m/s |
| $\tau_{\rm v}$ | 5 s | $f_c$ | 28 GHz |
| $B$ ($B_{\rm rms}$) | 100 MHz ($B/\sqrt{12}$) | $P_{\max}$ | 30 dBm |
| $\sigma^2$, $\sigma_r^2$ | $-84$ dBm | $T_s$ | 1 ms |
| $N_s=BT_s$ | $10^5$ | $\sigma_{\rm rcs}$ | 1 m$^2$ |
| $\kappa$ | 10 dB | $a$, $b$ | 9.61, 0.16 |
| $\eta_{\rm L}$, $\eta_{\rm N}$ | 2, 3 | $d_0$ | 1 m |
| $\Gamma_{\rm cap}$ | 100 m$^2$ | $\mu$, $\iota$ | 10, 1 |

**图 3（`fig:demo_env`）**：仿真环境的示意图。（图件为 `results/demo_env_fig1.png`，标题仍为占位）

### 5.2 数值结果

（正文仍为模板说明：如何组织三点式对比表 `tab:comparison`——该表为示例表，内容为 `--` 占位，未翻译。）
