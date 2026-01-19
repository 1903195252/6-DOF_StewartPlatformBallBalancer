Stewart 平台板球平衡机器人 (6-DOF Stewart Platform Ball Balancer)

🚀 项目简介

本项目旨在开发一款基于 6自由度 Stewart 并联机构 的桌面级高精度板球平衡机器人。该系统不仅是对经典控制理论（PID、LQR）的实践验证，更是对现代机器人学中逆运动学解算、多变量耦合控制以及自适应系统辨识的深度探索。

通过精确控制 6 个伺服电机的协同运动，该机器人能够实时调整上方平台的姿态（Pitch & Roll），克服摩擦力、通信延迟和传感器噪声，使自由滚动的小球能够：

稳态平衡：在任意指定位置保持静止。

动态追踪：精准跟随复杂的 2D 轨迹（如 8 字形 Lissajous 曲线）。

抗干扰：在受到外部推力干扰后迅速恢复平衡。

🛠️ 技术栈与核心特性

1. 机械结构 (Mechanical Design)

构型：6-RSS (Rotary-Spherical-Spherical) 旋转式 Stewart 并联机构。

布局：采用 成对分布 (Pairwise) 的非均匀底座布局，优化了特定方向的力矩输出和运动空间。

建模：基于 SolidWorks 进行全参数化设计，精确计算连杆长度与转动惯量。

2. 控制算法 (Control Architecture)

本项目采用了一套分层级的复合控制架构，实现了从感知到执行的闭环控制：

感知层 (State Estimation)：

Luenberger 状态观测器：融合传感器噪声数据，实时重构小球的位置与速度，提供平滑的微分信号。

决策层 (Advanced Control)：

LQI 控制器 (Linear Quadratic Integral)：在 LQR 最优控制的基础上引入积分项，有效消除了由摩擦力和机械虚位引起的稳态误差。

物理前馈 (Physics-based Feedforward)：基于轨迹规划计算理论加速度，反解所需倾角，大幅降低了动态追踪的相位滞后。

RLS 在线辨识 (Recursive Least Squares)：X/Y 双轴独立运行的递推最小二乘法，实时“学习”系统的物理参数（如板球动力学系数 K），赋予系统自适应能力。

执行层 (Actuation)：

向量化逆运动学 (Vectorized IK)：推导并优化了 6-RSS 机构的逆解公式，采用矩阵并行计算替代循环迭代，极大提升了运算效率。

3. 仿真与可视化 (Simulation & Visualization)

高保真物理引擎：模拟了干摩擦 (Stiction)、过程噪声、通信延迟 (40ms) 以及小球的转动惯量耦合效应。

7合1 实时仪表盘：MATLAB 仿真环境集成了 3D 动态渲染与 6 路实时数据流（位置追踪、姿态波形、控制分量分解、舵机角度、RLS 参数收敛曲线），实现了“数字孪生”级的监控体验。

📊 性能指标

自由度：6 DOF (本应用主要利用 Pitch/Roll/Z)

控制频率：200 Hz (仿真步长 5ms)

追踪精度：动态误差 < 2cm (在 8 字形轨迹下)

稳态误差：趋近于 0 (得益于 LQI 积分作用)

自适应速度：RLS 参数在 5-10 秒内收敛至真实值附近

📂 项目结构

.
├── Simulation/
│   ├── Stewart_Ball_Balancer.m   # 主仿真程序 (MATLAB)
│   └── Stewart_IK_Rotary.m       # 逆运动学解算函数
├── Hardware/
│   ├── SolidWorks/               # 3D 模型源文件
│   └── PCB/                      # Altium Designer 工程文件
└── Docs/
    ├── Kinematics_Derivation.pdf # 运动学数学推导
    └── Control_Block_Diagram.png # 控制系统框图


🌟 亮点展示

(此处可插入仿真生成的 GIF 动图，展示小球走 8 字形的流畅动作)

本项目展示了一个机器人工程专业学生从机械设计到嵌入式系统，再到高级算法实现的全栈开发能力。它不仅是一个有趣的桌面玩具，更是一个验证复杂控制理论的理想实验平台。
