# 闭环云台系统固件架构设计 (Tripod Head)

本项目采用了规范的单片机分层架构。通过图文结合的方式，快速厘清模块职责及核心控制流向。

---

## 一、 系统分层架构 (Architecture)

整个工程由下至上严格解耦为 **硬件外设层**、**系统组件层** 和 **应用层**，确保低耦合、高内聚。

```mermaid
graph TD
    subgraph 3_User_Layer["3. 应用层 (User)"]
        M["main.c / main.h 业务主控"]
        ISR["TIM2_IRQHandler 核心控制中断引擎"]
    end

    subgraph 2_System_Layer["2. 系统组件层 (System)"]
        D["Delay.c/h (微秒/毫秒阻塞与非阻塞延时)"]
        F["fifo.c/h (无阻塞串口环形缓冲池)"]
    end

    subgraph 1_Hardware_Layer["1. 硬件外设层 (Hardware)"]
        B["board.c/h (板级初始化抽象)"]
        O["OLED.c/h (OLED状态显示驱动)"]
        P["PID.c/h (核心伺服运算控制算法)"]
        E["Emm_V5.c/h (闭环步进: 张大头协议驱动)"]
        S["Serial.c/h / usart.c/h (串口DMA通信总线)"]
        T["Timer.c/h (定时器全局基准配置)"]
    end

    %% 层级间调用与关联关系
    M -. "依赖" .-> M
    M -- "外设初始化" --> B
    M -- "UI交互" --> O
    M -- "时序控制" --> D

    ISR -- "高频触发算法" --> P
    ISR -- "下发速度/位置" --> E
    E -- "封包收发" --> S
    S -- "出入缓冲" --> F
    T -- "硬件定时触发" --> ISR

    style 3_User_Layer fill:#f9f9f9,stroke:#333,stroke-dasharray: 5 5
    style 2_System_Layer fill:#f0f0f0,stroke:#333
    style 1_Hardware_Layer fill:#e3f2fd,stroke:#333,stroke-width:2px
```

---

## 二、 核心控制流可视 (Core Control Flow)

固件的大脑集中在 `User/main.c`。程序运行时主要分为 **主循环 (Super Loop)** 和 **高频定时器中断引擎** 两条线并发推进。

```mermaid
flowchart TD
    Start(["系统上电复位"]) --> Init["外设级联初始化 (board, OLED, Serial, Timers)"]
    Init --> StartTimers["Timer3_Start (启动时基并点亮OLED)"]
    StartTimers --> SuperLoop

    subgraph Super ["低频事件轮询：主循环 (Super Loop)"]
        SuperLoop["进入 while(1)"] --> CheckAngle["调用 Check_angle 回读X/Y绝对角度"]
        CheckAngle --> LimitCheck{"角度是否满量程越界?"}
        LimitCheck -- "是 (危急)" --> Emergency["Emm_V5_Stop_Now (紧急制动) <br/>挂起: Stop_flag=1"]
        LimitCheck -- "否 (正常)" --> OLED_Update
        Emergency --> OLED_Update["OLED 更新刷新显示云台物理角度"]
        OLED_Update --> CheckAngle
    end

    subgraph Hard_ISR ["硬实时多轴同步算法：高频中断引擎 (最高优先级)"]
        TIM2(["TIM2 周期更新中断触发"]) --> StopCheck{"Stop_flag 挂起保护状态?"}
        StopCheck -- "是 (挂起中)" --> ClearFlag
        StopCheck -- "否 (正常)" --> Calc["Vertical_out 算法求解目标位置 x_out, y_out"]
        Calc --> SendCmd["下发 X/Y轴 Emm_V5_Pos_Control 位置闭环指令"]
        SendCmd --> Sync["触发 Emm_V5_Synchronous_motion 多轴立刻同步执行"]
        Sync --> ClearFlag(["清除定时器中断标志位，退出中断"])
    end
```

---

## 三、 核心模块职责目录 (Module Dictionary)

### 3.1 应用层 (User)

- **`main.c/h`**：业务机入口。负责组件上电初始化、主循环任务流转调度、电机越限软保护判定（阈值比较），连接上下层模块逻辑。

### 3.2 系统组件层 (System)

- **`Delay.c/h`**：提供高精度的系统时间服务（特别是微秒级别），是满足外设通讯协议时序要求的保障。
- **`fifo.c/h`**：基于 Ring Buffer 环形队列思想封装。专门应对串口高频数据交互，有效防范连续指令堆叠时的数据重写与粘包丢失。

### 3.3 硬件外设层 (Hardware)

- **`board.c/h`**：板级抽象层，隔离底层硬件引脚与上层应用逻辑耦合，便于后期快速移植。
- **`PID.c/h`**：针对本云台应用深度优化的位置、速度闭环数学运算控制枢纽。
- **`Emm_V5.c/h`**：**张大头 5.0 闭环步进**指令驱动库。囊括停机、多轴同步控制与高效位置调控的 API 封装。
- **`OLED.c/h` & `OLED_Data.c/h`**：I2C/SPI 图形显示协议与绘制驱动，作为系统运行状态和传感数据的反馈窗口。
- **`Timer.c/h`**：系统心跳及硬实时控制基石。重构了 TIM2 等中断资源调度方案，主要用于支撑高精算法调度需求。
- **`Serial.c/h` & `usart.c/h`**：UART 数据链路设施层。依托中断/ DMA，非阻塞地完成底层收发任务并交付给数据缓冲区。
