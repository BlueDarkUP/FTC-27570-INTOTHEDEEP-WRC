# FTC 视觉抓取系统 (Vision Grasping System)

**作者:** BlueDarkUP

## 1. 系统概述

本项目是一个为FTC机器人设计的、高度模块化和智能化的视觉抓取系统。它利用OpenCV进行图像处理，能够精确识别视野中的目标（例如，特定颜色的立方体），计算其在三维空间中的位置和姿态，并最终生成驱动机械臂进行抓取的舵机指令。

该系统的核心特点是**分层架构**和**智能辅助**，旨在将复杂的视觉处理逻辑与机器人的主控制逻辑解耦，并为驾驶员提供清晰、可操作的行动建议，极大地提高了手动操作模式（TeleOp）下的抓取效率和成功率。

## 2. 系统架构

本系统采用清晰的分层架构设计，每一层各司其职，通过定义良好的数据对象进行通信。

```
+---------------------------------+
|   VisionGraspingTeleOp.java     |  <-- 顶层: 用户交互与决策
+---------------------------------+
              |
              | (获取 VisionTargetResult, 发送 GraspingTarget)
              v
+---------------------------------+
| VisionGraspingCalculator.java   |  <-- 中层: 核心计算与运动学反解
+---------------------------------+
              |
              | (接收 VisionTargetResult)
              v
+---------------------------------+
|     VisionGraspingAPI.java      |  <-- 门面: 封装所有视觉逻辑
+---------------------------------+
              |
              | (调用, 更新 VisionTargetResult)
              v
+---------------------------------+
|      SamplePipeline.java        |  <-- 底层: 图像处理与目标识别
+---------------------------------+
              |
              | (读取配置)
              v
+---------------------------------+
|      VisionConstants.java       |  <-- 基础: 所有配置与参数
+---------------------------------+
```

### 各模块职责

-   **`VisionGraspingTeleOp.java`**:
    -   **角色**: 总指挥。
    -   **职责**: 集成所有模块，处理驾驶员输入，根据视觉结果和计算结果，调用动作库执行高级指令。

-   **`VisionGraspingCalculator.java`**:
    -   **角色**: 大脑/计算核心。
    -   **职责**: 接收视觉系统输出的物理数据，通过一系列校准和补偿算法，计算出最终的舵机目标位置。

-   **`VisionGraspingAPI.java`**:
    -   **角色**: 门面/接口。
    -   **职责**: 封装所有与EasyOpenCV和摄像头相关的复杂初始化、管理和线程同步逻辑。对外提供一个极其简洁的接口 (`getLatestResult()`)。

-   **`SamplePipeline.java`**:
    -   **角色**: 眼睛/图像处理器。
    -   **职责**: 实现`OpenCvPipeline`，处理每一帧图像，包括颜色分割、轮廓检测、多级过滤，并最终生成包含丰富信息的`VisionTargetResult`。

-   **`VisionConstants.java`**:
    -   **角色**: 规则书/配置中心。
    -   **职责**: 集中管理所有可调参数，如摄像头参数、颜色范围、尺寸阈值、物理标定值等。

-   **数据类 (`.java`)**:
    -   `VisionTargetResult`: 视觉系统对外的最终输出，不可变，线程安全。
    -   `GraspingTarget`: 计算器对内的最终输出，包含舵机指令。
    -   `DetectedCube`, `CandidateInfo`, `TargetZoneInfo`: Pipeline内部用于数据流转的中间对象。

## 3. 如何配置与标定

要使此系统正常工作，必须进行精确的配置和标定。所有参数均位于 `VisionConstants.java` 文件中。

### 3.1 摄像头配置
1.  **`WEBCAM_NAME_STR`**: 确保此字符串与您在机器人配置文件中为摄像头设置的名称完全一致。
2.  **`CAMERA_WIDTH`, `CAMERA_HEIGHT`**: 设置为您希望摄像头使用的分辨率。推荐使用16:9的比例，如 `1280x720`。

### 3.2 核心物理标定 (最重要!)
1.  **`REAL_WORLD_VIEW_WIDTH_CM`**:
    -   **目的**: 计算 `PIXELS_PER_CM`，这是所有物理距离计算的基石。
    -   **方法**:
        1.  将机器人放置在一个平面上，正对一面墙。
        2.  让机器人的抓取基准点（例如，爪子的中心）距离墙面一个已知的、合适的距离（例如30厘米）。
        3.  在手机的摄像头预览中，观察视野的最左侧和最右侧边缘在墙上对应的点。
        4.  用尺子测量这两个点之间的**实际物理宽度（厘米）**。
        5.  将测量值填入此常量。

### 3.3 颜色标定
-   **`COLOR_HSV_RANGES`**:
    -   **目的**: 精确地识别目标颜色。
    -   **方法**: 使用FTC Dashboard或其他支持实时HSV调参的工具。
        1.  将 `ENABLE_DEBUG_VIEW` 设为 `true`。
        2.  将要识别的目标物体放置在摄像头前，确保光照条件与比赛场地相似。
        3.  在调试视图中，调整HSV的上下限（`new Scalar(H, S, V)`)，直到目标物体被清晰地高亮显示，且背景干扰最少。
        4.  将最终确定的HSV范围填入此`Map`中。注意，如果颜色（如红色）跨越了HSV色环的180°边界，需要为其定义两个范围。

### 3.4 目标物体与区域配置
-   调整 `TARGET_OBJECT_*` 常量以匹配您要抓取的物体的实际形状（长宽比）。
-   调整 `TARGET_RECT_*` 常量以定义屏幕上您认为理想的“可抓取区域”的物理尺寸和位置。

## 4. 如何使用

在您的 `TeleOp` 或 `Autonomous` 程序中集成此系统非常简单：

1.  **创建实例**:
    ```java
    private VisionGraspingAPI visionAPI;

    // 在 opModeIsActive() 之前
    visionAPI = new VisionGraspingAPI();
    visionAPI.init(hardwareMap);
    ```

2.  **在主循环中获取结果**:
    ```java
    // 在 while(opModeIsActive()) 循环内
    VisionTargetResult result = visionAPI.getLatestResult();

    if (result.isTargetFound) {
        // 目标在抓取区内，可以进行计算
        GraspingTarget grasp = VisionGraspingCalculator.calculate(result, telemetry);
        if (grasp.isInRange) {
            // 执行抓取动作...
        }
    } else {
        // 根据 result.nextTargetHorizontalOffsetCm 提供移动建议
    }
    ```

3.  **结束时释放资源 (必须!)**:
    ```java
    // 在OpMode结束时（例如，在主循环之后）
    visionAPI.close();
    ```

详细的用法请参考 `VisionGraspingTeleOp.java`，它是一个完整的、最佳实践的实现范例。

## 5. 性能调优
-   如果感觉视觉处理导致机器人响应变慢（FPS低），可以尝试减小 `VisionConstants.DOWNSCALE_FACTOR` 的值（例如，`0.5`）。这将以轻微的精度损失为代价，显著提高处理速度。
-   在正式比赛时，将 `ENABLE_DEBUG_VIEW` 设为 `false`，可以减少不必要的绘制开销。

---
**Happy coding!**