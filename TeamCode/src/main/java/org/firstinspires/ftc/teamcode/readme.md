# TeamCode 模块 | TeamCode Module

Welcome to the `TeamCode` module for the **27570 "INTO THE DEEP" Season**! This directory is the central hub for all our team's custom robot code, including autonomous programs, tele-operated logic, and various utilities and algorithm libraries.

## Table of Contents
- [Project Overview | 项目概述](#project-overview--项目概述)
- [Directory Structure | 目录结构](#directory-structure--目录结构)
- [Key Technologies | 关键技术](#key-technologies--关键技术)
- [Main Feature Modules | 主要功能模块](#main-feature-modules--主要功能模块)
  - [Core API (`API/`) | 核心接口](#core-api-api--核心接口)
  - [API User/Examples (`APIuser/`) | API应用与示例](#api-userexamples-apiuser--api应用与示例)
  - [Vision System (`vision/`) | 视觉系统](#vision-system-vision--视觉系统)
  - [PedroPathing Autonomous Navigation (`pedroPathing/`) | 自主导航库](#pedropathing-autonomous-navigation-pedropathing--自主导航库)
  - [Encoder Reading Utility (`ReadEncoder.java`) | 编码器读取工具](#encoder-reading-utility-readencoderjava--编码器读取工具)
  - [OpModes (Autonomous & TeleOp) | 操作模式](#opmodes-autonomous--teleop--操作模式)
- [Technical Highlights and Design Principles | 技术亮点与设计原则](#technical-highlights-and-design-principles--技术亮点与设计原则)
- [Getting Started | 快速上手](#getting-started--快速上手)
- [Usage and Contribution | 使用与贡献](#usage-and-contribution--使用与贡献)

## Project Overview | 项目概述
**中文**: 本项目是为 FIRST Tech Challenge (FTC) 27570队伍在 "INTO THE DEEP" 赛季开发的机器人控制代码。旨在实现一个功能强大、可靠且易于维护的机器人，能够高效完成赛季挑战中的各项任务，包括精准的自主导航、智能的目标识别与操作，以及流畅的遥控体验。
**English**: This project contains the robot control code for FTC Team 27570 for the "INTO THE DEEP" season. It aims to develop a robust, reliable, and maintainable robot capable of efficiently performing season-specific tasks, including precise autonomous navigation, intelligent object recognition and manipulation, and a seamless tele-operated experience.

## Directory Structure | 目录结构
- **`TeamCode/src/main/java/org/firstinspires/ftc/teamcode/`**
  - **`API/`**: (中文) 核心底层工具与算法。 | (English) Core low-level utilities and algorithms.
    - `PositionCalculator.java`: (中文) 通用位置/值归一化计算器。 | (English) General-purpose position/value normalization calculator.
    - `ServoKinematics.java`: (中文) 伺服驱动连杆机构的运动学解算。 | (English) Kinematic solver for servo-driven linkage mechanisms.
  - **`APIuser/`**: (中文) `API`模块的应用层封装和示例代码。 | (English) Application layer wrappers and example code using the `API` module.
    - `Degree2Pos.java`: (中文) `PositionCalculator` 的使用示例和测试。 | (English) Usage examples and tests for `PositionCalculator`.
    - `SlideControl.java`: (中文) 基于`ServoKinematics`的滑轨控制高级接口。 | (English) High-level interface for slide control based on `ServoKinematics`.
  - **`vision/`**: (中文) 智能视觉处理系统。详细信息请参阅 [`vision/readme.md`](vision/readme.md)。 | (English) Intelligent vision processing system. See [`vision/readme.md`](vision/readme.md) for details.
    - `GraspingCalculator.java`, `VisionGraspingAPI.java`, `pipeline/` (and sub-components).
  - **`pedroPathing/`**: (中文) 高级自主导航库。 | (English) Advanced autonomous navigation library.
    - `AutoCode/`: (中文) 使用此库的自主OpMode。 | (English) Autonomous OpModes utilizing this library.
    - `TeleOp/`: (中文) 可能包含与路径相关的遥控界面或测试。 | (English) May contain TeleOp interfaces or tests related to pathing.
    - `constants/`: (中文) 路径、PID等导航参数配置。 | (English) Configuration for pathing, PID, and other navigation parameters.
    - `examples/`: (中文) 库功能使用示例。 | (English) Examples demonstrating library features.
    - `tuners_tests/`: (中文) PID调优工具和路径测试程序。 | (English) PID tuning utilities and path testing programs.
  - `ReadEncoder.java`: (中文) 读取和显示电机编码器值的工具性OpMode。 | (English) Utility OpMode for reading and displaying motor encoder values.
  - `(Your Main Autonomous and TeleOp OpModes)`: (中文) 存放主要的比赛用自主和遥控程序。 | (English) Location for primary competition Autonomous and TeleOp programs.

## Key Technologies | 关键技术
- **FTC SDK**: (中文) FIRST Tech Challenge 官方软件开发工具包。 | (English) Official FTC Software Development Kit.
- **Java**: (中文) 主要编程语言。 | (English) Primary programming language.
- **EasyOpenCV / OpenCV**: (中文) 用于高级视觉处理。 | (English) For advanced vision processing.
- **PedroPathingLib**: (中文) (假设) 自主导航核心库。 | (English) (Assumed) Core library for autonomous navigation.

## Main Feature Modules | 主要功能模块

### Core API (`API/`) | 核心接口
This module provides foundational utilities for robot control.
*   **`PositionCalculator.java`**
    *   **中文**: 提供一个静态方法 `calculatePositionValue`，根据最小/最大位置、旋转角度、用户输入和反向标志来计算归一化的位置值 (0.0 到 1.0)。包含对零旋转角度的错误处理。此工具对于将传感器读数或手柄输入等不同输入缩放和映射到标准化范围以用于执行器或其他计算非常有用。
    *   **English**: Provides a static method `calculatePositionValue` to compute a normalized position value (0.0 to 1.0) based on min/max positions, a rotation angle, user input, and a reverse flag. Includes error handling for a zero rotation angle. Useful for scaling and mapping various inputs (like sensor readings or joystick values) to a standardized range for actuators or other calculations.
*   **`ServoKinematics.java`**
    *   **中文**: 为伺服驱动的连杆机构（如自定义滑轨）提供精确的运动学解算。核心方法 `calculateServoTarget(double targetExtensionCm)` 输入期望的连杆伸出距离 (cm)，输出伺服电机所需旋转角度和对应的逻辑位置 (0.0-1.0)。它基于详细的几何常量（曲柄臂、连杆长度、偏移量）和三角函数（余弦定理, `atan2`）进行计算，并包含对不可达目标的错误处理。`ServoTarget`内部类用于封装计算结果。该模块是实现精确机械控制的关键。
    *   **English**: Provides precise kinematic solutions for servo-driven linkages (e.g., custom slides). The core method `calculateServoTarget(double targetExtensionCm)` takes a desired linear extension (cm) and returns the required servo motor rotation angle and corresponding logical position (0.0-1.0). Calculations are based on detailed geometric constants (crank arm, connecting rod lengths, offsets) and trigonometry (law of cosines, `atan2`), including error handling for unreachable targets. The `ServoTarget` inner class encapsulates results. Key for precise mechanical control.

### API User/Examples (`APIuser/`) | API应用与示例
This module demonstrates and builds upon the `API/` functionalities.
*   **`Degree2Pos.java`**
    *   **中文**: 主要作为 `PositionCalculator` 的测试和示例程序。其 `main` 方法通过多组不同参数调用 `calculatePositionValue`，展示了其功能和归一化逻辑，包括反向计算和错误情况。
    *   **English**: Primarily a test and example program for `PositionCalculator`. Its `main` method calls `calculatePositionValue` with various parameters, demonstrating its functionality, normalization logic, including reversal and error cases.
*   **`SlideControl.java`**
    *   **中文**: 提供了一个更高级的接口 `moveToExtension(double desiredExtensionCm)` 来控制基于 `ServoKinematics` 的滑轨机构。它封装了调用运动学解算并获取目标舵机位置的逻辑，简化了上层代码（如OpModes）对滑轨的控制。
    *   **English**: Offers a higher-level interface `moveToExtension(double desiredExtensionCm)` for controlling a slide mechanism using `ServoKinematics`. It encapsulates the logic of calling the kinematic solver and retrieving the target servo position, simplifying slide control in higher-level code (e.g., OpModes).

### Vision System (`vision/`) | 视觉系统
**中文**: 一个复杂的智能视觉系统，用于在比赛中识别和定位目标物体（如赛季元素）。它利用OpenCV（通过EasyOpenCV集成）实现高效的图像处理流程，包括颜色分割、形态学滤波、轮廓检测和智能决策。系统采用分层架构，核心组件包括：
    - `VisionGraspingAPI`: 作为视觉系统的外观（Facade），简化与OpModes的交互。
    - `pipeline/VisionPipeline` & `pipeline/processing/DetectionProcessor`: 执行核心图像处理和目标决策逻辑。
    - `GraspingCalculator`: 将视觉数据转换为精确的伺服指令，并实现了非线性距离校正和透视失真补偿等高级算法，以提高抓取精度。
    - `VisionConstants`: 集中管理所有视觉相关参数，便于调试和优化。
    该系统不仅支持自主模式下的目标定位，还能在TeleOp模式下提供驾驶员辅助（如建议机器人移动方向）。详细架构和技术细节请参阅 [`vision/readme.md`](vision/readme.md)。
**English**: A sophisticated intelligent vision system for identifying and locating game objects. It leverages OpenCV (via EasyOpenCV) for an efficient image processing pipeline including color segmentation, morphological filtering, contour detection, and intelligent decision-making. Key components include:
    - `VisionGraspingAPI`: Acts as a facade, simplifying interaction with OpModes.
    - `pipeline/VisionPipeline` & `pipeline/processing/DetectionProcessor`: Execute core image processing and target decision logic.
    - `GraspingCalculator`: Converts vision data into precise servo commands, implementing advanced algorithms like non-linear distance correction and perspective distortion compensation for improved accuracy.
    - `VisionConstants`: Centralizes all vision-related parameters for easy tuning.
    Supports target localization in autonomous and provides driver assistance in TeleOp. For detailed architecture, see [`vision/readme.md`](vision/readme.md).

### PedroPathing Autonomous Navigation (`pedroPathing/`) | 自主导航库
**中文**: 一个功能强大的自主导航库，为机器人提供高级路径规划和运动控制能力。其模块化设计体现在以下子目录中：
    - `AutoCode/`: 存放利用此库实现的各种自主程序（OpModes），执行复杂的预设路径和动作序列。
    - `TeleOp/`: 可能包含用于测试路径段、校准或与路径系统交互的遥控OpMode。
    - `constants/`: 集中管理导航相关参数，如机器人物理参数、PID控制增益、运动速度/加速度限制、路径跟踪容差等，方便调优。
    - `examples/`: 提供库各项功能的使用示例，帮助理解和集成。
    - `tuners_tests/`: 包含用于调试和优化导航性能的工具，例如PID控制器调优程序、里程计校准工具、路径精度测试脚本等。
    该库可能支持的技术包括：基于里程计或外部传感器的实时定位、平滑轨迹生成（如贝塞尔曲线、样条曲线）、精确的路径跟随算法（如Pure Pursuit、Ramsete）、以及用于管理复杂自主逻辑的状态机。
**English**: A powerful autonomous navigation library providing advanced path planning and motion control. Its modular design is reflected in its subdirectories:
    - `AutoCode/`: Contains autonomous OpModes implementing complex pre-defined paths and action sequences.
    - `TeleOp/`: May include TeleOp OpModes for testing path segments, calibration, or interacting with the pathing system.
    - `constants/`: Centralizes navigation-related parameters like robot physical specs, PID gains, motion profiles (velocity/acceleration), and path following tolerances for easy tuning.
    - `examples/`: Provides usage examples for various library features.
    - `tuners_tests/`: Includes utilities for debugging and optimizing navigation performance, such as PID tuners, odometry calibration tools, and path accuracy test scripts.
    Likely supports features like real-time localization (odometry/sensors), smooth trajectory generation (Bezier curves, splines), precise path following algorithms (Pure Pursuit, Ramsete), and state machines for complex autonomous logic.

### Encoder Reading Utility (`ReadEncoder.java`) | 编码器读取工具
**中文**: 一个诊断性的 `LinearOpMode`，用于读取并实时显示机器人驱动电机（左前、右前、左后、右后）的编码器计数值。此工具在机器人初始化时重置编码器，并将电机设置为 `RUN_USING_ENCODER` 模式。在循环中，它通过Telemetry向Driver Station发送各电机的当前编码器位置。这对于验证电机接线、编码器功能、确定齿轮比或每英寸/厘米的脉冲数 (ticks per inch/cm) 以及初步的运动系统调试至关重要。
**English**: A diagnostic `LinearOpMode` for reading and displaying drive motor encoder ticks (left/right front, left/right back) in real-time. It resets encoders on initialization and sets motors to `RUN_USING_ENCODER` mode. In its loop, it telemeters current encoder positions to the Driver Station. Crucial for verifying motor wiring, encoder functionality, determining gear ratios or ticks per inch/cm, and initial motion system debugging.

### OpModes (Autonomous & TeleOp) | 操作模式
**中文**: [请在此处简要描述主要的自主和遥控OpMode的用途和关键功能。例如：“RedAllianceFullAuto.java - 完成红色联盟场地上从启动到停泊的完整自主序列，包括识别和放置N个元素。” 或 “MainTeleOp.java - 实现所有基本驱动功能，以及通过按钮控制所有机械臂和抓取机构。”]
**English**: [Briefly describe the purpose and key features of your main Autonomous and TeleOp OpModes here. E.g., "RedAllianceFullAuto.java - Completes the full autonomous sequence for the red alliance, from start to park, including identifying and placing N elements." or "MainTeleOp.java - Implements all basic drive functions, plus button controls for all arms and grippers."]

## Technical Highlights and Design Principles | 技术亮点与设计原则
- **Modularity (模块化)**: Code is organized into distinct, reusable modules (`API`, `vision`, `pedroPathing`) with clear responsibilities, promoting maintainability and parallel development.
- **Abstraction (抽象化)**: Higher-level classes (`SlideControl`) simplify complex underlying systems (`ServoKinematics`), making OpMode logic cleaner and more intuitive.
- **Data-Driven Design (数据驱动)**: Calculations like `PositionCalculator` and extensive use of constants in `vision` and `pedroPathing` allow for easy tuning and adaptation without deep code changes.
- **Robust Kinematics & Control (稳健的运动学与控制)**: The `ServoKinematics` module demonstrates a strong foundation in mechanical control, including handling edge cases and physical constraints.
- **Advanced Algorithms (高级算法)**: Implementation of sophisticated algorithms for vision processing (e.g., distortion correction in `GraspingCalculator`) and potentially for path planning and following in `pedroPathing`.
- **Comprehensive Utilities (完善的工具集)**: Tools like `ReadEncoder.java` and test/tuner suites within `pedroPathing` significantly aid in debugging, calibration, and performance optimization.
- **Clear Separation of Concerns (关注点分离)**: Logic for hardware interaction, algorithms, and high-level robot behavior are kept distinct.

## Getting Started | 快速上手
1.  **中文**: 确保已安装 Android Studio 及最新版 FTC SDK。 | **English**: Ensure Android Studio and the latest FTC SDK are installed.
2.  **中文**: 克隆本仓库到本地计算机。 | **English**: Clone this repository to your local machine.
3.  **中文**: 在 Android Studio 中导入项目 (通常是选择项目根目录的 `build.gradle` 或项目文件夹本身)。 | **English**: Import the project in Android Studio (usually by selecting the root `build.gradle` or the project folder).
4.  **中文**: 等待 Gradle 同步完成。构建项目 (`Build > Make Project`)。 | **English**: Wait for Gradle sync to complete. Build the project (`Build > Make Project`).
5.  **中文**: 将代码部署到机器人控制器手机或控制中心。 | **English**: Deploy the code to the Robot Controller phone or Control Hub.
6.  **中文**: [可选但推荐] 根据需要执行校准程序，例如摄像头校准 (`vision/`), 电机编码器和方向检查 (`ReadEncoder.java`), PID调优 (`pedroPathing/tuners_tests/`)。 | **English**: [Optional but Recommended] Perform calibration routines as needed, e.g., camera calibration (`vision/`), motor encoder/direction checks (`ReadEncoder.java`), PID tuning (`pedroPathing/tuners_tests/`).

## Usage and Contribution | 使用与贡献
**中文**: 在开发新功能或修改现有代码时，请遵循团队的编码规范和版本控制流程。我们鼓励模块化设计和清晰的文档注释。
**English**: When developing new features or modifying existing code, please adhere to the team's coding standards and version control practices. We encourage modular design and clear documentation.

*   **编码规范 | Coding Standards**: 
    *   **中文**: [请在此处填写团队的具体编码规范，例如：遵循标准Java命名约定，使用驼峰命名法，每个方法和复杂逻辑块前添加JavaDoc注释，常量使用全大写蛇形命名法]。
    *   **English**: [Please fill in specific team coding standards here, e.g., Follow standard Java naming conventions, use camelCase for methods/variables, add JavaDoc comments before each method and complex logic blocks, use UPPER_SNAKE_CASE for constants].
*   **版本控制 | Version Control (Git)**:
    *   **中文**: 主分支为 `main` (或 `master`)。开发新功能或修复bug应在单独的特性分支上进行 (例如 `feature/new-vision-algorithm` 或 `fix/pathing-bug`)。
    *   **English**: The main branch is `main` (or `master`). Develop new features or fix bugs on separate feature branches (e.g., `feature/new-vision-algorithm` or `fix/pathing-bug`).
    *   **中文**: 提交前确保代码能够编译通过。编写清晰、简洁且有意义的提交信息。
    *   **English**: Ensure code compiles before committing. Write clear, concise, and meaningful commit messages.
    *   **中文**: 定期从主分支拉取更新到特性分支 (`git pull origin main`) 以避免大的合并冲突。
    *   **English**: Regularly pull updates from the main branch into your feature branch (`git pull origin main`) to avoid large merge conflicts.
    *   **中文**: 完成开发后，发起 Pull Request (PR) 到 `main` 分支，请求至少一位其他团队成员进行代码审查。
    *   **English**: Once development is complete, create a Pull Request (PR) to the `main` branch and request a code review from at least one other team member.
*   **构建与部署 | Building and Deploying**:
    *   **中文**: 使用 Android Studio 的 `Run 'TeamCode'` 配置将代码部署到机器人控制器。
    *   **English**: Use the `Run 'TeamCode'` configuration in Android Studio to deploy code to the Robot Controller.
*   **测试 | Testing**:
    *   **中文**: 鼓励编写单元测试（如果适用）。对OpModes进行充分的场上测试，特别是在修改了核心模块（如`pedroPathing`或`vision`）之后。
    *   **English**: Unit testing is encouraged where applicable. Thoroughly test OpModes on the field, especially after modifying core modules like `pedroPathing` or `vision`.

祝编程愉快！
Happy coding!