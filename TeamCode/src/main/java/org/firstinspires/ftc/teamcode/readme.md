# TeamCode 模块 | TeamCode Module

欢迎来到 **27570 "INTO THE DEEP" 赛季**的 `TeamCode` 模块！
Welcome to the `TeamCode` module for the **27570 "INTO THE DEEP" Season**!

这个目录是存放我们队伍所有自定义机器人代码的核心位置，包括自主程序、遥控操作逻辑、以及各种工具和算法库。
This directory is the central hub for all our team's custom robot code, including autonomous programs, tele-operated logic, and various utilities and algorithm libraries.

## 主要功能模块 | Key Feature Modules

以下是我们队伍在 `TeamCode` 中开发和维护的主要功能模块：
Below are the primary feature modules developed and maintained by our team within `TeamCode`:

*   **`pedroPathing/`**
    *   **中文**: 包含我们先进的自主导航系统。该系统支持复杂的路径跟随、基于贝塞尔曲线的平滑轨迹生成、路径链组合、状态机控制以及模块化的机器人动作。
    *   **English**: Contains our advanced autonomous navigation system. This system supports sophisticated path following, smooth trajectory generation using Bezier curves, path chaining, state machine control, and modular robot actions.

*   **`vision/`**
    *   **中文**: 存放我们的智能视觉抓取系统。该系统利用OpenCV进行目标识别与定位，采用分层架构，并能在TeleOp模式下提供智能辅助。更详细的文档请参见 `vision/readme.md`。
    *   **English**: Houses our intelligent vision grasping system. This system utilizes OpenCV for target identification and localization, features a layered architecture, and provides smart assisted grasping in TeleOp mode. For more detailed documentation, please refer to `vision/readme.md`.

*   **`API/`**
    *   **中文**: 提供核心的机器人运动学和位置计算工具。例如 `ServoKinematics.java` 用于伺服驱动机构的运动学解算，`PositionCalculator.java` 可能用于更通用的位置相关计算。
    *   **English**: Provides core utilities for robot kinematics and position calculations. For example, `ServoKinematics.java` is used for the kinematic solutions of servo-driven mechanisms, and `PositionCalculator.java` may be used for more general position-related computations.

*   **`APIuser/`**
    *   **中文**: 包含基于 `API/` 中工具开发的上层应用或特定机构的控制器。例如 `Degree2Pos.java` 用于角度到伺服位置的转换，`SlideControl.java` 用于控制线性滑轨等。
    *   **English**: Contains higher-level applications or controllers for specific mechanisms, likely developed using the tools in `API/`. For instance, `Degree2Pos.java` handles angle-to-servo position conversions, and `SlideControl.java` is for controlling linear slides.

*   **`ReadEncoder.java`**
    *   **中文**: 这是一个工具性的OpMode，用于读取并显示机器人驱动电机的编码器值，方便调试和标定。
    *   **English**: This is a utility OpMode designed to read and display the encoder values of the robot's drive motors, useful for debugging and calibration.

## 使用与贡献 | Usage and Contribution

请在开发新功能或修改现有代码时，遵循团队的编码规范和版本控制流程。我们鼓励模块化设计和清晰的文档注释。
When developing new features or modifying existing code, please adhere to the team's coding standards and version control practices. We encourage modular design and clear documentation.

祝编程愉快！
Happy coding!