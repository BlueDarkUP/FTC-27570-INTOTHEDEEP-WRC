# INTO THE DEEP FTC Robot Controller & TeamCode

## 项目简介 | Project Overview

本仓库包含 FIRST Tech Challenge (FTC) 2024-2025 赛季“INTO THE DEEP”官方机器人控制器SDK与团队自定义代码开发环境。支持 Android Studio，适合团队协作与新手快速上手。

This repository provides the official SDK and team code base for the 2024-2025 FIRST Tech Challenge (FTC) "INTO THE DEEP" season. It supports development via Android Studio for building and deploying robot control software.

---

## 目录结构 | Directory Structure

- `FtcRobotController/`：官方机器人控制器SDK源码（勿随意修改）
- `TeamCode/`：团队自定义代码与算法实现目录
- `doc/`：文档与媒体资源
- `libs/`：外部依赖库
- `build.gradle`/`settings.gradle` 等：Gradle构建相关文件

---

## 环境要求 | Requirements

- Android Studio Ladybug (2024.2) 或更高版本
- JDK 21
- 推荐Windows 10/11操作系统

---

## 快速上手 | Getting Started

1. **克隆仓库 | Clone the repository**
   ```bash
   git clone https://github.com/BlueDarkUP/FTC-27570-INTO-THE-DEEP.git
   ```
2. **用Android Studio导入项目 | Import with Android Studio**
   - 选择“Import project (Gradle, Eclipse ADT, etc.)”
3. **编译与部署 | Build & Deploy**
   - 连接你的FTC机器人控制器设备
   - 点击“Run”按钮部署APK到设备
4. **Blocks/OnBot Java 用户 | For Blocks/OnBot Java Users**
   - 参考[官方Blocks教程](https://ftc-docs.firstinspires.org/programming_resources/blocks/Blocks-Tutorial.html)
   - 无需本地下载本仓库

---

## 示例代码与文档 | Sample Code & Documentation

- 官方样例代码路径：`FtcRobotController/src/main/java/org/firstinspires/ftc/robotcontroller/external/samples`
- 团队代码示例与规范：`TeamCode/src/main/java/org/firstinspires/ftc/teamcode`
- 官方文档：[FTC Documentation](https://ftc-docs.firstinspires.org/index.html)
- Javadoc参考：[FTC Javadoc](https://javadoc.io/doc/org.firstinspires.ftc)

---

## 项目特色 | Project Features

### 高级自主导航系统 (`TeamCode/src/main/java/org/firstinspires/ftc/teamcode/pedroPathing`)
本团队在 `pedroPathing` 目录中集成并定制了一套先进的自主导航系统，其主要特性包括：
- **复杂路径跟随**: 基于 `com.pedropathing` 库（或类似自研/第三方库），实现精准的路径跟踪能力。
- **平滑轨迹生成**: 利用贝塞尔曲线（`BezierCurve`）和直线（`BezierLine`）动态生成平滑且精确的机器人运动轨迹。
- **路径链组合**: 支持将多个独立路径（`Path`）组合成路径链（`PathChain`），以执行复杂的连续自主任务。
- **状态机控制**: 通过精密的路径状态（`pathState`）管理和更新机制（如 `autonomousPathUpdate` 方法），实现对自主程序序列的鲁棒控制。
- **模块化动作库**: 封装常用机器人动作（如抓取、手臂控制）到 `AlgorithmLibrary` 中，简化上层逻辑，提高代码复用性。
- **位姿估计与导航**: 使用 `Pose` 对象（包含X, Y坐标和朝向角）进行机器人位姿的精确表示和场地导航。

### Advanced Autonomous Navigation (`TeamCode/src/main/java/org/firstinspires/ftc/teamcode/pedroPathing`)
The team has integrated and customized an advanced autonomous navigation system within the `pedroPathing` directory, featuring:
- **Sophisticated Path Following**: Utilizes the `com.pedropathing` library (or a similar custom/third-party library) for precise path tracking capabilities.
- **Smooth Trajectory Generation**: Dynamically generates smooth and accurate robot movements using Bezier curves (`BezierCurve`) and lines (`BezierLine`).
- **Path Chaining**: Supports combining multiple individual paths (`Path`) into path chains (`PathChain`) to execute complex, sequential autonomous tasks.
- **State Machine Control**: Employs a meticulous path state (`pathState`) management and update mechanism (e.g., the `autonomousPathUpdate` method) for robust control over autonomous program sequences.
- **Modular Action Library**: Encapsulates common robot actions (e.g., grabbing, arm control) into an `AlgorithmLibrary`, simplifying higher-level logic and enhancing code reusability.
- **Pose Estimation and Navigation**: Uses `Pose` objects (comprising X, Y coordinates and heading angle) for accurate representation of robot pose and field navigation.

### 辅助驾驶手动程序 (`TeamCode/src/main/java/org/firstinspires/ftc/teamcode/pedroPathing/TeleOp`)
本团队在 `TeleOp` 目录中编写了L2和L3自动驾驶程序以供适应不同的手动赛场环境，其特性和使用说明如下：
- **全自动L3自动驾驶**: 基于自动程序编写方法，加入侧边intake路径，针对自动预期8+Park的程序编写同时兼容7+park版本（可跳过部分路径），完成长达2分钟的手动阶段纯自动运行（包含爬升）。
- **向心力控制**: `pedroPathing`自带的手动程序，开启向心力控制提升手动操作手感。
- **抓取自动纠位**: 当使用视觉抓取样本时若目标不在可抓取范围内则会横向移动底盘使其进入可抓取范围（请勿在朝向不正确时运行，可能会导致过冲撞墙等未知危险！）。
- **自动得分**: 先按下dpad右键以重新构建坐标系（注意准确性！这几乎无异于自动阶段前的摆车，朝向不正确或位置偏差过大会造成不小的影响！），再到得分点将所有挂在高杆上的标本全部推向左边，点击dpad右键即可开启自动得分，若出现任何异常立刻拨动左摇杆以接管自动驾驶，滑轨和大臂将会自动复位。
- **紧急复位**: 由于开启程序时可能电机初始位置不一定是零位，点击触控板即可强行重置。
## TeleOperated with Driver Assistance (`TeamCode/src/main/java/org/firstinspires/ftc/teamcode/pedroPathing/TeleOp`)

Our team has developed L2 and L3 driver assistance programs within the `TeleOp` directory to adapt to various manual field environments. Their features and usage instructions are as follows:

* **Full Autonomous L3 Driving**: This program is based on our autonomous routine development, incorporating a side intake path. It's designed for an expected 8+Park autonomous phase but also supports a 7+Park version (allowing certain paths to be skipped). It enables fully autonomous operation during the 2-minute teleoperated period, including climbing.
* **Centripetal Force Control**: The built-in teleoperated program within `pedroPathing` features centripetal force control, enhancing the feel of manual operation.
* **Automatic Gripper Alignment**: When using vision to pick up samples, if the target is not within the gripper's reach, the chassis will automatically strafe horizontally to bring it into range. **(Caution: Do not run this feature if the robot's orientation is incorrect, as it may lead to overshoots, wall collisions, or other unforeseen hazards!)**
* **Automatic Scoring**: First, press the **D-pad Right** button to re-establish the coordinate system. **(Note: Accuracy is crucial! This is almost identical to the robot's starting position before the autonomous phase. Incorrect orientation or significant position deviation can have a considerable impact!)** Then, at the scoring location, push all game elements hanging on the high pole to the left side. Click **D-pad Right** again to initiate automatic scoring. Should any anomaly occur, immediately move the **left joystick** to take over control from the autonomous system; the linear slide and main arm will automatically reset.
* **Emergency Reset**: As motor initial positions may not always be at zero when starting the program, tap the **touchpad** to force a reset.

### 智能视觉抓取系统 (`TeamCode/src/main/java/org/firstinspires/ftc/teamcode/vision`)
本团队在 `vision` 目录中开发了一套智能视觉抓取系统，其核心功能和架构如下（详细说明请参考该目录下的 `readme.md`）：
- **目标识别与定位**: 利用OpenCV进行图像处理，能够精确识别视野中的目标（例如，特定颜色的立方体或比赛元素），并计算其在三维空间中的相对位置和姿态。
- **分层系统架构**: 清晰划分用户交互、核心计算、视觉API封装、图像处理及配置等模块，提高代码可维护性。
- **智能辅助抓取**: 在手动操作（TeleOp）模式下，系统能为驾驶员提供关于目标位置的清晰反馈和行动建议，显著提高抓取效率和成功率。
- **详细标定流程**: 提供了针对摄像头参数、目标颜色HSV范围、物理尺寸的详细标定指南，确保系统精度。

### Intelligent Vision Grasping System (`TeamCode/src/main/java/org/firstinspires/ftc/teamcode/vision`)
The team has developed an intelligent vision grasping system within the `vision` directory, with core functionalities and architecture as follows (refer to the `readme.md` in that directory for detailed explanations):
- **Target Identification and Localization**: Leverages OpenCV for image processing to accurately identify targets in the field of view (e.g., specifically colored cubes or game elements) and calculate their relative 3D position and orientation.
- **Layered System Architecture**: Clearly divides modules for user interaction, core calculations, Vision API encapsulation, image processing, and configuration, enhancing code maintainability.
- **Smart Assisted Grasping**: In TeleOp mode, the system provides clear feedback and actionable suggestions to the driver regarding target location, significantly enhancing grasping efficiency and success rates.
- **Detailed Calibration Procedures**: Offers comprehensive guides for calibrating camera parameters, target color HSV ranges, and physical dimensions to ensure system accuracy.


## 贡献指南 | Contributing

欢迎PR和Issue！请阅读`.github/CONTRIBUTING.md`获取详细贡献流程和建议。建议先在团队内部fork并测试后再合并到主分支。

---

## 许可证 | License

本项目基于官方FTC SDK，遵循 FIRST 官方和相关第三方依赖的开源协议，详见 `LICENSE` 文件。

---

## 联系与支持 | Contact & Support

- 官方社区：[FIRST Tech Challenge Community](https://ftc-community.firstinspires.org/)
- 问题反馈：请通过GitHub Issue提交

---

