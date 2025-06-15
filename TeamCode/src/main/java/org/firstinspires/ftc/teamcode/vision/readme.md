# 视觉系统 | Vision System (`vision/`)

**中文**: 欢迎来到27570队伍 "INTO THE DEEP" 赛季的智能视觉系统模块！本模块负责在比赛过程中识别、定位和辅助操作赛季特定的目标物体。它采用先进的图像处理技术和算法，旨在提供高精度、高鲁棒性的视觉解决方案。

**English**: Welcome to the Intelligent Vision System module for Team 27570's "INTO THE DEEP" season! This module is responsible for identifying, locating, and assisting in the manipulation of season-specific game objects during matches. It employs advanced image processing techniques and algorithms to provide a high-precision, robust vision solution.

## 目录 | Table of Contents
- [系统概述 | System Overview](#系统概述--system-overview)
- [核心组件与架构 | Core Components & Architecture](#核心组件与架构--core-components--architecture)
- [工作流程 | Workflow](#工作流程--workflow)
- [技术亮点 | Technical Highlights](#技术亮点--technical-highlights)
- [配置与调优 | Configuration & Tuning](#配置与调优--configuration--tuning)
- [使用方法 | Usage](#使用方法--usage)
- [调试与可视化 | Debugging & Visualization](#调试与可视化--debugging--visualization)

## 系统概述 | System Overview
**中文**:
本视觉系统基于OpenCV库（通过EasyOpenCV集成到FTC环境中），能够处理来自摄像头的实时图像流。其主要目标是：
1.  **目标检测**: 准确识别视野内的目标物体（例如：特定颜色和形状的赛季元素）。
2.  **目标定位**: 计算已识别目标在机器人坐标系或场地坐标系中的精确位置和姿态。
3.  **数据转换**: 将视觉数据转换为机器人执行机构（如机械臂、夹爪）可用的控制指令。
4.  **驾驶员辅助**: 在遥控操作(TeleOp)模式下，向驾驶员提供视觉反馈和辅助建议，提高操作效率。

**English**:
This vision system, built upon the OpenCV library (integrated into the FTC environment via EasyOpenCV), processes real-time image streams from the camera. Its primary objectives are:
1.  **Object Detection**: Accurately identify target objects (e.g., game elements of specific colors and shapes) within the field of view.
2.  **Object Localization**: Calculate the precise position and orientation of identified targets in the robot's or field's coordinate system.
3.  **Data Transformation**: Convert vision data into actionable control commands for robot actuators (e.g., arms, grippers).
4.  **Driver Assistance**: Provide visual feedback and assistive suggestions to the driver during TeleOp mode to enhance operational efficiency.

## 核心组件与架构 | Core Components & Architecture
**中文**:
系统采用模块化设计，主要组件包括：
-   **`VisionPipeline` (及其子类，如 `SamplePipeline`)**: 图像处理流水线核心。负责从摄像头获取图像帧，并应用一系列处理步骤（如颜色分割、二值化、形态学操作、轮廓检测）来提取目标特征。
-   **`DetectionProcessor` (或类似功能的类)**: 位于流水线内部，负责对提取到的轮廓或特征进行分析、筛选和评分，最终确定最可信的目标。
-   **`GraspingCalculator`**: 视觉算法的核心。接收来自 `DetectionProcessor` 的原始目标数据（如像素坐标、大小），并应用复杂的数学模型和校准数据来计算目标的实际三维空间位置。关键功能包括：
    -   **非线性距离校正**: 补偿镜头畸变和不同距离下像素与实际尺寸的非线性关系。
    -   **透视失真补偿**: 根据目标在图像中的位置和姿态，校正透视效应带来的尺寸和形状误差。
    -   **坐标系转换**: 将目标位置从图像坐标系转换到机器人坐标系。
-   **`VisionGraspingAPI`**: 作为整个视觉系统的统一高级接口 (Facade)。OpModes通过此类与视觉系统交互，简化了视觉功能的调用（如启动/停止检测、获取目标信息）。
-   **`VisionConstants`**: 集中管理所有视觉相关的配置参数，如颜色阈值、图像处理参数、摄像头标定参数、物理尺寸等。便于参数调整和优化，无需修改核心代码。
-   **数据类 (例如 `VisionTargetResult`, `GraspingTarget`, `DetectedCube`)**: 用于封装和传递视觉处理过程中的各种数据结构。

**English**:
The system features a modular design with the following key components:
-   **`VisionPipeline` (and its subclasses, e.g., `SamplePipeline`)**: The core of the image processing pipeline. Responsible for acquiring image frames from the camera and applying a series of processing steps (e.g., color segmentation, binarization, morphological operations, contour detection) to extract target features.
-   **`DetectionProcessor` (or similar classes)**: Resides within the pipeline and is responsible for analyzing, filtering, and scoring the extracted contours or features to determine the most credible targets.
-   **`GraspingCalculator`**: The heart of the vision algorithms. It receives raw target data (e.g., pixel coordinates, size) from the `DetectionProcessor` and applies complex mathematical models and calibration data to calculate the target's actual 3D spatial position. Key functions include:
    -   **Non-linear Distance Correction**: Compensates for lens distortion and the non-linear relationship between pixel size and actual size at varying distances.
    -   **Perspective Distortion Compensation**: Corrects for size and shape errors caused by perspective effects based on the target's position and orientation in the image.
    -   **Coordinate System Transformation**: Converts target positions from the image coordinate system to the robot's coordinate system.
-   **`VisionGraspingAPI`**: Serves as the unified high-level interface (Facade) for the entire vision system. OpModes interact with the vision system through this class, simplifying calls to vision functions (e.g., starting/stopping detection, retrieving target information).
-   **`VisionConstants`**: Centralizes all vision-related configuration parameters, such as color thresholds, image processing parameters, camera calibration data, and physical dimensions. This facilitates parameter tuning and optimization without modifying core code.
-   **Data Classes (e.g., `VisionTargetResult`, `GraspingTarget`, `DetectedCube`)**: Used to encapsulate and pass various data structures throughout the vision processing stages.

## 工作流程 | Workflow
**中文**:
1.  **初始化**: OpMode通过 `VisionGraspingAPI` 初始化并启动视觉处理流水线。摄像头被打开，`VisionConstants` 中的参数被加载。
2.  **图像采集**: `VisionPipeline` 持续从摄像头获取图像帧。
3.  **图像预处理**: 对图像帧进行预处理，如高斯模糊、颜色空间转换 (例如从RGB到HSV或YCrCb)。
4.  **颜色分割**: 根据 `VisionConstants` 中设定的颜色阈值，对图像进行分割，提取出感兴趣颜色的区域。
5.  **形态学处理**: 应用开运算、闭运算等形态学操作，去除噪点，填充目标内部空洞，平滑边缘。
6.  **轮廓检测**: 找到分割后图像中的闭合轮廓。
7.  **轮廓筛选与目标决策 (`DetectionProcessor`)**:
    -   根据轮廓的几何特征（如面积、宽高比、周长、圆形度、凸性）进行筛选，排除不符合目标特征的轮廓。
    -   对符合条件的轮廓进行评分，选择分数最高的作为最终目标。
8.  **位置解算 (`GraspingCalculator`)**:
    -   获取选定目标的像素坐标、边界框等信息。
    -   应用非线性距离校正和透视失真补偿算法，计算目标相对于摄像头的精确距离和角度。
    -   将目标位置从摄像头坐标系转换到机器人中心坐标系或场地坐标系。
9.  **数据输出**: `VisionGraspingAPI` 将处理结果（如目标是否存在、目标位置、建议的机器人移动等）提供给OpMode。

**English**:
1.  **Initialization**: The OpMode initializes and starts the vision processing pipeline via `VisionGraspingAPI`. The camera is opened, and parameters from `VisionConstants` are loaded.
2.  **Image Acquisition**: `VisionPipeline` continuously acquires image frames from the camera.
3.  **Image Preprocessing**: Frames are preprocessed, e.g., Gaussian blur, color space conversion (e.g., RGB to HSV or YCrCb).
4.  **Color Segmentation**: Based on color thresholds defined in `VisionConstants`, the image is segmented to extract regions of interest by color.
5.  **Morphological Processing**: Morphological operations like opening and closing are applied to remove noise, fill holes within targets, and smooth edges.
6.  **Contour Detection**: Closed contours are found in the segmented image.
7.  **Contour Filtering & Target Decision (`DetectionProcessor`)**:
    -   Contours are filtered based on geometric properties (e.g., area, aspect ratio, perimeter, circularity, convexity) to exclude those not matching target characteristics.
    -   Qualifying contours are scored, and the one with the highest score is selected as the final target.
8.  **Position Calculation (`GraspingCalculator`)**:
    -   Pixel coordinates, bounding box, and other information of the selected target are obtained.
    -   Non-linear distance correction and perspective distortion compensation algorithms are applied to calculate the precise distance and angle of the target relative to the camera.
    -   The target's position is transformed from the camera coordinate system to the robot's center or field coordinate system.
9.  **Data Output**: `VisionGraspingAPI` provides the processing results (e.g., target presence, target position, suggested robot movements) to the OpMode.

## 技术亮点 | Technical Highlights
**中文**:
-   **高精度定位算法**: `GraspingCalculator` 中实现的非线性距离校正和透视失真补偿算法，显著提高了在不同距离和角度下目标定位的准确性。
-   **鲁棒的图像处理流程**: 结合颜色分割、形态学操作和智能轮廓筛选，使得系统在复杂光照和背景干扰下仍能稳定识别目标。
-   **模块化与可配置性**: 清晰的模块划分 (`Pipeline`, `Processor`, `Calculator`, `API`) 和集中的参数管理 (`VisionConstants`) 使得系统易于理解、维护、扩展和调优。
-   **驾驶员辅助功能**: 系统不仅服务于自主模式，还能在TeleOp模式下分析视野，向驾驶员提供目标相对位置信息或建议机器人移动方向，提升操作效率和准确性。
-   **实时性能**: 优化图像处理步骤，确保在FTC控制器的计算能力下达到可接受的实时处理帧率。
-   **调试友好**: 通常会提供调试视图 (Debug View) 到Driver Station的视频流上，将处理过程中的中间结果（如二值化图像、检测到的轮廓、目标边界框）可视化，方便快速诊断问题。

**English**:
-   **High-Precision Localization Algorithms**: The non-linear distance correction and perspective distortion compensation algorithms implemented in `GraspingCalculator` significantly improve the accuracy of target localization at various distances and angles.
-   **Robust Image Processing Pipeline**: The combination of color segmentation, morphological operations, and intelligent contour filtering enables stable target recognition even under complex lighting conditions and background interference.
-   **Modularity & Configurability**: Clear separation of modules (`Pipeline`, `Processor`, `Calculator`, `API`) and centralized parameter management (`VisionConstants`) make the system easy to understand, maintain, extend, and tune.
-   **Driver Assistance Features**: The system not only serves autonomous mode but can also analyze the field of view in TeleOp mode, providing the driver with target relative position information or suggesting robot movement directions, enhancing operational efficiency and accuracy.
-   **Real-time Performance**: Optimized image processing steps ensure an acceptable real-time processing frame rate on the FTC controller's computational resources.
-   **Debug-Friendly**: Typically provides a debug view streamed to the Driver Station, visualizing intermediate results of the processing pipeline (e.g., binarized images, detected contours, target bounding boxes) for rapid problem diagnosis.

## 配置与调优 | Configuration & Tuning
**中文**:
-   **`VisionConstants.java`**: 这是视觉系统调优的核心文件。几乎所有可调参数都集中在这里，包括：
    -   **颜色阈值**: HSV或YCrCb等颜色空间下的最小和最大阈值，用于分割特定颜色的目标。
    -   **图像尺寸**: 处理图像的宽度和高度。
    -   **摄像头标定参数**: 焦距、主点坐标、畸变系数（如果进行了摄像头标定）。
    -   **物理参数**: 目标物体的实际尺寸，摄像头安装高度和角度等。
    -   **处理参数**: 形态学操作的内核大小、轮廓筛选的面积/宽高比阈值等。
-   **调优步骤**:
    1.  **颜色标定**: 在实际场地光线下，调整颜色阈值，确保能准确稳定地分割出目标物体，同时最大限度地排除背景干扰。EasyOpenCV通常提供实时预览功能，方便观察阈值调整效果。
    2.  **轮廓筛选参数**: 根据目标特性，调整轮廓面积、宽高比等参数，确保只保留有效目标。
    3.  **距离和角度校准**: 打印校准图案或使用已知距离和角度的目标，验证 `GraspingCalculator` 的输出精度。根据偏差调整校正模型中的参数或查找表。

**English**:
-   **`VisionConstants.java`**: This is the core file for tuning the vision system. Almost all tunable parameters are centralized here, including:
    -   **Color Thresholds**: Min/max values in color spaces like HSV or YCrCb for segmenting specifically colored targets.
    -   **Image Dimensions**: Width and height of the processed image.
    -   **Camera Calibration Parameters**: Focal length, principal point coordinates, distortion coefficients (if camera calibration was performed).
    -   **Physical Parameters**: Actual dimensions of the target object, camera mounting height and angle, etc.
    -   **Processing Parameters**: Kernel sizes for morphological operations, area/aspect ratio thresholds for contour filtering, etc.
-   **Tuning Steps**:
    1.  **Color Calibration**: Under actual field lighting conditions, adjust color thresholds to ensure accurate and stable segmentation of target objects while maximally excluding background interference. EasyOpenCV typically provides a live preview feature to observe the effects of threshold adjustments.
    2.  **Contour Filtering Parameters**: Based on target characteristics, adjust contour area, aspect ratio, and other parameters to ensure only valid targets are retained.
    3.  **Distance and Angle Calibration**: Print calibration patterns or use targets at known distances and angles to verify the output accuracy of `GraspingCalculator`. Adjust parameters in the correction model or lookup tables based on observed deviations.

## 使用方法 | Usage
**中文**:
在OpMode中（无论是Autonomous还是TeleOp），通常通过以下步骤使用视觉系统：
1.  **实例化 `VisionGraspingAPI`**: `VisionGraspingAPI visionAPI = new VisionGraspingAPI(hardwareMap);`
2.  **初始化视觉系统**: `visionAPI.init();` (可能包括启动摄像头预览)
3.  **在循环中获取目标信息**:
    ```java
    // 在 opModeIsActive() 或主循环中
    VisionTargetResult targetResult = visionAPI.getLatestTargetResult(); // 或类似方法
    if (targetResult != null && targetResult.isTargetDetected()) {
        double distanceToTarget = targetResult.getDistanceCm();
        double angleToTarget = targetResult.getAngleDegrees();
        // 根据目标信息执行相应动作
    }
    // 对于驾驶员辅助，可能调用:
    // String suggestion = visionAPI.getMovementSuggestion();
    // telemetry.addData("Vision Suggestion", suggestion);
    ```
4.  **关闭视觉系统**: 在OpMode结束时，确保关闭摄像头和释放资源: `visionAPI.close();`

**English**:
In an OpMode (either Autonomous or TeleOp), the vision system is typically used as follows:
1.  **Instantiate `VisionGraspingAPI`**: `VisionGraspingAPI visionAPI = new VisionGraspingAPI(hardwareMap);`
2.  **Initialize the Vision System**: `visionAPI.init();` (may include starting the camera preview)
3.  **Get Target Information in a Loop**:
    ```java
    // Inside opModeIsActive() or the main loop
    VisionTargetResult targetResult = visionAPI.getLatestTargetResult(); // or similar method
    if (targetResult != null && targetResult.isTargetDetected()) {
        double distanceToTarget = targetResult.getDistanceCm();
        double angleToTarget = targetResult.getAngleDegrees();
        // Perform actions based on target information
    }
    // For driver assistance, you might call:
    // String suggestion = visionAPI.getMovementSuggestion();
    // telemetry.addData("Vision Suggestion", suggestion);
    ```
4.  **Close the Vision System**: At the end of the OpMode, ensure the camera is closed and resources are released: `visionAPI.close();`

## 调试与可视化 | Debugging & Visualization
**中文**:
-   **EasyOpenCV Camera Stream**: EasyOpenCV允许将处理后的视频流（或特定处理阶段的图像）发送到Driver Station的摄像头预览界面。这对于实时观察颜色分割效果、轮廓检测情况以及最终目标选定至关重要。
-   **Telemetry**: 将关键数据（如检测到的目标数量、目标坐标、距离、角度、处理耗时）通过Telemetry输出到Driver Station，方便监控系统状态和性能。
-   **日志记录**: 对于复杂问题，可以在代码中添加详细的日志记录，帮助追踪数据变化和决策过程。

**English**:
-   **EasyOpenCV Camera Stream**: EasyOpenCV allows streaming the processed video (or images from specific processing stages) to the Driver Station's camera preview interface. This is crucial for real-time observation of color segmentation effectiveness, contour detection, and final target selection.
-   **Telemetry**: Output key data (e.g., number of detected targets, target coordinates, distance, angle, processing time) via Telemetry to the Driver Station for monitoring system status and performance.
-   **Logging**: For complex issues, add detailed logging within the code to help trace data changes and decision-making processes.

---
祝视觉系统开发顺利！
Happy vision developing!
