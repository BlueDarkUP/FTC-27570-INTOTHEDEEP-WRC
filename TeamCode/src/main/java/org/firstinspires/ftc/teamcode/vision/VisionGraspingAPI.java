package org.firstinspires.ftc.teamcode.vision;

import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.vision.pipeline.VisionPipeline;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

/**
 * 视觉抓取API (Vision Grasping API) - 主接口/门面 (Facade)
 * 这是提供给外部程序（如TeleOp）使用的唯一公共入口点。它封装了所有内部实现细节，
 * 包括相机管理和复杂的图像处理管道，为上层应用提供一个简洁、稳定的接口。
 *
 * 外部代码（如 VisionGraspingTeleOp）应该只与这个类进行交互，避免直接访问内部的 pipeline 包。
 * 内部的公共静态类 VisionTargetResult 被特意保留在此处，是为了确保调用此API的
 *    GraspingCalculator 和 VisionGraspingTeleOp 无需进行任何代码修改，保持了向后兼容性。
 * 此类负责初始化和销毁所有相关的视觉资源。
 * @author BlueDarkUP
 * @version 2025/6 (Refactored)
 * To My Lover - Zyy
 */
public class VisionGraspingAPI {

    // --- 公共枚举，用于选择颜色 ---
    public enum AllianceColor {
        RED,
        BLUE
    }

    // --- 可配置常量 (外部可能需要引用，保留在此处) ---
    public static final String WEBCAM_NAME_STR = "Webcam";
    public static final int CAMERA_WIDTH = 1280;
    public static final int CAMERA_HEIGHT = 720;

    // --- 相机焦距 (GraspingCalculator.java 需要引用，保留在此处) ---
    public static final double CAMERA_FOCAL_LENGTH_PIXELS = 420.70588235294;

    // --- 内部变量 ---
    private OpenCvWebcam webcam;
    private VisionPipeline pipeline;
    private volatile VisionTargetResult latestResult = new VisionTargetResult();

    /**
     * 初始化视觉API
     * @param hardwareMap 硬件映射
     * @param targetColor 要识别的联盟颜色 (RED 或 BLUE)
     */
    public void init(HardwareMap hardwareMap, AllianceColor targetColor) {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, WEBCAM_NAME_STR), cameraMonitorViewId);

        // 将选择的颜色传递给Pipeline
        pipeline = new VisionPipeline(this, targetColor);

        webcam.setPipeline(pipeline);
        webcam.setMillisecondsPermissionTimeout(2000);
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                webcam.startStreaming(CAMERA_WIDTH, CAMERA_HEIGHT, OpenCvCameraRotation.UPRIGHT);
            }
            @Override
            public void onError(int errorCode) {
                // Handle error
            }
        });
    }

    public VisionTargetResult getLatestResult() {
        return latestResult;
    }

    // 允许Pipeline更新结果 (包内可见)
    public void setLatestResult(VisionTargetResult newResult) {
        this.latestResult = newResult;
    }

    // 允许Pipeline访问Webcam实例
    public OpenCvWebcam getWebcam() {
        return webcam;
    }

    public double getFps() {
        return webcam != null ? webcam.getFps() : 0;
    }

    public double getPipelineTimeMs() {
        return webcam != null ? webcam.getPipelineTimeMs() : 0;
    }

    public void close() {
        if (webcam != null) {
            webcam.stopStreaming();
            webcam.closeCameraDevice();
        }
        if (pipeline != null) {
            pipeline.releaseMats();
        }
    }

    /**
     * 视觉目标结果的数据类。
     * 注意：此类必须保留为 VisionGraspingAPI 的公共静态内部类，
     * 以确保 VisionGraspingTeleOp 和 GraspingCalculator 无需修改。
     */
    public static class VisionTargetResult {
        public final boolean isTargetFound;
        public final String color;
        public final double distanceCm;
        public final double objectAngleDeg;
        public final double lineAngleDeg;
        public final double targetWidthCm;
        public final double targetHeightCm;
        public final String nextMoveDirection;
        public final double rawDistanceToGraspLineCm;
        public final double horizontalOffsetPx;

        public VisionTargetResult() {
            this(false, "None", Double.POSITIVE_INFINITY, 0.0, 0.0, 0.0, 0.0, "None", 0.0, 0.0);
        }

        public VisionTargetResult(boolean isFound, String color, double distance, double objectAngle, double lineAngle,
                                  double widthCm, double heightCm,
                                  String nextMoveDir, double rawDistCmForMove, double hOffsetPx) {
            this.isTargetFound = isFound;
            this.color = color;
            this.distanceCm = distance;
            this.objectAngleDeg = objectAngle;
            this.lineAngleDeg = lineAngle;
            this.targetWidthCm = widthCm;
            this.targetHeightCm = heightCm;
            this.nextMoveDirection = nextMoveDir;
            this.rawDistanceToGraspLineCm = rawDistCmForMove;
            this.horizontalOffsetPx = hOffsetPx;
        }
    }
}