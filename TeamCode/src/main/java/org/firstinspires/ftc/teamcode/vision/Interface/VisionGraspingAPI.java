package org.firstinspires.ftc.teamcode.vision.Interface;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.vision.Config.VisionConstants;
import org.firstinspires.ftc.teamcode.vision.Data.VisionTargetResult;
import org.firstinspires.ftc.teamcode.vision.Process.SamplePipeline;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

/**
 * 视觉抓取API (Vision Grasping API) - 门面模式 (Facade Pattern) 的实现
 * 这个类封装了所有摄像头和OpenCV图像处理的复杂初始化和管理逻辑
 * 对外（主要是OpMode）提供一个极其简洁和易于使用的接口来获取视觉识别结果
 * @author BlueDarkUP
 * @version 2025/6
 * To My Lover - Zyy
 */
public class VisionGraspingAPI {

    private OpenCvWebcam webcam;
    private SamplePipeline pipeline;

    /**
     * 存储由 Pipeline 计算出的最新结果
     * 使用 `volatile` 关键字是至关重要的，因为它确保了当 Pipeline 线程更新这个变量时
     * OpMode 的主线程能够立即看到这个修改，从而避免了多线程数据不同步的问题
     */

    private volatile VisionTargetResult latestResult = new VisionTargetResult();

    /**
     * 初始化摄像头和图像处理管线。
     * @param hardwareMap 从 OpMode 传入的 HardwareMap 对象。
     */
    public void init(HardwareMap hardwareMap) {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, VisionConstants.WEBCAM_NAME_STR), cameraMonitorViewId);
        pipeline = new SamplePipeline(this);
        webcam.setPipeline(pipeline);
        webcam.setMillisecondsPermissionTimeout(2000);
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                webcam.startStreaming(VisionConstants.CAMERA_WIDTH, VisionConstants.CAMERA_HEIGHT, OpenCvCameraRotation.UPRIGHT);
            }
            @Override
            public void onError(int errorCode) {
                // 在此处理摄像头打开错误
            }
        });
    }

    /**
     * 获取最新的视觉识别结果。
     * @return 最新的 VisionTargetResult 对象。
     */
    public VisionTargetResult getLatestResult() {
        return latestResult;
    }

    /**
     * 获取摄像头当前的帧率 (FPS)。
     * @return FPS 值。
     */
    public double getFps() {
        return webcam != null ? webcam.getFps() : 0;
    }

    /**
     * 获取图像处理管线处理一帧图像所需的平均时间（毫秒）。
     * @return 处理时间（毫秒）。
     */
    public double getPipelineTimeMs() {
        return webcam != null ? webcam.getPipelineTimeMs() : 0;
    }

    /**
     * 关闭摄像头并释放所有资源。
     */
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
     * 由 Pipeline 内部调用，用于更新识别结果。
     * 设为 package-private，仅允许同包下的 Pipeline 类访问。
     * @param newResult 新的识别结果
     */
    public void updateLatestResult(VisionTargetResult newResult) {
        this.latestResult = newResult;
    }

    /**
     * 由 Pipeline 内部调用，用于控制视口暂停/恢复。
     * 设为 package-private。
     * @param paused 是否暂停
     */
    public void toggleViewport(boolean paused) {
        if (webcam != null) {
            if (paused) {
                webcam.pauseViewport();
            } else {
                webcam.resumeViewport();
            }
        }
    }
}