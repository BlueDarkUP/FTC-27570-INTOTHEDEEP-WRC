package org.firstinspires.ftc.teamcode.vision.pipeline;

import org.firstinspires.ftc.teamcode.vision.VisionGraspingAPI;
import org.firstinspires.ftc.teamcode.vision.pipeline.drawing.DrawingUtils;
import org.firstinspires.ftc.teamcode.vision.pipeline.model.DetectedCube;
import org.firstinspires.ftc.teamcode.vision.pipeline.model.TargetZoneInfo;
import org.firstinspires.ftc.teamcode.vision.pipeline.processing.DetectionProcessor;
import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Point;
import org.opencv.core.RotatedRect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

/**
 * 核心视觉处理管道
 * 这是继承自 OpenCvPipeline 的核心类，负责实现完整的图像处理流程。它按顺序执行
 * 图像预处理、颜色分割、轮廓检测，然后调用外部的辅助类（如 DetectionProcessor）
 * 来完成复杂的决策逻辑。
 *
 * 此类的主要职责是“流程控制”，而非“业务决策”。它管理着各个处理阶段和OpenCV的Mat对象生命周期。
 * 复杂的决策逻辑被委托给 DetectionProcessor 类，绘图功能被委托给 DrawingUtils 类。
 * 所有可调参数都应从 VisionConstants 类中引用，以保持本类的整洁。
 * @author BlueDarkUP
 * @version 2025/6 (Refactored)
 * To My Lover - Zyy
 */
public class VisionPipeline extends OpenCvPipeline {

    private final VisionGraspingAPI apiInstance;
    private final DetectionProcessor detectionProcessor;
    private final String targetColorName; // 要识别的颜色名称 ("RED" 或 "BLUE")

    private Mat bgr, hsv, processedFrameForDetection, masterMask, maskCombined, colorMask, medianBlurred, opened;
    private Mat kernel3x3;
    private MatOfPoint2f tempContour2f;
    private MatOfPoint tempScaledBoxPointsForDisplay;
    private MatOfPoint targetZoneContourAtOriginalScale, targetZoneContourAtProcessedScaleHolder;
    private Integer targetZoneCenterXAtOriginalScale, targetRectY1AtOriginalScale, targetRectY2AtOriginalScale;
    private boolean viewportPaused;

    public VisionPipeline(VisionGraspingAPI apiInstance, VisionGraspingAPI.AllianceColor targetColor) {
        this.apiInstance = apiInstance;
        this.detectionProcessor = new DetectionProcessor();

        // 将枚举转换为用于Map查找的字符串
        this.targetColorName = (targetColor == VisionGraspingAPI.AllianceColor.RED) ? "RED" : "BLUE";

        Mat dummyFrame = new Mat(VisionGraspingAPI.CAMERA_HEIGHT, VisionGraspingAPI.CAMERA_WIDTH, CvType.CV_8UC3);
        TargetZoneInfo info = DrawingUtils.drawTargetZoneCm(dummyFrame, VisionConstants.PIXELS_PER_CM_FOR_DRAWING, VisionGraspingAPI.CAMERA_WIDTH, VisionGraspingAPI.CAMERA_HEIGHT, VisionConstants.TARGET_RECT_WIDTH_CM, VisionConstants.TARGET_RECT_HEIGHT_CM, VisionConstants.TARGET_RECT_OFFSET_X_CM, VisionConstants.TARGET_RECT_OFFSET_Y_CM, VisionConstants.TARGET_RECT_COLOR, VisionConstants.TARGET_RECT_THICKNESS, VisionConstants.ARC_SAMPLING_POINTS);

        targetZoneContourAtOriginalScale = info.targetContour;
        targetZoneCenterXAtOriginalScale = info.centerX;
        targetRectY1AtOriginalScale = info.topY;
        targetRectY2AtOriginalScale = info.bottomY;
        dummyFrame.release();
    }

    @Override
    public void init(Mat firstFrame) {
        int h = firstFrame.rows(), w = firstFrame.cols();
        int pW = (int)(w * VisionConstants.DOWNSCALE_FACTOR), pH = (int)(h * VisionConstants.DOWNSCALE_FACTOR);
        if (VisionConstants.DOWNSCALE_FACTOR <= 0 || VisionConstants.DOWNSCALE_FACTOR > 1) { pW = w; pH = h; }

        bgr = new Mat(h, w, CvType.CV_8UC3);
        hsv = new Mat(pH, pW, CvType.CV_8UC3);
        masterMask = new Mat(pH, pW, CvType.CV_8U);
        maskCombined = new Mat(pH, pW, CvType.CV_8U);
        colorMask = new Mat(pH, pW, CvType.CV_8U);
        medianBlurred = new Mat(pH, pW, CvType.CV_8U);
        opened = new Mat(pH, pW, CvType.CV_8U);
        kernel3x3 = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(3, 3));
        processedFrameForDetection = new Mat(pH, pW, CvType.CV_8UC3);
        targetZoneContourAtProcessedScaleHolder = new MatOfPoint();
        tempContour2f = new MatOfPoint2f();
        tempScaledBoxPointsForDisplay = new MatOfPoint();
    }

    @Override
    public Mat processFrame(Mat inputRGBA) {
        Imgproc.cvtColor(inputRGBA, bgr, Imgproc.COLOR_RGBA2BGR);

        double processingScale = VisionConstants.DOWNSCALE_FACTOR;
        MatOfPoint targetZoneContourAtProcessedScale = null;
        Integer targetZoneCenterXAtProcessedScale = null;
        Integer targetRectY1AtProcessedScale = null;
        Integer targetRectY2AtProcessedScale = null;

        if (processingScale > 0 && processingScale < 1.0) {
            Imgproc.resize(bgr, processedFrameForDetection, new Size((int)(bgr.cols() * processingScale), (int)(bgr.rows() * processingScale)), 0, 0, Imgproc.INTER_LINEAR);
            if (targetZoneContourAtOriginalScale != null) {
                List<Point> scaledPoints = new ArrayList<>();
                for (Point p : targetZoneContourAtOriginalScale.toList()) {
                    scaledPoints.add(new Point(Math.round(p.x * processingScale), Math.round(p.y * processingScale)));
                }
                targetZoneContourAtProcessedScaleHolder.fromList(scaledPoints);
                targetZoneContourAtProcessedScale = targetZoneContourAtProcessedScaleHolder;
            }
            if (targetZoneCenterXAtOriginalScale != null) targetZoneCenterXAtProcessedScale = (int)Math.round(targetZoneCenterXAtOriginalScale * processingScale);
            if (targetRectY1AtOriginalScale != null) targetRectY1AtProcessedScale = (int)Math.round(targetRectY1AtOriginalScale * processingScale);
            if (targetRectY2AtOriginalScale != null) targetRectY2AtProcessedScale = (int)Math.round(targetRectY2AtOriginalScale * processingScale);
        } else {
            processingScale = 1.0;
            bgr.copyTo(processedFrameForDetection);
            targetZoneContourAtProcessedScale = targetZoneContourAtOriginalScale;
            targetZoneCenterXAtProcessedScale = targetZoneCenterXAtOriginalScale;
            targetRectY1AtProcessedScale = targetRectY1AtOriginalScale;
            targetRectY2AtProcessedScale = targetRectY2AtOriginalScale;
        }

        Imgproc.cvtColor(processedFrameForDetection, hsv, Imgproc.COLOR_BGR2HSV);
        masterMask.setTo(new Scalar(0));
        ArrayList<DetectedCube> allDetectedCubes = new ArrayList<>();
        double scaledMinArea = VisionConstants.MIN_SIZE_PIXELS * (processingScale * processingScale);
        double scaledMaxArea = VisionConstants.MAX_SIZE_PIXELS * (processingScale * processingScale);

        // --- 只处理选定的颜色 ---
        Scalar[][] hsvRanges = VisionConstants.COLOR_HSV_RANGES.get(this.targetColorName);
        if (hsvRanges != null) {
            colorMask.setTo(new Scalar(0));
            // 对一个颜色的所有范围进行或运算 (例如红色有两个范围)
            for (Scalar[] range : hsvRanges) {
                Core.inRange(hsv, range[0], range[1], maskCombined);
                Core.bitwise_or(colorMask, maskCombined, colorMask);
            }

            // 对主掩码应用颜色掩码 (虽然只有一个颜色，但保持此结构以便未来扩展)
            Core.bitwise_or(masterMask, colorMask, masterMask);

            // 形态学操作和轮廓查找
            Imgproc.medianBlur(colorMask, medianBlurred, 3);
            Imgproc.morphologyEx(medianBlurred, opened, Imgproc.MORPH_OPEN, kernel3x3, new Point(-1, -1), 2);
            List<MatOfPoint> contours = new ArrayList<>();
            Imgproc.findContours(opened, contours, new Mat(), Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);

            for (MatOfPoint contour : contours) {
                double area = Imgproc.contourArea(contour);
                if (area < scaledMinArea || area > scaledMaxArea || contour.rows() < 5) { contour.release(); continue; }
                contour.convertTo(tempContour2f, CvType.CV_32F);
                RotatedRect minAreaRect = Imgproc.minAreaRect(tempContour2f);
                double w = minAreaRect.size.width, h = minAreaRect.size.height;
                double angle = (w < h) ? minAreaRect.angle + 90.0 : minAreaRect.angle;
                double longSide = Math.max(w,h), shortSide = Math.min(w,h);
                if (shortSide < 1e-3) { contour.release(); continue; }
                double ar = longSide / shortSide;
                if (ar < VisionConstants.TARGET_OBJECT_ASPECT_RATIO * (1-VisionConstants.ASPECT_RATIO_TOLERANCE_PERCENT) || ar > VisionConstants.TARGET_OBJECT_ASPECT_RATIO * (1+VisionConstants.ASPECT_RATIO_TOLERANCE_PERCENT)) { contour.release(); continue; }
                Point[] boxPoints = new Point[4];
                minAreaRect.points(boxPoints);
                // 在创建DetectedCube时使用当前的目标颜色名称
                allDetectedCubes.add(new DetectedCube(this.targetColorName, (int)minAreaRect.center.x, (int)minAreaRect.center.y, boxPoints, processingScale, angle, w, h));
                contour.release();
            }
        }

        // 后续处理逻辑不变
        detectionProcessor.process(apiInstance, allDetectedCubes, processingScale, targetZoneContourAtProcessedScale, targetZoneCenterXAtProcessedScale, targetRectY1AtProcessedScale, targetRectY2AtProcessedScale, VisionConstants.PIXELS_PER_CM_FOR_DRAWING, targetRectY2AtOriginalScale);

        if (VisionConstants.ENABLE_DEBUG_VIEW) {
            TargetZoneInfo finalDisplayZoneInfo = DrawingUtils.drawTargetZoneCm(bgr, VisionConstants.PIXELS_PER_CM_FOR_DRAWING, VisionGraspingAPI.CAMERA_WIDTH, VisionGraspingAPI.CAMERA_HEIGHT, VisionConstants.TARGET_RECT_WIDTH_CM, VisionConstants.TARGET_RECT_HEIGHT_CM, VisionConstants.TARGET_RECT_OFFSET_X_CM, VisionConstants.TARGET_RECT_OFFSET_Y_CM, VisionConstants.TARGET_RECT_COLOR, VisionConstants.TARGET_RECT_THICKNESS, VisionConstants.ARC_SAMPLING_POINTS);
            if (finalDisplayZoneInfo.centerX != null) {
                Imgproc.line(bgr, new Point(finalDisplayZoneInfo.centerX, 0), new Point(finalDisplayZoneInfo.centerX, VisionGraspingAPI.CAMERA_HEIGHT), VisionConstants.TARGET_RECT_COLOR, 1);
                if (finalDisplayZoneInfo.targetContour != null) finalDisplayZoneInfo.targetContour.release();
            }
            DrawingUtils.drawGridOverlay(bgr, Math.max(1, (int)Math.round(5 * VisionConstants.PIXELS_PER_CM_FOR_DRAWING)), VisionConstants.GRID_COLOR, 1);
        }

        for (DetectedCube cube : allDetectedCubes) {
            Point[] boxPointsDisplay = new Point[cube.boundingBoxPoints.length];
            for(int i=0; i < cube.boundingBoxPoints.length; i++) {
                boxPointsDisplay[i] = new Point(cube.boundingBoxPoints[i].x / processingScale, cube.boundingBoxPoints[i].y / processingScale);
            }
            tempScaledBoxPointsForDisplay.fromArray(boxPointsDisplay);
            Imgproc.drawContours(bgr, Collections.singletonList(tempScaledBoxPointsForDisplay), 0, new Scalar(0, 255, 255), 1);
        }

        Imgproc.cvtColor(bgr, inputRGBA, Imgproc.COLOR_BGR2RGBA);
        return inputRGBA;
    }

    @Override
    public void onViewportTapped() {
        viewportPaused = !viewportPaused;
        if(apiInstance.getWebcam() != null) {
            if (viewportPaused) {
                apiInstance.getWebcam().pauseViewport();
            } else {
                apiInstance.getWebcam().resumeViewport();
            }
        }
    }

    public void releaseMats() {
        if (bgr != null) bgr.release();
        if (hsv != null) hsv.release();
        if (processedFrameForDetection != null) processedFrameForDetection.release();
        if (masterMask != null) masterMask.release();
        if (maskCombined != null) maskCombined.release();
        if (colorMask != null) colorMask.release();
        if (medianBlurred != null) medianBlurred.release();
        if (opened != null) opened.release();
        if (kernel3x3 != null) kernel3x3.release();
        if (targetZoneContourAtOriginalScale != null) targetZoneContourAtOriginalScale.release();
        if (tempContour2f != null) tempContour2f.release();
        if (tempScaledBoxPointsForDisplay != null) tempScaledBoxPointsForDisplay.release();
        if (targetZoneContourAtProcessedScaleHolder != null) targetZoneContourAtProcessedScaleHolder.release();
        if (detectionProcessor != null) detectionProcessor.release();
    }
}