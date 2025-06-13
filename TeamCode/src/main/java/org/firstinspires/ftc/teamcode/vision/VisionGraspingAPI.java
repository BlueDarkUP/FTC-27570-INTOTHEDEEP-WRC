package org.firstinspires.ftc.teamcode.vision;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
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
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;
import org.openftc.easyopencv.OpenCvWebcam;

import java.util.ArrayList;
import java.util.Collections;
import java.util.Comparator;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

/**
 * 视觉抓取API (Vision Grasping API)
 * 增加在只有一个目标时也提供微调建议的功能。
 * @author BlueDarkUP
 * @version 2025/6
 * To My Lover - Zyy
 */
public class VisionGraspingAPI {

    // --- 可配置常量 ---
    public static final String WEBCAM_NAME_STR = "Webcam";
    public static final int CAMERA_WIDTH = 1280;
    public static final int CAMERA_HEIGHT = 720;

    // --- 相机焦距（来自您的标定）---
    public static final double CAMERA_FOCAL_LENGTH_PIXELS = 420.70588235294;

    // --- 内部变量 ---
    private OpenCvWebcam webcam;
    private SamplePipeline pipeline;
    private volatile VisionTargetResult latestResult = new VisionTargetResult();

    public void init(HardwareMap hardwareMap) {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, WEBCAM_NAME_STR), cameraMonitorViewId);
        pipeline = new SamplePipeline(this);
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

        // 默认构造函数
        public VisionTargetResult() {
            this.isTargetFound = false;
            this.color = "None";
            this.distanceCm = Double.POSITIVE_INFINITY;
            this.objectAngleDeg = 0.0;
            this.lineAngleDeg = 0.0;
            this.targetWidthCm = 0.0;
            this.targetHeightCm = 0.0;
            this.nextMoveDirection = "None";
            this.rawDistanceToGraspLineCm = 0.0;
            this.horizontalOffsetPx = 0.0;
        }

        // 统一的构造函数，可以处理所有情况
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

    private static class SamplePipeline extends OpenCvPipeline {

        public static final boolean ENABLE_DEBUG_VIEW = true;
        public static final double PIXELS_PER_CM_FOR_DRAWING = 13.9;

        public static final double TARGET_RECT_WIDTH_CM = 31.5;
        public static final double TARGET_RECT_HEIGHT_CM = 24.0;
        public static final double TARGET_RECT_OFFSET_X_CM = 1.1;
        public static final double TARGET_RECT_OFFSET_Y_CM = 3.0;
        public static final Scalar TARGET_RECT_COLOR = new Scalar(255, 0, 255);
        public static final int TARGET_RECT_THICKNESS = 2;
        public static final int ARC_SAMPLING_POINTS = 20;
        public static final double DOWNSCALE_FACTOR = 1.0;
        public static final double MIN_SIZE_PIXELS = 200.0;
        public static final double MAX_SIZE_PIXELS = 25000.0;
        public static final double TARGET_OBJECT_ASPECT_RATIO = 7.0 / 3.0;
        public static final double ASPECT_RATIO_TOLERANCE_PERCENT = 0.25;
        public static final Map<String, Scalar[][]> COLOR_HSV_RANGES = new HashMap<>();
        static {
            COLOR_HSV_RANGES.put("YELLOW", new Scalar[][]{{new Scalar(15, 80, 180), new Scalar(45, 255, 255)}});
        }
        public static final Scalar GRID_COLOR = new Scalar(70, 70, 70);

        static class DetectedCube {
            public String color;
            public int centerXImagePx, centerYImagePx;
            public Point[] boundingBoxPoints;
            public double scaleFactor, angleDeg;
            public double rectWidthPx, rectHeightPx;

            public DetectedCube(String color, int centerX, int centerY, Point[] points, double scale, double angle, double widthPx, double heightPx) {
                this.color = color;
                this.centerXImagePx = centerX;
                this.centerYImagePx = centerY;
                this.boundingBoxPoints = points;
                this.scaleFactor = scale;
                this.angleDeg = angle;
                this.rectWidthPx = widthPx;
                this.rectHeightPx = heightPx;
            }
        }

        static class TargetZoneInfo {
            public MatOfPoint targetContour;
            public Integer centerX, topY, bottomY;
            public TargetZoneInfo(MatOfPoint c, Integer cx, Integer ty, Integer by) {
                this.targetContour = c; this.centerX = cx; this.topY = ty; this.bottomY = by;
            }
        }

        static class CandidateInfo {
            int cubeIndex;
            double primaryScore, secondaryScore, lineAngleDeg, distanceCm;
            Point centerInProcessed, intersectionPointProcessed;
            double horizontalDistanceToCenterPx;
        }

        private VisionGraspingAPI apiInstance;
        private Mat bgr, hsv, processedFrameForDetection, masterMask, maskCombined, colorMask, medianBlurred, opened;
        private Mat kernel3x3;
        private MatOfPoint2f tempContour2f;
        private MatOfPoint tempScaledBoxPointsForDisplay;
        private MatOfPoint targetZoneContourAtOriginalScale, targetZoneContourAtProcessedScaleHolder;
        private Integer targetZoneCenterXAtOriginalScale, targetRectY1AtOriginalScale, targetRectY2AtOriginalScale;
        private boolean viewportPaused;

        public SamplePipeline(VisionGraspingAPI apiInstance) {
            this.apiInstance = apiInstance;
            Mat dummyFrame = new Mat(CAMERA_HEIGHT, CAMERA_WIDTH, CvType.CV_8UC3);
            TargetZoneInfo info = drawTargetZoneCm(dummyFrame, PIXELS_PER_CM_FOR_DRAWING, CAMERA_WIDTH, CAMERA_HEIGHT, TARGET_RECT_WIDTH_CM, TARGET_RECT_HEIGHT_CM, TARGET_RECT_OFFSET_X_CM, TARGET_RECT_OFFSET_Y_CM, TARGET_RECT_COLOR, TARGET_RECT_THICKNESS, ARC_SAMPLING_POINTS);
            targetZoneContourAtOriginalScale = info.targetContour;
            targetZoneCenterXAtOriginalScale = info.centerX;
            targetRectY1AtOriginalScale = info.topY;
            targetRectY2AtOriginalScale = info.bottomY;
            dummyFrame.release();
        }

        @Override
        public void init(Mat firstFrame) {
            int h = firstFrame.rows(), w = firstFrame.cols();
            int pW = (int)(w * DOWNSCALE_FACTOR), pH = (int)(h * DOWNSCALE_FACTOR);
            if (DOWNSCALE_FACTOR <= 0 || DOWNSCALE_FACTOR > 1) { pW = w; pH = h; }
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
            double drawScale = 1.0, processingScale = 1.0;
            MatOfPoint targetZoneContourAtProcessedScale = null;
            Integer targetZoneCenterXAtProcessedScale = null;
            Integer targetRectY1AtProcessedScale = null;
            Integer targetRectY2AtProcessedScale = null;

            if (DOWNSCALE_FACTOR > 0 && DOWNSCALE_FACTOR < 1.0) {
                processingScale = DOWNSCALE_FACTOR;
                drawScale = 1.0 / DOWNSCALE_FACTOR;
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
                bgr.copyTo(processedFrameForDetection);
                targetZoneContourAtProcessedScale = targetZoneContourAtOriginalScale;
                targetZoneCenterXAtProcessedScale = targetZoneCenterXAtOriginalScale;
                targetRectY1AtProcessedScale = targetRectY1AtOriginalScale;
                targetRectY2AtProcessedScale = targetRectY2AtOriginalScale;
            }

            Imgproc.cvtColor(processedFrameForDetection, hsv, Imgproc.COLOR_BGR2HSV);
            masterMask.setTo(new Scalar(0));
            ArrayList<DetectedCube> allDetectedCubes = new ArrayList<>();
            double scaledMinArea = MIN_SIZE_PIXELS * (processingScale * processingScale);
            double scaledMaxArea = MAX_SIZE_PIXELS * (processingScale * processingScale);

            for (Map.Entry<String, Scalar[][]> entry : COLOR_HSV_RANGES.entrySet()) {
                colorMask.setTo(new Scalar(0));
                for (Scalar[] range : entry.getValue()) {
                    Core.inRange(hsv, range[0], range[1], maskCombined);
                    Core.bitwise_or(colorMask, maskCombined, colorMask);
                }
                Core.bitwise_or(masterMask, colorMask, masterMask);
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
                    if (ar < TARGET_OBJECT_ASPECT_RATIO * (1-ASPECT_RATIO_TOLERANCE_PERCENT) || ar > TARGET_OBJECT_ASPECT_RATIO * (1+ASPECT_RATIO_TOLERANCE_PERCENT)) { contour.release(); continue; }
                    Point[] boxPoints = new Point[4];
                    minAreaRect.points(boxPoints);
                    allDetectedCubes.add(new DetectedCube(entry.getKey(), (int)minAreaRect.center.x, (int)minAreaRect.center.y, boxPoints, processingScale, angle, w, h));
                    contour.release();
                }
            }

            processDetections(allDetectedCubes, processingScale, targetZoneContourAtProcessedScale, targetZoneCenterXAtProcessedScale, targetRectY1AtProcessedScale, targetRectY2AtProcessedScale, PIXELS_PER_CM_FOR_DRAWING, targetRectY2AtOriginalScale);

            if (ENABLE_DEBUG_VIEW) {
                TargetZoneInfo finalDisplayZoneInfo = drawTargetZoneCm(bgr, PIXELS_PER_CM_FOR_DRAWING, CAMERA_WIDTH, CAMERA_HEIGHT, TARGET_RECT_WIDTH_CM, TARGET_RECT_HEIGHT_CM, TARGET_RECT_OFFSET_X_CM, TARGET_RECT_OFFSET_Y_CM, TARGET_RECT_COLOR, TARGET_RECT_THICKNESS, ARC_SAMPLING_POINTS);
                if (finalDisplayZoneInfo.centerX != null) {
                    Imgproc.line(bgr, new Point(finalDisplayZoneInfo.centerX, 0), new Point(finalDisplayZoneInfo.centerX, CAMERA_HEIGHT), TARGET_RECT_COLOR, 1);
                    if (finalDisplayZoneInfo.targetContour != null) finalDisplayZoneInfo.targetContour.release();
                }
                drawGridOverlay(bgr, Math.max(1, (int)Math.round(5 * PIXELS_PER_CM_FOR_DRAWING)), GRID_COLOR, 1);
            }

            for (DetectedCube cube : allDetectedCubes) {
                Point[] boxPointsDisplay = scaleRectPoints(cube.boundingBoxPoints, 1.0 / processingScale);
                tempScaledBoxPointsForDisplay.fromArray(boxPointsDisplay);
                Imgproc.drawContours(bgr, Collections.singletonList(tempScaledBoxPointsForDisplay), 0, new Scalar(0, 255, 255), 1);
            }

            Imgproc.cvtColor(bgr, inputRGBA, Imgproc.COLOR_BGR2RGBA);
            return inputRGBA;
        }

        private void processDetections(List<DetectedCube> allCubes, double processingScale, MatOfPoint targetZoneContourProcessed, Integer targetZoneCenterXAtProcessedScale, Integer targetRectY1Processed, Integer targetRectY2Processed, double pixelsPerCmForDrawing, double targetRectY2OriginalScale) {
            if (allCubes.isEmpty()) {
                apiInstance.latestResult = new VisionTargetResult();
                return;
            }

            ArrayList<CandidateInfo> candidatesInsideZone = new ArrayList<>();
            ArrayList<CandidateInfo> candidatesOutsideZone = new ArrayList<>();

            // 纯粹地分类 ---
            // 遍历所有检测到的方块，仅根据位置将其放入正确的列表中。
            for (int i = 0; i < allCubes.size(); i++) {
                DetectedCube cube = allCubes.get(i);
                Point centerP = new Point(cube.centerXImagePx, cube.centerYImagePx);
                CandidateInfo c = new CandidateInfo();
                c.cubeIndex = i;
                c.centerInProcessed = centerP;
                if (targetZoneCenterXAtProcessedScale != null) {
                    c.horizontalDistanceToCenterPx = centerP.x - targetZoneCenterXAtProcessedScale;
                }

                boolean isInside = false;
                if (targetZoneContourProcessed != null && !targetZoneContourProcessed.empty()) {
                    targetZoneContourProcessed.convertTo(tempContour2f, CvType.CV_32F);
                    if (Imgproc.pointPolygonTest(tempContour2f, centerP, false) >= 0) {
                        isInside = true;
                    }
                }

                if (isInside) {
                    candidatesInsideZone.add(c);
                } else {
                    candidatesOutsideZone.add(c);
                }
            }

            // 为区域内的候选者计算详细抓取参数
            double circleRadiusP = (TARGET_RECT_WIDTH_CM / 2.0) * pixelsPerCmForDrawing * processingScale;
            if (targetZoneCenterXAtProcessedScale != null && circleRadiusP > 0) {
                for (CandidateInfo c : candidatesInsideZone) {
                    Point centerP = c.centerInProcessed;
                    double distToLine = Math.abs(c.horizontalDistanceToCenterPx);

                    // 只有当它在半圆的水平范围内时，才进行几何计算
                    if (distToLine <= circleRadiusP + 1) {
                        Point intersectP = new Point(targetZoneCenterXAtProcessedScale, Math.round(centerP.y + Math.sqrt(Math.max(0, circleRadiusP * circleRadiusP - distToLine * distToLine))));

                        // 确保计算出的投影点在有效Y范围内
                        if (targetRectY1Processed <= intersectP.y && intersectP.y <= targetRectY2Processed) {
                            c.lineAngleDeg = Math.toDegrees(Math.atan2(intersectP.x - centerP.x, intersectP.y - centerP.y));
                            double distPx = targetRectY2OriginalScale - (intersectP.y / processingScale);
                            c.distanceCm = (distPx >= 0) ? distPx / pixelsPerCmForDrawing : Double.POSITIVE_INFINITY;
                            c.primaryScore = c.distanceCm; // 使用距离作为主要排序依据
                        } else {
                            c.distanceCm = Double.POSITIVE_INFINITY; // 计算失败，设为无穷大，排序时会排到最后
                            c.primaryScore = Double.POSITIVE_INFINITY;
                        }
                    } else { // 在矩形直线部分，距离就是到底边的垂直距离
                        double distPx = targetRectY2OriginalScale - (c.centerInProcessed.y / processingScale);
                        c.distanceCm = (distPx >= 0) ? distPx / pixelsPerCmForDrawing : Double.POSITIVE_INFINITY;
                        c.primaryScore = c.distanceCm;
                        c.lineAngleDeg = (c.horizontalDistanceToCenterPx > 0) ? -90.0 : 90.0; // 直线部分，角度是固定的
                    }
                }
            }

            // 按距离对区域内候选者进行排序
            Collections.sort(candidatesInsideZone, Comparator.comparingDouble(a -> a.primaryScore));

            // 最终的、分层决策逻辑
            if (candidatesInsideZone.size() >= 2) {
                // --- 场景 1: 富矿区 (>=2) ---
                CandidateInfo bestTarget = candidatesInsideZone.get(0);
                DetectedCube bestCube = allCubes.get(bestTarget.cubeIndex);
                double widthCm = (bestCube.rectWidthPx / processingScale * bestTarget.distanceCm) / CAMERA_FOCAL_LENGTH_PIXELS;
                double heightCm = (bestCube.rectHeightPx / processingScale * bestTarget.distanceCm) / CAMERA_FOCAL_LENGTH_PIXELS;
                apiInstance.latestResult = new VisionTargetResult(true, bestCube.color, bestTarget.distanceCm, bestCube.angleDeg, bestTarget.lineAngleDeg, widthCm, heightCm, "In Position", 0, 0);

            } else if (candidatesInsideZone.size() == 1) {
                // --- 场景 2: 最后一个 (==1) ---
                CandidateInfo currentTarget = candidatesInsideZone.get(0);
                DetectedCube currentCube = allCubes.get(currentTarget.cubeIndex);
                double widthCm = (currentCube.rectWidthPx / processingScale * currentTarget.distanceCm) / CAMERA_FOCAL_LENGTH_PIXELS;
                double heightCm = (currentCube.rectHeightPx / processingScale * currentTarget.distanceCm) / CAMERA_FOCAL_LENGTH_PIXELS;

                if (!candidatesOutsideZone.isEmpty()) {
                    Collections.sort(candidatesOutsideZone, Comparator.comparingDouble(a -> Math.abs(a.horizontalDistanceToCenterPx)));
                    CandidateInfo nextTarget = candidatesOutsideZone.get(0);
                    double hOffsetPx = nextTarget.horizontalDistanceToCenterPx / processingScale;
                    String moveDir = hOffsetPx > 0 ? "Right" : "Left";
                    double distPx = targetRectY2OriginalScale - (nextTarget.centerInProcessed.y / processingScale);
                    double rawDistCm = (distPx >= 0) ? (distPx / pixelsPerCmForDrawing) : 0;
                    apiInstance.latestResult = new VisionTargetResult(true, currentCube.color, currentTarget.distanceCm, currentCube.angleDeg, currentTarget.lineAngleDeg, widthCm, heightCm, moveDir, rawDistCm, hOffsetPx);
                } else {
                    apiInstance.latestResult = new VisionTargetResult(true, currentCube.color, currentTarget.distanceCm, currentCube.angleDeg, currentTarget.lineAngleDeg, widthCm, heightCm, "In Position", 0, 0);
                }

            } else { // candidatesInsideZone.isEmpty()
                // --- 场景 3: 区域内无目标 ---
                if (!candidatesOutsideZone.isEmpty()) {
                    Collections.sort(candidatesOutsideZone, Comparator.comparingDouble(a -> Math.abs(a.horizontalDistanceToCenterPx)));
                    CandidateInfo nextTarget = candidatesOutsideZone.get(0);
                    DetectedCube nextCube = allCubes.get(nextTarget.cubeIndex);
                    double hOffsetPx = nextTarget.horizontalDistanceToCenterPx / processingScale;
                    String moveDir = hOffsetPx > 0 ? "Right" : "Left";
                    double distPx = targetRectY2OriginalScale - (nextTarget.centerInProcessed.y / processingScale);
                    double rawDistCm  = (distPx >= 0) ? (distPx / pixelsPerCmForDrawing) : 0;
                    double widthCm = (nextCube.rectWidthPx / processingScale * rawDistCm) / CAMERA_FOCAL_LENGTH_PIXELS;
                    double heightCm = (nextCube.rectHeightPx / processingScale * rawDistCm) / CAMERA_FOCAL_LENGTH_PIXELS;
                    apiInstance.latestResult = new VisionTargetResult(false, "None", Double.POSITIVE_INFINITY, 0, 0, widthCm, heightCm, moveDir, rawDistCm, hOffsetPx);
                } else {
                    // --- 场景 4: 视野里什么都没有 ---
                    apiInstance.latestResult = new VisionTargetResult();
                }
            }
        }

        private Point scalePoint(Point p, double s) { return (Math.abs(s - 1.0) < 1e-6) ? p : new Point(p.x * s, p.y * s); }
        private Point[] scaleRectPoints(Point[] pts, double s) {
            if (Math.abs(s - 1.0) < 1e-6) return pts;
            Point[] scaled = new Point[pts.length];
            for (int i = 0; i < pts.length; i++) scaled[i] = scalePoint(pts[i], s);
            return scaled;
        }

        private void drawGridOverlay(Mat f, int size, Scalar c, int t) {
            if (!ENABLE_DEBUG_VIEW) return;
            for (int x = 0; x < f.cols(); x += size) Imgproc.line(f, new Point(x, 0), new Point(x, f.rows()), c, t);
            for (int y = 0; y < f.rows(); y += size) Imgproc.line(f, new Point(0, y), new Point(f.cols(), y), c, t);
        }

        private TargetZoneInfo drawTargetZoneCm(Mat frame, double pixels_per_cm_val, int frame_width_px, int frame_height_px, double rect_width_cm, double rect_height_cm, double offset_x_cm, double offset_y_cm, Scalar color, int thickness, int arc_points_sampling) {
            if (pixels_per_cm_val <= 0) return new TargetZoneInfo(null, null, null, null);

            int rect_width_px = (int) Math.round(rect_width_cm * pixels_per_cm_val);
            int rect_height_px = (int) Math.round(rect_height_cm * pixels_per_cm_val);
            int offset_x_px = (int) Math.round(offset_x_cm * pixels_per_cm_val);
            int offset_y_px = (int) Math.round(offset_y_cm * pixels_per_cm_val);
            if (rect_width_px <= 0 || rect_height_px <= 0) return new TargetZoneInfo(null, null, null, null);

            int rect_center_x_px = frame_width_px / 2 + offset_x_px;
            int rect_center_y_px = frame_height_px / 2 + offset_y_px;
            int x1 = rect_center_x_px - rect_width_px / 2;
            int y1_top_edge = rect_center_y_px - rect_height_px / 2;
            int x2 = rect_center_x_px + rect_width_px / 2;
            int y2_bottom_edge = rect_center_y_px + rect_height_px / 2;
            ArrayList<Point> contour_points = new ArrayList<>();
            int arc_radius_px = rect_width_px / 2;

            if (arc_radius_px > 0) {
                if (ENABLE_DEBUG_VIEW) Imgproc.ellipse(frame, new Point(rect_center_x_px, y2_bottom_edge), new Size(arc_radius_px, arc_radius_px), 0, 180, 360, color, thickness);
                for (int i = 0; i <= arc_points_sampling; i++) {
                    double angle_rad = Math.toRadians(360 - (i * 180.0 / arc_points_sampling));
                    contour_points.add(new Point(rect_center_x_px + (int)Math.round(arc_radius_px * Math.cos(angle_rad)), y2_bottom_edge + (int)Math.round(arc_radius_px * Math.sin(angle_rad))));
                }
            } else {
                contour_points.add(new Point(x2, y2_bottom_edge));
                contour_points.add(new Point(x1, y2_bottom_edge));
            }

            if (ENABLE_DEBUG_VIEW) Imgproc.line(frame, new Point(x1, y1_top_edge), new Point(x1, y2_bottom_edge), color, thickness);
            if (!contour_points.isEmpty() && (contour_points.get(contour_points.size() - 1).x != x1 || contour_points.get(contour_points.size() - 1).y != y2_bottom_edge)) contour_points.add(new Point(x1, y2_bottom_edge));
            contour_points.add(new Point(x1, y1_top_edge));

            if (arc_radius_px > 0) {
                if (ENABLE_DEBUG_VIEW) Imgproc.ellipse(frame, new Point(rect_center_x_px, y1_top_edge), new Size(arc_radius_px, arc_radius_px), 0, 180, 360, color, thickness);
                for (int i = 0; i <= arc_points_sampling; i++) {
                    double angle_rad = Math.toRadians(180 + (i * 180.0 / arc_points_sampling));
                    contour_points.add(new Point(rect_center_x_px + (int)Math.round(arc_radius_px * Math.cos(angle_rad)), y1_top_edge + (int)Math.round(arc_radius_px * Math.sin(angle_rad))));
                }
            } else {
                contour_points.add(new Point(x2, y1_top_edge));
            }

            if (ENABLE_DEBUG_VIEW) Imgproc.line(frame, new Point(x2, y1_top_edge), new Point(x2, y2_bottom_edge), color, thickness);
            if (!contour_points.isEmpty() && (contour_points.get(contour_points.size() - 1).x != x2 || contour_points.get(contour_points.size() - 1).y != y1_top_edge)) contour_points.add(new Point(x2, y1_top_edge));
            contour_points.add(new Point(x2, y2_bottom_edge));

            MatOfPoint final_contour = new MatOfPoint();
            final_contour.fromList(contour_points);

            return new TargetZoneInfo(final_contour, rect_center_x_px, y1_top_edge, y2_bottom_edge);
        }

        @Override
        public void onViewportTapped() {
            viewportPaused = !viewportPaused;
            if (apiInstance.webcam != null) {
                if (viewportPaused) apiInstance.webcam.pauseViewport(); else apiInstance.webcam.resumeViewport();
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
        }
    }
}