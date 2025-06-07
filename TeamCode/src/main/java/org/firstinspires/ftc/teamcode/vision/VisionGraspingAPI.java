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
import java.util.Arrays;
import java.util.Collections;
import java.util.HashMap;
import java.util.List;
import java.util.Locale;
import java.util.Map;

/**
 * 视觉抓取API (Vision Grasping API)
 * 封装了所有摄像头和OpenCV图像处理的逻辑，对外提供简单的接口获取识别结果。
 * @author BlueDarkUP
 * @version 2025/6
 */
public class VisionGraspingAPI {

    // --- 可配置常量 ---
    public static final String WEBCAM_NAME_STR = "Webcam";
    public static final int CAMERA_WIDTH = 1280;
    public static final int CAMERA_HEIGHT = 720;
    public static final double REAL_WORLD_VIEW_WIDTH_CM = 92.0;

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

        public VisionTargetResult() {
            this.isTargetFound = false;
            this.color = "None";
            this.distanceCm = Double.POSITIVE_INFINITY;
            this.objectAngleDeg = 0.0;
            this.lineAngleDeg = 0.0;
        }

        public VisionTargetResult(boolean isFound, String color, double distance, double objectAngle, double lineAngle) {
            this.isTargetFound = isFound;
            this.color = color;
            this.distanceCm = distance;
            this.objectAngleDeg = objectAngle;
            this.lineAngleDeg = lineAngle;
        }
    }

    private static class SamplePipeline extends OpenCvPipeline {

        // --- Pipeline 内部常量 ---
        public static final boolean ENABLE_DEBUG_VIEW = false;
        public static final double PIXELS_PER_CM = CAMERA_WIDTH / REAL_WORLD_VIEW_WIDTH_CM;
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

        // --- Pipeline-specific helper classes ---
        static class DetectedCube {
            public String color;
            public int centerXImagePx, centerYImagePx;
            public Point[] boundingBoxPoints;
            public double scaleFactor, angleDeg;
            public DetectedCube(String color, int centerX, int centerY, Point[] points, double scale, double angle) {
                this.color = color; this.centerXImagePx = centerX; this.centerYImagePx = centerY;
                this.boundingBoxPoints = points; this.scaleFactor = scale; this.angleDeg = angle;
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
            TargetZoneInfo info = drawTargetZoneCm(dummyFrame, PIXELS_PER_CM, CAMERA_WIDTH, CAMERA_HEIGHT, TARGET_RECT_WIDTH_CM, TARGET_RECT_HEIGHT_CM, TARGET_RECT_OFFSET_X_CM, TARGET_RECT_OFFSET_Y_CM, TARGET_RECT_COLOR, TARGET_RECT_THICKNESS, ARC_SAMPLING_POINTS);
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
                    allDetectedCubes.add(new DetectedCube(entry.getKey(), (int)minAreaRect.center.x, (int)minAreaRect.center.y, boxPoints, processingScale, angle));
                    contour.release();
                }
            }

            drawDetections(bgr, allDetectedCubes, drawScale, processingScale, targetZoneContourAtProcessedScale, targetZoneCenterXAtProcessedScale, targetRectY1AtProcessedScale, targetRectY2AtProcessedScale, PIXELS_PER_CM, targetRectY2AtOriginalScale);

            if (ENABLE_DEBUG_VIEW) {
                TargetZoneInfo finalDisplayZoneInfo = drawTargetZoneCm(bgr, PIXELS_PER_CM, CAMERA_WIDTH, CAMERA_HEIGHT, TARGET_RECT_WIDTH_CM, TARGET_RECT_HEIGHT_CM, TARGET_RECT_OFFSET_X_CM, TARGET_RECT_OFFSET_Y_CM, TARGET_RECT_COLOR, TARGET_RECT_THICKNESS, ARC_SAMPLING_POINTS);
                if (finalDisplayZoneInfo.centerX != null) {
                    Imgproc.line(bgr, new Point(finalDisplayZoneInfo.centerX, 0), new Point(finalDisplayZoneInfo.centerX, CAMERA_HEIGHT), TARGET_RECT_COLOR, 1);
                    if (finalDisplayZoneInfo.targetContour != null) finalDisplayZoneInfo.targetContour.release();
                }
                drawGridOverlay(bgr, Math.max(1, (int)Math.round(5 * PIXELS_PER_CM)), GRID_COLOR, 1);
            }

            Imgproc.cvtColor(bgr, inputRGBA, Imgproc.COLOR_BGR2RGBA);
            return inputRGBA;
        }

        private void drawDetections(Mat displayOutput, List<DetectedCube> cubes, double drawScaleToDisplay, double processingScale, MatOfPoint targetZoneContourProcessed, Integer targetZoneCenterXAtProcessedScale, Integer targetRectY1Processed, Integer targetRectY2Processed, double originalPixelsPerCm, double targetRectY2OriginalScale) {
            if (cubes.isEmpty()) {
                apiInstance.latestResult = new VisionTargetResult();
                return;
            }

            ArrayList<CandidateInfo> candidates = new ArrayList<>();
            double circleRadiusP = (TARGET_RECT_WIDTH_CM / 2.0) * originalPixelsPerCm * processingScale;

            for (int i = 0; i < cubes.size(); i++) {
                DetectedCube cube = cubes.get(i);
                Point centerP = new Point(cube.centerXImagePx, cube.centerYImagePx);
                Point intersectP = null;
                double lineAngle = 0.0, distCm = Double.POSITIVE_INFINITY;

                if (targetZoneCenterXAtProcessedScale != null && circleRadiusP > 0) {
                    double distToLine = Math.abs(targetZoneCenterXAtProcessedScale - centerP.x);
                    if (distToLine <= circleRadiusP + 1) {
                        intersectP = new Point(targetZoneCenterXAtProcessedScale, Math.round(centerP.y + Math.sqrt(Math.max(0, circleRadiusP*circleRadiusP - distToLine*distToLine))));
                        lineAngle = Math.toDegrees(Math.atan2(intersectP.x - centerP.x, intersectP.y - centerP.y));
                        double distPx = targetRectY2OriginalScale - (intersectP.y / processingScale);
                        if (distPx >= 0) distCm = distPx / originalPixelsPerCm;
                    }
                }

                boolean isValid = true;
                if (targetZoneContourProcessed != null && !targetZoneContourProcessed.empty()) {
                    targetZoneContourProcessed.convertTo(tempContour2f, CvType.CV_32F);
                    if (Imgproc.pointPolygonTest(tempContour2f, centerP, false) < 0) isValid = false;
                }
                if (isValid && (intersectP == null || !(targetRectY1Processed <= intersectP.y && intersectP.y <= targetRectY2Processed))) isValid = false;

                if (isValid) {
                    CandidateInfo c = new CandidateInfo();
                    c.cubeIndex = i; c.primaryScore = distCm; c.secondaryScore = Math.abs(cube.angleDeg - 90.0) + Math.abs(lineAngle);
                    c.lineAngleDeg = lineAngle; c.centerInProcessed = centerP; c.intersectionPointProcessed = intersectP; c.distanceCm = distCm;
                    candidates.add(c);
                }
            }

            if (!candidates.isEmpty()) {
                Collections.sort(candidates, (a, b) -> (Math.abs(a.primaryScore - b.primaryScore) > 1e-6) ? Double.compare(a.primaryScore, b.primaryScore) : Double.compare(a.secondaryScore, b.secondaryScore));
                if (candidates.get(0).primaryScore != Double.POSITIVE_INFINITY) {
                    DetectedCube bestCube = cubes.get(candidates.get(0).cubeIndex);
                    apiInstance.latestResult = new VisionTargetResult(true, bestCube.color, candidates.get(0).distanceCm, bestCube.angleDeg, candidates.get(0).lineAngleDeg);
                } else {
                    apiInstance.latestResult = new VisionTargetResult();
                }
            } else {
                apiInstance.latestResult = new VisionTargetResult();
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