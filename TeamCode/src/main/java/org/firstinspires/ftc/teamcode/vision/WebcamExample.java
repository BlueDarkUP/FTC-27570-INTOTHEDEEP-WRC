/**
 * 我会努力让视觉程序变的优雅的
 * @author BlueDarkUP
 * @version 2025/6
 *
 */
package org.firstinspires.ftc.teamcode.vision;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.API.ServoKinematics;
import org.firstinspires.ftc.teamcode.API.ServoKinematics.ServoTarget;
import org.firstinspires.ftc.teamcode.API.PositionCalculator;
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

@TeleOp(name="WebcamExample", group="Vision")
public class WebcamExample extends LinearOpMode {

    // [新增] 调试视图总开关。调试时设为 true，比赛时设为 false 以提升性能。
    public static final boolean ENABLE_DEBUG_VIEW = true;

    public static final String WEBCAM_NAME_STR = "Webcam";
    public static final int CAMERA_WIDTH = 1280;
    public static final int CAMERA_HEIGHT = 720;

    public static final double REAL_WORLD_VIEW_WIDTH_CM = 92.0;
    public static double PIXELS_PER_CM;

    public static final double TARGET_RECT_WIDTH_CM = 31.5;
    public static final double TARGET_RECT_HEIGHT_CM = 24.0;
    public static final double TARGET_RECT_OFFSET_X_CM = 0.0;
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
        COLOR_HSV_RANGES.put("YELLOW", new Scalar[][]{
                // 默认范围
                {new Scalar(15, 80, 100), new Scalar(45, 255, 255)},
                // 可以添加更多范围
        });
    }


    public static final Scalar BOX_COLOR_DEFAULT = new Scalar(0, 255, 0);
    public static final Scalar BOX_COLOR_SELECTED_BEST = new Scalar(0, 0, 255);
    public static final Scalar BLOCK_CENTER_CIRCLE_COLOR = new Scalar(0, 255, 255);
    public static final Scalar INTERSECTION_POINT_COLOR = new Scalar(0, 0, 255);
    public static final Scalar CENTER_TO_INTERSECTION_LINE_COLOR = new Scalar(200, 200, 0);
    public static final Scalar ANGLE_TEXT_COLOR = new Scalar(200, 200, 0);
    public static final Scalar DISTANCE_TEXT_COLOR = new Scalar(200, 100, 200);
    public static final Scalar GRID_COLOR = new Scalar(70, 70, 70);


    OpenCvWebcam webcam;
    SamplePipeline pipeline;

    @Override
    public void runOpMode() {
        if (REAL_WORLD_VIEW_WIDTH_CM > 0 && CAMERA_WIDTH > 0) {
            PIXELS_PER_CM = CAMERA_WIDTH / REAL_WORLD_VIEW_WIDTH_CM;
            telemetry.addData("Vision", String.format(Locale.US, "PIXELS_PER_CM: %.2f", PIXELS_PER_CM));
        } else {
            PIXELS_PER_CM = 0;
            telemetry.addData("Vision", "WARNING: PIXELS_PER_CM is zero or undefined!");
        }


        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, WEBCAM_NAME_STR), cameraMonitorViewId);
        pipeline = new SamplePipeline();
        webcam.setPipeline(pipeline);
        webcam.setMillisecondsPermissionTimeout(100000);
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                webcam.startStreaming(CAMERA_WIDTH, CAMERA_HEIGHT, OpenCvCameraRotation.UPRIGHT);
                telemetry.addData("Camera", "Streaming started at " + CAMERA_WIDTH + "x" + CAMERA_HEIGHT);
                telemetry.update();
            }
            @Override
            public void onError(int errorCode) {
                telemetry.addData("Camera", "Error opening camera: " + errorCode);
                telemetry.update();
            }
        });

        telemetry.addLine("Waiting for start");
        telemetry.update();

        waitForStart();

        ElapsedTime timer = new ElapsedTime();
        while (opModeIsActive()) {
            telemetry.addData("Frame Count", webcam.getFrameCount());
            telemetry.addData("FPS", String.format(Locale.US, "%.2f", webcam.getFps()));
            telemetry.addData("Pipeline time ms", webcam.getPipelineTimeMs());
            telemetry.addLine("--- Vision Results ---");
            telemetry.addData("Cubes Detected", pipeline.latestNumberOfCubesDetected);
            telemetry.addData("Best Cube Color", pipeline.latestBestCubeColor);
            telemetry.addData("Best Cube Dist (Target)", String.format(Locale.US, "%.1f cm", pipeline.latestBestCubeDistance));
            telemetry.addData("α (物体姿态角)", String.format(Locale.US, "%.1f deg", pipeline.latestBestCubeAngle));
            telemetry.addData("β (目标连线角)", String.format(Locale.US, "%.1f deg", pipeline.latestBestCubeLineAngle));

            telemetry.addLine("--- 机械臂与舵机计算 ---");
            // 1. 滑轨舵机 (基于距离)
            // 调用运动学 API 计算滑轨舵机的位置
            ServoTarget sliderTarget = ServoKinematics.calculateServoTarget(pipeline.latestBestCubeDistance);

            // 检查返回值是否有效并显示
            if (sliderTarget != null) {
                telemetry.addData("滑轨舵机 (距离) 位置", String.format(Locale.US, "%.4f", sliderTarget.servoPosition));
                telemetry.addData("滑轨舵机 (距离) 角度", String.format(Locale.US, "%.1f deg", sliderTarget.rotationDegrees));
            } else {
                telemetry.addData("滑轨舵机 (距离)", "N/A (无目标或超范围)");
            }

            // 2. A/B舵机 (基于角度)
            // 检查是否检测到有效目标
            if (!pipeline.latestBestCubeColor.equals("None")) {
                double alpha = pipeline.latestBestCubeAngle;      // 物体自身姿态角 (α)
                double beta = pipeline.latestBestCubeLineAngle;   // 目标连线与垂直线的夹角 (β)

                // A舵机角度 = -β (用于左右对准)
                double servoAAngle = -beta;

                // B舵机角度 = α + β (用于姿态校准)
                double servoBAngle = alpha + beta;

                // 通过API计算舵机目标位置
                double TURN_SERVO_POS = PositionCalculator.calculatePositionValue(0.00, 0.00, 0.00, false, servoAAngle);    // 具体数值等待实际测量
                double ROTATE_SERVO_POS = PositionCalculator.calculatePositionValue(0.00, 0.00, 0.00, false, servoBAngle);  // 具体数值等待实际测量
                // 显示计算出的 A 和 B 舵机角度
                telemetry.addData("A舵机 (偏航) 目标角度", String.format(Locale.US, "%.1f deg", servoAAngle));
                telemetry.addData("B舵机 (校准) 目标角度", String.format(Locale.US, "%.1f deg", servoBAngle));
                telemetry.addData("转台舵机目标位置", String.format(Locale.US, "%.1f deg", TURN_SERVO_POS));
                telemetry.addData("旋转舵机目标位置", String.format(Locale.US, "%.1f deg", ROTATE_SERVO_POS));


            } else {
                // 如果未检测到目标，则显示 "N/A"
                telemetry.addData("A舵机 (偏航) 目标角度", "N/A (无目标)");
                telemetry.addData("B舵机 (校准) 目标角度", "N/A (无目标)");
            }


            telemetry.update();

            if (gamepad1.a) {
                webcam.stopStreaming();
                telemetry.addLine("Streaming stopped.");
                telemetry.update();
            }

            sleep(100);
        }

        if (pipeline != null) {
            pipeline.releaseMats();
        }
    }
    static class DetectedCube {
        public String color;
        public int centerXImagePx;
        public int centerYImagePx;
        public Point[] boundingBoxPoints;
        public double scaleFactor;
        public double angleDeg;

        public DetectedCube(String color, int centerXImagePx, int centerYImagePx,
                            Point[] boundingBoxPoints, double scaleFactor, double angleDeg) {
            this.color = color;
            this.centerXImagePx = centerXImagePx;
            this.centerYImagePx = centerYImagePx;
            this.boundingBoxPoints = boundingBoxPoints;
            this.scaleFactor = scaleFactor;
            this.angleDeg = angleDeg;
        }

        @Override
        public String toString() {
            return String.format(Locale.US, "Cube[Color:%s, Center:(%d,%d), Angle:%.1fdeg @%.2fx]",
                    color, centerXImagePx, centerYImagePx, angleDeg, scaleFactor);
        }
    }

    static class TargetZoneInfo {
        public MatOfPoint targetContour;
        public Integer centerX;
        public Integer topY;
        public Integer bottomY;

        public TargetZoneInfo(MatOfPoint targetContour, Integer centerX, Integer topY, Integer bottomY) {
            this.targetContour = targetContour;
            this.centerX = centerX;
            this.topY = topY;
            this.bottomY = bottomY;
        }
    }

    class SamplePipeline extends OpenCvPipeline {
        private Mat bgr;
        private Mat hsv;
        private Mat processedFrameForDetection;
        private Mat masterMask;
        private Mat maskCombined;
        private Mat colorMask;
        private Mat medianBlurred;
        private Mat opened;
        private Mat sureBg;
        private Mat distTransform;
        private Mat sureFg;
        private Mat unknown;
        private Mat markers;
        private Mat imgForWatershed;

        private Mat kernel3x3;

        private MatOfPoint2f tempContour2f;
        private MatOfPoint tempScaledBoxPointsForDisplay;

        private MatOfPoint targetZoneContourAtOriginalScale;
        private Integer targetZoneCenterXAtOriginalScale;
        private Integer targetRectY1AtOriginalScale;
        private Integer targetRectY2AtOriginalScale;


        private MatOfPoint targetZoneContourAtProcessedScaleHolder;
        public String latestBestCubeColor = "None";
        public double latestBestCubeAngle = 0.0;
        public double latestBestCubeDistance = Double.POSITIVE_INFINITY;
        public double latestBestCubeLineAngle = 0.0;
        public int latestNumberOfCubesDetected = 0;

        private boolean viewportPaused;

        public SamplePipeline() {
            Mat dummyFrame = new Mat(CAMERA_HEIGHT, CAMERA_WIDTH, CvType.CV_8UC3);
            TargetZoneInfo info = drawTargetZoneCm(dummyFrame, PIXELS_PER_CM,
                    CAMERA_WIDTH, CAMERA_HEIGHT,
                    TARGET_RECT_WIDTH_CM, TARGET_RECT_HEIGHT_CM,
                    TARGET_RECT_OFFSET_X_CM, TARGET_RECT_OFFSET_Y_CM,
                    TARGET_RECT_COLOR, TARGET_RECT_THICKNESS, ARC_SAMPLING_POINTS);
            targetZoneContourAtOriginalScale = info.targetContour;
            targetZoneCenterXAtOriginalScale = info.centerX;
            targetRectY1AtOriginalScale = info.topY;
            targetRectY2AtOriginalScale = info.bottomY;
            dummyFrame.release();
        }

        @Override
        public void init(Mat firstFrame) {
            int originalHeight = firstFrame.rows();
            int originalWidth = firstFrame.cols();

            int processingWidth = (int) (originalWidth * DOWNSCALE_FACTOR);
            int processingHeight = (int) (originalHeight * DOWNSCALE_FACTOR);
            if (DOWNSCALE_FACTOR <= 0 || DOWNSCALE_FACTOR > 1) {
                processingWidth = originalWidth;
                processingHeight = originalHeight;
            }

            bgr = new Mat(originalHeight, originalWidth, CvType.CV_8UC3);
            hsv = new Mat(processingHeight, processingWidth, CvType.CV_8UC3);
            masterMask = new Mat(processingHeight, processingWidth, CvType.CV_8U);
            maskCombined = new Mat(processingHeight, processingWidth, CvType.CV_8U);
            colorMask = new Mat(processingHeight, processingWidth, CvType.CV_8U);
            medianBlurred = new Mat(processingHeight, processingWidth, CvType.CV_8U);
            opened = new Mat(processingHeight, processingWidth, CvType.CV_8U);
            sureBg = new Mat(processingHeight, processingWidth, CvType.CV_8U);
            distTransform = new Mat(processingHeight, processingWidth, CvType.CV_32F);
            sureFg = new Mat(processingHeight, processingWidth, CvType.CV_8U);
            unknown = new Mat(processingHeight, processingWidth, CvType.CV_8U);
            markers = new Mat(processingHeight, processingWidth, CvType.CV_32S);
            imgForWatershed = new Mat(processingHeight, processingWidth, CvType.CV_8UC3);

            kernel3x3 = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(3, 3));

            processedFrameForDetection = new Mat(processingHeight, processingWidth, CvType.CV_8UC3);
            targetZoneContourAtProcessedScaleHolder = new MatOfPoint();

            tempContour2f = new MatOfPoint2f();
            tempScaledBoxPointsForDisplay = new MatOfPoint();
        }

        @Override
        public Mat processFrame(Mat inputRGBA) {
            Imgproc.cvtColor(inputRGBA, bgr, Imgproc.COLOR_RGBA2BGR);

            double drawScale = 1.0;
            double processingScale = 1.0;

            MatOfPoint targetZoneContourAtProcessedScale = null;
            Integer targetZoneCenterXAtProcessedScale = null;
            Integer targetRectY1AtProcessedScale = null;
            Integer targetRectY2AtProcessedScale = null;

            if (DOWNSCALE_FACTOR > 0 && DOWNSCALE_FACTOR < 1.0) {
                processingScale = DOWNSCALE_FACTOR;
                drawScale = 1.0 / DOWNSCALE_FACTOR;

                int scaledWidth = (int) (bgr.cols() * processingScale);
                int scaledHeight = (int) (bgr.rows() * processingScale);

                Imgproc.resize(bgr, processedFrameForDetection, new Size(scaledWidth, scaledHeight), 0, 0, Imgproc.INTER_LINEAR);

                if (targetZoneContourAtOriginalScale != null) {
                    List<Point> originalPoints = targetZoneContourAtOriginalScale.toList();
                    List<Point> scaledPoints = new ArrayList<>();
                    for (Point p : originalPoints) {
                        scaledPoints.add(new Point(Math.round(p.x * processingScale), Math.round(p.y * processingScale)));
                    }
                    targetZoneContourAtProcessedScaleHolder.fromList(scaledPoints);
                    targetZoneContourAtProcessedScale = targetZoneContourAtProcessedScaleHolder;
                }

                if (targetZoneCenterXAtOriginalScale != null) {
                    targetZoneCenterXAtProcessedScale = (int) Math.round(targetZoneCenterXAtOriginalScale * processingScale);
                }
                if (targetRectY1AtOriginalScale != null) {
                    targetRectY1AtProcessedScale = (int) Math.round(targetRectY1AtOriginalScale * processingScale);
                }
                if (targetRectY2AtOriginalScale != null) {
                    targetRectY2AtProcessedScale = (int) Math.round(targetRectY2AtOriginalScale * processingScale);
                }
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

            double scaledMinAreaPixels = MIN_SIZE_PIXELS * (processingScale * processingScale);
            double scaledMaxAreaPixels = MAX_SIZE_PIXELS * (processingScale * processingScale);

            for (Map.Entry<String, Scalar[][]> entry : COLOR_HSV_RANGES.entrySet()) {
                String colorName = entry.getKey();
                Scalar[][] ranges = entry.getValue();

                colorMask.setTo(new Scalar(0));

                for (Scalar[] range : ranges) {
                    Core.inRange(hsv, range[0], range[1], maskCombined);
                    Core.bitwise_or(colorMask, maskCombined, colorMask);
                }

                Core.bitwise_or(masterMask, colorMask, masterMask);

                Imgproc.medianBlur(colorMask, medianBlurred, 3);

                Imgproc.morphologyEx(medianBlurred, opened, Imgproc.MORPH_OPEN, kernel3x3, new Point(-1, -1), 2);

                Imgproc.dilate(opened, sureBg, kernel3x3, new Point(-1, -1), 2);

                Imgproc.distanceTransform(opened, distTransform, Imgproc.DIST_L2, 5);
                Core.MinMaxLocResult mmr = Core.minMaxLoc(distTransform);
                double distThreshold = 0.57 * mmr.maxVal;
                Imgproc.threshold(distTransform, sureFg, distThreshold, 255, Imgproc.THRESH_BINARY);
                sureFg.convertTo(sureFg, CvType.CV_8U);

                Core.subtract(sureBg, sureFg, unknown);

                Imgproc.connectedComponents(sureFg, markers);

                markers.setTo(new Scalar(0), unknown);

                Imgproc.GaussianBlur(processedFrameForDetection, imgForWatershed, new Size(5, 5), 0);

                Imgproc.watershed(imgForWatershed, markers);

                Core.MinMaxLocResult markerMax = Core.minMaxLoc(markers);
                int maxMarkerId = (int) markerMax.maxVal;

                List<MatOfPoint> contours = new ArrayList<>();
                Mat hierarchy = new Mat();

                for (int markerId = 1; markerId <= maxMarkerId; markerId++) {
                    colorMask.setTo(new Scalar(0));
                    Core.compare(markers, new Scalar(markerId), colorMask, Core.CMP_EQ);

                    contours.clear();
                    Imgproc.findContours(colorMask, contours, hierarchy, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);

                    for (MatOfPoint contour : contours) {
                        double area = Imgproc.contourArea(contour);
                        if (!(scaledMinAreaPixels <= area && area <= scaledMaxAreaPixels)) {
                            contour.release();
                            continue;
                        }

                        if (contour.rows() < 5) {
                            contour.release();
                            continue;
                        }

                        contour.convertTo(tempContour2f, CvType.CV_32F);
                        RotatedRect minAreaRect = Imgproc.minAreaRect(tempContour2f);
                        Point rectCenter = minAreaRect.center;
                        Size rectSize = minAreaRect.size;
                        double cvAngle = minAreaRect.angle;

                        double rectWidthRot = rectSize.width;
                        double rectHeightRot = rectSize.height;

                        double objectOrientationAngleDeg;
                        if (rectWidthRot < rectHeightRot) {
                            objectOrientationAngleDeg = cvAngle + 90.0;
                        } else {
                            objectOrientationAngleDeg = cvAngle;
                        }

                        double side1 = rectWidthRot;
                        double side2 = rectHeightRot;
                        if (side1 < 1e-3 || side2 < 1e-3) {
                            contour.release();
                            continue;
                        }
                        double longSide = Math.max(side1, side2);
                        double shortSide = Math.min(side1, side2);
                        double detectedAspectRatio = longSide / shortSide;

                        double lowerBoundAR = TARGET_OBJECT_ASPECT_RATIO * (1.0 - ASPECT_RATIO_TOLERANCE_PERCENT);
                        double upperBoundAR = TARGET_OBJECT_ASPECT_RATIO * (1.0 + ASPECT_RATIO_TOLERANCE_PERCENT);

                        if (!(lowerBoundAR <= detectedAspectRatio && detectedAspectRatio <= upperBoundAR)) {
                            contour.release();
                            continue;
                        }

                        Point[] boxPoints = new Point[4];
                        minAreaRect.points(boxPoints);

                        allDetectedCubes.add(new DetectedCube(
                                colorName,
                                (int) Math.round(rectCenter.x),
                                (int) Math.round(rectCenter.y),
                                boxPoints,
                                processingScale,
                                objectOrientationAngleDeg
                        ));
                        contour.release();
                    }
                }
                hierarchy.release();
            }

            // 识别和评分逻辑总是执行
            drawDetections(bgr, allDetectedCubes, drawScale,
                    targetZoneContourAtProcessedScale, targetZoneCenterXAtProcessedScale,
                    targetRectY1AtProcessedScale, targetRectY2AtProcessedScale,
                    PIXELS_PER_CM, targetRectY2AtOriginalScale);

            // [修改] 只有在调试模式开启时，才绘制辅助线和网格
            if (WebcamExample.ENABLE_DEBUG_VIEW) {
                // 绘制目标区域
                TargetZoneInfo finalDisplayZoneInfo = drawTargetZoneCm(bgr, PIXELS_PER_CM,
                        CAMERA_WIDTH, CAMERA_HEIGHT,
                        TARGET_RECT_WIDTH_CM, TARGET_RECT_HEIGHT_CM,
                        TARGET_RECT_OFFSET_X_CM, TARGET_RECT_OFFSET_Y_CM,
                        TARGET_RECT_COLOR, TARGET_RECT_THICKNESS, ARC_SAMPLING_POINTS);

                // 绘制中心垂直线
                if (finalDisplayZoneInfo.centerX != null) {
                    Imgproc.line(bgr, new Point(finalDisplayZoneInfo.centerX, 0),
                            new Point(finalDisplayZoneInfo.centerX, CAMERA_HEIGHT), TARGET_RECT_COLOR, 1);
                    if (finalDisplayZoneInfo.targetContour != null) {
                        finalDisplayZoneInfo.targetContour.release();
                    }
                }

                // 绘制网格
                int grid_size_in_pixels = (int) Math.round(5 * PIXELS_PER_CM);
                grid_size_in_pixels = Math.max(1, grid_size_in_pixels);
                drawGridOverlay(bgr, grid_size_in_pixels, GRID_COLOR, 1);
            }

            Imgproc.cvtColor(bgr, inputRGBA, Imgproc.COLOR_BGR2RGBA);

            return inputRGBA;
        }

        private Point scalePoint(Point point, double scaleToApply) {
            if (Math.abs(scaleToApply - 1.0) < 1e-6) {
                return point;
            }
            return new Point(point.x * scaleToApply, point.y * scaleToApply);
        }
        private Point[] scaleRectPoints(Point[] boxPoints, double scaleToApply) {
            if (Math.abs(scaleToApply - 1.0) < 1e-6) {
                return boxPoints;
            }
            Point[] scaledPoints = new Point[boxPoints.length];
            for (int i = 0; i < boxPoints.length; i++) {
                scaledPoints[i] = new Point(boxPoints[i].x * scaleToApply, boxPoints[i].y * scaleToApply);
            }
            return scaledPoints;
        }
        private void drawGridOverlay(Mat frame, int grid_size_px, Scalar color, int thickness) {
            // [修改] 如果关闭调试视图，直接返回，不执行任何绘制
            if (!WebcamExample.ENABLE_DEBUG_VIEW) {
                return;
            }
            int h = frame.rows();
            int w = frame.cols();
            if (grid_size_px < 1) grid_size_px = 1;
            for (int x = 0; x < w; x += grid_size_px) {
                Imgproc.line(frame, new Point(x, 0), new Point(x, h), color, thickness);
            }
            for (int y = 0; y < h; y += grid_size_px) {
                Imgproc.line(frame, new Point(0, y), new Point(w, y), color, thickness);
            }
        }
        private TargetZoneInfo drawTargetZoneCm(Mat frame, double pixels_per_cm_val,
                                                int frame_width_px, int frame_height_px,
                                                double rect_width_cm, double rect_height_cm,
                                                double offset_x_cm, double offset_y_cm,
                                                Scalar color, int thickness,
                                                int arc_points_sampling) {
            if (pixels_per_cm_val <= 0) {
                return new TargetZoneInfo(null, null, null, null);
            }

            int rect_width_px = (int) Math.round(rect_width_cm * pixels_per_cm_val);
            int rect_height_px = (int) Math.round(rect_height_cm * pixels_per_cm_val);
            int offset_x_px = (int) Math.round(offset_x_cm * pixels_per_cm_val);
            int offset_y_px = (int) Math.round(offset_y_cm * pixels_per_cm_val);
            if (rect_width_px <= 0 || rect_height_px <= 0) {
                return new TargetZoneInfo(null, null, null, null);
            }

            int frame_center_x_px = frame_width_px / 2;
            int frame_center_y_px = frame_height_px / 2;
            int rect_center_x_px = frame_center_x_px + offset_x_px;
            int rect_center_y_px = frame_center_y_px + offset_y_px;
            int x1 = rect_center_x_px - rect_width_px / 2;
            int y1_top_edge = rect_center_y_px - rect_height_px / 2;
            int x2 = rect_center_x_px + rect_width_px / 2;
            int y2_bottom_edge = rect_center_y_px + rect_height_px / 2;
            ArrayList<Point> contour_points = new ArrayList<>();
            int arc_radius_px = rect_width_px / 2;

            int arc_b_center_y = y2_bottom_edge;
            if (arc_radius_px > 0) {
                // [修改] 将绘图操作包裹在调试开关内
                if (WebcamExample.ENABLE_DEBUG_VIEW) {
                    Imgproc.ellipse(frame, new Point(rect_center_x_px, arc_b_center_y), new Size(arc_radius_px, arc_radius_px), 0, 180, 360, color, thickness);
                }
                for (int i = 0; i <= arc_points_sampling; i++) {
                    double angle_rad = Math.toRadians(360 - (i * 180.0 / arc_points_sampling));
                    int pt_x = rect_center_x_px + (int) Math.round(arc_radius_px * Math.cos(angle_rad));
                    int pt_y = arc_b_center_y + (int) Math.round(arc_radius_px * Math.sin(angle_rad));
                    contour_points.add(new Point(pt_x, pt_y));
                }
            } else {
                contour_points.add(new Point(x2, y2_bottom_edge));
                contour_points.add(new Point(x1, y2_bottom_edge));
            }
            // [修改] 将绘图操作包裹在调试开关内
            if (WebcamExample.ENABLE_DEBUG_VIEW) {
                Imgproc.line(frame, new Point(x1, y1_top_edge), new Point(x1, y2_bottom_edge), color, thickness);
            }
            if (!contour_points.isEmpty() && (contour_points.get(contour_points.size() - 1).x != x1 || contour_points.get(contour_points.size() - 1).y != y2_bottom_edge)) {
                contour_points.add(new Point(x1, y2_bottom_edge));
            }
            contour_points.add(new Point(x1, y1_top_edge));
            int arc_a_center_y = y1_top_edge;
            if (arc_radius_px > 0) {
                // [修改] 将绘图操作包裹在调试开关内
                if (WebcamExample.ENABLE_DEBUG_VIEW) {
                    Imgproc.ellipse(frame, new Point(rect_center_x_px, arc_a_center_y), new Size(arc_radius_px, arc_radius_px), 0, 180, 360, color, thickness);
                }
                for (int i = 0; i <= arc_points_sampling; i++) {
                    double angle_rad = Math.toRadians(180 + (i * 180.0 / arc_points_sampling));
                    int pt_x = rect_center_x_px + (int) Math.round(arc_radius_px * Math.cos(angle_rad));
                    int pt_y = arc_a_center_y + (int) Math.round(arc_radius_px * Math.sin(angle_rad));
                    contour_points.add(new Point(pt_x, pt_y));
                }
            } else {
                contour_points.add(new Point(x2, y1_top_edge));
            }
            // [修改] 将绘图操作包裹在调试开关内
            if (WebcamExample.ENABLE_DEBUG_VIEW) {
                Imgproc.line(frame, new Point(x2, y1_top_edge), new Point(x2, y2_bottom_edge), color, thickness);
            }
            if (!contour_points.isEmpty() && (contour_points.get(contour_points.size() - 1).x != x2 || contour_points.get(contour_points.size() - 1).y != y1_top_edge)) {
                contour_points.add(new Point(x2, y1_top_edge));
            }
            contour_points.add(new Point(x2, y2_bottom_edge));

            MatOfPoint final_contour = new MatOfPoint();
            final_contour.fromList(contour_points);

            return new TargetZoneInfo(final_contour, rect_center_x_px, y1_top_edge, y2_bottom_edge);
        }

        private void drawDetections(Mat displayOutput, List<DetectedCube> cubes, double drawScaleToDisplay,
                                    MatOfPoint targetZoneContourProcessed, Integer targetZoneCenterXAtProcessedScale,
                                    Integer targetRectY1Processed, Integer targetRectY2Processed,
                                    double originalPixelsPerCm, double targetRectY2OriginalScale) {

            if (cubes.isEmpty()) {
                latestBestCubeColor = "None";
                latestBestCubeAngle = 0.0;
                latestBestCubeDistance = Double.POSITIVE_INFINITY;
                latestBestCubeLineAngle = 0.0;
                latestNumberOfCubesDetected = 0;
                return;
            }


            double circleRadiusPxAtDisplayScale = (TARGET_RECT_WIDTH_CM / 2.0) * originalPixelsPerCm * drawScaleToDisplay;

            ArrayList<Map<String, Object>> candidateCubesWithScores = new ArrayList<>();
            double targetBottomYOnDisplay = -1;
            if (targetRectY2Processed != null) {
                targetBottomYOnDisplay = Math.round(targetRectY2Processed * drawScaleToDisplay);
            } else if (targetRectY2OriginalScale != -1 && Math.abs(drawScaleToDisplay - 1.0) < 1e-3) {
                targetBottomYOnDisplay = targetRectY2OriginalScale;
            }

            for (int i = 0; i < cubes.size(); i++) {
                DetectedCube cube = cubes.get(i);
                Point centerInProcessedCoords = new Point(cube.centerXImagePx, cube.centerYImagePx);

                Point drawCenterOnDisplayInt = scalePoint(centerInProcessedCoords, drawScaleToDisplay);

                Point intersectionPoint = null;
                double lineAngleDegFromVertical = 0.0;
                double distToBottomCm = Double.POSITIVE_INFINITY;

                if (targetZoneCenterXAtProcessedScale != null && circleRadiusPxAtDisplayScale > 0) {
                    int lineXAtDisplayScale = (int) Math.round(targetZoneCenterXAtProcessedScale * drawScaleToDisplay);
                    double h = drawCenterOnDisplayInt.x;
                    double k = drawCenterOnDisplayInt.y;
                    double r = circleRadiusPxAtDisplayScale;

                    double distToLine = Math.abs(lineXAtDisplayScale - h);

                    if (distToLine <= r + 1) {
                        double sqrtValSquared = r * r - distToLine * distToLine;
                        double sqrtVal = Math.sqrt(Math.max(0, sqrtValSquared));
                        double y1Intersect = k - sqrtVal;
                        double y2Intersect = k + sqrtVal;

                        double intersectionY = Math.max(y1Intersect, y2Intersect);
                        intersectionPoint = new Point(lineXAtDisplayScale, Math.round(intersectionY));

                        double dx = intersectionPoint.x - drawCenterOnDisplayInt.x;
                        double dy = intersectionPoint.y - drawCenterOnDisplayInt.y;

                        if (!(Math.abs(dx) < 1e-3 && Math.abs(dy) < 1e-3)) {
                            // atan2(dx, dy) 是计算与Y轴（垂直线）的夹角
                            // 结果是：右侧为负, 左侧为正。
                            lineAngleDegFromVertical = Math.toDegrees(Math.atan2(dx, dy));
                        } else {
                            lineAngleDegFromVertical = 0.0;
                        }

                        if (targetBottomYOnDisplay != -1 && originalPixelsPerCm > 0) {
                            double intersectYOrigScale = intersectionPoint.y / drawScaleToDisplay;
                            if (DOWNSCALE_FACTOR > 0 && DOWNSCALE_FACTOR < 1.0) {
                                intersectYOrigScale = intersectYOrigScale / DOWNSCALE_FACTOR;
                            }

                            if (targetRectY2OriginalScale != -1) {
                                double distPxOriginalImg = targetRectY2OriginalScale - intersectYOrigScale;
                                distToBottomCm = distPxOriginalImg / originalPixelsPerCm;
                            } else {
                                distToBottomCm = Double.POSITIVE_INFINITY;
                            }
                        }
                    }
                }

                boolean validCandidate = true;
                if (targetZoneContourProcessed != null && !targetZoneContourProcessed.empty()) {
                    targetZoneContourProcessed.convertTo(tempContour2f, CvType.CV_32F);
                    if (Imgproc.pointPolygonTest(tempContour2f, centerInProcessedCoords, false) < 0) {
                        validCandidate = false;
                    }
                }

                if (validCandidate && targetRectY1Processed != null && targetRectY2Processed != null) {
                    if (intersectionPoint == null) {
                        validCandidate = false;
                    } else {
                        int scaledTargetY1Display = (int) Math.round(targetRectY1Processed * drawScaleToDisplay);
                        int scaledTargetY2Display = (int) Math.round(targetRectY2Processed * drawScaleToDisplay);
                        if (!(scaledTargetY1Display <= intersectionPoint.y && intersectionPoint.y <= scaledTargetY2Display)) {
                            validCandidate = false;
                        }
                    }
                }

                if (validCandidate) {
                    double objAForScore = Math.abs(cube.angleDeg - 90.0);
                    double lineAngleAbsForScore = Math.abs(lineAngleDegFromVertical);
                    double secondaryScore = objAForScore + lineAngleAbsForScore;

                    Map<String, Object> candidateData = new HashMap<>();
                    candidateData.put("cube_index", i);
                    candidateData.put("primary_score", distToBottomCm);
                    candidateData.put("secondary_score", secondaryScore);
                    candidateData.put("line_angle_deg_display", lineAngleDegFromVertical);
                    candidateData.put("draw_center", drawCenterOnDisplayInt);
                    candidateData.put("intersection_point", intersectionPoint);
                    candidateData.put("dist_cm_display", distToBottomCm);
                    candidateCubesWithScores.add(candidateData);
                }
            }

            int bestCubeOriginalIndex = -1;
            if (!candidateCubesWithScores.isEmpty()) {
                Collections.sort(candidateCubesWithScores, (a, b) -> {
                    double primaryA = (Double) a.get("primary_score");
                    double primaryB = (Double) b.get("primary_score");
                    if (primaryA != primaryB) {
                        return Double.compare(primaryA, primaryB);
                    }
                    double secondaryA = (Double) a.get("secondary_score");
                    double secondaryB = (Double) b.get("secondary_score");
                    return Double.compare(secondaryA, secondaryB);
                });

                if ((Double) candidateCubesWithScores.get(0).get("primary_score") != Double.POSITIVE_INFINITY) {
                    bestCubeOriginalIndex = (Integer) candidateCubesWithScores.get(0).get("cube_index");
                }
            }

            if (bestCubeOriginalIndex != -1) {
                DetectedCube bestCube = cubes.get(bestCubeOriginalIndex);
                Map<String, Object> bestCandidateData = null;
                for (Map<String, Object> data : candidateCubesWithScores) {
                    if ((Integer) data.get("cube_index") == bestCubeOriginalIndex) {
                        bestCandidateData = data;
                        break;
                    }
                }

                if (bestCandidateData != null) {
                    latestBestCubeColor = bestCube.color;
                    latestBestCubeAngle = bestCube.angleDeg;
                    latestBestCubeDistance = (Double) bestCandidateData.get("dist_cm_display");
                    latestBestCubeLineAngle = (Double) bestCandidateData.get("line_angle_deg_display");
                    latestNumberOfCubesDetected = cubes.size();
                }
            } else {
                latestBestCubeColor = "None";
                latestBestCubeAngle = 0.0;
                latestBestCubeDistance = Double.POSITIVE_INFINITY;
                latestBestCubeLineAngle = 0.0;
                latestNumberOfCubesDetected = cubes.size(); // [修正一个小bug] 即使没有best cube，也应该报告检测到的总数
            }


            // [修改] 仅当调试视图开启时，才执行下面的所有绘图操作
            if (!WebcamExample.ENABLE_DEBUG_VIEW) {
                return; // 如果关闭调试，则直接返回，不绘制任何东西
            }

            for (Map<String, Object> candidateData : candidateCubesWithScores) {
                int currentCubeOriginalIndex = (Integer) candidateData.get("cube_index");
                DetectedCube cubeToDraw = cubes.get(currentCubeOriginalIndex);

                Scalar boxColorToUse;
                if (currentCubeOriginalIndex == bestCubeOriginalIndex) {
                    boxColorToUse = BOX_COLOR_SELECTED_BEST;
                } else {
                    boxColorToUse = BOX_COLOR_DEFAULT;
                }

                Point drawCenterOnDisplayInt = (Point) candidateData.get("draw_center");
                Point intersectionPointToDraw = (Point) candidateData.get("intersection_point");
                double lineAngleToDisplay = (Double) candidateData.get("line_angle_deg_display");
                double distCmToDisplay = (Double) candidateData.get("dist_cm_display");

                tempScaledBoxPointsForDisplay.fromList(Arrays.asList(scaleRectPoints(cubeToDraw.boundingBoxPoints, drawScaleToDisplay)));
                Imgproc.drawContours(displayOutput, Arrays.asList(tempScaledBoxPointsForDisplay), 0, boxColorToUse, 2);

                Imgproc.circle(displayOutput, drawCenterOnDisplayInt, 4, BLOCK_CENTER_CIRCLE_COLOR, -1);

                double min_x_on_display = Double.POSITIVE_INFINITY;
                double min_y_on_display = Double.POSITIVE_INFINITY;
                double max_y_on_display = Double.NEGATIVE_INFINITY;
                for (Point p : scaleRectPoints(cubeToDraw.boundingBoxPoints, drawScaleToDisplay)) {
                    min_x_on_display = Math.min(min_x_on_display, p.x);
                    min_y_on_display = Math.min(min_y_on_display, p.y);
                    max_y_on_display = Math.max(max_y_on_display, p.y);
                }

                int fontHeightApprox = 15;
                int textYColor = (int) min_y_on_display - 50;
                int textYObjA = (int) min_y_on_display - 35;
                int textYScore = (int) min_y_on_display - 20;
                int textYDist = (int) min_y_on_display - 5;

                if (textYDist < fontHeightApprox) {
                    textYColor = (int) max_y_on_display + fontHeightApprox;
                    textYObjA = textYColor + fontHeightApprox;
                    textYScore = textYObjA + fontHeightApprox;
                    textYDist = textYScore + fontHeightApprox;
                }

                Imgproc.putText(displayOutput, cubeToDraw.color, new Point(min_x_on_display, textYColor), Imgproc.FONT_HERSHEY_SIMPLEX, 0.5,
                        boxColorToUse, 1);
                Imgproc.putText(displayOutput, String.format(Locale.US, "ObjA:%.1f", cubeToDraw.angleDeg), new Point(min_x_on_display, textYObjA),
                        Imgproc.FONT_HERSHEY_SIMPLEX, 0.4, boxColorToUse, 1);
                Imgproc.putText(displayOutput, String.format(Locale.US, "S2:%.1f", (Double) candidateData.get("secondary_score")), new Point(min_x_on_display, textYScore),
                        Imgproc.FONT_HERSHEY_SIMPLEX, 0.4, boxColorToUse, 1);
                if (distCmToDisplay != Double.POSITIVE_INFINITY) {
                    Imgproc.putText(displayOutput, String.format(Locale.US, "Dist:%.1fcm", distCmToDisplay), new Point(min_x_on_display, textYDist),
                            Imgproc.FONT_HERSHEY_SIMPLEX, 0.4, DISTANCE_TEXT_COLOR, 1);
                }

                if (circleRadiusPxAtDisplayScale > 0) {
                    Imgproc.circle(displayOutput, drawCenterOnDisplayInt, (int) Math.round(circleRadiusPxAtDisplayScale),
                            BLOCK_CENTER_CIRCLE_COLOR, 1);
                }

                if (intersectionPointToDraw != null) {
                    Imgproc.circle(displayOutput, intersectionPointToDraw, 5, INTERSECTION_POINT_COLOR, -1);
                    Imgproc.line(displayOutput, drawCenterOnDisplayInt, intersectionPointToDraw,
                            CENTER_TO_INTERSECTION_LINE_COLOR, 1);

                    String angleText = String.format(Locale.US, "LA:%.1f", lineAngleToDisplay);
                    int midXLine = (int) ((drawCenterOnDisplayInt.x + intersectionPointToDraw.x) / 2);
                    int midYLine = (int) ((drawCenterOnDisplayInt.y + intersectionPointToDraw.y) / 2);

                    int textOffsetX = 7;
                    double dxLine = intersectionPointToDraw.x - drawCenterOnDisplayInt.x;
                    if (dxLine < 0) {
                        int[] textSizes = new int[1];
                        Size textSize = Imgproc.getTextSize(angleText, Imgproc.FONT_HERSHEY_SIMPLEX, 0.4, 1, textSizes);
                        textOffsetX = -7 - (int) textSize.width;
                    }
                    Imgproc.putText(displayOutput, angleText, new Point(midXLine + textOffsetX, midYLine - 5),
                            Imgproc.FONT_HERSHEY_SIMPLEX, 0.4, ANGLE_TEXT_COLOR, 1, Imgproc.LINE_AA);
                }
            }
        }

        @Override
        public void onViewportTapped() {
            viewportPaused = !viewportPaused;

            if (viewportPaused) {
                webcam.pauseViewport();
            } else {
                webcam.resumeViewport();
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
            if (sureBg != null) sureBg.release();
            if (distTransform != null) distTransform.release();
            if (sureFg != null) sureFg.release();
            if (unknown != null) unknown.release();
            if (markers != null) markers.release();
            if (imgForWatershed != null) imgForWatershed.release();
            if (kernel3x3 != null) kernel3x3.release();
            if (targetZoneContourAtOriginalScale != null) targetZoneContourAtOriginalScale.release();
            if (tempContour2f != null) tempContour2f.release();
            if (tempScaledBoxPointsForDisplay != null) tempScaledBoxPointsForDisplay.release();
            if (targetZoneContourAtProcessedScaleHolder != null) targetZoneContourAtProcessedScaleHolder.release();
        }
    }
}