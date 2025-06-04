/**
 * 我会努力让视觉程序变的优雅的
 * @author BlueDarkUP
 * @version 2025/6/4
 *
 */
package org.firstinspires.ftc.teamcode.vision;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

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

@TeleOp(name="WebcamExample", group="Vision")
public class WebcamExample extends LinearOpMode {


    public static final String WEBCAM_NAME_STR = "Webcam"; //你怎么知道给这个摄像头分配的id是01.00.00
    public static final int CAMERA_WIDTH = 1280; // 图像宽度（像素）
    public static final int CAMERA_HEIGHT = 720; // 图像高度（像素）

    public static final double REAL_WORLD_VIEW_WIDTH_CM = 92.0; // 摄像头视野在实际世界中的宽度（厘米）
    public static double PIXELS_PER_CM;



    public static final double TARGET_RECT_WIDTH_CM = 31.5; // 双倍送给小臂
    public static final double TARGET_RECT_HEIGHT_CM = 24.0; // 滑轨行程
    public static final double TARGET_RECT_OFFSET_X_CM = 0.0; // 区域X偏移
    public static final double TARGET_RECT_OFFSET_Y_CM = 5.0; // 区域Y偏移


    public static final Scalar TARGET_RECT_COLOR = new Scalar(255, 0, 255);
    public static final int TARGET_RECT_THICKNESS = 2;
    public static final int ARC_SAMPLING_POINTS = 20;


    public static final double DOWNSCALE_FACTOR = 1.0; // 图像处理缩放因子（鬼知道为什么改了机器就崩溃）
    public static final double MIN_SIZE_PIXELS = 200.0; // 最小像素面积
    public static final double MAX_SIZE_PIXELS = 25000.0; // 最大像素面积

    public static final double TARGET_OBJECT_ASPECT_RATIO = 7.0 / 3.0; // 目标物体的期望长宽比
    public static final double ASPECT_RATIO_TOLERANCE_PERCENT = 0.25; // 长宽比容忍度（百分比）


    public static final Map<String, Scalar[][]> COLOR_HSV_RANGES = new HashMap<>(); // 存储各种颜色的HSV范围
    static {
        COLOR_HSV_RANGES.put("YELLOW", new Scalar[][]{ // 会有别的的
                {new Scalar(15, 80, 170), new Scalar(45, 255, 255)}
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
        // 计算尺寸转换比例系数
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
                webcam.startStreaming(CAMERA_WIDTH, CAMERA_HEIGHT, OpenCvCameraRotation.UPRIGHT); // 开始传输图像流
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
            // 官方给的升血压实时统计信息
            telemetry.addData("Frame Count", webcam.getFrameCount());
            telemetry.addData("FPS", String.format(Locale.US, "%.2f", webcam.getFps()));
            telemetry.addData("Total frame time ms", webcam.getTotalFrameTimeMs());
            telemetry.addData("Pipeline time ms", webcam.getPipelineTimeMs());
            telemetry.addData("Overhead time ms", webcam.getOverheadTimeMs());
            telemetry.addData("Theoretical max FPS", webcam.getCurrentPipelineMaxFps());
            // 加点有用的（用于结构控制）
            telemetry.addLine("--- Vision Results ---");
            telemetry.addData("Cubes Detected", pipeline.latestNumberOfCubesDetected); // 检测到的方块数量
            telemetry.addData("Best Cube Color", pipeline.latestBestCubeColor); // 最佳方块颜色
            telemetry.addData("Best Cube Angle (Obj)", String.format(Locale.US, "%.1f deg", pipeline.latestBestCubeAngle)); // 最佳方块的物体角度
            telemetry.addData("Best Cube Dist (Target)", String.format(Locale.US, "%.1f cm", pipeline.latestBestCubeDistance)); // 最佳方块到目标区域的距离
            telemetry.addData("Best Cube Line Angle", String.format(Locale.US, "%.1f deg", pipeline.latestBestCubeLineAngle)); // 最佳方块连线的角度
            telemetry.update();

            if (gamepad1.a) { // 如果手柄1的A键被按下（其实没有蛋用）
                webcam.stopStreaming(); // 停止摄像头流
                telemetry.addLine("Streaming stopped.");
                telemetry.update();
            }

            sleep(100);
        }

        // 手动释放Mat对象还是更权威
        if (pipeline != null) {
            pipeline.releaseMats();
        }
    }

    // 检测到的方块数据结构
    static class DetectedCube {
        public String color; // 方块颜色
        public int centerXImagePx; // 图像中的中心X坐标（像素）
        public int centerYImagePx; // 图像中的中心Y坐标（像素）
        public Point[] boundingBoxPoints; // 边界框的四个顶点
        public double scaleFactor; // 缩放因子
        public double angleDeg; // 物体的旋转角度（度）

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
        public String toString() { // 转换为字符串
            return String.format(Locale.US, "Cube[Color:%s, Center:(%d,%d), Angle:%.1fdeg @%.2fx]",
                    color, centerXImagePx, centerYImagePx, angleDeg, scaleFactor);
        }
    }

    // 目标区域信息数据结构
    static class TargetZoneInfo {
        public MatOfPoint targetContour; // 目标区域的轮廓
        public Integer centerX; // 目标区域的中心X坐标
        public Integer topY; // 目标区域顶部Y坐标
        public Integer bottomY; // 目标区域底部Y坐标

        public TargetZoneInfo(MatOfPoint targetContour, Integer centerX, Integer topY, Integer bottomY) {
            this.targetContour = targetContour;
            this.centerX = centerX;
            this.topY = topY;
            this.bottomY = bottomY;
        }
    }

    class SamplePipeline extends OpenCvPipeline {
        // 定义各种Mat对象用于中间步骤，避免重复分配内存
        private Mat bgr; // BGR颜色空间的图像
        private Mat hsv; // HSV颜色空间的图像
        private Mat processedFrameForDetection; // 用于检测的缩放/原始帧
        private Mat masterMask; // 主掩码，合并所有颜色掩码
        private Mat maskCombined; // 临时掩码，用于inRange操作
        private Mat colorMask; // 单个颜色范围的掩码
        private Mat medianBlurred; // 中值滤波后的掩码
        private Mat opened; // 开运算后的掩码
        private Mat sureBg; // 确定背景区域
        private Mat distTransform; // 距离变换结果
        private Mat sureFg; // 确定前景区域
        private Mat unknown; // 未知区域
        private Mat markers; // 分水岭算法的标记图像
        private Mat imgForWatershed; // 用于分水岭算法的图像

        private Mat kernel3x3; // 3x3的矩形核，用于形态学操作

        private MatOfPoint2f tempContour2f; // 临时MatOfPoint2f对象，用于轮廓转换
        private MatOfPoint tempScaledBoxPointsForDisplay; // 临时MatOfPoint对象，用于缩放后的边界框显示

        // 存储原始尺寸下目标区域的轮廓和中心点，用于参考
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

        // 管道构造函数
        public SamplePipeline() {
            // 在构造函数中预计算目标区域的轮廓和中心点，避免每帧重复计算
            Mat dummyFrame = new Mat(CAMERA_HEIGHT, CAMERA_WIDTH, CvType.CV_8UC3); // 创建一个虚拟帧
            TargetZoneInfo info = drawTargetZoneCm(dummyFrame, PIXELS_PER_CM, // 在虚拟帧上绘制目标区域
                    CAMERA_WIDTH, CAMERA_HEIGHT,
                    TARGET_RECT_WIDTH_CM, TARGET_RECT_HEIGHT_CM,
                    TARGET_RECT_OFFSET_X_CM, TARGET_RECT_OFFSET_Y_CM,
                    TARGET_RECT_COLOR, TARGET_RECT_THICKNESS, ARC_SAMPLING_POINTS);
            targetZoneContourAtOriginalScale = info.targetContour; // 获取原始尺寸的目标区域轮廓
            targetZoneCenterXAtOriginalScale = info.centerX; // 获取原始尺寸的目标区域中心X
            targetRectY1AtOriginalScale = info.topY; // 获取原始尺寸的目标区域顶部Y
            targetRectY2AtOriginalScale = info.bottomY; // 获取原始尺寸的目标区域底部Y
            dummyFrame.release(); // 释放虚拟帧的内存
        }

        @Override
        public void init(Mat firstFrame) { // 管道初始化方法
            int originalHeight = firstFrame.rows();
            int originalWidth = firstFrame.cols();

            // 为所有Mat对象分配内存
            bgr = new Mat(originalHeight, originalWidth, CvType.CV_8UC3);
            hsv = new Mat(originalHeight, originalWidth, CvType.CV_8UC3);
            masterMask = new Mat(originalHeight, originalWidth, CvType.CV_8U);
            maskCombined = new Mat(originalHeight, originalWidth, CvType.CV_8U);
            colorMask = new Mat(originalHeight, originalWidth, CvType.CV_8U);
            medianBlurred = new Mat(originalHeight, originalWidth, CvType.CV_8U);
            opened = new Mat(originalHeight, originalWidth, CvType.CV_8U);
            sureBg = new Mat(originalHeight, originalWidth, CvType.CV_8U);
            distTransform = new Mat(originalHeight, originalWidth, CvType.CV_32F);
            sureFg = new Mat(originalHeight, originalWidth, CvType.CV_8U);
            unknown = new Mat(originalHeight, originalWidth, CvType.CV_8U);
            markers = new Mat(originalHeight, originalWidth, CvType.CV_32S);
            imgForWatershed = new Mat(originalHeight, originalWidth, CvType.CV_8UC3);

            kernel3x3 = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(3, 3)); // 创建3x3矩形核

            // 初始化processedFrameForDetection
            if (DOWNSCALE_FACTOR < 1.0 && DOWNSCALE_FACTOR > 0) {
                int scaledWidth = (int) (originalWidth * DOWNSCALE_FACTOR);
                int scaledHeight = (int) (originalHeight * DOWNSCALE_FACTOR);
                processedFrameForDetection = new Mat(scaledHeight, scaledWidth, CvType.CV_8UC3);
                targetZoneContourAtProcessedScaleHolder = new MatOfPoint();
            } else {
                processedFrameForDetection = new Mat(originalHeight, originalWidth, CvType.CV_8UC3);
            }

            tempContour2f = new MatOfPoint2f();
            tempScaledBoxPointsForDisplay = new MatOfPoint();
        }

        @Override
        public Mat processFrame(Mat inputRGBA) { // 核心图像处理方法，每帧图像都会调用
            Imgproc.cvtColor(inputRGBA, bgr, Imgproc.COLOR_RGBA2BGR); // 将输入帧从RGBA转换为BGR

            double drawScale = 1.0; // 绘制时的缩放比例
            double processingScale = 1.0; // 处理时的缩放比例

            MatOfPoint targetZoneContourAtProcessedScale = null;
            Integer targetZoneCenterXAtProcessedScale = null;
            Integer targetRectY1AtProcessedScale = null;
            Integer targetRectY2AtProcessedScale = null;

            // 根据DOWNSCALE_FACTOR进行图像缩放处理
            if (DOWNSCALE_FACTOR < 1.0 && DOWNSCALE_FACTOR > 0) {
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

            Imgproc.cvtColor(processedFrameForDetection, hsv, Imgproc.COLOR_BGR2HSV); // 将处理帧转换为HSV颜色空间

            int height = processedFrameForDetection.rows();
            int width = processedFrameForDetection.cols();

            masterMask.setTo(new Scalar(0)); // 清空主掩码

            ArrayList<DetectedCube> allDetectedCubes = new ArrayList<>(); // 存储所有检测到的方块

            double scaledMinAreaPixels = MIN_SIZE_PIXELS * (processingScale * processingScale);
            double scaledMaxAreaPixels = MAX_SIZE_PIXELS * (processingScale * processingScale);

            // 遍历所有定义的颜色范围进行检测
            for (Map.Entry<String, Scalar[][]> entry : COLOR_HSV_RANGES.entrySet()) {
                String colorName = entry.getKey(); // 当前颜色名称
                Scalar[][] ranges = entry.getValue();

                colorMask.setTo(new Scalar(0)); // 清空当前颜色掩码

                // 创建二值掩码
                for (Scalar[] range : ranges) {
                    Core.inRange(hsv, range[0], range[1], maskCombined); // 创建单个范围的掩码
                    Core.bitwise_or(colorMask, maskCombined, colorMask); // 将其与主颜色掩码合并
                }

                Core.bitwise_or(masterMask, colorMask, masterMask); // 与主掩码合并

                Imgproc.medianBlur(colorMask, medianBlurred, 5); // 非常好的中值滤波，使我的分水岭算法旋转，爱来自北部的京都

                // 形态学开运算：先腐蚀后膨胀
                Imgproc.morphologyEx(medianBlurred, opened, Imgproc.MORPH_OPEN, kernel3x3, new Point(-1, -1), 4);

                // 膨胀操作：确定背景
                Imgproc.dilate(opened, sureBg, kernel3x3, new Point(-1, -1), 3);

                // 距离变换：计算每个像素到最近背景像素的距离
                Imgproc.distanceTransform(opened, distTransform, Imgproc.DIST_L2, 5);
                Core.MinMaxLocResult mmr = Core.minMaxLoc(distTransform); // 找到距离变换结果的最大值
                double distThreshold = 0.57 * mmr.maxVal; // 设置前景阈值
                // 将距离变换结果转换为二值图像，得到确定前景区域
                Imgproc.threshold(distTransform, sureFg, distThreshold, 255, Imgproc.THRESH_BINARY);
                sureFg.convertTo(sureFg, CvType.CV_8U);

                Core.subtract(sureBg, sureFg, unknown); // 确定背景减去确定前景，得到未知区域

                // 对确定前景区域进行连通组件标记，作为分水岭算法的初始标记
                Imgproc.connectedComponents(sureFg, markers);

                markers.setTo(new Scalar(0), unknown); // 将未知区域标记为0（未分类）

                // 对原始图像进行高斯模糊，为分水岭算法做准备
                Imgproc.GaussianBlur(processedFrameForDetection, imgForWatershed, new Size(5, 5), 0);

                Imgproc.watershed(imgForWatershed, markers); // 执行分水岭算法，分离粘连的物体

                Core.MinMaxLocResult markerMax = Core.minMaxLoc(markers);
                int maxMarkerId = (int) markerMax.maxVal;

                List<MatOfPoint> contours = new ArrayList<>();
                Mat hierarchy = new Mat();

                // 遍历分水岭算法得到的每个标记区域
                for (int markerId = 1; markerId <= maxMarkerId; markerId++) { // 0是背景，-1是边界
                    colorMask.setTo(new Scalar(0));
                    Core.compare(markers, new Scalar(markerId), colorMask, Core.CMP_EQ); // 提取当前标记的区域

                    contours.clear();
                    // 在当前标记区域中寻找轮廓
                    Imgproc.findContours(colorMask, contours, hierarchy, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);

                    for (MatOfPoint contour : contours) { // 遍历每个找到的轮廓
                        double area = Imgproc.contourArea(contour); // 计算轮廓面积
                        // 过滤面积不符合要求的轮廓
                        if (!(scaledMinAreaPixels <= area && area <= scaledMaxAreaPixels)) {
                            contour.release();
                            continue;
                        }

                        // 过滤点数过少的轮廓（小于5个点的轮廓不构成有效形状）
                        if (contour.rows() < 5) {
                            contour.release();
                            continue;
                        }

                        contour.convertTo(tempContour2f, CvType.CV_32F); // 将轮廓转换为MatOfPoint2f
                        RotatedRect minAreaRect = Imgproc.minAreaRect(tempContour2f); // 获取最小外接旋转矩形
                        Point rectCenter = minAreaRect.center; // 矩形中心点
                        Size rectSize = minAreaRect.size; // 矩形尺寸
                        double cvAngle = minAreaRect.angle; // OpenCV计算的旋转角度

                        double rectWidthRot = rectSize.width;
                        double rectHeightRot = rectSize.height;

                        // 根据宽度和高度调整物体实际的旋转角度
                        double objectOrientationAngleDeg;
                        if (rectWidthRot < rectHeightRot) {
                            objectOrientationAngleDeg = cvAngle + 90.0;
                        } else {
                            objectOrientationAngleDeg = cvAngle;
                        }

                        double side1 = rectWidthRot;
                        double side2 = rectHeightRot;
                        if (side1 < 1e-3 || side2 < 1e-3) { // 避免除以零
                            contour.release();
                            continue;
                        }
                        double longSide = Math.max(side1, side2); // 长边
                        double shortSide = Math.min(side1, side2); // 短边
                        double detectedAspectRatio = longSide / shortSide; // 检测到的长宽比

                        // 根据容忍度计算长宽比的上下限
                        double lowerBoundAR = TARGET_OBJECT_ASPECT_RATIO * (1.0 - ASPECT_RATIO_TOLERANCE_PERCENT);
                        double upperBoundAR = TARGET_OBJECT_ASPECT_RATIO * (1.0 + ASPECT_RATIO_TOLERANCE_PERCENT);

                        // 过滤长宽比不符合要求的轮廓
                        if (!(lowerBoundAR <= detectedAspectRatio && detectedAspectRatio <= upperBoundAR)) {
                            contour.release();
                            continue;
                        }

                        Point[] boxPoints = new Point[4];
                        minAreaRect.points(boxPoints); // 获取旋转矩形的四个顶点

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
            drawDetections(bgr, allDetectedCubes, drawScale,
                    targetZoneContourAtProcessedScale, targetZoneCenterXAtProcessedScale,
                    targetRectY1AtProcessedScale, targetRectY2AtProcessedScale,
                    PIXELS_PER_CM, targetRectY2AtOriginalScale);

            TargetZoneInfo finalDisplayZoneInfo = drawTargetZoneCm(bgr, PIXELS_PER_CM,
                    CAMERA_WIDTH, CAMERA_HEIGHT,
                    TARGET_RECT_WIDTH_CM, TARGET_RECT_HEIGHT_CM,
                    TARGET_RECT_OFFSET_X_CM, TARGET_RECT_OFFSET_Y_CM,
                    TARGET_RECT_COLOR, TARGET_RECT_THICKNESS, ARC_SAMPLING_POINTS);
            if (finalDisplayZoneInfo.centerX != null) {
                Imgproc.line(bgr, new Point(finalDisplayZoneInfo.centerX, 0),
                        new Point(finalDisplayZoneInfo.centerX, CAMERA_HEIGHT), TARGET_RECT_COLOR, 1);
                if (finalDisplayZoneInfo.targetContour != null) {
                    finalDisplayZoneInfo.targetContour.release();
                }
            }
            int grid_size_in_pixels = (int) Math.round(5 * PIXELS_PER_CM);
            grid_size_in_pixels = Math.max(1, grid_size_in_pixels);
            drawGridOverlay(bgr, grid_size_in_pixels, GRID_COLOR, 1);

            Imgproc.cvtColor(bgr, inputRGBA, Imgproc.COLOR_BGR2RGBA);

            return inputRGBA;
        }

        private Point scalePoint(Point point, double scaleToApply) {
            if (Math.abs(scaleToApply - 1.0) < 1e-3) {
                return point;
            }
            return new Point(Math.round(point.x * scaleToApply), Math.round(point.y * scaleToApply));
        }
        private Point[] scaleRectPoints(Point[] boxPoints, double scaleToApply) {
            if (Math.abs(scaleToApply - 1.0) < 1e-3) {
                return boxPoints;
            }
            Point[] scaledPoints = new Point[boxPoints.length];
            for (int i = 0; i < boxPoints.length; i++) {
                scaledPoints[i] = new Point(Math.round(boxPoints[i].x * scaleToApply), Math.round(boxPoints[i].y * scaleToApply));
            }
            return scaledPoints;
        }
        private void drawGridOverlay(Mat frame, int grid_size_px, Scalar color, int thickness) {
            int h = frame.rows();
            int w = frame.cols();
            if (grid_size_px < 1) grid_size_px = 1;
            for (int x = 0; x < w; x += grid_size_px) { // 绘制垂直线
                Imgproc.line(frame, new Point(x, 0), new Point(x, h), color, thickness);
            }
            for (int y = 0; y < h; y += grid_size_px) { // 绘制水平线
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
                Imgproc.ellipse(frame, new Point(rect_center_x_px, arc_b_center_y), new Size(arc_radius_px, arc_radius_px), 0, 180, 360, color, thickness);
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
            Imgproc.line(frame, new Point(x1, y1_top_edge), new Point(x1, y2_bottom_edge), color, thickness);
            if (!contour_points.isEmpty() && (contour_points.get(contour_points.size() - 1).x != x1 || contour_points.get(contour_points.size() - 1).y != y2_bottom_edge)) {
                contour_points.add(new Point(x1, y2_bottom_edge));
            }
            contour_points.add(new Point(x1, y1_top_edge));
            int arc_a_center_y = y1_top_edge;
            if (arc_radius_px > 0) {
                Imgproc.ellipse(frame, new Point(rect_center_x_px, arc_a_center_y), new Size(arc_radius_px, arc_radius_px), 0, 180, 360, color, thickness);
                for (int i = 0; i <= arc_points_sampling; i++) {
                    double angle_rad = Math.toRadians(180 + (i * 180.0 / arc_points_sampling));
                    int pt_x = rect_center_x_px + (int) Math.round(arc_radius_px * Math.cos(angle_rad));
                    int pt_y = arc_a_center_y + (int) Math.round(arc_radius_px * Math.sin(angle_rad));
                    contour_points.add(new Point(pt_x, pt_y));
                }
            } else {
                contour_points.add(new Point(x2, y1_top_edge));
            }
            Imgproc.line(frame, new Point(x2, y1_top_edge), new Point(x2, y2_bottom_edge), color, thickness);
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

            // 遍历所有检测到的方块，计算其与目标区域的关系和得分
            for (int i = 0; i < cubes.size(); i++) {
                DetectedCube cube = cubes.get(i);
                Point centerInProcessedCoords = new Point(cube.centerXImagePx, cube.centerYImagePx);

                Point drawCenterOnDisplayInt = scalePoint(centerInProcessedCoords, drawScaleToDisplay); // 缩放中心点到显示尺寸

                Point intersectionPoint = null; // 交点
                double lineAngleDegFromVertical = 0.0; // 连线与垂直线的角度
                double distToBottomCm = Double.POSITIVE_INFINITY; // 到目标区域底部的距离（厘米）

                // 如果目标区域中心和圆弧半径有效，则计算交点和距离
                if (targetZoneCenterXAtProcessedScale != null && circleRadiusPxAtDisplayScale > 0) {
                    int lineXAtDisplayScale = (int) Math.round(targetZoneCenterXAtProcessedScale * drawScaleToDisplay);
                    double h = drawCenterOnDisplayInt.x;
                    double k = drawCenterOnDisplayInt.y;
                    double r = circleRadiusPxAtDisplayScale;

                    double distToLine = Math.abs(lineXAtDisplayScale - h);

                    if (distToLine <= r + 1) { // 如果方块的圆与垂直线相交
                        double sqrtValSquared = r * r - distToLine * distToLine;
                        double sqrtVal = Math.sqrt(Math.max(0, sqrtValSquared)); // 避免负数开方
                        double y1Intersect = k - sqrtVal; // 交点Y1
                        double y2Intersect = k + sqrtVal; // 交点Y2

                        double intersectionY = Math.max(y1Intersect, y2Intersect); // 选择较低的交点
                        intersectionPoint = new Point(lineXAtDisplayScale, Math.round(intersectionY)); // 确定交点

                        double dx = intersectionPoint.x - drawCenterOnDisplayInt.x;
                        double dy = intersectionPoint.y - drawCenterOnDisplayInt.y;

                        if (!(Math.abs(dx) < 1e-3 && Math.abs(dy) < 1e-3)) {
                            lineAngleDegFromVertical = Math.toDegrees(Math.atan2(dx, dy));
                        } else {
                            lineAngleDegFromVertical = 0.0;
                        }

                        if (targetBottomYOnDisplay != -1 && originalPixelsPerCm > 0) {
                            double intersectYOrigScale = intersectionPoint.y / drawScaleToDisplay;
                            if (DOWNSCALE_FACTOR < 1.0 && DOWNSCALE_FACTOR > 0) {
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

                boolean validCandidate = true; // 标记是否为有效候选方块
                if (targetZoneContourProcessed != null && !targetZoneContourProcessed.empty()) {
                    targetZoneContourProcessed.convertTo(tempContour2f, CvType.CV_32F);
                    if (Imgproc.pointPolygonTest(tempContour2f, centerInProcessedCoords, false) < 0) {
                        validCandidate = false;
                    }
                }

                // 检查交点是否在目标区域的Y范围内
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

                // 如果是有效候选方块，计算其得分并加入列表
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
                // 对候选方块进行排序：首先按主要得分（距离）升序，然后按次要得分（角度）升序
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

                // 选择得分最高的方块作为最佳方块（距离不是无穷大）
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
                latestNumberOfCubesDetected = 0;
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

        // 释放所有OpenCV Mat对象，防止内存泄漏。在OpMode停止时调用
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