// Filename: SamplePipeline.java
package org.firstinspires.ftc.teamcode.vision;

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
import java.util.Map;

/**
 * OpenCvPipeline 的具体实现，负责处理每一帧图像并识别目标。
 */
public class SamplePipeline extends OpenCvPipeline {

    private final VisionGraspingAPI apiInstance;
    private Mat bgr, hsv, processedFrameForDetection, masterMask, maskCombined, colorMask, medianBlurred, opened;
    private Mat kernel3x3;
    private MatOfPoint2f tempContour2f;
    private MatOfPoint tempScaledBoxPointsForDisplay;
    private MatOfPoint targetZoneContourAtOriginalScale, targetZoneContourAtProcessedScaleHolder;
    private Integer targetZoneCenterXAtOriginalScale, targetRectY1AtOriginalScale, targetRectY2AtOriginalScale;
    private boolean viewportPaused;

    public SamplePipeline(VisionGraspingAPI apiInstance) {
        this.apiInstance = apiInstance;
        // 预计算原始尺寸下的目标区域轮廓
        Mat dummyFrame = new Mat(VisionConstants.CAMERA_HEIGHT, VisionConstants.CAMERA_WIDTH, CvType.CV_8UC3);
        TargetZoneInfo info = drawTargetZoneCm(dummyFrame, VisionConstants.PIXELS_PER_CM, VisionConstants.CAMERA_WIDTH, VisionConstants.CAMERA_HEIGHT, VisionConstants.TARGET_RECT_WIDTH_CM, VisionConstants.TARGET_RECT_HEIGHT_CM, VisionConstants.TARGET_RECT_OFFSET_X_CM, VisionConstants.TARGET_RECT_OFFSET_Y_CM, VisionConstants.TARGET_RECT_COLOR, VisionConstants.TARGET_RECT_THICKNESS, VisionConstants.ARC_SAMPLING_POINTS);
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
        if (viewportPaused) {
            return inputRGBA;
        }

        Imgproc.cvtColor(inputRGBA, bgr, Imgproc.COLOR_RGBA2BGR);
        double drawScale = 1.0, processingScale = 1.0;
        MatOfPoint targetZoneContourAtProcessedScale = null;
        Integer targetZoneCenterXAtProcessedScale = null;
        Integer targetRectY1AtProcessedScale = null;
        Integer targetRectY2AtProcessedScale = null;

        if (VisionConstants.DOWNSCALE_FACTOR > 0 && VisionConstants.DOWNSCALE_FACTOR < 1.0) {
            processingScale = VisionConstants.DOWNSCALE_FACTOR;
            drawScale = 1.0 / VisionConstants.DOWNSCALE_FACTOR;
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
        double scaledMinArea = VisionConstants.MIN_SIZE_PIXELS * (processingScale * processingScale);
        double scaledMaxArea = VisionConstants.MAX_SIZE_PIXELS * (processingScale * processingScale);

        for (Map.Entry<String, Scalar[][]> entry : VisionConstants.COLOR_HSV_RANGES.entrySet()) {
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
                if (ar < VisionConstants.TARGET_OBJECT_ASPECT_RATIO * (1-VisionConstants.ASPECT_RATIO_TOLERANCE_PERCENT) || ar > VisionConstants.TARGET_OBJECT_ASPECT_RATIO * (1+VisionConstants.ASPECT_RATIO_TOLERANCE_PERCENT)) { contour.release(); continue; }
                Point[] boxPoints = new Point[4];
                minAreaRect.points(boxPoints);
                allDetectedCubes.add(new DetectedCube(entry.getKey(), (int)minAreaRect.center.x, (int)minAreaRect.center.y, boxPoints, processingScale, angle));
                contour.release();
            }
        }

        drawDetectionsAndSetResult(allDetectedCubes, processingScale, targetZoneContourAtProcessedScale, targetZoneCenterXAtProcessedScale, targetRectY1AtProcessedScale, targetRectY2AtProcessedScale, VisionConstants.PIXELS_PER_CM, targetRectY2AtOriginalScale);

        if (VisionConstants.ENABLE_DEBUG_VIEW) {
            drawDebugOverlays(bgr);
        }

        Imgproc.cvtColor(bgr, inputRGBA, Imgproc.COLOR_BGR2RGBA);
        return inputRGBA;
    }

    private void drawDetectionsAndSetResult(List<DetectedCube> cubes, double processingScale, MatOfPoint targetZoneContourProcessed, Integer targetZoneCenterXAtProcessedScale, Integer targetRectY1Processed, Integer targetRectY2Processed, double originalPixelsPerCm, double targetRectY2OriginalScale) {
        if (cubes.isEmpty()) {
            apiInstance.updateLatestResult(new VisionTargetResult());
            return;
        }

        ArrayList<CandidateInfo> candidates = new ArrayList<>();
        double circleRadiusP = (VisionConstants.TARGET_RECT_WIDTH_CM / 2.0) * originalPixelsPerCm * processingScale;

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
            candidates.sort((a, b) -> (Math.abs(a.primaryScore - b.primaryScore) > 1e-6) ? Double.compare(a.primaryScore, b.primaryScore) : Double.compare(a.secondaryScore, b.secondaryScore));
            if (candidates.get(0).primaryScore != Double.POSITIVE_INFINITY) {
                DetectedCube bestCube = cubes.get(candidates.get(0).cubeIndex);
                apiInstance.updateLatestResult(new VisionTargetResult(true, bestCube.color, candidates.get(0).distanceCm, bestCube.angleDeg, candidates.get(0).lineAngleDeg));
            } else {
                apiInstance.updateLatestResult(new VisionTargetResult());
            }
        } else {
            apiInstance.updateLatestResult(new VisionTargetResult());
        }
    }

    private void drawDebugOverlays(Mat frame) {
        TargetZoneInfo finalDisplayZoneInfo = drawTargetZoneCm(frame, VisionConstants.PIXELS_PER_CM, VisionConstants.CAMERA_WIDTH, VisionConstants.CAMERA_HEIGHT, VisionConstants.TARGET_RECT_WIDTH_CM, VisionConstants.TARGET_RECT_HEIGHT_CM, VisionConstants.TARGET_RECT_OFFSET_X_CM, VisionConstants.TARGET_RECT_OFFSET_Y_CM, VisionConstants.TARGET_RECT_COLOR, VisionConstants.TARGET_RECT_THICKNESS, VisionConstants.ARC_SAMPLING_POINTS);
        if (finalDisplayZoneInfo.centerX != null) {
            Imgproc.line(frame, new Point(finalDisplayZoneInfo.centerX, 0), new Point(finalDisplayZoneInfo.centerX, VisionConstants.CAMERA_HEIGHT), VisionConstants.TARGET_RECT_COLOR, 1);
            if (finalDisplayZoneInfo.targetContour != null) finalDisplayZoneInfo.targetContour.release();
        }
        drawGridOverlay(frame, Math.max(1, (int)Math.round(5 * VisionConstants.PIXELS_PER_CM)), VisionConstants.GRID_COLOR, 1);
    }

    private void drawGridOverlay(Mat f, int size, Scalar c, int t) {
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
            if (VisionConstants.ENABLE_DEBUG_VIEW) Imgproc.ellipse(frame, new Point(rect_center_x_px, y2_bottom_edge), new Size(arc_radius_px, arc_radius_px), 0, 180, 360, color, thickness);
            for (int i = 0; i <= arc_points_sampling; i++) {
                double angle_rad = Math.toRadians(360 - (i * 180.0 / arc_points_sampling));
                contour_points.add(new Point(rect_center_x_px + (int)Math.round(arc_radius_px * Math.cos(angle_rad)), y2_bottom_edge + (int)Math.round(arc_radius_px * Math.sin(angle_rad))));
            }
        } else {
            contour_points.add(new Point(x2, y2_bottom_edge));
            contour_points.add(new Point(x1, y2_bottom_edge));
        }

        if (VisionConstants.ENABLE_DEBUG_VIEW) Imgproc.line(frame, new Point(x1, y1_top_edge), new Point(x1, y2_bottom_edge), color, thickness);
        if (!contour_points.isEmpty() && (contour_points.get(contour_points.size() - 1).x != x1 || contour_points.get(contour_points.size() - 1).y != y2_bottom_edge)) contour_points.add(new Point(x1, y2_bottom_edge));
        contour_points.add(new Point(x1, y1_top_edge));

        if (arc_radius_px > 0) {
            if (VisionConstants.ENABLE_DEBUG_VIEW) Imgproc.ellipse(frame, new Point(rect_center_x_px, y1_top_edge), new Size(arc_radius_px, arc_radius_px), 0, 180, 360, color, thickness);
            for (int i = 0; i <= arc_points_sampling; i++) {
                double angle_rad = Math.toRadians(180 + (i * 180.0 / arc_points_sampling));
                contour_points.add(new Point(rect_center_x_px + (int)Math.round(arc_radius_px * Math.cos(angle_rad)), y1_top_edge + (int)Math.round(arc_radius_px * Math.sin(angle_rad))));
            }
        } else {
            contour_points.add(new Point(x2, y1_top_edge));
        }

        if (VisionConstants.ENABLE_DEBUG_VIEW) Imgproc.line(frame, new Point(x2, y1_top_edge), new Point(x2, y2_bottom_edge), color, thickness);
        if (!contour_points.isEmpty() && (contour_points.get(contour_points.size() - 1).x != x2 || contour_points.get(contour_points.size() - 1).y != y1_top_edge)) contour_points.add(new Point(x2, y1_top_edge));
        contour_points.add(new Point(x2, y2_bottom_edge));

        MatOfPoint final_contour = new MatOfPoint();
        final_contour.fromList(contour_points);

        return new TargetZoneInfo(final_contour, rect_center_x_px, y1_top_edge, y2_bottom_edge);
    }

    @Override
    public void onViewportTapped() {
        viewportPaused = !viewportPaused;
        apiInstance.toggleViewport(viewportPaused);
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