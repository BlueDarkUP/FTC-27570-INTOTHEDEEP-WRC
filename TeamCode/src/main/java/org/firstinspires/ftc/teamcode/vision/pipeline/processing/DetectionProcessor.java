package org.firstinspires.ftc.teamcode.vision.pipeline.processing;

import org.firstinspires.ftc.teamcode.vision.VisionGraspingAPI;
import org.firstinspires.ftc.teamcode.vision.pipeline.VisionConstants;
import org.firstinspires.ftc.teamcode.vision.pipeline.model.CandidateInfo;
import org.firstinspires.ftc.teamcode.vision.pipeline.model.DetectedCube;
import org.opencv.core.CvType;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Point;
import org.opencv.imgproc.Imgproc;
import java.util.ArrayList;
import java.util.Collections;
import java.util.Comparator;
import java.util.List;

/**
 * 检测后处理器与决策核心
 * 该类是视觉系统的“大脑”，负责实现所有复杂的决策逻辑。它接收一个包含所有被检测物体的列表，
 * 然后根据一系列规则（如位置、距离、排序）来判断哪个是当前应抓取的目标，哪个是下一个目标，
 * 并最终生成一个完整的 VisionTargetResult。
 *
 * 所有关于“抓哪个”、“下一步怎么走”的智能判断逻辑都应在此类中实现。
 * 此类将原始的、分散的检测信息（List<DetectedCube>）处理成一个结构化的、有明确指令的
 *    最终结果（VisionGraspingAPI.VisionTargetResult）。
 * 它完全与OpenCV的图像处理流程解耦，只处理已经检测到的数据。
 * @author BlueDarkUP
 * @version 2025/6 (Refactored)
 * To My Lover - Zyy
 */
public class DetectionProcessor {

    private final MatOfPoint2f tempContour2f = new MatOfPoint2f();

    public void process(VisionGraspingAPI apiInstance, List<DetectedCube> allCubes, double processingScale, MatOfPoint targetZoneContourProcessed, Integer targetZoneCenterXAtProcessedScale, Integer targetRectY1Processed, Integer targetRectY2Processed, double pixelsPerCmForDrawing, double targetRectY2OriginalScale) {
        if (allCubes.isEmpty()) {
            apiInstance.setLatestResult(new VisionGraspingAPI.VisionTargetResult());
            return;
        }

        ArrayList<CandidateInfo> candidatesInsideZone = new ArrayList<>();
        ArrayList<CandidateInfo> candidatesOutsideZone = new ArrayList<>();

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

        double circleRadiusP = (VisionConstants.TARGET_RECT_WIDTH_CM / 2.0) * pixelsPerCmForDrawing * processingScale;
        if (targetZoneCenterXAtProcessedScale != null && circleRadiusP > 0) {
            for (CandidateInfo c : candidatesInsideZone) {
                Point centerP = c.centerInProcessed;
                double distToLine = Math.abs(c.horizontalDistanceToCenterPx);

                if (distToLine <= circleRadiusP + 1) {
                    Point intersectP = new Point(targetZoneCenterXAtProcessedScale, Math.round(centerP.y + Math.sqrt(Math.max(0, circleRadiusP * circleRadiusP - distToLine * distToLine))));
                    if (targetRectY1Processed <= intersectP.y && intersectP.y <= targetRectY2Processed) {
                        c.lineAngleDeg = Math.toDegrees(Math.atan2(intersectP.x - centerP.x, intersectP.y - centerP.y));
                        double distPx = targetRectY2OriginalScale - (intersectP.y / processingScale);
                        c.distanceCm = (distPx >= 0) ? distPx / pixelsPerCmForDrawing : Double.POSITIVE_INFINITY;
                        c.primaryScore = c.distanceCm;
                    } else {
                        c.distanceCm = Double.POSITIVE_INFINITY;
                        c.primaryScore = Double.POSITIVE_INFINITY;
                    }
                } else {
                    double distPx = targetRectY2OriginalScale - (c.centerInProcessed.y / processingScale);
                    c.distanceCm = (distPx >= 0) ? distPx / pixelsPerCmForDrawing : Double.POSITIVE_INFINITY;
                    c.primaryScore = c.distanceCm;
                    c.lineAngleDeg = (c.horizontalDistanceToCenterPx > 0) ? -90.0 : 90.0;
                }
            }
        }

        Collections.sort(candidatesInsideZone, Comparator.comparingDouble(a -> a.primaryScore));

        if (candidatesInsideZone.size() >= 2) {
            CandidateInfo bestTarget = candidatesInsideZone.get(0);
            DetectedCube bestCube = allCubes.get(bestTarget.cubeIndex);
            double widthCm = (bestCube.rectWidthPx / processingScale * bestTarget.distanceCm) / VisionGraspingAPI.CAMERA_FOCAL_LENGTH_PIXELS;
            double heightCm = (bestCube.rectHeightPx / processingScale * bestTarget.distanceCm) / VisionGraspingAPI.CAMERA_FOCAL_LENGTH_PIXELS;
            apiInstance.setLatestResult(new VisionGraspingAPI.VisionTargetResult(true, bestCube.color, bestTarget.distanceCm, bestCube.angleDeg, bestTarget.lineAngleDeg, widthCm, heightCm, "In Position", 0, 0));

        } else if (candidatesInsideZone.size() == 1) {
            CandidateInfo currentTarget = candidatesInsideZone.get(0);
            DetectedCube currentCube = allCubes.get(currentTarget.cubeIndex);
            double widthCm = (currentCube.rectWidthPx / processingScale * currentTarget.distanceCm) / VisionGraspingAPI.CAMERA_FOCAL_LENGTH_PIXELS;
            double heightCm = (currentCube.rectHeightPx / processingScale * currentTarget.distanceCm) / VisionGraspingAPI.CAMERA_FOCAL_LENGTH_PIXELS;
            if (!candidatesOutsideZone.isEmpty()) {
                Collections.sort(candidatesOutsideZone, Comparator.comparingDouble(a -> Math.abs(a.horizontalDistanceToCenterPx)));
                CandidateInfo nextTarget = candidatesOutsideZone.get(0);
                double hOffsetPx = nextTarget.horizontalDistanceToCenterPx / processingScale;
                String moveDir = hOffsetPx > 0 ? "Right" : "Left";
                double distPx = targetRectY2OriginalScale - (nextTarget.centerInProcessed.y / processingScale);
                double rawDistCm = (distPx >= 0) ? (distPx / pixelsPerCmForDrawing) : 0;
                apiInstance.setLatestResult(new VisionGraspingAPI.VisionTargetResult(true, currentCube.color, currentTarget.distanceCm, currentCube.angleDeg, currentTarget.lineAngleDeg, widthCm, heightCm, moveDir, rawDistCm, hOffsetPx));
            } else {
                apiInstance.setLatestResult(new VisionGraspingAPI.VisionTargetResult(true, currentCube.color, currentTarget.distanceCm, currentCube.angleDeg, currentTarget.lineAngleDeg, widthCm, heightCm, "In Position", 0, 0));
            }
        } else {
            if (!candidatesOutsideZone.isEmpty()) {
                Collections.sort(candidatesOutsideZone, Comparator.comparingDouble(a -> Math.abs(a.horizontalDistanceToCenterPx)));
                CandidateInfo nextTarget = candidatesOutsideZone.get(0);
                DetectedCube nextCube = allCubes.get(nextTarget.cubeIndex);
                double hOffsetPx = nextTarget.horizontalDistanceToCenterPx / processingScale;
                String moveDir = hOffsetPx > 0 ? "Right" : "Left";
                double distPx = targetRectY2OriginalScale - (nextTarget.centerInProcessed.y / processingScale);
                double rawDistCm  = (distPx >= 0) ? (distPx / pixelsPerCmForDrawing) : 0;
                double widthCm = (nextCube.rectWidthPx / processingScale * rawDistCm) / VisionGraspingAPI.CAMERA_FOCAL_LENGTH_PIXELS;
                double heightCm = (nextCube.rectHeightPx / processingScale * rawDistCm) / VisionGraspingAPI.CAMERA_FOCAL_LENGTH_PIXELS;
                apiInstance.setLatestResult(new VisionGraspingAPI.VisionTargetResult(false, "None", Double.POSITIVE_INFINITY, 0, 0, widthCm, heightCm, moveDir, rawDistCm, hOffsetPx));
            } else {
                apiInstance.setLatestResult(new VisionGraspingAPI.VisionTargetResult());
            }
        }
    }

    public void release() {
        if (tempContour2f != null) tempContour2f.release();
    }
}