package org.firstinspires.ftc.teamcode.vision;

import org.firstinspires.ftc.teamcode.API.PositionCalculator;
import org.firstinspires.ftc.teamcode.API.ServoKinematics;
import org.firstinspires.ftc.teamcode.API.ServoKinematics.ServoTarget;

import java.util.Locale;

/**
 /**
 * 抓取计算器API
 * 这是一个独立的工具类，封装了所有与“如何抓取”相关的数学计算逻辑。
 * 它将视觉系统提供的原始数据（如距离、角度）转换为机器人可以直接使用的具体数值
 * （如伺服舵机位置、底盘移动距离）。
 *
 * 此类完全解耦了视觉识别与机器人运动学，使得两部分的逻辑可以独立修改和维护。
 * 它依赖于 VisionGraspingAPI.VisionTargetResult 的数据结构来获取输入。
 * 所有校正因子、运动学模型和计算公式都集中在此，便于统一调整和优化抓取表现。
 * @author BlueDarkUP
 * @version 2025/6 (Refactored)
 * To My Lover - Zyy
 */
public class GraspingCalculator {

    // --- 计算常量 ---
    public static final double DISTANCE_CORRECTION_EXPONENT = 0.95;
    public static final double DISTANCE_CORRECTION_REFERENCE_CM = 24.0;
    public static final double LEFT_SIDE_AIM_CORRECTION_DEGREES = -13.0;
    public static final double LATERAL_DISTANCE_ATTENUATION_FACTOR = 0.005;

    /**
     * 用于封装抓取计算结果的数据结构。
     */
    public static class GraspCalculations {
        public final boolean isWithinRange;
        public final double finalCorrectedDistanceCm;
        public final double sliderServoPos;
        public final double turnServoPos;
        public final double rotateServoPos;
        // 用于遥测的中间值
        public final double exponentialCorrectedDistance;
        public final double lateralAttenuation;
        public final double lateralOffsetCm;
        public final double servoAAngle;
        public final double servoBAngle;

        public GraspCalculations(boolean isWithinRange, double finalCorrectedDistanceCm, double sliderServoPos, double turnServoPos, double rotateServoPos, double exponentialCorrectedDistance, double lateralAttenuation, double lateralOffsetCm, double servoAAngle, double servoBAngle) {
            this.isWithinRange = isWithinRange;
            this.finalCorrectedDistanceCm = finalCorrectedDistanceCm;
            this.sliderServoPos = sliderServoPos;
            this.turnServoPos = turnServoPos;
            this.rotateServoPos = rotateServoPos;
            this.exponentialCorrectedDistance = exponentialCorrectedDistance;
            this.lateralAttenuation = lateralAttenuation;
            this.lateralOffsetCm = lateralOffsetCm;
            this.servoAAngle = servoAAngle;
            this.servoBAngle = servoBAngle;
        }
    }

    /**
     * 用于封装移动建议计算结果的数据结构。
     */
    public static class MoveSuggestion {
        public final double moveCm;
        public final double estimatedDepthCm;

        public MoveSuggestion(double moveCm, double estimatedDepthCm) {
            this.moveCm = moveCm;
            this.estimatedDepthCm = estimatedDepthCm;
        }
    }


    /**
     * 根据视觉结果计算所有抓取所需的伺服位置。
     * @param visionResult 从 VisionGraspingAPI 获取的视觉结果。
     * @return 一个 GraspCalculations 对象，包含所有计算出的伺服位置和中间值。
     */
    public static GraspCalculations calculateGrasp(VisionGraspingAPI.VisionTargetResult visionResult) {
        if (!visionResult.isTargetFound) {
            // 如果没找到目标，返回一个无效的默认结果
            return new GraspCalculations(false, 0, 0, 0, 0, 0, 0, 0, 0, 0);
        }

        // 1. 指数距离校正
        double exponentialCorrectedDistance = DISTANCE_CORRECTION_REFERENCE_CM * Math.pow(visionResult.distanceCm / DISTANCE_CORRECTION_REFERENCE_CM, DISTANCE_CORRECTION_EXPONENT);

        // 2. 横向衰减计算
        double lateralOffsetCm = Math.abs(visionResult.distanceCm * Math.tan(Math.toRadians(visionResult.lineAngleDeg)));
        double attenuation = 1.0 - LATERAL_DISTANCE_ATTENUATION_FACTOR * (lateralOffsetCm * lateralOffsetCm);
        attenuation = Math.max(0.8, attenuation); // 衰减下限

        // 3. 最终校正距离
        double finalCorrectedDistance = exponentialCorrectedDistance * attenuation;

        // 4. 计算滑轨伺服位置
        ServoTarget sliderTarget = ServoKinematics.calculateServoTarget(finalCorrectedDistance);
        boolean isWithinRange = (sliderTarget != null);
        double sliderServoPos = isWithinRange ? sliderTarget.servoPosition : 0.0;

        // 5. 计算旋转和转向伺服角度及位置
        double alpha = visionResult.objectAngleDeg;
        double beta = visionResult.lineAngleDeg;

        double servoAAngle = -beta + 90; // 转向伺服A的角度 (Turn)
        if (beta > 0) { // 如果目标在左侧
            servoAAngle += LEFT_SIDE_AIM_CORRECTION_DEGREES;
        }
        double servoBAngle = alpha + beta; // 旋转伺服B的角度 (Rotate)

        double turnServoPos = PositionCalculator.calculatePositionValue(0.53, 1, 90, true, servoAAngle);
        double rotateServoPos = PositionCalculator.calculatePositionValue(0.07, 0.62, 90, false, servoBAngle);

        return new GraspCalculations(isWithinRange, finalCorrectedDistance, sliderServoPos, turnServoPos, rotateServoPos, exponentialCorrectedDistance, attenuation, lateralOffsetCm, servoAAngle, servoBAngle);
    }

    /**
     * 根据视觉结果计算精确的横向移动建议。
     * @param visionResult 从 VisionGraspingAPI 获取的视觉结果。
     * @return 一个 MoveSuggestion 对象，包含建议的移动距离(cm)。
     */
    public static MoveSuggestion calculateMove(VisionGraspingAPI.VisionTargetResult visionResult) {
        // 当 isTargetFound 为 true 时，rawDistCm 是准确的 distanceCm
        // 当 isTargetFound 为 false 时，rawDistCm 是估算的
        double depthForCalc = visionResult.isTargetFound ? visionResult.distanceCm : visionResult.rawDistanceToGraspLineCm;
        double hOffsetPx = visionResult.horizontalOffsetPx;

        // 使用与抓取计算相同的指数校正来估算深度
        double estimatedDepthCm = DISTANCE_CORRECTION_REFERENCE_CM * Math.pow(depthForCalc / DISTANCE_CORRECTION_REFERENCE_CM, DISTANCE_CORRECTION_EXPONENT);

        // 根据相似三角形原理计算横向移动距离
        double finalMoveCm = (hOffsetPx * estimatedDepthCm) / VisionGraspingAPI.CAMERA_FOCAL_LENGTH_PIXELS;

        return new MoveSuggestion(finalMoveCm, estimatedDepthCm);
    }
}