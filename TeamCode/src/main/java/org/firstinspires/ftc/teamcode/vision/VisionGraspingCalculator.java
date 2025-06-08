package org.firstinspires.ftc.teamcode.vision;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.API.PositionCalculator;
import org.firstinspires.ftc.teamcode.API.ServoKinematics;
import org.firstinspires.ftc.teamcode.API.ServoKinematics.ServoTarget;
/**
 * @author BlueDarkUP
 * @version 2025/6
 * To My Lover - Zyy
 */
public class VisionGraspingCalculator {

    /**
     * X轴非线性修正指数
     */
    public static final double DISTANCE_CORRECTION_EXPONENT = 1.06;
    public static final double DISTANCE_CORRECTION_REFERENCE_CM = 24.0;

    /**
     * 左侧目标瞄准修正角度
     * 当目标在视野左侧时，A舵机角度会加上此值
     */
    public static final double LEFT_SIDE_AIM_CORRECTION_DEGREES = -15.0;

    /**
     * 横向距离衰减因子
     * 用于修正因目标偏离中心线导致的透视距离误差
     * 值越大，衰减效果越强
     * 0.0 = 无衰减
     */
    public static final double LATERAL_DISTANCE_ATTENUATION_FACTOR = 0.007;

    /**
     * 主要的计算方法。接收视觉结果和Telemetry对象，返回一个GraspingTarget对象
     * @param visionResult 从VisionGraspingAPI获取的原始视觉数据
     * @param telemetry OpMode的Telemetry对象，用于在计算过程中输出调试信息
     * @return 包含所有舵机目标位置的GraspingTarget对象
     */
    public static GraspingTarget calculate(VisionTargetResult visionResult, Telemetry telemetry) {
        // 指数修正
        double exponentialCorrectedDistance = DISTANCE_CORRECTION_REFERENCE_CM * Math.pow(visionResult.distanceCm / DISTANCE_CORRECTION_REFERENCE_CM, DISTANCE_CORRECTION_EXPONENT);
        telemetry.addData("1. Exp Corrected Dist", "%.1f cm", exponentialCorrectedDistance);

        // 横向衰减
        double lateralOffsetCm = Math.abs(visionResult.distanceCm * Math.tan(Math.toRadians(visionResult.lineAngleDeg)));
        double attenuation = 1.0 - LATERAL_DISTANCE_ATTENUATION_FACTOR * (lateralOffsetCm * lateralOffsetCm);
        attenuation = Math.max(0.8, attenuation); // 确保衰减系数不会小于一个合理的下限
        telemetry.addData("2. Lateral Attenuation", "%.3f (at %.1f cm offset)", attenuation, lateralOffsetCm);


        double finalCorrectedDistance = exponentialCorrectedDistance * attenuation;
        telemetry.addData("3. Final Corrected Dist", "%.1f cm", finalCorrectedDistance);


        ServoTarget sliderTarget = ServoKinematics.calculateServoTarget(finalCorrectedDistance);

        if (sliderTarget == null) {
            telemetry.addData("-> Slider Servo Pos", "Out of Range");
            return new GraspingTarget(0, 0, 0, false);
        }

        telemetry.addData("-> Slider Servo Pos", "%.4f", sliderTarget.servoPosition);


        double alpha = visionResult.objectAngleDeg;
        double beta = visionResult.lineAngleDeg;

        double servoAAngle = -beta + 90;
        if (beta > 0) { // 目标在左侧
            servoAAngle += LEFT_SIDE_AIM_CORRECTION_DEGREES;
            telemetry.addData("Aim Correction", "Applied %.1f deg", LEFT_SIDE_AIM_CORRECTION_DEGREES);
        }

        double servoBAngle = alpha + beta;

        double turnServoPos = PositionCalculator.calculatePositionValue(0.53, 1, 90, true, servoAAngle);
        double rotateServoPos = PositionCalculator.calculatePositionValue(0.07, 0.62, 90, false, servoBAngle);


        return new GraspingTarget(sliderTarget.servoPosition, turnServoPos, rotateServoPos, true);
    }
}