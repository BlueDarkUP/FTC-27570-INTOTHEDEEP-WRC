package org.firstinspires.ftc.teamcode.vision.app;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.API.ServoKinematics;
import org.firstinspires.ftc.teamcode.API.ServoKinematics.ServoTarget;
import org.firstinspires.ftc.teamcode.API.PositionCalculator;
import org.firstinspires.ftc.teamcode.pedroPathing.constants.AlgorithmLibrary;
import org.firstinspires.ftc.teamcode.pedroPathing.constants.ConstantMap;
import org.firstinspires.ftc.teamcode.vision.Interface.VisionGraspingAPI;

@TeleOp(name="Vision Grasping TeleOp", group="Main")
public class VisionGraspingTeleOp extends LinearOpMode {
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
     * 横向距离衰减因子。
     * 用于修正因目标偏离中心线导致的透视距离误差。
     * 值越大，衰减效果越强
     * 0.0 = 无衰减。
     */
    public static final double LATERAL_DISTANCE_ATTENUATION_FACTOR = 0.007;


    private VisionGraspingAPI visionAPI;
    private AlgorithmLibrary algorithmLibrary;

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.addLine("初始化API，初始化结构");

        visionAPI = new VisionGraspingAPI();
        visionAPI.init(hardwareMap);

        algorithmLibrary = new AlgorithmLibrary(hardwareMap);
        algorithmLibrary.Initialize_All_For_Vision();

        telemetry.addLine("Initialization Complete. Waiting for start...");
        telemetry.update();

        waitForStart();

        while (opModeIsActive() && !isStopRequested()) {

            VisionGraspingAPI.VisionTargetResult result = visionAPI.getLatestResult();
            telemetry.addData("Vision FPS", "%.2f", visionAPI.getFps());
            telemetry.addData("Vision Pipeline (ms)", "%.1f", visionAPI.getPipelineTimeMs());
            telemetry.addLine("--- Vision Result ---");
            telemetry.addData("Target Found", result.isTargetFound);

            if (result.isTargetFound) {
                telemetry.addData("Target Color", result.color);
                telemetry.addData("Raw Distance (cm)", "%.1f", result.distanceCm);
                telemetry.addData("Object Angle (α)", "%.1f deg", result.objectAngleDeg);
                telemetry.addData("Line Angle (β)", "%.1f deg", result.lineAngleDeg);

                telemetry.addLine("--- Servo Calculation ---");
                // 对原始距离进行指数修正
                double exponentialCorrectedDistance = DISTANCE_CORRECTION_REFERENCE_CM * Math.pow(result.distanceCm / DISTANCE_CORRECTION_REFERENCE_CM, DISTANCE_CORRECTION_EXPONENT);
                telemetry.addData("1. Exp Corrected Dist", "%.1f cm", exponentialCorrectedDistance);

                // 步骤B: 计算横向衰减
                double lateralOffsetCm = Math.abs(result.distanceCm * Math.tan(Math.toRadians(result.lineAngleDeg)));

                // 二次方衰减
                // attenuation = 1 - factor * (offset / reference)^2
                double attenuation = 1.0 - LATERAL_DISTANCE_ATTENUATION_FACTOR * (lateralOffsetCm * lateralOffsetCm);
                // 确保衰减系数不会小于一个合理的下限，比如0.8，防止过度缩回
                attenuation = Math.max(0.8, attenuation);

                telemetry.addData("2. Lateral Attenuation", "%.3f (at %.1f cm offset)", attenuation, lateralOffsetCm);

                // 应用衰减，得到最终的滑轨目标距离
                double finalCorrectedDistance = exponentialCorrectedDistance * attenuation;
                telemetry.addData("3. Final Corrected Dist", "%.1f cm", finalCorrectedDistance);

                // 使用最终修正后的距离来调用运动学API
                ServoTarget sliderTarget = ServoKinematics.calculateServoTarget(finalCorrectedDistance);

                if (sliderTarget != null) {
                    telemetry.addData("-> Slider Servo Pos", "%.4f", sliderTarget.servoPosition);
                } else {
                    telemetry.addData("-> Slider Servo Pos", "Out of Range");
                }

                double alpha = result.objectAngleDeg;
                double beta = result.lineAngleDeg;

                double servoAAngle = -beta + 90;
                if (beta > 0) {
                    servoAAngle += LEFT_SIDE_AIM_CORRECTION_DEGREES;
                    telemetry.addData("Aim Correction", "Applied %.1f deg", LEFT_SIDE_AIM_CORRECTION_DEGREES);
                }

                double servoBAngle = alpha + beta;

                double TURN_SERVO_POS = PositionCalculator.calculatePositionValue(0.53, 1, 90, true, servoAAngle);
                double ROTATE_SERVO_POS = PositionCalculator.calculatePositionValue(0.07, 0.62, 90, false, servoBAngle);


                // --- 4. 执行抓取动作 ---
                if (gamepad1.right_bumper && sliderTarget != null) {
                    algorithmLibrary.performVisionGrasp(sliderTarget.servoPosition, TURN_SERVO_POS, ROTATE_SERVO_POS);
                    sleep(500);
                }

            } else {
                telemetry.addLine("--- No Target Detected ---");
            }

            if (gamepad1.left_bumper) {
                AlgorithmLibrary.camera_arm.setPosition(ConstantMap.Camera_Arm_PutDown_Position);
            }

            telemetry.update();
        }
        visionAPI.close();
    }
}