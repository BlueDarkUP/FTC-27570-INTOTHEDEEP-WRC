package org.firstinspires.ftc.teamcode.vision;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.pedroPathing.constants.AlgorithmLibrary;
import org.firstinspires.ftc.teamcode.pedroPathing.constants.ConstantMap;

import java.util.Locale;

/**
 * 调用视觉API的TeleOP
 * 处理新的移动建议逻辑
 * @author BlueDarkUP
 * @version 2025/6.5
 * To My Lover - Zyy
 */

@TeleOp(name="Vision Grasping TeleOp", group="Main")
public class VisionGraspingTeleOp extends LinearOpMode {

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
            telemetry.addData("Target in Zone", result.isTargetFound);

            // --- 显示抓取信息 (如果找到目标) ---
            if (result.isTargetFound) {
                // 调用计算器API获取所有抓取参数
                GraspingCalculator.GraspCalculations grasp = GraspingCalculator.calculateGrasp(result);

                telemetry.addData("Target Color", result.color);
                telemetry.addData("Raw Distance (cm)", "%.1f", result.distanceCm);
                telemetry.addData("Object Angle (α)", "%.1f deg", result.objectAngleDeg);
                telemetry.addData("Line Angle (β)", "%.1f deg", result.lineAngleDeg);

                telemetry.addLine("--- Servo Calculation ---");
                telemetry.addData("1. Exp Corrected Dist", "%.1f cm", grasp.exponentialCorrectedDistance);
                telemetry.addData("2. Lateral Attenuation", "%.3f (at %.1f cm offset)", grasp.lateralAttenuation, grasp.lateralOffsetCm);
                telemetry.addData("3. Final Corrected Dist", "%.1f cm", grasp.finalCorrectedDistanceCm);

                if (grasp.isWithinRange) {
                    telemetry.addData("-> Slider Servo Pos", "%.4f", grasp.sliderServoPos);
                    telemetry.addData("TurnTableServoPos", grasp.turnServoPos);
                    telemetry.addData("RotateServoPose", grasp.rotateServoPos);
                } else {
                    telemetry.addData("-> Slider Servo Pos", "Out of Range");
                }

                if (result.lineAngleDeg > 0) { // 如果目标在左侧
                    telemetry.addData("Aim Correction", "Applied %.1f deg", GraspingCalculator.LEFT_SIDE_AIM_CORRECTION_DEGREES);
                }

                if (gamepad1.right_bumper && grasp.isWithinRange) {
                    algorithmLibrary.performVisionGrasp(grasp.sliderServoPos, grasp.turnServoPos, grasp.rotateServoPos);
                    sleep(500);
                }
            }

            // --- 显示移动建议 (如果不为 "None") ---
            if (!result.nextMoveDirection.equals("None")) {
                if (result.nextMoveDirection.equals("In Position")) {
                    telemetry.addLine("--- Next Action: In Position ---");
                } else {
                    telemetry.addLine("--- Next Target Plan ---");
                    telemetry.addData("1. Raw Data", "H-Offset: %.1f px, Raw-Depth: %.1f cm", result.horizontalOffsetPx, result.rawDistanceToGraspLineCm);

                    // 调用计算器API获取移动建议
                    GraspingCalculator.MoveSuggestion move = GraspingCalculator.calculateMove(result);

                    telemetry.addData("2. Corrected Depth", "%.1f cm", move.estimatedDepthCm);
                    telemetry.addData("3. PRECISE MOVE ACTION", String.format(Locale.US, "Move %s by %.1f cm", result.nextMoveDirection, Math.abs(move.moveCm)));
                }
            } else {
                telemetry.addLine("--- No Target Detected ---");
                telemetry.addData("Next Action", "Scan for Targets");
            }

            if (gamepad1.left_bumper) {
                algorithmLibrary.camera_arm.setPosition(ConstantMap.Camera_Arm_PutDown_Position);
            }

            telemetry.update();
        }
        visionAPI.close();
    }
}