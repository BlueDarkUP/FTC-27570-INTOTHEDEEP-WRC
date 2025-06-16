package org.firstinspires.ftc.teamcode.vision;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.pedroPathing.constants.AlgorithmLibrary;
import org.firstinspires.ftc.teamcode.pedroPathing.constants.ConstantMap;

import java.util.Locale;

/**
 * 调用视觉API的示例手动操作程序 (TeleOp)
 * 本程序演示了如何正确地初始化、使用和关闭 VisionGraspingAPI。
 * 它从API获取最新的视觉结果，并结合 GraspingCalculator 的计算，
 * 将复杂的数据转换成直观的遥测信息和可执行的机器人动作。
 *
 * 这个类是 VisionGraspingAPI 和 GraspingCalculator 的“消费者”或“客户端”。
 * 它专注于用户交互和机器人控制逻辑，将所有复杂的视觉处理和数学计算委托给专门的API。
 * 实现了对 "nextMoveDirection" 字段的处理，可以根据视觉建议进行精确的横向移动。
 * @author BlueDarkUP
 * @version 2025/6 (Refactored)
 * To My Lover - Zyy
 */
@TeleOp(name="Vision Grasping TeleOp", group="Main")
public class VisionGraspingTeleOp extends LinearOpMode {
    private static final VisionGraspingAPI.AllianceColor TARGET_COLOR = VisionGraspingAPI.AllianceColor.RED; // 可选项: .RED 或 .BLUE


    private VisionGraspingAPI visionAPI;
    private AlgorithmLibrary algorithmLibrary;

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.addLine("初始化API，初始化结构");
        telemetry.addData(">> 识别颜色", TARGET_COLOR.toString());
        telemetry.update();

        visionAPI = new VisionGraspingAPI();
        // 在初始化时传入选择的颜色
        visionAPI.init(hardwareMap, TARGET_COLOR);

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