package org.firstinspires.ftc.teamcode.vision;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.pedroPathing.constants.AlgorithmLibrary;
import org.firstinspires.ftc.teamcode.pedroPathing.constants.ConstantMap;

/**
 * @author BlueDarkUP
 * @version 2025/6
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
        waitForStart();

        while (opModeIsActive() && !isStopRequested()) {
            VisionTargetResult result = visionAPI.getLatestResult();

            telemetry.addData("Vision FPS", "%.2f", visionAPI.getFps());
            telemetry.addData("Vision Pipeline (ms)", "%.1f", visionAPI.getPipelineTimeMs());
            telemetry.addLine("--- Vision Result ---");
            telemetry.addData("Target Found in Zone", result.isTargetFound);
            telemetry.addData("Graspable Targets", result.graspableTargetsInZone);

            if (gamepad1.right_bumper) {
                // --- 按下 Bumper 时的逻辑 ---
                if (result.isTargetFound) { // 抓取区有目标
                    GraspingTarget graspTarget = VisionGraspingCalculator.calculate(result, telemetry);
                    if (graspTarget.isInRange) {
                        telemetry.addLine("正在执行抓取...");
                        telemetry.update();
                        algorithmLibrary.performVisionGrasp(graspTarget.sliderServoPosition, graspTarget.turnServoPosition, graspTarget.rotateServoPosition);
                        sleep(500); // 抓取动作的延时
                    } else {
                        telemetry.addLine("目标在抓取区但超出机械臂范围！");
                    }
                } else {
                    telemetry.addLine("当前抓取区无目标。");
                }

                // 下一步规划
                telemetry.addLine("\n--- Next Step Planning ---");
                if (result.graspableTargetsInZone > 1) {
                    telemetry.addLine("下一步: 无需移动, 此处还有目标。");
                } else {
                    if (result.nextTargetHorizontalOffsetCm != 0.0) {
                        if (result.nextTargetHorizontalOffsetCm > 0) {
                            telemetry.addLine(String.format("建议: 向右移动 %.1f cm", result.nextTargetHorizontalOffsetCm));
                        } else {
                            telemetry.addLine(String.format("建议: 向左移动 %.1f cm", -result.nextTargetHorizontalOffsetCm));
                        }
                    } else {
                        telemetry.addLine("建议: 视野内已无其他目标, 请大幅移动。");
                    }
                }

            } else {
                // --- 未按下 Bumper 时的正常显示 ---
                if (result.isTargetFound) {
                    GraspingTarget graspTarget = VisionGraspingCalculator.calculate(result, telemetry);
                    telemetry.addData("-> Graspable", graspTarget.isInRange);
                } else {
                    telemetry.addLine("-> Graspable: N/A");
                }
                telemetry.addLine("\n按下 Right Bumper 以抓取并获取下一步建议");
            }

            if (gamepad1.left_bumper) {
                AlgorithmLibrary.camera_arm.setPosition(ConstantMap.Camera_Arm_PutDown_Position);
            }
            telemetry.update();
        }
        visionAPI.close();
    }
}