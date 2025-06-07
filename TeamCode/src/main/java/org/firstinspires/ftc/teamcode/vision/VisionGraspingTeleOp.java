package org.firstinspires.ftc.teamcode.vision;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.pedroPathing.constants.AlgorithmLibrary;
import org.firstinspires.ftc.teamcode.pedroPathing.constants.ConstantMap;

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
            telemetry.addData("Target Found", result.isTargetFound);

            if (result.isTargetFound) {
                GraspingTarget graspTarget = VisionGraspingCalculator.calculate(result, telemetry);
                if (gamepad1.right_bumper && graspTarget.isInRange) {
                    algorithmLibrary.performVisionGrasp(graspTarget.sliderServoPosition, graspTarget.turnServoPosition, graspTarget.rotateServoPosition);
                    sleep(500);
                }
            }
            if (gamepad1.left_bumper) {
                AlgorithmLibrary.camera_arm.setPosition(ConstantMap.Camera_Arm_PutDown_Position);
            }
            telemetry.update();
        }
        visionAPI.close();
    }
}