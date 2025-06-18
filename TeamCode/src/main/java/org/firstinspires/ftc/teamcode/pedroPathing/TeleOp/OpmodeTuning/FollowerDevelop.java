package org.firstinspires.ftc.teamcode.pedroPathing.TeleOp.OpmodeTuning;

import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Localizer;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.pathgen.Point;
import com.pedropathing.util.Constants;
import  com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.pedroPathing.constants.AlgorithmLibrary;
import org.firstinspires.ftc.teamcode.pedroPathing.constants.FConstants;
import org.firstinspires.ftc.teamcode.pedroPathing.constants.LConstants;
import org.firstinspires.ftc.teamcode.vision.VisionGraspingAPI;


/**
 * This is an example teleop that showcases movement and robot-centric driving.
 *
 * @author Baron Henderson - 20077 The Indubitables
 * @version 2.0, 12/30/2024
 */

@TeleOp(name = "FollowerDevelop", group = "Tuning")
public class FollowerDevelop extends OpMode {
    private Follower follower;
    private AlgorithmLibrary Algorithm;
    private VisionGraspingAPI visionAPI;
    Switcher HoldFlag,AutoFollowerFlag;
    private final Pose startPose = new Pose(0,0,0);
    private PathChain ToCenter;

    @Override
    public void init() {
        follower = new Follower(hardwareMap,FConstants.class,LConstants.class);
        follower.setStartingPose(startPose);
        Algorithm= new AlgorithmLibrary(hardwareMap);
        HoldFlag = new Switcher();
        AutoFollowerFlag = new Switcher();
        visionAPI = new VisionGraspingAPI();
        visionAPI.init(hardwareMap, VisionGraspingAPI.AllianceColor.BLUE);
    }

    @Override
    public void init_loop() {
    }

    @Override
    public void start() {
        follower.startTeleopDrive();
    }

    @Override
    public void loop() {

        /* Update Pedro to move the robot based on:
        - Forward/Backward Movement: -gamepad1.left_stick_y
        - Left/Right Movement: -gamepad1.left_stick_x
        - Turn Left/Right Movement: -gamepad1.right_stick_x
        - Robot-Centric Mode: true
        */
        HoldPoseTester();
        try {
            AutoPathTester();
        } catch (InterruptedException e) {
            throw new RuntimeException(e);
        }
        follower.setTeleOpMovementVectors(-gamepad1.left_stick_y, -gamepad1.left_stick_x, -gamepad1.right_stick_x, true);
        follower.update();

        telemetry.addData("X", follower.getPose().getX());
        telemetry.addData("Y", follower.getPose().getY());
        telemetry.addData("Heading in Degrees", Math.toDegrees(follower.getPose().getHeading()));

        /* Update Telemetry to the Driver Hub */
        telemetry.update();
    }
    private void BuildPath(){
        ToCenter = follower.pathBuilder()
                .addPath(new BezierLine(follower.getPose(),startPose))
                .setLinearHeadingInterpolation(follower.getPose().getHeading(),startPose.getHeading())
                .build();
    }
    private void HoldPoseTester(){
        if(gamepad1.cross&&!HoldFlag.LastFlag) {
                Pose nowPose = follower.getPose();
                follower.holdPoint(nowPose);
                follower.update();
                telemetry.addLine("Holding Point");
                telemetry.update();
                while(!gamepad1.cross) {
                    follower.holdPoint(nowPose);
                    follower.update();
                }
                follower.startTeleopDrive();
                follower.setTeleOpMovementVectors(-gamepad1.left_stick_y, -gamepad1.left_stick_x, -gamepad1.right_stick_x, true);
                follower.update();
            }

        HoldFlag.RecordLF(gamepad1.cross);
    }

    private void AutoPathTester() throws InterruptedException {
        if(gamepad1.circle&&!AutoFollowerFlag.LastFlag){
            BuildPath();
            follower.followPath(ToCenter,true);
            follower.update();
            while(Algorithm.AutoPilotBreak(gamepad1)&&!follower.atParametricEnd()){
                follower.update();
                telemetry.addData("X", follower.getPose().getX());
                telemetry.addData("Y", follower.getPose().getY());
                telemetry.addData("Heading in Degrees", Math.toDegrees(follower.getPose().getHeading()));
                /* Update Telemetry to the Driver Hub */
                telemetry.update();
            }
            follower.holdPoint(new Pose(0,0,0));
            Thread.sleep(120);
            follower.startTeleopDrive();
            follower.setTeleOpMovementVectors(-gamepad1.left_stick_y, -gamepad1.left_stick_x, -gamepad1.right_stick_x, true);
            follower.update();
        }
        AutoFollowerFlag.RecordLF(gamepad1.circle);
    }

    @Override
    public void stop() {
        if(visionAPI != null)
            visionAPI.close();
    }
}