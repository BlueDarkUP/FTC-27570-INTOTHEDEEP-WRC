package org.firstinspires.ftc.teamcode.pedroPathing.TeleOp;

import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.pathgen.Point;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.pedroPathing.constants.ConstantMap;
import org.firstinspires.ftc.teamcode.pedroPathing.constants.AlgorithmLibrary;
import org.firstinspires.ftc.teamcode.pedroPathing.constants.FConstants;
import org.firstinspires.ftc.teamcode.pedroPathing.constants.LConstants;
import org.firstinspires.ftc.teamcode.vision.GraspingCalculator;
import org.firstinspires.ftc.teamcode.vision.VisionGraspingAPI;

/**
 * This is the fucking best teleop code in the world.
 * @author Luca Li
 * @version 2025/5
 *(L2 autopilot connected!
 */

@TeleOp (name = "TeleOp-Code-27570-RED",group = "A111-Competition")

public class TeleOp_27570_Red extends OpMode {
    private static Follower follower;
    private AlgorithmLibrary Algorithm;
    private VisionGraspingAPI visionAPI;
    private boolean BackGrabLastFlag=false,IntakeRotateLastFlag=false;
    private boolean ClawLastFlag=false,IntakeSlideLastFlag = false;
    private boolean ArmLastFlag = false,CameraArmLastFlag = false;
    private boolean buildAutoScoring1LastFlag = false;
    private boolean buildAutoScoring2LastFlag = false;
    private boolean RebuildMapIsReady = false;
    private boolean EmergencyFlag = false;
    private boolean VLflag = false;

    private static double nextPointDistance = 0;
    private static Pose PoseNow = null;

    private final Pose GetSpecPosition = new Pose(8.955,31,Math.toRadians(0));
    private static Pose ScoringInitialPosition = new Pose();
    private PathChain ToIntake,Scoring,GetSpec;
    private boolean Eflag1 = false,Eflag2 = false;

    private ElapsedTime Emergencytimer1 = new ElapsedTime(),Emergencytimer2 = new ElapsedTime();

    private Timer opmodeTimer;
    private final Pose startPose = new Pose(10,7.8,0);

    /** This method is call once when init is played, it initializes the follower **/
    private void PathBuilderIntake(){
        ToIntake = follower.pathBuilder()
                .addPath(new BezierLine(new Point(PoseNow),new Point(PoseNow.getX()+nextPointDistance,PoseNow.getY())))
                .setConstantHeadingInterpolation(PoseNow.getHeading())
                .build();
    }
    private void PathBuilderScoring(){
        double ScoreNowY = ScoringInitialPosition.getY()-nextPointDistance;
        if(ScoreNowY<64){
            ScoreNowY=64;
        }
        Scoring = follower.pathBuilder()
                .addPath(new BezierLine(new Point(GetSpecPosition),new Point(new Pose(ConstantMap.ScorePoseX,ScoreNowY))))
                .setConstantHeadingInterpolation(GetSpecPosition.getHeading())
                .build();
        GetSpec = follower.pathBuilder()
                .addPath(new BezierLine(new Point(new Pose(ConstantMap.ScorePoseX,ScoreNowY)),new Point(GetSpecPosition)))
                .setConstantHeadingInterpolation(GetSpecPosition.getHeading())
                .build();
    }
    @Override
    public void init() {
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();
        follower = new Follower(hardwareMap, FConstants.class, LConstants.class);
        follower.setStartingPose(startPose);
        Algorithm = new AlgorithmLibrary(hardwareMap);
        visionAPI = new VisionGraspingAPI();
        visionAPI.init(hardwareMap, VisionGraspingAPI.AllianceColor.BLUE);
        try {
            Algorithm.Initialize_All_For_TeleOp();
        } catch (InterruptedException e) {
            throw new RuntimeException(e);
        }
        Emergencytimer1.reset();
        Emergencytimer2.reset();
    }

    /** This method is called continuously after Init while waiting to be started. **/
    @Override
    public void init_loop() {
    }

    /** This method is called once at the start of the OpMode. **/
    @Override
    public void start() {
        follower.startTeleopDrive();
    }

    /** This is the main loop of the opmode and runs continuously after play **/
    @Override
    public void loop() {

        /* Update Pedro to move the robot based on:
        - Forward/Backward Movement: -gamepad1.left_stick_y
        - Left/Right Movement: -gamepad1.left_stick_x
        - Turn Left/Right Movement: -gamepad1.right_stick_x
        - Robot-Centric Mode: true
        */

        follower.setTeleOpMovementVectors(Math.pow(-gamepad1.left_stick_y,3), Math.pow(-gamepad1.left_stick_x,3), +gamepad1.left_trigger*gamepad1.left_trigger-gamepad1.right_trigger*gamepad1.right_trigger, false);
        follower.update();

        PoseNow = follower.getPose();
        BackGrabController();
        RotateController();
        EmergencyInitMotors();
        CameraArmController();
        ClimbController();
        try {
            ResetHeading();
            SlideController();
            VisionController();
            AutoScoringController();
            ForwardClawController();
            ArmUpController();
        } catch (InterruptedException e) {
            throw new RuntimeException(e);
        }
        /* Telemetry Outputs of our Follower */
        telemetry.addLine("请在使用自动驾驶时专注于场上情况随时接管哦,,Ծ‸Ծ,,");
        telemetry.addLine("自动模式运行时不要触碰左摇杆！          つ♡⊂");
        telemetry.addData("X:", follower.getPose().getX());
        telemetry.addData("Y:", follower.getPose().getY());
        telemetry.addData("Heading in Degrees", Math.toDegrees(follower.getPose().getHeading()));

        /* Update Telemetry to the Driver Hub */
        telemetry.update();

    }

    @Override
    public void stop() {
        if(visionAPI != null)
            visionAPI.close();
    }
    private void VisionController() throws InterruptedException {
        VisionGraspingAPI.VisionTargetResult result = visionAPI.getLatestResult();
        if (result.isTargetFound) {
            GraspingCalculator.GraspCalculations grasp = GraspingCalculator.calculateGrasp(result);
            if(gamepad1.right_stick_button&&!VLflag) {
                Algorithm.SpinnerToCenter();
                Pose nowPose = follower.getPose();
                if(!result.nextMoveDirection.equals("None")&&!grasp.isWithinRange) {
                    GraspingCalculator.MoveSuggestion move = GraspingCalculator.calculateMove(result);
                    nextPointDistance = -move.moveCm* ConstantMap.CM_TO_INCH;
                    PathBuilderIntake();
                    follower.followPath(ToIntake);
                    follower.update();
                    Algorithm.ClawFlag = true;
                    Algorithm.BackGrabFlag = true;
                    Algorithm.BackGrabAction();
                    Algorithm.ForwardGrabController();
                    while (!follower.atParametricEnd()) {
                        if(!Algorithm.AutoPilotBreak(gamepad1)){
                            Algorithm.followerReset(follower,follower.getPose());
                            follower.startTeleopDrive();
                            return;
                        }
                    }
                }
                follower.holdPoint(nowPose);
                follower.update();
                while(follower.getPose()!=nowPose){
                    follower.holdPoint(nowPose);
                    follower.update();
                }
                VisionIntake();
                follower.startTeleopDrive();
                follower.setTeleOpMovementVectors(Math.pow(-gamepad1.left_stick_y,3), Math.pow(-gamepad1.left_stick_x,3), +gamepad1.left_trigger*gamepad1.left_trigger-gamepad1.right_trigger*gamepad1.right_trigger, false);
                follower.update();
            }
            VLflag = gamepad1.right_stick_button;
        }
    }
    private void VisionIntake() throws InterruptedException {
        VisionGraspingAPI.VisionTargetResult result = visionAPI.getLatestResult();
        if (result.isTargetFound) {
            GraspingCalculator.GraspCalculations grasp = GraspingCalculator.calculateGrasp(result);
            if(grasp.isWithinRange){
                Algorithm.ClawFlag = true;
                Algorithm.BackGrabFlag = true;
                Algorithm.BackGrabAction();
                Algorithm.ForwardGrabController();
                Algorithm.performVisionGrasp(grasp.sliderServoPos, grasp.turnServoPos, grasp.rotateServoPos);
                Algorithm.IntakeSlideFlag=true;
                Algorithm.SlideController();
            }
        }
    }
    private void CameraArmController(){
        if(gamepad1.dpad_down&&!CameraArmLastFlag){
            Algorithm.CameraArmController();
        }

        CameraArmLastFlag = gamepad1.dpad_down;
    }
    private void ResetHeading() throws InterruptedException {
        if(gamepad1.dpad_up){
            follower.poseUpdater.resetIMU();
        }
    }
    private void BackGrabController(){
        if(gamepad1.square&&!BackGrabLastFlag){
            Algorithm.BackGrabAction();
        }

        BackGrabLastFlag = gamepad1.square;
    }
    private void ForwardClawController() throws InterruptedException {
        if(gamepad1.cross&!ClawLastFlag){
            Algorithm.IntakeController();
        }
        ClawLastFlag = gamepad1.cross;
    }
    private void SlideController() throws InterruptedException {
        if(gamepad1.right_bumper&!IntakeSlideLastFlag){
            Algorithm.SlideController();
        }
        IntakeSlideLastFlag = gamepad1.right_bumper;
    }
    private void RotateController(){
        if(gamepad1.triangle&!IntakeRotateLastFlag){
            Algorithm.RotateController();
        }
        IntakeRotateLastFlag = gamepad1.triangle;
    }
    private void ArmUpController() throws InterruptedException {
        if(gamepad1.left_bumper&!ArmLastFlag){
            Algorithm.ArmController();
        }
        ArmLastFlag = gamepad1.left_bumper;
    }
    private void AutoScoringController() throws InterruptedException {
        if(gamepad1.dpad_right&&!buildAutoScoring1LastFlag){
            Algorithm.followerReset(follower,GetSpecPosition);
            RebuildMapIsReady = true;
        }
        if(gamepad1.dpad_left&&!buildAutoScoring2LastFlag&&RebuildMapIsReady){
            ScoringInitialPosition = follower.getPose();
            //the flag below is for auto pilot knowing go scoring or get specimen.
            boolean scOrGeFLag = false;
            nextPointDistance=0;
            PathBuilderScoring();
            Algorithm.AutoPilotInitializeHardware();
            while(Algorithm.AutoPilotBreak(gamepad1)){
                if(follower.atParametricEnd()) {
                    if (scOrGeFLag) {
                        Algorithm.ForwardGrabController();
                        Algorithm.BackGrabAction();
                        Thread.sleep(150);
                        follower.followPath(Scoring,true);
                        follower.update();
                        scOrGeFLag = false;
                        Thread.sleep(500);
                        Algorithm.SpinnerToCenter();
                        continue;
                    }
                    Algorithm.BackGrabAction();
                    VisionIntake();
                    follower.followPath(GetSpec,true);
                    follower.update();
                    scOrGeFLag = true;
                    PathBuilderScoring();
                }
            }
            RebuildMapIsReady = false;
            Algorithm.SlideController();
            Algorithm.ArmController();
            follower.startTeleopDrive();
            follower.setTeleOpMovementVectors(Math.pow(-gamepad1.left_stick_y,3), Math.pow(-gamepad1.left_stick_x,3), +gamepad1.left_trigger*gamepad1.left_trigger-gamepad1.right_trigger*gamepad1.right_trigger, false);
            follower.update();
        }

        buildAutoScoring2LastFlag = gamepad1.dpad_left;
        buildAutoScoring1LastFlag = gamepad1.dpad_right;
    }
    private void ClimbController(){
        if (gamepad1.ps){
            Algorithm.ClimbController();
        }
    }
    private void EmergencyInitMotors(){
        if (gamepad1.touchpad) {
            Algorithm.EmergencyMotorPowerSetting();
            EmergencyFlag = true;
        }
        while (EmergencyFlag) {
            if(Algorithm.IsTop(AlgorithmLibrary.Left_Hanging_Motor)&&!Eflag1) {
                if (Emergencytimer1.milliseconds() > 50) {
                    Algorithm.InitializeLift();
                    Eflag1 = true;
                }
            }else{Emergencytimer1.reset();}

            if (Algorithm.IsTop(AlgorithmLibrary.BigArm)&&!Eflag2){
                if(Emergencytimer2.milliseconds() > 50){
                    Algorithm.InitializeArm();
                    Eflag2=true;
                }
            }else{Emergencytimer2.reset();}

            if (Eflag1&&Eflag2){
                EmergencyFlag = false;
                Eflag1 = false;
                Eflag2 = false;
            }
        }
    }
}
//To my lover JSY