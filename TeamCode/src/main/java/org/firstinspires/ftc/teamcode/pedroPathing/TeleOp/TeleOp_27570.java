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
import org.firstinspires.ftc.teamcode.vision.Data.GraspingTarget;
import org.firstinspires.ftc.teamcode.vision.Interface.VisionGraspingAPI;
import org.firstinspires.ftc.teamcode.vision.CoreCalcu.VisionGraspingCalculator;
import org.firstinspires.ftc.teamcode.vision.Data.VisionTargetResult;

/**
 * This is the fucking best teleop code in the world.
 * @author Luca Li
 * @version 2025/5
 *(L2 autopilot connected!
 */

@TeleOp (name = "TeleOp-Code-27570",group = "Competition")

public class TeleOp_27570 extends OpMode {
    private static Follower follower;
    private AlgorithmLibrary Algorithm;
    private VisionGraspingAPI visionAPI;
    private boolean BackGrabFlag=false,BackGrabLastFlag=false,IntakeRotateFlag=false,IntakeRotateLastFlag=false;
    private boolean ClawFlag=false,ClawLastFlag=false,IntakeSlideFlag = false,IntakeSlideLastFlag = false;
    private boolean ArmFlag = false,ArmLastFlag = false,CameraArmFlag = false,CameraArmLastFlag = false;
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
        visionAPI.init(hardwareMap);
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

        follower.setTeleOpMovementVectors(-gamepad1.left_stick_y, -gamepad1.left_stick_x, +gamepad1.left_trigger*gamepad1.left_trigger-gamepad1.right_trigger*gamepad1.right_trigger, true);
        follower.update();

        PoseNow = follower.getPose();
        BackGrabController();
        RotateController();
        SlideController();
        EmergencyInitMotors();
        ArmUpController();
        CameraArmController();
        ClimbController();
        try {
            VisionController();
            ForwardClawController();
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
        VisionTargetResult result = visionAPI.getLatestResult();
        if (result.isTargetFound) {
            GraspingTarget graspTarget = VisionGraspingCalculator.calculate(result, telemetry);
            if(gamepad1.right_stick_button&&!VLflag) {
                if(!graspTarget.isInRange) {
                    nextPointDistance = result.nextTargetHorizontalOffsetCm * ConstantMap.CM_TO_INCH;
                    PathBuilderIntake();
                    follower.followPath(ToIntake);
                    follower.update();
                    Algorithm.BackGrabAction(ConstantMap.BackGrab_Initialize);
                    BackGrabFlag=false;
                    Algorithm.ForwardGrabController("Open");
                    while (follower.isBusy()) {
                        if(!Algorithm.AutoPilotBreak(gamepad1)){
                            Algorithm.followerReset(follower,follower.getPose());
                            break;
                        }
                    }
                }
                VisionIntake();
            }
            VLflag = gamepad1.right_stick_button;
        }
    }
    private void VisionIntake() throws InterruptedException {
        VisionTargetResult result = visionAPI.getLatestResult();
        if (result.isTargetFound) {
            GraspingTarget graspTarget = VisionGraspingCalculator.calculate(result, telemetry);
            if(graspTarget.isInRange){
                Algorithm.BackGrabAction(ConstantMap.BackGrab_Initialize);
                Algorithm.ForwardGrabController("Open");
                BackGrabFlag = false;
                Algorithm.performVisionGrasp(graspTarget.sliderServoPosition, graspTarget.turnServoPosition, graspTarget.rotateServoPosition);
                Algorithm.SlideController("Back");
                IntakeSlideFlag = false;
            }
        }
    }
    private void CameraArmController(){
        if(gamepad1.dpad_down&&!CameraArmLastFlag){
            if(CameraArmFlag){
                Algorithm.CameraArmController(ConstantMap.Camera_Arm_PutDown_Position);
                CameraArmFlag = false;
            }
            else {
                CameraArmFlag = true;
                Algorithm.BackGrabAction(ConstantMap.Camera_Arm_Initialize_Position);
            }
        }

        CameraArmLastFlag = gamepad1.dpad_down;
    }
    private void BackGrabController(){
        if(gamepad1.square&&!BackGrabLastFlag){
            if(BackGrabFlag){
                Algorithm.BackGrabAction(ConstantMap.BackGrab_Initialize);
                BackGrabFlag = false;
            }
            else {
                BackGrabFlag = true;
                Algorithm.BackGrabAction(ConstantMap.BackGrab_TightPosition);
            }
        }

        BackGrabLastFlag = gamepad1.square;
    }
    private void ForwardClawController() throws InterruptedException {
        if(gamepad1.cross&!ClawLastFlag){
            if(ClawFlag){
                Algorithm.IntakeController("Put down");
                ClawFlag = false;
            }else {
                Algorithm.IntakeController("Take");
                IntakeSlideFlag=true;
                ClawFlag = true;
            }
        }
        ClawLastFlag = gamepad1.cross;
    }
    private void SlideController(){
        if(gamepad1.right_bumper&!IntakeSlideLastFlag){
            if(IntakeSlideFlag){
                Algorithm.SlideController("Back");
                IntakeSlideFlag = false;
            }else {
                Algorithm.SlideController("Out");
                IntakeSlideFlag = true;
            }
        }
        IntakeSlideLastFlag = gamepad1.right_bumper;
    }
    private void RotateController(){
        if(gamepad1.triangle&!IntakeRotateLastFlag){
            if(IntakeRotateFlag){
                Algorithm.RotateController("Back");
                IntakeRotateFlag = false;
            }else {
                Algorithm.RotateController("Turn");
                IntakeRotateFlag = true;
            }
        }
        IntakeRotateLastFlag = gamepad1.triangle;
    }
    private void ArmUpController(){
        if(gamepad1.left_bumper&!ArmLastFlag){
            if(ArmFlag){
                Algorithm.ArmController("Down");
                ArmFlag = false;
            }else {
                Algorithm.ArmController("Up");
                ArmFlag = true;
            }
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
            while(Algorithm.AutoPilotBreak(gamepad1)){
                if(!follower.isBusy()) {
                    if (scOrGeFLag) {
                        Algorithm.ForwardGrabController("Open");
                        Algorithm.BackGrabAction(ConstantMap.BackGrab_TightPosition);
                        BackGrabFlag=true;
                        Thread.sleep(150);
                        follower.followPath(Scoring);
                        follower.update();
                        scOrGeFLag = false;
                        continue;
                    }
                    Algorithm.BackGrabAction(ConstantMap.BackGrab_Initialize);
                    BackGrabFlag=false;
                    VisionIntake();
                    follower.followPath(GetSpec);
                    follower.update();
                    scOrGeFLag = true;
                    PathBuilderScoring();
                }
            }
            RebuildMapIsReady = false;
            Algorithm.SlideController("Back");
            Algorithm.ArmController("Down");
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