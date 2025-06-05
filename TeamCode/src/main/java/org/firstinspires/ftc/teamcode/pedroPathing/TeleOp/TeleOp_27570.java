package org.firstinspires.ftc.teamcode.pedroPathing.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.util.Constants;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.pedroPathing.constants.ConstantMap;
import org.firstinspires.ftc.teamcode.pedroPathing.constants.AlgorithmLibrary;
import org.firstinspires.ftc.teamcode.pedroPathing.constants.FConstants;
import org.firstinspires.ftc.teamcode.pedroPathing.constants.LConstants;
/**
 * This is the fucking best teleop code in the world.
 * @author Luca Li
 * @version 2025/5
 *
 */

@TeleOp (name = "TeleOp-Code-27570",group = "Competition")

public class TeleOp_27570 extends OpMode {
    private Follower follower;
    private AlgorithmLibrary Algorithm;
    private boolean BackGrabFlag=false,BackGrabLastFlag=false,IntakeRotateFlag=false,IntakeRotateLastFlag=false;
    private boolean ClawFlag=false,ClawLastFlag=false,IntakeSlideFlag = false,IntakeSlideLastFlag = false;
    private boolean ArmFlag = false,ArmLastFlag = false;
    private boolean EmergencyFlag = false;

    boolean Eflag1 = false,Eflag2 = false;
    private ElapsedTime Emergencytimer1 = new ElapsedTime(),Emergencytimer2 = new ElapsedTime();
    private final Pose startPose = new Pose(0,0,0);

    /** This method is call once when init is played, it initializes the follower **/
    @Override
    public void init() {
        follower = new Follower(hardwareMap, FConstants.class, LConstants.class);
        follower.setStartingPose(startPose);
        Algorithm = new AlgorithmLibrary(hardwareMap);
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

        follower.setTeleOpMovementVectors(-gamepad1.left_stick_y, -gamepad1.left_stick_x, +gamepad1.left_trigger-gamepad1.right_trigger, true);
        follower.update();

        BackGrabController();
        try {
            ForwardClawController();
        } catch (InterruptedException e) {
            throw new RuntimeException(e);
        }
        RotateController();
        SlideController();
        EmergencyInitMotors();
        ArmUpController();
        ClimbController();
        /* Telemetry Outputs of our Follower */
        telemetry.addData("X", follower.getPose().getX());
        telemetry.addData("Y", follower.getPose().getY());
        telemetry.addData("Heading in Degrees", Math.toDegrees(follower.getPose().getHeading()));

        /* Update Telemetry to the Driver Hub */
        telemetry.update();

    }

    /** We do not use this because everything automatically should disable **/
    @Override
    public void stop() {
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
    private void ClimbController(){
        if (gamepad1.ps){
            ClimbController();
        }
    }
    private void EmergencyInitMotors(){
        if (gamepad1.touchpad) {
            Algorithm.EmergencyMotorPowerSetting();
            EmergencyFlag = true;
        }
        while (EmergencyFlag) {
            if(Algorithm.IsTop(AlgorithmLibrary.Left_Hanging_Motor)&&!Eflag1) {
                if (Emergencytimer1.milliseconds() > 110) {
                    Algorithm.InitializeLift();
                    Eflag1 = true;
                }
            }else{Emergencytimer1.reset();}

            if (Algorithm.IsTop(AlgorithmLibrary.BigArm)&&!Eflag2){
                if(Emergencytimer2.milliseconds() > 110){
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