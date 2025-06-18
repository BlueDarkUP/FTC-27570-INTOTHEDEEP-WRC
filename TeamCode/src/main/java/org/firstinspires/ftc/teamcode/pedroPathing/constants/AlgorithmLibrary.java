package org.firstinspires.ftc.teamcode.pedroPathing.constants;


import androidx.annotation.NonNull;

import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * 优雅永不过时
 * @author LucaLi
 * @version 2025/5
 */
public class AlgorithmLibrary {

    public ElapsedTime EmergencyInitTimer = new ElapsedTime();
    public static DcMotorEx Left_Hanging_Motor = null;
    public static DcMotorEx Right_Hanging_Motor = null;
    public static DcMotorEx BigArm = null;
    public static Servo BackArm,back_grab,forward_claw,intake_rotate,camera_arm,arm_forward,forward_slide,intake_spinner;

    public  boolean BackGrabFlag=false,IntakeRotateFlag=false;
    public  boolean ClawFlag=false,IntakeSlideFlag = false;
    public  boolean ArmFlag = false,CameraArmFlag = false;
    public  boolean SpinnerFlag = false;
    public boolean EmergencyFlag = false;
    public static boolean VLflag = false;
    private int MotorLastPosition;

    public AlgorithmLibrary(HardwareMap hardwareMap){
        //Get hardware
        Left_Hanging_Motor = hardwareMap.get(DcMotorEx.class, "LeftHangingMotor");
        Right_Hanging_Motor = hardwareMap.get(DcMotorEx.class, "RightHangingMotor");
        BigArm = hardwareMap.get(DcMotorEx.class,"big_arm");
        arm_forward = hardwareMap.get(Servo.class,"arm_forward");
        forward_slide = hardwareMap.get(Servo.class,"forward_slide");
        BackArm = hardwareMap.get(Servo.class,"back_arm");
        forward_claw =hardwareMap.get(Servo.class,"forward_claw");
        intake_rotate= hardwareMap.get(Servo.class,"intake_rotate");
        back_grab = hardwareMap.get(Servo.class,"backgrab");
        intake_spinner = hardwareMap.get(Servo.class,"rotate_platform");
        camera_arm = hardwareMap.get(Servo.class, "camera_arm");

        //Set motor direction
        Left_Hanging_Motor.setDirection(DcMotorSimple.Direction.REVERSE);
        Right_Hanging_Motor.setDirection(DcMotorSimple.Direction.FORWARD);
        BigArm.setDirection(DcMotorSimple.Direction.REVERSE);
        arm_forward.setDirection(Servo.Direction.REVERSE);
        forward_slide.setDirection(Servo.Direction.REVERSE);
    }
    public void InitializeLift(){
        Left_Hanging_Motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Right_Hanging_Motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        Left_Hanging_Motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Right_Hanging_Motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Left_Hanging_Motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Right_Hanging_Motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
    public void InitializeArm(){
        BigArm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BigArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BigArm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

    }
    public void Initialize_All_For_Autonomous(){
        Left_Hanging_Motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Right_Hanging_Motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BigArm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Left_Hanging_Motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Right_Hanging_Motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BigArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Left_Hanging_Motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Right_Hanging_Motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BigArm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //Set servo position
        back_grab.setPosition(ConstantMap.BackGrab_TightPosition);
        BackArm.setPosition(ConstantMap.BACK_ARM_INITIALIZE_POSITION);
        arm_forward.setPosition(ConstantMap.Arm_Forward_Initialize_Position);
        forward_slide.setPosition(ConstantMap.Slide_In_Position);
        forward_claw.setPosition(ConstantMap.ForwardClaw_Initialize_Position);
        intake_rotate.setPosition(ConstantMap.Intake_rotate_Turned_Position);
        intake_spinner.setPosition(ConstantMap.Intake_spinner_Initial_Position);
        camera_arm.setPosition(ConstantMap.Camera_Arm_Initialize_Position);
    }
    public void Initialize_All_For_TeleOp() throws InterruptedException {
        //Initialize motor
        Left_Hanging_Motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Right_Hanging_Motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BigArm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Left_Hanging_Motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Right_Hanging_Motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BigArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Left_Hanging_Motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Right_Hanging_Motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BigArm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //Set servo position
        back_grab.setPosition(ConstantMap.BackGrab_Initialize);
        BackArm.setPosition(ConstantMap.BACK_ARM_INITIALIZE_POSITION);
        arm_forward.setPosition(ConstantMap.Arm_Forward_Initialize_Position);
        forward_slide.setPosition(ConstantMap.Slide_In_Position);
        forward_claw.setPosition(ConstantMap.ForwardClaw_Initialize_Position);
        intake_rotate.setPosition(ConstantMap.Intake_rotate_Turned_Position);
        intake_spinner.setPosition(ConstantMap.Intake_spinner_Initial_Position);
        camera_arm.setPosition(ConstantMap.Camera_Arm_Initialize_Position);
    }

    public void Initialize_All_For_Vision() throws InterruptedException {
        arm_forward.setPosition(ConstantMap.Arm_Forward_Initialize_Position);
        forward_slide.setPosition(ConstantMap.Slide_In_Position);
        forward_claw.setPosition(ConstantMap.ForwardClaw_Initialize_Position);
        intake_rotate.setPosition(ConstantMap.Intake_rotate_Turned_Position);
        intake_spinner.setPosition(ConstantMap.Intake_spinner_Initial_Position);
        camera_arm.setPosition(ConstantMap.Camera_Arm_PutDown_Position);
    }

    public void AutoPilotInitializeHardware() throws InterruptedException {
        BackGrabFlag=true;
        ClawFlag=true;
        IntakeSlideFlag=true;
        SlideController();
        ForwardGrabController();
        BackGrabAction();
    }
    public void SpinnerToCenter(){
        intake_spinner.setPosition(ConstantMap.Intake_spinner_Initial_Position);
        arm_forward.setPosition(ConstantMap.Arm_Forward_Initialize_Position);
        intake_rotate.setPosition(ConstantMap.Intake_rotate_Turned_Position);
        SpinnerFlag=false;
    }
    public void performVisionGrasp(double sliderServoPosition, double turnServoPosition, double rotateServoPosition) throws InterruptedException {
        forward_slide.setPosition(sliderServoPosition);
        intake_spinner.setPosition(turnServoPosition);
        Thread.sleep((long)(200*sliderServoPosition));
        arm_forward.setPosition(ConstantMap.Arm_Forward_Down_Position);
        Thread.sleep(120);
        intake_rotate.setPosition(rotateServoPosition);
        Thread.sleep(150);
        forward_claw.setPosition(ConstantMap.ForwardClaw_Tight_Position);
        Thread.sleep(200);
        ClawFlag = true;
        IntakeSlideFlag=true;
        /*
        arm_forward.setPosition(ConstantMap.Arm_Forward_Initialize_Position);
        forward_slide.setPosition(ConstantMap.Slide_In_Position);
        intake_rotate.setPosition(0.62); // Or a constant
        intake_spinner.setPosition(ConstantMap.Intake_spinner_Initial_Position);
        camera_arm.setPosition(0.113); // Or a constant*/
    }

    public void ArmController() throws InterruptedException {
        if(ArmFlag){
            /*
            BigArmMotorAction(ConstantMap.Big_Arm_Reset_Position);
            BackArm.setPosition(ConstantMap.BACK_ARM_RESET_POSITION);
            LiftAction(ConstantMap.Lift_Down_Position);
             */
            ArmFlag=false;
            return;
        }
        /*if(!CameraArmFlag){
            CameraArmController();
            Thread.sleep(100);
        }/*
        LiftAction(ConstantMap.Lift_Up_HighChamber_Position);
        BigArmMotorAction(ConstantMap.Big_Arm_Set_Position);
        BackArm.setPosition(ConstantMap.BACK_ARM_SET_POSITION);*/
        ArmFlag=true;
    }
    public void CameraArmController(){
        if(CameraArmFlag) {
            camera_arm.setPosition(ConstantMap.Camera_Arm_Initialize_Position);
            CameraArmFlag=false;
            return;
        }
        camera_arm.setPosition(ConstantMap.Camera_Arm_PutDown_Position);
        CameraArmFlag=true;
    }
    public void IntakeController() throws InterruptedException {
        if(ClawFlag) {
            ForwardGrabController();
            return;
        }
        arm_forward.setPosition(ConstantMap.Arm_Forward_Down_Position);
        Thread.sleep(60);
        ForwardGrabController();
        Thread.sleep(110);
        arm_forward.setPosition(ConstantMap.Arm_Forward_Up_Position);
    }
    public void SpinnerController(){
        if(SpinnerFlag){
            intake_spinner.setPosition(ConstantMap.Intake_spinner_Initial_Position);
            SpinnerFlag=false;
            return;
        }
        intake_spinner.setPosition(ConstantMap.Intake_spinner_PutDown_Position);
        SpinnerFlag=true;
    }
    public void ForwardGrabController() throws InterruptedException {
        if(ClawFlag){
            forward_claw.setPosition(ConstantMap.ForwardClaw_Initialize_Position);
            ClawFlag=false;
            return;
        }
        forward_claw.setPosition(ConstantMap.ForwardClaw_Tight_Position);
        ClawFlag=true;
    }
    public void RotateController(){
        if(IntakeRotateFlag) {
            intake_rotate.setPosition(ConstantMap.Intake_rotate_Initial_Position);
            IntakeRotateFlag=false;
            return;
        }
        IntakeRotateFlag=true;
        intake_rotate.setPosition(ConstantMap.Intake_rotate_Turned_Position);
    }
    public void BackGrabAction(){
        if(BackGrabFlag) {
            back_grab.setPosition(ConstantMap.BackGrab_Initialize);
            BackGrabFlag=false;
            return;
        }
        back_grab.setPosition(ConstantMap.BackGrab_TightPosition);
        BackGrabFlag=true;
    }
    public void SlideController() throws InterruptedException {
        if(IntakeSlideFlag){
            arm_forward.setPosition(ConstantMap.Arm_Forward_Putdown_Position);
            forward_slide.setPosition(ConstantMap.Slide_In_Position);
            IntakeRotateFlag=true;
            RotateController();
            IntakeSlideFlag=false;
            Thread.sleep((int)(300*forward_slide.getPosition()));
            SpinnerFlag=false;
            SpinnerController();
            return;
        }
        SpinnerFlag=true;
        SpinnerController();
        arm_forward.setPosition(ConstantMap.Arm_Forward_Up_Position);
        forward_slide.setPosition(ConstantMap.Slide_Out_Position);
        IntakeSlideFlag=true;
    }
    public void EmergencyMotorPowerSetting(){
        BigArm.setPower(-0.5);
        Left_Hanging_Motor.setPower(-0.7);
        Right_Hanging_Motor.setPower(-0.7);
    }
    public boolean IsTop(@NonNull DcMotorEx Motor){
        if(Motor.getCurrentPosition()<MotorLastPosition+1||Motor.getCurrentPosition()>MotorLastPosition-1){
            MotorLastPosition = Motor.getCurrentPosition();
            return true;
        }
        MotorLastPosition = Motor.getCurrentPosition();
        return false;
    }

    public void ClimbController(){
        LiftAction(ConstantMap.Lift_Up_Climb_Position);
        while(true){
            if(!Left_Hanging_Motor.isBusy()&&!Right_Hanging_Motor.isBusy()){
                Left_Hanging_Motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                Right_Hanging_Motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                Left_Hanging_Motor.setPower(-1);
                Right_Hanging_Motor.setPower(-1);
            }
        }
    }
    public boolean AutoPilotBreak(Gamepad gamepad){
        return gamepad.left_stick_x==0&& gamepad.left_stick_y==0;

    }
    public void followerReset(Follower follower, Pose SetPose){
        follower.breakFollowing();
        follower.setPose(SetPose);
    }
    private void LiftAction(int Position){
        //Set target position
        Left_Hanging_Motor.setTargetPosition(Position);
        Right_Hanging_Motor.setTargetPosition(Position);

        //Set mode
        Left_Hanging_Motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Right_Hanging_Motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        //Set Power
        Left_Hanging_Motor.setPower(1);
        Right_Hanging_Motor.setPower(1);
    }
    private void BigArmMotorAction(int Position){
        //Set Target Position
        BigArm.setTargetPosition(Position);

        //Set Mode
        BigArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        if(Position == ConstantMap.Big_Arm_Set_Position){
            BigArm.setPower(ConstantMap.Big_Arm_Up_Power);
            return;
        }
        //Set Power
        BigArm.setPower(ConstantMap.Big_Arm_Down_Power);
    }
}