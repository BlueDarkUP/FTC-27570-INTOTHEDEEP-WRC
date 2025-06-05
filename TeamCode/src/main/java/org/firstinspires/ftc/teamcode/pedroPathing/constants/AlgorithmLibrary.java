package org.firstinspires.ftc.teamcode.pedroPathing.constants;


import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.Objects;

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
        back_grab.setPosition(ConstantMap.BackGrab_Initialize);
        BackArm.setPosition(ConstantMap.BACK_ARM_INITIALIZE_POSITION);
        arm_forward.setPosition(ConstantMap.Arm_Forward_Initialize_Position);
        forward_slide.setPosition(ConstantMap.Slide_In_Position);
        forward_claw.setPosition(ConstantMap.ForwardClaw_Initialize_Position);
        intake_rotate.setPosition(ConstantMap.Intake_rotate_Initial_Position);
        intake_spinner.setPosition(ConstantMap.Intake_spinner_Initial_Position);
    }
    public void Initialize_All_For_TeleOp() throws InterruptedException {
        //Initialize motor
        Left_Hanging_Motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Right_Hanging_Motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BigArm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Left_Hanging_Motor.setPower(-0.7);
        Right_Hanging_Motor.setPower(-0.7);
        BigArm.setPower(-0.4);
        Thread.sleep(100);
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
        intake_rotate.setPosition(ConstantMap.Intake_rotate_Initial_Position);
        intake_spinner.setPosition(ConstantMap.Intake_spinner_Initial_Position);
    }
    public void ArmController(String flag){
        if(Objects.equals(flag,"Up")){
            LiftAction(ConstantMap.Lift_Up_HighChamber_Position);
            BigArmMotorAction(ConstantMap.Big_Arm_Set_Position);
            BackArmAction(ConstantMap.BACK_ARM_SET_POSITION);
            return;
        }
        BigArmMotorAction(ConstantMap.Big_Arm_Reset_Position);
        BackArmAction(ConstantMap.BACK_ARM_RESET_POSITION);
        LiftAction(ConstantMap.Lift_Down_Position);
    }

    public void IntakeController(String flag) throws InterruptedException {
        if(Objects.equals(flag,"Take")) {
            arm_forward.setPosition(ConstantMap.Arm_Forward_Down_Position);
            Thread.sleep(60);
            ForwardGrabController("Close");
            Thread.sleep(120);
            arm_forward.setPosition(ConstantMap.Arm_Forward_Up_Position);
            return;
        }
        ForwardGrabController("Open");
    }
    public void SpinnerController(String flag){
        if(Objects.equals(flag,"Put down")){
            intake_spinner.setPosition(ConstantMap.Intake_spinner_PutDown_Position);
            return;
        }
        intake_spinner.setPosition(ConstantMap.Intake_spinner_Initial_Position);
    }
    public void ForwardGrabController(String flag){
        if(Objects.equals(flag, "Open")){
            forward_claw.setPosition(ConstantMap.ForwardClaw_Initialize_Position);
            return;
        }
        forward_claw.setPosition(ConstantMap.ForwardClaw_Tight_Position);
    }
    public void RotateController(String flag){
        if(Objects.equals(flag,"Turn")) {
            intake_rotate.setPosition(ConstantMap.Intake_rotate_Turned_Position);
            return;
        }
        intake_rotate.setPosition(ConstantMap.Intake_rotate_Initial_Position);
    }
    public void BackGrabAction(double Position){
        back_grab.setPosition(Position);
    }
    public void SlideController(String flag){
        if(Objects.equals(flag,"Out")){
            intake_spinner.setPosition(ConstantMap.Intake_spinner_Initial_Position);
            arm_forward.setPosition(ConstantMap.Arm_Forward_Up_Position);
            forward_slide.setPosition(ConstantMap.Slide_Out_Position);
            return;
        }
        arm_forward.setPosition(ConstantMap.Arm_Forward_Initialize_Position);
        forward_slide.setPosition(ConstantMap.Slide_In_Position);
        intake_spinner.setPosition(ConstantMap.Intake_spinner_PutDown_Position);
    }
    public void EmergencyMotorPowerSetting(){
        BigArm.setPower(-0.5);
        Left_Hanging_Motor.setPower(-0.7);
        Right_Hanging_Motor.setPower(-0.7);
    }
    public boolean IsTop(DcMotorEx Motor){
        if(Motor.getCurrentPosition()<MotorLastPosition+3||Motor.getCurrentPosition()>MotorLastPosition-3){
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
                Left_Hanging_Motor.setPower(-1);
                Right_Hanging_Motor.setPower(-1);
            }
        }
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
    private void BackArmAction(double Position){
        BackArm.setPosition(Position);
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