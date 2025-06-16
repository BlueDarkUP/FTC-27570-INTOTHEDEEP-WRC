package org.firstinspires.ftc.teamcode.pedroPathing.constants;

import org.firstinspires.ftc.teamcode.API.PositionCalculator;
/**
 * 用常量爽飞了
 * @author LucaLi
 * @version 2025/5
 */
public class ConstantMap {
    public static final double ScorePoseX = 39;
    public static final double ScorePoseY_LeftTop = 76.5;
    public static final double AutoIntakeX_LeftTop = 65;
    public static final double AutoIntakePoseY = 48;
    public static final double AutoIntakeControlPoseY = 20;
    public static final double IntakeToGetControlPoseY = 30;
    public static final double CM_TO_INCH = 0.393700787;
    public static final int SleepMSAfterScoring = 600;
    //Back grab position
    public static final double BackGrab_Initialize = 0;
    public static final double BackGrab_TightPosition = 0.6;
    public static final double BackGrab_LaxPosition = 0.6;

    //Big arm
    public static final int Big_Arm_Set_Position = 387;
    public static final int Big_Arm_Reset_Position = 0;
    public static final double Big_Arm_Up_Power = 0.9;
    public static final double Big_Arm_Down_Power = 0.6;

    //Back arm position
    public static final double BACK_ARM_SET_POSITION =  0.85;
    public static final double BACK_ARM_RESET_POSITION = 0.15;
    public static final double BACK_ARM_INITIALIZE_POSITION = 0.06;
    //Lift position
    public static final int Lift_Up_HighChamber_Position = 730;
    public static final int Lift_Up_Climb_Position = 1600;
    public static final int Lift_Down_Position = 0;

    //Slide position
    public static final double Slide_In_Position  = 0;
    public static final double Slide_Out_Position = 0.9;
    //Arm forward position
    public static final double Arm_Forward_Initialize_Position = 0.9;
    public static final double Arm_Forward_Putdown_Position = 0.73;
    public static final double Arm_Forward_Up_Position = 0.34;
    public static final double Arm_Forward_Down_Position = 0.19;
    //Forward claw position
    public static final double ForwardClaw_Tight_Position = 0.5;
    public static final double ForwardClaw_Lax_Position = 0.37;
    public static final double ForwardClaw_Initialize_Position = 0;
    //Intake rotation
    public static final double Intake_rotate_Initial_Position = 0.07;
    public static final double Intake_rotate_Turned_Position = 0.62;
    public static final double Intake_spinner_Initial_Position = 0.53;//PositionCalculator.calculatePositionValue(0.53, 1.00, 90, true, 90);
    public static final double Intake_spinner_PutDown_Position = 0.1;

    public static final double Camera_Arm_Initialize_Position = 0.763;
    public static final double Camera_Arm_PutDown_Position = 0.10;
}
