package org.firstinspires.ftc.teamcode.pedroPathing.TeleOp.OpmodeTuning;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.pedroPathing.constants.AlgorithmLibrary;
import org.firstinspires.ftc.teamcode.pedroPathing.constants.FConstants;
import org.firstinspires.ftc.teamcode.pedroPathing.constants.LConstants;

/**
 * This is a teleop code mainly for servo tuning.
 *
 * @author Luca Li
 * @version 2025/5
 */

@TeleOp(name = "ServoTuning", group = "Examples")
public class ServoTuning extends OpMode {
    private Follower follower;
    Servo TuningServo;
    AlgorithmLibrary Algorithm;
    private boolean Flag = false;
    private boolean LastFlag = false;
    private double nowPosition = 0;
    private final Pose startPose = new Pose(0,0,0);

    /** This method is call once when init is played, it initializes the follower **/
    @Override
    public void init() {
        follower = new Follower(hardwareMap, FConstants.class, LConstants.class);
        follower.setStartingPose(startPose);
        TuningServo = hardwareMap.get(Servo.class,"back_arm"/*change this to whatever you need*/);
        Algorithm = new AlgorithmLibrary(hardwareMap);
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

        follower.setTeleOpMovementVectors(-gamepad1.left_stick_y, -gamepad1.left_stick_x, -gamepad1.right_stick_x, true);
        follower.update();/*
        if(gamepad1.a&&!LastFlag){
            if(Flag){
                algorithm.BackGrabAction();
                Flag=false;
            }
            else {
                Flag=true;
                algorithm.BackGrabAction();
            }
        }
        LastFlag = gamepad1.a;
        */
        nowPosition+= Math.pow(gamepad1.left_trigger - gamepad1.right_trigger,3);
            TuningServo.setPosition(nowPosition);
        /* Telemetry Outputs of our Follower */
       telemetry.addData("Servo value",nowPosition);
        /* Update Telemetry to the Driver Hub */
        telemetry.update();

    }

    /** We do not use this because everything automatically should disable **/
    @Override
    public void stop() {
    }
}
