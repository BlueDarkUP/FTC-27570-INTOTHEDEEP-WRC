package org.firstinspires.ftc.teamcode.APIuser;

import org.firstinspires.ftc.teamcode.API.ServoKinematics;

public class SlideControl {

    public void moveToExtension(double desiredExtensionCm) {
        double servoAngle = ServoKinematics.getServoRotationDegrees(desiredExtensionCm);
        System.out.println(servoAngle);
    }

    public static void main(String[] args) {
        SlideControl controller = new SlideControl();
        controller.moveToExtension(10.0);
    }
}