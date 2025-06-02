package org.firstinspires.ftc.teamcode.APIuser;

import org.firstinspires.ftc.teamcode.API.ServoKinematics;

public class SlideControl {

    public void moveToExtension(double desiredExtensionCm) {
        double servoAngle = ServoKinematics.getServoRotationDegrees(desiredExtensionCm);

        if (Double.isNaN(servoAngle)) {
            System.out.println("Cannot reach extension: " + desiredExtensionCm + " cm");
            // Handle error: maybe log, or don't move servo
        } else {
            System.out.println("To reach " + desiredExtensionCm + " cm, rotate servo by: " + String.format("%.2f", servoAngle) + " degrees.");
            // Code to send servoAngle to your actual servo motor
            // e.g., myServo.setAngle(servoAngle);
        }
    }

    public static void main(String[] args) {
        SlideControl controller = new SlideControl();

        controller.moveToExtension(1.0);  // Example call
        controller.moveToExtension(10.0);
        controller.moveToExtension(40.0); // Likely unreachable
    }
}