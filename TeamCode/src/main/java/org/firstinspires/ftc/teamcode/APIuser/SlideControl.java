package org.firstinspires.ftc.teamcode.APIuser;

import org.firstinspires.ftc.teamcode.API.ServoKinematics;
import org.firstinspires.ftc.teamcode.API.ServoKinematics.ServoTarget;

public class SlideControl {
    public void moveToExtension(double desiredExtensionCm) {
        System.out.printf("请求滑轨移动到: %.2f cm\n", desiredExtensionCm);
        ServoTarget target = ServoKinematics.calculateServoTarget(desiredExtensionCm);

        if (target != null) {
            double servoPosition = target.servoPosition;

            System.out.printf(">> 计算出的舵机位置为: %.4f\n", servoPosition);

        } else {
            // 处理无效输入的情况
            System.out.println(">> 无法计算舵机位置。请检查目标距离是否在有效范围 [0, 24] cm 内。");
        }
    }

    public static void main(String[] args) {
        SlideControl controller = new SlideControl();
        controller.moveToExtension(12.0);
        System.out.println("------------------------------------");
    }
}