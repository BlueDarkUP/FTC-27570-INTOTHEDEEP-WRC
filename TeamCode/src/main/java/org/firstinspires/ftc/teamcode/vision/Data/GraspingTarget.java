package org.firstinspires.ftc.teamcode.vision.Data;

/**
 * 存储最终计算出的抓取目标舵机位置的数据类
 * 这是从“感知”到“执行”的关键桥梁，是 VisionGraspingCalculator 的输出
 * 同样，这是一个不可变对象，以确保数据在传递过程中的一致性
 * @author BlueDarkUP
 * @version 2025/6
 * To My Lover - Zyy
 */
public class GraspingTarget {

    public final double sliderServoPosition;
    public final double turnServoPosition;
    public final double rotateServoPosition;
    public final boolean isInRange;

    /**
     * 构造函数。
     * @param sliderServoPosition 滑轨舵机的位置
     * @param turnServoPosition   转向舵机的位置
     * @param rotateServoPosition 旋转舵机的位置
     * @param isInRange           计算出的目标是否在有效范围内
     */
    public GraspingTarget(double sliderServoPosition, double turnServoPosition, double rotateServoPosition, boolean isInRange) {
        this.sliderServoPosition = sliderServoPosition;
        this.turnServoPosition = turnServoPosition;
        this.rotateServoPosition = rotateServoPosition;
        this.isInRange = isInRange;
    }
}