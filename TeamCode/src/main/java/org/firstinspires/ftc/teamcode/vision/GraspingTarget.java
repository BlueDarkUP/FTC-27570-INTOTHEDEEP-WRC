// Filename: GraspingTarget.java
package org.firstinspires.ftc.teamcode.vision;

/**
 * 存储最终计算出的抓取目标舵机位置的数据类。
 */
public class GraspingTarget {

    public final double sliderServoPosition;
    public final double turnServoPosition;
    public final double rotateServoPosition;
    public final boolean isInRange;

    /**
     * 构造函数
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