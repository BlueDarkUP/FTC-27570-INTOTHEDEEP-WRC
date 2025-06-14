/**
 * 用于根据给定的参数和公式计算一个位置值，并将结果归一化到 [0, 1) 范围。
 * @author BlueDarkUP
 * @version 2025/6
 */
package org.firstinspires.ftc.teamcode.API;
public class PositionCalculator {

    /**
     * @param positionMin   位置的最小值。
     * @param positionMax   位置的最大值。
     * @param rotateAngle   旋转角度。
     * @param reverse       如果为 true，则应用反向公式；否则应用标准公式。
     * @param userInput     用户输入值。
     * @return              计算得到的位置值，归一化到 [0, 1) 范围。
     */
    public static double calculatePositionValue(
            double positionMin,
            double positionMax,
            double rotateAngle,
            boolean reverse,
            double userInput
    ) {
        if (rotateAngle == 0) {
            throw new IllegalArgumentException("Rotate_angle cannot be zero to avoid division by zero.");
        }
        double commonTerm = ((positionMax - positionMin) / rotateAngle) * userInput;
        double rawResult;
        if (reverse) {
            rawResult = 1.0 - commonTerm;
        } else {
            rawResult = commonTerm + positionMin;
        }
        double normalizedResult = rawResult % 1.0;
        if (normalizedResult < 0) {
            normalizedResult += 1.0;
        }
        return normalizedResult;
    }
}