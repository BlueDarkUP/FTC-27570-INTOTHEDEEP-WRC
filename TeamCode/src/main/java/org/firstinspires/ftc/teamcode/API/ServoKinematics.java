/**
 * 用于解算连杆，期望伸长距离返回曲柄角度和相应舵机位置
 * @author BlueDarkUP
 * @version 2025/6
 *
 */
package org.firstinspires.ftc.teamcode.API;

import java.lang.Math;

public class ServoKinematics {

    // --- 几何常量 (单位: 厘米) ---
    private static final double R_CRANK_LENGTH_CM = 11.3; // 曲柄AB长度
    private static final double L_CONNECTING_ROD_LENGTH_CM = 12.0; // 连杆BC长度
    private static final double H_EFFECTIVE_VERTICAL_OFFSET_CM = 8.7; // C点相对A点的垂直偏移
    private static final double X_C_RETRACTED_RELATIVE_TO_A_CM = -17; // 完全收回时C点的X坐标

    private static final double MAX_EXTENSION_CM = 24.0;  //滑轨从完全收回到完全伸出的最大物理行程 (cm)

    private static final double SERVO_POSITION_AT_MIN_EXTENSION = 0.0;  //当滑轨在最小伸出位置(0cm)时，对应的舵机逻辑位置。

    private static final double SERVO_POSITION_AT_MAX_EXTENSION = 0.9;  //当滑轨在最大伸出位置时，对应的舵机逻辑位置。

    public static class ServoTarget {
        public final double rotationDegrees; // 从初始位置需要旋转的角度 (0-360°)
        public final double servoPosition;   // 舵机目标位置 (例如 0.0 到 1.0)

        public ServoTarget(double rotationDegrees, double servoPosition) {
            this.rotationDegrees = rotationDegrees;
            this.servoPosition = servoPosition;
        }

        @Override
        public String toString() {
            // 输出舵机位置时保留4位小数以提高精度
            return String.format("ServoTarget[旋转角度: %.2f°, 舵机位置: %.4f]", rotationDegrees, servoPosition);
        }
    }


    private static double calculateCrankAngleRad(double xCTargetAbsCm) {
        double dAc = Math.sqrt(Math.pow(xCTargetAbsCm, 2) + Math.pow(H_EFFECTIVE_VERTICAL_OFFSET_CM, 2));
        if (dAc > (R_CRANK_LENGTH_CM + L_CONNECTING_ROD_LENGTH_CM) || dAc < Math.abs(L_CONNECTING_ROD_LENGTH_CM - R_CRANK_LENGTH_CM)) {
            return Double.NaN;
        }
        double alpha = Math.atan2(-H_EFFECTIVE_VERTICAL_OFFSET_CM, xCTargetAbsCm);
        double cosBetaArgNumerator = Math.pow(R_CRANK_LENGTH_CM, 2) + Math.pow(dAc, 2) - Math.pow(L_CONNECTING_ROD_LENGTH_CM, 2);
        double cosBetaArgDenominator = 2 * R_CRANK_LENGTH_CM * dAc;
        if (cosBetaArgDenominator == 0) return Double.NaN;
        double cosBetaArg = Math.max(-1.0, Math.min(1.0, cosBetaArgNumerator / cosBetaArgDenominator));
        double beta = Math.acos(cosBetaArg);
        return alpha + beta;
    }


    public static ServoTarget calculateServoTarget(double targetExtensionCm) {
        if (targetExtensionCm < 0 || targetExtensionCm > MAX_EXTENSION_CM) {
            System.err.printf("错误: 目标伸出距离 %.2f cm 超出有效范围 [0.0, %.2f] cm。\n", targetExtensionCm, MAX_EXTENSION_CM);
            return null;
        }

        double initialAngleRad = calculateCrankAngleRad(X_C_RETRACTED_RELATIVE_TO_A_CM);
        if (Double.isNaN(initialAngleRad)) {
            System.err.println("错误: 无法计算初始收回位置的角度。请检查几何常量。");
            return null;
        }

        double maxExtensionAngleRad = calculateCrankAngleRad(X_C_RETRACTED_RELATIVE_TO_A_CM + MAX_EXTENSION_CM);
        if (Double.isNaN(maxExtensionAngleRad)) {
            System.err.printf("错误: 最大伸出位置 (%.2f cm) 几何上不可达。\n", MAX_EXTENSION_CM);
            return null;
        }

        double targetAngleRad = calculateCrankAngleRad(X_C_RETRACTED_RELATIVE_TO_A_CM + targetExtensionCm);
        if (Double.isNaN(targetAngleRad)) {
            System.err.printf("错误: 目标伸出距离 %.2f cm 几何上不可达。\n", targetExtensionCm);
            return null;
        }

        // --- 舵机旋转角度计算 ---
        double rotationRequiredRad = targetAngleRad - initialAngleRad;
        double rotationRequiredDeg = Math.toDegrees(rotationRequiredRad);
        double normalizedRotationDeg = (rotationRequiredDeg % 360 + 360) % 360;

        // 舵机位置映射计算
        // 1. 计算总的可用角度行程 (从0cm到24cm)
        double totalAngleRangeRad = maxExtensionAngleRad - initialAngleRad;
        if (Math.abs(totalAngleRangeRad) < 1e-9) {
            System.err.println("错误: 机构的总行程角度范围为零，无法计算舵机位置。");
            return null;
        }

        // 2. 计算当前目标占总角度行程的比例 (0.0 到 1.0)
        double travelFraction = rotationRequiredRad / totalAngleRangeRad;

        // 3. 将此比例线性映射到设定的舵机位置范围 [0.0, 0.9]
        double servoPositionRange = SERVO_POSITION_AT_MAX_EXTENSION - SERVO_POSITION_AT_MIN_EXTENSION;
        double servoPosition = SERVO_POSITION_AT_MIN_EXTENSION + (travelFraction * servoPositionRange);

        return new ServoTarget(normalizedRotationDeg, servoPosition);
    }

    public static void main(String[] args) {
        System.out.println("--- 舵机运动学计算测试 ---");
        System.out.printf("设定: 滑轨行程 0.0cm -> 舵机位置 %.2f\n", SERVO_POSITION_AT_MIN_EXTENSION);
        System.out.printf("设定: 滑轨行程 %.1fcm -> 舵机位置 %.2f\n", MAX_EXTENSION_CM, SERVO_POSITION_AT_MAX_EXTENSION);
        System.out.println("--------------------------");

        double[] testExtensions = {0.0, 1.0, 12.0, 24.0, -1.0};

        for (double ext : testExtensions) {
            System.out.printf("\n计算目标伸出: %.2f cm...\n", ext);
            ServoTarget target = calculateServoTarget(ext);

            if (target != null) {
                System.out.println("结果 -> " + target);
            } else {
                System.out.println("结果 -> 计算失败或输入无效。");
            }
        }
    }
}