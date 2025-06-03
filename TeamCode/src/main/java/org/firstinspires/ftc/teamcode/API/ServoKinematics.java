package org.firstinspires.ftc.teamcode.API;

import java.lang.Math;

public class ServoKinematics {

    private static final double R_CRANK_LENGTH_CM = 12.85;
    private static final double L_CONNECTING_ROD_LENGTH_CM = 12.0;
    private static final double H_EFFECTIVE_VERTICAL_OFFSET_CM = 8.5;
    private static final double X_C_RETRACTED_RELATIVE_TO_A_CM = -9.5;

    /**
     * 计算C点（连杆与滑轨连接点）的绝对水平X坐标（相对于A点）
     * 所需的曲柄AB旋转角度（弧度制）。
     * A点被设为坐标系原点 (0,0)。C点的Y坐标为 -H_EFFECTIVE_VERTICAL_OFFSET_CM。
     *
     * @param xCTargetAbsCm C点相对于A点的目标X坐标 (厘米)。
     * @return 曲柄AB的角度（弧度制）。返回 Double.NaN 如果目标位置在几何上不可达。
     */
    private static double calculateCrankAngleRad(double xCTargetAbsCm) {
        // 计算A点到C点的直线距离
        double dAc = Math.sqrt(Math.pow(xCTargetAbsCm, 2) + Math.pow(H_EFFECTIVE_VERTICAL_OFFSET_CM, 2));

        // 可达？
        if (dAc > (R_CRANK_LENGTH_CM + L_CONNECTING_ROD_LENGTH_CM) ||
                dAc < Math.abs(L_CONNECTING_ROD_LENGTH_CM - R_CRANK_LENGTH_CM)) {
            return Double.NaN;
        }

        // 计算直线AC与正X轴的夹角
        double alpha = Math.atan2(-H_EFFECTIVE_VERTICAL_OFFSET_CM, xCTargetAbsCm);

        // 在三角形ABC中，使用余弦定理计算角BAC
        double cosBetaArgNumerator = Math.pow(R_CRANK_LENGTH_CM, 2) + Math.pow(dAc, 2) - Math.pow(L_CONNECTING_ROD_LENGTH_CM, 2);
        double cosBetaArgDenominator = 2 * R_CRANK_LENGTH_CM * dAc;

        if (cosBetaArgDenominator == 0) {
            return Double.NaN;
        }
        double cosBetaArg = cosBetaArgNumerator / cosBetaArgDenominator;

        // 防止浮点误差
        cosBetaArg = Math.max(-1.0, Math.min(1.0, cosBetaArg));
        double beta = Math.acos(cosBetaArg);

        // 确定曲柄AB的角度 (theta)。
        double theta = alpha + beta;

        return theta;
    }

    /**
     * 计算从初始收回位置移动到目标伸出距离所需的舵机旋转角度。
     *
     * @param targetExtensionCm 目标伸出距离 (从完全收回0cm算起，单位: 厘米)。
     * @return 舵机需要从初始位置逆时针旋转的角度 (0-360度)。
     *         返回 Double.NaN 如果初始位置或目标位置不可达。
     */
    public static double getServoRotationDegrees(double targetExtensionCm) {
        // 计算曲柄初始角度
        double initialCrankAngleRad = calculateCrankAngleRad(X_C_RETRACTED_RELATIVE_TO_A_CM);

        if (Double.isNaN(initialCrankAngleRad)) {
            System.err.println("错误: 无法计算初始收回位置的舵机角度。请检查常量设置。");
            return Double.NaN;
        }

        // 转换为绝对X坐标
        double xCTargetAbsCm = X_C_RETRACTED_RELATIVE_TO_A_CM + targetExtensionCm;

        // 计算所需曲柄角度
        double targetCrankAngleRad = calculateCrankAngleRad(xCTargetAbsCm);

        if (Double.isNaN(targetCrankAngleRad)) {
            System.err.printf("错误: 目标伸出距离 %.2f cm 几何上不可达。\n", targetExtensionCm);
            return Double.NaN;
        }

        // 计算舵机角度
        double rotationRequiredRad = targetCrankAngleRad - initialCrankAngleRad;
        double rotationRequiredDeg = Math.toDegrees(rotationRequiredRad);

        // 归一化
        rotationRequiredDeg = (rotationRequiredDeg % 360 + 360) % 360;

        return rotationRequiredDeg;
    }




    public static void main(String[] args) {
        // 测试用例:
        double extension1 = 1.0;
        double angle1 = getServoRotationDegrees(extension1);
        if (!Double.isNaN(angle1)) {
            System.out.printf("目标伸出: %.2f cm, 舵机旋转: %.2f°\n", extension1, angle1);
        } else {
            System.out.printf("目标伸出: %.2f cm, 无法计算角度。\n", extension1);
        }
    }
}