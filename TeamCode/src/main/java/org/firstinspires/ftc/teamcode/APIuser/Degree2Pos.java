package org.firstinspires.ftc.teamcode.APIuser;

import org.firstinspires.ftc.teamcode.API.PositionCalculator;

public class Degree2Pos {

    public static void main(String[] args) {
        System.out.println("--- 调用示例 ---");

        // 示例 1:
        double positionMin1 = 0.1;
        double positionMax1 = 0.71;
        double rotateAngle1 = 180.0;
        boolean reverse1 = false;
        double userInput1 = 90.0;

        try {
            double result1 = PositionCalculator.calculatePositionValue(
                    positionMin1, positionMax1, rotateAngle1, reverse1, userInput1
            );
            System.out.printf("计算结果 (%.1f, %.2f, %.1f, %b, %.1f): %.10f%n",
                    positionMin1, positionMax1, rotateAngle1, reverse1, userInput1, result1);
        } catch (IllegalArgumentException e) {
            System.out.println("发生错误: " + e.getMessage());
        }
        System.out.println("-----------------------------");

        // 示例 2: reverse=True 的情况 (原始结果 6.0，归一化后 0.0)
        double positionMin2 = 10.0;
        double positionMax2 = 20.0;
        double rotateAngle2 = 5.0;
        boolean reverse2 = true;
        double userInput2 = 2.5;

        try {
            double result2 = PositionCalculator.calculatePositionValue(
                    positionMin2, positionMax2, rotateAngle2, reverse2, userInput2
            );
            System.out.printf("计算结果 (%.1f, %.1f, %.1f, %b, %.1f): %.10f%n",
                    positionMin2, positionMax2, rotateAngle2, reverse2, userInput2, result2);
        } catch (IllegalArgumentException e) {
            System.out.println("发生错误: " + e.getMessage());
        }
        System.out.println("-----------------------------");

        // 示例 3: 原始结果为 1.3 的情况 (归一化后 0.3)
        double positionMin3 = 0.0;
        double positionMax3 = 1.3;
        double rotateAngle3 = 1.0;
        boolean reverse3 = false;
        double userInput3 = 1.0;

        try {
            double result3 = PositionCalculator.calculatePositionValue(
                    positionMin3, positionMax3, rotateAngle3, reverse3, userInput3
            );
            System.out.printf("计算结果 (%.1f, %.1f, %.1f, %b, %.1f): %.10f%n",
                    positionMin3, positionMax3, rotateAngle3, reverse3, userInput3, result3);
        } catch (IllegalArgumentException e) {
            System.out.println("发生错误: " + e.getMessage());
        }
        System.out.println("-----------------------------");

        // 示例 4: 原始结果为 -0.3 的情况 (归一化后 0.7)
        double positionMin4 = 0.0;
        double positionMax4 = -0.3;
        double rotateAngle4 = 1.0;
        boolean reverse4 = false;
        double userInput4 = 1.0;

        try {
            double result4 = PositionCalculator.calculatePositionValue(
                    positionMin4, positionMax4, rotateAngle4, reverse4, userInput4
            );
            System.out.printf("计算结果 (%.1f, %.1f, %.1f, %b, %.1f): %.10f%n",
                    positionMin4, positionMax4, rotateAngle4, reverse4, userInput4, result4);
        } catch (IllegalArgumentException e) {
            System.out.println("发生错误: " + e.getMessage());
        }
        System.out.println("-----------------------------");

        // 示例 5: rotateAngle 为 0 导致错误的情况
        double positionMin5 = 1.0;
        double positionMax5 = 5.0;
        double rotateAngle5 = 0.0;
        boolean reverse5 = false;
        double userInput5 = 10.0;

        try {
            double result5 = PositionCalculator.calculatePositionValue(
                    positionMin5, positionMax5, rotateAngle5, reverse5, userInput5
            );
            System.out.printf("计算结果 (%.1f, %.1f, %.1f, %b, %.1f): %.10f%n",
                    positionMin5, positionMax5, rotateAngle5, reverse5, userInput5, result5);
        } catch (IllegalArgumentException e) {
            System.out.println("发生错误" + e.getMessage());
        }
        System.out.println("-----------------------------");
    }
}