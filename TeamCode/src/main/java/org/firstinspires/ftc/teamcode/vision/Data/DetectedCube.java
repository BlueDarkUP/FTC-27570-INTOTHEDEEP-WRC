package org.firstinspires.ftc.teamcode.vision.Data;

import org.opencv.core.Point;

/**
 * Pipeline 内部使用的数据类，用于存储一个被检测到的色块（Cube）的**原始信息**
 * 这个类代表了视觉识别流程的早期阶段，记录了从图像中直接观察到的几何和颜色属性
 * @author BlueDarkUP
 * @version 2025/6
 * To My Lover - Zyy
 */
public class DetectedCube {
    public String color;
    public int centerXImagePx, centerYImagePx;
    public Point[] boundingBoxPoints;
    public double scaleFactor, angleDeg;

    public DetectedCube(String color, int centerX, int centerY, Point[] points, double scale, double angle) {
        this.color = color;
        this.centerXImagePx = centerX;
        this.centerYImagePx = centerY;
        this.boundingBoxPoints = points;
        this.scaleFactor = scale;
        this.angleDeg = angle;
    }
}