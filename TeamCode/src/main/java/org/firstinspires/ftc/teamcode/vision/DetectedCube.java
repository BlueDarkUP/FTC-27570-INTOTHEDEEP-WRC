package org.firstinspires.ftc.teamcode.vision;

import org.opencv.core.Point;

/**
 * Pipeline 内部使用的数据类，用于存储一个被检测到的色块（Cube）的信息。
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