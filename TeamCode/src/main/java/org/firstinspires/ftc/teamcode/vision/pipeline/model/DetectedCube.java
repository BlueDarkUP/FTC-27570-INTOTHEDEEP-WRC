package org.firstinspires.ftc.teamcode.vision.pipeline.model;

import org.opencv.core.Point;

/**
 * 检测到的物块数据模型
 * 这是一个简单的数据传输对象（DTO），用于封装在图像中检测到的单个潜在目标（物块）的基础信息。
 * 它代表了从轮廓分析中直接提取的原始数据。
 *
 * 此类中的数据通常是基于像素坐标系，是未经业务逻辑处理的“原始”检测结果。
 * 它是视觉处理流程中，从轮廓查找阶段到决策处理阶段的数据载体。
 * @author BlueDarkUP
 * @version 2025/6 (Refactored)
 * To My Lover - Zyy
 */
public class DetectedCube {
    public final String color;
    public final int centerXImagePx;
    public final int centerYImagePx;
    public final Point[] boundingBoxPoints;
    public final double scaleFactor;
    public final double angleDeg;
    public final double rectWidthPx;
    public final double rectHeightPx;

    public DetectedCube(String color, int centerX, int centerY, Point[] points, double scale, double angle, double widthPx, double heightPx) {
        this.color = color;
        this.centerXImagePx = centerX;
        this.centerYImagePx = centerY;
        this.boundingBoxPoints = points;
        this.scaleFactor = scale;
        this.angleDeg = angle;
        this.rectWidthPx = widthPx;
        this.rectHeightPx = heightPx;
    }
}