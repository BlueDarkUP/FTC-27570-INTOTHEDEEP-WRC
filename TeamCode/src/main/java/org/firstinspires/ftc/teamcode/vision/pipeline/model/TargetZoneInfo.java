package org.firstinspires.ftc.teamcode.vision.pipeline.model;

import org.opencv.core.MatOfPoint;

/**
 * 目标区域信息数据模型
 * 这是一个数据传输对象（DTO），用于存储在屏幕上绘制的目标抓取区域的关键几何信息，
 * 例如其轮廓、中心线X坐标以及顶部和底部的Y坐标。
 *
 * 此对象由 DrawingUtils 中的绘图函数生成，并由 VisionPipeline 使用。
 * 它使得“判断一个点是否在目标区域内”的逻辑变得清晰和可重用。
 * @author BlueDarkUP
 * @version 2025/6 (Refactored)
 * To My Lover - Zyy
 */
public class TargetZoneInfo {
    public final MatOfPoint targetContour;
    public final Integer centerX;
    public final Integer topY;
    public final Integer bottomY;

    public TargetZoneInfo(MatOfPoint c, Integer cx, Integer ty, Integer by) {
        this.targetContour = c;
        this.centerX = cx;
        this.topY = ty;
        this.bottomY = by;
    }
}