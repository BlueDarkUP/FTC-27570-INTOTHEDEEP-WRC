package org.firstinspires.ftc.teamcode.vision;

import org.opencv.core.MatOfPoint;

/**
 * Pipeline 内部使用的数据类，用于存储目标区域的几何信息。
 * @author BlueDarkUP
 * @version 2025/6
 * To My Lover - Zyy
 */
public class TargetZoneInfo {
    public MatOfPoint targetContour;
    public Integer centerX, topY, bottomY;

    public TargetZoneInfo(MatOfPoint c, Integer cx, Integer ty, Integer by) {
        this.targetContour = c;
        this.centerX = cx;
        this.topY = ty;
        this.bottomY = by;
    }
}