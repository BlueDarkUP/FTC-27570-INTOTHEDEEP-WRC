package org.firstinspires.ftc.teamcode.vision.Data;

import org.opencv.core.MatOfPoint;

/**
 * Pipeline 内部使用的数据类，用于存储目标抓取区域的几何信息
 * 封装了描述目标区域边界的轮廓和一些关键坐标
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