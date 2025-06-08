package org.firstinspires.ftc.teamcode.vision.Data;

import org.opencv.core.Point;

/**
 * Pipeline 内部使用的数据类，用于评估和排序可能是最终目标的候选对象
 * 这个类代表了识别流程的中间阶段，它在 DetectedCube 的原始信息基础上
 * 计算出了对机器人决策更有意义的物理量和评估分数
 * @author BlueDarkUP
 * @version 2025/6
 * To My Lover - Zyy
 */
public class CandidateInfo {
    public int cubeIndex;
    public double primaryScore;
    public double secondaryScore;
    public double lineAngleDeg;
    public double distanceCm;
    public Point centerInProcessed;
    public Point intersectionPointProcessed;
    // 新增字段
    public double horizontalOffsetCm; // 候选目标相对于摄像头中心线的横向偏移量
}