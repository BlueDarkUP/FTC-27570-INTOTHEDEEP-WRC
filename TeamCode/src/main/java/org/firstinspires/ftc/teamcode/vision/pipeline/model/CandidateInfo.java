package org.firstinspires.ftc.teamcode.vision.pipeline.model;

import org.opencv.core.Point;
/**
 * 候选目标信息数据模型
 * 这是一个在处理阶段使用的数据模型，用于存储关于一个 DetectedCube 的附加评估信息。
 * 当一个物块被认为是潜在的抓取目标时，会创建一个 CandidateInfo 对象来保存其得分、
 * 计算出的抓取角度和距离等，用于后续的排序和决策。
 *
 * 此类是 DetectionProcessor 内部逻辑的核心数据结构。
 * 它将原始的检测数据（来自DetectedCube）与高级的决策数据（如得分）结合在一起。
 * @author BlueDarkUP
 * @version 2025/6 (Refactored)
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
    public double horizontalDistanceToCenterPx;
}