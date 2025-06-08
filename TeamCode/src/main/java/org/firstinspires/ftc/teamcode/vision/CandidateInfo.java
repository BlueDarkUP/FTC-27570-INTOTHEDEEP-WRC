// Filename: CandidateInfo.java
package org.firstinspires.ftc.teamcode.vision;

import org.opencv.core.Point;

/**
 * Pipeline 内部使用的数据类，用于评估和排序可能是最终目标的候选对象。
 * @author BlueDarkUP
 * @version 2025/6
 * To My Lover - Zyy
 */
public class CandidateInfo {
    int cubeIndex;
    double primaryScore, secondaryScore, lineAngleDeg, distanceCm;
    Point centerInProcessed, intersectionPointProcessed;
    // 新增字段
    double horizontalOffsetCm; // 候选目标相对于摄像头中心线的横向偏移量
}