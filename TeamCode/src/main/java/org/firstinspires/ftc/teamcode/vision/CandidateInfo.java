package org.firstinspires.ftc.teamcode.vision;

import org.opencv.core.Point;

/**
 * Pipeline 内部使用的数据类，用于评估和排序可能是最终目标的候选对象。
 */
public class CandidateInfo {
    int cubeIndex;
    double primaryScore, secondaryScore, lineAngleDeg, distanceCm;
    Point centerInProcessed, intersectionPointProcessed;
}