package org.firstinspires.ftc.teamcode.vision;

/**
 * 封装单次视觉识别结果的数据
 * 这是一个不可变对象，一旦创建，其值就不能被修改
 * @author BlueDarkUP
 * @version 2025/6
 * To My Lover - Zyy
 */
public class VisionTargetResult {

    public final boolean isTargetFound;
    public final String color;
    public final double distanceCm;
    public final double objectAngleDeg;
    public final double lineAngleDeg;

    // --- 新增字段 ---
    public final int graspableTargetsInZone; // 抓取区域内的有效目标数量
    public final double nextTargetHorizontalOffsetCm; // 下一个视野内目标的横向偏移量（厘米），0表示无需移动

    /**
     * 默认构造函数，用于表示未找到目标的情况
     */
    public VisionTargetResult() {
        this.isTargetFound = false;
        this.color = "None";
        this.distanceCm = Double.POSITIVE_INFINITY;
        this.objectAngleDeg = 0.0;
        this.lineAngleDeg = 0.0;
        // --- 初始化新增字段 ---
        this.graspableTargetsInZone = 0;
        this.nextTargetHorizontalOffsetCm = 0.0; // 默认值
    }

    /**
     * 完整构造函数，用于创建一个有效的识别结果
     * @param isFound          是否找到了目标
     * @param color            目标的颜色
     * @param distance         目标距离
     * @param objectAngle      物体本身的角度
     * @param lineAngle        物体中心与目标点的连线角度
     * @param graspableTargets 抓取区内目标数
     * @param nextOffset       下一步横向偏移
     */
    public VisionTargetResult(boolean isFound, String color, double distance, double objectAngle, double lineAngle, int graspableTargets, double nextOffset) {
        this.isTargetFound = isFound;
        this.color = color;
        this.distanceCm = distance;
        this.objectAngleDeg = objectAngle;
        this.lineAngleDeg = lineAngle;
        // --- 赋值新增字段 ---
        this.graspableTargetsInZone = graspableTargets;
        this.nextTargetHorizontalOffsetCm = nextOffset;
    }
}