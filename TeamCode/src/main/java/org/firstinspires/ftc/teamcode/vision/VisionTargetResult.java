package org.firstinspires.ftc.teamcode.vision;

/**
 * 封装单次视觉识别结果的数据
 * 这是一个不可变对象，一旦创建，其值就不能被修改
 */
public class VisionTargetResult {

    public final boolean isTargetFound;
    public final String color;
    public final double distanceCm;
    public final double objectAngleDeg;
    public final double lineAngleDeg;

    /**
     * 默认构造函数，用于表示未找到目标的情况
     */
    public VisionTargetResult() {
        this.isTargetFound = false;
        this.color = "None";
        this.distanceCm = Double.POSITIVE_INFINITY;
        this.objectAngleDeg = 0.0;
        this.lineAngleDeg = 0.0;
    }

    /**
     * 完整构造函数，用于创建一个有效的识别结果
     * @param isFound      是否找到了目标
     * @param color        目标的颜色
     * @param distance     目标距离
     * @param objectAngle  物体本身的角度
     * @param lineAngle    物体中心与目标点的连线角度
     */
    public VisionTargetResult(boolean isFound, String color, double distance, double objectAngle, double lineAngle) {
        this.isTargetFound = isFound;
        this.color = color;
        this.distanceCm = distance;
        this.objectAngleDeg = objectAngle;
        this.lineAngleDeg = lineAngle;
    }
}