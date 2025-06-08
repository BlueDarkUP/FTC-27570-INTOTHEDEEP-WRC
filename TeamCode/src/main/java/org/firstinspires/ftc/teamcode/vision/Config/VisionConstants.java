package org.firstinspires.ftc.teamcode.vision.Config;

import org.opencv.core.Scalar;
import java.util.HashMap;
import java.util.Map;

/**
 * 存放所有视觉相关配置常量的类
 * 将所有可调参数集中在此处，便于管理和调试，避免在代码中出现“魔法数字”
 * 这是一个final类，并且构造函数是私有的，以防止被继承或实例化
 * @author BlueDarkUP
 * @version 2025/6
 * To My Lover - Zyy
 */
public final class VisionConstants {
    private VisionConstants() {}

    // 摄像头与通用配置
    public static final String WEBCAM_NAME_STR = "Webcam";
    public static final int CAMERA_WIDTH = 1280;
    public static final int CAMERA_HEIGHT = 720;
    public static final double REAL_WORLD_VIEW_WIDTH_CM = 92.0;

    // Pipeline行为配置
    public static final boolean ENABLE_DEBUG_VIEW = false; // 是否在屏幕上绘制调试信息（目标区域、网格等）
    public static final double PIXELS_PER_CM = CAMERA_WIDTH / REAL_WORLD_VIEW_WIDTH_CM;
    public static final double DOWNSCALE_FACTOR = 1.0; // 图像处理时的缩放因子（1.0为不缩放）

    // --- 目标区域配置 ---
    public static final double TARGET_RECT_WIDTH_CM = 31.5;
    public static final double TARGET_RECT_HEIGHT_CM = 24.0;
    public static final double TARGET_RECT_OFFSET_X_CM = 1.1;
    public static final double TARGET_RECT_OFFSET_Y_CM = 3.0;
    public static final Scalar TARGET_RECT_COLOR = new Scalar(255, 0, 255);
    public static final int TARGET_RECT_THICKNESS = 2;
    public static final int ARC_SAMPLING_POINTS = 20; // 用于绘制圆弧轮廓的采样点数

    // 目标物体检测配置
    public static final double MIN_SIZE_PIXELS = 200.0; // 物体在原始分辨率下的最小面积（像素）
    public static final double MAX_SIZE_PIXELS = 25000.0; // 物体在原始分辨率下的最大面积（像素）
    public static final double TARGET_OBJECT_ASPECT_RATIO = 7.0 / 3.0; // 目标物体的长宽比
    public static final double ASPECT_RATIO_TOLERANCE_PERCENT = 0.25; // 长宽比的容差（百分比）

    // 颜色配置
    public static final Map<String, Scalar[][]> COLOR_HSV_RANGES = new HashMap<>();
    static {
        COLOR_HSV_RANGES.put("YELLOW", new Scalar[][]{{new Scalar(15, 70, 180), new Scalar(45, 255, 255)}});
    }
    public static final Scalar GRID_COLOR = new Scalar(70, 70, 70); // 调试网格线的颜色
}