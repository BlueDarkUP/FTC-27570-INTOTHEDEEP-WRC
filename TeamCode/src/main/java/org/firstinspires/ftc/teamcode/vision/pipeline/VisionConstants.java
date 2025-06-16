package org.firstinspires.ftc.teamcode.vision.pipeline;

import org.opencv.core.Scalar;
import java.util.HashMap;
import java.util.Map;

/**
 * 视觉管道常量库
 * 这是一个专门存放 VisionPipeline 及其相关组件使用的所有常量的最终类。
 * 将所有“魔数”（如颜色范围、尺寸阈值、校正因子）集中在此，使得参数的调整和维护变得非常方便。
 *
 * 这是一个final类，并且构造函数是私有的，以防止被错误地实例化。
 * 修改此文件中的值会直接影响视觉识别的效果，是进行现场调试和优化的主要入口。
 * 所有的常量都应该是 public static final，以便在整个 pipeline 包中轻松访问。
 * @author BlueDarkUP
 * @version 2025/6 (Refactored)
 * To My Lover - Zyy
 */
public final class VisionConstants {

    // 私有构造函数防止实例化
    private VisionConstants() {}

    // --- 调试视图 ---
    public static final boolean ENABLE_DEBUG_VIEW = true;

    // --- 目标区域定义 (基于像素/厘米比例) ---
    public static final double PIXELS_PER_CM_FOR_DRAWING = 13.9;
    public static final double TARGET_RECT_WIDTH_CM = 31.5;
    public static final double TARGET_RECT_HEIGHT_CM = 24.5;
    public static final double TARGET_RECT_OFFSET_X_CM = 1.1;
    public static final double TARGET_RECT_OFFSET_Y_CM = 3.0;
    public static final Scalar TARGET_RECT_COLOR = new Scalar(255, 0, 255);
    public static final int TARGET_RECT_THICKNESS = 2;
    public static final int ARC_SAMPLING_POINTS = 20;

    // --- 图像处理参数 ---
    public static final double DOWNSCALE_FACTOR = 1.0; // 1.0 表示不缩放

    // --- 物体过滤参数 ---
    public static final double MIN_SIZE_PIXELS = 150.0;
    public static final double MAX_SIZE_PIXELS = 400000.0;
    public static final double TARGET_OBJECT_ASPECT_RATIO = 7.0 / 3.0;
    public static final double ASPECT_RATIO_TOLERANCE_PERCENT = 0.5;

    // --- 颜色检测范围 (HSV) ---
    public static final Map<String, Scalar[][]> COLOR_HSV_RANGES = new HashMap<>();
    static {
        // 红色有两个范围，因为H值在180度处环绕
        COLOR_HSV_RANGES.put("RED", new Scalar[][]{
                {new Scalar(0, 70, 180), new Scalar(10, 255, 255)},
                {new Scalar(170, 70, 180), new Scalar(180, 255, 255)}
        });
        // 蓝色范围
        COLOR_HSV_RANGES.put("BLUE", new Scalar[][]{
                {new Scalar(100, 70, 130), new Scalar(130, 255, 255)}
        });
    }

    // --- 调试视图网格 ---
    public static final Scalar GRID_COLOR = new Scalar(70, 70, 70);
}