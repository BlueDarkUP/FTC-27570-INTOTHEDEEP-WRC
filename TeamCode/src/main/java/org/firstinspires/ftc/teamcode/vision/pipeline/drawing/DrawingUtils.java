package org.firstinspires.ftc.teamcode.vision.pipeline.drawing;

import org.firstinspires.ftc.teamcode.vision.pipeline.VisionConstants;
import org.firstinspires.ftc.teamcode.vision.pipeline.model.TargetZoneInfo;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;

import java.util.ArrayList;

/**
 * 调试绘图工具类
 * 这是一个静态工具类，封装了所有在相机画面上绘制调试信息（如目标区域、网格线）的方法。
 * 将所有绘图逻辑集中到此类，可以使主管道（VisionPipeline）的代码更加简洁，专注于处理流程。
 *
 * 此类中的所有方法都应是静态的，因为它不维护任何状态。
 * 绘图操作的开关通常由 VisionConstants.ENABLE_DEBUG_VIEW 常量控制。
 * 此类的方法通常会返回一个 TargetZoneInfo 对象，其中包含了所绘制区域的几何信息，供后续使用。
 * @author BlueDarkUP
 * @version 2025/6 (Refactored)
 * To My Lover - Zyy
 */
public final class DrawingUtils {

    private DrawingUtils() {}

    public static void drawGridOverlay(Mat frame, int size, Scalar color, int thickness) {
        if (!VisionConstants.ENABLE_DEBUG_VIEW) return;
        for (int x = 0; x < frame.cols(); x += size) Imgproc.line(frame, new Point(x, 0), new Point(x, frame.rows()), color, thickness);
        for (int y = 0; y < frame.rows(); y += size) Imgproc.line(frame, new Point(0, y), new Point(frame.cols(), y), color, thickness);
    }

    public static TargetZoneInfo drawTargetZoneCm(Mat frame, double pixels_per_cm_val, int frame_width_px, int frame_height_px, double rect_width_cm, double rect_height_cm, double offset_x_cm, double offset_y_cm, Scalar color, int thickness, int arc_points_sampling) {
        if (pixels_per_cm_val <= 0) return new TargetZoneInfo(null, null, null, null);

        int rect_width_px = (int) Math.round(rect_width_cm * pixels_per_cm_val);
        int rect_height_px = (int) Math.round(rect_height_cm * pixels_per_cm_val);
        int offset_x_px = (int) Math.round(offset_x_cm * pixels_per_cm_val);
        int offset_y_px = (int) Math.round(offset_y_cm * pixels_per_cm_val);
        if (rect_width_px <= 0 || rect_height_px <= 0) return new TargetZoneInfo(null, null, null, null);

        int rect_center_x_px = frame_width_px / 2 + offset_x_px;
        int rect_center_y_px = frame_height_px / 2 + offset_y_px;
        int x1 = rect_center_x_px - rect_width_px / 2;
        int y1_top_edge = rect_center_y_px - rect_height_px / 2;
        int x2 = rect_center_x_px + rect_width_px / 2;
        int y2_bottom_edge = rect_center_y_px + rect_height_px / 2;
        ArrayList<Point> contour_points = new ArrayList<>();
        int arc_radius_px = rect_width_px / 2;

        if (arc_radius_px > 0) {
            if (VisionConstants.ENABLE_DEBUG_VIEW) Imgproc.ellipse(frame, new Point(rect_center_x_px, y2_bottom_edge), new Size(arc_radius_px, arc_radius_px), 0, 180, 360, color, thickness);
            for (int i = 0; i <= arc_points_sampling; i++) {
                double angle_rad = Math.toRadians(360 - (i * 180.0 / arc_points_sampling));
                contour_points.add(new Point(rect_center_x_px + (int)Math.round(arc_radius_px * Math.cos(angle_rad)), y2_bottom_edge + (int)Math.round(arc_radius_px * Math.sin(angle_rad))));
            }
        } else {
            contour_points.add(new Point(x2, y2_bottom_edge));
            contour_points.add(new Point(x1, y2_bottom_edge));
        }

        if (VisionConstants.ENABLE_DEBUG_VIEW) Imgproc.line(frame, new Point(x1, y1_top_edge), new Point(x1, y2_bottom_edge), color, thickness);
        if (!contour_points.isEmpty() && (contour_points.get(contour_points.size() - 1).x != x1 || contour_points.get(contour_points.size() - 1).y != y2_bottom_edge)) contour_points.add(new Point(x1, y2_bottom_edge));
        contour_points.add(new Point(x1, y1_top_edge));

        if (arc_radius_px > 0) {
            if (VisionConstants.ENABLE_DEBUG_VIEW) Imgproc.ellipse(frame, new Point(rect_center_x_px, y1_top_edge), new Size(arc_radius_px, arc_radius_px), 0, 180, 360, color, thickness);
            for (int i = 0; i <= arc_points_sampling; i++) {
                double angle_rad = Math.toRadians(180 + (i * 180.0 / arc_points_sampling));
                contour_points.add(new Point(rect_center_x_px + (int)Math.round(arc_radius_px * Math.cos(angle_rad)), y1_top_edge + (int)Math.round(arc_radius_px * Math.sin(angle_rad))));
            }
        } else {
            contour_points.add(new Point(x2, y1_top_edge));
        }

        if (VisionConstants.ENABLE_DEBUG_VIEW) Imgproc.line(frame, new Point(x2, y1_top_edge), new Point(x2, y2_bottom_edge), color, thickness);
        if (!contour_points.isEmpty() && (contour_points.get(contour_points.size() - 1).x != x2 || contour_points.get(contour_points.size() - 1).y != y1_top_edge)) contour_points.add(new Point(x2, y1_top_edge));
        contour_points.add(new Point(x2, y2_bottom_edge));

        MatOfPoint final_contour = new MatOfPoint();
        final_contour.fromList(contour_points);

        return new TargetZoneInfo(final_contour, rect_center_x_px, y1_top_edge, y2_bottom_edge);
    }
}