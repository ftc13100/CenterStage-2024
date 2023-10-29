package org.firstinspires.ftc.teamcode.processors;

import android.graphics.Canvas;
import android.graphics.Color;
import android.graphics.Paint;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.jetbrains.annotations.NotNull;
import org.jetbrains.annotations.Nullable;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

import kotlin.Metadata;

@Metadata(
        mv = {1, 8, 0},
        k = 1,
        d1 = {"\u0000d\n\u0002\u0018\u0002\n\u0002\u0018\u0002\n\u0002\b\u0002\n\u0002\u0018\u0002\n\u0000\n\u0002\u0018\u0002\n\u0002\b\u0003\n\u0002\u0018\u0002\n\u0002\b\u0006\n\u0002\u0010\u0006\n\u0002\b\u0003\n\u0002\u0010\u0002\n\u0000\n\u0002\u0010\b\n\u0002\b\u0002\n\u0002\u0018\u0002\n\u0000\n\u0002\u0018\u0002\n\u0000\n\u0002\u0010\u0007\n\u0002\b\u0002\n\u0002\u0018\u0002\n\u0002\b\u0004\n\u0002\u0010\u0000\n\u0002\b\u0003\n\u0002\u0010\t\n\u0002\b\u0002\u0018\u00002\u00020\u0001:\u0001+B\u0005¢\u0006\u0002\u0010\u0002J\u0018\u0010\u0010\u001a\u00020\u00112\u0006\u0010\u0012\u001a\u00020\u00042\u0006\u0010\u0013\u001a\u00020\u0006H\u0002J\"\u0010\u0014\u001a\u00020\u00152\u0006\u0010\u0016\u001a\u00020\u00172\u0006\u0010\u0018\u001a\u00020\u00172\b\u0010\u0019\u001a\u0004\u0018\u00010\u001aH\u0016J\u0018\u0010\u001b\u001a\u00020\u001c2\u0006\u0010\u0013\u001a\u00020\u00062\u0006\u0010\u001d\u001a\u00020\u001eH\u0002J8\u0010\u001f\u001a\u00020\u00152\u0006\u0010 \u001a\u00020!2\u0006\u0010\"\u001a\u00020\u00172\u0006\u0010#\u001a\u00020\u00172\u0006\u0010\u001d\u001a\u00020\u001e2\u0006\u0010$\u001a\u00020\u001e2\u0006\u0010%\u001a\u00020&H\u0016J\u001a\u0010'\u001a\u00020&2\b\u0010(\u001a\u0004\u0018\u00010\u00042\u0006\u0010)\u001a\u00020*H\u0016R\u000e\u0010\u0003\u001a\u00020\u0004X\u0082\u000e¢\u0006\u0002\n\u0000R\u000e\u0010\u0005\u001a\u00020\u0006X\u0082\u000e¢\u0006\u0002\n\u0000R\u000e\u0010\u0007\u001a\u00020\u0006X\u0082\u000e¢\u0006\u0002\n\u0000R\u000e\u0010\b\u001a\u00020\u0006X\u0082\u000e¢\u0006\u0002\n\u0000R\u001a\u0010\t\u001a\u00020\nX\u0086\u000e¢\u0006\u000e\n\u0000\u001a\u0004\b\u000b\u0010\f\"\u0004\b\r\u0010\u000eR\u000e\u0010\u000f\u001a\u00020\u0004X\u0082\u000e¢\u0006\u0002\n\u0000¨\u0006,"},
        d2 = {"Lorg/firstinspires/ftc/teamcode/processors/BeaverProcessorJava;", "Lorg/firstinspires/ftc/vision/VisionProcessor;", "()V", "hsvMat", "Lorg/opencv/core/Mat;", "rectCenter", "Lorg/opencv/core/Rect;", "rectLeft", "rectRight", "selection", "Lorg/firstinspires/ftc/teamcode/processors/BeaverProcessorJava$Selected;", "getSelection", "()Lorg/firstinspires/ftc/teamcode/processors/BeaverProcessorJava$Selected;", "setSelection", "(Lorg/firstinspires/ftc/teamcode/processors/BeaverProcessorJava$Selected;)V", "submat", "getAvgSaturation", "", "input", "rect", "init", "", "width", "", "height", "calibration", "Lorg/firstinspires/ftc/robotcore/internal/camera/calibration/CameraCalibration;", "makeGraphicsRect", "Landroid/graphics/Rect;", "scaleBmpPxToCanvasPx", "", "onDrawFrame", "canvas", "Landroid/graphics/Canvas;", "onscreenWidth", "onscreenHeight", "scaleCanvasDensity", "userContext", "", "processFrame", "frame", "captureTimeNanos", "", "Selected", "TeamCode_debug"}
)
public final class BeaverProcessorJava implements VisionProcessor {
    public Rect rectCenter = new Rect(150, 110, 40, 40);
    public Rect rectRight = new Rect(275, 110, 40, 40);
    Selected selection = Selected.NONE;
    private Mat submat = new Mat();
    private final Mat hsvMat = new Mat();

    Telemetry telemetry;

    @NotNull
    public Selected getSelection() {
        return this.selection;
    }

    public BeaverProcessorJava(Telemetry telemetry) { this.telemetry = telemetry; }

    public void init(int width, int height, @Nullable CameraCalibration calibration) {
    }

    public Object processFrame(Mat frame, long captureTimeNanos) {
        Imgproc.cvtColor(frame, this.hsvMat, 41);

        double satRectCenter = this.getAvgSaturation(this.hsvMat, this.rectCenter);
        double satRectRight = this.getAvgSaturation(this.hsvMat, this.rectRight);
        double var10 = Math.max(satRectCenter, satRectRight);
        this.selection = var10 == satRectCenter ? BeaverProcessorJava.Selected.CENTER : (var10 == satRectRight ? BeaverProcessorJava.Selected.RIGHT : BeaverProcessorJava.Selected.LEFT);
        telemetry.addData("Identified", selection);
        telemetry.update();
        return this.selection;
    }

    private double getAvgSaturation(Mat input, Rect rect) {
        this.submat = input.submat(rect);
        Scalar color = Core.mean(this.submat);
        return color.val[1];
    }

    private android.graphics.Rect makeGraphicsRect(Rect rect, float scaleBmpPxToCanvasPx) {
        float var4 = (float)rect.x * scaleBmpPxToCanvasPx;
        int left = (int)((float)Math.rint(var4));
        float var5 = (float)rect.y * scaleBmpPxToCanvasPx;
        int top = (int)((float)Math.rint(var5));
        float var6 = (float)rect.width * scaleBmpPxToCanvasPx;
        int right = left + (int)((float)Math.rint(var6));
        float var7 = (float)rect.height * scaleBmpPxToCanvasPx;
        int bottom = top + (int)((float)Math.rint(var7));
        return new android.graphics.Rect(left, top, right, bottom);
    }

    public void onDrawFrame(@NotNull Canvas canvas, int onscreenWidth, int onscreenHeight, float scaleBmpPxToCanvasPx, float scaleCanvasDensity, @NotNull Object userContext) {
        Paint selectedPaint = new Paint();
        selectedPaint.setColor(Color.GREEN);
        selectedPaint.setStyle(Paint.Style.STROKE);
        selectedPaint.setStrokeWidth(scaleCanvasDensity * 4);

        Paint nonSelectedPaint = new Paint();
        nonSelectedPaint.setColor(Color.RED);
        nonSelectedPaint.setStyle(Paint.Style.STROKE);
        nonSelectedPaint.setStrokeWidth(scaleCanvasDensity * 4);

        android.graphics.Rect drawRectangleCenter = makeGraphicsRect(rectCenter, scaleBmpPxToCanvasPx);
        android.graphics.Rect drawRectangleRight = makeGraphicsRect(rectRight, scaleBmpPxToCanvasPx);

        selection = (Selected) userContext;

        switch (selection) {
            case LEFT, NONE:
                canvas.drawRect(drawRectangleCenter, nonSelectedPaint);
                canvas.drawRect(drawRectangleRight, nonSelectedPaint);
                break;

            case RIGHT:
                canvas.drawRect(drawRectangleCenter, nonSelectedPaint);
                canvas.drawRect(drawRectangleRight, selectedPaint);
                break;

            case CENTER:
                canvas.drawRect(drawRectangleCenter, selectedPaint);
                canvas.drawRect(drawRectangleRight, nonSelectedPaint);
                break;
        }
    }
    public enum Selected {
        LEFT,
        RIGHT,
        CENTER,
        NONE
    }
}
