package org.firstinspires.ftc.teamcode.processors

import android.graphics.Bitmap
import android.graphics.Canvas
import android.graphics.Color
import android.graphics.Paint
import org.firstinspires.ftc.robotcore.external.Telemetry
import org.firstinspires.ftc.robotcore.external.function.Consumer
import org.firstinspires.ftc.robotcore.external.function.Continuation
import org.firstinspires.ftc.robotcore.external.stream.CameraStreamSource
import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration
import org.firstinspires.ftc.vision.VisionProcessor
import org.opencv.android.Utils
import org.opencv.core.Core
import org.opencv.core.Mat
import org.opencv.core.Rect
import org.opencv.imgproc.Imgproc
import java.util.concurrent.atomic.AtomicReference
import kotlin.math.round

class BeaverProcessor(
    private val telemetry: Telemetry,
) : VisionProcessor, CameraStreamSource {
    private var rectCenter = Rect(80, 180, 150, 160)
    private var rectRight = Rect(460, 180, 175, 180)

    var selection = Selected.NONE

    private var submat = Mat()
    private var hsvMat = Mat()

    private val lastFrame = AtomicReference(Bitmap.createBitmap(1, 1, Bitmap.Config.RGB_565))

    private val hueThresh = 150.0;

    override fun init(width: Int, height: Int, calibration: CameraCalibration) {
        lastFrame.set(Bitmap.createBitmap(width, height, Bitmap.Config.RGB_565));
    }

    override fun processFrame(frame: Mat, captureTimeNanos: Long): Any {
        Imgproc.cvtColor(frame, hsvMat, Imgproc.COLOR_RGB2HSV)

        val satRectCenter = getAvgSaturation(hsvMat, rectCenter)
        val satRectRight = getAvgSaturation(hsvMat, rectRight)

        telemetry.addData("Center Average Saturation", satRectCenter)
        telemetry.addData("Right Average Saturation", satRectRight)

        selection = when (maxOf(satRectCenter, satRectRight, hueThresh)) {
            satRectCenter -> Selected.CENTER
            satRectRight -> Selected.RIGHT
            else -> Selected.LEFT
        }

        val b = Bitmap.createBitmap(frame.width(), frame.height(), Bitmap.Config.RGB_565)

        Utils.matToBitmap(frame, b)
        lastFrame.set(b)

        return selection
    }

    private fun getAvgSaturation(input: Mat, rect: Rect): Double {
        submat = input.submat(rect)
        val color = Core.mean(submat)
        return color.`val`[1]
    }

    private fun makeGraphicsRect(
        rect: Rect,
        scaleBmpPxToCanvasPx: Float,
    ): android.graphics.Rect {
        val left = round(rect.x * scaleBmpPxToCanvasPx).toInt()
        val top = round(rect.y * scaleBmpPxToCanvasPx).toInt()
        val right = left + round(rect.width * scaleBmpPxToCanvasPx).toInt()
        val bottom = top + round(rect.height * scaleBmpPxToCanvasPx).toInt()

        return android.graphics.Rect(left, top, right, bottom)
    }

    override fun onDrawFrame(
        canvas: Canvas,
        onscreenWidth: Int,
        onscreenHeight: Int,
        scaleBmpPxToCanvasPx: Float,
        scaleCanvasDensity: Float,
        userContext: Any,
    ) {
        val selectedPaint = Paint()
        selectedPaint.color = Color.GREEN
        selectedPaint.style = Paint.Style.STROKE
        selectedPaint.strokeWidth = scaleCanvasDensity * 4

        val nonSelectedPaint = Paint()
        nonSelectedPaint.color = Color.RED
        nonSelectedPaint.style = Paint.Style.STROKE
        nonSelectedPaint.strokeWidth = scaleCanvasDensity * 4

        val drawRectangleCenter = makeGraphicsRect(rectCenter, scaleBmpPxToCanvasPx)
        val drawRectangleRight = makeGraphicsRect(rectRight, scaleBmpPxToCanvasPx)

        selection = userContext as Selected

        when (selection) {
            Selected.LEFT -> {
                canvas.drawRect(drawRectangleCenter, nonSelectedPaint)
                canvas.drawRect(drawRectangleRight, nonSelectedPaint)
            }

            Selected.RIGHT -> {
                canvas.drawRect(drawRectangleCenter, nonSelectedPaint)
                canvas.drawRect(drawRectangleRight, selectedPaint)
            }

            Selected.CENTER -> {
                canvas.drawRect(drawRectangleCenter, selectedPaint)
                canvas.drawRect(drawRectangleRight, nonSelectedPaint)
            }

            Selected.NONE -> {
                canvas.drawRect(drawRectangleCenter, nonSelectedPaint)
                canvas.drawRect(drawRectangleRight, nonSelectedPaint)
            }
        }
    }

    enum class Selected {
        LEFT,
        RIGHT,
        CENTER,
        NONE
    }

    override fun getFrameBitmap(continuation: Continuation<out Consumer<Bitmap>>) {
        continuation.dispatch { it.accept(lastFrame.get()) }
    }
}