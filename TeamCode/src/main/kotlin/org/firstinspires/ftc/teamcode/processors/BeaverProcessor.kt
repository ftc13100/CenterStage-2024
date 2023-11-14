package org.firstinspires.ftc.teamcode.processors

import android.graphics.Bitmap
import android.graphics.Canvas
import android.graphics.Color
import android.graphics.Paint
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

class BeaverProcessor : VisionProcessor, CameraStreamSource {
    private var rectLeft = Rect(110, 42, 40, 40);
    private var rectCenter = Rect(160, 42, 40, 40);
    private var rectRight = Rect(210, 42, 40, 40);

    var selection = Selected.NONE

    private var submat = Mat()
    private var hsvMat = Mat()

    private val lastFrame = AtomicReference(Bitmap.createBitmap(1, 1, Bitmap.Config.RGB_565))

    override fun init(width: Int, height: Int, calibration: CameraCalibration?) {
        lastFrame.set(Bitmap.createBitmap(width, height, Bitmap.Config.RGB_565));
    }

    override fun processFrame(frame: Mat, captureTimeNanos: Long): Any {
        Imgproc.cvtColor(frame, hsvMat, Imgproc.COLOR_RGB2HSV)

        val satRectLeft = getAvgSaturation(hsvMat, rectLeft)
        val satRectCenter = getAvgSaturation(hsvMat, rectCenter)
        val satRectRight = getAvgSaturation(hsvMat, rectRight)

        selection = when (maxOf(satRectLeft, satRectCenter, satRectRight)) {
            satRectCenter -> Selected.CENTER
            satRectRight -> Selected.RIGHT
            satRectLeft -> Selected.LEFT
            else -> Selected.RIGHT
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
        scaleBmpPxToCanvasPx: Float
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

        val drawRectangleLeft = makeGraphicsRect(rectLeft, scaleBmpPxToCanvasPx)
        val drawRectangleCenter = makeGraphicsRect(rectCenter, scaleBmpPxToCanvasPx)
        val drawRectangleRight = makeGraphicsRect(rectRight, scaleBmpPxToCanvasPx)

        selection = userContext as Selected

        when (selection) {
            Selected.LEFT -> {
                canvas.drawRect(drawRectangleLeft, selectedPaint)
                canvas.drawRect(drawRectangleCenter, nonSelectedPaint)
                canvas.drawRect(drawRectangleRight, nonSelectedPaint)
            }
            Selected.RIGHT -> {
                canvas.drawRect(drawRectangleLeft, nonSelectedPaint)
                canvas.drawRect(drawRectangleCenter, nonSelectedPaint)
                canvas.drawRect(drawRectangleRight, selectedPaint)
            }
            Selected.CENTER -> {
                canvas.drawRect(drawRectangleLeft, nonSelectedPaint)
                canvas.drawRect(drawRectangleCenter, selectedPaint)
                canvas.drawRect(drawRectangleRight, nonSelectedPaint)
            }
            Selected.NONE -> {
                canvas.drawRect(drawRectangleLeft, nonSelectedPaint)
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