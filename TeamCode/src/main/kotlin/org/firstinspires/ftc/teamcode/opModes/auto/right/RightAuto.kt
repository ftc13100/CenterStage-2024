package org.firstinspires.ftc.teamcode.opModes.auto.right

import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.OpMode
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName
import org.firstinspires.ftc.teamcode.utils.BeaverProcessor
import org.firstinspires.ftc.vision.VisionPortal
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor

@Autonomous
class RightAuto : OpMode() {
    private lateinit var visionPortal: VisionPortal

    private lateinit var aprilTag: AprilTagProcessor
    private lateinit var beaverProcessor: BeaverProcessor

    override fun init() {
        initVisionPortal()
    }

    override fun init_loop() {
        telemetry.addData("Identified: ", beaverProcessor.selection)
        telemetry.update()
    }

    override fun loop() {
        telemetry.addData("Identified ", beaverProcessor.selection)

        for (detection : AprilTagDetection in aprilTag.detections) {
            if (detection.metadata != null) {
                telemetry.addLine(
                    String.format(
                        "\n==== (ID %d) %s",
                        detection.id,
                        detection.metadata.name
                    )
                )
                telemetry.addLine(
                    String.format(
                        "XYZ %6.1f %6.1f %6.1f  (inch)",
                        detection.ftcPose.x,
                        detection.ftcPose.y,
                        detection.ftcPose.z
                    )
                )
                telemetry.addLine(
                    String.format(
                        "PRY %6.1f %6.1f %6.1f  (deg)",
                        detection.ftcPose.pitch,
                        detection.ftcPose.roll,
                        detection.ftcPose.yaw
                    )
                )
                telemetry.addLine(
                    String.format(
                        "RBE %6.1f %6.1f %6.1f  (inch, deg, deg)",
                        detection.ftcPose.range,
                        detection.ftcPose.bearing,
                        detection.ftcPose.elevation
                    )
                )
            } else {
                telemetry.addLine(String.format("\n==== (ID %d) Unknown", detection.id))
                telemetry.addLine(
                    String.format(
                        "Center %6.0f %6.0f   (pixels)",
                        detection.center.x,
                        detection.center.y
                    )
                )
            }
        }

        telemetry.update()
    }

    override fun stop() {
        visionPortal.close()
    }

    private fun initVisionPortal() {
        aprilTag = AprilTagProcessor.easyCreateWithDefaults()

        visionPortal = VisionPortal.Builder()
            .setCamera(hardwareMap.get(WebcamName::class.java, "lifecam"))
            .addProcessors(aprilTag, beaverProcessor)
            .build()
    }
}