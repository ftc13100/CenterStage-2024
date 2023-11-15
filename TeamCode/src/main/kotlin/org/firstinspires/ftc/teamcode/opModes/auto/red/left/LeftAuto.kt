package org.firstinspires.ftc.teamcode.opModes.auto.red.left

import android.util.Size
import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry
import com.acmerobotics.roadrunner.geometry.Pose2d
import com.acmerobotics.roadrunner.geometry.Vector2d
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.OpMode
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit
import org.firstinspires.ftc.teamcode.constants.ControlBoard
import org.firstinspires.ftc.teamcode.constants.PoseStorage
import org.firstinspires.ftc.teamcode.processors.BeaverProcessor
import org.firstinspires.ftc.teamcode.processors.BeaverProcessor.Selected
import org.firstinspires.ftc.teamcode.subsystems.drive.DriveSubsystem
import org.firstinspires.ftc.teamcode.utils.roadrunner.trajectorysequence.TrajectorySequence
import org.firstinspires.ftc.vision.VisionPortal
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor

@Autonomous(name = "Left Auto (Red)", group = "Red Auto")
class LeftAuto : OpMode() {
    private lateinit var beaverProcessor: BeaverProcessor
    private lateinit var visionPortal: VisionPortal

    private lateinit var aprilTag: AprilTagProcessor

    private lateinit var drive: DriveSubsystem

    private val startPose = Pose2d(-36.0, -61.5, Math.toRadians(90.0))
    private lateinit var path: TrajectorySequence

    override fun init() {
        telemetry = MultipleTelemetry(telemetry, FtcDashboard.getInstance().telemetry)

        initVisionPortal()

        drive = DriveSubsystem(hardwareMap)

        FtcDashboard.getInstance().startCameraStream(beaverProcessor, visionPortal.fps.toDouble())
    }

    override fun init_loop() {
        val selection = beaverProcessor.selection

        telemetry.addData("Identified: ", selection)
        telemetry.update()

        path = drive.trajectorySequenceBuilder(startPose)
            .lineTo(Vector2d(-36.0, -35.0))
            .apply {
                when (selection) {
                    Selected.LEFT -> {
                        this.lineToSplineHeading(
                            Pose2d(
                                -45.0,
                                -30.0,
                                Math.toRadians(-45.0)
                            )
                        )
                    }

                    Selected.CENTER -> {
                        this.lineTo(
                            Vector2d(
                                -36.0,
                                -30.0
                            )
                        )
                    }

                    Selected.RIGHT -> {
                        this.lineToSplineHeading(
                            Pose2d(
                                -30.0,
                                -30.0,
                                Math.toRadians(-135.0)
                            )
                        )
                    }

                    else -> {
                        this.lineToSplineHeading(
                            Pose2d(
                                -45.0,
                                -30.0,
                                Math.toRadians(-45.0)
                            )
                        )
                    }
                }
            }
            .waitSeconds(1.0)
            .lineTo(Vector2d(-36.0, -35.0))
            .apply {
                if (selection == Selected.RIGHT)
                    this
                        .lineToSplineHeading(
                            Pose2d(
                                -36.0,
                                -58.0,
                                Math.toRadians(-90.0)
                            )
                        )
                        .lineTo(
                            Vector2d(
                                -10.0,
                                -58.0
                            )
                        )
                        .lineTo(
                            Vector2d(
                                -10.0,
                                -35.0
                            )
                        )
            }
            .lineToSplineHeading(
                Pose2d(
                    35.0,
                    -35.0,
                    Math.toRadians(180.0)
                )
            )
            .addTemporalMarker(
                when (selection) {
                    Selected.LEFT -> 8.0
                    Selected.RIGHT -> 13.0
                    Selected.CENTER -> 8.0
                    Selected.NONE -> 8.0
                }
            ) {

            }
            .waitSeconds(3.0)
            .build()
    }

    override fun start() {
        drive.poseEstimate = startPose
        drive.followTrajectorySequenceAsync(path)

        visionPortal.setProcessorEnabled(beaverProcessor, false)
    }

    override fun loop() {
        updateTelemetry()

        drive.update()
        PoseStorage.poseEstimate = drive.poseEstimate
    }

    override fun stop() {
        visionPortal.close()
        PoseStorage.poseEstimate = drive.poseEstimate
    }

    private fun initVisionPortal() {
        aprilTag = AprilTagProcessor.Builder()
            .setOutputUnits(DistanceUnit.INCH, AngleUnit.RADIANS)
            .build()

        beaverProcessor = BeaverProcessor(telemetry)

        visionPortal = VisionPortal.Builder()
            .setCamera(hardwareMap.get(WebcamName::class.java, ControlBoard.CAMERA.deviceName))
            .enableLiveView(true)
            .setAutoStopLiveView(true)
            .setCameraResolution(Size(640, 480))
            .setStreamFormat(VisionPortal.StreamFormat.MJPEG)
            .addProcessors(aprilTag, beaverProcessor)
            .build()
    }

    private fun updateTelemetry() {
        telemetry.addData("Identified: ", beaverProcessor.selection)

        for (detection: AprilTagDetection in aprilTag.detections) {
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
}