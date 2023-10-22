package org.firstinspires.ftc.teamcode.opModes.auto.blue.right

import android.util.Size
import com.acmerobotics.dashboard.FtcDashboard
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
import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive
import org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence.TrajectorySequence
import org.firstinspires.ftc.vision.VisionPortal
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor

@Autonomous(name = "Right Auto (Blue)")
class RightAuto: OpMode() {
    private lateinit var beaverProcessor: BeaverProcessor
    private lateinit var visionPortal: VisionPortal

    private lateinit var aprilTag: AprilTagProcessor

    private lateinit var drive: SampleMecanumDrive

    private val startPose = Pose2d(-36.0, 61.5, Math.toRadians(-90.0))
    private lateinit var path: TrajectorySequence

    private var isFound = false
    private var tagId = 0

    override fun init() {
        initVisionPortal()

        drive = SampleMecanumDrive(hardwareMap)

        FtcDashboard.getInstance().startCameraStream(beaverProcessor, visionPortal.fps.toDouble())
    }

    override fun init_loop() {
        telemetry.addData("Identified: ", beaverProcessor.selection)
        telemetry.update()

        path = drive.trajectorySequenceBuilder(startPose)

            .lineTo(Vector2d(-36.0, 35.0))
            .addTemporalMarker(2.0) {
                drive.poseEstimate
            }
            .waitSeconds(2.0)
            .lineToSplineHeading(Pose2d(35.0, 35.0, Math.toRadians(180.0)))
            .addTemporalMarker(8.0) {
                drive.poseEstimate
            }
            .waitSeconds(3.0)
            .build()
    }

    override fun start() {
        tagId = when (beaverProcessor.selection) {
            BeaverProcessor.Selected.LEFT -> AprilTagGameDatabase.getCurrentGameTagLibrary().allTags.first { it.name == "BlueAllianceLeft" }.id
            BeaverProcessor.Selected.CENTER -> AprilTagGameDatabase.getCurrentGameTagLibrary().allTags.first { it.name == "BlueAllianceCenter" }.id
            BeaverProcessor.Selected.RIGHT -> AprilTagGameDatabase.getCurrentGameTagLibrary().allTags.first { it.name == "BlueAllianceRight" }.id
            BeaverProcessor.Selected.NONE -> AprilTagGameDatabase.getCurrentGameTagLibrary().allTags.first { it.name == "BlueAllianceCenter" }.id
        }


        drive.poseEstimate = startPose
        drive.followTrajectorySequenceAsync(path)

//        visionPortal.setProcessorEnabled(beaverProcessor, false)
    }
    override fun loop() {
        telemetry.addData("Identified: ", beaverProcessor.selection)

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

        drive.update()
        telemetry.update()

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

        beaverProcessor = BeaverProcessor()

        visionPortal = VisionPortal.Builder()
            .setCamera(hardwareMap.get(WebcamName::class.java, ControlBoard.CAMERA.deviceName))
            .enableLiveView(true)
            .setAutoStopLiveView(true)
            .setCameraResolution(Size(640, 480))
            .setStreamFormat(VisionPortal.StreamFormat.MJPEG)
            .addProcessors(aprilTag, beaverProcessor)
            .build()
    }
}