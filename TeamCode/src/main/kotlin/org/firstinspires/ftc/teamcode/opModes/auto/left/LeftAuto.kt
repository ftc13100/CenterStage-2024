package org.firstinspires.ftc.teamcode.opModes.auto.left

import android.util.Size
import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.roadrunner.geometry.Pose2d
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.OpMode
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit
import org.firstinspires.ftc.teamcode.constants.ControlBoard
import org.firstinspires.ftc.teamcode.processors.BeaverProcessor
import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive
import org.firstinspires.ftc.vision.VisionPortal
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor

@Autonomous
class LeftAuto: OpMode() {
    private lateinit var beaverProcessor: BeaverProcessor
    private lateinit var visionPortal: VisionPortal

    private lateinit var aprilTag: AprilTagProcessor

    private lateinit var drive: SampleMecanumDrive

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
    }

    override fun start() {
        tagId = when (beaverProcessor.selection) {
            BeaverProcessor.Selected.LEFT -> AprilTagGameDatabase.getCurrentGameTagLibrary().allTags.first { it.name == "BlueAllianceLeft" }.id
            BeaverProcessor.Selected.CENTER -> AprilTagGameDatabase.getCurrentGameTagLibrary().allTags.first { it.name == "BlueAllianceCenter" }.id
            BeaverProcessor.Selected.RIGHT -> AprilTagGameDatabase.getCurrentGameTagLibrary().allTags.first { it.name == "BlueAllianceRight" }.id
            BeaverProcessor.Selected.NONE -> AprilTagGameDatabase.getCurrentGameTagLibrary().allTags.first { it.name == "BlueAllianceCenter" }.id
        }

//        visionPortal.setProcessorEnabled(beaverProcessor, false)
    }
    override fun loop() {
        telemetry.addData("Identified: ", beaverProcessor.selection)

        val targetPose = Pose2d(0.0, 4.0, 0.0)

        val detections = aprilTag.freshDetections ?: aprilTag.detections

        val tagPose = when (val detection = detections.find { it.metadata.id == tagId }) {
            is AprilTagDetection -> Pose2d(detection.ftcPose.yaw, detection.ftcPose.range, detection.ftcPose.bearing)
            else -> targetPose
        }
        val drivePower = (targetPose - tagPose)

        drive.setWeightedDrivePower(drivePower)
        drive.update()
    }

    override fun stop() {
        visionPortal.close()
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