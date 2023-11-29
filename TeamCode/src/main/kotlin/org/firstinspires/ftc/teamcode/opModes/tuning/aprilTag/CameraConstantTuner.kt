package org.firstinspires.ftc.teamcode.opModes.tuning.aprilTag

import com.acmerobotics.roadrunner.geometry.Pose2d
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.OpMode
import org.firstinspires.ftc.teamcode.subsystems.drive.DriveSubsystem
import org.firstinspires.ftc.teamcode.subsystems.vision.VisionSubsystem
import kotlin.math.cos
import kotlin.math.sin

@Autonomous
class CameraConstantTuner : OpMode() {
    private lateinit var driveSubsystem: DriveSubsystem
    private lateinit var visionSubsystem: VisionSubsystem
    override fun init() {
        visionSubsystem = VisionSubsystem(hardwareMap, telemetry)
        driveSubsystem = DriveSubsystem(hardwareMap)

        visionSubsystem.targetId = TARGET_ID
    }

    override fun init_loop() {
        telemetry.clearAll()

        if (!visionSubsystem.detectionPoses.containsKey(TARGET_ID)) {
            telemetry.addLine("Target ID not found! Reorient to find target ID!")
        } else {
            telemetry.addLine("Target ID found!")
        }

        telemetry.update()
    }

    override fun start() {
        val detectionPose = visionSubsystem.targetPose

        if (detectionPose == null) {
            terminateOpModeNow()
            return
        }

        val startPose = driveSubsystem.poseEstimate

        val trajectory = driveSubsystem.trajectorySequenceBuilder(startPose)
            .lineToSplineHeading(
                Pose2d(
                    startPose.x - DriveSubsystem.DESIRED_TAG_DISTANCE * sin(
                        detectionPose.yaw
                    ), startPose.y + DriveSubsystem.DESIRED_TAG_DISTANCE * cos(
                        detectionPose.yaw
                    ), Math.toRadians(180.0) + detectionPose.yaw
                )
            )
            .build()

        driveSubsystem.followTrajectorySequenceAsync(trajectory)
    }

    override fun loop() {
        driveSubsystem.periodic()

        if (driveSubsystem.isBusy)
            return


        val detectionPose = visionSubsystem.targetPose

        telemetry.addData("Final X: ", detectionPose?.x)
        telemetry.addData("Final Y: ", detectionPose?.y)
    }

    companion object {
        @JvmField
        var TARGET_ID = 5
    }
}