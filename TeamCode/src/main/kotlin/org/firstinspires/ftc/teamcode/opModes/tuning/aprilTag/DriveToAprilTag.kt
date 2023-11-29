package org.firstinspires.ftc.teamcode.opModes.tuning.aprilTag

import com.acmerobotics.dashboard.config.Config
import com.acmerobotics.roadrunner.geometry.Pose2d
import com.arcrobotics.ftclib.command.CommandOpMode
import com.arcrobotics.ftclib.command.InstantCommand
import com.arcrobotics.ftclib.command.RunCommand
import com.arcrobotics.ftclib.command.WaitUntilCommand
import com.arcrobotics.ftclib.gamepad.GamepadEx
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl
import org.firstinspires.ftc.teamcode.commands.drive.DriveCommand
import org.firstinspires.ftc.teamcode.commands.drive.TrajectoryCommand
import org.firstinspires.ftc.teamcode.subsystems.drive.DriveSubsystem
import org.firstinspires.ftc.teamcode.subsystems.vision.VisionSubsystem
import org.firstinspires.ftc.vision.VisionPortal.CameraState
import java.util.concurrent.TimeUnit
import kotlin.math.cos
import kotlin.math.sin

@TeleOp
@Config
class DriveToAprilTag : CommandOpMode() {
    private lateinit var driveSubsystem: DriveSubsystem
    private lateinit var visionSubsystem: VisionSubsystem

    private lateinit var driveCommand: DriveCommand
    private lateinit var driveToTagCommand: TrajectoryCommand

    private lateinit var driver: GamepadEx
    override fun initialize() {
        driveSubsystem = DriveSubsystem(hardwareMap)
        visionSubsystem = VisionSubsystem(hardwareMap, telemetry)

        driver = GamepadEx(gamepad1)

        driveCommand = DriveCommand(
            driveSubsystem, driver::getLeftX, driver::getLeftY, driver::getRightX, zoneVal = 0.0
        )

        driveSubsystem.defaultCommand = driveCommand

        WaitUntilCommand { visionSubsystem.cameraState == CameraState.STREAMING }.andThen(
            InstantCommand({
                visionSubsystem.portal.getCameraControl(ExposureControl::class.java)
                    .setExposure(2, TimeUnit.MILLISECONDS)
                visionSubsystem.portal.getCameraControl(GainControl::class.java).gain = 500
            })
        )

        driveToTagCommand = TrajectoryCommand(
            driveSubsystem::poseEstimate, driveSubsystem
        ) {
            val detectionPose = visionSubsystem.targetPose
                ?: return@TrajectoryCommand driveSubsystem.trajectorySequenceBuilder(it)
                    .waitSeconds(0.0).build()

            return@TrajectoryCommand driveSubsystem.trajectorySequenceBuilder(it)
                .lineToSplineHeading(
                    Pose2d(
                        it.x + CAMERA_X + detectionPose.x - DriveSubsystem.DESIRED_TAG_DISTANCE * sin(
                            detectionPose.yaw
                        ),
                        it.y + CAMERA_Y + detectionPose.y - DriveSubsystem.DESIRED_TAG_DISTANCE * cos(
                            detectionPose.yaw
                        ),
                        Math.toRadians(180.0) + detectionPose.yaw
                    )
                ).build()
        }

        WaitUntilCommand { visionSubsystem.cameraState == CameraState.STREAMING }.andThen(
            InstantCommand({
                visionSubsystem.portal.getCameraControl(ExposureControl::class.java)
                    .setExposure(6, TimeUnit.MILLISECONDS)
                visionSubsystem.portal.getCameraControl(GainControl::class.java).gain = 250
            })
        ).schedule()

        RunCommand({ visionSubsystem.targetId = targetId }).perpetually().schedule()

        schedule(
            RunCommand({
                for ((_, pose) in visionSubsystem.detectionPoses) {
                    telemetry.addLine(
                        String.format(
                            "XYZ %6.1f %6.1f %6.1f  (inch)", pose.x, pose.y, pose.z
                        )
                    )
                    telemetry.addLine(
                        String.format(
                            "PRY %6.1f %6.1f %6.1f  (deg)", pose.pitch, pose.roll, pose.yaw
                        )
                    )
                    telemetry.addLine(
                        String.format(
                            "RBE %6.1f %6.1f %6.1f  (inch, deg, deg)",
                            pose.range,
                            pose.bearing,
                            pose.elevation
                        )
                    )
                }

                telemetry.update()
            }).perpetually(), driveCommand
        )

    }

    companion object {
        @JvmField
        var targetId = 5

        @JvmField
//        var CAMERA_X = 8.5
        var CAMERA_X = 0.0

        @JvmField
//        var CAMERA_Y = 4.5
        var CAMERA_Y = 0.0

    }
}