package org.firstinspires.ftc.teamcode.opModes.tuning.aprilTag

import com.acmerobotics.dashboard.config.Config
import com.acmerobotics.roadrunner.geometry.Pose2d
import com.arcrobotics.ftclib.command.CommandOpMode
import com.arcrobotics.ftclib.command.InstantCommand
import com.arcrobotics.ftclib.command.RunCommand
import com.arcrobotics.ftclib.command.WaitUntilCommand
import com.arcrobotics.ftclib.gamepad.GamepadEx
import com.arcrobotics.ftclib.gamepad.GamepadKeys
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl
import org.firstinspires.ftc.teamcode.commands.drive.DriveCommand
import org.firstinspires.ftc.teamcode.commands.drive.DriveToTagCommand
import org.firstinspires.ftc.teamcode.subsystems.drive.DriveSubsystem
import org.firstinspires.ftc.teamcode.subsystems.vision.VisionSubsystem
import org.firstinspires.ftc.vision.VisionPortal.CameraState
import java.util.concurrent.TimeUnit

@TeleOp
@Config
class DriveToAprilTag : CommandOpMode() {
    private lateinit var driveSubsystem: DriveSubsystem
    private lateinit var visionSubsystem: VisionSubsystem

    private lateinit var driveCommand: DriveCommand
    private lateinit var driveToTagCommand: DriveToTagCommand

    private lateinit var driver: GamepadEx
    override fun initialize() {
        driveSubsystem = DriveSubsystem(hardwareMap)
        visionSubsystem = VisionSubsystem(hardwareMap, telemetry)

        driver = GamepadEx(gamepad1)

        driveCommand = DriveCommand(
            driveSubsystem,
            driver::getLeftX,
            driver::getLeftY,
            driver::getRightX,
            zoneVal = 0.0
        )

        driveSubsystem.defaultCommand = driveCommand

        driveToTagCommand = DriveToTagCommand(targetId, driveSubsystem, visionSubsystem) {
            visionSubsystem.targetPose?.let {
                Pose2d(
                    it.range,
                    it.yaw,
                    it.bearing
                )
            }
        }

        WaitUntilCommand { visionSubsystem.cameraState == CameraState.STREAMING }
            .andThen(
                InstantCommand({
                    visionSubsystem.portal.getCameraControl(ExposureControl::class.java)
                        .setExposure(2, TimeUnit.MILLISECONDS)
                    visionSubsystem.portal.getCameraControl(GainControl::class.java).gain = 500
                })
            )
            .schedule()

        RunCommand({ visionSubsystem.targetId = targetId })
            .perpetually()
            .schedule()

        schedule(
            RunCommand({
                for ((_, pose) in visionSubsystem.detectionPoses) {
                    telemetry.addLine(
                        String.format(
                            "XYZ %6.1f %6.1f %6.1f  (inch)",
                            pose.x,
                            pose.y,
                            pose.z
                        )
                    )
                    telemetry.addLine(
                        String.format(
                            "PRY %6.1f %6.1f %6.1f  (deg)",
                            pose.pitch,
                            pose.roll,
                            pose.yaw
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
            })
                .perpetually(),
            driveCommand
        )

        driver.getGamepadButton(GamepadKeys.Button.A).whileHeld(driveToTagCommand)
    }

    companion object {
        @JvmField
        var targetId = 5;
    }
}