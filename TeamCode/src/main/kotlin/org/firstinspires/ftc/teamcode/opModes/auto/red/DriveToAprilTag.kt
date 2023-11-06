package org.firstinspires.ftc.teamcode.opModes.auto.red

import com.acmerobotics.dashboard.config.Config
import com.arcrobotics.ftclib.command.CommandOpMode
import com.arcrobotics.ftclib.command.InstantCommand
import com.arcrobotics.ftclib.command.RunCommand
import com.arcrobotics.ftclib.command.WaitUntilCommand
import com.arcrobotics.ftclib.gamepad.GamepadEx
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.CameraControl
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit
import org.firstinspires.ftc.teamcode.commands.drive.DriveCommand
import org.firstinspires.ftc.teamcode.constants.ControlBoard
import org.firstinspires.ftc.teamcode.processors.BeaverProcessor
import org.firstinspires.ftc.teamcode.subsystems.drive.DriveSubsystem
import org.firstinspires.ftc.teamcode.subsystems.vision.VisionSubsystem
import org.firstinspires.ftc.vision.VisionPortal
import org.firstinspires.ftc.vision.VisionPortal.CameraState
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor
import java.util.concurrent.TimeUnit
import kotlin.reflect.typeOf

@TeleOp
@Config
class DriveToAprilTag : CommandOpMode() {
    private lateinit var driveSubsystem: DriveSubsystem
    private lateinit var visionSubsystem: VisionSubsystem

    private lateinit var driveCommand: DriveCommand

    private lateinit var driver: GamepadEx
    override fun initialize() {
        driveSubsystem = DriveSubsystem(hardwareMap)
        visionSubsystem = VisionSubsystem(hardwareMap, telemetry)

        driver = GamepadEx(gamepad1)

        driveCommand = DriveCommand(driveSubsystem, driver::getLeftX, driver::getLeftY, driver::getRightX, zoneVal = 0.0)

        driveSubsystem.defaultCommand = driveCommand
        register(driveSubsystem)

        WaitUntilCommand { visionSubsystem.cameraState == CameraState.STREAMING }
            .andThen(
                InstantCommand({ visionSubsystem.portal.getCameraControl(ExposureControl::class.java).setExposure(5, TimeUnit.MILLISECONDS) })
            )
            .schedule()


        RunCommand({ visionSubsystem.targetId = targetId })
            .perpetually()
            .schedule()

        schedule(
            RunCommand({
                for (pose in visionSubsystem.detectionPoses) {
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
    }

    companion object {
        @JvmField
        var targetId = 5;
    }
}