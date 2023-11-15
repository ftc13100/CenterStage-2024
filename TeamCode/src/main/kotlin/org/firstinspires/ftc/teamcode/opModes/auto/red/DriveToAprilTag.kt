package org.firstinspires.ftc.teamcode.opModes.auto.red

import com.arcrobotics.ftclib.command.CommandOpMode
import com.arcrobotics.ftclib.command.InstantCommand
import com.arcrobotics.ftclib.command.RunCommand
import com.arcrobotics.ftclib.command.WaitUntilCommand
import com.arcrobotics.ftclib.gamepad.GamepadEx
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit
import org.firstinspires.ftc.teamcode.commands.drive.DriveCommand
import org.firstinspires.ftc.teamcode.constants.ControlBoard
import org.firstinspires.ftc.teamcode.subsystems.drive.DriveSubsystem
import org.firstinspires.ftc.vision.VisionPortal
import org.firstinspires.ftc.vision.VisionPortal.CameraState
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor
import java.util.concurrent.TimeUnit

@TeleOp
class DriveToAprilTag : CommandOpMode() {
    private lateinit var portal: VisionPortal
    private lateinit var processor: AprilTagProcessor

    private lateinit var driveSubsystem: DriveSubsystem

    private lateinit var driveCommand: DriveCommand

    private lateinit var driver: GamepadEx
    override fun initialize() {
        initVisionPortal()

        driveSubsystem = DriveSubsystem(hardwareMap)

        driver = GamepadEx(gamepad1)

        driveCommand = DriveCommand(
            driveSubsystem,
            driver::getLeftX,
            driver::getLeftY,
            driver::getRightX,
            zoneVal = 0.0
        )

        driveSubsystem.defaultCommand = driveCommand
        register(driveSubsystem)

        schedule(
            WaitUntilCommand { portal.cameraState == CameraState.STREAMING }
                .andThen(
                    InstantCommand({
                        portal.getCameraControl(ExposureControl::class.java)
                            .setExposure(5, TimeUnit.MILLISECONDS)
                    })
                )
        )

        schedule(
            RunCommand({
                for (detection in processor.detections) {
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
            }).perpetually(),
            driveCommand
        )
    }

    private fun initVisionPortal() {
        processor = AprilTagProcessor.Builder()
            .setOutputUnits(DistanceUnit.INCH, AngleUnit.RADIANS)
            .build()

        portal = VisionPortal.Builder()
            .setCamera(hardwareMap.get(WebcamName::class.java, ControlBoard.CAMERA.deviceName))
            .addProcessors(processor)
            .enableLiveView(true)
            .setStreamFormat(VisionPortal.StreamFormat.MJPEG)
            .build()
    }

}