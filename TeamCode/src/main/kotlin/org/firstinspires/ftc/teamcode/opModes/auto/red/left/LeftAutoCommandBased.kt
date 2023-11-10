package org.firstinspires.ftc.teamcode.opModes.auto.red.left

import com.acmerobotics.roadrunner.geometry.Pose2d
import com.acmerobotics.roadrunner.geometry.Vector2d
import com.arcrobotics.ftclib.command.CommandOpMode
import com.arcrobotics.ftclib.command.WaitUntilCommand
import com.qualcomm.hardware.motors.RevRoboticsUltraPlanetaryHdHexMotor
import com.qualcomm.hardware.rev.RevBlinkinLedDriver
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import org.firstinspires.ftc.teamcode.commands.drive.DriveToTagCommand
import org.firstinspires.ftc.teamcode.commands.drive.TrajectoryCommand
import org.firstinspires.ftc.teamcode.constants.AutoStartPose
import org.firstinspires.ftc.teamcode.processors.BeaverProcessor.Selected
import org.firstinspires.ftc.teamcode.subsystems.drive.DriveSubsystem
import org.firstinspires.ftc.teamcode.subsystems.vision.VisionSubsystem

@Autonomous
class LeftAutoCommandBased: CommandOpMode() {
    private lateinit var driveSubsystem: DriveSubsystem
    private lateinit var visionSubsystem: VisionSubsystem

    private lateinit var trajectory: TrajectoryCommand

    private lateinit var selection: Selected
    override fun initialize() {
        driveSubsystem = DriveSubsystem(hardwareMap)
        visionSubsystem = VisionSubsystem(hardwareMap, telemetry)

        trajectory = TrajectoryCommand(
            AutoStartPose.RED_LEFT::startPose,
            driveSubsystem
        ) { startPose -> driveSubsystem
                .trajectorySequenceBuilder(startPose)
                .lineTo(Vector2d(-36.0, -35.0))
                .apply {
                    when (selection) {
                        Selected.LEFT -> lineToSplineHeading(
                            Pose2d(
                                -45.0,
                                -30.0,
                                Math.toRadians(-45.0)
                            )
                        )

                        Selected.CENTER -> lineTo(Vector2d(-36.0, 30.0))
                        Selected.RIGHT -> lineToSplineHeading(
                            Pose2d(
                                -30.0,
                                -30.0,
                                Math.toRadians(-135.0)
                            )
                        )

                        else -> lineToSplineHeading(Pose2d(-45.0, -30.0, Math.toRadians(-45.0)))
                    }
                }
                .waitSeconds(1.0)
                .lineTo(Vector2d(-36.0, -35.0))
                .apply {
                    if (selection == Selected.RIGHT)
                        this.lineToSplineHeading(Pose2d(-36.0, -58.0, Math.toRadians(-90.0)))
                            .lineTo(Vector2d(-10.0, -58.0))
                            .lineTo(Vector2d(-10.0, -35.0))
                }
                .lineToSplineHeading(Pose2d(35.0, -35.0, Math.toRadians(180.0)))
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

        WaitUntilCommand(this::isStarted).andThen(
            trajectory
        ).andThen(
            DriveToTagCommand(5, driveSubsystem, visionSubsystem) {
                visionSubsystem.targetPose?.let {
                    Pose2d(
                        it.range,
                        it.yaw,
                        it.bearing
                    )
                }
            }
        ).andThen(

        ).schedule()
    }
}