package org.firstinspires.ftc.teamcode.opModes.auto.red.park

import com.arcrobotics.ftclib.command.CommandOpMode
import com.arcrobotics.ftclib.hardware.motors.Motor
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import org.firstinspires.ftc.teamcode.commands.drive.TrajectoryCommand
import org.firstinspires.ftc.teamcode.commands.intake.IntakeCommand
import org.firstinspires.ftc.teamcode.constants.AutoStartPose
import org.firstinspires.ftc.teamcode.constants.ControlBoard
import org.firstinspires.ftc.teamcode.subsystems.drive.DriveSubsystem
import org.firstinspires.ftc.teamcode.subsystems.intake.IntakeSubsystem
import org.firstinspires.ftc.teamcode.utils.roadrunner.drive.DriveConstants


@Autonomous(name = "Red Park + 2 Backstage Pixel (Left)", preselectTeleOp = "MainTeleOp")
class ParkDepositBackstageLeftStart : CommandOpMode() {
    private lateinit var intakeMotor: Motor

    private lateinit var driveSubsystem: DriveSubsystem
    private lateinit var intakeSubsystem: IntakeSubsystem

    private lateinit var outtakeCommand: IntakeCommand
    override fun initialize() {

        intakeMotor = Motor(hardwareMap, ControlBoard.INTAKE.deviceName)

        driveSubsystem = DriveSubsystem(hardwareMap)
        intakeSubsystem = IntakeSubsystem(intakeMotor)

        outtakeCommand = IntakeCommand(intakeSubsystem, false)

        TrajectoryCommand(
            AutoStartPose.BLUE_RIGHT::startPose, driveSubsystem
        ) {
            driveSubsystem.trajectorySequenceBuilder(it)
                .strafeLeft(4.0)
                .forward(84.0,
                    DriveSubsystem.getVelocityConstraint(DriveConstants.MAX_VEL, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                    DriveSubsystem.getAccelerationConstraint(50.0)
                )
                .build()

        }
            .andThen(
                outtakeCommand.withTimeout(1000)
                    .alongWith(
                        TrajectoryCommand(
                            driveSubsystem::poseEstimate, driveSubsystem
                        ) {
                            driveSubsystem.trajectorySequenceBuilder(it)
                                .back(5.0)
                                .build()
                        }
                    )
            )
            .andThen(
                TrajectoryCommand(
                    driveSubsystem::poseEstimate, driveSubsystem
                )  {
                    driveSubsystem.trajectorySequenceBuilder(it)
                        .forward(2.0)
                        .build()
                }
            )
            .schedule()
    }
}