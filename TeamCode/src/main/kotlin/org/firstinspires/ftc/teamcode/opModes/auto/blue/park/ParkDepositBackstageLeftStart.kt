package org.firstinspires.ftc.teamcode.opModes.auto.blue.park

import com.arcrobotics.ftclib.command.CommandOpMode
import com.arcrobotics.ftclib.hardware.motors.Motor
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import org.firstinspires.ftc.teamcode.commands.drive.TrajectoryCommand
import org.firstinspires.ftc.teamcode.commands.intake.IntakeCommand
import org.firstinspires.ftc.teamcode.constants.AutoStartPose
import org.firstinspires.ftc.teamcode.constants.ControlBoard
import org.firstinspires.ftc.teamcode.subsystems.drive.DriveSubsystem
import org.firstinspires.ftc.teamcode.subsystems.intake.IntakeSubsystem

@Autonomous
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
            AutoStartPose.BLUE_LEFT::startPose, driveSubsystem
        ) {
            driveSubsystem.trajectorySequenceBuilder(it).strafeLeft(45.0).build()
        }.andThen(
            outtakeCommand.withTimeout(5000)
        ).schedule()
    }
}