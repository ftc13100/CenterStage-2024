package org.firstinspires.ftc.teamcode.commands.outtake

import com.arcrobotics.ftclib.command.CommandBase
import org.firstinspires.ftc.teamcode.subsystems.elevator.OpenElevatorSubsystem

class FlipOuttakeCommand(
    private val elevatorSubsystem: OpenElevatorSubsystem,
) : CommandBase() {
    override fun initialize() {
        elevatorSubsystem.flipOuttake()
    }

}