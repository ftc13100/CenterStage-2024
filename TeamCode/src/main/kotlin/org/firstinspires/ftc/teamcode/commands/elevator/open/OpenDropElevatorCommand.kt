package org.firstinspires.ftc.teamcode.commands.elevator.open

import com.arcrobotics.ftclib.command.CommandBase
import org.firstinspires.ftc.teamcode.subsystems.elevator.OpenElevatorSubsystem

class OpenDropElevatorCommand(
    private val subsystem: OpenElevatorSubsystem
) : CommandBase() {
    override fun execute() {
        subsystem.spinDown()
    }

    override fun end(interrupted: Boolean) {
        subsystem.stallSpin()
    }

    override fun isFinished(): Boolean = subsystem.isPressed()

}