package org.firstinspires.ftc.teamcode.commands.elevator.closed

import com.arcrobotics.ftclib.command.CommandBase
import org.firstinspires.ftc.teamcode.subsystems.elevator.ElevatorSubsystem

class HeightCommand(
    private val subsystem: ElevatorSubsystem,
    private val goal: Double,
) : CommandBase() {

    override fun initialize() {
        subsystem.setpoint = goal
    }
}