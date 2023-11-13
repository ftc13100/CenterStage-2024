package org.firstinspires.ftc.teamcode.commands.elevator.closed

import com.arcrobotics.ftclib.command.CommandBase
import org.firstinspires.ftc.teamcode.subsystems.elevator.ProfiledElevatorSubsystem

class ProfiledHeightCommand(
    private val subsystem: ProfiledElevatorSubsystem,
    private val goal: Double
) : CommandBase() {
    override fun initialize() {
        subsystem.setGoal(goal)
    }
}