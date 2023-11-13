package org.firstinspires.ftc.teamcode.commands.elevator.combined

import com.arcrobotics.ftclib.command.InstantCommand
import com.arcrobotics.ftclib.command.SequentialCommandGroup
import com.arcrobotics.ftclib.command.WaitUntilCommand
import org.firstinspires.ftc.teamcode.commands.elevator.closed.HeightCommand
import org.firstinspires.ftc.teamcode.subsystems.elevator.ElevatorSubsystem

class OuttakeCommand(
    height: Double,
    elevatorSubsystem: ElevatorSubsystem
) : SequentialCommandGroup(
    HeightCommand(elevatorSubsystem, height),
    WaitUntilCommand(elevatorSubsystem::atSetpoint),
    InstantCommand({ elevatorSubsystem.flipOuttake() }, elevatorSubsystem)
)