package org.firstinspires.ftc.teamcode.opModes.teleOp

import com.arcrobotics.ftclib.command.CommandOpMode
import com.arcrobotics.ftclib.command.InstantCommand
import com.arcrobotics.ftclib.gamepad.GamepadEx
import com.arcrobotics.ftclib.hardware.motors.Motor
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.constants.ControlBoard
import org.firstinspires.ftc.teamcode.subsystems.elevator.ElevatorSubsystem

@TeleOp
class ElevatorTest: CommandOpMode() {

    private lateinit var elevatorSubsystem: ElevatorSubsystem

    private lateinit var driver: GamepadEx

    private lateinit var elevatorLeft: Motor
    private lateinit var elevatorRight: Motor

    private lateinit var lowCommand: InstantCommand
    private lateinit var midCommand: InstantCommand
    private lateinit var highCommand: InstantCommand
    override fun initialize() {
        elevatorLeft = Motor(hardwareMap, ControlBoard.ELEVATOR_LEFT.deviceName)
        elevatorRight = Motor(hardwareMap, ControlBoard.ELEVATOR_RIGHT.deviceName)

        elevatorSubsystem = ElevatorSubsystem(elevatorLeft, elevatorRight)

        lowCommand = InstantCommand({ elevatorSubsystem.setpoint = 500.0 })
        midCommand = InstantCommand({ elevatorSubsystem.setpoint = 1000.0 })
        highCommand = InstantCommand({ elevatorSubsystem.setpoint = 2400.0 })
    }
}