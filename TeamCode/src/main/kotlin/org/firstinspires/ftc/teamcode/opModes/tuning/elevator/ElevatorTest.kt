package org.firstinspires.ftc.teamcode.opModes.tuning.elevator

import com.arcrobotics.ftclib.command.CommandOpMode
import com.arcrobotics.ftclib.command.InstantCommand
import com.arcrobotics.ftclib.gamepad.GamepadEx
import com.arcrobotics.ftclib.gamepad.GamepadKeys
import com.arcrobotics.ftclib.hardware.motors.Motor
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.Servo
import org.firstinspires.ftc.teamcode.constants.ControlBoard
import org.firstinspires.ftc.teamcode.subsystems.elevator.ElevatorSubsystem

@TeleOp
class ElevatorTest : CommandOpMode() {
    private lateinit var elevatorSubsystem: ElevatorSubsystem

    private lateinit var driver: GamepadEx

    private lateinit var elevatorLeft: Motor
    private lateinit var elevatorRight: Motor

    private lateinit var servoLeft: Servo
    private lateinit var servoRight: Servo

    private lateinit var groundCommand: InstantCommand
    private lateinit var lowCommand: InstantCommand
    private lateinit var midCommand: InstantCommand
    private lateinit var highCommand: InstantCommand
    override fun initialize() {
        elevatorLeft = Motor(hardwareMap, ControlBoard.ELEVATOR_LEFT.deviceName)
        elevatorRight = Motor(hardwareMap, ControlBoard.ELEVATOR_RIGHT.deviceName)

        servoLeft = hardwareMap.get(Servo::class.java, ControlBoard.SERVO_ELEVATOR_LEFT.deviceName)
        servoRight =
            hardwareMap.get(Servo::class.java, ControlBoard.SERVO_ELEVATOR_RIGHT.deviceName)

        elevatorSubsystem = ElevatorSubsystem(elevatorLeft, elevatorRight, servoLeft, servoRight)

        groundCommand = InstantCommand({ elevatorSubsystem.setpoint = 0.0 })
        lowCommand = InstantCommand({ elevatorSubsystem.setpoint = 500.0 })
        midCommand = InstantCommand({ elevatorSubsystem.setpoint = 1000.0 })
        highCommand = InstantCommand({ elevatorSubsystem.setpoint = 2400.0 })

        driver = GamepadEx(gamepad1)

        driver.getGamepadButton(GamepadKeys.Button.DPAD_UP).whenPressed(highCommand)
        driver.getGamepadButton(GamepadKeys.Button.DPAD_LEFT).whenPressed(midCommand)
        driver.getGamepadButton(GamepadKeys.Button.DPAD_RIGHT).whenPressed(lowCommand)
        driver.getGamepadButton(GamepadKeys.Button.DPAD_DOWN).whenPressed(groundCommand)
    }
}