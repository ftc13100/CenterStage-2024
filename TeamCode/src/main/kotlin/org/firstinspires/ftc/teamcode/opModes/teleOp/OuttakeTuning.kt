package org.firstinspires.ftc.teamcode.opModes.teleOp

import com.acmerobotics.dashboard.config.Config
import com.arcrobotics.ftclib.command.CommandOpMode
import com.arcrobotics.ftclib.command.RunCommand
import com.arcrobotics.ftclib.gamepad.GamepadEx
import com.arcrobotics.ftclib.hardware.motors.Motor
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.Servo
import com.qualcomm.robotcore.hardware.TouchSensor
import org.firstinspires.ftc.teamcode.constants.ControlBoard
import org.firstinspires.ftc.teamcode.subsystems.elevator.OpenElevatorSubsystem

@Config
@TeleOp
class OuttakeTuning() : CommandOpMode() {
    private lateinit var flipperServo: Servo
    private lateinit var elevatorSubsystem: OpenElevatorSubsystem

    private lateinit var elevatorLeft: Motor
    private lateinit var elevatorRight: Motor

    private lateinit var servoLeft: Servo
    private lateinit var servoRight: Servo

    private lateinit var limit: TouchSensor

    private lateinit var driver: GamepadEx
    private lateinit var operator: GamepadEx
    override fun initialize() {
        elevatorLeft = Motor(hardwareMap, ControlBoard.ELEVATOR_LEFT.deviceName)
        elevatorRight = Motor(hardwareMap, ControlBoard.ELEVATOR_RIGHT.deviceName)

        servoLeft = hardwareMap.get(Servo::class.java, ControlBoard.SERVO_ELEVATOR_LEFT.deviceName)
        servoRight =
            hardwareMap.get(Servo::class.java, ControlBoard.SERVO_ELEVATOR_RIGHT.deviceName)

        flipperServo = hardwareMap.get(Servo::class.java, "flipperServo")


        limit = hardwareMap.get(TouchSensor::class.java, ControlBoard.LIMIT_SWITCH.deviceName)

        elevatorSubsystem = OpenElevatorSubsystem(
            elevatorLeft, elevatorRight, flipperServo, limit, telemetry, servoLeft, servoRight
        )

        driver = GamepadEx(gamepad1)
        operator = GamepadEx(gamepad2)

        RunCommand({
            elevatorSubsystem.runFlipper(position)
        }).schedule()
    }

    companion object {
        @JvmField
        var position = 0.0
    }
}