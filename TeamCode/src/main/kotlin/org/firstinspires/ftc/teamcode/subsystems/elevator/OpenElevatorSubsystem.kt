package org.firstinspires.ftc.teamcode.subsystems.elevator

import com.arcrobotics.ftclib.command.SubsystemBase
import com.arcrobotics.ftclib.hardware.motors.Motor
import com.arcrobotics.ftclib.hardware.motors.MotorGroup
import com.qualcomm.robotcore.hardware.Servo
import com.qualcomm.robotcore.hardware.TouchSensor
import org.firstinspires.ftc.robotcore.external.Telemetry
import org.firstinspires.ftc.teamcode.utils.ServoGroup

class OpenElevatorSubsystem(
    leftMotor: Motor,
    rightMotor: Motor,
    private val limit: TouchSensor,
    val telemetry: Telemetry,
    leftServo: Servo,
    rightServo: Servo,
) : SubsystemBase() {
    private val elevatorServos = ServoGroup(leftServo, rightServo)

    private val elevatorMotors = MotorGroup(leftMotor, rightMotor)

    val currentPos: Double
        get() = elevatorMotors.positions[0]

    val currentVel: Double
        get() = elevatorMotors.velocities[0]

    val flipped: Boolean
        get() = elevatorServos.position == 1.0


    init {
        elevatorMotors.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE)
        elevatorMotors.resetEncoder()
//        ElevatorMotor.inverted = true
    }

    fun spinUp() {
        elevatorMotors.set(1.0)
    }

    fun setPower(power: Double) = elevatorMotors.set(power)

    fun spinDown() {
        if (!isPressed()) {
            elevatorMotors.set(-1.0)
        }
    }

    fun isPressed(): Boolean {
        return limit.isPressed
    }

    fun stallSpin() = elevatorMotors.stopMotor()

    fun flipOuttake() =
        if (flipped) elevatorServos.position = 0.0 else elevatorServos.position = 1.0
}