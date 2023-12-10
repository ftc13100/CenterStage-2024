package org.firstinspires.ftc.teamcode.subsystems.elevator

import com.arcrobotics.ftclib.command.SubsystemBase
import com.arcrobotics.ftclib.hardware.motors.Motor
import com.arcrobotics.ftclib.hardware.motors.MotorGroup
import com.qualcomm.robotcore.hardware.Servo
import com.qualcomm.robotcore.hardware.TouchSensor
import org.firstinspires.ftc.robotcore.external.Telemetry

class OpenElevatorSubsystem(
    leftMotor: Motor,
    rightMotor: Motor,
    private val limit: TouchSensor,
    val telemetry: Telemetry,
    private val leftServo: Servo,
    private val rightServo: Servo,
) : SubsystemBase() {

    private val elevatorMotors = MotorGroup(leftMotor, rightMotor)

    val currentPos: Double
        get() = elevatorMotors.positions[0]

    val currentVel: Double
        get() = elevatorMotors.velocities[0]


    init {
        leftMotor.inverted = true
//        leftServo.direction = Servo.Direction.REVERSE
//        rightServo.direction = Servo.Direction.REVERSE

        elevatorMotors.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE)
        elevatorMotors.resetEncoder()
//        ElevatorMotor.inverted = true
    }

    fun spinUp() {
        elevatorMotors.set(0.6)
    }

    fun setPower(power: Double) = elevatorMotors.set(power)

    fun spinDown() {
        if (!isPressed()) {
            elevatorMotors.set(-0.6)
        }
    }

    fun isPressed(): Boolean {
        return limit.isPressed
    }

    fun stallSpin() = elevatorMotors.stopMotor()

    fun flipOuttake(position: Double) {
        rightServo.position = position
        leftServo.position = 1 - position
    }
}