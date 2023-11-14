package org.firstinspires.ftc.teamcode.subsystems.elevator

import com.arcrobotics.ftclib.command.SubsystemBase
import com.arcrobotics.ftclib.hardware.motors.Motor
import com.arcrobotics.ftclib.hardware.motors.MotorGroup
import com.qualcomm.robotcore.hardware.Servo
import com.qualcomm.robotcore.hardware.TouchSensor
import org.firstinspires.ftc.robotcore.external.Telemetry
import org.firstinspires.ftc.teamcode.utils.ServoGroup

class OpenElevatorSubsystem(
    motorLeft: Motor,
    motorRight: Motor,
    private val limit: TouchSensor,
    val telemetry: Telemetry,
    servoLeft: Servo,
    servoRight: Servo,
) : SubsystemBase() {
    private val elevatorServos = ServoGroup(servoLeft, servoRight)

    private val elevatorMotors = MotorGroup(motorLeft, motorRight)

    val flipped
        get() = elevatorServos.position == 1.0


    init {
        elevatorMotors.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE)
        elevatorMotors.resetEncoder()
//        ElevatorMotor.inverted = true
    }

    fun spinUp() {
        elevatorMotors.set(1.0)
        val current = elevatorMotors.currentPosition
        telemetry.addData("Position", current)
        telemetry.update()

    }

    fun getPosition(): Double = elevatorMotors.currentPosition.toDouble()

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