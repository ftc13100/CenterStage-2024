package org.firstinspires.ftc.teamcode.subsystems.elevator
import com.arcrobotics.ftclib.command.SubsystemBase
import com.arcrobotics.ftclib.hardware.motors.Motor
import com.qualcomm.robotcore.hardware.Servo
import com.qualcomm.robotcore.hardware.TouchSensor
import org.firstinspires.ftc.robotcore.external.Telemetry
import org.firstinspires.ftc.teamcode.utils.ServoGroup

class OpenElevatorSubsystem(
    private val elevatorMotor: Motor,
    private val limit: TouchSensor,
    val telemetry: Telemetry,
    servoLeft: Servo,
    servoRight: Servo
) : SubsystemBase() {

    private val elevatorServos = ServoGroup(servoLeft, servoRight)
    val flipped
        get() = elevatorServos.position == 1.0

    init {
        elevatorMotor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE)
        elevatorMotor.resetEncoder()
//        ElevatorMotor.inverted = true
    }

    fun spinUp() {
        elevatorMotor.set(1.0)
        val current = elevatorMotor.currentPosition
        telemetry.addData("Position", current)
        telemetry.update()

    }

    fun getPosition(): Double = elevatorMotor.currentPosition.toDouble()

    fun spinDown() {
        if(!isPressed()) {
            elevatorMotor.set(-1.0)
        }
    }

    fun isPressed(): Boolean {
        return limit.isPressed
    }

    fun stallSpin() = elevatorMotor.stopMotor()

    fun flipOuttake() = if (flipped) elevatorServos.position = 0.0 else elevatorServos.position = 1.0
}