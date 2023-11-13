package org.firstinspires.ftc.teamcode.subsystems.elevator
import com.arcrobotics.ftclib.controller.PIDFController
import com.arcrobotics.ftclib.hardware.motors.Motor
import com.arcrobotics.ftclib.hardware.motors.MotorGroup
import com.qualcomm.robotcore.hardware.Servo
import org.firstinspires.ftc.teamcode.utils.PIDSubsystem
import org.firstinspires.ftc.teamcode.utils.ServoGroup

class ElevatorSubsystem(
    elevatorLeft: Motor,
    elevatorRight: Motor,
    servoLeft: Servo,
    servoRight: Servo
) : PIDSubsystem(
    PIDFController(
        0.0,
        0.0,
        0.0,
        0.0
    )
) {
    private val elevatorMotors = MotorGroup(elevatorLeft, elevatorRight)
    private val elevatorServos = ServoGroup(servoLeft, servoRight)

    val atSetpoint
        get() = getMeasurement() == setpoint

    private val flipped
        get() = elevatorServos.position == 1.0

    override fun useOutput(output: Double, setpoint: Double) = elevatorMotors.set(output)

    override fun getMeasurement(): Double = elevatorMotors.currentPosition.toDouble()

    fun flipOuttake() = if (flipped) elevatorServos.position = 0.0 else elevatorServos.position = 0.0
}
