package org.firstinspires.ftc.teamcode.subsystems.elevator
import com.arcrobotics.ftclib.controller.PIDFController
import com.arcrobotics.ftclib.hardware.motors.Motor
import com.arcrobotics.ftclib.hardware.motors.MotorGroup
import org.firstinspires.ftc.teamcode.utils.PIDSubsystem

class ElevatorSubsystem(
    elevatorLeft: Motor,
    elevatorRight: Motor
) : PIDSubsystem(
    PIDFController(
        0.0,
        0.0,
        0.0,
        0.0
    )
) {
    private val elevatorMotors = MotorGroup(elevatorLeft, elevatorRight)
    override fun useOutput(output: Double, setpoint: Double) = elevatorMotors.set(output)

    override fun getMeasurement(): Double = elevatorMotors.currentPosition.toDouble()

}
