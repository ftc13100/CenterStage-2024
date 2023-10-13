package org.firstinspires.ftc.teamcode.subsystems.elevator

import com.arcrobotics.ftclib.controller.PIDFController
import com.arcrobotics.ftclib.hardware.motors.Motor
import org.firstinspires.ftc.teamcode.utils.PIDSubsystem

class ElevatorSubsystem(
    private val elevatorMotor: Motor
) : PIDSubsystem(
    PIDFController(
        0.0,
        0.0,
        0.0,
        0.0
    )
) {
    override fun useOutput(output: Double, setpoint: Double) = elevatorMotor.set(output)

    override fun getMeasurement(): Double = elevatorMotor.currentPosition.toDouble()


}