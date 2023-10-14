package org.firstinspires.ftc.teamcode.subsystems.elevator

import com.arcrobotics.ftclib.controller.wpilibcontroller.ProfiledPIDController
import com.arcrobotics.ftclib.hardware.motors.Motor
import com.arcrobotics.ftclib.trajectory.TrapezoidProfile
import org.firstinspires.ftc.teamcode.utils.ProfiledPIDSubsystem

class ProfiledElevatorSubsystem(
    private val elevatorMotor: Motor
) : ProfiledPIDSubsystem(
    ProfiledPIDController(
        0.0,
        0.0,
        0.0,
        TrapezoidProfile.Constraints(

        )
    )
) {
    override fun useOutput(output: Double, setpoint: TrapezoidProfile.State) = elevatorMotor.set(output)

    override fun getMeasurement(): Double = elevatorMotor.currentPosition.toDouble()

}