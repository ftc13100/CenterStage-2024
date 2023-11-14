package org.firstinspires.ftc.teamcode.subsystems.elevator

import com.arcrobotics.ftclib.controller.wpilibcontroller.ProfiledPIDController
import com.arcrobotics.ftclib.hardware.motors.Motor
import com.arcrobotics.ftclib.hardware.motors.MotorGroup
import com.arcrobotics.ftclib.trajectory.TrapezoidProfile
import org.firstinspires.ftc.teamcode.utils.ProfiledPIDSubsystem

class ProfiledElevatorSubsystem(
    elevatorLeft: Motor,
    elevatorRight: Motor,
) : ProfiledPIDSubsystem(
    ProfiledPIDController(
        0.0,
        0.0,
        0.0,
        TrapezoidProfile.Constraints(

        )
    )
) {
    private val elevatorMotors = MotorGroup(elevatorLeft, elevatorRight)

    override fun useOutput(output: Double, setpoint: TrapezoidProfile.State) =
        elevatorMotors.set(output)

    override fun getMeasurement(): Double = elevatorMotors.currentPosition.toDouble()

}