package org.firstinspires.ftc.teamcode.subsystems.elevator

import com.arcrobotics.ftclib.controller.wpilibcontroller.ElevatorFeedforward
import com.arcrobotics.ftclib.controller.wpilibcontroller.ProfiledPIDController
import com.arcrobotics.ftclib.hardware.motors.Motor
import com.arcrobotics.ftclib.hardware.motors.MotorGroup
import com.arcrobotics.ftclib.trajectory.TrapezoidProfile
import org.firstinspires.ftc.teamcode.constants.ElevatorConstants
import org.firstinspires.ftc.teamcode.utils.ProfiledPIDSubsystem

class ProfiledElevatorSubsystem(
    elevatorLeft: Motor,
    elevatorRight: Motor,
) : ProfiledPIDSubsystem(
    ProfiledPIDController(
        ElevatorConstants.PIDF.P.value,
        ElevatorConstants.PIDF.I.value,
        ElevatorConstants.PIDF.D.value,
        TrapezoidProfile.Constraints(
            ElevatorConstants.Constraints.MAX_VEL.value,
            ElevatorConstants.Constraints.MAX_ACCEL.value
        )
    )
) {
    private val elevatorMotors = MotorGroup(elevatorLeft, elevatorRight)

    private val feedforward = ElevatorFeedforward(
        ElevatorConstants.Feedforward.S.value,
        ElevatorConstants.Feedforward.G.value,
        ElevatorConstants.Feedforward.V.value,
    )

    override fun useOutput(output: Double, setpoint: TrapezoidProfile.State) =
        elevatorMotors.set(output + feedforward.calculate(setpoint.velocity))

    override fun getMeasurement(): Double = elevatorMotors.positions[0]
}