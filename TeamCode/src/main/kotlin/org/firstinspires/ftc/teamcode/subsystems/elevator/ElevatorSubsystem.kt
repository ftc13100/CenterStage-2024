package org.firstinspires.ftc.teamcode.subsystems.elevator

import com.acmerobotics.dashboard.config.Config
import com.arcrobotics.ftclib.controller.PIDFController
import com.arcrobotics.ftclib.controller.wpilibcontroller.ElevatorFeedforward
import com.arcrobotics.ftclib.hardware.motors.Motor
import com.arcrobotics.ftclib.hardware.motors.MotorGroup
import com.qualcomm.robotcore.hardware.Servo
import org.firstinspires.ftc.teamcode.constants.ElevatorConstants
import org.firstinspires.ftc.teamcode.utils.PIDSubsystem
import org.firstinspires.ftc.teamcode.utils.ServoGroup

@Config
class ElevatorSubsystem(
    elevatorLeft: Motor,
    elevatorRight: Motor,
    servoLeft: Servo,
    servoRight: Servo,
) : PIDSubsystem(
    PIDFController(
        ElevatorConstants.PIDF.P.value,
        ElevatorConstants.PIDF.I.value,
        ElevatorConstants.PIDF.D.value,
        ElevatorConstants.PIDF.F.value,
    )
) {
    private val elevatorMotors = MotorGroup(elevatorLeft, elevatorRight)
    private val elevatorServos = ServoGroup(servoLeft, servoRight)

    private val feedforward = ElevatorFeedforward(
        ElevatorConstants.Feedforward.S.value,
        ElevatorConstants.Feedforward.G.value,
        ElevatorConstants.Feedforward.V.value,
    )

    val atSetpoint
        get() = getMeasurement() == setpoint

    private val flipped
        get() = elevatorServos.position == 1.0

    private val currentPos: Double
        get() = elevatorMotors.positions[0]

    private val currentVel: Double
        get() = elevatorMotors.velocities[0]

    override fun useOutput(output: Double, setpoint: Double) {
        controller.setPIDF(p, i, d, f)
        elevatorMotors.set(output + feedforward.calculate(currentVel))
    }

    override fun getMeasurement(): Double = currentPos

    fun flipOuttake() =
        if (flipped) elevatorServos.position = 0.0 else elevatorServos.position = 0.0

    companion object {
        @JvmField
        var p = 0.0

        @JvmField
        var i = 0.0

        @JvmField
        var d = 0.0

        @JvmField
        var f = 0.0
    }
}
