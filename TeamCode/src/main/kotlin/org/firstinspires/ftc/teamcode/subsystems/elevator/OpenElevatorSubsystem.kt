package org.firstinspires.ftc.teamcode.subsystems.elevator
import com.arcrobotics.ftclib.command.SubsystemBase
import com.arcrobotics.ftclib.hardware.motors.Motor
import com.qualcomm.robotcore.hardware.TouchSensor
import org.firstinspires.ftc.robotcore.external.Telemetry

class OpenElevatorSubsystem(
    private val elevatorMotor: Motor,
    private val limit: TouchSensor,
    val telemetry: Telemetry,
) : SubsystemBase() {
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
}