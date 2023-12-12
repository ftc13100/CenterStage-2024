package org.firstinspires.ftc.teamcode.subsystems.intake

import com.arcrobotics.ftclib.command.SubsystemBase
import com.arcrobotics.ftclib.hardware.motors.Motor

class IntakeSubsystem(
    private val intake: Motor,
) : SubsystemBase() {
    fun intake() = intake.set(0.4)

    fun outtake() = intake.set(-0.4)

    fun stop() = intake.stopMotor()
}