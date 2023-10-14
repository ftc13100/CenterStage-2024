package org.firstinspires.ftc.teamcode.utils

import com.arcrobotics.ftclib.command.SubsystemBase
import com.arcrobotics.ftclib.controller.wpilibcontroller.ProfiledPIDController
import com.arcrobotics.ftclib.trajectory.TrapezoidProfile

abstract class ProfiledPIDSubsystem(
    private val controller: ProfiledPIDController
) : SubsystemBase() {
    private var enabled: Boolean = true
    private var goal = controller.goal
        set(value) {
            controller.goal = value
            field = value
        }

    protected abstract fun useOutput(output: Double, setpoint: TrapezoidProfile.State)

    protected abstract fun getMeasurement(): Double

    override fun periodic() {
        if (enabled) {
            useOutput(controller.calculate(getMeasurement()), goal)
        }
    }

    fun enable() {
        enabled  = true
        controller.reset(getMeasurement())
    }

    fun disable() {
        enabled = false
        useOutput(0.0, TrapezoidProfile.State())
    }

    fun setGoal(goal: Double) {
        this.goal = TrapezoidProfile.State(goal, 0.0)
    }
    fun isEnabled(): Boolean {
        return enabled
    }
}