package org.firstinspires.ftc.teamcode.opModes.teleOp.tuning.elevator

import com.acmerobotics.dashboard.config.Config
import com.arcrobotics.ftclib.command.CommandOpMode
import com.arcrobotics.ftclib.command.RunCommand
import com.arcrobotics.ftclib.hardware.motors.Motor
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.Servo
import org.firstinspires.ftc.teamcode.constants.ControlBoard
import org.firstinspires.ftc.teamcode.subsystems.elevator.ElevatorSubsystem

@Config
@TeleOp
class ElevatorPIDTuner : CommandOpMode() {
    private lateinit var elevatorLeft: Motor
    private lateinit var elevatorRight: Motor
    private lateinit var servoLeft: Servo
    private lateinit var servoRight: Servo

    private lateinit var elevatorSubsystem: ElevatorSubsystem

    override fun initialize() {
        elevatorLeft = Motor(hardwareMap, ControlBoard.ELEVATOR_LEFT.deviceName)
        elevatorRight = Motor(hardwareMap, ControlBoard.ELEVATOR_RIGHT.deviceName)

        servoLeft = hardwareMap.get(Servo::class.java, ControlBoard.SERVO_ELEVATOR_LEFT.deviceName)
        servoRight =
            hardwareMap.get(Servo::class.java, ControlBoard.SERVO_ELEVATOR_RIGHT.deviceName)

        elevatorSubsystem = ElevatorSubsystem(elevatorLeft, elevatorRight, servoLeft, servoRight)

        RunCommand({ elevatorSubsystem.setpoint = TARGET }).perpetually().schedule()
    }

    companion object {
        @JvmField
        var TARGET = 0.0
    }
}