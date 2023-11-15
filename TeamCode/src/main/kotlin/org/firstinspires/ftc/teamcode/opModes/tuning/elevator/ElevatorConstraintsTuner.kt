package org.firstinspires.ftc.teamcode.opModes.tuning.elevator

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry
import com.arcrobotics.ftclib.hardware.motors.Motor
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.hardware.Servo
import com.qualcomm.robotcore.hardware.TouchSensor
import com.qualcomm.robotcore.util.ElapsedTime
import org.firstinspires.ftc.teamcode.constants.ControlBoard.ELEVATOR_LEFT
import org.firstinspires.ftc.teamcode.constants.ControlBoard.ELEVATOR_RIGHT
import org.firstinspires.ftc.teamcode.constants.ControlBoard.LIMIT_SWITCH
import org.firstinspires.ftc.teamcode.constants.ControlBoard.SERVO_ELEVATOR_LEFT
import org.firstinspires.ftc.teamcode.constants.ControlBoard.SERVO_ELEVATOR_RIGHT
import org.firstinspires.ftc.teamcode.subsystems.elevator.OpenElevatorSubsystem

@Autonomous
class ElevatorConstraintsTuner : LinearOpMode() {
    private lateinit var leftMotor: Motor
    private lateinit var rightMotor: Motor

    private lateinit var leftServo: Servo
    private lateinit var rightServo: Servo

    private lateinit var limit: TouchSensor

    private lateinit var elevator: OpenElevatorSubsystem

    private var lastPos = 0.0
    private var lastVel = 0.0

    private var maxVel = 0.0
    private var maxAccel = 0.0

    private val timer = ElapsedTime(ElapsedTime.Resolution.SECONDS)
    override fun runOpMode() {
        leftMotor = Motor(hardwareMap, ELEVATOR_LEFT.deviceName)
        rightMotor = Motor(hardwareMap, ELEVATOR_RIGHT.deviceName)

        leftServo = hardwareMap.get(Servo::class.java, SERVO_ELEVATOR_LEFT.deviceName)
        rightServo = hardwareMap.get(Servo::class.java, SERVO_ELEVATOR_RIGHT.deviceName)

        limit = hardwareMap.get(TouchSensor::class.java, LIMIT_SWITCH.deviceName)

        telemetry = MultipleTelemetry(telemetry, FtcDashboard.getInstance().telemetry)

        elevator =
            OpenElevatorSubsystem(leftMotor, rightMotor, limit, telemetry, leftServo, rightServo)

        waitForStart()

        var dt = 0L
        var dv = 0.0

        timer.reset()
        while (opModeIsActive()) {
            while (timer.seconds() < TIME_TO_RUN) {
                elevator.spinUp()

                dt = timer.nanoseconds() - dt
                dv = elevator.currentVel - dv / dt

                maxVel = elevator.currentVel.coerceAtLeast(maxVel)
                maxAccel = dv.coerceAtLeast(maxAccel)

                telemetry.addData("Max Velocity", maxVel)
                telemetry.addData("Max Acceleration", maxAccel)
                telemetry.update()


                if (isStopRequested) break
            }

            telemetry.addData("Final Max Velocity", maxVel)
            telemetry.addData("Final Max Acceleration", maxAccel)
            telemetry.update()
        }
    }

    companion object {
        @JvmField
        var TIME_TO_RUN = 2.0
    }
}