package org.firstinspires.ftc.teamcode.opModes.tuning.elevator

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.config.Config
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry
import com.acmerobotics.roadrunner.profile.MotionProfileGenerator
import com.acmerobotics.roadrunner.profile.MotionState
import com.acmerobotics.roadrunner.util.NanoClock
import com.arcrobotics.ftclib.controller.wpilibcontroller.ElevatorFeedforward
import com.arcrobotics.ftclib.hardware.motors.Motor
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.hardware.CRServo
import com.qualcomm.robotcore.hardware.Servo
import com.qualcomm.robotcore.hardware.TouchSensor
import org.firstinspires.ftc.teamcode.constants.ControlBoard
import org.firstinspires.ftc.teamcode.constants.ElevatorConstants
import org.firstinspires.ftc.teamcode.subsystems.elevator.OpenElevatorSubsystem

@Config
@Autonomous
class ElevatorFeedforwardTuner : LinearOpMode() {
    private lateinit var leftMotor: Motor
    private lateinit var rightMotor: Motor

    private lateinit var leftServo: CRServo
    private lateinit var rightServo: CRServo
    private lateinit var flipperServo: Servo

    private lateinit var limit: TouchSensor

    private lateinit var subsystem: OpenElevatorSubsystem

    override fun runOpMode() {
        telemetry = MultipleTelemetry(telemetry, FtcDashboard.getInstance().telemetry)

        leftMotor = Motor(hardwareMap, ControlBoard.ELEVATOR_LEFT.deviceName)
        rightMotor = Motor(hardwareMap, ControlBoard.ELEVATOR_RIGHT.deviceName)

        leftServo = hardwareMap.get(CRServo::class.java, ControlBoard.SERVO_ELEVATOR_LEFT.deviceName)
        rightServo =
            hardwareMap.get(CRServo::class.java, ControlBoard.SERVO_ELEVATOR_RIGHT.deviceName)
        flipperServo = hardwareMap.get(Servo::class.java, "flipperServo")

        limit = hardwareMap.get(TouchSensor::class.java, ControlBoard.LIMIT_SWITCH.deviceName)

        subsystem =
            OpenElevatorSubsystem(leftMotor, rightMotor, flipperServo, limit, telemetry, leftServo, rightServo)

        val clock = NanoClock.system()

        waitForStart()

        if (isStopRequested) return

        var forward = true
        var activeProfile = generateProfile(true)
        var profileStart = clock.seconds()

        while (!isStopRequested) {
            val profileTime = clock.seconds() - profileStart
            val feedforward = ElevatorFeedforward(kS, kG, kV, kA)

            if (profileTime > activeProfile.duration()) {
                forward = !forward
                activeProfile = generateProfile(forward)
                profileStart = clock.seconds()
            }

            val motionState = activeProfile[profileTime]
            val targetPower = feedforward.calculate(motionState.v, motionState.a)

            subsystem.setPower(targetPower)
            val currentVelocity = subsystem.currentVel

            //update telemetry
            telemetry.addData("targetVelocity", motionState.v)
            telemetry.addData("measuredVelocity", currentVelocity)
            telemetry.addData("error", motionState.v - currentVelocity)

            telemetry.update()
        }
    }

    companion object {
        @JvmField
        var DISTANCE = 2000.0 // encoder ticks

        @JvmStatic
        fun generateProfile(forward: Boolean) = MotionProfileGenerator.generateSimpleMotionProfile(
            start = if (forward) MotionState(
                0.0, 0.0
            ) else MotionState(
                DISTANCE, 0.0
            ),
            goal = if (forward) MotionState(
                DISTANCE, 0.0, 0.0
            ) else MotionState(
                0.0, 0.0
            ),
            maxVel = ElevatorConstants.Constraints.MAX_ACCEL.value,
            maxAccel = ElevatorConstants.Constraints.MAX_ACCEL.value
        )

        @JvmField
        var kG = 0.0

        @JvmField
        var kS = 0.0

        @JvmField
        var kV = 0.0

        @JvmField
        var kA = 0.0
    }
}