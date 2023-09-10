package org.firstinspires.ftc.teamcode.utils.drive

import com.acmerobotics.roadrunner.control.PIDCoefficients
import com.acmerobotics.roadrunner.drive.MecanumDrive
import com.acmerobotics.roadrunner.followers.HolonomicPIDVAFollower
import com.acmerobotics.roadrunner.followers.TrajectoryFollower
import com.acmerobotics.roadrunner.geometry.Pose2d
import com.arcrobotics.ftclib.command.Subsystem
import com.qualcomm.hardware.lynx.LynxModule
import com.qualcomm.robotcore.hardware.*
import org.firstinspires.ftc.teamcode.constants.ControlBoard.*
import org.firstinspires.ftc.teamcode.roadrunner.drive.DriveConstants.*
import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive
import org.firstinspires.ftc.teamcode.roadrunner.drive.StandardTrackingWheelLocalizer
import org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence.TrajectorySequenceRunner
import org.firstinspires.ftc.teamcode.roadrunner.util.LynxModuleUtil

class BeaverMecanumDrive(
    hardwareMap: HardwareMap,
) : MecanumDrive(
    kV,
    kA,
    kStatic,
    trackWidth = TRACK_WIDTH,
    lateralMultiplier = LATERAL_MULTIPLIER
), Subsystem {

    private val leftFront: DcMotorEx = hardwareMap[DcMotorEx::class.java, DRIVE_LEFT_FRONT.deviceName]
    private val leftRear: DcMotorEx = hardwareMap[DcMotorEx::class.java, DRIVE_LEFT_REAR.deviceName]
    private val rightFront: DcMotorEx = hardwareMap[DcMotorEx::class.java, DRIVE_RIGHT_FRONT.deviceName]
    private val rightRear: DcMotorEx = hardwareMap[DcMotorEx::class.java, DRIVE_RIGHT_REAR.deviceName]

    private val motors: List<DcMotorEx>  = listOf<DcMotorEx>(leftFront, leftRear, rightRear, rightFront)

    private var trajectorySequenceRunner: TrajectorySequenceRunner? = null

    private val VEL_CONSTRAINT = SampleMecanumDrive.getVelocityConstraint(MAX_VEL, MAX_ANG_VEL, TRACK_WIDTH)
    private val ACCEL_CONSTRAINT = SampleMecanumDrive.getAccelerationConstraint(MAX_ACCEL)

    private val batteryVoltageSensor: VoltageSensor

    init {
        val follower: TrajectoryFollower = HolonomicPIDVAFollower(
            SampleMecanumDrive.TRANSLATIONAL_PID,
            SampleMecanumDrive.TRANSLATIONAL_PID,
            SampleMecanumDrive.HEADING_PID,
            Pose2d(0.5, 0.5, Math.toRadians(5.0)),
            0.5
        )
        LynxModuleUtil.ensureMinimumFirmwareVersion(hardwareMap)
        batteryVoltageSensor = hardwareMap.voltageSensor.iterator().next()

        for (module in hardwareMap.getAll(LynxModule::class.java)) {
            module.bulkCachingMode = LynxModule.BulkCachingMode.AUTO
        }

        for (motor in motors) {
            val motorConfigurationType = motor.motorType.clone()
            motorConfigurationType.achieveableMaxRPMFraction = 1.0
            motor.motorType = motorConfigurationType
        }

        if (RUN_USING_ENCODER) {
            motors.forEach { it.mode = DcMotor.RunMode.RUN_USING_ENCODER }
        }

        motors.forEach { it.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE }

        if (RUN_USING_ENCODER && MOTOR_VELO_PID != null) {
            motors.forEach { it.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, PIDFCoefficients(
                MOTOR_VELO_PID.p, MOTOR_VELO_PID.i, MOTOR_VELO_PID.d, MOTOR_VELO_PID.f * 12/(batteryVoltageSensor.voltage))
            )}
        }
        // TODO: reverse any motors using DcMotor.setDirection()

        localizer = StandardTrackingWheelLocalizer(hardwareMap)
        trajectorySequenceRunner =
            TrajectorySequenceRunner(follower, SampleMecanumDrive.HEADING_PID)
    }

    override val rawExternalHeading: Double
        get() = 0.0

    override fun getWheelPositions(): List<Double> {
        TODO("Not yet implemented")
    }

    override fun setMotorPowers(frontLeft: Double, rearLeft: Double, rearRight: Double, frontRight: Double) {
        TODO("Not yet implemented")
    }

    companion object {
        @JvmField
        var LATERAL_MULTIPLIER = 1.8

        @JvmField
        var TRANSLATIONAL_PID = PIDCoefficients(0.0145, 0.0, 0.0)

        @JvmField
        var HEADING_PID = PIDCoefficients(0.0000000451028, 0.00007, 0.0)

        @JvmField
        var VX_WEIGHT = 1.0

        @JvmField
        var VY_WEIGHT = 1.0

        @JvmField
        var OMEGA_WEIGHT = 1.0
    }
}
