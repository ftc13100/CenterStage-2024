package org.firstinspires.ftc.teamcode.opModes.auto.red.park

import com.arcrobotics.ftclib.hardware.motors.Motor
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.OpMode
import org.firstinspires.ftc.teamcode.constants.AutoStartPose
import org.firstinspires.ftc.teamcode.constants.ControlBoard
import org.firstinspires.ftc.teamcode.subsystems.drive.DriveSubsystem
import org.firstinspires.ftc.teamcode.subsystems.intake.IntakeSubsystem
import org.firstinspires.ftc.teamcode.utils.roadrunner.drive.DriveConstants

@Autonomous(name = "Red Park (Right)", preselectTeleOp = "MainTeleOp")
class ParkRightStart : OpMode() {
    private lateinit var intakeMotor: Motor

    private lateinit var driveSubsystem: DriveSubsystem
    private lateinit var intakeSubsystem: IntakeSubsystem
    override fun init() {
        intakeMotor = Motor(hardwareMap, ControlBoard.INTAKE.deviceName)

        driveSubsystem = DriveSubsystem(hardwareMap)
        intakeSubsystem = IntakeSubsystem(intakeMotor)

        val trajectory = driveSubsystem.trajectorySequenceBuilder(AutoStartPose.RED_RIGHT.startPose)
            .strafeLeft(2.0)
            .forward(42.0,
                DriveSubsystem.getVelocityConstraint(DriveConstants.MAX_VEL, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                DriveSubsystem.getAccelerationConstraint(50.0)
            )
            .build()

        driveSubsystem.poseEstimate = AutoStartPose.RED_RIGHT.startPose
        driveSubsystem.followTrajectorySequenceAsync(trajectory)
    }

    override fun loop() {
        driveSubsystem.update()
    }
}