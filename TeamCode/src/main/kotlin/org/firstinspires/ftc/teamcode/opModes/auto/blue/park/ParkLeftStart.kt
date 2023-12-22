package org.firstinspires.ftc.teamcode.opModes.auto.blue.park

import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.OpMode
import org.firstinspires.ftc.teamcode.constants.AutoStartPose
import org.firstinspires.ftc.teamcode.subsystems.drive.DriveSubsystem

@Autonomous
class ParkLeftStart : OpMode() {
//    private lateinit var intakeMotor: Motor

    private lateinit var driveSubsystem: DriveSubsystem

    //    private lateinit var intakeSubsystem: IntakeSubsystem
    override fun init() {
//        intakeMotor = Motor(hardwareMap, ControlBoard.INTAKE.deviceName)

        driveSubsystem = DriveSubsystem(hardwareMap)
//        intakeSubsystem = IntakeSubsystem(intakeMotor)

        val trajectory = driveSubsystem.trajectorySequenceBuilder(AutoStartPose.BLUE_LEFT.startPose)
            .strafeLeft(45.0)
            .build()

        driveSubsystem.followTrajectorySequenceAsync(trajectory)
    }

    override fun loop() {
        driveSubsystem.update()
    }
}