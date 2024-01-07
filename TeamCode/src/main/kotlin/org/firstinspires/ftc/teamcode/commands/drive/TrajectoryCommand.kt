package org.firstinspires.ftc.teamcode.commands.drive

import com.acmerobotics.roadrunner.geometry.Pose2d
import com.arcrobotics.ftclib.command.CommandBase
import org.firstinspires.ftc.teamcode.subsystems.drive.DriveSubsystem
import org.firstinspires.ftc.teamcode.utils.roadrunner.trajectorysequence.TrajectorySequence

class TrajectoryCommand(
    private val startPose: () -> Pose2d,
    private val drive: DriveSubsystem,
    private val trajectoryToRun: (startPose: Pose2d) -> TrajectorySequence,
) : CommandBase() {
    private lateinit var trajectory: TrajectorySequence

    init {
        addRequirements(drive)
    }

    override fun initialize() {
        trajectory = trajectoryToRun.invoke(
            startPose.invoke()
        )

        drive.poseEstimate = trajectory.start()
        drive.followTrajectorySequenceAsync(trajectory)
    }

    override fun execute() {
        drive.update()
    }

    override fun isFinished(): Boolean = !drive.isBusy
}