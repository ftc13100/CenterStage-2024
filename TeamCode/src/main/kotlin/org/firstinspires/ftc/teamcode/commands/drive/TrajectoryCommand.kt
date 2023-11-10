package org.firstinspires.ftc.teamcode.commands.drive

import com.acmerobotics.roadrunner.geometry.Pose2d
import com.arcrobotics.ftclib.command.CommandBase
import org.firstinspires.ftc.teamcode.subsystems.drive.DriveSubsystem
import org.firstinspires.ftc.teamcode.utils.roadrunner.trajectorysequence.TrajectorySequence
import org.firstinspires.ftc.teamcode.utils.roadrunner.trajectorysequence.TrajectorySequenceBuilder

class TrajectoryCommand(
    private val trajectoryToRun: (startPose: Pose2d) -> TrajectorySequence,
    private val startPose: Pose2d,
    private val drive: DriveSubsystem
) : CommandBase() {
    private lateinit var trajectory: TrajectorySequence

    init {
        addRequirements(drive)
    }

    override fun initialize() {
        trajectory = trajectoryToRun.invoke(startPose)
    }

    override fun execute() {
        drive.followTrajectorySequenceAsync(trajectory)
    }

    override fun isFinished(): Boolean = !drive.isBusy
}