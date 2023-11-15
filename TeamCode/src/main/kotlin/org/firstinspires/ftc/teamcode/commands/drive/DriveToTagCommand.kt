package org.firstinspires.ftc.teamcode.commands.drive

import com.acmerobotics.roadrunner.geometry.Pose2d
import com.arcrobotics.ftclib.command.CommandBase
import org.firstinspires.ftc.teamcode.subsystems.drive.DriveSubsystem
import org.firstinspires.ftc.teamcode.subsystems.vision.VisionSubsystem

class DriveToTagCommand(
    private val targetTag: Int,
    private val driveSubsystem: DriveSubsystem,
    private val visionSubsystem: VisionSubsystem,
    private val targetPose: () -> Pose2d?,
) : CommandBase() {
    private var atTarget = false

    init {
        addRequirements(driveSubsystem, visionSubsystem)
    }

    override fun initialize() {
        visionSubsystem.targetId = targetTag
    }

    override fun execute() {
        atTarget = driveSubsystem.driveToTag(targetPose.invoke())
    }

    override fun isFinished(): Boolean = atTarget
}