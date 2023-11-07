package org.firstinspires.ftc.teamcode.commands.drive

import com.acmerobotics.roadrunner.geometry.Pose2d
import com.arcrobotics.ftclib.command.CommandBase
import org.firstinspires.ftc.teamcode.subsystems.drive.DriveSubsystem
import org.firstinspires.ftc.teamcode.subsystems.vision.VisionSubsystem
import java.util.function.Supplier

class DriveToTagCommand(
    private val targetTag: Int,
    private val targetPose: Supplier<Pose2d>,
    private val driveSubsystem: DriveSubsystem,
    private val visionSubsystem: VisionSubsystem
) : CommandBase() {
    private var atTarget = false

    init {
        addRequirements(driveSubsystem, visionSubsystem)
    }
    override fun initialize() {
        visionSubsystem.targetId = targetTag
    }

    override fun execute() {
        atTarget = driveSubsystem.driveToTag(targetPose.get())
    }

    override fun isFinished(): Boolean = atTarget
}