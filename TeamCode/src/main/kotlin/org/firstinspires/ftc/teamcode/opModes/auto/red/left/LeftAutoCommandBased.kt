package org.firstinspires.ftc.teamcode.opModes.auto.red.left

import com.arcrobotics.ftclib.command.CommandOpMode
import com.arcrobotics.ftclib.command.FunctionalCommand
import com.arcrobotics.ftclib.command.SequentialCommandGroup
import org.firstinspires.ftc.teamcode.subsystems.drive.DriveSubsystem

class LeftAutoCommandBased : CommandOpMode() {
    private lateinit var drive: DriveSubsystem
    override fun initialize() {
        drive = DriveSubsystem(hardwareMap)

        schedule(
            SequentialCommandGroup(
                FunctionalCommand(
                    {},
                    {},
                    {},
                    drive::isBusy
                )
            )
        )
    }
}