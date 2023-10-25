package org.firstinspires.ftc.teamcode.opModes.auto.red.left

import com.arcrobotics.ftclib.command.CommandOpMode
import com.arcrobotics.ftclib.command.FunctionalCommand
import com.arcrobotics.ftclib.command.SequentialCommandGroup
import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive

class LeftAutoCommandBased: CommandOpMode() {
    private lateinit var drive: SampleMecanumDrive
    override fun initialize() {
        drive = SampleMecanumDrive(hardwareMap)

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