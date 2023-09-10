package org.firstinspires.ftc.teamcode.commands.drive

import com.arcrobotics.ftclib.command.CommandBase
import org.firstinspires.ftc.teamcode.subsystems.drive.DriveSubsystem
import java.util.function.DoubleSupplier

class DriveCommand(
    private val subsystem: DriveSubsystem,
    private val leftX: DoubleSupplier,
    private val leftY: DoubleSupplier,
    private val rightX: DoubleSupplier
) : CommandBase() {
    override fun execute() {
        subsystem.drive(
            leftY = leftY.asDouble,
            leftX = leftX.asDouble,
            rightX = rightX.asDouble,
        )
    }
}