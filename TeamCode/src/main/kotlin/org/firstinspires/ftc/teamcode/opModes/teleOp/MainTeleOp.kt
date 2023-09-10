package org.firstinspires.ftc.teamcode.opModes.teleOp

import com.arcrobotics.ftclib.command.CommandOpMode
import com.arcrobotics.ftclib.gamepad.GamepadEx
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.commands.drive.DriveCommand
import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive
import org.firstinspires.ftc.teamcode.subsystems.drive.DriveSubsystem

@TeleOp
class MainTeleOp: CommandOpMode() {
    private lateinit var driveSubsystem: DriveSubsystem
    private lateinit var driveCommand: DriveCommand

    private lateinit var driver: GamepadEx
    override fun initialize() {
        driveSubsystem = DriveSubsystem(SampleMecanumDrive(hardwareMap), false)

        driver = GamepadEx(gamepad1)

        driveCommand = DriveCommand(driveSubsystem, leftX = driver::getLeftX, leftY = driver::getLeftY, rightX = driver::getRightX)

        register(driveSubsystem)

        driveSubsystem.defaultCommand = driveCommand
    }
}