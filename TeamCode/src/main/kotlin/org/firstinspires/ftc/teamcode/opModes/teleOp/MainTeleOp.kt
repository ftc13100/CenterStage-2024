package org.firstinspires.ftc.teamcode.opModes.teleOp

import com.arcrobotics.ftclib.command.CommandOpMode
import com.arcrobotics.ftclib.gamepad.GamepadEx
import com.arcrobotics.ftclib.gamepad.GamepadKeys
import com.arcrobotics.ftclib.hardware.motors.Motor
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.commands.drive.DriveCommand
import org.firstinspires.ftc.teamcode.commands.intake.IntakeCommand
import org.firstinspires.ftc.teamcode.constants.ControlBoard
import org.firstinspires.ftc.teamcode.subsystems.drive.DriveSubsystem
import org.firstinspires.ftc.teamcode.subsystems.intake.IntakeSubsystem

@TeleOp
class MainTeleOp: CommandOpMode() {
    private lateinit var driveSubsystem: DriveSubsystem
    private lateinit var intakeSubsystem: IntakeSubsystem

    private lateinit var driveCommand: DriveCommand
    private lateinit var intakeCommand: IntakeCommand
    private lateinit var outtakeCommand: IntakeCommand

    private lateinit var intake: Motor

    private lateinit var driver: GamepadEx
    override fun initialize() {
        intake = Motor(hardwareMap, ControlBoard.INTAKE.deviceName)

        driveSubsystem = DriveSubsystem(hardwareMap)
        intakeSubsystem = IntakeSubsystem(intake)

        driver = GamepadEx(gamepad1)

        intakeCommand = IntakeCommand(intakeSubsystem, intake = true)
        outtakeCommand = IntakeCommand(intakeSubsystem, intake = false)

        driveCommand = DriveCommand(driveSubsystem, leftX = driver::getLeftX, leftY = driver::getLeftY, rightX = driver::getRightX, zoneVal = 0.15)

        driver.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER).whileHeld(intakeCommand)
        driver.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER).whileHeld(outtakeCommand)

        register(driveSubsystem)

        driveSubsystem.defaultCommand = driveCommand
    }
}