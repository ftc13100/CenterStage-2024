package org.firstinspires.ftc.teamcode.opModes.teleOp

import com.arcrobotics.ftclib.command.CommandOpMode
import com.arcrobotics.ftclib.gamepad.GamepadEx
import com.arcrobotics.ftclib.gamepad.GamepadKeys
import com.arcrobotics.ftclib.hardware.motors.Motor
import com.qualcomm.hardware.lynx.LynxModule
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.Servo
import com.qualcomm.robotcore.hardware.TouchSensor
import org.firstinspires.ftc.teamcode.commands.drive.DriveCommand
import org.firstinspires.ftc.teamcode.commands.elevator.open.OpenDropElevatorCommand
import org.firstinspires.ftc.teamcode.commands.elevator.open.OpenRaiseElevatorCommand
import org.firstinspires.ftc.teamcode.commands.intake.IntakeCommand
import org.firstinspires.ftc.teamcode.constants.ControlBoard
import org.firstinspires.ftc.teamcode.subsystems.drive.DriveSubsystem
import org.firstinspires.ftc.teamcode.subsystems.elevator.OpenElevatorSubsystem
import org.firstinspires.ftc.teamcode.subsystems.intake.IntakeSubsystem

@TeleOp
class MainTeleOp : CommandOpMode() {
    private lateinit var driveSubsystem: DriveSubsystem
    private lateinit var intakeSubsystem: IntakeSubsystem
    private lateinit var elevatorSubsystem: OpenElevatorSubsystem

    private lateinit var driveCommand: DriveCommand
    private lateinit var intakeCommand: IntakeCommand
    private lateinit var outtakeCommand: IntakeCommand
    private lateinit var raiseElevatorCommand: OpenRaiseElevatorCommand
    private lateinit var dropElevatorCommand: OpenDropElevatorCommand

    private lateinit var intake: Motor
    private lateinit var elevatorLeft: Motor
    private lateinit var elevatorRight: Motor

    private lateinit var servoLeft: Servo
    private lateinit var servoRight: Servo

    private lateinit var limit: TouchSensor

    private lateinit var driver: GamepadEx
    private lateinit var operator: GamepadEx

    override fun initialize() {
        intake = Motor(hardwareMap, ControlBoard.INTAKE.deviceName)

        elevatorLeft = Motor(hardwareMap, ControlBoard.ELEVATOR_LEFT.deviceName)
        elevatorRight = Motor(hardwareMap, ControlBoard.ELEVATOR_RIGHT.deviceName)

        servoLeft = hardwareMap.get(Servo::class.java, ControlBoard.SERVO_ELEVATOR_LEFT.deviceName)
        servoRight =
            hardwareMap.get(Servo::class.java, ControlBoard.SERVO_ELEVATOR_RIGHT.deviceName)

        limit = hardwareMap.get(TouchSensor::class.java, ControlBoard.LIMIT_SWITCH.deviceName)

        driveSubsystem = DriveSubsystem(hardwareMap)
        intakeSubsystem = IntakeSubsystem(intake)

        elevatorSubsystem = OpenElevatorSubsystem(elevatorLeft, elevatorRight, limit, telemetry, servoLeft, servoRight)

        driver = GamepadEx(gamepad1)
        operator = GamepadEx(gamepad2)

        intakeCommand = IntakeCommand(intakeSubsystem, intake = true)
        outtakeCommand = IntakeCommand(intakeSubsystem, intake = false)

        raiseElevatorCommand = OpenRaiseElevatorCommand(elevatorSubsystem)
        dropElevatorCommand = OpenDropElevatorCommand(elevatorSubsystem)

        driveCommand = DriveCommand(
            driveSubsystem,
            leftX = driver::getLeftX,
            leftY = driver::getLeftY,
            rightX = driver::getRightX,
            zoneVal = 0.15
        )

        hardwareMap.getAll(LynxModule::class.java)
            .forEach { it.bulkCachingMode = LynxModule.BulkCachingMode.AUTO }

        driver.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER).whileHeld(intakeCommand)
        driver.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER).whileHeld(outtakeCommand)

        operator.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER).whileHeld(raiseElevatorCommand)
        operator.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER).whileHeld(dropElevatorCommand)

        register(driveSubsystem)

        driveSubsystem.defaultCommand = driveCommand
    }
}