package org.firstinspires.ftc.teamcode.subsystems.vision

import com.arcrobotics.ftclib.command.SubsystemBase
import com.qualcomm.robotcore.hardware.HardwareMap
import org.firstinspires.ftc.robotcore.external.Telemetry
import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraControls
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.CameraControl
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit
import org.firstinspires.ftc.teamcode.constants.ControlBoard
import org.firstinspires.ftc.teamcode.processors.BeaverProcessor
import org.firstinspires.ftc.vision.VisionPortal
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection
import org.firstinspires.ftc.vision.apriltag.AprilTagPoseFtc
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor

class VisionSubsystem (
    hardwareMap: HardwareMap,
    telemetry: Telemetry
) : SubsystemBase() {
    val portal: VisionPortal
    private val beaverProcessor: BeaverProcessor
    private val aprilTag: AprilTagProcessor
    val detections: List<AprilTagDetection>
        get() = aprilTag.detections

    val detectionPoses: List<AprilTagPoseFtc>
        get() = detections.map { it.ftcPose }

    val selection
        get() = beaverProcessor.selection

    var targetId = 0;

    val cameraState
        get() = portal.cameraState

    init {
        beaverProcessor = BeaverProcessor(telemetry)

        aprilTag = AprilTagProcessor.Builder()
            .setOutputUnits(DistanceUnit.INCH, AngleUnit.RADIANS)
            .build()

        portal = VisionPortal.Builder()
            .setCamera(hardwareMap.get(WebcamName::class.java, ControlBoard.CAMERA.deviceName))
            .addProcessors(beaverProcessor, aprilTag)
            .build()
    }

    fun <T : Class<CameraControl>> getCameraControl(control: T) = portal.getCameraControl(control)
}