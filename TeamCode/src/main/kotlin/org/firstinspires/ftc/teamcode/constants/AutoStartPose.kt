package org.firstinspires.ftc.teamcode.constants

import com.acmerobotics.roadrunner.geometry.Pose2d

enum class AutoStartPose(val startPose: Pose2d) {
    RED_LEFT(Pose2d()),
    RED_RIGHT(Pose2d()),
    BLUE_LEFT(Pose2d()),
    BLUE_RIGHT(Pose2d()),
}