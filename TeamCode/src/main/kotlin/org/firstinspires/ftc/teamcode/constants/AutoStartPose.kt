package org.firstinspires.ftc.teamcode.constants

import com.acmerobotics.roadrunner.geometry.Pose2d

enum class AutoStartPose(val startPose: Pose2d) {
    RED_LEFT(
        Pose2d(
            -36.0, -61.5, Math.toRadians(-90.0)
        )
    ),

    RED_RIGHT(
        Pose2d(
            10.0, -61.5, Math.toRadians(0.0)
        )
    ),

    BLUE_LEFT(
        Pose2d(
            10.0, 61.5, Math.toRadians(0.0)
        )
    ),

    BLUE_RIGHT(
        Pose2d(
            -36.0, 61.5, Math.toRadians(0.0)
        )
    ),
}