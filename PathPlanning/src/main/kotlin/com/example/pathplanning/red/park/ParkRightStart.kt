package com.example.pathplanning.red.park

import com.acmerobotics.roadrunner.geometry.Pose2d
import com.noahbres.meepmeep.MeepMeep
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder

object ParkRightStart {
    @JvmStatic
    fun main(args: Array<String>) {
        val meepMeep = MeepMeep(600)
        val startPose = Pose2d(10.0, -61.5, Math.toRadians(0.0))
        val base = DefaultBotBuilder(meepMeep)
            .setDimensions(17.0, 17.0)
            .setConstraints(
                36.58665032249647,
                36.58665032249647,
                Math.toRadians(163.028007),
                Math.toRadians(123.30945),
                15.75 // in

            )

            .followTrajectorySequence {
                it.trajectorySequenceBuilder(startPose)
                    .strafeLeft(4.0)
                    .forward(42.0)
                    .build()
            }

        meepMeep.setBackground(
            MeepMeep.Background.FIELD_CENTERSTAGE_JUICE_DARK
        )
            .addEntity(base)
            .start()
    }
}