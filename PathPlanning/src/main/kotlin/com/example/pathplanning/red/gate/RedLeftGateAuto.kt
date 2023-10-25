package com.example.pathplanning.red.gate

import com.acmerobotics.roadrunner.geometry.Pose2d
import com.acmerobotics.roadrunner.geometry.Vector2d
import com.noahbres.meepmeep.MeepMeep
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder

object RedLeftGateAuto {
    @JvmStatic
    fun main(args: Array<String>) {
        val meepMeep = MeepMeep(600)
        val startPose = Pose2d(-36.0, -61.5, Math.toRadians(90.0))
        val base = DefaultBotBuilder(meepMeep)
            .setDimensions(17.0, 17.0)
            .setConstraints(
                36.58665032249647,
                36.58665032249647,
                Math.toRadians(163.028007),
                Math.toRadians(123.30945),
                15.75 // in
            )
//            .build()
            .followTrajectorySequence {
                it.trajectorySequenceBuilder(startPose)
                    .splineTo(Vector2d(-36.0, -35.0), startPose.heading)
                    .addTemporalMarker(2.0) {
                        it.poseEstimate
                    }
                    .waitSeconds(2.0)
                    .splineTo(Vector2d(-36.0, -10.0), startPose.heading)
                    .splineToSplineHeading(Pose2d(-16.0, -10.0, Math.toRadians(180.0)), Math.toRadians(0.0))
                    .splineToSplineHeading(Pose2d(30.0, -18.0, Math.toRadians(180.0)), Math.toRadians(0.0))
                    .splineToSplineHeading(Pose2d(47.5, -35.0, Math.toRadians(180.0)), Math.toRadians(-90.0))
//                    .lineToSplineHeading(Pose2d(35.0, -35.0, Math.toRadians(180.0)))
                    .addTemporalMarker(11.0) {
                        it.poseEstimate
                    }
                    .waitSeconds(3.0)
                    .build()
            }

        meepMeep.setBackground(
            MeepMeep.Background.FIELD_CENTERSTAGE_JUICE_DARK
        )
            .addEntity(base)
            .start()
    }
}