package com.example.pathplanning

import com.acmerobotics.roadrunner.geometry.Pose2d
import com.acmerobotics.roadrunner.geometry.Vector2d
import com.noahbres.meepmeep.MeepMeep
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder
import java.io.File
import javax.imageio.ImageIO

object PathPlanning {
    @JvmStatic
    fun main(args: Array<String>) {
        val meepMeep = MeepMeep(600)
        val startPose = Pose2d(-38.0, -56.0, Math.toRadians(90.0))
        val base = DefaultBotBuilder(meepMeep)
            .setDimensions(15.0, 15.0)
            .setConstraints(
                38.110287416570166,
                38.110287416570166,
                Math.toRadians(457.2273162437774),
                Math.toRadians(138.19991297468354),
                13.1 // in

            )
//            .build()
            .followTrajectorySequence {
                it.trajectorySequenceBuilder(startPose)
                    .lineToSplineHeading(Pose2d(-38.0, -35.0))
                    .lineTo(Vector2d(40.0, -35.0))
                    .addTemporalMarker(5.0) {
                        it.poseEstimate
                    }
                    .waitSeconds(3.0)
//                    .splineTo(Vector2d(60.0, 50.0), Math.toRadians(180.0))
                    .build()
            }

        meepMeep.setBackground(
            MeepMeep.Background.FIELD_CENTERSTAGE_JUICE_DARK
        )
            .addEntity(base)
            .start()
    }
}