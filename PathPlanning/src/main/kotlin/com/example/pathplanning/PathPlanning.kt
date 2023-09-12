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
        val startPose = Pose2d(56.0, -36.0, Math.toRadians(180.0))
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
                    .lineTo(Vector2d(50.0, -36.0))
                    .build()
            }

        meepMeep.setBackground(
            ImageIO.read(
                File("PathPlanning/src/main/resources/temp_centerstage_field.png")
            )
//        MeepMeep.Background.FIELD_CENTERSTAGE_OFFICIAL
        )
            .addEntity(base)
            .start()
    }
}