package com.example.pathplanning.red.truss

import com.acmerobotics.roadrunner.geometry.Pose2d
import com.acmerobotics.roadrunner.geometry.Vector2d
import com.noahbres.meepmeep.MeepMeep
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder
import kotlin.random.Random

object RedLeftTrussAuto {
    enum class Selected {
        LEFT,
        RIGHT,
        CENTER,
        NONE
    }
    @JvmStatic
    fun main(args: Array<String>) {
        val meepMeep = MeepMeep(600)
        val startPose = Pose2d(-36.0, -61.5, Math.toRadians(-90.0))
        val selection = Selected.RIGHT
//        val selection = when (Random.nextInt(0, 3)) {
//            0 -> Selected.LEFT
//            1 -> Selected.CENTER
//            2 -> Selected.RIGHT
//            else -> Selected.NONE
//        }

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
                    .lineTo(Vector2d(-36.0, -35.0))
                    .apply {
                        when (selection) {
                            Selected.LEFT -> {
                                this.lineToSplineHeading(
                                    Pose2d(
                                        -45.0,
                                        -30.0,
                                        Math.toRadians(-45.0)
                                    )
                                )
                            }
                            Selected.CENTER -> {
                                this.lineTo(
                                    Vector2d(
                                        -36.0,
                                        -30.0
                                    )
                                )
                            }
                            Selected.RIGHT -> {
                                this.lineToSplineHeading(
                                    Pose2d(
                                        -30.0,
                                        -30.0,
                                        Math.toRadians(-135.0)
                                    )
                                )
                            }
                            else -> {
                                this.lineToSplineHeading(
                                    Pose2d(
                                        -45.0,
                                        -30.0,
                                        Math.toRadians(-45.0)
                                    )
                                )
                            }
                        }
                    }
                    .waitSeconds(1.0)
                    .lineTo(Vector2d(-36.0, -35.0))
                    .apply {
                        if (selection == Selected.RIGHT)
                            this
                                .lineToSplineHeading(
                                    Pose2d(
                                        -36.0,
                                        -58.0,
                                        Math.toRadians(-90.0)
                                    )
                                )
                                .lineTo(
                                    Vector2d(
                                        -10.0,
                                        -58.0
                                    )
                                )
                                .lineTo(
                                    Vector2d(
                                        -10.0,
                                        -35.0
                                    )
                                )
                    }
                    .lineToSplineHeading(
                        Pose2d(
                            35.0,
                            -35.0,
                            Math.toRadians(180.0)
                        )
                    )
                    .addTemporalMarker(
                        when (selection) {
                            Selected.LEFT -> 8.0
                            Selected.RIGHT -> 13.0
                            Selected.CENTER -> 8.0
                            Selected.NONE -> 8.0
                        }
                    ) {

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