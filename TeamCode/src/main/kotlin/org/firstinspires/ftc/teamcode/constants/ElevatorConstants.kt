package org.firstinspires.ftc.teamcode.constants

class ElevatorConstants {
    enum class PIDF(val value: Double) {
        P(0.0), I(0.0), D(0.0), F(0.0)
    }

    enum class Feedforward(val value: Double) {
        G(0.0), S(0.0), V(0.0)
    }

    enum class Constraints(val value: Double) {
        MAX_VEL(0.0), MAX_ACCEL(0.0)
    }
}