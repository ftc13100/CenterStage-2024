package org.firstinspires.ftc.teamcode.constants

class ElevatorConstants {
    enum class PIDF(value: Double) {
        kP(0.0),
        kI(0.0),
        kD(0.0),
        kF(0.0)
    }

    enum class Constraints(value: Double) {
        maxVel(0.0),
        maxAccel(0.0)
    }
}