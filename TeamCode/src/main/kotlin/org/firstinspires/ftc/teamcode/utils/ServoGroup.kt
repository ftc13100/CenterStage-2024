package org.firstinspires.ftc.teamcode.utils

import com.qualcomm.robotcore.hardware.Servo

class ServoGroup(
    private vararg val servos: Servo,
) {
    var position
        get() = servos[0].position
        set(position) {
            servos.forEach { it.position = position }
        }
}