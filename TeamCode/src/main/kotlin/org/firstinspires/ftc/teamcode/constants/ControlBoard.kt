package org.firstinspires.ftc.teamcode.constants


enum class ControlBoard(val deviceName: String) {
    // Drive motors
    DRIVE_LEFT_FRONT("leftFront"),
    DRIVE_RIGHT_FRONT("rightFront"),
    DRIVE_LEFT_REAR("leftRear"),
    DRIVE_RIGHT_REAR("rightRear"),

    // Intake
    INTAKE("intake"),

    // Elevator
    ELEVATOR_LEFT("elevatorLeft"),
    ELEVATOR_RIGHT("elevatorRight"),

    SERVO_ELEVATOR_LEFT("leftElevatorServo"),
    SERVO_ELEVATOR_RIGHT("rightElevatorServo"),

    LIMIT_SWITCH("limit"),

    // Odometry
    ODO_LEFT_ENCODER("odoLeft"),
    ODO_RIGHT_ENCODER("leftRear"),
    ODO_STRAFE_ENCODER("rightRear"),

    // Camera
    CAMERA("lifecam")
}
