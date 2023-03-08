package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;

public class servoTutorial extends LinearOpMode {
    Servo testServo;

    @Override
    public void runOpMode() {
        testServo = hardwareMap.get(Servo.class, "testServo");
        testServo.setDirection(Servo.Direction.REVERSE);

        waitForStart();

        //Range is between 0 and 1
        testServo.setPosition(0.5);
    }
}
