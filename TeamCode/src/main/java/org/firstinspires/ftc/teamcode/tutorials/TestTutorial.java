package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class TestTutorial extends LinearOpMode {
    HardwareClassTutorial robot = new HardwareClassTutorial();

    @Override
    public void runOpMode() {
        robot.init(hardwareMap);

        waitForStart();

        robot.leftMotor.setPower(1);
        robot.claw.setPosition(robot.CLAW_GRAB);
    }
}
