package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@Autonomous
public class AutoDcMotorTutorial extends LinearOpMode {
    DcMotor testMotor;

    @Override
    public void runOpMode() {
        testMotor = hardwareMap.get(DcMotor.class, "testMotor");
        testMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        waitForStart();

        testMotor.setPower(1);
        sleep(5000);
        testMotor.setPower(-0.5);
        sleep(2500);
        testMotor.setPower(0);
    }
}
