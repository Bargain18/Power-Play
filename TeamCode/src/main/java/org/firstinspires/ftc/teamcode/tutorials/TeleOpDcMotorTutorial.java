package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad1;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

@TeleOp
public class TeleOpDcMotorTutorial extends OpMode {

    DcMotor testMotor;

    public void init() {
        testMotor = hardwareMap.get(DcMotor.class, "testMotor");

        testMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        testMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void loop() {
        float motorPower = -gamepad1.left_stick_y;

        if (gamepad1.left_stick_y > 0.1 || gamepad1.left_stick_y < -0.1) {
            testMotor.setPower(motorPower);
        } else {
            testMotor.setPower(0);
        }
    }
}
