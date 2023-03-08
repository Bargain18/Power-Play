package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp
public class RobotCentricMecanumDrive extends LinearOpMode {
    DcMotor frontLeft;
    DcMotor frontRight;
    DcMotor backLeft;
    DcMotor backRight;

    @Override
    public void runOpMode() {
        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        backLeft = hardwareMap.get(DcMotor.class, "backLeft");
        backRight = hardwareMap.get(DcMotor.class, "backRight");

        frontRight.setDirection(DcMotor.Direction.REVERSE);
        backRight.setDirection(DcMotor.Direction.REVERSE);

        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        waitForStart();

        while (opModeIsActive()) {
            float right = -gamepad1.right_stick_y;
            float left = -gamepad1.left_stick_y;
            float leftS = gamepad1.left_trigger;
            float rightS = gamepad1.right_trigger;

            if (Math.abs(left) > 0.1 || Math.abs(right) > 0.1) {
                frontRight.setPower(right);
                frontLeft.setPower(left);
                backRight.setPower(right);
                backLeft.setPower(left);
            } else if (rightS > 0.1) {
                frontRight.setPower(-rightS);
                backLeft.setPower(-rightS);
                frontLeft.setPower(rightS);
                backRight.setPower(rightS);
            } else if (leftS > 0.1) {
                frontRight.setPower(rightS);
                backLeft.setPower(rightS);
                frontLeft.setPower(-rightS);
                backRight.setPower(-rightS);
            } else {
                frontRight.setPower(0);
                frontLeft.setPower(0);
                backRight.setPower(0);
                backLeft.setPower(0);
            }
        }
    }
}
