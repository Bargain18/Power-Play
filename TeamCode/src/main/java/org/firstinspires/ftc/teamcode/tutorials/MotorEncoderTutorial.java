package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

public class MotorEncoderTutorial extends LinearOpMode {
    DcMotor leftMotor;
    DcMotor rightMotor;

    @Override
    public void runOpMode() {
        leftMotor = hardwareMap.get(DcMotor.class, "leftMotor");
        rightMotor = hardwareMap.get(DcMotor.class, "rightMotor");

        leftMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        /* Encoder modes:
        RUN_WITHOUT_ENCODER
        RUN_USING_ENCODER
        RUN_TO_POSITION
        STOP_AND_RESET_POSITION
         */

        resetEncoders();
        leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        waitForStart();

        driveForward(1, 1000);

        leftMotor.setTargetPosition(1000);
        rightMotor.setTargetPosition(1000);

        leftMotor.setPower(1);
        rightMotor.setPower(1);

        while (leftMotor.isBusy() && rightMotor.isBusy()) {

        }
    }

    void driveForward(double power, double ticks) {

        while (leftMotor.getCurrentPosition() < ticks && rightMotor.getCurrentPosition() < ticks && opModeIsActive()) {
            leftMotor.setPower(power);
            rightMotor.setPower(power);
        }
        leftMotor.setPower(0);
        rightMotor.setPower(0);

        resetEncoders();
    }

    void resetEncoders() {
        leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }
}
