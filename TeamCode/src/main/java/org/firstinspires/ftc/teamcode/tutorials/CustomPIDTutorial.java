package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;

public class CustomPIDTutorial extends LinearOpMode {
    DcMotor testMotor;
    ElapsedTime PIDTimer = new ElapsedTime();
    double integral = 0;
    PIDCoefficients testPID = new PIDCoefficients(0,0,0);

    @Override
    public void runOpMode() {
        testMotor = hardwareMap.get(DcMotor.class, "testMotor");
        testMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        testMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        testMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        waitForStart();
    }

    void moveTestMotor(double targetPosition) {
        double error = testMotor.getCurrentPosition();
        double lastError = 0;

        while (Math.abs(error) <= 9) {
            error = testMotor.getCurrentPosition() - testMotor.getTargetPosition();
            double changeInError = lastError - error;
            integral += changeInError * PIDTimer.time();
            double derivative = changeInError / PIDTimer.time();
            double P = testPID.p * error;
            double I = testPID.i * integral;
            double D = testPID.d * derivative;
            testMotor.setPower(P + I + D);
            error = lastError;
            PIDTimer.reset();
        }
    }
}
