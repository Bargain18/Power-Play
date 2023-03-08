package org.firstinspires.ftc.teamcode;

import android.graphics.drawable.GradientDrawable;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

public class HardwareClassTutorial {

    public DcMotor leftMotor;
    public DcMotor rightMotor;
    public Servo claw;

    public BNO055IMU imu;
    public Orientation angles;

    public static final double CLAW_GRAB = 0.75;
    public static final double CLAW_RELEASE = 0.4;

    HardwareMap hwMap;

    public HardwareClassTutorial(){}

    public void init(HardwareMap hwMap) {
        leftMotor = hwMap.get(DcMotor.class, "leftMotor");
        rightMotor = hwMap.get(DcMotor.class, "rightMotor");
        claw = hwMap.get(Servo.class, "claw");

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.calibrationDataFile = "BNO05IMUCalibration.json";
        imu = hwMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        leftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        leftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }
}
