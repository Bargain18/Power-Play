package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.drive.advanced.PoseStorage;


public class Drive {
    DcMotor leftFront;
    DcMotor rightFront;
    DcMotor leftBack;
    DcMotor rightBack;
    public IMU imu;

    public final double SPRINT = 1.0;
    public final double NORMAL = 0.7;
    public final double SLOW = 0.5;

    public double state = NORMAL;

    SampleMecanumDrive drive;

    public Drive(HardwareMap hardwareMap) {
//        leftFront = hardwareMap.get(DcMotor.class, "leftfront");
//        rightFront = hardwareMap.get(DcMotor.class, "rightfront");
//        leftBack = hardwareMap.get(DcMotor.class, "leftback");
//        rightBack = hardwareMap.get(DcMotor.class, "rightback");
//
//        rightBack.setDirection(DcMotorSimple.Direction.REVERSE);
//        rightFront.setDirection(DcMotorSimple.Direction.REVERSE);
//
//        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        drive = new SampleMecanumDrive(hardwareMap);
        drive.imu.resetYaw();

//        // Retrieve the IMU from the hardware map
//        imu = hardwareMap.get(IMU.class, "imu");
//        // Adjust the orientation parameters to match your robot
//        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
//                RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
//                RevHubOrientationOnRobot.UsbFacingDirection.UP));
//        // Without this, the REV Hub's orientation is assumed to be logo up / USB forward
//        imu.initialize(parameters);
//        imu.resetYaw();
    }

    public void updateRobot(double left_stick_y, double left_stick_x, double right_stick_x) {
        double y = -left_stick_y; // Remember, this is reversed!
        double x = left_stick_x * 1.1; // Counteract imperfect strafing
        double rx = right_stick_x;

        // Denominator is the largest motor power (absolute value) or 1
        // This ensures all the powers maintain the same ratio, but only when
        // at least one is out of the range [-1, 1]
        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1.0);
        double frontLeftPower = (y + x + rx) / denominator * state;
        double backLeftPower = (y - x + rx) / denominator * state;
        double frontRightPower = (y - x - rx) / denominator * state;
        double backRightPower = (y + x - rx) / denominator * state;

//            leftFront.setPower(frontLeftPower);
//            leftBack.setPower(backLeftPower);
//            rightFront.setPower(frontRightPower);
//            rightBack.setPower(backRightPower);
        drive.setMotorPowers(frontLeftPower, backLeftPower, backRightPower, frontRightPower);
        drive.update();
    }

    public Pose2d updateField(double left_stick_y, double left_stick_x, double right_stick_x) {
        double y = -left_stick_y; // Remember, this is reversed!
        double x = left_stick_x * 1.1; // Counteract imperfect strafing
        double rx = right_stick_x;

        double botHeading = drive.getRawExternalHeading();

        // Rotate the movement direction counter to the bot's rotation
        double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
        double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);

        // Denominator is the largest motor power (absolute value) or 1
        // This ensures all the powers maintain the same ratio, but only when
        // at least one is out of the range [-1, 1]

        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1.0);
        double frontLeftPower = (rotY + rotX + rx) / denominator * state;
        double backLeftPower = (rotY - rotX + rx) / denominator * state;
        double frontRightPower = (rotY - rotX - rx) / denominator * state;
        double backRightPower = (rotY + rotX - rx) / denominator * state;

        drive.setMotorPowers(frontLeftPower, backLeftPower, backRightPower, frontRightPower);
        drive.update();
        PoseStorage.currentPose = drive.getPoseEstimate();
        return drive.getPoseEstimate();
    }

    public Pose2d updateFieldPose(double left_stick_y, double left_stick_x, double right_stick_x) {
        // Read pose
        Pose2d poseEstimate = drive.getPoseEstimate();

        // Create a vector from the gamepad x/y inputs
        // Then, rotate that vector by the inverse of that heading
        Vector2d input = new Vector2d(
                -left_stick_y,
                -left_stick_x
        ).rotated(-poseEstimate.getHeading());

        // Pass in the rotated input + right stick value for rotation
        // Rotation is not part of the rotated input thus must be passed in separately
        drive.setWeightedDrivePower(
                new Pose2d(
                        input.getX(),
                        input.getY(),
                        -right_stick_x
                )
        );
        return drive.getPoseEstimate();
    }

    public void resetHeading(HardwareMap hardwareMap) {
        Pose2d curPose = PoseStorage.currentPose;
        drive = new SampleMecanumDrive(hardwareMap);
        drive.setPoseEstimate(new Pose2d(curPose.getX(), curPose.getY()));
        drive.imu.resetYaw();
    }
}

