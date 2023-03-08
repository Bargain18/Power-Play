package org.firstinspires.ftc.teamcode.teleOp;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.*;

@TeleOp
@Config
public class teleOp1 extends LinearOpMode {
    @Override
    public void runOpMode() {
        Claw claw = new Claw(hardwareMap);
        Drive drive = new Drive(hardwareMap);
        Arm arm = new Arm(hardwareMap, telemetry);
        claw.setPos(1);

        Controller gp1 = new Controller(gamepad1);

        telemetry.addData("Running", "Waiting for start");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            gp1.update();

            if (gp1.current.left_bumper && !gp1.previous.left_bumper) {
                claw.change();
            }
            if (gamepad1.right_bumper) {
                drive.state = drive.SPRINT;
            } else {
                drive.state = drive.NORMAL;
            }

            if (gamepad1.options) {
                drive.resetHeading(hardwareMap);
            }

            if (gp1.current.dpad_up && !gp1.previous.dpad_up) {
                arm.next();
            }
            if (gp1.current.dpad_down && !gp1.previous.dpad_down) {
                arm.prev();
            }

            if (gamepad1.a) {
                arm.setTarget(ArmConstants.GROUND);
            }
            if (gamepad1.x) {
                arm.setTarget(ArmConstants.MID);
            }
            if (gamepad1.y) {
                arm.setTarget(ArmConstants.HIGH);
            }
            if (gamepad1.b) {
                arm.setTarget(ArmConstants.BACKHIGH);
            }

            if (drive.state != drive.SPRINT && arm.target != ArmConstants.GROUND) {
                drive.state = drive.SLOW;
            }

            claw.update();
            Pose2d pose = drive.updateField(gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);
            arm.update();
            telemetry.addData("Pose X", pose.getX());
            telemetry.addData("Pose Y", pose.getY());
            telemetry.addData("Pose heading", pose.getHeading());
            telemetry.update();
        }
    }
}
