package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.profile.MotionProfile;
import com.acmerobotics.roadrunner.profile.MotionState;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.acmerobotics.roadrunner.profile.MotionProfileGenerator;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@Config
@TeleOp
public class ArmPID extends OpMode {
    Claw claw;

    private PIDController controller;

    public static double p = 0, i = 0, d = 0, f = 0;
    public static int target = 0;
    public int prevTarget = target;

    public static double mV = 400.0, mA = 200.0;

    public final double TICKS_IN_DEGREES = 220 / 90.0;
    public double maxPower = Math.toRadians(162 / TICKS_IN_DEGREES);

    public MotionProfile profile;
    public MotionState curState;

    public MultipleTelemetry telo;

    public ElapsedTime timer = new ElapsedTime();

    boolean reset = true;

    Arm arm;

    @Override
    public void init() {
        claw = new Claw(hardwareMap);
        claw.setPos(1);
        claw.update();

        controller = new PIDController(p, i, d);
        telo = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        curState = new MotionState(0,0, 0);
        profile = MotionProfileGenerator.generateSimpleMotionProfile(curState, curState, mV, mA);

        arm = new Arm(hardwareMap, telemetry);
    }

    @Override
    public void loop() {
        controller.setPID(p, i, d);

        if (prevTarget != target) {
            timer.reset();
            prevTarget = target;
            profile = MotionProfileGenerator.generateSimpleMotionProfile(curState, new MotionState(target, 0, 0), mV, mA);
        }
        int armPos = arm.top.getCurrentPosition();
        curState = profile.get(timer.time());
        double pid = controller.calculate(armPos, curState.getX());
        double ff = Math.sin(Math.PI/(2 * maxPower) * Math.toRadians(armPos / TICKS_IN_DEGREES)) * f;
        //double ff =  Math.cos(Math.toRadians(armPos/TICKS_IN_DEGREES)) * f;
        for (DcMotor i : arm.motors) {
            i.setPower(pid + ff);
        }
        telo.addData("target", target);
        telo.addData("pos", armPos);
        telo.addData("velocity", arm.top.getVelocity());
        telo.addData("state", curState.getX());
        telo.addData("time", timer.time());
        telo.addData("prevTarget", reset);
        telo.addData("feedforward", ff);
        telo.update();
    }
}
