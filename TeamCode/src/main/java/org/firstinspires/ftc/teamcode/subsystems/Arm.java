package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.teamcode.subsystems.ArmConstants.*;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.profile.MotionProfile;
import com.acmerobotics.roadrunner.profile.MotionProfileGenerator;
import com.acmerobotics.roadrunner.profile.MotionState;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.sun.tools.javac.util.ArrayUtils;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.Arrays;

public class Arm {
    public final DcMotorEx top, low;
    public final DcMotor[] motors;

    public final double TICKS_IN_DEGREES = 220 / 90.0;
    public double maxPower = Math.toRadians(162 / TICKS_IN_DEGREES);

    public double target = GROUND;
    public int targetIndex = 0;
    public double armPos = GROUND;

    PIDController controller;
    public MotionProfile profile;
    public MotionState curState;

    public MultipleTelemetry telo;

    public ElapsedTime timer = new ElapsedTime();

    Double[] states = new Double[] {GROUND, LOW, MID, HIGH, BACKHIGH};

    public Arm(HardwareMap hardwareMap, Telemetry telemetry) {
        top = hardwareMap.get(DcMotorEx.class, "toplift"); //change id
        low = hardwareMap.get(DcMotorEx.class, "lowlift"); //change id
        motors = new DcMotor[] {top, low};

        for (DcMotor i: motors) {
            i.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
            i.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
            //i.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            i.setDirection(DcMotorSimple.Direction.REVERSE);
        }

        controller = new PIDController(kP, kI, kD);
        telo = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        curState = new MotionState(GROUND,0, 0);
        profile = MotionProfileGenerator.generateSimpleMotionProfile(curState, curState, mV, mA);
    }

    public void setTarget(double newTarget) {
        if (target != newTarget) {
            target = newTarget;
            timer.reset();
            profile = MotionProfileGenerator.generateSimpleMotionProfile(curState, new MotionState(target, 0, 0), mV, mA);
            targetIndex = Arrays.asList(states).indexOf(newTarget);
        }
    }

    public void next() {
        if (targetIndex < states.length - 1) {
            targetIndex++;
        }
        setTarget(states[targetIndex]);
    }

    public void prev() {
        if (targetIndex > 0) {
            targetIndex--;
        }
        setTarget(states[targetIndex]);
    }

    public void update() {
        armPos = top.getCurrentPosition();
        curState = profile.get(timer.time());
        double x = curState.getX();
        double pid = controller.calculate(armPos, x);
        double ff = Math.sin(Math.PI/(2 * maxPower) * Math.toRadians(armPos / TICKS_IN_DEGREES)) * kCos;
        for (DcMotor i : motors) {
            i.setPower(pid + ff);
        }
        telo.addData("target", target);
        telo.addData("pos", armPos);
        telo.addData("state", x);
        telo.update();
    }
}
