package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Claw {
    Servo claw;
    public double goingTo;

    public final double OPEN = 0.45; //tweak value!
    public final double CLOSED = 0.56; //tweak value!

    public Claw(HardwareMap hardwareMap){
        claw = hardwareMap.get(Servo.class, "claw");
        claw.setDirection(Servo.Direction.FORWARD);
    }

    //1 -> closed
    //0 -> opened
    public void setPos(int state) {
        if (state == 1) {
            goingTo = CLOSED;
        } else {
            goingTo = OPEN;
        }
    }

    public void setPosSpecific(double pos) {
        claw.setPosition(pos);
    }

    public void update() {
        claw.setPosition(goingTo);
    }

    public void change() {
        if (goingTo == CLOSED) {
            goingTo = OPEN;
        } else {
            goingTo = CLOSED;
        }
    }
}
