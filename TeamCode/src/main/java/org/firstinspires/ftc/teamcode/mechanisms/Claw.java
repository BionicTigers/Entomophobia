package org.firstinspires.ftc.teamcode.mechanisms;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.internal.system.Deadline;
import org.firstinspires.ftc.teamcode.util.Mechanism;

import java.util.concurrent.TimeUnit;

public class Claw extends Mechanism {
    public Servo servo;

    public Deadline fastDrop = new Deadline (1, TimeUnit.SECONDS);

    public Claw (Servo grab) {
        super();
        //Servos.get(0), bigger # = Counter-clockwise
        servos.add(grab);
    }

    @Override
    public void update(Gamepad gp1, Gamepad gp2) {
        if (gp2.left_trigger > 0.3) {
            //Opens the claw
            open();
        } else {
            close();
        }
    }

    @Override
    public void write() {

    }

    public void open() {
        servos.get(0).setPosition(0.2);
    }

    public void close() {
        servos.get(0).setPosition(0);
    }

    public void init() {
        servos.get(0).setPosition(0);
    }

    public void quickDrop() {
        open();
        fastDrop.reset();
        while (!fastDrop.hasExpired()) {}
        close();
    }
}