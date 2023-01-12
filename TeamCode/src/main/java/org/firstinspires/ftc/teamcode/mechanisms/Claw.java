package org.firstinspires.ftc.teamcode.mechanisms;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.util.Mechanism;

public class Claw extends Mechanism {
    public Servo servo;

    public Claw (Servo grab) {
        super();
        //Servos.get(0), bigger # = Counter-clockwise
        servos.add(grab);
    }

    @Override
    public void update(Gamepad gp1, Gamepad gp2) {
        if (gp2.right_trigger > 0.3) {
            //Closes the claw
            close();
        } else if (gp2.left_trigger > 0.3) {
            //Opens the claw
            open();
        }
    }

    @Override
    public void write() {

    }

    public void open() {
        servos.get(0).setPosition(0.1);
    }

    public void close() {
        servos.get(0).setPosition(0);
    }

    public void init() {
        servos.get(0).setPosition(0);
    }
}