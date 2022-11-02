package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;

public class Claw extends Mechanism{
    public Servo servol;
    public Servo servor;

    public Claw (Servo l, Servo r) {
        super();
        servol = l;
        servos.add(servol);
        servor = r;
        servos.add(servor);
    }
    @Override
    public void update(Gamepad gp1, Gamepad gp2) {
        if (gp1.right_trigger > 0.3) {
            servos.get(0).setPosition(0.5);
            servos.get(1).setPosition(0.5);
        } else if (gp1.left_trigger > 0.3) {
            servos.get(0).setPosition(0.75);
            servos.get(1).setPosition(0.25);
        }
    }

    @Override
    public void write() {

    }
}
