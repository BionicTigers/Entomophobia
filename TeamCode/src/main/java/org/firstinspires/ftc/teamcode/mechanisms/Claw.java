package org.firstinspires.ftc.teamcode.mechanisms;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.internal.system.Deadline;
import org.firstinspires.ftc.teamcode.util.Mechanism;

import java.util.concurrent.TimeUnit;

public class Claw extends Mechanism {
    public Servo servo;

    private DistanceSensor distance;

    public boolean hijack = false;
    public boolean open = false;
    public boolean comboPressed = false;

    private RevBlinkinLedDriver blinkin;

    private int cycles = 0;
    private double dist = 0;

    public Deadline fastDrop = new Deadline (1, TimeUnit.SECONDS);

    public Claw (Servo grab, DistanceSensor distance, RevBlinkinLedDriver blinkin) {
        super();
        //Servos.get(0), bigger # = Counter-clockwise
        servo = grab;
        servos.add(grab);

        this.distance = distance;
        this.blinkin = blinkin;
    }

    @Override
    public void update(Gamepad gp1, Gamepad gp2) {

        if ((gp1.back && gp1.a) || (gp2.back && gp2.a)) {
            hijack = true;
        }

        open = (gp1.left_trigger >= 0.3) || (gp2.left_trigger >= 0.3);
    }

    @Override
    public void write() {

        if (hijack) {
            littleClose();
        } else {
            close();
        }

        if (open) {
            open();
            hijack = false;
        }

        if(cycles == 10 && coneDetected()) {
            blinkin.setPattern(RevBlinkinLedDriver.BlinkinPattern.GREEN);
        } else {
            blinkin.setPattern(RevBlinkinLedDriver.BlinkinPattern.RED);
        }
    }

    public void open() {
        servo.setPosition(0.3);
    }

    public void close() {
        servo.setPosition(0);
    }

    public void littleClose() {
        servo.setPosition(0.2);
    }

    public void init() {
        servo.setPosition(0.4);
    }

    public double getDistance() {
        if (cycles == 30)
            dist = distance.getDistance(DistanceUnit.MM);

        return dist;
    }

    public boolean coneDetected() {
        if(getDistance() < 20) {
            return true;
        }
        else return false;
    }

    public void quickDrop() {
        open();
        fastDrop.reset();
        while (!fastDrop.hasExpired()) {}
        close();
    }
}