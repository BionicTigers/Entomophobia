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
    public Telemetry telemetry;

    public boolean hijack = false;
    public boolean open = false;
    public boolean comboPressed = false;

    private RevBlinkinLedDriver blinkin;

    private int cycles = 0;
    private double dist = 0;

    private boolean cachedConeDetect;

    public Deadline fastDrop = new Deadline (1, TimeUnit.SECONDS);

    public Claw (Servo grab, DistanceSensor distance, RevBlinkinLedDriver blinkin, Telemetry T) {
        super();
        //Servos.get(0), bigger # = Counter-clockwise
        servo = grab;
        servos.add(grab);

        this.distance = distance;
        this.blinkin = blinkin;
        telemetry = T;
    }

    @Override
    public void update(Gamepad gp1, Gamepad gp2) {

        if ((gp1.back && gp1.a) || (gp2.back && gp2.a)) {
            hijack = true;
        }

        open = (gp1.b || (gp2.left_trigger >= 0.3));
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
        telemetry.addData("cone", dist);

//        if (cachedConeDetect == coneDetected()) {
//            return;
//        }
//
//        cachedConeDetect = coneDetected();

        if (coneDetected()) {

            blinkin.setPattern(RevBlinkinLedDriver.BlinkinPattern.GREEN);
            System.out.println("green");
        } else {
            blinkin.setPattern(RevBlinkinLedDriver.BlinkinPattern.RED);
            System.out.println("red");
        }
    }

    public void open() {
        servo.setPosition(1);
    }

    public void close() {
        servo.setPosition(0.1);
    }

    public void littleClose() {
        servo.setPosition(0.7);
    }

    public void init() {
        servo.setPosition(1);
    }

    public double getDistance() {
        cycles++;
        if (cycles == 20) {
            dist = distance.getDistance(DistanceUnit.CM);
            cycles = 0;
        }

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