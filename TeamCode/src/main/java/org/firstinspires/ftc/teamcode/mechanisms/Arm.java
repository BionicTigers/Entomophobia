package org.firstinspires.ftc.teamcode.mechanisms;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.internal.system.Deadline;
import org.firstinspires.ftc.teamcode.util.Mechanism;

import java.util.concurrent.TimeUnit;

public class Arm extends Mechanism {
    public Telemetry telemetry;

    public CRServo left;
    public CRServo right;
//    private DigitalChannel limit1;
//    private DigitalChannel limit2;

    public Arm (CRServo l, CRServo r, Telemetry T/*, DigitalChannel CBFront, DigitalChannel CBBack*/) {
        super();
        crServos.add(l);
        left = l;

        crServos.add(r);
        right = r;

        right.setDirection(DcMotorSimple.Direction.REVERSE);
//        limit1 = CBFront;
//        limit2 = CBBack;
//        sensors.add(limit1);
//        sensors.add(limit2);
//        sensors.get(0).setMode(DigitalChannel.Mode.INPUT);
//        sensors.get(1).setMode(DigitalChannel.Mode.INPUT);


        telemetry = T;
    }

    @Override
    public void update(Gamepad gp1, Gamepad gp2) {
        left.setPower(0);
        right.setPower(0);

        if (gp2.right_bumper) {
            move(0.4);
        }
        if (gp2.left_bumper) {
            move(-0.4);
        }

        if (gp2.dpad_up) {
            move(1);
        }
        if (gp2.dpad_left) {
            move(1);
        }
        if (gp2.dpad_down) {
            move(1);
        }
        if (gp2.dpad_right) {
            move(-1);
        }
    }

    @Override
    public void write() {
//        telemetry.addData("Bottom Switch", !sensors.get(0).getState());
//        telemetry.addData("Top Switch", !sensors.get(1).getState());
    }

    /**
     * Moves the lift forward
     * @param mod number from -1 to 1 that sets the speed of the move
     */
    public void move(double mod) {
        left.setPower(mod);
        right.setPower(mod);
    }
}
