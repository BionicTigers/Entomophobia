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
    public boolean slowMode;
//    private DigitalChannel limit1;
//    private DigitalChannel limit2;

    public Arm (CRServo l, CRServo r, Telemetry T/*, DigitalChannel CBFront, DigitalChannel CBBack*/) {
        super();
        crServos.add(l);
        crServos.add(r);
        crServos.get(1).setDirection(DcMotorSimple.Direction.REVERSE);
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
        if (gp2.right_bumper/* && sensors.get(0).getState()*/) {
            forward();
        } else if (gp2.left_bumper/* && sensors.get(1).getState()*/) {
            backward();
        } else {
            crServos.get(0).setPower(0);
            crServos.get(1).setPower(0);
        }

        if (gp2.right_trigger > 0.3) {
            slowMode = true;
        } else {
            slowMode = false;
        }
    }

    @Override
    public void write() {
//        telemetry.addData("Bottom Switch", !sensors.get(0).getState());
//        telemetry.addData("Top Switch", !sensors.get(1).getState());
    }

    public void forward() {
        crServos.get(0).setPower(1);
        crServos.get(1).setPower(1);
    }

    public void backward() {
        crServos.get(0).setPower(-1);
        crServos.get(1).setPower(-1);
    }
}
