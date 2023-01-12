package org.firstinspires.ftc.teamcode.mechanisms;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.util.Mechanism;

public class Arm extends Mechanism {
    public Telemetry telemetry;
    public double position;

    public Arm (CRServo l, CRServo r, Telemetry T) {
        super();
        crServos.add(l);
        crServos.add(r);
        crServos.get(1).setDirection(DcMotorSimple.Direction.REVERSE);

        telemetry = T;
    }

    @Override
    public void update(Gamepad gp1, Gamepad gp2) {
        if (gp2.right_bumper) {
            forward();
        } else if (gp2.left_bumper) {
            backward();
        } else {
            crServos.get(0).setPower(0);
            crServos.get(1).setPower(0);
        }
    }

    @Override
    public void write() {
        telemetry.addData("Position", position);
    }

    public void forward() {
        crServos.get(0).setPower(1);
        crServos.get(1).setPower(1);
        position = position + 1;
    }

    public void backward() {
        crServos.get(0).setPower(-1);
        crServos.get(1).setPower(-1);
        position = position - 1;
    }
}
