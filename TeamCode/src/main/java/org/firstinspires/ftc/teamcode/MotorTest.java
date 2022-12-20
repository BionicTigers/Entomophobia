package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;

public class MotorTest extends Mechanism{
    private DcMotorEx motor1;
    private DcMotorEx motor2;

    public MotorTest(DcMotorEx m1, DcMotorEx m2){
        motor1 = m1;
        motor2 = m2;
    }

    public void update(Gamepad gp1, Gamepad gp2) {
        motor1.setPower(0.5);
        motor2.setPower(0.5);
    }

    public void write() {

    }
}
