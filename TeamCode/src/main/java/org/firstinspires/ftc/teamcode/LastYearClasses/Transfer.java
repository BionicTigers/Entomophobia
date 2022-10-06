package org.firstinspires.ftc.teamcode.LastYearClasses;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Mechanism;

public class Transfer extends Mechanism {

    public boolean up;
    public boolean down;

    public Transfer(DcMotorEx transferMotor, Servo transferServo1, Servo transferservo2){
        super();
        motors.add(transferMotor);
        servos.add(transferServo1);
        servos.add(transferservo2);
    }

    @Override
    public void update(Gamepad gp1, Gamepad gp2) {
        if(gp2.dpad_up){
            up = true;
        } else if(gp2.dpad_down){
            down = true;
        } else{
            up = false;
            down = false;
        }
    }

    public void write() {
        //Controls transfer
        if (up){
            motors.get(0).setPower(60);
        } else if(down){
            motors.get(0).setPower(-60);
        } else{
            motors.get(0).setPower(0);
        }
    }
}
