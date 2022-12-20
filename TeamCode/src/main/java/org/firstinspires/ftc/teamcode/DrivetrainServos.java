package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;

public class DrivetrainServos extends Mechanism {

    public DrivetrainServos(Servo Drive1,Servo Drive2,Servo Drive3){
        super();
        servos.add(Drive1);
        servos.add(Drive2);
        servos.add(Drive3);

    }

    public void update(Gamepad gp1, Gamepad gp2) {

    }

    @Override
    public void write(){

    }
}
