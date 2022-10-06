package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.AutoStuff.Variables;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class PositionalTransfer extends Mechanism{
    public String position = "Mid";
    public Telemetry telemetry;
    public DcMotorEx motor;
    public int trim = 0;
    public int trim2 = 0;
    public boolean currentlyPressed = false;
    private DigitalChannel channel;
    private HardwareMap hardwareMap;
    private boolean reset = false;
    private boolean liftOverride = false;
    private boolean isPressed = false;

    public PositionalTransfer(DcMotorEx m, Telemetry T, DigitalChannel channel, Servo servo1, Servo servo2){
        super();
        telemetry = T;
        telemetry.addData("<", "Fully initialized");
        DcMotorEx motor = m;
        motors.add(motor);
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motors.get(0).setTargetPosition(0);
        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        sensors.add(channel);
        sensors.get(0).setMode(DigitalChannel.Mode.INPUT);
        servos.add(servo1);
        servos.add(servo2);
    }

     //Moves the arm to the bottom
//    public void moveToBottom(){
//        motor.setTargetPosition(-250);
//    }
//
//     //Moves the arm to the middle
//    public void moveToMiddle(){
//        motor.setTargetPosition(-500);
//    }
//    //Moves the arm to the top
//    public void moveToTop(){
//        motor.setTargetPosition(-773);
//    }

    public void update(Gamepad gp1, Gamepad gp2){
        if(gp2.right_trigger >= 0.5){
            position = "Up";
        } else if(gp2.left_trigger >= 0.3){
            position = "Mid";
        } else if((gp1.right_trigger >= .3  && position != "Up" )){
            position = "Down";
        }
        else if(gp2.start){
            position = "shared";
        }
         else if (gp2.left_bumper) {
            trim = trim + 1;
        }
         if(gp2.start && gp2.back && !isPressed){
             if(liftOverride) liftOverride = false;
             else liftOverride = true;
             isPressed = true;
         }
         else if(!gp2.start && !gp2.back && isPressed){
             isPressed = false;
         }

        if (gp2.right_stick_button) {
            trim = trim + 3;
        }
        if(gp1.y){
            trim2 ++;
        }
        if(gp2.back){
            position = "shared";
        }
        telemetry.addData("position", motors.get(0).getCurrentPosition());
        telemetry.addData("Amps", motors.get(0).getCurrent(CurrentUnit.AMPS));
        telemetry.addData("Is pressed? ", sensors.get(0).getState());
        reset = gp1.right_stick_button || gp2.left_bumper;
        if(gp2.dpad_down){
            position = "intake";
        }

//        telemetry.update();
    }

    public void write(){
        if(motors.get(0).getCurrent(CurrentUnit.AMPS) >= 5.5 && position.equals("Up") && motors.get(0).getCurrentPosition() > 2000 * 223/312){
            trim = 1650 - motors.get(0).getCurrentPosition();
        }
        if(position == "Up" && !liftOverride){
            motors.get(0).setPower(100);
            motors.get(0).setTargetPosition(1650 - trim);
        }
        else if(position == "Down" && sensors.get(0).getState() && !liftOverride) {
            motors.get(0).setPower(60);
            motors.get(0).setTargetPosition(-trim2);

            if(reset || trim2 < 50) {
                trim2 = trim2 + 10;
            }
        }
        else if (position == "Mid"&& !liftOverride){
            motors.get(0).setPower(30);
            motors.get(0).setTargetPosition(600);//* 223/312);
        }
        else if (position == "intake"&& !liftOverride)
        {
            motors.get(0).setTargetPosition(450 * 223/312);
        }
        else if(position.equals("shared") && !liftOverride)
        {
            motors.get(0).setTargetPosition(1475*223/312);
        }
        if (position == "Down")
        {
            position = "Mid";
        }
        if (!sensors.get(0).getState() && !currentlyPressed) {
            trim = 0;
            trim2 = 0;
            motors.get(0).setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motors.get(0).setTargetPosition(0);
            motors.get(0).setMode(DcMotor.RunMode.RUN_TO_POSITION);
            currentlyPressed = true;
        }
        if (sensors.get(0).getState()) {
            currentlyPressed = false;
        }
        if(liftOverride){
            motors.get(0).setPower(0);
        }
    }
}