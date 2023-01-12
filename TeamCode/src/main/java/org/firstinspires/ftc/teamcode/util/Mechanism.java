package org.firstinspires.ftc.teamcode.util;

import android.hardware.Sensor;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;

import java.util.ArrayList;

public abstract class  Mechanism {
    public ArrayList<DcMotorEx> motors;

    public ArrayList<Servo> servos;

    protected ArrayList<CRServo> crServos;

    protected ArrayList<DigitalChannel> sensors;


    //DO NOT MAKE PUBLIC SHOULD ONLY BE ACESSED BY METHOD
    private ArrayList<String> telemetryCaptions = new ArrayList<String>();
    private ArrayList<String> telemetryDatas = new ArrayList<String>();


    public Mechanism(){
        motors = new ArrayList<DcMotorEx>();
        servos = new ArrayList<Servo>();
        crServos = new ArrayList<CRServo>();
        sensors = new ArrayList<DigitalChannel>();
    }


    public abstract void update(Gamepad gp1, Gamepad gp2);

    public abstract void write();

    public void addPublicTelemetry(String caption, String data) {
        //resets to remove the data from last cycle
        //adds the telemetry to the array list that then can be cycled through
        telemetryCaptions.add(caption);
        telemetryDatas.add(data);
    }

    public void updatePublicTelem(){
        telemetryCaptions = new ArrayList<String>();
        telemetryDatas = new ArrayList<String>();
    }

    public ArrayList<String> getTelemetryData() {
        return telemetryDatas;
    }
    public ArrayList<String> getTelemetryCaptions() {
        return telemetryCaptions;
    }

}