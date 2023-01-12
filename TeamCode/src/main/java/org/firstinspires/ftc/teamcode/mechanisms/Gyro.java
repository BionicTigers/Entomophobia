package org.firstinspires.ftc.teamcode.mechanisms;

import androidx.annotation.NonNull;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.*;
import org.firstinspires.ftc.teamcode.util.Mechanism;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.Gamepad;

public class Gyro extends Mechanism implements BNO055IMU{
    BNO055IMU imu;
    private Telemetry telemetry;
        public Gyro(Telemetry T, BNO055IMU gyro){
            telemetry = T;
        }

    @Override
    public boolean initialize(@NonNull Parameters parameters) {
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json";
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";
        imu.initialize(parameters);
        return false;
    }

    @NonNull
    @Override
    public Parameters getParameters() {
        return null;
    }

    @Override
    public void close() {

    }

    @Override
    public Orientation getAngularOrientation() {
        return null;
    }


    @Override
    public Orientation getAngularOrientation(AxesReference reference, AxesOrder order, org.firstinspires.ftc.robotcore.external.navigation.AngleUnit angleUnit) {
        return getAngularOrientation().toAxesReference(reference).toAxesOrder(order).toAngleUnit(angleUnit);
    }

    @Override
    public Acceleration getOverallAcceleration() {
        return null;
    }

    @Override
    public AngularVelocity getAngularVelocity() {
        return null;
    }

    @Override
    public Acceleration getLinearAcceleration() {
        return null;
    }

    @Override
    public Acceleration getGravity() {
        return null;
    }

    @Override
    public Temperature getTemperature() {
        return null;
    }

    @Override
    public MagneticFlux getMagneticFieldStrength() {
        return null;
    }

    @Override
    public Quaternion getQuaternionOrientation() {
        return null;
    }

    @Override
    public Position getPosition() {
        return null;
    }

    @Override
    public Velocity getVelocity() {
        return null;
    }

    @Override
    public Acceleration getAcceleration() {
        return null;
    }

    @Override
    public void startAccelerationIntegration(Position initialPosition, Velocity initialVelocity, int msPollInterval) {

    }

    @Override
    public void stopAccelerationIntegration() {

    }

    @Override
    public SystemStatus getSystemStatus() {
        return null;
    }

    @Override
    public SystemError getSystemError() {
        return null;
    }

    @Override
    public CalibrationStatus getCalibrationStatus() {
        return null;
    }

    @Override
    public boolean isSystemCalibrated() {
        return false;
    }

    @Override
    public boolean isGyroCalibrated() {
        return false;
    }

    @Override
    public boolean isAccelerometerCalibrated() {
        return false;
    }

    @Override
    public boolean isMagnetometerCalibrated() {
        return false;
    }

    @Override
    public CalibrationData readCalibrationData() {
        return null;
    }

    @Override
    public void writeCalibrationData(CalibrationData data) {

    }

    @Override
    public byte read8(Register register) {
        return 0;
    }

    @Override
    public byte[] read(Register register, int cb) {
        return new byte[0];
    }

    @Override
    public void write8(Register register, int bVal) {

    }

    @Override
    public void write(Register register, byte[] data) {

    }
    @Override
    public void update(Gamepad gp1, Gamepad gp2) {
        telemetry.addData("orientation:", "Angle:x=%6.1f,y=%6.1f,z=%6.1f",
                imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES).firstAngle,
                imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES).secondAngle,
                imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES).thirdAngle);
        telemetry.addData("Velocity", "Vel: x=%6.1f,y=%6.1f,z=%6.1f",
                imu.getVelocity().xVeloc,
                imu.getVelocity().yVeloc,
                imu.getVelocity().zVeloc);
        telemetry.addData("LinearAccel", "Dist: x=%6.1f,y=%6.1f,z=%6.1f",
                imu.getLinearAcceleration().xAccel,
                imu.getLinearAcceleration().yAccel,
                imu.getLinearAcceleration().zAccel);
    }
    @Override
    public void write() {

    }
}
