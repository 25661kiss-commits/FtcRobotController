package org.firstinspires.ftc.teamcode.mechaisms;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class TestBench1 {

    private DigitalChannel touchSensor;
    private DcMotor motor; // testmotor
    private double ticksPerRev;

    public void init(HardwareMap hwMap) {
        touchSensor = hwMap.get(DigitalChannel.class,"touch_sensor");
        touchSensor.setMode(DigitalChannel.Mode.INPUT);
        motor = hwMap.get(DcMotor.class, "motor");
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor.setDirection(DcMotorSimple.Direction.REVERSE);
        ticksPerRev = motor.getMotorType().getTicksPerRev();


    }
    public boolean isTouchSensorPressed() {
        return !touchSensor.getState();

    }
    public void setMotorSpeed(double speed) {
        //this accepts values from -1.0 to 1.0
        motor.setPower(speed);
    }
    public double getMotorRevs() {
        return motor.getCurrentPosition()/ticksPerRev;
    }
    public void setMotorZeroBehavoir(DcMotor.ZeroPowerBehavior zeroBehavior) {


    }

}
