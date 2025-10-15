package org.firstinspires.ftc.teamcode.mechaisms;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class TestBenchD {
    private DcMotor motor;//the motor
    private double TicksPerRev;
    public void init(HardwareMap hwMap){
        motor = hwMap.get(DcMotor.class, "testmotor");
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        TicksPerRev = motor.getMotorType().getTicksPerRev();
    }
    public void setMotorSpeed(double speed){
        motor.setPower(speed);
    }
    public double getMotorRevs(){
        return motor.getCurrentPosition() / TicksPerRev;//3-1 + 5-1 ratio is 15-1 ratio
    }
}
