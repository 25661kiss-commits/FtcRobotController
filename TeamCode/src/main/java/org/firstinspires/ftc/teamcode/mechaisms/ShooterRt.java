package org.firstinspires.ftc.teamcode.mechaisms;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class ShooterRt {

    private DcMotorEx fire;
    private Servo shoot;
    private DcMotor suckSuckSuck;
    public void init(HardwareMap HwMap){
        fire = HwMap.get(DcMotorEx.class,"front_left_motor");

    }
    public void shoot(double distance){
        double shooterPower = 0;//basic motor power
        double shooterSpeed = 0;
        if(distance < 30){
            return;
        }else if(distance <= 35){
            shooterPower = 0.5;
            shooterSpeed = 700;
        }else if(distance <= 48){
            shooterPower = 0.625;
            shooterSpeed = 720;
        }else if(distance <= 85){
            shooterPower = 0.75;
            shooterSpeed = 800;
        }else if(distance > 85){
            shooterPower = 0.85;
            shooterSpeed = 900;
        }
        fire.setPower(shooterPower);
        while(fire.getVelocity() < shooterSpeed){
            telemetry.addData("speed",fire.getVelocity());

            telemetry.update();
            telemetry.clear();
        }
        //!!!!!push ball here!!!!!!!!!!!!!
        telemetry.addData("speed",fire.getVelocity());
        telemetry.update();
        telemetry.clear();
        fire.setPower(0);
    }
}
