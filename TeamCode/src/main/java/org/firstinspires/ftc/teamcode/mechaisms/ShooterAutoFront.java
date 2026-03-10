package org.firstinspires.ftc.teamcode.mechaisms;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class ShooterAutoFront {
    public DcMotorEx leftShooter;
    public DcMotorEx rightShooter;
    public DcMotor FrontIntake;
    public DcMotor RearIntake;
    public Servo BallStopLeft;
    public Servo BallStopRight;

    public int targetSpeed;

    public enum ShooterState{
        SHOOT,
        IDLE,
    }
    public ShooterState state;
    public ShooterAutoFront(HardwareMap hwmap){
        BallStopLeft = hwmap.get(Servo.class,"ball_stop_left");
        BallStopRight = hwmap.get(Servo.class,"ball_stop_right");
        RearIntake = hwmap.get(DcMotor.class,"left_intake_motor");
        BallStopLeft.setDirection(Servo.Direction.REVERSE);
        BallStopRight.setDirection(Servo.Direction.FORWARD);
        RearIntake.setDirection(DcMotorSimple.Direction.REVERSE);
        leftShooter = hwmap.get(DcMotorEx.class,"shooter_motor");
        rightShooter = hwmap.get(DcMotorEx.class,"shooter2");
        leftShooter.setDirection(DcMotorSimple.Direction.FORWARD);
        rightShooter.setDirection(DcMotorSimple.Direction.REVERSE);
    }


    public void update(){

        switch (state){
            case IDLE:
                if(leftShooter.getVelocity() < targetSpeed){
                    leftShooter.setPower(1);
                }else{
                    leftShooter.setPower(0.5);
                }
                if(rightShooter.getVelocity() < targetSpeed){
                    rightShooter.setPower(1);
                }else{
                    rightShooter.setPower(0.5);
                }
                RearIntake.setPower(0);
                BallStopLeft.setPosition(0);
                BallStopRight.setPosition(0.25);
                break;
            case SHOOT:
                if(leftShooter.getVelocity() < targetSpeed){
                    leftShooter.setPower(1);
                }else{
                    leftShooter.setPower(0.5);
                }
                if(rightShooter.getVelocity() < targetSpeed){
                    rightShooter.setPower(1);
                }else{
                    rightShooter.setPower(0.5);
                }
                BallStopLeft.setPosition(0.3);
                BallStopRight.setPosition(0.7);
                if(rightShooter.getVelocity() >= targetSpeed && leftShooter.getVelocity() >= targetSpeed){
                    RearIntake.setPower(0.75);
                    FrontIntake.setPower(0.75);
                }else{
                    RearIntake.setPower(0);
                    FrontIntake.setPower(0);
                }

                break;
            default:
                break;

        }
    }
    public void setShooterState(ShooterState shootState){
        state = shootState;
        switch (state){
            case IDLE:
                FrontIntake.setPower(0);
                break;
            case SHOOT:
                FrontIntake.setPower(1);
                break;
            default:
                break;
        }
    }
}
