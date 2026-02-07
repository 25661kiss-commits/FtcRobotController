package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.mechaisms.MecanumDriveTele;

@TeleOp
public class ServoTestJavaCup extends OpMode {




    private Servo ballStopLeft;
    private Servo ballStopRight;





    @Override
    public void init(){


        ballStopLeft = hardwareMap.get(Servo.class,"ball_stop_left");
        ballStopRight = hardwareMap.get(Servo.class,"ball_stop_right");


        ballStopLeft.setDirection(Servo.Direction.REVERSE);
        ballStopRight.setDirection(Servo.Direction.FORWARD);

    }

    @Override
    public void loop() {
        if(gamepad2.left_bumper){

            ballStopLeft.setPosition(0.6);
        }else{
            ballStopLeft.setPosition(0.3);
        }
        if(gamepad2.right_bumper){

            ballStopRight.setPosition(0.6);
        }else{
            ballStopRight.setPosition(0.325);
        }
    }
}
