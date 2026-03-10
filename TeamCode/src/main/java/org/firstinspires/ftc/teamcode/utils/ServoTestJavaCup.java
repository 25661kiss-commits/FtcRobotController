package org.firstinspires.ftc.teamcode.utils;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
@Disabled
public class ServoTestJavaCup extends OpMode {




    private Servo ballStopLeft;
    private Servo ballStopRight;





    @Override
    public void init(){


        ballStopLeft = hardwareMap.get(Servo.class,"ball_stop_left");
        ballStopRight = hardwareMap.get(Servo.class,"ball_stop_right");


        ballStopLeft.setDirection(Servo.Direction.FORWARD);
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
