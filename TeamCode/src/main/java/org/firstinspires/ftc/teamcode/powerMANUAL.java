package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp
@Disabled
public class powerMANUAL extends OpMode {
    DcMotorEx Motor;
    double time;
    ElapsedTime milis = new ElapsedTime();
    @Override
    public void init() {
        Motor = hardwareMap.get(DcMotorEx.class,"shooter_motor");

    }

    @Override
    public void loop() {

        if(gamepad1.a) {
            Motor.setPower(1);
            if(Motor.getVelocity() < 1200) {
                time = milis.milliseconds();
            }
        }else{
            Motor.setPower(0.5);
            milis.reset();
        }

        telemetry.addData("time",time);
        telemetry.addData("speed",Motor.getVelocity());
    }
}
