package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp
public class FULLPOWER extends OpMode {
    private DcMotorEx shooterMotor2;
    @Override
    public void init() {
        shooterMotor2 = hardwareMap.get(DcMotorEx.class,"shooter_motor");
    }

    @Override
    public void loop() {
        shooterMotor2.setPower(1);
        telemetry.addData("speed",shooterMotor2.getVelocity());
    }
}
