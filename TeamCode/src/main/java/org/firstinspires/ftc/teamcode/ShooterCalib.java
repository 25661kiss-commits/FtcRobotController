package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@TeleOp
public class ShooterCalib extends OpMode {
    private DcMotorEx shooterMotor;
    private DcMotor rtIntake;
    private DcMotor ltIntake;
    private CRServo rtFire;
    private CRServo ltFire;
    private double shooterPower = 0;
    @Override
    public void init() {
        shooterMotor = hardwareMap.get(DcMotorEx.class,"shooter_motor");
        ltIntake = hardwareMap.get(DcMotor.class,"left_intake_motor");
        rtIntake = hardwareMap.get(DcMotor.class,"right_intake_motor");
        rtFire = hardwareMap.get(CRServo.class,"right_fire_servo");
        ltFire = hardwareMap.get(CRServo.class,"left_fire_servo");
    }

    @Override
    public void loop() {
        if(gamepad1.dpad_up){
            shooterPower += 0.025;
            try {
                Thread.sleep(100);
            } catch (InterruptedException e) {
                throw new RuntimeException(e);
            }
        }
        if(gamepad1.dpad_down){
            shooterPower -= 0.025;
            try {
                Thread.sleep(100);
            } catch (InterruptedException e) {
                throw new RuntimeException(e);
            }
        }
        if(gamepad1.left_bumper){
            rtFire.setPower(-1);
        }else{
            rtFire.setPower(0);
        }
        if(gamepad1.right_bumper){
            ltFire.setPower(1);
        }else{
            ltFire.setPower(0);
        }
        ltIntake.setPower(-gamepad1.left_stick_y);
        rtIntake.setPower(-gamepad1.right_stick_y);
        shooterMotor.setPower(shooterPower);
        telemetry.addData("power:",shooterPower);
        telemetry.addData("speed:",shooterMotor.getVelocity());
    }
}
