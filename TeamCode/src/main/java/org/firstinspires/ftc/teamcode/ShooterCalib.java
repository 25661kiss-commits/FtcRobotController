package org.firstinspires.ftc.teamcode;

import static java.lang.Math.tan;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp
public class ShooterCalib extends OpMode {
    private DcMotorEx shooterMotor;
    private DcMotorEx shooterMotor2;
    private DcMotor rtIntake;
    private DcMotor ltIntake;
    private CRServo rtFire;
    private CRServo ltFire;
    private Limelight3A limelight3A;
    private double shooterVelocity = 0;
    @Override
    public void init() {
        shooterMotor = hardwareMap.get(DcMotorEx.class,"shooter_motor");
        shooterMotor2 = hardwareMap.get(DcMotorEx.class,"shooter2");
        shooterMotor2.setDirection(DcMotorSimple.Direction.REVERSE);
        ltIntake = hardwareMap.get(DcMotor.class,"left_intake_motor");
        rtIntake = hardwareMap.get(DcMotor.class,"right_intake_motor");
        rtFire = hardwareMap.get(CRServo.class,"right_fire_servo");
        ltFire = hardwareMap.get(CRServo.class,"left_fire_servo");
        limelight3A = hardwareMap.get(Limelight3A.class,"limelight");
        limelight3A.pipelineSwitch(5);//1 is green
    }
    @Override
    public void start() {
        limelight3A.start();
    }
    @Override
    public void loop() {
        if(gamepad1.dpad_up){
            shooterVelocity += 20;
            try {
                Thread.sleep(100);
            } catch (InterruptedException e) {
                throw new RuntimeException(e);
            }
        }
        if(gamepad1.dpad_down){
            shooterVelocity -= 20;
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
        if(shooterMotor.getVelocity() < shooterVelocity){
            shooterMotor.setPower(1);
        }else{
            shooterMotor.setPower(0.5);
        }
        if(shooterMotor2.getVelocity() < shooterVelocity){
            shooterMotor2.setPower(1);
        }else{
            shooterMotor2.setPower(0.5);
        }

        telemetry.addData("power:", shooterVelocity);
        telemetry.addData("speed:",shooterMotor.getVelocity());
        telemetry.addData("speed2:",shooterMotor2.getVelocity());
        telemetry.addData("distance:",getLLDistance());
    }
    private double getLLDistance(){
        LLResult llResult = limelight3A.getLatestResult();
        if (llResult != null & llResult.isValid()) {
            //telemetry.addData("target X offset", llResult.getTx());
            //telemetry.addData("Target y offset", llResult.getTy());
            //telemetry.addData("Target area offset", llResult.getTa());
            double y = llResult.getTy();
            double angleRadians = 3.14*((23+y)/180);
            double targetDist = 26.25 / tan(angleRadians);
            //telemetry.addData("distance:",targetDist);
            return targetDist;
        }else{
            return -1;
        }
    }
}
