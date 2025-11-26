package org.firstinspires.ftc.teamcode;

import static java.lang.Math.tan;

import com.qualcomm.hardware.limelightvision.LLResult;
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

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.mechaisms.MecanumDriveTele;

@TeleOp
public class TeleOpWW26 extends OpMode {
    MecanumDriveTele drive = new MecanumDriveTele();
    private Limelight3A limelight3A;
    double forward,strafe,rotate;
    private final double targetSpeedHigh = 0.7;
    private final double targetSpeedMed = 0.4;
    private final double targetSpeedLow = 0.2;
    private DcMotorEx shooterMotor;
    private DcMotorEx shooterMotor2;
    private DcMotor rtIntake;
    private DcMotor ltIntake;
    private CRServo rtFire;
    private CRServo ltFire;
    private RevColorSensorV3 color;
    private RevColorSensorV3 color2;
    private RevColorSensorV3 rtcolor;
    private RevColorSensorV3 rtcolor2;
    private double shooterPower = 0.5;
    private IMU imu;//my stuff daniel
    private boolean fire = false;
    private double idleSpeed = 0;
    private double setSpeed = 0;
    private double speedGoal = 0;
    private DigitalChannel led0;
    private DigitalChannel led1;
    private DigitalChannel led2;
    private DigitalChannel led3;

    @Override
    public void init(){
        color = hardwareMap.get(RevColorSensorV3.class,"color_sensor_left");
        color2 = hardwareMap.get(RevColorSensorV3.class,"color_sensor_left_front");
        rtcolor = hardwareMap.get(RevColorSensorV3.class,"color_sensor_right");
        rtcolor2 = hardwareMap.get(RevColorSensorV3.class,"color_sensor_right_front");
        drive.init(hardwareMap, DcMotor.RunMode.RUN_USING_ENCODER);
        imu = drive.getImu();
        ltIntake = hardwareMap.get(DcMotor.class,"left_intake_motor");
        rtIntake = hardwareMap.get(DcMotor.class,"right_intake_motor");
        shooterMotor = hardwareMap.get(DcMotorEx.class,"shooter_motor");
        shooterMotor2 = hardwareMap.get(DcMotorEx.class,"shooter2");
        shooterMotor2.setDirection(DcMotorSimple.Direction.REVERSE);
        ltIntake = hardwareMap.get(DcMotor.class,"left_intake_motor");
        rtIntake = hardwareMap.get(DcMotor.class,"right_intake_motor");
        rtFire = hardwareMap.get(CRServo.class,"right_fire_servo");
        ltFire = hardwareMap.get(CRServo.class,"left_fire_servo");
        limelight3A = hardwareMap.get(Limelight3A.class,"limelight");
        limelight3A.pipelineSwitch(5);//1 is green
        led0 = hardwareMap.get(DigitalChannel.class,"led0");
        led1 = hardwareMap.get(DigitalChannel.class,"led1");
        led2 = hardwareMap.get(DigitalChannel.class,"led2");
        led3 = hardwareMap.get(DigitalChannel.class,"led3");
        led0.setMode(DigitalChannel.Mode.OUTPUT);
        led1.setMode(DigitalChannel.Mode.OUTPUT);
        led2.setMode(DigitalChannel.Mode.OUTPUT);
        led3.setMode(DigitalChannel.Mode.OUTPUT);

    }

    @Override
    public void start() {
        limelight3A.start();
    }

    @Override
    public void loop(){
        //movement


        forward = gamepad1.left_stick_y;
        strafe = (-gamepad1.left_stick_x)*0.5;
        rotate = -gamepad1.right_stick_x;

        double speeed = getLLRotationOffset();
        double inject = 0;

        //line up with target
        if(gamepad1.right_bumper) {
            //fire line up
            double rot = 0;
            LLResult llResult = limelight3A.getLatestResult();
            if(llResult != null & llResult.isValid()){
                rot = llResult.getTx();
            }else{
                rot = -1;
            }

            if(rot == -1){//cant see the tag or other problem

                    drive.drive(0,0,-targetSpeedHigh);

            }else{//tag visible
                if (rot >= 20){drive.drive(0,strafe,-targetSpeedMed);}
                else if(rot < 20 & rot > 3){ drive.drive(0,strafe,-targetSpeedLow);}
                else if(rot <=3  & rot >= -3){ drive.drive(0,strafe,0);}
                else if(rot > -20 & rot < -3){ drive.drive(0,strafe,targetSpeedLow);}
                //else if(rot <= -20){drive.drive(0,0,-targetSpeedMed);}
                else{drive.drive(0,strafe,0);}//catchall
            }
        }else if(gamepad1.left_bumper){
            //fire line up
            double rot = 0;
            LLResult llResult = limelight3A.getLatestResult();
            if(llResult != null & llResult.isValid()){
                rot = llResult.getTx();
            }else{
                rot = -1;
            }

            if(rot == -1){//cant see the tag or other problem

                drive.drive(0,0,targetSpeedHigh);

            }else{//tag visible
                //if (rot >= 20){drive.drive(0,0,-targetSpeedMed);}
                /*else*/ if(rot <= -20){drive.drive(0,strafe,targetSpeedMed);}
                else if(rot > -20 & rot < -3){ drive.drive(0,strafe,targetSpeedLow);}
                else if(rot <=3  & rot >= -3){ drive.drive(0,strafe,0);}
                else if(rot < 20 & rot > 3){ drive.drive(0,strafe,-targetSpeedLow);}//you can strafe arround the target
                else{drive.drive(0,strafe,0);}//catchall
            }
        }else{
            drive.drive(forward,strafe,rotate);
        }
        setSpeed = idleSpeed;
        speedGoal = 0;
        if(gamepad2.right_trigger > 0.2 || gamepad2.left_trigger > 0.2){//trigger pressed!!!

            setSpeed = idleSpeed;
        }else{
            setSpeed = idleSpeed;
        }
        if(gamepad2.a){
            idleSpeed = 0.525;
        }

        if(gamepad2.b){
            idleSpeed = 0;
        }
        if(gamepad2.dpad_up){
            idleSpeed += 0.025;
            drive.drive(0,0,0);
            try {
                Thread.sleep(200);
            } catch (InterruptedException e) {
                throw new RuntimeException(e);
            }
        }
        if(gamepad2.dpad_down){
            idleSpeed -= 0.025;
            drive.drive(0,0,0);
            try {
                Thread.sleep(200);
            } catch (InterruptedException e) {
                throw new RuntimeException(e);
            }
        }
        double velocity = shooterMotor.getVelocity();
        if(gamepad2.right_bumper){
            if(rtcolor.getDistance(DistanceUnit.CM) > 3.5){
                rtFire.setPower(-1);
                if(rtcolor.getDistance(DistanceUnit.CM) < 3.5 && rtcolor2.getDistance(DistanceUnit.CM) < 3) {
                    rtIntake.setPower(0);
                }else{
                    rtIntake.setPower(-0.5);//right front intake speed
                }
            }else{
                rtFire.setPower(0);
                if(rtcolor.getDistance(DistanceUnit.CM) < 3.5 && rtcolor2.getDistance(DistanceUnit.CM) < 3) {
                    rtIntake.setPower(0);
                }else{
                    rtIntake.setPower(-0.5);//right front intake speed
                }
            }
        }else if(gamepad2.right_trigger > 0.2){//fire
            rtFire.setPower(-1);
            rtIntake.setPower(-1);
            /*
            double distance = getLLDistance();
            if(distance < 30){
                //nothing
                setSpeed = idleSpeed;
                speedGoal = 0;
            }else if(distance <= 35){
                setSpeed = 525;
                speedGoal = 500;
            }else if(distance <= 48){
                setSpeed = 0.55;
                speedGoal = 500;
            }else if(distance <= 85){
                setSpeed = 0.575;
                speedGoal = 600;
            }else if(distance > 85){
                setSpeed = 0.6;
                speedGoal = 650;
            }*/

        }else{
            rtFire.setPower(0);
            rtIntake.setPower(-gamepad2.right_stick_y);

        }
        if(gamepad2.left_bumper){
            if(color.getDistance(DistanceUnit.CM) > 3.5){
                ltFire.setPower(1);
                if(color.getDistance(DistanceUnit.CM) < 3.5 && color2.getDistance(DistanceUnit.CM) < 3) {
                    ltIntake.setPower(0);
                }else{
                    ltIntake.setPower(-0.5);//left front intake speed
                }
            }else{
                ltFire.setPower(0);
                if(color.getDistance(DistanceUnit.CM) < 3.5 && color2.getDistance(DistanceUnit.CM) < 3) {
                    ltIntake.setPower(0);
                }else{
                    ltIntake.setPower(-0.5);//left front intake speed
                }
            }
        }else if(gamepad2.left_trigger > 0.2){//fire
            ltFire.setPower(1);
            ltIntake.setPower(-1);
            /*
            double distance = getLLDistance();
            if(distance < 30){
                //nothing
                setSpeed = idleSpeed;
                speedGoal = 0;
            }else if(distance <= 35){
                setSpeed = 525;
                speedGoal = 500;
            }else if(distance <= 48){
                setSpeed = 0.55;
                speedGoal = 500;
            }else if(distance <= 85){
                setSpeed = 0.575;
                speedGoal = 600;
            }else if(distance > 85){
                setSpeed = 0.6;
                speedGoal = 650;
            }*/

        }else{
            ltFire.setPower(0);
            ltIntake.setPower(-gamepad2.left_stick_y);

        }
        double distance = getLLDistance();
        int TargetVelocity;
        if(distance > 80){
            TargetVelocity = 730;
        }else if(distance < 70 & distance > 40){
            TargetVelocity = (int) (580 + ((distance - 40)*10));
        }else if(distance < 36){
            TargetVelocity = 580;
        }else{
            TargetVelocity = 630;
        }
        if(velocity < TargetVelocity){//speed up
            shooterMotor.setPower(1);
            led0.setState(true);//off leds
            led1.setState(true);
            led2.setState(true);
            led3.setState(true);
        }else {//fast eneough
            shooterMotor.setPower(0.5);
            led0.setState(false);
            led1.setState(false);
            led2.setState(false);
            led3.setState(false);
        }
        if(shooterMotor2.getVelocity() < TargetVelocity){//speed up
            shooterMotor2.setPower(1);
        }else {//fast eneough
            shooterMotor2.setPower(0.5);
        }


        telemetry.addData("power:",setSpeed);
        telemetry.addData("speed:",velocity);
        telemetry.addData("speed2:",shooterMotor2.getVelocity());
        telemetry.addData("target",TargetVelocity);


    }

    private double getLLRotationOffset(){
        LLResult llResult = limelight3A.getLatestResult();
        if (llResult != null & llResult.isValid()) {
            telemetry.addData("target X offset", llResult.getTx());
            telemetry.addData("Target y offset", llResult.getTy());
            telemetry.addData("Target area offset", llResult.getTa());
            double y = llResult.getTy();
            double angleRadians = 3.14*((23+y)/180);
            double targetDist = 26.25 / tan(angleRadians);
            telemetry.addData("distance:",targetDist);
            return llResult.getTx();
        }else{
            return -1;
        }

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
    private void delayMs(int millis){
        try {
            Thread.sleep(millis);

        } catch (InterruptedException e) {
            throw new RuntimeException(e);

        }
    }
}
