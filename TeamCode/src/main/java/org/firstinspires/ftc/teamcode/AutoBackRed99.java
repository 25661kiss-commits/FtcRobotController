package org.firstinspires.ftc.teamcode;

import static java.lang.Math.abs;
import static java.lang.Math.min;
import static java.lang.Math.tan;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.mechaisms.MecanumDriveTele;
import org.firstinspires.ftc.teamcode.mechaisms.gobuildaPinpoint;

@Autonomous
public class AutoBackRed99 extends OpMode {
    MecanumDriveTele drive = new MecanumDriveTele();
    private Limelight3A limelight3A;
    double forward,strafe,rotate;
    private final double targetSpeedHigh = 0.4;
    private final double targetSpeedMed = 0.4;
    private final double targetSpeedLow = 0.2;
    private DcMotorEx shooterMotor;
    private DcMotor rtIntake;
    private DcMotor ltIntake;
    private CRServo rtFire;
    private CRServo ltFire;
    gobuildaPinpoint pin = new gobuildaPinpoint();
    GoBildaPinpointDriver odo;
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
    private DistanceSensor dista;
    private DistanceSensor distb;
    private RevColorSensorV3 color;//color sensor rear left
    private RevColorSensorV3 color2;//color sensor front left
    private RevColorSensorV3 rtcolor;//color sensor rear left
    private RevColorSensorV3 rtcolor2;//color sensor front left
    private DcMotorEx shooterMotor2;
    private final double intakeRt = 0.5;
    private final double intakeLt = -0.5;
    @Override
    public void init() {
        drive.init(hardwareMap, DcMotor.RunMode.RUN_USING_ENCODER);
        dista = hardwareMap.get(DistanceSensor.class,"dist_sensor_right");
        distb = hardwareMap.get(DistanceSensor.class,"dist_sensor_left");
        color = hardwareMap.get(RevColorSensorV3.class,"color_sensor_left_front");
        color2 = hardwareMap.get(RevColorSensorV3.class,"color_sensor_left_front");
        rtcolor = hardwareMap.get(RevColorSensorV3.class,"color_sensor_right_front");
        rtcolor2 = hardwareMap.get(RevColorSensorV3.class,"color_sensor_right_front");//compile issue
        imu = drive.getImu();
        pin.init(hardwareMap);
        odo = pin.getPinpoint();
        ltIntake = hardwareMap.get(DcMotor.class,"left_intake_motor");
        rtIntake = hardwareMap.get(DcMotor.class,"right_intake_motor");
        shooterMotor = hardwareMap.get(DcMotorEx.class,"shooter_motor");
        shooterMotor2 = hardwareMap.get(DcMotorEx.class,"shooter2");
        shooterMotor2.setDirection(DcMotorSimple.Direction.REVERSE);
        ltIntake = hardwareMap.get(DcMotor.class,"left_intake_motor");
        rtIntake = hardwareMap.get(DcMotor.class,"right_intake_motor");
        ltIntake.setDirection(DcMotorSimple.Direction.REVERSE);
        ltIntake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rtIntake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rtIntake.setDirection(DcMotorSimple.Direction.REVERSE);
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
        odo.resetPosAndIMU();
        odo.update();
    }
    @Override
    public void start() {
        limelight3A.start();
    }
    private boolean done = true;
    @Override
    public void loop() {
        if (done) {
            delayMs(1000);
            shooterMotor2.setPower(0.3);
            shooterMotor.setPower(0.3);
            odo.update();
            while(abs(odo.getPosX(DistanceUnit.CM)) < 5) {
                drive.drive(-0.5,0,0);
                odo.update();
            }
            drive.drive(0,0,0);
            boolean stox = true;
            while(stox){
                double rot = 0;
                LLResult llResult = limelight3A.getLatestResult();
                if(llResult != null & llResult.isValid()){
                    rot = llResult.getTx();
                }else{
                    rot = -1;
                }
                if(rot > -5 && rot < 5){
                    stox = false;
                }
                drive.drive(0,0,-0.2);
            }
            drive.drive(0,0,0);
            int stooop = 0;
            delayMs(400);
            while(stooop < 15){// aim
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
                    if (rot >= 20){drive.drive(0,strafe,-targetSpeedMed);}
                    else if(rot < 20 & rot > 2){ drive.drive(0,strafe,-targetSpeedLow);}
                    else if(rot <=2  & rot >= -2){ drive.drive(0,strafe,0); stooop++;}
                    else if(rot > -20 & rot < -2){ drive.drive(0,strafe,targetSpeedLow);}
                    //else if(rot <= -20){drive.drive(0,0,-targetSpeedMed);}
                    else{drive.drive(0,strafe,0);}//catchall

                }
                if(rot > 2 || rot < -2){
                    stooop = 0;
                }
                double velocity = shooterMotor.getVelocity();
                telemetry.addData("speed:",velocity);
            }

            double dist = getLLDistance();
            double targetspeed = 840;
            while(shooterMotor2.getVelocity() < targetspeed || shooterMotor.getVelocity() < targetspeed){
                if(shooterMotor.getVelocity() <= targetspeed){
                    shooterMotor.setPower(1);
                }else{
                    shooterMotor.setPower(0.5);
                }
                if(shooterMotor2.getVelocity() <= targetspeed){
                    shooterMotor2.setPower(1);
                }else{
                    shooterMotor2.setPower(0.5);
                }
                telemetry.addData("speed1:", shooterMotor.getVelocity());
                telemetry.addData("speed2:", shooterMotor2.getVelocity());
                telemetry.addData("speedt:", targetspeed);
                telemetry.update();
                telemetry.clear();
            }
            ElapsedTime timer = new ElapsedTime();//shoot 2
            rtFire.setPower(-1);
            ltFire.setPower(1);
            timer.reset();
            while(timer.milliseconds() < 3000){
                if(shooterMotor.getVelocity() <= targetspeed){
                    shooterMotor.setPower(1);
                }else{
                    shooterMotor.setPower(0.5);
                }
                if(shooterMotor2.getVelocity() <= targetspeed){
                    shooterMotor2.setPower(1);
                }else{
                    shooterMotor2.setPower(0.5);
                }
            }
//get back up to speed
            while(shooterMotor2.getVelocity() < targetspeed || shooterMotor.getVelocity() < targetspeed){
                if(shooterMotor.getVelocity() <= targetspeed){
                    shooterMotor.setPower(1);
                }else{
                    shooterMotor.setPower(0.5);
                }
                if(shooterMotor2.getVelocity() <= targetspeed){
                    shooterMotor2.setPower(1);
                }else{
                    shooterMotor2.setPower(0.5);
                }
                telemetry.addData("speed1:", shooterMotor.getVelocity());
                telemetry.addData("speed2:", shooterMotor2.getVelocity());
                telemetry.addData("speedt:", targetspeed);
                telemetry.addData("dist:", dist);
                telemetry.update();
                telemetry.clear();
            }
            //delayMs(2000);shoot 3rd ball
            rtIntake.setPower(0.75);
            timer.reset();
            while(timer.milliseconds() < 1000){
                if(shooterMotor.getVelocity() <= targetspeed){
                    shooterMotor.setPower(1);
                }else{
                    shooterMotor.setPower(0.5);
                }
                if(shooterMotor2.getVelocity() <= targetspeed){
                    shooterMotor2.setPower(1);
                }else{
                    shooterMotor2.setPower(0.5);
                }
            }
            shooterMotor2.setPower(0.5);
            shooterMotor.setPower(0.5);
            int goodlops = 0;
            while(goodlops < 12){
                odo.update();
                if(odo.getHeading(AngleUnit.DEGREES) < -88 && odo.getHeading(AngleUnit.DEGREES) > -90){
                    goodlops++;
                    drive.drive(0,0,0);
                }else{
                    goodlops = 0;
                    if(odo.getHeading(AngleUnit.DEGREES) > -89){
                        drive.drive(0,0,-0.2);
                    }else{
                        drive.drive(0,0,0.2);
                    }
                }

            }
            odo.update();
            drive.drive(0,0,0);
            double ypos = odo.getPosY(DistanceUnit.CM);
            while(abs(ypos - odo.getPosY(DistanceUnit.CM)) < (8.9*3)){
                drive.drive(-0.2,0,0);
                odo.update();
            }
            drive.drive(0,0,0);
            while(distb.getDistance(DistanceUnit.CM) > 23){
                if(odo.getHeading(AngleUnit.DEGREES) > -90){
                    drive.drive(0,0.2,-0.1);
                }else{
                    drive.drive(0,0.2,0.1);
                }
                odo.update();
            }
            drive.drive(0,0,0);
            delayMs(1000);
            while(distb.getDistance(DistanceUnit.CM) < 35){
                if(odo.getHeading(AngleUnit.DEGREES) > -90){
                    drive.drive(0,0.2,-0.1);
                }else{
                    drive.drive(0,0.2,0.1);
                }
                odo.update();
            }
            drive.drive(0,0,0);
            ltFire.setPower(0.25);
            rtIntake.setPower(1);
            while(color.getDistance(DistanceUnit.CM) > 3){
                drive.drive(-0.1,0,0);

            }
            ltFire.setPower(0);
            rtIntake.setPower(0);
            odo.update();
            ypos = odo.getPosX(DistanceUnit.CM);
            while(abs(ypos-odo.getPosX(DistanceUnit.CM)) < 6) {
                drive.drive(0,0.2,0);
                odo.update();
            }
            drive.drive(0,0,0);
            ltIntake.setPower(0);
            //second/third ball grab
            rtIntake.setPower(0.75);
            rtFire.setPower(-0.25);
            drive.drive(-0.1,0,0);
            while(distb.getDistance(DistanceUnit.CM) > 30){
                if(color.getDistance(DistanceUnit.CM) <=3){
                    rtFire.setPower(0);
                }
            }
            drive.drive(0.0,0,0);
            rtIntake.setPower(0);
            rtFire.setPower(0);

            ltIntake.setPower(0);
            rtIntake.setPower(0);
            odo.update();
            drive.drive(0,0,0);
            ltIntake.setPower(0);

            ltIntake.setPower(0);
            rtIntake.setPower(0);
            shooterMotor2.setPower(0);
            shooterMotor.setPower(0);
            rtFire.setPower(0);
            ltFire.setPower(0);
            ltIntake.setPower(0);
            delayMs(30000);
        }
        drive.drive(0,0,0);
    }
    private double getLLRotationOffset(){
        LLResult llResult = limelight3A.getLatestResult();
        if (llResult != null & llResult.isValid()) {
            telemetry.addData("target X offset", llResult.getTx());
            telemetry.addData("Target y offset", llResult.getTy());
            telemetry.addData("Target area offset", llResult.getTa());
            double y = llResult.getTy();
            double angleRadians = 3.14*((19+y)/180);
            double targetDist = 25.25 / tan(angleRadians);
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
