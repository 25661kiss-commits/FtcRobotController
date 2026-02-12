package org.firstinspires.ftc.teamcode;

import static java.lang.Math.abs;
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
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.mechaisms.MecanumDriveTele;
import org.firstinspires.ftc.teamcode.mechaisms.gobuildaPinpoint;

@Autonomous
public class AutoBackBlueBF extends OpMode {

    final double TagDist= 13.125;
    MecanumDriveTele drive = new MecanumDriveTele();
    private Limelight3A limelight3A;
    double forward,strafe,rotate;
    private final double targetSpeedHigh = 0.4;
    private final double targetSpeedMed = 0.2;
    private final double targetSpeedLow = 0.1;
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
    private Servo ballStopLeft;
    private Servo ballStopRight;
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
        limelight3A.pipelineSwitch(3);//1 is green
        led0 = hardwareMap.get(DigitalChannel.class,"led0");
        led1 = hardwareMap.get(DigitalChannel.class,"led1");
        led2 = hardwareMap.get(DigitalChannel.class,"led2");
        led3 = hardwareMap.get(DigitalChannel.class,"led3");
        led0.setMode(DigitalChannel.Mode.OUTPUT);
        led1.setMode(DigitalChannel.Mode.OUTPUT);
        led2.setMode(DigitalChannel.Mode.OUTPUT);
        led3.setMode(DigitalChannel.Mode.OUTPUT);

        ballStopLeft = hardwareMap.get(Servo.class,"ball_stop_left");
        ballStopRight = hardwareMap.get(Servo.class,"ball_stop_right");


        ballStopLeft.setDirection(Servo.Direction.REVERSE);
        ballStopRight.setDirection(Servo.Direction.FORWARD);
        odo.resetPosAndIMU();
        odo.update();
        delayMs(1000);
    }
    @Override
    public void start() {
        limelight3A.start();
        delayMs(2000);
    }


    private boolean done = true;
    @Override
    public void loop() {
        if (done) {
            shooterMotor2.setPower(1);
            shooterMotor.setPower(1);
            ballStopLeft.setPosition(0.7);
            ballStopRight.setPosition(0.7);
            odo.update();
            done = false;
            while(abs(odo.getPosX(DistanceUnit.CM)) < 5) {
                drive.drive(-0.5,0,0);
                odo.update();
                ModulateSpeed(740);
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
                drive.drive(0,0,0.2);
                ModulateSpeed(740);
            }
            drive.drive(0,0,0);
            int stooop = 0;
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

                    drive.drive(0,0,-targetSpeedHigh);

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
                ModulateSpeed(740);
            }

            double dist = getLLDistance();
            double targetspeed = 740;
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
            //!\\get back up to speed
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
            rtFire.setPower(0);
            ltFire.setPower(0);
            shooterMotor2.setPower(0.5);
            shooterMotor.setPower(0.5);
            shooterMotor2.setPower(0);
            shooterMotor.setPower(0);
            //!\\big curved turn--------------------------------------------------------------------
            rtIntake.setPower(0.85);//was .75
            while(abs(odo.getPosX(DistanceUnit.CM)) < 30 && abs(odo.getPosY(DistanceUnit.CM)) < 36){//!\\ control turn
                drive.drive(-0.5,0,0.25);
                odo.update();
                telemetry.addData("angle",odo.getHeading(AngleUnit.DEGREES));
                telemetry.addData("x",odo.getPosX(DistanceUnit.CM));
                telemetry.addData("y",odo.getPosY(DistanceUnit.CM));
                telemetry.update();
                telemetry.clear();
                ModulateSpeed(740);
            }
            rtIntake.setPower(0.75);
            drive.drive(0,0,0);
            //!\\straghten out----------------------------------------------------------------------
            int goodloops = 0;
            while(goodloops < 5){
                if(odo.getHeading(AngleUnit.DEGREES) < 92 && odo.getHeading(AngleUnit.DEGREES) > 88){
                    drive.drive(0,0,0);
                    goodloops++;
                    telemetry.addData("on target",0);
                }else{
                    if(abs(odo.getHeading(AngleUnit.DEGREES)) > 90){
                        drive.drive(0,0,-0.025);
                    }else{
                        drive.drive(0,0,0.025);
                    }
                    goodloops = 0;
                }
                odo.update();
                telemetry.addData("angle",odo.getHeading(AngleUnit.DEGREES));
                telemetry.addData("x",odo.getPosX(DistanceUnit.CM));
                telemetry.addData("y",odo.getPosY(DistanceUnit.CM));
                telemetry.update();
                telemetry.clear();
            }
            //!\\ start of pick up balls------------------------------------------------------------

            rtFire.setPower(0);
            odo.update();
            double ypos = odo.getPosX(DistanceUnit.CM);
            while(abs(ypos-odo.getPosX(DistanceUnit.CM)) < 2){
                odo.update();
                if(odo.getHeading(AngleUnit.DEGREES) > 90){
                    drive.drive(0,-0.2,-0.1);
                }else{
                    drive.drive(0,-0.2,0.1);
                }
                ModulateSpeed(740);
            }
            drive.drive(0,0,0);
            shooterMotor2.setPower(0.6);
            shooterMotor.setPower(0.6);
            ballStopRight.setPosition(0.3);
            ballStopLeft.setPosition(0.25);
            drive.drive(0,0,0);
            rtIntake.setPower(0.85);//was .75
            rtFire.setPower(-0.2);//was .3
            ltFire.setPower(0);
            //!\\ after slide picup balls on diag -----------------------------------------------------
            drive.drive(-0.15,-0.11,0);
            double xpos = odo.getPosY(DistanceUnit.CM);
            while(distb.getDistance(DistanceUnit.CM) > 37 || abs(xpos-odo.getPosY(DistanceUnit.CM)) < 35){
                if(color.getDistance(DistanceUnit.CM) <=3.4){
                    ltFire.setPower(0);
                }
                if(rtcolor.getDistance(DistanceUnit.CM) <= 3.4){
                    rtFire.setPower(0);
                }
                ModulateSpeed(740);
                odo.update();
            }

            drive.drive(0.0,0,0);
            rtIntake.setPower(0);
            ltFire.setPower(0);
            //!\\backup-----------------------------------------------------------------------------
            drive.drive(0,0,0);
            rtIntake.setPower(0);
            ltFire.setPower(0);
            rtFire.setPower(0);
            odo.update();
            xpos = odo.getPosY(DistanceUnit.CM);
            while(abs(xpos-odo.getPosY(DistanceUnit.CM)) < 35){
                odo.update();
                if(odo.getHeading(AngleUnit.DEGREES) > 90){
                    drive.drive(0.5,0,-0.1);
                }else{
                    drive.drive(0.5,0,0.1);
                }
                ModulateSpeed(740);
            }
            drive.drive(0,0,0);
            stooop = 0;

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

                    drive.drive(0,0,-targetSpeedHigh);

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
                ModulateSpeed(740);
            }
            while(getLLDistance() < 110){
                drive.drive(0.5,0,0);
                ModulateSpeed(740);
            }
            drive.drive(0,0,0);

            odo.update();

            drive.drive(0,0,0);
            stooop = 0;
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

                    drive.drive(0,0,-targetSpeedHigh);

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
                ModulateSpeed(740);
            }

            dist = getLLDistance();
            targetspeed = 740;
            while(shooterMotor2.getVelocity() < targetspeed || shooterMotor.getVelocity() < targetspeed){
                ModulateSpeed(740);
                telemetry.addData("speed1:", shooterMotor.getVelocity());
                telemetry.addData("speed2:", shooterMotor2.getVelocity());
                telemetry.addData("speedt:", targetspeed);
                telemetry.update();
                telemetry.clear();
            }
            rtFire.setPower(-1);
            ltFire.setPower(1);
            ballStopLeft.setPosition(0.7);
            ballStopRight.setPosition(0.7);
            timer.reset();
            while(timer.milliseconds() < 1000){
                ModulateSpeed(740);
            }
            //!\\get back up to speed
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
                ModulateSpeed(740);
            }
            drive.drive(-0.5,0,0);
            rtFire.setPower(0);
            ltFire.setPower(0);
            rtIntake.setPower(0);
            shooterMotor2.setPower(0.5);
            shooterMotor.setPower(0.5);
            shooterMotor2.setPower(0);
            shooterMotor.setPower(0);

            ltIntake.setPower(0);
            rtIntake.setPower(0);
            odo.update();
            delayMs(500);
            drive.drive(0,0,0);
            ltIntake.setPower(0);

            ltIntake.setPower(0);
            rtIntake.setPower(0);
            shooterMotor2.setPower(0);
            shooterMotor.setPower(0);
            rtFire.setPower(0);
            ltFire.setPower(0);
            ltIntake.setPower(0);
        }
        drive.drive(0,0,0);
        telemetry.addData("angle",odo.getHeading(AngleUnit.DEGREES));
        telemetry.addData("x",odo.getPosX(DistanceUnit.CM));
        telemetry.addData("y",odo.getPosY(DistanceUnit.CM));
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
            telemetry.addData("target X offset", llResult.getTx());
            telemetry.addData("Target y offset", llResult.getTy());
            telemetry.addData("Target area offset", llResult.getTa());
            double y = llResult.getTy();
            double angleRadians = 3.14*((19.97+y)/180);
            double targetDist = 18.25 / tan(angleRadians);
            telemetry.addData("distance:",targetDist);
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
    private void ModulateSpeed(int targetspeed){
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
        /*telemetry.addData("speed1:", shooterMotor.getVelocity());
        telemetry.addData("speed2:", shooterMotor2.getVelocity());
        telemetry.addData("speedt:", targetspeed);
        telemetry.update();
        telemetry.clear();*/
    }
}
