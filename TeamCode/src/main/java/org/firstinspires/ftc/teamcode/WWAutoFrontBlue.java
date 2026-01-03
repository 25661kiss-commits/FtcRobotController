package org.firstinspires.ftc.teamcode;

import static java.lang.Math.tan;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.mechaisms.MecanumDriveTele;
import org.firstinspires.ftc.teamcode.mechaisms.gobuildaPinpoint;

@Autonomous
@Disabled
public class WWAutoFrontBlue extends OpMode {
    MecanumDriveTele drive = new MecanumDriveTele();
    private Limelight3A limelight3A;
    double forward,strafe,rotate;
    private final double targetSpeedHigh = 0.7;
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
    private DcMotorEx shooterMotor2;
    @Override
    public void init() {
        drive.init(hardwareMap, DcMotor.RunMode.RUN_USING_ENCODER);
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
        if(done){
            shooterMotor.setPower(1);
            shooterMotor2.setPower(1);
            done = false;
            //once
            double dist = getLLDistance();
            while(dist < 30){
                dist = getLLDistance();
                drive.drive(0.5,0,0);
                double velocity = shooterMotor.getVelocity();
                telemetry.addData("speed:",velocity);
            }
            drive.drive(0,0,0);
            boolean stooop = true;
            delayMs(400);
            while(stooop){
                //fire line up
                double rot = 0;
                LLResult llResult = limelight3A.getLatestResult();
                if(llResult != null & llResult.isValid()){
                    rot = llResult.getTx();
                }else{
                    rot = -1;
                }

                if(rot == -1){//cant see the tag or other problem

                    //drive.drive(0,0,-targetSpeedHigh);

                }else{//tag visible
                    if (rot >= 20){drive.drive(0,strafe,-targetSpeedMed);}
                    else if(rot < 20 & rot > 3){ drive.drive(0,strafe,-targetSpeedLow);}
                    else if(rot <=3  & rot >= -3){ drive.drive(0,strafe,0); stooop = false;}
                    else if(rot > -20 & rot < -3){ drive.drive(0,strafe,targetSpeedLow);}
                    //else if(rot <= -20){drive.drive(0,0,-targetSpeedMed);}
                    else{drive.drive(0,strafe,0);}//catchall

                }
                double velocity = shooterMotor.getVelocity();
                telemetry.addData("speed:",velocity);
            }
            double velocity = shooterMotor.getVelocity();

            while(velocity <= 680){
                velocity = shooterMotor.getVelocity();
                telemetry.addData("speed:",velocity);
            }
            ElapsedTime timer = new ElapsedTime();
            timer.reset();

            while(timer.milliseconds() < 2000){
                ltFire.setPower(1);
                if(shooterMotor.getVelocity() <= 680){
                    shooterMotor.setPower(1);
                }else{
                    shooterMotor.setPower(0.5);
                }
                if(shooterMotor2.getVelocity() <= 680){
                    shooterMotor2.setPower(1);
                }else{
                    shooterMotor2.setPower(0.5);
                }
            }
            //fire second ball
            time  = this.getRuntime();
            timer.reset();

            while(timer.milliseconds() < 5000){
                rtFire.setPower(-1);
                ltFire.setPower(1);
                ltIntake.setPower(-0.75);
                if(shooterMotor.getVelocity() <= 680){
                    shooterMotor.setPower(1);
                }else{
                    shooterMotor.setPower(0.5);
                }
                if(shooterMotor2.getVelocity() <= 680){
                    shooterMotor2.setPower(1);
                }else{
                    shooterMotor2.setPower(0.5);
                }
            }
            ltIntake.setPower(0);
            timer.reset();
            ltFire.setPower(0);
            rtFire.setPower(0);
            odo.update();

            //odo.resetPosAndIMU();
            odo.update();
            drive.drive(0,0,0.2);
            // move over to first line of balls
            while(odo.getHeading(AngleUnit.DEGREES) < 90){
                odo.update();
            }
            drive.drive(-0.4,0,0);
            delayMs(1500);
            drive.drive(0,0,0);
            shooterMotor2.setPower(0);
            shooterMotor.setPower(0);
            rtFire.setPower(0);
            ltFire.setPower(0);
            ltIntake.setPower(0);


        }
        double velocity = shooterMotor.getVelocity();
        telemetry.addData("speed:",velocity);
        telemetry.addData("wedd",odo.getHeading(AngleUnit.DEGREES));

        odo.update();
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
            double angleRadians = 3.14*((19+y)/180);
            double targetDist = 25.25 / tan(angleRadians);
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
}
