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
public class AutoFrontBlueBF extends OpMode {
    @Deprecated
    final double TagDist= 13.125;
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
    @Deprecated
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
        limelight3A.pipelineSwitch(4);//1 is green
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
        ballStopLeft = hardwareMap.get(Servo.class,"ball_stop_left");
        ballStopRight = hardwareMap.get(Servo.class,"ball_stop_right");


        ballStopLeft.setDirection(Servo.Direction.REVERSE);
        ballStopRight.setDirection(Servo.Direction.FORWARD);
    }
    @Override
    public void start() {
        limelight3A.start();
    }
    private boolean done = true;
    @Override
    public void loop() {
        if(done){
            delayMs(1000);
            shooterMotor.setPower(1);
            shooterMotor2.setPower(1);
            ballStopLeft.setPosition(0.7);
            ballStopRight.setPosition(0.7);
            done = false;
            //!\\ back up on limemlight distance to shoot-------------------------------------------
            double dist = getLLDistance();
            while(dist < 45){//back up
                dist = getLLDistance();
                odo.update();
                if (odo.getHeading(AngleUnit.DEGREES) < 0) {
                    drive.drive(0.5, 0, 0.1);
                } else {
                    drive.drive(0.5, 0, -0.1);
                }

                double velocity = shooterMotor.getVelocity();
                telemetry.addData("speed:",velocity);
                if(shooterMotor.getVelocity() <= 660){
                    shooterMotor.setPower(1);
                }else{
                    shooterMotor.setPower(0.5);
                }
                if(shooterMotor2.getVelocity() <= 660){
                    shooterMotor2.setPower(1);
                }else{
                    shooterMotor2.setPower(0.5);
                }
            }
            drive.drive(0,0,0);
            boolean stooop = true;
            //!\\ aim at target---------------------------------------------------------------------
            delayMs(400);
            while(stooop){// aim
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
                if(shooterMotor.getVelocity() <= 660){
                    shooterMotor.setPower(1);
                }else{
                    shooterMotor.setPower(0.5);
                }
                if(shooterMotor2.getVelocity() <= 660){
                    shooterMotor2.setPower(1);
                }else{
                    shooterMotor2.setPower(0.5);
                }
            }
            double velocity = shooterMotor.getVelocity();
            //!\\ spool up before 1st ball----------------------------------------------------------
            while(velocity <= 660 & shooterMotor2.getVelocity() <= 660){//wait for shooters to be at speed
                velocity = shooterMotor.getVelocity();
                telemetry.addData("speed:",velocity);
                if(shooterMotor.getVelocity() <= 660){
                    shooterMotor.setPower(1);
                }else{
                    shooterMotor.setPower(0.5);
                }
                if(shooterMotor2.getVelocity() <= 660){
                    shooterMotor2.setPower(1);
                }else{
                    shooterMotor2.setPower(0.5);
                }
            }
            ElapsedTime timer = new ElapsedTime();

            timer.reset();
            //!\\ fire first ball(s)----------------------------------------------------------------
            while(timer.milliseconds() < 1500){//fire first ball(s)
                rtFire.setPower(intakeLt);
                ltFire.setPower(-intakeLt);
                if(shooterMotor.getVelocity() <= 660){
                    shooterMotor.setPower(1);
                }else{
                    shooterMotor.setPower(0.5);
                }
                if(shooterMotor2.getVelocity() <= 660){
                    shooterMotor2.setPower(1);
                }else{
                    shooterMotor2.setPower(0.5);
                }
            }
            //fire second ball
            time  = this.getRuntime();
            timer.reset();
            //!\\ spool up before 3rd ball----------------------------------------------------------
            while(velocity <= 660 & shooterMotor2.getVelocity() <= 660){//wait for shooters to be at speed
                velocity = shooterMotor.getVelocity();
                telemetry.addData("speed:",velocity);
                if(shooterMotor.getVelocity() <= 660){
                    shooterMotor.setPower(1);
                }else{
                    shooterMotor.setPower(0.5);
                }
                if(shooterMotor2.getVelocity() <= 660){
                    shooterMotor2.setPower(1);
                }else{
                    shooterMotor2.setPower(0.5);
                }
            }
            //!\\ wait to shoot 3rd ball------------------------------------------------------------
            timer.reset();
            while(timer.milliseconds() < 700){
                velocity = shooterMotor.getVelocity();
                telemetry.addData("speed:",velocity);
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
            //!\\ shoot 3rd ball--------------------------------------------------------------------
            timer.reset();
            while(timer.milliseconds() < 1000){//fire second ball(s)
                rtIntake.setPower(1);
                rtFire.setPower(intakeLt);
                ltFire.setPower(-intakeLt);
                if(shooterMotor.getVelocity() <= 660){
                    shooterMotor.setPower(1);
                }else{
                    shooterMotor.setPower(0.5);
                }
                if(shooterMotor2.getVelocity() <= 660){
                    shooterMotor2.setPower(1);
                }else{
                    shooterMotor2.setPower(0.5);
                }
            }
            //!\\ turn 45 degreees on odo-----------------------------------------------------------
            int loops = 0;
            while(loops < 4){
                if(abs(odo.getHeading(AngleUnit.DEGREES) - 45) > 2) {
                    if (odo.getHeading(AngleUnit.DEGREES) < 50) {
                        drive.drive(0, 0, 0.3);
                    } else {
                        drive.drive(0, 0, -0.3);
                    }
                    loops = 0;
                }else{
                    drive.drive(0,0,0);
                    loops++;
                }
                odo.update();
            }

            drive.drive(0,0,0);
            rtIntake.setPower(0.85);//was .75
            rtFire.setPower(-0.2);//was .3
            ballStopLeft.setPosition(0.25);
            ballStopRight.setPosition(0.25);
            ltFire.setPower(0);
            //!\\ after slide picup balls on diag -----------------------------------------------------
            drive.drive(-0.15,0,0);
            while(distb.getDistance(DistanceUnit.CM) > 37){
                if(color.getDistance(DistanceUnit.CM) <=3.4){
                    ltFire.setPower(0);
                }
                if(rtcolor.getDistance(DistanceUnit.CM) <= 3.4){
                    rtFire.setPower(0);
                }
                if (odo.getHeading(AngleUnit.DEGREES) < 45) {
                    drive.drive(-0.15, 0, 0.05);
                } else {
                    drive.drive(-0.15, 0, -0.05);
                }
                odo.update();
                //ModulateSpeed(740);
            }
            rtFire.setPower(0);//was .3
            ltFire.setPower(0);
            drive.drive(0.0,0,0);
            rtIntake.setPower(0);
            ltFire.setPower(0);
            drive.drive(0,0,0);
            rtIntake.setPower(0);
            rtFire.setPower(0);
            ltFire.setPower(0);
            odo.update();
            drive.drive(0,0,0);
            ltIntake.setPower(0);
            timer.reset();
            ltIntake.setPower(0);
            rtIntake.setPower(0);
            odo.update();
            drive.drive(0,0,0);
            ltIntake.setPower(0);
            timer.reset();
            ltIntake.setPower(0);
            rtIntake.setPower(0);
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
}
