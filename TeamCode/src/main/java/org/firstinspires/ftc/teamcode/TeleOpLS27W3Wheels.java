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



/*
* reffrence
* TARGET - aiming
*
*
*
*
* */


@TeleOp
public class TeleOpLS27W3Wheels extends OpMode {
    MecanumDriveTele drive = new MecanumDriveTele();//drive
    private Limelight3A limelight3A;//limelight obj
    double forward,strafe,rotate;// contol values
    private final double targetSpeedHigh = 0.7;// high target speed
    private final double targetSpeedMed = 0.4;//med turn speed
    private final double targetSpeedLow = 0.2;//slow turning speed
    private DcMotorEx shooterMotor;//left shooter motor
    private DcMotorEx shooterMotor2;//right shooter motor
    private DcMotor rtIntake;//intake
    private CRServo rtFire;//right fire servo
    private CRServo ltFire;//left fire servo
    private RevColorSensorV3 color;//color sensor rear left
    private RevColorSensorV3 color2;//color sensor front left
    private RevColorSensorV3 rtcolor;//color sensor rear left
    private RevColorSensorV3 rtcolor2;//color sensor front left
    private IMU imu;//imu
    private double setSpeed = 0;//motor target speedd
    private DigitalChannel led0;//the leds on the back (red)
    private DigitalChannel led1;
    private DigitalChannel led2;
    private DigitalChannel led3;


    @Override
    public void init(){
        color = hardwareMap.get(RevColorSensorV3.class,"color_sensor_left_front");
        color2 = hardwareMap.get(RevColorSensorV3.class,"color_sensor_left_front");
        rtcolor = hardwareMap.get(RevColorSensorV3.class,"color_sensor_right_front");
        rtcolor2 = hardwareMap.get(RevColorSensorV3.class,"color_sensor_right_front");//compile issue
        drive.init(hardwareMap, DcMotor.RunMode.RUN_USING_ENCODER);
        imu = drive.getImu();

        rtIntake = hardwareMap.get(DcMotor.class,"right_intake_motor");
        shooterMotor = hardwareMap.get(DcMotorEx.class,"shooter_motor");
        shooterMotor2 = hardwareMap.get(DcMotorEx.class,"shooter2");
        shooterMotor2.setDirection(DcMotorSimple.Direction.REVERSE);
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
        if(gamepad1.right_trigger > 0.2 || gamepad1.left_trigger > 0.2){
            forward = forward*0.25;
            strafe = strafe*0.25;
            rotate = rotate*0.25;
        }
        getLLRotationOffset();
        double rot = 0;
        LLResult llResult = limelight3A.getLatestResult();
        if(llResult != null & llResult.isValid()){
            rot = llResult.getTx();
        }else{
            rot = -1;
        }
        //line up with target
        if(gamepad1.right_bumper) { //TARGET
            //fire line up



            if(rot == -1){//cant see the tag or other problem

                    drive.drive(0,0,-targetSpeedHigh);

            }else{//tag visible
                if (rot >= 20){drive.drive(0,strafe,-targetSpeedMed);}
                else if(rot < 20 & rot > 2){ drive.drive(0,strafe,-targetSpeedLow);}
                else if(rot <=2  & rot >= -2){ drive.drive(0,strafe,0);}
                else if(rot > -20 & rot < -2){ drive.drive(0,strafe,targetSpeedLow);}
                //else if(rot <= -20){drive.drive(0,0,-targetSpeedMed);}
                else{drive.drive(0,strafe,0);}//catchall
            }
        }else if(gamepad1.left_bumper){
            //fire line up

            if(rot == -1){//cant see the tag or other problem

                drive.drive(0,0,targetSpeedHigh);

            }else{//tag visible
                //if (rot >= 20){drive.drive(0,0,-targetSpeedMed);}
                /*else*/ if(rot <= -20){drive.drive(0,strafe,targetSpeedMed);}
                else if(rot > -20 & rot < -2){ drive.drive(0,strafe,targetSpeedLow);}
                else if(rot <=2  & rot >= -2){ drive.drive(0,strafe,0);}
                else if(rot < 20 & rot > 2){ drive.drive(0,strafe,-targetSpeedLow);}//you can strafe arround the target
                else{drive.drive(0,strafe,0);}//catchall
            }
        }else{
            drive.drive(forward,strafe,rotate);
        }
        double distance = getLLDistance();
        int TargetVelocity;
        if(distance > 80){
            TargetVelocity = (int) (800 + ((distance-82.5)*6.3));//was 860
        }else if(distance < 90 & distance > 30){
            TargetVelocity = (int) (660 + ((distance - 34)*(40/11))); //set the intermediate power orignal 580 dist 70
        }else if(distance < 36){
            TargetVelocity = 700;//original 580
        }else{
            TargetVelocity = 740;//original 630
        }
        double ltdef = 0;
        double rtdef = 0;
        double velocity = shooterMotor.getVelocity();
        if(gamepad2.left_stick_y > 0.5 || gamepad2.right_stick_y > 0.5 || gamepad2.left_bumper || gamepad2.right_bumper){// joysticks move intake
            rtIntake.setPower(-1);
            if(color.getDistance(DistanceUnit.CM) > 3.6){
                ltdef= 0.25;//was .25
            }else{
                ltdef = 0;
            }
            if(rtcolor.getDistance(DistanceUnit.CM) > 3.6){
                rtdef = 0.25;//was.25

            }else{
                rtdef = 0;
            }

        }else if(gamepad2.a){
            rtIntake.setPower(1);
        }else{
            rtIntake.setPower(0);
        }
        if(rot < 2 && rot > -2 && (!(rot == -1))){//dont shoot unless within zone
            if(gamepad2.left_bumper && shooterMotor.getVelocity() > (TargetVelocity - 30)){
                ltFire.setPower(1);
            }else{
                ltFire.setPower(ltdef);
            }
            if(gamepad2.right_bumper && shooterMotor2.getVelocity() > (TargetVelocity - 30)){
                rtFire.setPower(-1);
            }else{
                rtFire.setPower(-rtdef);
            }
        }else{
            ltFire.setPower(ltdef);
            rtFire.setPower(-rtdef);
        }


        if(velocity < TargetVelocity){//speed up /!\ shooter speed adjustments /!\ SHOOT
            shooterMotor.setPower(1);
            //led2.setState(true);//off leds
            led3.setState(true);
        }else {//fast eneough
            shooterMotor.setPower(0.5);
            //led2.setState(false);
            led3.setState(false);
        }
        if(shooterMotor2.getVelocity() < TargetVelocity){//speed up /!\ shooter speed adjustments /!\ SHOOT
            shooterMotor2.setPower(1);
            led0.setState(true);
            //led1.setState(true);

        }else {//fast eneough
            shooterMotor2.setPower(0.5);
            led0.setState(false);
            //led1.setState(false);

        }
        if(color.getDistance(DistanceUnit.CM) > 3.6){
            led1.setState(true);
        }else{
            led1.setState(false);
        }
        if(rtcolor.getDistance(DistanceUnit.CM) > 3.4){
            led2.setState(true);
        }else{
            led2.setState(false);
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
