package org.firstinspires.ftc.teamcode.utils;

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
public class LimeLightTestX extends OpMode {
    final double TagDist= 22.5;
    MecanumDriveTele drive = new MecanumDriveTele();//drive
    private Limelight3A limelight3A;//limelight obj
    double forward,strafe,rotate;// contol values
    private final double targetSpeedHigh = 0.4;// high target speed
    private final double targetSpeedMed = 0.2;//med turn speed
    private final double targetSpeedLow = 0.1;//slow turning speed
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
    private int idleSpeed = 740;


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
        limelight3A.pipelineSwitch(0);//1 is green
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
        getLLDistance();


    }
    @Override
    public void stop() {
        // This runs when the match is over
        limelight3A.stop();
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
