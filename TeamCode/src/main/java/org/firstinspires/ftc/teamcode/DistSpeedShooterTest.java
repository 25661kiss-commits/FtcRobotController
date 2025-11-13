package org.firstinspires.ftc.teamcode;

import static java.lang.Math.tan;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@TeleOp
@Disabled
public class DistSpeedShooterTest extends OpMode {
    private Limelight3A limelight3A;
    private DcMotorEx fireMotor;
    @Override
    public void init() {
        limelight3A = hardwareMap.get(Limelight3A.class,"limelight");
        limelight3A.pipelineSwitch(5);//1 is green
        fireMotor = hardwareMap.get(DcMotorEx.class,"shooter_motor");
    }
    @Override
    public void start() {
        limelight3A.start();
    }
    private double idleSpeed = 0;
    private double setSpeed = 0;
    @Override
    public void loop() {

        setSpeed = idleSpeed;
        if(gamepad1.right_trigger > 0.2){//trigger pressed!!!
            double distance = getLLDistance();
            if(distance < 30){
                //nothing
                setSpeed = idleSpeed;
            }else if(distance <= 35){
                setSpeed = 0.5;
            }else if(distance <= 48){
                setSpeed = 0.625;
            }else if(distance <= 85){
                setSpeed = 0.75;
            }else if(distance > 85){
                setSpeed = 0.85;
            }

        }else{
            setSpeed = idleSpeed;
        }
        if(gamepad1.a){
            idleSpeed = 0.5;
        }
        telemetry.addData("speed:",fireMotor.getVelocity());
        telemetry.addData("set speed:",setSpeed);
        fireMotor.setPower(setSpeed);
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
}
