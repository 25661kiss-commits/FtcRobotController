package org.firstinspires.ftc.teamcode.autonomous;

import static java.lang.Double.max;
import static java.lang.Double.min;
import static java.lang.Math.tan;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.mechaisms.MekanumDrive;

public class AutoMovements extends MekanumDrive {
    private Limelight3A limelight;
    public LLResult llResult;
    public void initAuto(HardwareMap HwMap, Limelight3A _limelight){
        this.init(HwMap);//init the drive

        limelight = _limelight;
        limelight.pipelineSwitch(5);//1 is green
        this.frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        this.frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        this.backRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        this.backLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

    }
    public double getTagDist(){
        llResult = null;
        llResult = limelight.getLatestResult();
        if (llResult != null & llResult.isValid()) {

            double y = llResult.getTy();
            double angleRadians = 3.14*((19+y)/180);
            double targetDist = 25.25 / tan(angleRadians);
            return targetDist;
        }else{
            return -1;
        }
    }
    public double getTagAngle(){
        llResult = null;
        llResult = limelight.getLatestResult();
        if (llResult != null & llResult.isValid()) {

            double x = llResult.getTx();
            return x;
        }else{
            return -1;
        }
    }
    public void moveTagRealative(double distance,double MoE,double maxSpeed,boolean dirCorr){
        double dist = this.getTagDist();
        while(dist > distance){//while it is not close eneough MoE = margin of error
            dist = this.getTagDist();
            if(llResult != null & llResult.isValid()){//direction correcting
                this.drive(max(min(maxSpeed,(dist - distance)),maxSpeed * -1)/*decelerate the last 4 inches constraining between maximum speed*/,0/*no strafeing*/, 0/*IMPLEMENT direction correction */);
            }//TODO implement direction correction

        }
        this.backLeftMotor.setPower(0);
        this.backRightMotor.setPower(0);
        this.frontLeftMotor.setPower(0);
        this.frontRightMotor.setPower(0);
    }

}
