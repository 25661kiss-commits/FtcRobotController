package org.firstinspires.ftc.teamcode;

import static java.lang.Math.abs;
import static java.lang.Math.tan;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.teamcode.mechaisms.MecanumDriveTele;

@TeleOp
public class MecanumDriveColinOrientated extends OpMode {
    MecanumDriveTele drive = new MecanumDriveTele();
    private Limelight3A limelight3A;
    double forward,strafe,rotate;
    private final double targetSpeedHigh = 1;
    private final double targetSpeedMed = 0.4;
    private final double targetSpeedLow = 0.2;
    private IMU imu;//my stuff daniel
    private boolean fire = false;

    @Override
    public void init(){
        drive.init(hardwareMap, DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        imu = drive.getImu();
        limelight3A = hardwareMap.get(Limelight3A.class,"limelight");
        limelight3A.pipelineSwitch(5);//1 is green

    }

    @Override
    public void start() {
        limelight3A.start();
    }

    @Override
    public void loop(){
        //movement
        forward = gamepad1.left_stick_y;
        strafe = -gamepad1.left_stick_x;
        rotate = gamepad1.right_stick_x;
        drive.drive(forward,strafe,rotate);
        if(gamepad1.a) {
            double speed = getLLRotationOffset();
        }
        //line up with target
        if(gamepad1.left_bumper | gamepad1.right_bumper) {
            fire = true;
            //turn left to limelight tag...
            double rotation = getLLRotationOffset();
            double targetSpeed;
            if (rotation > 0 & rotation != -1) {
                //positive stuff
                while (rotation == -1 || rotation > 1) {

                    rotation = getLLRotationOffset();
                    if (rotation == -1) {
                        drive.drive(0, 0, -targetSpeedHigh);
                    } else {
                        if(abs(rotation) < 10){
                            targetSpeed = targetSpeedLow;
                        }else{
                            targetSpeed = targetSpeedMed;
                        }
                        drive.drive(0, 0, -targetSpeed);
                    }

                    telemetry.update();
                    telemetry.clear();
                }
                //drive.drive(0, 0, 0);
                while (rotation < 0) {

                    rotation = getLLRotationOffset();
                    drive.drive(0, 0, 0.2);

                    telemetry.update();
                    telemetry.clear();
                }
            } else if (rotation < 0 & rotation != -1) {
                //negative stuff
                while (rotation == -1 || rotation < -1) {

                    rotation = getLLRotationOffset();
                    if (rotation == -1) {
                        drive.drive(0, 0, targetSpeedHigh);
                    } else {
                        if(abs(rotation) < 10){
                            targetSpeed = targetSpeedLow;
                        }else{
                            targetSpeed = targetSpeedMed;
                        }
                        drive.drive(0, 0, targetSpeed);
                    }


                    telemetry.update();
                    telemetry.clear();
                }
                //drive.drive(0, 0, 0);
                while (rotation > 0) {

                    rotation = getLLRotationOffset();
                    drive.drive(0, 0, -0.2);

                    telemetry.update();
                    telemetry.clear();
                }
            } else if (rotation == -1) {
                //invalid stuff
                if (gamepad1.left_bumper) {
                    while (rotation == -1 || rotation < -1) {

                        rotation = getLLRotationOffset();
                        if (rotation == -1) {
                            drive.drive(0, 0, targetSpeedHigh);
                        } else {
                            if(abs(rotation) < 10){
                                targetSpeed = targetSpeedLow;
                            }else{
                                targetSpeed = targetSpeedMed;
                            }
                            drive.drive(0, 0, targetSpeed);
                        }


                        telemetry.update();
                        telemetry.clear();
                    }
                    //drive.drive(0, 0, 0);
                    while (rotation > 0) {

                        rotation = getLLRotationOffset();
                        drive.drive(0, 0, -0.2);

                        telemetry.update();
                        telemetry.clear();
                    }
                } else if(gamepad1.right_bumper){

                    while (rotation == -1 || rotation > 1) {

                        rotation = getLLRotationOffset();
                        if (rotation == -1) {
                            drive.drive(0, 0, -0.6);
                        } else {
                            if(abs(rotation) < 10){
                                targetSpeed = targetSpeedLow;
                            }else{
                                targetSpeed = targetSpeedMed;
                            }
                            drive.drive(0, 0, -targetSpeed);

                        }

                        telemetry.update();
                        telemetry.clear();
                    }
                    //drive.drive(0, 0, 0);
                    while (rotation < 0) {

                        rotation = getLLRotationOffset();
                        drive.drive(0, 0, 0.2);

                        telemetry.update();
                        telemetry.clear();
                    }
                }
            } else {
                //handle exception this really shoulden't happen
            }






            delayMs(300);






            //second time
            rotation = getLLRotationOffset();
            if (rotation > 0 & rotation != -1) {
                //positive stuff
                while (rotation == -1 || rotation > 1) {

                    rotation = getLLRotationOffset();
                    if (rotation == -1) {
                        drive.drive(0, 0, -targetSpeedHigh);
                    } else {
                        if(abs(rotation) < 10){
                            targetSpeed = targetSpeedLow;
                        }else{
                            targetSpeed = targetSpeedMed;
                        }
                        drive.drive(0, 0, -targetSpeed);
                    }

                    telemetry.update();
                    telemetry.clear();
                }
                //drive.drive(0, 0, 0);
                while (rotation < 0) {

                    rotation = getLLRotationOffset();
                    drive.drive(0, 0, 0.2);

                    telemetry.update();
                    telemetry.clear();
                }
            } else if (rotation < 0 & rotation != -1) {
                //negative stuff
                while (rotation == -1 || rotation < -1) {

                    rotation = getLLRotationOffset();
                    if (rotation == -1) {
                        drive.drive(0, 0, targetSpeedHigh);
                    } else {
                        if(abs(rotation) < 10){
                            targetSpeed = targetSpeedLow;
                        }else{
                            targetSpeed = targetSpeedMed;
                        }
                        drive.drive(0, 0, targetSpeed);
                    }


                    telemetry.update();
                    telemetry.clear();
                }
                //drive.drive(0, 0, 0);
                while (rotation > 0) {

                    rotation = getLLRotationOffset();
                    drive.drive(0, 0, -0.2);

                    telemetry.update();
                    telemetry.clear();
                }
            } else if (rotation == -1) {
                //invalid stuff
                if (gamepad1.left_bumper) {
                    while (rotation == -1 || rotation < -1) {

                        rotation = getLLRotationOffset();
                        if (rotation == -1) {
                            drive.drive(0, 0, targetSpeedHigh);
                        } else {
                            if(abs(rotation) < 10){
                                targetSpeed = targetSpeedLow;
                            }else{
                                targetSpeed = targetSpeedMed;
                            }
                            drive.drive(0, 0, targetSpeed);
                        }


                        telemetry.update();
                        telemetry.clear();
                    }
                    //drive.drive(0, 0, 0);
                    while (rotation > 0) {

                        rotation = getLLRotationOffset();
                        drive.drive(0, 0, -0.2);

                        telemetry.update();
                        telemetry.clear();
                    }
                } else if(gamepad1.right_bumper){

                    while (rotation == -1 || rotation > 1) {

                        rotation = getLLRotationOffset();
                        if (rotation == -1) {
                            drive.drive(0, 0, -0.6);
                        } else {
                            if(abs(rotation) < 10){
                                targetSpeed = targetSpeedLow;
                            }else{
                                targetSpeed = targetSpeedMed;
                            }
                            drive.drive(0, 0, -targetSpeed);

                        }

                        telemetry.update();
                        telemetry.clear();
                    }
                    //drive.drive(0, 0, 0);
                    while (rotation < 0) {

                        rotation = getLLRotationOffset();
                        drive.drive(0, 0, 0.2);

                        telemetry.update();
                        telemetry.clear();
                    }
                }
            } else {
                //handle exception this really shoulden't happen
            }
            double dist = getLLDistance();
            if(fire){

            }

        }
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
