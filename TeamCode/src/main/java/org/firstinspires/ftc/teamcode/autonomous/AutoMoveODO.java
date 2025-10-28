package org.firstinspires.ftc.teamcode.autonomous;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.linearOpMode;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.opMode;
import static java.lang.Math.abs;
import static java.lang.Thread.sleep;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.mechaisms.MekanumDrive;
import org.firstinspires.ftc.teamcode.mechaisms.gobuildaPinpoint;

public class AutoMoveODO  {
    public GoBildaPinpointDriver odo;
    public AutoRotateDegrese chassis;

    // fn - GET odometry
    public void init(GoBildaPinpointDriver _odo,AutoRotateDegrese _chassis){
        odo=_odo;
        chassis=_chassis;
        chassis.frontRightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        chassis.frontLeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        chassis.backRightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        chassis.backLeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }
    public void moveInches(double ticks, double maxSpeed, Telemetry telemetryA,PrintODO PRINTOUT){
        try {
            sleep(1000);
        } catch (InterruptedException e) {
            throw new RuntimeException(e);
        }
        odo.update();


        while(abs(odo.getPosX(DistanceUnit.INCH)) < ticks){
            PRINTOUT.PRINTOUT();
            odo.update();
            chassis.frontLeftMotor.setPower(maxSpeed);
            chassis.backLeftMotor.setPower(maxSpeed);
            chassis.frontRightMotor.setPower(maxSpeed);
            chassis.backRightMotor.setPower(maxSpeed);

        }
        chassis.backLeftMotor.setPower(0);
        chassis.backRightMotor.setPower(0);
        chassis.frontLeftMotor.setPower(0);
        chassis.frontRightMotor.setPower(0);
    }
    public void moveInchesHoris(double ticks, double maxSpeed, Telemetry telemetryA,PrintODO PRINTOUT){
        try {
            sleep(1000);
        } catch (InterruptedException e) {
            throw new RuntimeException(e);
        }
        odo.update();


        while(abs(odo.getPosY(DistanceUnit.INCH)) < ticks){
            PRINTOUT.PRINTOUT();
            odo.update();
            chassis.frontLeftMotor.setPower(maxSpeed);
            chassis.backLeftMotor.setPower(maxSpeed);
            chassis.frontRightMotor.setPower(maxSpeed);
            chassis.backRightMotor.setPower(maxSpeed);

        }
        chassis.backLeftMotor.setPower(0);
        chassis.backRightMotor.setPower(0);
        chassis.frontLeftMotor.setPower(0);
        chassis.frontRightMotor.setPower(0);
    }
}
