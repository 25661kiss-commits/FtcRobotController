package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.mechaisms.MekanumDrive;
import org.firstinspires.ftc.teamcode.mechaisms.gobuildaPinpoint;

public class AutoRotateODO  {
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
    public void rotateDegrees(double degrees, double maxSpeed, Telemetry telemetryA){
        odo.update();
        while(odo.getHeading(AngleUnit.DEGREES) < degrees){
            odo.update();
            chassis.frontLeftMotor.setPower(maxSpeed);
            chassis.backLeftMotor.setPower(maxSpeed);
            chassis.frontRightMotor.setPower(-maxSpeed);
            chassis.backRightMotor.setPower(-maxSpeed);

        }
        chassis.backLeftMotor.setPower(0);
        chassis.backRightMotor.setPower(0);
        chassis.frontLeftMotor.setPower(0);
        chassis.frontRightMotor.setPower(0);
    }
}
