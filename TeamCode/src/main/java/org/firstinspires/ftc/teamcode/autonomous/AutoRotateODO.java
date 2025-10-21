package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;

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
    }
    public void rotateDegrees(double degrees, double maxSpeed, Telemetry telemetryA){
        odo.update();
        while(odo.getHeading(AngleUnit.DEGREES) < degrees - 1){
            odo.update();
            chassis.drive(0,0,maxSpeed);
            telemetryA.addData("odoz:",odo.getHeading(AngleUnit.DEGREES));
        }
        chassis.backLeftMotor.setPower(0);
        chassis.backRightMotor.setPower(0);
        chassis.frontLeftMotor.setPower(0);
        chassis.frontRightMotor.setPower(0);
    }
}
