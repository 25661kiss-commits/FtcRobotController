package org.firstinspires.ftc.teamcode.autonomous;

import static java.lang.Math.abs;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;

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
    }
    public void moveTicks(double ticks, double maxSpeed, Telemetry telemetryA){
        odo.update();
        double startState = abs(odo.getPosY(DistanceUnit.CM));
        while(abs(odo.getPosY(DistanceUnit.CM)) - startState < abs(ticks)){
            odo.update();
            chassis.drive(maxSpeed,0,0);
            telemetryA.addData("odoz:",odo.getPosY(DistanceUnit.CM) + startState);
        }
        chassis.backLeftMotor.setPower(0);
        chassis.backRightMotor.setPower(0);
        chassis.frontLeftMotor.setPower(0);
        chassis.frontRightMotor.setPower(0);
    }
}
