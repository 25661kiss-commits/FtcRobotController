package org.firstinspires.ftc.teamcode.autonomous;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;
import static java.lang.Math.abs;
import org.firstinspires.ftc.robotcore.external.Telemetry;

public class AutoRotateDegrese extends AutoMovements{
    public void rotateTagReative(double angle, double maxSpeed, Telemetry telemetryA){

        double camAngle = this.getTagAngle();//read tag do nothing with the return value we just need to set llresubb
        camAngle = 10;
        telemetryA.addData("telemetry","test");
        while(abs(camAngle) > 1) {
            camAngle = this.getTagAngle();//read tag do nothing with the return value we just need to set llresult
            this.drive(0,0,maxSpeed);
            telemetryA.addData("tx:", camAngle);
            telemetryA.addData("FR:", this.frontRightMotor.getPower());
            telemetryA.addData("FL:", this.frontLeftMotor.getPower());
            telemetryA.update();
        }
        this.backLeftMotor.setPower(0);
        this.backRightMotor.setPower(0);
        this.frontLeftMotor.setPower(0);
        this.frontRightMotor.setPower(0);
    }
}
