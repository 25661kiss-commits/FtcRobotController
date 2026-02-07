package org.firstinspires.ftc.teamcode.utils;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.teamcode.mechaisms.MecanumDriveTele;

@TeleOp
@Disabled
public class MekanumDriveFeildOrentated extends OpMode {
    MecanumDriveTele drive = new MecanumDriveTele();
    double forward,strafe,rotate;
    private IMU imu;//my stuff daniel
    @Override
    public void init(){
        drive.init(hardwareMap);
        imu = drive.getImu();

    }

    @Override
    public void loop(){
        forward = -gamepad1.left_stick_y;
        strafe = gamepad1.left_stick_x;
        rotate = gamepad1.right_stick_x;
        telemetry.addData("imu yaw", imu.getRobotYawPitchRollAngles().getYaw());
        telemetry.addData("strafe",forward);
        telemetry.addData("strafe",strafe);
        telemetry.addData("rotate",rotate);
        drive.driveFeildRelative(forward,strafe,rotate);
    }
}
