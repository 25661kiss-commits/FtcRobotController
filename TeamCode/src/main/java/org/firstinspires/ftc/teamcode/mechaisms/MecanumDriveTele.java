package org.firstinspires.ftc.teamcode.mechaisms;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotor.RunMode;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.autonomous.PrintODO;

public class MecanumDriveTele {
    public DcMotor frontLeftMotor, frontRightMotor,backLeftMotor,backRightMotor;//define motors
    private IMU imu;
    //init function
    public void  init(HardwareMap HwMap){//this dose not use the limeight means that the default value value is null so it wont error if  no limelight3A objet is provided later we wil use a if(!(limelight3A == null)){statements if limelight3A exists} to detect if a limelight3A object is passed
        // import motors form the configureation

        frontLeftMotor = HwMap.get(DcMotor.class, "front_left_motor");
        frontRightMotor = HwMap.get(DcMotor.class, "front_right_motor");
        backLeftMotor = HwMap.get(DcMotor.class, "back_left_motor");
        backRightMotor = HwMap.get(DcMotor.class, "back_right_motor");

        //set motor diretion
        frontRightMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        backRightMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        frontLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        // set runmode run using encoder
        frontRightMotor.setMode(RunMode.RUN_USING_ENCODER);
        frontLeftMotor.setMode(RunMode.RUN_USING_ENCODER);
        backRightMotor.setMode(RunMode.RUN_USING_ENCODER);
        backLeftMotor.setMode(RunMode.RUN_USING_ENCODER);

        // imu initalization
        // using rev control hub internal imu

        imu = HwMap.get(IMU.class, "imu");
        // imu diretion
        RevHubOrientationOnRobot RevOrientation = new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD);

        //initalize imu
        imu.initialize(new IMU.Parameters(RevOrientation));

        //reset the yaw daniel
        imu.resetYaw();
        //limelight3A stuff

    }
    public void  init(HardwareMap HwMap, RunMode encoderMode){//this dose not use the limeight means that the default value value is null so it wont error if  no limelight3A objet is provided later we wil use a if(!(limelight3A == null)){statements if limelight3A exists} to detect if a limelight3A object is passed
        // import motors form the configureation

        frontLeftMotor = HwMap.get(DcMotor.class, "front_left_motor");
        frontRightMotor = HwMap.get(DcMotor.class, "front_right_motor");
        backLeftMotor = HwMap.get(DcMotor.class, "back_left_motor");
        backRightMotor = HwMap.get(DcMotor.class, "back_right_motor");

        //set motor diretion
        frontRightMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        backRightMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        frontLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        // set runmode run using encoder
        frontRightMotor.setMode(encoderMode);
        frontLeftMotor.setMode(encoderMode);
        backRightMotor.setMode(encoderMode);
        backLeftMotor.setMode(encoderMode);

        frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // imu initalization
        // using rev control hub internal imu

        imu = HwMap.get(IMU.class, "imu");
        // imu diretion
        RevHubOrientationOnRobot RevOrientation = new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD);

        //initalize imu
        imu.initialize(new IMU.Parameters(RevOrientation));

        //reset the yaw daniel
        imu.resetYaw();
        //limelight3A stuff

    }

    public IMU getImu(){//daniel
        return imu;//daniel
    }
    public void drive(double forward, double strafe, double rotate){
        //if(PRINTOUT != null){
          //  PRINTOUT.PRINTOUT();
        //}
        double frontLeftPower = forward + strafe + rotate;
        double backLeftPower = forward - strafe + rotate;
        double frontRightPower = forward - strafe - rotate;
        double backRightPower = forward + strafe - rotate;

        double maxPower = 1.0;
        double maxSpeed = 1.0;

        maxPower = Math.max(maxPower, Math.abs(frontLeftPower));
        maxPower = Math.max(maxPower, Math.abs(frontRightPower));
        maxPower = Math.max(maxPower, Math.abs(backLeftPower));
        maxPower = Math.max(maxPower, Math.abs(backRightPower));
//motor speeds
        frontLeftMotor.setPower(maxSpeed * (frontLeftPower / maxPower)*0.85);
        backLeftMotor.setPower(maxSpeed * (backLeftPower / maxPower)*1.1);
        frontRightMotor.setPower(maxSpeed * (frontRightPower / maxPower)*0.85);
        backRightMotor.setPower(maxSpeed * (backRightPower / maxPower)*1.1);
    }

    public void driveFeildRelative( double forward, double strafe, double rotate){
        double theta = Math.atan2(forward,strafe);
        double r = Math.hypot(strafe, forward);
        theta = AngleUnit.normalizeRadians(theta -
                imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS));

        double newForward = r * Math.sin(theta);
        double newStrafe = r * Math.cos(theta);

        this.drive(newForward, newStrafe, rotate);
    }

}
