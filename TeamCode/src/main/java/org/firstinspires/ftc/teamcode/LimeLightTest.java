package org.firstinspires.ftc.teamcode;

import static java.lang.Math.tan;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareDevice;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.mechaisms.gobuildaPinpoint;

@TeleOp
public class LimeLightTest extends OpMode {
    private double TPMM = 68.440839782642795137756153472092;//ticks per inch
    private Limelight3A limelight3A;
    gobuildaPinpoint pinpoint = new gobuildaPinpoint();
    GoBildaPinpointDriver odo;

    @Override
    public void init () {
        limelight3A = hardwareMap.get(Limelight3A.class,"limelight");
        limelight3A.pipelineSwitch(5);//1 is green
        pinpoint.init(hardwareMap);
        odo = pinpoint.getPinpoint();
        odo.setEncoderResolution(TPMM,DistanceUnit.MM);
        odo.setOffsets(-3.5,-4.5,DistanceUnit.INCH);
        odo.resetPosAndIMU();
    }
    @Override
    public void start() {
        limelight3A.start();
    }

    @Override
    public void loop() {
    LLResult llResult = limelight3A.getLatestResult();
    if (llResult != null & llResult.isValid()) {
        telemetry.addData("target X offset", llResult.getTx());
        telemetry.addData("Target y offset", llResult.getTy());
        telemetry.addData("Target area offset", llResult.getTa());
        double y = llResult.getTy();
        double angleRadians = 3.14*((19+y)/180);
        double targetDist = 25.25 / tan(angleRadians);
        telemetry.addData("distance:",targetDist);
    }
    odo.update();
    telemetry.addData("pinpointX",odo.getPosX(DistanceUnit.INCH));
    telemetry.addData("pinpointY",odo.getPosY(DistanceUnit.INCH));
    telemetry.addData("pinpointZ",odo.getHeading(AngleUnit.DEGREES));






    }
}
