package org.firstinspires.ftc.teamcode;

import static java.lang.Thread.sleep;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.autonomous.AutoMoveODO;
import org.firstinspires.ftc.teamcode.autonomous.AutoMovements;
import org.firstinspires.ftc.teamcode.autonomous.AutoRotateDegrese;
import org.firstinspires.ftc.teamcode.autonomous.AutoRotateODO;
import org.firstinspires.ftc.teamcode.autonomous.PrintODO;
import org.firstinspires.ftc.teamcode.mechaisms.gobuildaPinpoint;

@Autonomous
public class AutoFrontBlue extends OpMode {
    private double TPMM = 68.440839782642795137756153472092;//ticks per inch
    private AutoRotateDegrese autoMovements = new AutoRotateDegrese();
    Limelight3A limelight3A;
    GoBildaPinpointDriver odo;
    private gobuildaPinpoint pinpoint = new gobuildaPinpoint();
    private AutoRotateODO ODORot = new AutoRotateODO();
    private PrintODO PRINTOUT = new PrintODO();
    private AutoMoveODO ODOMove = new AutoMoveODO();
    private IMU imu;
    @Override
    public void init() {
        limelight3A = hardwareMap.get(Limelight3A.class,"limelight");
        limelight3A.pipelineSwitch(5);//1 is green
        autoMovements.initAuto(hardwareMap,PRINTOUT,limelight3A);
        pinpoint.init(hardwareMap);
        odo = pinpoint.getPinpoint();
        PRINTOUT.init(odo,telemetry);
        odo.setEncoderResolution(TPMM,DistanceUnit.MM);
        odo.setOffsets(-3.5,-4.5,DistanceUnit.INCH);
        odo.resetPosAndIMU();

        ODORot.init(odo, autoMovements);
        ODOMove.init(odo,autoMovements);
        imu = autoMovements.getImu();
    }

    @Override
    public void start() {
        limelight3A.start();

    }
    private boolean done = false;
    @Override
    public void loop() {
        if(!done){//do this only once
            done = true;
            ODOMove.moveInches(3,0.5,telemetry,PRINTOUT);
            autoMovements.rotateTagReative(0,0.5,telemetry);
            autoMovements.moveTagRealative(85,0,0.5,false);
            //autoMovements.moveTagRealative(90,0,1,false);
            ODORot.rotateDegrees(82,0.5,telemetry);
            ODOMove.moveInchesHoris(18,0.5,telemetry,PRINTOUT);

            //128.5 ticks
        }
    }
}
