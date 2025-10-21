package org.firstinspires.ftc.teamcode;

import static java.lang.Thread.sleep;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.autonomous.AutoMoveODO;
import org.firstinspires.ftc.teamcode.autonomous.AutoMovements;
import org.firstinspires.ftc.teamcode.autonomous.AutoRotateDegrese;
import org.firstinspires.ftc.teamcode.autonomous.AutoRotateODO;
import org.firstinspires.ftc.teamcode.mechaisms.gobuildaPinpoint;

@Autonomous
public class AutoFrontBlue extends OpMode {
    private AutoRotateDegrese autoMovements = new AutoRotateDegrese();
    Limelight3A limelight3A;
    GoBildaPinpointDriver odo;
    private gobuildaPinpoint pinpoint = new gobuildaPinpoint();
    private AutoRotateODO ODORot = new AutoRotateODO();
    private AutoMoveODO ODOMove = new AutoMoveODO();
    @Override
    public void init() {
        limelight3A = hardwareMap.get(Limelight3A.class,"limelight");
        autoMovements.initAuto(hardwareMap,limelight3A);
        pinpoint.init(hardwareMap);
        odo = pinpoint.getPinpoint();
        ODORot.init(odo, autoMovements);
        ODOMove.init(odo,autoMovements);
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
            autoMovements.rotateTagReative(0,0.5,telemetry);
            autoMovements.moveTagRealative(100,0,1,false);
            autoMovements.moveTagRealative(90,0,1,false);
            ODORot.rotateDegrees(90,1,telemetry);
            ODOMove.moveTicks(128,1,telemetry);
            //128.5 ticks
        }
    }
}
