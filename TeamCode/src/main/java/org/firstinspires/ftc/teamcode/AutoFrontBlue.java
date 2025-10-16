package org.firstinspires.ftc.teamcode;

import static java.lang.Thread.sleep;

import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.autonomous.AutoMovements;

@Autonomous
public class AutoFrontBlue extends OpMode {
    private AutoMovements autoMovements = new AutoMovements();

    Limelight3A limelight3A;
    @Override
    public void init() {
        limelight3A = hardwareMap.get(Limelight3A.class,"limelight");
        autoMovements.initAuto(hardwareMap,limelight3A);

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
            autoMovements.moveTagRealative(100,0,1,false);
            autoMovements.moveTagRealative(90,0,0.2,false);

        }
    }
}
