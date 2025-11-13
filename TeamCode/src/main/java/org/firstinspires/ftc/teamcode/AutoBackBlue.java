package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.autonomous.AutoMovements;
import org.firstinspires.ftc.teamcode.autonomous.PrintODO;

@Autonomous
@Disabled
public class AutoBackBlue extends OpMode {
    private AutoMovements autoMovements = new AutoMovements();
    private PrintODO PRINTOUT = new PrintODO();
    Limelight3A limelight3A;
    @Override
    public void init() {
        limelight3A = hardwareMap.get(Limelight3A.class,"limelight");
        autoMovements.initAuto(hardwareMap,PRINTOUT,limelight3A);

    }

    @Override
    public void start() {
        limelight3A.start();

    }

    @Override
    public void loop() {

    }
}
