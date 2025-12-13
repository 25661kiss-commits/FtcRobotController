package org.firstinspires.ftc.teamcode;

import static java.lang.Math.tan;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.mechaisms.gobuildaPinpoint;

@TeleOp
public class LimeLightTestTags extends OpMode {
    private double TPMM = 68.440839782642795137756153472092;//ticks per inch
    private Limelight3A limelight3A;
    gobuildaPinpoint pinpoint = new gobuildaPinpoint();
    GoBildaPinpointDriver odo;

    @Override
    public void init () {
        limelight3A = hardwareMap.get(Limelight3A.class,"limelight");
        limelight3A.pipelineSwitch(0);//1 is green

    }
    @Override
    public void start() {
        limelight3A.start();
    }

    @Override
    public void loop() {
        limelight3A.pipelineSwitch(0);//1 is green
        LLResult lla = limelight3A.getLatestResult();
        while(!(lla.getPipelineIndex() == 0)){
            lla = limelight3A.getLatestResult();
        }
        LLResult llResult = limelight3A.getLatestResult();
        int Obolisk = 0;
        if (llResult != null & llResult.isValid()) {

            Obolisk=21;
        }else{
            limelight3A.pipelineSwitch(1);//1 is green
            lla = limelight3A.getLatestResult();
            while(!(lla.getPipelineIndex() == 1)){
                lla = limelight3A.getLatestResult();
            }
            LLResult llResult2 = limelight3A.getLatestResult();
            if (llResult2 != null & llResult2.isValid()) {

                Obolisk=22;
            }else{
                limelight3A.pipelineSwitch(2);//1 is green
                lla = limelight3A.getLatestResult();
                while(!(lla.getPipelineIndex() == 2)){
                    lla = limelight3A.getLatestResult();
                }
                LLResult llResult3 = limelight3A.getLatestResult();
                if (llResult3 != null & llResult3.isValid()) {

                    Obolisk=23;
                }else{
                    Obolisk=0;
                }
            }
        }

        llResult = null;
        telemetry.addData("tag",Obolisk);
        telemetry.update();
        telemetry.clear();
        Obolisk=0;







    }
}
