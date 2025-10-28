package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class PrintODO {
    private GoBildaPinpointDriver odo;
    private Telemetry telemetryA;
    public void init(GoBildaPinpointDriver _odo,Telemetry _telemetryA){
        odo = _odo;
        telemetryA = _telemetryA;
    }
    public void PRINTOUT(){
        odo.update();
        telemetryA.clear();
        telemetryA.addData("odoz",odo.getHeading(AngleUnit.DEGREES));
        telemetryA.addData("odoy",odo.getPosY(DistanceUnit.INCH));
        telemetryA.addData("odox",odo.getPosX(DistanceUnit.INCH));
        telemetryA.update();
    }
}
