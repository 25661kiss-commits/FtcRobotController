package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp
@Disabled
public class VariableExample extends OpMode {

    public void init(){

        int teamnumber = 25661;
        double hieght = 18.25;
    boolean claw = true;
    String name = "Kiss";
telemetry.addData("team number",teamnumber);
        telemetry.addData("hieght",hieght);
        telemetry.addData("claw",claw);
        telemetry.addData("name",name);
    }
public void loop(){


}


}
