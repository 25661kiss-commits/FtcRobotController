package org.firstinspires.ftc.teamcode.mechaisms;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class gobuildaPinpoint {
    GoBildaPinpointDriver odo;

    public void init(HardwareMap HwMap){
        odo = HwMap.get(GoBildaPinpointDriver.class,"odo");

        odo.resetPosAndIMU();

    }

    public GoBildaPinpointDriver getPinpoint(){
        return odo;
    }
}
