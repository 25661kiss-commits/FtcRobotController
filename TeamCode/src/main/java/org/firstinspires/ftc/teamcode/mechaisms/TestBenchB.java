package org.firstinspires.ftc.teamcode.mechaisms;

import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class TestBenchB {
    private DigitalChannel touchSensor;
    public void init(HardwareMap hwMap){
        touchSensor = hwMap.get(DigitalChannel.class,"touch_sensor");
        touchSensor.setMode(DigitalChannel.Mode.INPUT);
    }
    public boolean TouchSensorPressed(){
        return !touchSensor.getState();
    }
}
