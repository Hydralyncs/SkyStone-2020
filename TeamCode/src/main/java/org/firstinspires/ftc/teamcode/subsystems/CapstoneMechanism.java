package org.firstinspires.ftc.teamcode.subsystems;

import android.graphics.Paint;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

@Config
public class CapstoneMechanism {

    public boolean isExtended = false;

    public static double EXTEND_POS = 0.63;
    public static double RETRACT_POS = 0.15;

    private Servo servo;

    public CapstoneMechanism(HardwareMap hardwareMap){
        servo = hardwareMap.servo.get("capstone");
        retract();
    }

    public void extend(){
        isExtended = true;
        servo.setPosition(EXTEND_POS);
    }

    public void retract(){
        isExtended = false;
        servo.setPosition(RETRACT_POS);
    }

    public void toggle(){
        if(isExtended){
            retract();
        } else {
            extend();
        }
    }

}
