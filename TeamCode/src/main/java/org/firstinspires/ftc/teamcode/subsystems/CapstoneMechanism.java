package org.firstinspires.ftc.teamcode.subsystems;

import android.graphics.Paint;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class CapstoneMechanism {

    private boolean isExtended = false;

    public static double EXTEND_POS = 1;
    public static double RETRACT_POS = 0;

    private Servo servo;

    public CapstoneMechanism(HardwareMap hardwareMap){
        servo = hardwareMap.servo.get("capstone");
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
