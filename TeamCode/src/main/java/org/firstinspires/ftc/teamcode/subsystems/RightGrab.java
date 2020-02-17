package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class RightGrab {

    private boolean isClosed;
    private boolean isRetracted;


    private Servo rightBig;
    private Servo rightSmall;

    private double OPEN_POS = 0.8;// middle
    private double CLOSE_POS = 0.05;

    private double RETRACT_POS = 0.37; // middle
    private double EXTEND_POS = 0.81;


    public RightGrab(HardwareMap hardwareMap){

        rightSmall = hardwareMap.servo.get("rightSmall");
        rightBig = hardwareMap.servo.get("rightBig");

        retract();
        setSmallPosition(0.4);
        System.out.println("ur bad");
        // jk ok

    }

    public void setSmallPosition(double position){ rightSmall.setPosition(position);}

    public void setBigPosition(double position){
        rightBig.setPosition(position);
    }

    public void open(){
        rightSmall.setPosition(OPEN_POS);
    }

    public void close(){
        rightSmall.setPosition(CLOSE_POS);
    }

    public void retract(){
        rightBig.setPosition(RETRACT_POS);
    }

    public void extend(){
        rightBig.setPosition(EXTEND_POS);
    }

    public void toggleSmall(){
        if(isClosed){
            isClosed = false;
            open();
        }else {
            isClosed = true;
            close();
        }

    }
    public void toggleBig(){
        if(isRetracted){
            isRetracted = false;
            extend();
        }else {
            isRetracted = true;
            retract();
        }

    }


}
