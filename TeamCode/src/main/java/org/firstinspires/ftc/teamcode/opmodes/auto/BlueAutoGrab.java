package org.firstinspires.ftc.teamcode.opmodes.auto;


import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.path.heading.ConstantInterpolator;
import com.acmerobotics.roadrunner.path.heading.LinearInterpolator;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.Const;
import org.firstinspires.ftc.teamcode.roadrunner.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.subsystems.Detector;
import org.firstinspires.ftc.teamcode.subsystems.DriveTrain;
import org.firstinspires.ftc.teamcode.subsystems.Hook;
import org.firstinspires.ftc.teamcode.subsystems.LeftGrab;
import org.firstinspires.ftc.teamcode.subsystems.RightGrab;
import org.yaml.snakeyaml.scanner.Constant;

import java.util.Vector;

import kotlin.Unit;

@Autonomous()
public class BlueAutoGrab extends LinearOpMode {


    private DriveTrain drive;
    private RightGrab rightGrab;
    private Detector detector;
    private LeftGrab leftGrab;
    private Hook hook;

    private int stonePosition = 0;


    public void runOpMode() throws InterruptedException {


        drive = new DriveTrain(hardwareMap);
        detector = new Detector(hardwareMap);
        //rightGrab = new RightGrab(hardwareMap);
        rightGrab = new RightGrab(hardwareMap);
        leftGrab = new LeftGrab(hardwareMap);
        hook = new Hook(hardwareMap);

        hook.close();

        leftGrab.retract();
        leftGrab.open();

        rightGrab.retract();
        rightGrab.open();

        drive.setPoseEstimate(new Pose2d(-39, 62, Math.toRadians(-90)));
        //rightGrab.open();

        detector.startStreaming();
        while (!isStarted() && !isStopRequested()) {
            double position = detector.detector.foundRectangle().x + detector.detector.foundRectangle().width / 2;
            if (position < 70) {
                stonePosition = 1;
            } else if (position < 125) {
                stonePosition = 2;
            } else {
                stonePosition = 0;
            }
            telemetry.addData("Stone Position: ", stonePosition);
            telemetry.addData("Position: ", position);
            telemetry.update();
        }
        detector.phoneCam.stopStreaming();

        double offset = 0;
        if (stonePosition == 1) {
            offset = 0.5;
        }

        if (stonePosition == 2) {
            offset = 0.5;
        }

        drive.followTrajectory(drive.trajectoryBuilder()
                .splineTo(new Pose2d(-53,36,Math.toRadians(180)))
                .build());
        update();

        drive.followTrajectory(drive.trajectoryBuilder()
                .addMarker(0.2,()->{
                    rightGrab.setBigPosition(0.7);
                    return Unit.INSTANCE;
                })
                .lineTo(new Vector2d(-44-(stonePosition*8)-offset,31),new ConstantInterpolator(Math.PI))
                .build());
        update();

        drive.followTrajectory(drive.trajectoryBuilder()
                .addMarker(()->{
                    rightGrab.extend();
                    return Unit.INSTANCE;
                })
                .lineTo(new Vector2d(-44-(stonePosition*8),31),new ConstantInterpolator(Math.PI))
                .build());
        update();

        sleep(100);
        rightGrab.close();
        sleep(400);
        rightGrab.setBigPosition(0.55);
        sleep(100);



        drive.followTrajectory(drive.trajectoryBuilder()
                .reverse()
                .splineTo(new Pose2d(-12,40,Math.toRadians(180)),new ConstantInterpolator(Math.PI))
                .lineTo(new Vector2d(12,40))
                .splineTo(new Pose2d(49,30.5,Math.toRadians(180)), new ConstantInterpolator(Math.PI))
                .build());

        update();


        rightGrab.setBigPosition(0.6);
        sleep(100);
        rightGrab.open();
        sleep(100);
        rightGrab.retract();
        sleep(100);





        double yOffset = 0;
        if(stonePosition == 1){
            yOffset = 0;
        }
        if(stonePosition == 2){
            yOffset = -0.4;
        }

        double xOffset = 0;
        if(stonePosition == 2){
            xOffset = 0;
        }
        if(stonePosition == 1){
            xOffset = 0;
        }

        drive.followTrajectory(drive.trajectoryBuilder()
                .lineTo(new Vector2d(-5,36),new ConstantInterpolator(Math.PI))
                .build());
        update();

        drive.followTrajectory(drive.trajectoryBuilder()
                .addMarker(0.2,()->{
                    rightGrab.setBigPosition(0.7);
                    return Unit.INSTANCE;
                })
                .lineTo(new Vector2d(-19-stonePosition*8,30),new ConstantInterpolator(Math.PI))
                .build());
        update();

        drive.followTrajectory(drive.trajectoryBuilder()
                .addMarker(()->{
                    rightGrab.extend();
                    return Unit.INSTANCE;
                })
                .lineTo(new Vector2d(-19-stonePosition*8,30),new ConstantInterpolator(Math.PI))
                .build());
        update();

        sleep(100);
        rightGrab.close();
        sleep(400);
        rightGrab.setBigPosition(0.55);
        sleep(100);



        drive.followTrajectory(drive.trajectoryBuilder()
                .reverse()
                .splineTo(new Pose2d(-8,38,Math.PI),new ConstantInterpolator(Math.PI))
                .lineTo(new Vector2d(12,38),new ConstantInterpolator(Math.PI))
                .splineTo(new Pose2d(58,30
                        ,Math.PI),new ConstantInterpolator(Math.PI))
                .build());
        update();

        rightGrab.setBigPosition(0.6);
        sleep(100);
        rightGrab.open();
        sleep(100);
        rightGrab.retract();
        sleep(100);

        drive.followTrajectory(drive.trajectoryBuilder()
                .lineTo(new Vector2d(62,36),new LinearInterpolator(Math.PI,-Math.PI/2))
                .lineTo(new Vector2d(62,28),new ConstantInterpolator(Math.PI/2))
                .build());
        update();

        hook.open();
        sleep(300);

        drive.followTrajectory(drive.trajectoryBuilder()
                .lineTo(new Vector2d(40,64),new LinearInterpolator(Math.PI/2,Math.PI/1.5))
                .build());
        update();
        hook.close();
        sleep(100);

        drive.followTrajectory(drive.trajectoryBuilder()
                .splineTo(new Pose2d(20,48,Math.PI))
                .lineTo(new Vector2d(4,48),new ConstantInterpolator(Math.PI))
                .build());
        update();

        /*

        drive.followTrajectory(drive.trajectoryBuilder()
                .splineTo(new Pose2d(5,34,Math.PI),new ConstantInterpolator(Math.PI))
                .build());
        update();

         */



/*
        drive.followTrajectory(drive.trajectoryBuilder()
                .reverse()
                .splineTo(new Pose2d(0,36,Math.toRadians(180)))
                .lineTo(new Vector2d(40,36)).build());

        update();

        rightGrab.extend();
        sleep(700);
        rightGrab.open();
        sleep(250);
        rightGrab.retract();
        sleep(500);

        drive.followTrajectory(drive.trajectoryBuilder()
                .lineTo(new Vector2d(0,36)).build());

        update();
        */

    }
    public void update(){
        if(isStopRequested()){
            return;
        }

        while(!isStopRequested()&&drive.isBusy()){
            drive.update();
        }
    }


}
