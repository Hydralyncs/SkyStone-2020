package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.path.heading.ConstantInterpolator;
import com.acmerobotics.roadrunner.path.heading.LinearInterpolator;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Const;
import org.firstinspires.ftc.teamcode.roadrunner.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.subsystems.Detector;
import org.firstinspires.ftc.teamcode.subsystems.DriveTrain;
import org.firstinspires.ftc.teamcode.subsystems.Hook;
import org.firstinspires.ftc.teamcode.subsystems.LeftGrab;
import org.firstinspires.ftc.teamcode.subsystems.LiftExt;
import org.firstinspires.ftc.teamcode.subsystems.RightGrab;
import org.yaml.snakeyaml.scanner.Constant;

import java.util.Vector;

import kotlin.Unit;

@Autonomous()
public class BlueThreeStone extends LinearOpMode {


    private DriveTrain drive;
    private RightGrab rightGrab;
    private Detector detector;
    private LeftGrab leftGrab;
    private Hook hook;
    private LiftExt lift;

    private int stonePosition = 0;


    public void runOpMode() throws InterruptedException {


        drive = new DriveTrain(hardwareMap);
        detector = new Detector(hardwareMap);
        //rightGrab = new RightGrab(hardwareMap);
        rightGrab = new RightGrab(hardwareMap);
        leftGrab = new LeftGrab(hardwareMap);
        hook = new Hook(hardwareMap);
        lift = new LiftExt(hardwareMap);

        lift.retract();

        hook.close();

        leftGrab.retract();
        leftGrab.close();

        rightGrab.retract();
        rightGrab.open();

        drive.setPoseEstimate(new Pose2d(-38, 63, Math.toRadians(-90)));
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
                .addMarker(0.8,()->{
                    rightGrab.setBigPosition(0.76);
                    rightGrab.setSmallPosition(0.8);
                    return Unit.INSTANCE;
                })
                .splineTo(new Pose2d(-53,38,Math.toRadians(180)))
                .build());
        update();

        drive.followTrajectory(drive.trajectoryBuilder()
                .lineTo(new Vector2d(-44-(stonePosition*8)-offset,32),new ConstantInterpolator(Math.PI))
                .build());
        update();

        rightGrab.extend();
        rightGrab.close();
        waitFor(400);
        rightGrab.setBigPosition(0.5);
        waitFor(100);



        drive.followTrajectory(drive.trajectoryBuilder()
                .reverse()
                .splineTo(new Pose2d(-12,40,Math.toRadians(180)),new ConstantInterpolator(Math.PI))
                .lineTo(new Vector2d(12,40))
                .splineTo(new Pose2d(49,28.5,Math.toRadians(180)), new ConstantInterpolator(Math.PI))
                .build());

        update();


        rightGrab.setBigPosition(0.6);
        waitFor(100);
        rightGrab.setSmallPosition(0.5);
        waitFor(100);
        rightGrab.retract();
        rightGrab.close();


        drive.followTrajectory(drive.trajectoryBuilder()
                .splineTo(new Pose2d(-5,34,Math.PI),new ConstantInterpolator(Math.PI))
                .addMarker(()->{
                    rightGrab.setBigPosition(0.78);
                    rightGrab.open();
                    return Unit.INSTANCE;
                })
                .splineTo(new Pose2d(-20-stonePosition*8,33,Math.PI),new ConstantInterpolator(Math.PI))
                .build());
        update();

        drive.followTrajectory(drive.trajectoryBuilder()
                .addMarker(new Vector2d(-20-stonePosition*8,31),()->{
                    rightGrab.extend();
                    rightGrab.close();
                    return Unit.INSTANCE;
                })
                .lineTo(new Vector2d(-20-stonePosition*8,30),new ConstantInterpolator(Math.PI))
                .build());
        update();


        waitFor(400);
        rightGrab.setBigPosition(0.5);
        waitFor(100);



        drive.followTrajectory(drive.trajectoryBuilder()
                .reverse()
                .splineTo(new Pose2d(-5,37,Math.PI),new ConstantInterpolator(Math.PI))
                .lineTo(new Vector2d(12,37),new ConstantInterpolator(Math.PI))
                .splineTo(new Pose2d(57,27.5,Math.PI),new ConstantInterpolator(Math.PI))
                .build());
        update();

        rightGrab.setBigPosition(0.6);
        waitFor(100);
        rightGrab.setSmallPosition(0.5);
        waitFor(100);




        double thirdStoneXPos = -20;
        if(stonePosition == 0){
            thirdStoneXPos = -28;
        }

        drive.followTrajectory(drive.trajectoryBuilder()
                .addMarker(()->{
                    rightGrab.retract();
                    rightGrab.close();
                    return Unit.INSTANCE;
                })
                .splineTo(new Pose2d(5,34,Math.PI),new ConstantInterpolator(Math.PI))
                .addMarker(new Vector2d(-2,34),()->{
                    rightGrab.setBigPosition(0.78);
                    rightGrab.open();
                    return Unit.INSTANCE;
                })
                .splineTo(new Pose2d(thirdStoneXPos,30,Math.PI),new ConstantInterpolator(Math.PI))
                .build());
        update();

        drive.followTrajectory(drive.trajectoryBuilder()
                .addMarker(new Vector2d(thirdStoneXPos,28),()->{
                    rightGrab.close();
                    return Unit.INSTANCE;
                })
                .lineTo(new Vector2d(thirdStoneXPos,27),new ConstantInterpolator(Math.PI))
                .build());
        update();

        waitFor(300);

        rightGrab.setBigPosition(0.5);
        waitFor(100);

        drive.followTrajectory(drive.trajectoryBuilder()
                .reverse()
                .splineTo(new Pose2d(-5,35,Math.PI),new ConstantInterpolator(Math.PI))
                .lineTo(new Vector2d(12,35),new ConstantInterpolator(Math.PI))
                .splineTo(new Pose2d(44,26.5,Math.PI),new ConstantInterpolator(Math.PI))
                .build());
        update();

        rightGrab.setBigPosition(0.6);
        waitFor(100);
        rightGrab.setSmallPosition(0.5);
        waitFor(100);

        drive.followTrajectory(drive.trajectoryBuilder()
                .addMarker(()->{
                    rightGrab.retract();
                    return Unit.INSTANCE;
                })
                .addMarker(0.2,()->{
                    rightGrab.close();
                    return Unit.INSTANCE;
                })
                .lineTo(new Vector2d(58,30),new LinearInterpolator(Math.PI,-Math.PI/2))
                .lineTo(new Vector2d(58,21),new ConstantInterpolator(Math.PI/2))
                .build());
        update();

        hook.open();
        waitFor(300);

        drive.followTrajectory(drive.trajectoryBuilder()
                .lineTo(new Vector2d(40,64),new LinearInterpolator(Math.PI/2,Math.PI/1.5))
                .build());
        update();
        hook.close();
        waitFor(100);

        drive.followTrajectory(drive.trajectoryBuilder()
                .splineTo(new Pose2d(20,44,Math.PI),new ConstantInterpolator(Math.PI))
                .lineTo(new Vector2d(4,44),new ConstantInterpolator(Math.PI))
                .build());
        update();






    }
    public void update(){
        if(isStopRequested()){
            return;
        }

        while(!isStopRequested()&&drive.isBusy()){
            drive.update();
        }
    }

    public void toStone(int stonePos, int offsetX, int offSetY) {
        drive.followTrajectory(drive.trajectoryBuilder()
                .lineTo(new Vector2d(-5,36),new ConstantInterpolator(Math.PI))
                .splineTo(new Pose2d(-19-stonePos*8-offsetX,30.8-offSetY,Math.PI),new ConstantInterpolator(Math.PI))
                .build());
    }

    public void waitFor(double millis){

        ElapsedTime timer = new ElapsedTime();

        while(timer.milliseconds()<millis&&!isStopRequested()){
            drive.update();
        }

    }

}
