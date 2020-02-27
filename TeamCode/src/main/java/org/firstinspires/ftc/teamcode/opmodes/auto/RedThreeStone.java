package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.acmerobotics.roadrunner.drive.Drive;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.path.heading.ConstantInterpolator;
import com.acmerobotics.roadrunner.path.heading.HeadingInterpolator;
import com.acmerobotics.roadrunner.path.heading.LinearInterpolator;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.vuforia.CameraDevice;

import org.firstinspires.ftc.teamcode.roadrunner.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.subsystems.Detector;
import org.firstinspires.ftc.teamcode.subsystems.DriveTrain;
import org.firstinspires.ftc.teamcode.subsystems.Hook;
import org.firstinspires.ftc.teamcode.subsystems.LeftGrab;
import org.firstinspires.ftc.teamcode.subsystems.LiftExt;
import org.firstinspires.ftc.teamcode.subsystems.RightGrab;

import kotlin.Unit;

@Autonomous()
public class RedThreeStone extends LinearOpMode {

    int stonePosition;

    private LiftExt lift;
    private Hook hook;
    private RightGrab rightGrab;
    private DriveTrain drive;
    private Detector detector;
    private LeftGrab leftGrab;

    public void runOpMode() throws InterruptedException {

        lift = new LiftExt(hardwareMap);
        hook = new Hook(hardwareMap);
        rightGrab = new RightGrab(hardwareMap);
        drive = new DriveTrain(hardwareMap);
        detector = new Detector(hardwareMap);
        leftGrab = new LeftGrab(hardwareMap);

        hook.close();

        lift.retract();

        rightGrab.retract();
        rightGrab.close();

        leftGrab.retract();
        leftGrab.open();

        drive.setPoseEstimate(new Pose2d(-38, -62, Math.toRadians(90)));
        detector.startStreaming();
        while(!isStarted()&&!isStopRequested()){
            double position = detector.detector.foundRectangle().x+detector.detector.foundRectangle().width/2;
            if(position<70){
                stonePosition = 2;
            } else if(position<140){
                stonePosition = 1;
            } else {
                stonePosition = 0;
            }
            telemetry.addData("Stone Position: ",stonePosition);
            telemetry.update();
        }
        detector.phoneCam.stopStreaming();

        drive.followTrajectory(drive.trajectoryBuilder()
                .splineTo(new Pose2d(-15,-35,0))
                .build());
        update();

        drive.followTrajectory(drive.trajectoryBuilder()
                .reverse()
                .addMarker(()->{
                    leftGrab.setBigPosition(0.1);
                    leftGrab.setSmallPosition(0);
                    return Unit.INSTANCE;
                })
                .lineTo(new Vector2d(-20-(8*stonePosition),-32),new ConstantInterpolator(0))
                .build());
        update();

        drive.followTrajectory(drive.trajectoryBuilder()
                .addMarker(()->{
                    leftGrab.extend();
                    return Unit.INSTANCE;
                })
                .lineTo(new Vector2d(-20-(8*stonePosition),-31),new ConstantInterpolator(0))
                .build());
        update();

        leftGrab.close();
        waitFor(350);
        leftGrab.setBigPosition(0.35);
        waitFor(100);

        drive.followTrajectory(drive.trajectoryBuilder()
                .splineTo(new Pose2d(-10,-39,0),new ConstantInterpolator(0))
                .lineTo(new Vector2d(10,-39),new ConstantInterpolator(0))
                .splineTo(new Pose2d(49,-30,0),new ConstantInterpolator(0))
                .build());
        update();

        leftGrab.setBigPosition(0.2);
        waitFor(50);
        leftGrab.setSmallPosition(0.25);
        waitFor(100);
        leftGrab.retract();
        waitFor(200);
        leftGrab.close();



        double offset = 0;
        if(stonePosition == 1){
            offset = 1;
        }
        if(stonePosition == 2){
            offset = 1;
        }

        drive.followTrajectory(drive.trajectoryBuilder()
                .reverse()
                .splineTo(new Pose2d(10,-36,0),new ConstantInterpolator(0))
                .lineTo(new Vector2d(-35,-36), new ConstantInterpolator(0))
                .addMarker(()->{
                    leftGrab.setBigPosition(0.1);
                    leftGrab.setSmallPosition(0);
                    return Unit.INSTANCE;
                })
                .splineTo(new Pose2d(-44-(8*stonePosition),-29,0),new ConstantInterpolator(0))
                .build());
        update();

        drive.followTrajectory(drive.trajectoryBuilder()
                .addMarker(new Vector2d(-44-(8*stonePosition),-27.5),()->{
                    leftGrab.extend();
                    leftGrab.close();
                    return Unit.INSTANCE;
                })
                .lineTo(new Vector2d(-44-(8*stonePosition),-27),new ConstantInterpolator(0))
                .build());
        update();

        waitFor(300);
        leftGrab.setBigPosition(0.35);
        waitFor(100);

        drive.followTrajectory(drive.trajectoryBuilder()
                .splineTo(new Pose2d(-10,-39,0),new ConstantInterpolator(0))
                .lineTo(new Vector2d(10,-39),new ConstantInterpolator(0))
                .splineTo(new Pose2d(58,-28,0),new ConstantInterpolator(0))
                .build());
        update();

        leftGrab.setBigPosition(0.2);
        waitFor(100);
        leftGrab.setSmallPosition(0.25);
        waitFor(100);
        leftGrab.retract();

        double thirdStoneXPos = -20;
        if(stonePosition == 0){
            thirdStoneXPos = -28;
        }

        drive.followTrajectory(drive.trajectoryBuilder()
                .reverse()
                .addMarker(()->{
                    leftGrab.retract();
                    leftGrab.close();
                    return Unit.INSTANCE;
                })
                .splineTo(new Pose2d(5,-35,0),new ConstantInterpolator(0))
                .addMarker(new Vector2d(-2,-35),()->{
                    leftGrab.setBigPosition(0.1);
                    leftGrab.setSmallPosition(0);
                    return Unit.INSTANCE;
                })
                .splineTo(new Pose2d(thirdStoneXPos,-27,0),new ConstantInterpolator(0))
                .build());
        update();

        drive.followTrajectory(drive.trajectoryBuilder()
                .addMarker(new Vector2d(thirdStoneXPos,-25.5),()->{
                    leftGrab.close();
                    return Unit.INSTANCE;
                })
                .lineTo(new Vector2d(thirdStoneXPos,-25),new ConstantInterpolator(0))
                .build());
        update();

        waitFor(300);
        leftGrab.setBigPosition(0.35);
        waitFor(100);

        drive.followTrajectory(drive.trajectoryBuilder()
                .splineTo(new Pose2d(-7,-38,0),new ConstantInterpolator(0))
                .lineTo(new Vector2d(12,-38),new ConstantInterpolator(0))
                .splineTo(new Pose2d(48,-26,0),new ConstantInterpolator(0))
                .build());
        update();

        leftGrab.setBigPosition(0.2);
        waitFor(100);
        leftGrab.setSmallPosition(0.25);
        waitFor(100);
        leftGrab.retract();

        drive.followTrajectory(drive.trajectoryBuilder()
                .addMarker(()->{
                    leftGrab.retract();
                    return Unit.INSTANCE;
                })
                .addMarker(0.2,()->{
                    leftGrab.close();
                    return Unit.INSTANCE;
                })
                .lineTo(new Vector2d(58,-30),new LinearInterpolator(0,-Math.PI/2))
                .lineTo(new Vector2d(58,-21),new ConstantInterpolator(-Math.PI/2))
                .build());
        update();

        hook.open();
        waitFor(300);

        drive.followTrajectory(drive.trajectoryBuilder()
                .lineTo(new Vector2d(40,-64),new LinearInterpolator(-Math.PI/2,-Math.PI/1.5))
                .build());
        update();
        hook.close();
        waitFor(100);

        drive.followTrajectory(new TrajectoryBuilder(new Pose2d(drive.getPoseEstimate().getX(),drive.getPoseEstimate().getY(),Math.PI/2), DriveConstants.BASE_CONSTRAINTS)
                .splineTo(new Pose2d(24,-34,Math.PI),new ConstantInterpolator(Math.PI))
                .lineTo(new Vector2d(4,-34),new ConstantInterpolator(Math.PI))
                .build());
        update();


    }

    public void update(){
        while(!isStopRequested()&&drive.isBusy()){
            drive.update();
        }
    }

    public void waitFor(double millis){

        ElapsedTime timer = new ElapsedTime();

        while(timer.milliseconds()<millis&&!isStopRequested()){
            drive.update();
        }

    }


}
