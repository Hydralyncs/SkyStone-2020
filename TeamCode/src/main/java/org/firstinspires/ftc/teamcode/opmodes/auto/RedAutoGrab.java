package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.acmerobotics.roadrunner.drive.Drive;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.path.heading.ConstantInterpolator;
import com.acmerobotics.roadrunner.path.heading.HeadingInterpolator;
import com.acmerobotics.roadrunner.path.heading.LinearInterpolator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.vuforia.CameraDevice;

import org.firstinspires.ftc.teamcode.subsystems.Detector;
import org.firstinspires.ftc.teamcode.subsystems.DriveTrain;
import org.firstinspires.ftc.teamcode.subsystems.Hook;
import org.firstinspires.ftc.teamcode.subsystems.LeftGrab;
import org.firstinspires.ftc.teamcode.subsystems.RightGrab;

import kotlin.Unit;

@Autonomous()
public class RedAutoGrab extends LinearOpMode {

    int stonePosition;

    private Hook hook;
    private RightGrab rightGrab;
    private DriveTrain drive;
    private Detector detector;
    private LeftGrab leftGrab;

    public void runOpMode() throws InterruptedException {

        hook = new Hook(hardwareMap);
        rightGrab = new RightGrab(hardwareMap);
        drive = new DriveTrain(hardwareMap);
        detector = new Detector(hardwareMap);
        leftGrab = new LeftGrab(hardwareMap);

        hook.close();

        rightGrab.retract();
        rightGrab.open();

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
                .lineTo(new Vector2d(-20-(8*stonePosition),-31.5),new ConstantInterpolator(0))
                .build());
        update();

        drive.followTrajectory(drive.trajectoryBuilder()
                .addMarker(()->{
                    leftGrab.extend();
                    return Unit.INSTANCE;
                })
                .lineTo(new Vector2d(-20-(8*stonePosition),-31.5),new ConstantInterpolator(0))
                .build());
        update();

        leftGrab.close();
        sleep(350);
        leftGrab.setBigPosition(0.35);
        sleep(100);

        drive.followTrajectory(drive.trajectoryBuilder()
                .splineTo(new Pose2d(-10,-39,0),new ConstantInterpolator(0))
                .lineTo(new Vector2d(10,-39),new ConstantInterpolator(0))
                .splineTo(new Pose2d(45,-29,0),new ConstantInterpolator(0))
                .build());
        update();

        leftGrab.setBigPosition(0.25);
        sleep(50);
        leftGrab.open();
        sleep(100);
        leftGrab.retract();
        sleep(100);


        double offset = 0;
        if(stonePosition == 1){
            offset = 1;
        }
        if(stonePosition == 2){
            offset = 1;
        }

        drive.followTrajectory(drive.trajectoryBuilder()
                .reverse()
                .splineTo(new Pose2d(10,-35,0),new ConstantInterpolator(0))
                .lineTo(new Vector2d(-35,-35), new ConstantInterpolator(0))
                .build());
        update();

        drive.followTrajectory(drive.trajectoryBuilder()
                .addMarker(()->{
                    leftGrab.setBigPosition(0.1);
                    leftGrab.setSmallPosition(0);
                    return Unit.INSTANCE;
                })
                .lineTo(new Vector2d(-44-(8*stonePosition),-28.5),new ConstantInterpolator(0))
                .build());
        update();

        drive.followTrajectory(drive.trajectoryBuilder()
                .addMarker(()->{
                    leftGrab.extend();
                    return Unit.INSTANCE;
                })
                .lineTo(new Vector2d(-44-(8*stonePosition),-28.5),new ConstantInterpolator(0))
                .build());
        update();

        leftGrab.close();
        sleep(400);
        leftGrab.setBigPosition(0.35);
        sleep(100);

        drive.followTrajectory(drive.trajectoryBuilder()
                .splineTo(new Pose2d(-10,-39,0),new ConstantInterpolator(0))
                .lineTo(new Vector2d(10,-39),new ConstantInterpolator(0))
                .splineTo(new Pose2d(58,-28.5,0),new ConstantInterpolator(0))
                .build());
        update();

        leftGrab.setBigPosition(0.2);
        sleep(100);
        leftGrab.open();
        sleep(100);
        leftGrab.retract();
        sleep(100);

        drive.followTrajectory(drive.trajectoryBuilder()
                .lineTo(new Vector2d(58,-34),new ConstantInterpolator(0))
                .build());
        update();

        drive.turn(-Math.PI/2);
        update();

        drive.followTrajectory(drive.trajectoryBuilder()
                .lineTo(new Vector2d(62,-27.5),new ConstantInterpolator(-Math.PI/2))
                .build());
        update();

        hook.open();
        sleep(300);

        drive.followTrajectory(drive.trajectoryBuilder()
                .lineTo(new Vector2d(54,-32),new ConstantInterpolator(-Math.PI/2))
                .lineTo(new Vector2d(40,-68),new LinearInterpolator(-Math.PI/2,-Math.PI/1.4))
                .build());
        update();

        hook.close();


        drive.followTrajectory(drive.trajectoryBuilder()
                .back(5)
                .splineTo(new Pose2d(20,-52,Math.PI))
                .lineTo(new Vector2d(4,-52),new ConstantInterpolator(Math.PI))
                .build());
        update();
        /*
        drive.followTrajectory(drive.trajectoryBuilder()
                .reverse()
                .splineTo(new Pose2d(12,-34.5,0),new ConstantInterpolator(0))
                .lineTo(new Vector2d(6,-34.5), new ConstantInterpolator(0))
                .build());
        update();
         */

    }

    public void update(){
        while(!isStopRequested()&&drive.isBusy()){
            drive.update();
        }
    }


}
