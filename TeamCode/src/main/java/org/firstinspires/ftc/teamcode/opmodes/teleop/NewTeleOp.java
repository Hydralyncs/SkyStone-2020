package org.firstinspires.ftc.teamcode.opmodes.teleop;


import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.subsystems.CapstoneMechanism;
import org.firstinspires.ftc.teamcode.subsystems.Claw;
import org.firstinspires.ftc.teamcode.subsystems.DriveTrain;
import org.firstinspires.ftc.teamcode.subsystems.Hook;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.LiftExt;
import org.firstinspires.ftc.teamcode.subsystems.LeftGrab;
import org.firstinspires.ftc.teamcode.subsystems.RightGrab;


@TeleOp(name="New TeleOp",group="teleop")
public class NewTeleOp extends LinearOpMode {

    private CapstoneMechanism capstoneMechanism;
    private Claw claw;
    private DriveTrain dt;
    private Hook hook;
    private Intake intake;
    private RightGrab rightGrab;
    private LiftExt lift;
    private LeftGrab leftGrab;


    private boolean x2WasPressed = false;
    private boolean y2WasPressed = false;
    private boolean aWasPressed = false;
    private boolean bWasPressed = false;
    private boolean xWasPressed = false;
    private boolean yWasPressed = false;
    private boolean a2WasPressed = false;

    private double xPower;
    private double yPower;
    private double turnPower;
    private double denominator;

    @Override
    public void runOpMode() throws InterruptedException {

        capstoneMechanism = new CapstoneMechanism(hardwareMap);
        leftGrab = new LeftGrab(hardwareMap);
        claw = new Claw(hardwareMap);
        hook = new Hook(hardwareMap);
        dt = new DriveTrain(hardwareMap);
        intake = new Intake(this);
        lift = new LiftExt(hardwareMap);
        rightGrab = new RightGrab(hardwareMap);


        leftGrab.close();
        leftGrab.retract();
        rightGrab.close();
        rightGrab.retract();
        double multiplier=0.5;
        waitForStart();

        while (opModeIsActive()) {



            if(gamepad1.right_bumper){
                multiplier = 1;
            } else {
                multiplier = 0.5;
            }

            xPower = Math.pow(gamepad1.left_stick_y,3)*multiplier;
            yPower = Math.pow(gamepad1.left_stick_x,3)*multiplier;
            turnPower = Math.pow(-gamepad1.right_stick_x,3) * 0.85 * multiplier;

            if(Math.abs(xPower) + Math.abs(yPower) + Math.abs(turnPower) > 1){
                denominator = Math.abs(xPower) + Math.abs(yPower) + Math.abs(turnPower);
                xPower = xPower/denominator;
                yPower = yPower/denominator;
                turnPower = turnPower/denominator;
                dt.setDrivePower(new Pose2d(xPower,yPower,turnPower));
            } else {
                dt.setDrivePower(new Pose2d(xPower,yPower,turnPower));
            }



            // drivetrain


            // slide
            if(lift.isExtended && claw.isClosed && gamepad2.left_stick_y > 0){
                lift.setPower(Math.pow(-gamepad2.left_stick_y,3)*0.13);
            } else {
                /*
                double liftHeight = lift.getCurrentHeight();
                double liftPower = Math.pow(-gamepad2.left_stick_y,3)*(1+LiftExt.GRAVITY_FF);
                if(liftHeight<5&&gamepad2.left_stick_y>0.4){
                    lift.setPower(Math.pow(liftPower,3)*(liftHeight/5-liftHeight));
                }else {

                 */
                    lift.setPower(Math.pow(-gamepad2.left_stick_y,3)*(1+LiftExt.GRAVITY_FF));

            }

            // intake
            intake.setPower(gamepad2.right_trigger-gamepad2.left_trigger);

            // hook
            if(gamepad1.left_bumper && !aWasPressed){
                aWasPressed = true;
                hook.toggle();
            }
            if(!gamepad1.left_bumper){
                aWasPressed = false;
            }

            if(gamepad2.b && !bWasPressed){
                bWasPressed = true;
                claw.toggle();
            }
            if(!gamepad2.b){
                bWasPressed = false;
            }


            // claw

            if(gamepad2.x && !xWasPressed){
                xWasPressed = true;
                capstoneMechanism.toggle();
            }
            if(!gamepad2.x){
                xWasPressed = false;
            }

            if(gamepad2.a && !a2WasPressed){
                a2WasPressed = true;
                lift.toggleCrank();
            }
            if(!gamepad2.a){
                a2WasPressed = false;
            }





            claw.repeat();

            telemetry.addData("Wheels",dt.getWheelPositions());
            //telemetry.addData("Lift",lift.getCurrentHeight());
            telemetry.addData("Gamepad2 stick: ",gamepad2.left_stick_y);
            telemetry.update();


        }
    }
}
