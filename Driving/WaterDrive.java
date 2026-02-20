package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;


@TeleOp(name="Water Drive")
public class WaterDrive extends LinearOpMode{
    // Declare OpMode members for each of the 4 motors.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftDrive = null;
    private DcMotor rightDrive = null;
    private DcMotor top1Drive = null;
    private DcMotor top2Drive = null;

    private DcMotor rotateClaw1 = null;
    private DcMotor controlClaw1 = null;
    private DcMotor rotateClaw2 = null;
    private DcMotor controlClaw2 = null;


    @Override
    public void runOpMode() {
        leftDrive = hardwareMap.get(DcMotor.class, "leftDrive");
        rightDrive = hardwareMap.get(DcMotor.class, "rightDrive");
        top1Drive = hardwareMap.get(DcMotor.class, "top1Drive");
        top2Drive = hardwareMap.get(DcMotor.class, "top2Drive");
        rotateClaw1 = hardwareMap.get(DcMotor.class, "rotateClaw1");
        controlClaw1 = hardwareMap.get(DcMotor.class, "controlClaw1");
        rotateClaw2 = hardwareMap.get(DcMotor.class, "rotateClaw2");
        controlClaw2 = hardwareMap.get(DcMotor.class, "controlClaw2");


        boolean rotating = false;

        waitForStart();

        while (opModeIsActive()) {

            // right stick is y = front/back x = turn
            if (gamepad1.right_stick_y == -1) {
                leftDrive.setPower(0.5);
                rightDrive.setPower(0.5);
            }

            if (gamepad1.right_stick_y == 1) {
                leftDrive.setPower(-0.5);
                rightDrive.setPower(-0.5);
            }

            if (gamepad1.right_stick_x == -1) {
                leftDrive.setPower(0.5);
                rightDrive.setPower(-0.5);
            }

            if (gamepad1.right_stick_x == 1) {
                leftDrive.setPower(-0.5);
                rightDrive.setPower(0.5);
            }

            // left stick y = up/down
            if (gamepad1.left_stick_y == -1) {
                top1Drive.setPower(0.5);
                top2Drive.setPower(0.5);
            }

            if (gamepad1.left_stick_y == 1) {
                top1Drive.setPower(-0.5);
                top2Drive.setPower(-0.5);
            }

            // dpad up/down = pitch
            if (gamepad1.dpad_up) {
                top1Drive.setPower(0.5);
                top2Drive.setPower(-0.5);
            }

            if (gamepad1.dpad_down) {
                top1Drive.setPower(-0.5);
                top2Drive.setPower(0.5);
            }

            // rotate claw1
            if (gamepad1.b) {
                rotateClaw1.setPower(0.5);
            } else {
                rotateClaw1.setPower(0);
            }

            if (gamepad1.x) {
                rotateClaw1.setPower(-0.5);
            } else {
                rotateClaw1.setPower(0);
            }

            // rotate claw2

            if (gamepad1.dpad_left) {
                rotateClaw2.setPower(0.5);
            } else {
                rotateClaw2.setPower(0);
            }

            if (gamepad1.dpad_left) {
                rotateClaw2.setPower(-0.5);
            } else {
                rotateClaw2.setPower(0);
            }

            // open/close claw1

            if (gamepad1.a) {
                controlClaw1.setPower(0.5);
            } else {
                controlClaw1.setPower(0);
            }

            // open/close claw2

            if (gamepad1.dpad_down) {
                controlClaw2.setPower(0.5);
            } else {
                controlClaw2.setPower(0);
            }
        }
    }
}