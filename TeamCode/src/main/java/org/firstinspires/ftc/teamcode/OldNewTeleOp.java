package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotor.RunMode;
import com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior;
import com.qualcomm.robotcore.hardware.Servo;

//@TeleOp(name="NewTeleOp")
@TeleOp(name="! Consistent Arm Android Teleop !")

public class OldNewTeleOp extends LinearOpMode {

    private DcMotor RightFrontDrive = null;
    private DcMotor LeftFrontDrive = null;
    private DcMotor LeftBackDrive = null;
    private DcMotor RightBackDrive = null;
    private DcMotor lift = null;
    private Servo claw = null;
    private Servo claw2 = null;
    public int ClawPosition = 0;
    public boolean buttonIsPressed = false;
    @Override
    public void runOpMode() throws InterruptedException
    {
        telemetry.addData("status", "Initialized");
        telemetry.update();

        RightFrontDrive = hardwareMap.dcMotor.get("RFD");
        LeftFrontDrive = hardwareMap.dcMotor.get("LFD");
        LeftBackDrive = hardwareMap.dcMotor.get("LBD");
        RightBackDrive = hardwareMap.dcMotor.get("RBD");
        lift = hardwareMap.dcMotor.get("lift");
        claw = hardwareMap.servo.get("CLAW");
        claw2 = hardwareMap.servo.get("CLAWTWO");
        //back drivers esta aqui
        RightBackDrive.setMode(RunMode.RUN_WITHOUT_ENCODER);
        LeftFrontDrive.setMode(RunMode.RUN_WITHOUT_ENCODER);
        LeftBackDrive.setMode(RunMode.RUN_WITHOUT_ENCODER);
        RightFrontDrive.setMode(RunMode.RUN_WITHOUT_ENCODER);
        lift.setMode(RunMode.STOP_AND_RESET_ENCODER);
        lift.setMode(RunMode.RUN_WITHOUT_ENCODER);
        
        telemetry.update();
        //Set wheel diection
        RightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        LeftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        RightBackDrive.setDirection(DcMotor.Direction.FORWARD);
        LeftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        lift.setDirection(DcMotor.Direction.FORWARD);

        RightFrontDrive.setZeroPowerBehavior(ZeroPowerBehavior.BRAKE);
        RightBackDrive.setZeroPowerBehavior(ZeroPowerBehavior.BRAKE);
        LeftBackDrive.setZeroPowerBehavior(ZeroPowerBehavior.BRAKE);
        LeftFrontDrive.setZeroPowerBehavior(ZeroPowerBehavior.BRAKE);
        lift.setZeroPowerBehavior(ZeroPowerBehavior.BRAKE);



        //constantly run code to make sure that the game has not begun
        waitForStart();

        while(opModeIsActive())
        {
            telemetry.addData("Working", "Working");
            telemetry.addData("Left Trigger", gamepad1.left_trigger);
            telemetry.addData("Right Trigger", gamepad1.right_trigger);
            telemetry.addData("Lift Position", lift.getCurrentPosition());
            telemetry.addData("gamepad2 left stick", gamepad2.left_stick_y);
            telemetry.addData("gamepad2 right stick", gamepad2.right_stick_x);
            telemetry.update();
            // killswitch
            if(!gamepad1.b) {
                // prioritize rotation over strafing
                if (gamepad1.right_stick_x>0.1 || gamepad1.right_stick_x<-0.1) {
                    LeftFrontDrive.setPower(gamepad1.right_stick_x/2);
                    LeftBackDrive.setPower(gamepad1.right_stick_x/2);
                    RightFrontDrive.setPower(-gamepad1.right_stick_x/2);
                    RightBackDrive.setPower(-gamepad1.right_stick_x/2);
                }
                else {
                    LeftFrontDrive.setPower((-gamepad1.left_stick_y + gamepad1.left_stick_x)/2);
                    LeftBackDrive.setPower((-gamepad1.left_stick_y - gamepad1.left_stick_x)/2);
                    RightFrontDrive.setPower((-gamepad1.left_stick_y - gamepad1.left_stick_x)/2);
                    RightBackDrive.setPower((-gamepad1.left_stick_y + gamepad1.left_stick_x)/2);
                }
                if(gamepad1.dpad_up){
                    LeftFrontDrive.setPower(.5);
                    RightFrontDrive.setPower(.5);
                    RightBackDrive.setPower(.5);
                    LeftBackDrive.setPower(.5);
                }
                if(gamepad1.dpad_right){
                    LeftFrontDrive.setPower(.5);
                    RightFrontDrive.setPower(-.5);
                    RightBackDrive.setPower(.5);
                    LeftBackDrive.setPower(-.5);
                }
                if(gamepad1.dpad_left){
                    LeftFrontDrive.setPower(-.5);
                    RightFrontDrive.setPower(.5);
                    RightBackDrive.setPower(-.5);
                    LeftBackDrive.setPower(.5);
                }
                if(gamepad1.dpad_down){
                    LeftFrontDrive.setPower(-.5);
                    RightFrontDrive.setPower(-.5);
                    RightBackDrive.setPower(-.5);
                    LeftBackDrive.setPower(-.5);
                }
            } else {
                // if killed (holding b), add 0 power
                LeftFrontDrive.setPower(0);
                LeftBackDrive.setPower(0);
                RightFrontDrive.setPower(0);
                RightBackDrive.setPower(0);
            }
            // left stick of second game pad = movement of arm
            lift.setPower(2*gamepad1.left_trigger-2*gamepad1.right_trigger);
            // left bumper = close, right = open
            if(gamepad1.left_bumper) {
                ClawPosition = 1;
            }
            if(gamepad1.right_bumper) {
                ClawPosition = 2;
            }
            /*if(gamepad2.left_bumper) {
                ClawPosition = 1;
            }
            if(gamepad2.right_bumper) {
                ClawPosition = 2;
            }*/
            /*if(gamepad1.x) {
                if(!buttonIsPressed && ClawPosition == 1) {
                    ClawPosition = 2;
                    buttonIsPressed = true;
                }
                else if(!buttonIsPressed) {
                    ClawPosition = 1;
                    buttonIsPressed = true;
                }
            } else {
                buttonIsPressed = false;
            }*/
            if(ClawPosition == 1) {
                claw.setPosition(.5);
                claw2.setPosition(0.5);

            }
            if(ClawPosition == 2) {
                claw.setPosition(1);
                claw2.setPosition(-.5);

            }
            
            
        }
    }
}
