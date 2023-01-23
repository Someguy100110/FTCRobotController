/*
    Utility class to control the robot's driving during autonomus
*/

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;

public class DirectionAS extends LinearOpMode {

    // Declare class members.
    private DcMotor leftFrontDrive = null;
    private DcMotor rightFrontDrive = null;
    private DcMotor leftBackDrive = null;
    private DcMotor rightBackDrive = null;
    private DcMotor lift = null;
    private Servo claw = null;
    private Servo claw2 = null;
    private Telemetry telemetry;

    BNO055IMU imu;

    HardwareMap hardwareMap = null;

    private ElapsedTime runtime = new ElapsedTime();

    private double pulsesPerInch = 537.7 / (Math.PI * 3.93701);
    private int rPulsesPerInch = (int) Math.floor(pulsesPerInch);
    // constructor
    public DirectionAS(OpMode opmode) {
        // set up wheels

        hardwareMap = opmode.hardwareMap;

        telemetry = opmode.telemetry;

        leftFrontDrive  = hardwareMap.dcMotor.get("LFD");
        rightFrontDrive = hardwareMap.dcMotor.get("RFD");
        leftBackDrive  = hardwareMap.dcMotor.get("LBD");
        rightBackDrive = hardwareMap.dcMotor.get("RBD");
        lift = hardwareMap.dcMotor.get("lift");
        claw = hardwareMap.servo.get("CLAW");
        claw2 = hardwareMap.servo.get("CLAWTWO");

        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);

        leftFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // Set up IMU
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json";
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);
    }


    // drive forwards
    public void forwards(double power, int ms) {
        setAllPower(-power);

        sleep(ms);

        setAllPower(0d);
    }

    // drive right
    public void right(double power, int ms) {
        setMotorPower(-power, power, power, -power);

        sleep(ms);

        setAllPower(0d);
    }

    // drive left
    public void left(double power, int ms) {
        setMotorPower(power, -power, -power, power);

        sleep(ms);

        setAllPower(0d);
    }

    // drive backwakrds
    public void backwards(double power, int ms) {
        setAllPower(power);

        sleep(ms);

        setAllPower(0d);
    }

    public void stop(int ms) {
        setAllPower(0d);
        sleep(ms);
    }


    public double getAbsoluteAngle() {
        return imu.getAngularOrientation(
                AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES
        ).firstAngle;
    }
    public void setMotorMode (DcMotor.RunMode run){
        leftFrontDrive.setMode(run);
        rightFrontDrive.setMode(run);
        leftBackDrive.setMode(run);
        rightBackDrive.setMode(run);
    }
    /*public void turnPID(double degrees) {
        turnToPID(degrees + getAbsoluteAngle());
    }

    public void turnToPID(double targetAngle) {
        TurnPIDController pid = new TurnPIDController(targetAngle, 0.01, 0.01, 0.003);
        double error = targetAngle - getAbsoluteAngle();


        while (Math.abs(error) > .5 || pid.getLastSlope() > 0.75) {
            double motorPower = pid.update(-getAbsoluteAngle());
            setMotorPower(-motorPower, motorPower, -motorPower, motorPower);
            error = targetAngle + getAbsoluteAngle();
            telemetry.addData("motorpower", motorPower);
            telemetry.addData("gyro", getAbsoluteAngle());
            telemetry.addData("error", error);
            telemetry.update();
        }
        setAllPower(0);
    }*/

    public void turnRight(double power, int time) {
        leftFrontDrive.setPower(power);
        rightFrontDrive.setPower(power);
        leftBackDrive.setPower(-power);
        rightBackDrive.setPower(-power);
        sleep(time);
        stop();
    }
    public void setTarget(int pos1, int pos2, int pos3, int pos4){
        leftFrontDrive.setTargetPosition(pos1);
        rightFrontDrive.setTargetPosition(pos2);
        leftBackDrive.setTargetPosition(pos3);
        rightBackDrive.setTargetPosition(pos4);
    }
    public void setAllTarget(int pos){
        leftFrontDrive.setTargetPosition(pos);
        rightFrontDrive.setTargetPosition(pos);
        leftBackDrive.setTargetPosition(pos);
        rightBackDrive.setTargetPosition(pos);
    }

    // Set power to all motors
    public void setAllPower(double p) {setMotorPower(p,p,p,p);}

    // Set power to motors
    public void setMotorPower(double p1, double p2, double p3, double p4) {
        leftFrontDrive.setPower(p1);
        rightFrontDrive.setPower(p2);
        leftBackDrive.setPower(p3);
        rightBackDrive.setPower(p4);
    }
    public boolean isAllBusy(){
        return leftFrontDrive.isBusy() && leftBackDrive.isBusy() && rightFrontDrive.isBusy() && rightBackDrive.isBusy();
    }
    public void forwardInches(double in, double speed, boolean focus){
        int ptoi = (int) in*rPulsesPerInch;
        setMotorMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        setMotorMode(DcMotor.RunMode.RUN_USING_ENCODER);
        setAllTarget(ptoi);
        setAllPower(speed);
        setMotorMode(DcMotor.RunMode.RUN_TO_POSITION);
        if(focus){
            while(isAllBusy()){
                telemetry.addData("lfd pos", leftFrontDrive.getCurrentPosition());
                telemetry.addData("rfd pos", rightFrontDrive.getCurrentPosition());
                telemetry.addData("lbd pos", leftBackDrive.getCurrentPosition());
                telemetry.addData("rbd pos", rightBackDrive.getCurrentPosition());
                telemetry.update();
            }
            setAllPower(0d);
        }
    }
    public void turnDegree(int degree, double speed, boolean focus){
        int ptod = (int) (17.702585 * degree * pulsesPerInch) / 90;
        //18.30779053
        setMotorMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        setMotorMode(DcMotor.RunMode.RUN_USING_ENCODER);
        setTarget(ptod, -ptod, ptod, -ptod);
        setAllPower(speed);
        setMotorMode(DcMotor.RunMode.RUN_TO_POSITION);
        if(focus){
            while(isAllBusy()){

            }
            setAllPower(0d);
        }
    }
    public void strafeInches(double in, double speed, boolean focus){
        int ptoi = (int) in*rPulsesPerInch;
        setMotorMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        setMotorMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftFrontDrive.setTargetPosition(ptoi);
        rightFrontDrive.setTargetPosition(-ptoi);
        leftBackDrive.setTargetPosition(-ptoi);
        rightBackDrive.setTargetPosition(ptoi);
        setMotorPower(speed, speed, speed, speed);
        setMotorMode(DcMotor.RunMode.RUN_TO_POSITION);
        if(focus){
            while(isAllBusy()){

            }
            setAllPower(0d);
        }
    }
    public void openClaw(){
        claw.setPosition(.5);
        claw2.setPosition(0.5);
    }
    public void closeClaw(){
        claw.setPosition(1);
        claw2.setPosition(-.5);
    }
    public void moveArm(int pulse, double speed, boolean focus){
        lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lift.setTargetPosition(pulse);
        lift.setPower(speed);
        lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        if(focus){
            while(lift.isBusy()){
                telemetry.addData("lift position", lift.getCurrentPosition());
            }
            lift.setPower(0d);
        }
    }
    @Override
    public void runOpMode() {

    }

}

