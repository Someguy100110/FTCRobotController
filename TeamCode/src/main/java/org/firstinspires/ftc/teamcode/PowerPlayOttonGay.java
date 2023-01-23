package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;

// @Autonomous(name="Otto Octavious's Greatest Invention")
@Autonomous(name="! New Current Otton !")
public class PowerPlayOttonGay extends LinearOpMode{
    ConeDetectionPipeline detectionPipeline;
    int coneNumber = 0;
    @Override
    public void runOpMode(){
        DirectionAS go = new DirectionAS(this);
        detectionPipeline = new ConeDetectionPipeline();

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        OpenCvCamera camera = OpenCvCameraFactory.getInstance().createInternalCamera(OpenCvInternalCamera.CameraDirection.BACK, cameraMonitorViewId);

        camera.openCameraDevice();
        camera.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);

        camera.setPipeline(detectionPipeline);
        waitForStart();
        // 1 Goes backward to see color
        go.closeClaw();
        go.stop(100);
        go.forwardInches(-12, .5, true);
        go.stop(100);
        coneNumber = detectionPipeline.getLatestResult();
        /* 2 Goes to nearest highest junction and puts cone on it by turning around
         It then moves forward, turns to the junction, and delivers it.
         */
        go.moveArm(-500, 1, true);
        go.turnDegree(-240, .5, true);
        go.stop(100);
        go.turnDegree(60, .25, true);
        go.stop(100);
        go.forwardInches(36, .5, true);
        go.turnDegree(-45, .25, true);
        go.stop(100);
        go.moveArm(-8200, 1, true);
        go.stop(100);
        go.forwardInches(4.5, .5, true);
        go.stop(100);
        go.openClaw();
        // 3, Moves back from the Junction, then moves to the stack of cones, getting as many as it can while delivering it
        go.moveArm(7700, 1, true);
        go.forwardInches(-4.5, .5, true);
        go.turnDegree(-45, .25, true);
        go.forwardInches(24, .5, true);
        go.turnDegree(90, .25, true);
        go.forwardInches(24, .5, true);
        go.turnDegree(90, .25, true);
        go.forwardInches(24, .5, true);
        go.closeClaw();
        go.moveArm(-7700, 1, true);
        go.turnDegree(180, .25, true);
        go.forwardInches(24, .5, true);
        go.turnDegree(-45, .25, true);
        go.forwardInches(4.5, .5, true);
        go.openClaw();
        go.forwardInches(-4.5, .5, true);
        go.moveArm(8700, 1, true);
        go.turnDegree(-45, .25, true);
        go.forwardInches(48, .5, true);
        if(coneNumber == 1){
            go.strafeInches(24, .5, true);
        }
        if(coneNumber == 3) {
            go.strafeInches(-24, .5, true);
        }
    }
}
    
