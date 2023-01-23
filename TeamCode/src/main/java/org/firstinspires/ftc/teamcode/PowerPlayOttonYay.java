package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;

// @Autonomous(name="Otto Octavious's Greatest Invention")
@Autonomous(name="! Color Test Otton 2 !")
public class PowerPlayOttonYay extends LinearOpMode{
    ConeDetectionPipeline detectionPipeline;
    int coneNumber = 0;
    @Override
    public void runOpMode(){
        DirectionAS go = new DirectionAS(this);
        detectionPipeline = new ConeDetectionPipeline();

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        OpenCvCamera camera = OpenCvCameraFactory.getInstance().createInternalCamera(OpenCvInternalCamera.CameraDirection.BACK, cameraMonitorViewId);

        camera.openCameraDevice();
        camera.startStreaming(320, 240, OpenCvCameraRotation.SIDEWAYS_LEFT);

        camera.setPipeline(detectionPipeline);
        waitForStart();
        go.closeClaw();
        go.stop(100);
        go.forwardInches(-16, .5, true);
        go.stop(100);
        coneNumber = detectionPipeline.getLatestResult();
        go.moveArm(-1500, 1, true);
        go.forwardInches(-8, .5, true);
        if(coneNumber == 1){
            go.strafeInches(25, .5, true);
        }
        else if(coneNumber == 3){
            go.strafeInches(-26, .5, true);
        }
        go.forwardInches(-1, .5, true);
        go.openClaw();
    }
}


    
