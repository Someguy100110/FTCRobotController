package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvInternalCamera;
import org.openftc.easyopencv.OpenCvCameraRotation;


@TeleOp()
public class OpenCVTestOpMode extends LinearOpMode {
    ConeDetectionPipeline detectionPipeline;

    public void runOpMode() {
        telemetry.addData("Status", "waiting for start");
        telemetry.update();

        detectionPipeline = new ConeDetectionPipeline();

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        OpenCvCamera camera = OpenCvCameraFactory.getInstance().createInternalCamera(OpenCvInternalCamera.CameraDirection.BACK, cameraMonitorViewId);

        camera.openCameraDevice();
        camera.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);

        camera.setPipeline(detectionPipeline);

        //int detected_position = 0;

        //while ((detected_position = detectionPipeline.getLatestResult()) == 0) {}

        waitForStart();

        telemetry.addData("Status", "Running");
        telemetry.update();

        while (opModeIsActive()) {

            telemetry.addData("Result", detectionPipeline.getLatestResult());
            telemetry.addData("Red", detectionPipeline.getRGBA()[0]);
            telemetry.addData("Green", detectionPipeline.getRGBA()[1]);
            telemetry.addData("Blue", detectionPipeline.getRGBA()[2]);
            telemetry.update();



            sleep(3000);
        }
    }
}
