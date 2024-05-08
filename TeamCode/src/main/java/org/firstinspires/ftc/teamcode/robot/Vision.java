package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.hardware.HardwareMap;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;

import org.firstinspires.ftc.robotcore.external.JavaUtil;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.utils.Constants;
import org.firstinspires.ftc.teamcode.utils.MathUtilities;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

//TODO: exposure tuning? (srcpy)

/**
 * The Vision Subsystem of the Robot
 */
public class Vision implements Constants {

    VisionPortal.Builder visionPortalBuilder;
    int FRONT_CAM_VIEW_ID;
    int BACK_CAM_VIEW_ID;
    AprilTagProcessor frontAprilTagProcessor;
    AprilTagProcessor backAprilTagProcessor;
    VisionPortal frontVisionPortal;
    VisionPortal backVisionPortal;
    ArrayList<AprilTagDetection> frontDetections;
    ArrayList<AprilTagDetection> backDetections;

    /**
     * Instantiates the Vision Subsystem
     *
     * @param hwMap the hardware map
     */
    public Vision(HardwareMap hwMap) {
        List<Integer> myPortalsList;
        myPortalsList = JavaUtil.makeIntegerList(VisionPortal.makeMultiPortalView(2, VisionPortal.MultiPortalLayout.HORIZONTAL));
        FRONT_CAM_VIEW_ID = (Integer) JavaUtil.inListGet(myPortalsList, JavaUtil.AtMode.FROM_START, 0, false);
        BACK_CAM_VIEW_ID = (Integer) JavaUtil.inListGet(myPortalsList, JavaUtil.AtMode.FROM_START, 1, false);

        AprilTagProcessor.Builder myAprilTagProcessorBuilder = new AprilTagProcessor.Builder();
        frontAprilTagProcessor = myAprilTagProcessorBuilder.setOutputUnits(DistanceUnit.INCH, AngleUnit.DEGREES).build();
        backAprilTagProcessor = myAprilTagProcessorBuilder.setOutputUnits(DistanceUnit.INCH, AngleUnit.DEGREES).build();

        visionPortalBuilder = new VisionPortal.Builder();
        visionPortalBuilder.setCamera(hwMap.get(WebcamName.class, "Webcam 1"));
        visionPortalBuilder.setStreamFormat(VisionPortal.StreamFormat.YUY2);
        visionPortalBuilder.addProcessor(frontAprilTagProcessor);
        visionPortalBuilder.setLiveViewContainerId(FRONT_CAM_VIEW_ID);
        frontVisionPortal = visionPortalBuilder.build();

        visionPortalBuilder = new VisionPortal.Builder();
        visionPortalBuilder.setCamera(hwMap.get(WebcamName.class, "Webcam 2"));
        visionPortalBuilder.setStreamFormat(VisionPortal.StreamFormat.YUY2);
        visionPortalBuilder.addProcessor(backAprilTagProcessor);
        visionPortalBuilder.setLiveViewContainerId(BACK_CAM_VIEW_ID);
        backVisionPortal = visionPortalBuilder.build();

        frontDetections = new ArrayList<>();
        backDetections = new ArrayList<>();
    }

    /**
     * Updates the Robot Vision, call in each loop
     *
     * @return the robot pose [x, y, theta] in inches and degrees
     */
    public double[] update() {
        frontDetections = frontAprilTagProcessor.getDetections();
        backDetections = backAprilTagProcessor.getDetections();

        double[] averagedPose = new double[4];

        int frontSize = frontDetections.size();
        for(int i = 0; i < frontSize; i++) {
            double[] pose = localize(i, true);
            averagedPose[0] += pose[0];
            averagedPose[1] += pose[1];
            averagedPose[2] += pose[3] == 1 ? 5 * Math.cos( Math.toRadians(pose[2]) ) : Math.cos( Math.toRadians(pose[2]) );
            averagedPose[3] += pose[3] == 1 ? 5 * Math.sin( Math.toRadians(pose[2]) ) : Math.sin( Math.toRadians(pose[2]) );
        }

        int backSize = backDetections.size();
        for(int i = 0; i < backSize; i++) {
            double[] pose = localize(i, false);
            averagedPose[0] += pose[0];
            averagedPose[1] += pose[1];
            averagedPose[2] += pose[3] == 1.0 ? 5 * Math.cos( Math.toRadians(pose[2]) ) : Math.cos( Math.toRadians(pose[2]) );
            averagedPose[3] += pose[3] == 1.0 ? 5 * Math.sin( Math.toRadians(pose[2]) ) : Math.sin( Math.toRadians(pose[2]) );
        }

        if(frontSize + backSize == 0)
            return null;

        averagedPose[0] /= (frontSize + backSize);
        averagedPose[1] /= (frontSize + backSize);

        double heading = MathUtilities.addAngles(Math.toDegrees( Math.atan2(averagedPose[3],averagedPose[2]) ), 0.0);
        return new double[]{averagedPose[0], averagedPose[1], heading};
    }

    private double[] localize(int i, boolean isFrontCam) {
        AprilTagDetection aprilTagDetection = isFrontCam ? frontDetections.get(i) : backDetections.get(i);

        int id = aprilTagDetection.id;
        double range = aprilTagDetection.ftcPose.range;
        double yaw = aprilTagDetection.ftcPose.yaw;
        double bearing = aprilTagDetection.ftcPose.bearing;
        boolean isIntakeTag = (id >= 7 && id <= 10);

        double tagX = isIntakeTag ? APRIL_TAG_INTAKE_X : APRIL_TAG_SCORING_X;
        double tagY = getTagYCoordinate(id);

        double fieldHeading = isFrontCam == isIntakeTag ? -yaw : MathUtilities.addAngles(-yaw, -180.0);

        double cameraDeltaX = range * Math.cos(Math.toRadians(bearing - yaw));
        double cameraDeltaY = range * Math.sin(Math.toRadians(bearing - yaw));

        double cameraX = isIntakeTag ? tagX - cameraDeltaX : tagX + cameraDeltaX;
        double cameraY = isIntakeTag ? tagY - cameraDeltaY : tagY + cameraDeltaY;

        double fieldHeadingInRadians = Math.toRadians(fieldHeading);

        double localizedX;
        double localizedY;
        if(isFrontCam) {
            localizedX = cameraX - FRONT_CAM_FORWARD_DIST * Math.cos(fieldHeadingInRadians)
                    + FRONT_CAM_HORIZONTAL_DIST * Math.sin(fieldHeadingInRadians);
            localizedY = cameraY - FRONT_CAM_HORIZONTAL_DIST * Math.cos(fieldHeadingInRadians)
                    - FRONT_CAM_FORWARD_DIST * Math.sin(fieldHeadingInRadians);
        }
        else {
            localizedX = cameraX + BACK_CAM_DIST * Math.cos(fieldHeadingInRadians);
            localizedY = cameraY + BACK_CAM_DIST * Math.sin(fieldHeadingInRadians);
        }

        return new double[] {localizedX, localizedY, fieldHeading, id == 7 || id == 10 ? 1.0 : 0.0};
    }

    private double getTagYCoordinate(int id) {
        switch(id) {
            case 1:
                return APRIL_TAG_1_Y;
            case 2:
                return APRIL_TAG_2_Y;
            case 3:
                return APRIL_TAG_3_Y;
            case 4:
                return APRIL_TAG_4_Y;
            case 5:
                return APRIL_TAG_5_Y;
            case 6:
                return APRIL_TAG_6_Y;
            case 7:
                return APRIL_TAG_7_Y;
            case 8:
                return APRIL_TAG_8_Y;
            case 9:
                return APRIL_TAG_9_Y;
            default:
                return APRIL_TAG_10_Y;
        }
    }

    /**
     * Get the Individual April Tag Localization Headings for telemetry
     * Brady did this with a HashMap (he's addicted)
     *
     * @return a hashMap of the tag id's and headings
     */
    public HashMap<Integer, Double[]> getHeadings() {
        HashMap<Integer, Double[]> headings = new HashMap<>();
        for( int i = 0; i < frontDetections.size(); i++ ) {
            int id = frontDetections.get(i).id;
            double yaw = frontDetections.get(i).ftcPose.yaw;
            yaw = (id >= 7 && id <= 10) ? -yaw : MathUtilities.addAngles(-yaw, -180.0);
            headings.put( id, new Double[] { yaw, frontDetections.get(i).ftcPose.y } );
        }

        for( int i = 0; i < backDetections.size(); i++ ) {
            int id = backDetections.get(i).id;
            double yaw = backDetections.get(i).ftcPose.yaw;
            yaw = !(id >= 7 && id <= 10) ? -yaw : MathUtilities.addAngles(-yaw, -180.0);
            headings.put( id, new Double[] { yaw, backDetections.get(i).ftcPose.y } );
        }
        return headings;
    }

    /**
     * Returns the number of Total April Tag Detections
     *
     * @return the number of detections
     */
    public int getNumDetections() {
        return frontDetections.size() + backDetections.size();
    }

    /**
     * Returns the number of April Tag Detections fom the Front Camera
     *
     * @return the number of detections
     */
    public int getNumFrontDetections() {
        return frontDetections.size();
    }

    /**
     * Returns the number of April Tag Detections from the Back Camera
     *
     * @return the number of detections
     */
    public int getNumBackDetections() {
        return backDetections.size();
    }
}
