package frc.robot.vision;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.swerve.CommandSwerveDrivetrain;
import redrocklib.logging.SmartDashboardNumber;

public class Localization {
    public static final Pose2d redCliffPose = new Pose2d(4,0.25, new Rotation2d());
    public static final Pose2d blueCliffPose = new Pose2d(4,0.25, new Rotation2d());
    // public static final Pose2d redCliffPose = new Pose2d(15.468,2.321, new Rotation2d());
    // public static final Pose2d blueCliffPose = new Pose2d(15.468,5.604, new Rotation2d());
    public static final Pose2d turretOffset = new Pose2d(0.177, 0.190, new Rotation2d());

    private static int[] validIDs = {1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14};
    private static String[] limeLightNames = {"left", "right"};
    private static double[][] limeLightStdvs = {
        {0.8, 0.8, 9999},
        {0.8, 0.8, 9999}
    };

    private static LimeLightPoseEstimateWrapper[] wrappers;

    private static SmartDashboardNumber kStdvDemoninator = new SmartDashboardNumber("localization/stdv-denom-scale", 30);
    private static SmartDashboardNumber heading = new SmartDashboardNumber("localization/heading", 0);

    public static void initialize() {
        if(wrappers != null)
            return;
        wrappers = new LimeLightPoseEstimateWrapper[limeLightNames.length];
        for (int i = 0; i < limeLightNames.length; i++) {
            wrappers[i] = new LimeLightPoseEstimateWrapper().withName(limeLightNames[i]);
            LimelightHelpers.SetFiducialIDFiltersOverride(limeLightNames[i], validIDs);
        }
    }

    public static LimeLightPoseEstimateWrapper[] getPoseEstimates(double headingDegrees) {
        heading.putNumber(headingDegrees);
        if(wrappers == null)
            initialize();
        for(int i = 0; i < limeLightNames.length; i++){
            String s = "limelight-" + limeLightNames[i];
            SmartDashboard.putNumber("localization/"+s+"/heading", headingDegrees);
            LimelightHelpers.SetRobotOrientation(s, headingDegrees, CommandSwerveDrivetrain.getInstance().getRotationRateDegrees(), 0, 0, 0, 0);
            wrappers[i].withPoseEstimate(LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(s))
                        .withTagInVision(LimelightHelpers.getTV(s));
        }
        SmartDashboard.putNumber("localization/pose/x", getPose2d().getX());
        SmartDashboard.putNumber("localization/pose/y", getPose2d().getY());
        SmartDashboard.putNumber("localization/pose/heading", getPose2d().getRotation().getDegrees());

        return wrappers;
    }

    public static double getDistanceToTargetRed() {
        return Math.hypot(getTurretPose2d().getX() - redCliffPose.getX(), getTurretPose2d().getY() - redCliffPose.getY());
    }

    public static double getDistanceToTargetBlue() {
        return Math.hypot(getTurretPose2d().getX() - blueCliffPose.getX(), getTurretPose2d().getY() - blueCliffPose.getY());
    }

    public static Pose2d getPose2d() {
        return CommandSwerveDrivetrain.getInstance().getPose();
    }

    /**
     * Finds the turret's field relative position
     * @return a <code>Pose2d</code> which represents the locaiton of the turret.
     */
    public static Pose2d getTurretPose2d() {
        Pose2d botPose = getPose2d();
        double theta = botPose.getRotation().getRadians();
        // Add matrix transform to robot pose 
        return new Pose2d(
            botPose.getX() + turretOffset.getX()*Math.cos(theta) - turretOffset.getY()*Math.sin(theta),
            botPose.getY() + turretOffset.getX()*Math.sin(theta) + turretOffset.getY()*Math.cos(theta),
            new Rotation2d()
        );
    }

    public static Rotation2d getAngleToRed() {
        return Rotation2d.fromRadians(
            Math.atan2(redCliffPose.getY() - getTurretPose2d().getY(), redCliffPose.getX() - getTurretPose2d().getX())
        );
    }

    public static Rotation2d getAngleToBlue() {
        return Rotation2d.fromRadians(
            Math.atan2(blueCliffPose.getY() - getTurretPose2d().getY(), blueCliffPose.getX() - getTurretPose2d().getX())
        );
    }

    public static class LimeLightPoseEstimateWrapper {
        public LimelightHelpers.PoseEstimate poseEstimate;
        public String name;
        public boolean tiv;
        private SmartDashboardNumber[] kStdvs = new SmartDashboardNumber[3];
        public Field2d field = new Field2d();

        public Matrix<N3, N1> getStdvs(double distanceToTarget) {
            return VecBuilder.fill(
                adjustStdv(kStdvs[0].getNumber(), distanceToTarget),
                adjustStdv(kStdvs[1].getNumber(), distanceToTarget),
                adjustStdv(kStdvs[2].getNumber(), distanceToTarget)
            );
        }

        public LimeLightPoseEstimateWrapper withName(String name) {
            this.name = name;
            double[] stdvDefVals = new double[] {0.8, 0.8, 9999};
            for (int i = 0; i < Localization.limeLightNames.length; i++) {
                if (Localization.limeLightNames[i].equals(name)) {
                    stdvDefVals = limeLightStdvs[i];
                    break;
                }
            }

            kStdvs[0] = new SmartDashboardNumber(this.name + "/" + this.name + "-stdvX", stdvDefVals[0]);
            kStdvs[1] = new SmartDashboardNumber(this.name + "/" + this.name + "-stdvY", stdvDefVals[1]);
            kStdvs[2] = new SmartDashboardNumber(this.name + "/" + this.name + "-stdvTheta", stdvDefVals[2]);

            SmartDashboard.putData(this.name + "/" + this.name + "field", this.field);

            return this;
        }

        public LimeLightPoseEstimateWrapper withPoseEstimate(LimelightHelpers.PoseEstimate estimate) {
            this.poseEstimate = estimate;
            return this;
        }

        public LimeLightPoseEstimateWrapper withTagInVision(boolean b) {
            this.tiv = b;
            SmartDashboard.putBoolean(this.name + "/" + this.name + "-tag-in-vision", b);
            return this;
        }

        private double adjustStdv(double stdv, double distanceToTarget) {
            return stdv + stdv * (distanceToTarget * distanceToTarget) / Localization.kStdvDemoninator.getNumber();
        }
    }
}
