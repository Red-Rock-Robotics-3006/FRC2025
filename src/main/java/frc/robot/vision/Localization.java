package frc.robot.vision;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.Utils;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.swerve.CommandSwerveDrivetrain;
import frc.robot.vision.LimelightHelpers.PoseEstimate;
import frc.robot.vision.LimelightHelpers.RawFiducial;
import redrocklib.logging.SmartDashboardNumber;

public class Localization {
    private static boolean isSim = false;

    private static int[] validIDs = {6, 7, 8, 9, 10, 11, 17, 18, 19, 20, 21, 22};
    private static String[] limeLightNames = {"climb", "front"};//, "right", "back"};
    private static double[][] limeLightStdvs = {
        // {0.8, 0.8, 9999},
        {0.9, 0.9, 9999},
        {1.2, 1.2, 9999}
    };

    // public static final double timeOf/fset = Utils.getCurrentTimeSeconds();
    private static SmartDashboardNumber timeOffset = new SmartDashboardNumber("localization/timeoffset", Utils.getCurrentTimeSeconds());

    private static LimeLightPoseEstimateWrapper[] wrappers;

    private static SmartDashboardNumber kStdvDemoninator = new SmartDashboardNumber("localization/stdv-denom-scale", 30);
    private static SmartDashboardNumber heading = new SmartDashboardNumber("localization/heading", 0);

    public static void initialize() {
        if(wrappers != null)
            return;
        wrappers = new LimeLightPoseEstimateWrapper[limeLightNames.length]; // TODO revert on ll addition
        for (int i = 0; i < limeLightNames.length; i++) { // TODO revert on ll addition
            wrappers[i] = new LimeLightPoseEstimateWrapper().withName(limeLightNames[i]);
            LimelightHelpers.SetFiducialIDFiltersOverride(limeLightNames[i], validIDs);
        }
        isSim = Utils.isSimulation();
    }

    public static void updateHeading(double headingDegrees) {
        heading.putNumber(headingDegrees);
        if(wrappers == null)
            initialize();
        for(int i = 0; i < limeLightNames.length; i++){
            String name = "limelight-" + limeLightNames[i];
            SmartDashboard.putNumber("localization/"+name+"/heading", headingDegrees);
            LimelightHelpers.SetRobotOrientation(name, headingDegrees, CommandSwerveDrivetrain.getInstance().getRotationRateDegrees(), 0, 0, 0, 0);
            PoseEstimate estimate = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(name);
            if (estimate != null){ // TODO revert on ll addition
                Logger.recordOutput("limelight/" + limeLightNames[i] + "/avgTagDist", estimate.avgTagDist);
                double ambiguitySum = 0;
                for (RawFiducial fiducial : estimate.rawFiducials)
                    ambiguitySum += fiducial.ambiguity;
                wrappers[i].withPoseEstimate(estimate).withTagInVision(LimelightHelpers.getTV(name)).withAmbiguity(ambiguitySum/estimate.rawFiducials.length);
            }

            Pose2d mt1Pose = LimelightHelpers.getBotPose2d_wpiBlue(name);
            Logger.recordOutput("limelight/" + limeLightNames[i] + "/MT1", mt1Pose==null?new Pose2d():mt1Pose);
            Logger.recordOutput("limelight/" + limeLightNames[i] + "/MT2", estimate==null?new Pose2d():estimate.pose);
        }

        Pose2d pose = getPose2d();
        SmartDashboard.putNumber("localization/pose/x", pose.getX());
        SmartDashboard.putNumber("localization/pose/y", pose.getY());
        SmartDashboard.putNumber("localization/pose/heading", pose.getRotation().getDegrees());
    }

    public static LimeLightPoseEstimateWrapper[] getPoseEstimates() {
        // heading.putNumber(headingDegrees);
        // if(wrappers == null)
        //     initialize();
        // for(int i = 0; i < limeLightNames.length; i++){
        //     String s = "limelight-" + limeLightNames[i];
        //     SmartDashboard.putNumber("localization/"+s+"/heading", headingDegrees);
        //     LimelightHelpers.SetRobotOrientation(s, headingDegrees, CommandSwerveDrivetrain.getInstance().getRotationRateDegrees(), 0, 0, 0, 0);
        //     wrappers[i].withPoseEstimate(LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(s))
        //                 .withTagInVision(LimelightHelpers.getTV(s));
        // }

        // Pose2d pose = getPose2d();
        // SmartDashboard.putNumber("localization/pose/x", pose.getX());
        // SmartDashboard.putNumber("localization/pose/y", pose.getY());
        // SmartDashboard.putNumber("localization/pose/heading", pose.getRotation().getDegrees());

        return wrappers;
    }

    public static Pose2d getPose2d() {
        return CommandSwerveDrivetrain.getInstance().getPose();
    }

    public static double getMegatag1Pose2dFromClimb() {
        // if (LimelightHelpers.getTV(limeLightNames[1])) {
        //     SmartDashboard.putBoolean("locaization/mt1-tiv", true);
        // if (isSim) return 0;
        // return 0;
        return LimelightHelpers.getBotPoseEstimate_wpiBlue("limelight-" + limeLightNames[0]).pose.getRotation().getDegrees();
        // }
        // SmartDashboard.putBoolean("locaization/mt1-tiv", false);
        // return new Pose2d();
    }

    public static class LimeLightPoseEstimateWrapper {
        public LimelightHelpers.PoseEstimate poseEstimate = new PoseEstimate();
        public String name;
        public boolean tiv;
        private SmartDashboardNumber[] kStdvs = new SmartDashboardNumber[3];
        public Field2d field = new Field2d();
        private double ambiguity;

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

            SmartDashboard.putData(this.name + "/" + this.name + "-field", this.field);

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

        public LimeLightPoseEstimateWrapper withAmbiguity(double ambiguity) {
            this.ambiguity = ambiguity;
            Logger.recordOutput("limelight/" + this.name + "/ambiguity", ambiguity);
            return this;
        }

        private double adjustStdv(double stdv, double distanceToTarget) {
            return stdv + stdv * (distanceToTarget * distanceToTarget) / Localization.kStdvDemoninator.getNumber();
        }
    }
}
