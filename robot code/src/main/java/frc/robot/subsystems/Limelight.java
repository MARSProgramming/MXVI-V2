package frc.robot.subsystems;

import java.sql.Driver;

import edu.wpi.first.math.MatBuilder;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Limelight extends SubsystemBase{
    double[] botpose;
    private DrivetrainSubsystem dt;
    private Timer mTimerLeft = new Timer();
    private Timer mTimerRight = new Timer();
    public Limelight(DrivetrainSubsystem drive){
        dt = drive;
        mTimerLeft.reset();
        mTimerRight.reset();
    }

    public boolean hasTarget(){
        double leftTV = NetworkTableInstance.getDefault().getTable("limelight-left").getEntry("tv").getDouble(0);
        double rightTV = NetworkTableInstance.getDefault().getTable("limelight-right").getEntry("tv").getDouble(0);
        return leftTV == 1.0 || rightTV == 1.0;
    }

    public void resetPose(){
        if(DriverStation.isDisabled()){
            mTimerLeft.reset();
            mTimerRight.reset();
        }
        double distance = DriverStation.isTeleop() ? 3.5 : 1.2;
        double leftTV = NetworkTableInstance.getDefault().getTable("limelight-left").getEntry("tv").getDouble(0);
        double rightTV = NetworkTableInstance.getDefault().getTable("limelight-right").getEntry("tv").getDouble(0);
        double leftDist = NetworkTableInstance.getDefault().getTable("limelight-left").getEntry("targetpose_cameraspace").getDoubleArray(new double[7])[2];
        double rightDist = NetworkTableInstance.getDefault().getTable("limelight-right").getEntry("targetpose_cameraspace").getDoubleArray(new double[7])[2];
        boolean resetLeft = ((leftTV == 1.0 && rightTV == 0.0) || (leftTV == 1.0 && leftDist < rightDist)) && leftDist < distance;
        boolean resetRight = ((leftTV == 0.0 && rightTV == 1.0) || (rightTV == 1.0 && rightDist < leftDist)) && rightDist < distance;
        
        if(resetLeft){
            if(DriverStation.isAutonomous()){
                dt.setVisionMeasurementStdDevs(new MatBuilder<>(Nat.N3(), Nat.N1()).fill(0.03, 0.03, 0.07));
            }
            else if(dt.getPose().getX() < 3.4){
                dt.setVisionMeasurementStdDevs(new MatBuilder<>(Nat.N3(), Nat.N1()).fill(0.01, 0.01, 0.07));
            }
            String key = DriverStation.getAlliance() == Alliance.Blue ? "botpose_wpiblue" : "botpose_wpired";
            botpose = NetworkTableInstance.getDefault().getTable("limelight-left").getEntry(key).getDoubleArray(new double[7]);
            double x = botpose[0];
            double y = botpose[1];
            double z = botpose[2];

            if(dt.getPose().getTranslation().getDistance(new Translation2d(x, y)) < 1 || DriverStation.isTeleop() || DriverStation.isDisabled()){
                if(!mTimerLeft.hasElapsed(0.3) || dt.getPose().getTranslation().getDistance(new Translation2d(x, y)) < 0.3){
                    dt.addVisionMeasurement(new Pose2d(x, y, dt.getGyroscopeRotation()), botpose[6]/1000);
                    mTimerLeft.start();
                }
            }
        }
        else{
            mTimerLeft.stop();
            mTimerLeft.reset();
        }

        if(resetRight){
            if(dt.getPose().getX() > 13){
                dt.setVisionMeasurementStdDevs(new MatBuilder<>(Nat.N3(), Nat.N1()).fill(0.03, 0.1, 0.07));
            }
            if(dt.getPose().getX() < 3.4){
                dt.setVisionMeasurementStdDevs(new MatBuilder<>(Nat.N3(), Nat.N1()).fill(0.01, 0.01, 0.07));
            }
            String key = DriverStation.getAlliance() == Alliance.Blue ? "botpose_wpiblue" : "botpose_wpired";
            botpose = NetworkTableInstance.getDefault().getTable("limelight-right").getEntry(key).getDoubleArray(new double[7]);
            double x = botpose[0];
            double y = botpose[1];
            double z = botpose[2];

            if(dt.getPose().getTranslation().getDistance(new Translation2d(x, y)) < 1 || DriverStation.isTeleop() || DriverStation.isDisabled()){
                if(!mTimerRight.hasElapsed(0.3) || dt.getPose().getTranslation().getDistance(new Translation2d(x, y)) < 0.3){
                    dt.addVisionMeasurement(new Pose2d(x, y, dt.getGyroscopeRotation()), botpose[6]/1000);
                    mTimerRight.start();
                }
            }
        }
        else{
            mTimerRight.stop();
            mTimerRight.reset();
        }
    }

    @Override
    public void periodic(){
        resetPose();
    }
}
