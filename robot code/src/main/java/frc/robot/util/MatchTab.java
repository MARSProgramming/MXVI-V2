package frc.robot.util;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.HttpCamera;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.ComplexWidget;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.MiniSystems.Elevator;
import frc.robot.subsystems.MiniSystems.Grasper;
import frc.robot.subsystems.MiniSystems.Pivot;
import frc.robot.subsystems.MiniSystems.Wrist;


public class MatchTab extends SubsystemBase {

      DrivetrainSubsystem mDrivetrainSubsystem;
      Elevator mElevator;
      Grasper mGrasper;
      Pivot mPivot;
      Wrist mWrist;

      private ShuffleboardTab Match = Shuffleboard.getTab("Match");
      private ShuffleboardTab TestInfo = Shuffleboard.getTab("Test");
      private ShuffleboardLayout PilotControlsList = Match.getLayout("Pilot Controls", BuiltInLayouts.kList).withSize(2,5).withPosition(6,0);
      private ShuffleboardLayout CopilotControlsList = Match.getLayout("Copilot Controls", BuiltInLayouts.kList).withSize(2,5).withPosition(8,0);
      private ShuffleboardLayout TestPilotControlsList = TestInfo.getLayout("Pilot Controls [TEST]", BuiltInLayouts.kList).withSize(2,5).withPosition(6,0);

      private ShuffleboardLayout LEDLegend = Match.getLayout("LED Legend", BuiltInLayouts.kList).withSize(2,5).withPosition(10,0);
      // pose information from Drivetrain
      private GenericEntry XPos =  Match.add("Robot X Position", 0).withSize(2,1).withPosition(0, 4).getEntry();
      private GenericEntry YPos =  Match.add("Robot Y Position", 0).withSize(2,1).withPosition(2, 4).getEntry();
      private GenericEntry Rotation = Match.add("Robot Rotation", 0).withSize(2,1).withPosition(0, 5).getEntry();
      private GenericEntry Pigeon =  Match.add("Robot Pigeon Angle", 0).withSize(2,1).withPosition(2, 5).getEntry();


      // ////////////////
      // TEST/SUBSYSTEM INFORMATION
      // ////////////////
    
      // Elevator Position and Velocity
      private GenericEntry ElevatorVelo =  TestInfo.add("Elevator Velocity", 0).withSize(2,1).withPosition(0, 0).getEntry();
      private GenericEntry ElevatorPos = TestInfo.add("Elevator Position", 0).withSize(2, 1).withPosition(0, 1).getEntry(); 
      
      // Grasper Position and Velocity
      private GenericEntry GrasperVelo = TestInfo.add("Grasper Velocity", 0).withSize(2, 1).withPosition(4,0).getEntry();
      private GenericEntry GrasperPos = TestInfo.add("Grasper Position", 0).withSize(2,1).withPosition(4, 1).getEntry();
      
      // Pivot Position (no velocity yet)
      private GenericEntry PivotPos = TestInfo.add("Pivot Position", 0).withSize(2,1).withPosition(0, 3).getEntry();

      // Wrist Position and Velocity
      private GenericEntry WristVelo = TestInfo.add("Wrist Velocity", 0).withSize(2,1).withPosition(4, 3).getEntry();
      private GenericEntry WristPos = TestInfo.add("Wrist Position", 0).withSize(2,1).withPosition(4, 4).getEntry();

      private ComplexWidget commandScheduler = Match.add("CommandScheduling", CommandScheduler.getInstance()).withSize(2,2).withPosition(4, 3);
      public MatchTab(DrivetrainSubsystem drivetrain, Elevator elevator, Grasper grasper, Pivot pivot, Wrist wrist) {
        mDrivetrainSubsystem = drivetrain;
        mElevator = elevator;
        mGrasper = grasper;
        mPivot = pivot;
        mWrist = wrist;

        // Test Controls: Pilot
        TestPilotControlsList.addString("MANUAL Elevator Down", () -> "Left Trigger");
        TestPilotControlsList.addString("MANUAL Elevator Up", () -> "Right Trigger");
        TestPilotControlsList.addString("Run Grasper Intake", () -> "D-Pad Up");
        TestPilotControlsList.addString("Pivot Back", () -> "Left Bumper");
        TestPilotControlsList.addString("Pivot Forward", () -> "Right Bumper");
        TestPilotControlsList.addString("Wrist Forward", () -> "D-Pad Right");
        TestPilotControlsList.addString("Wrist Backward", () -> "D-Pad Left"); 

    }

    public void configureDashboard() {

        
        // Create camera
        HttpCamera httpCamera = new HttpCamera("Cameras", "http://limelight.local:5800/stream.mjpg");
        CameraServer.addCamera(httpCamera);
        Shuffleboard.getTab("Tab").add(httpCamera);
        
        // Pilot Controls
      
        PilotControlsList.addString("Drive Translation", () -> "Left Joystick X/Y");
        PilotControlsList.addString("Drive Rotation", () -> "Right Joystick X");
        PilotControlsList.addString("Run Grasper Outtake", () -> "Right Trigger");
        PilotControlsList.addString("Run Grasper Hold Cube", () -> "Right Bumper");
        PilotControlsList.addString("Run Grasper Intake", () -> "Left Bumper");
        PilotControlsList.addString("Align to Score", () -> "X Button");
        PilotControlsList.addString("Zero Yaw", () -> "Y Button"); 
        // Copilot Controls

        CopilotControlsList.addString("Setpoint Shoot Cube High", () -> "Right Joystick Push");
        CopilotControlsList.addString("Setpoint Shoot Mid", () -> "Left Joystick Push");
        CopilotControlsList.addString("Pivot Back", () -> "Left Bumper");
        CopilotControlsList.addString("Pivot Forward", () -> "Left Trigger");
        CopilotControlsList.addString("Run Elevator Up", () -> "Right Trigger");
        CopilotControlsList.addString("Setpoint Tilted Cone Intake", () -> "X Button");
        CopilotControlsList.addString("Setpoint Upright Cone Intake", () -> "Y Button");
        CopilotControlsList.addString("Setpoint Score Mid", () -> "A Button");
        CopilotControlsList.addString("Setpoint Cube Intake", () -> "B Button");
        CopilotControlsList.addString("HP Station Setpoint", () -> "D-Pad Up");
        CopilotControlsList.addString("Setpoint Score High", () -> "D-Pad Down");
        CopilotControlsList.addString("Run Wrist", () -> "D-Pad Right/Left");
        
        
        // LED Legend

        LEDLegend.addString("Auto", () -> "RAINBOW");
        LEDLegend.addString("TeleOp", () -> "RED");
        LEDLegend.addString("Cone", () -> "YELLOW");
        LEDLegend.addString("Cone Lined Up", () -> "FLASHING YELLOW");
        LEDLegend.addString("Cube", () -> "PURPLE");
        LEDLegend.addString("Cube Lined Up", () -> "FLASHING PURPLE");


      }
    

    @Override
    public void periodic() {
      // adds live information from Drivetrain to match tab
      double XPoseValue = mDrivetrainSubsystem.getPose().getX();
      double YPoseValue = mDrivetrainSubsystem.getPose().getX();
      double RotationValue = mDrivetrainSubsystem.getPose().getRotation().getDegrees();
      double PigeonValue = Math.toDegrees(mDrivetrainSubsystem.getPigeonAngle());

      // adds live information from elevator to match tab
      double ElevatorVelocity = mElevator.getElevatorVelocity();
      double ElevatorPosition = mElevator.getElevatorPosition();

      // adds live information from grasper to match tab
      double GrasperVelocity = mGrasper.getGrasperVelocity();
      double GrasperPosition = mGrasper.getGrasperPosition();

      // Add live information from pivot to match tab
      double PivotPosition = mPivot.getEncoderPos();
      
      // Add live information from wrist to match tab
      double WristVelocity = mWrist.getWristVelocity();
      double WristPosition = mWrist.getWristPosition();
      
       // set all LiveValues
        XPos.setDouble(XPoseValue);
        YPos.setDouble(YPoseValue);
        Rotation.setDouble(RotationValue);
        Pigeon.setDouble(PigeonValue);

        ElevatorVelo.setDouble(ElevatorVelocity);
        ElevatorPos.setDouble(ElevatorPosition);

        GrasperVelo.setDouble(GrasperVelocity);
        GrasperPos.setDouble(GrasperPosition);

        PivotPos.setDouble(PivotPosition);

        WristVelo.setDouble(WristVelocity);
        WristPos.setDouble(WristPosition);

    }

}
