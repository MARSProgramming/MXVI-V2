package frc.robot.util;
import java.util.Map;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.ComplexWidget;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.Drive;
import frc.robot.commands.Manipulator.Elevator.ElevatorIntakeHigh;
import frc.robot.commands.Manipulator.Wrist.WristCarry;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.MiniSystems.Elevator;
import frc.robot.subsystems.MiniSystems.Grasper;
import frc.robot.subsystems.MiniSystems.Pivot;
import frc.robot.subsystems.MiniSystems.Wrist;


public class ConstantsTab extends SubsystemBase{
    private ShuffleboardTab Constant = Shuffleboard.getTab("Constants");
    private ShuffleboardLayout DrivetrainConstants = Constant.getLayout("Drivetrain Constants", BuiltInLayouts.kList).withSize(2,5).withPosition(0,0);
    private ShuffleboardLayout ElevatorConstants = Constant.getLayout("Elevator Constants", BuiltInLayouts.kList).withSize(2,5).withPosition(2,0);
    private ShuffleboardLayout WristConstants = Constant.getLayout("Wrist Constants", BuiltInLayouts.kList).withSize(2,5).withPosition(4,0);
    private ShuffleboardLayout GrasperConstants = Constant.getLayout("Grasper Constants", BuiltInLayouts.kList).withSize(2,5).withPosition(6,0);
    private ShuffleboardLayout PivotConstants = Constant.getLayout("Pivot Constants", BuiltInLayouts.kList).withSize(2,5).withPosition(8,0);
    
    private GenericEntry DriveMultiplier;
    private GenericEntry DrivekP; 
    private GenericEntry DrivekI; 
    private GenericEntry DrivekD; 
    private GenericEntry DriveAutoBalanceP; 
    private GenericEntry DriveRRTapeAlignP; 

    private GenericEntry ElevatorForwardLim; 
    private GenericEntry ElevatorReverseLim; 
    private GenericEntry ElevatorkP; 
    private GenericEntry ElevatorkI; 
    private GenericEntry ElevatorkD; 
    private GenericEntry ElevatorPeakForwardOut; 
    private GenericEntry ElevatorPeakReverseOut; 
    private GenericEntry ElevatorIntakeHighPos; 
    private GenericEntry ElevatorIntakePos; 
    private GenericEntry ElevatorBottomPos; 
    private GenericEntry ElevatorScoreHighPos; 
    private GenericEntry ElevatorScoreMidPos; 
    private GenericEntry ElevatorStowPos; 

    private GenericEntry WristkP; 
    private GenericEntry WristkI; 
    private GenericEntry WristkD; 
    private GenericEntry WristForwardLimit; 
    private GenericEntry WristReverseLimit; 
    private GenericEntry WristIntakeBackPos; 
    private GenericEntry WristIntakeUpPos; 
    private GenericEntry WristIntakeCubePos; 
    private GenericEntry WristIntakeScoreHighPos; 
    private GenericEntry WristIntakeScoreMidPos; 
    private GenericEntry WristIntakeScoreLowPos; 
    private GenericEntry WristCarryPos; 
    private GenericEntry WristShootPos; 
    private GenericEntry WristLoadPos; 
    private GenericEntry WristStowPos; 
    private GenericEntry PivotkP;
    private GenericEntry PivotkI; 
    private GenericEntry PivotkD;
    private GenericEntry PivotForwardLimit;  
    private GenericEntry PivotReverseLimit; 
    private GenericEntry PivotIntakeBackPos; 
    private GenericEntry PivotScoreHighPos; 
    private GenericEntry PivotCubePos; 
    private GenericEntry PivotIntakeHighPos; 
    private GenericEntry PivotLoadPos; 
    private GenericEntry PivotLoadDoublePos; 
    private GenericEntry PivotZero; 
    private GenericEntry PivotShootHighPos; 
    private GenericEntry PivotStowPos; 

    private GenericEntry GrasperkP; 
    private GenericEntry GrasperkI; 
    private GenericEntry GrasperkD; 
    private GenericEntry GrasperkF; 

    private GenericEntry LeftYBlue1; 
    private GenericEntry LeftYBlue2;  
    private GenericEntry LeftYBlue3; 

    private GenericEntry MidYBlue1; 
    private GenericEntry MidYBlue2;  
    private GenericEntry MidYBlue3; 

    private GenericEntry RightYBlue1; 
    private GenericEntry RightYBlue2; 
    private GenericEntry RightYBlue3; 
    
    private GenericEntry LeftYRed1;  
    private GenericEntry LeftYRed2; 
    private GenericEntry LeftYRed3; 

    private GenericEntry MidYRed1; 
    private GenericEntry MidYRed2; 
    private GenericEntry MidYRed3; 

    private GenericEntry RightYRed1; 
    private GenericEntry RightYRed2; 
    private GenericEntry RightYRed3; 

    public void configureDashboard(){ 
        //Drive Constants
        DriveMultiplier = DrivetrainConstants.addPersistent("Max Speed Multiplier", Constants.Drive.MAX_SPEED_MULTIPLIER).withWidget(BuiltInWidgets.kTextView).getEntry(); 
        DrivekP = DrivetrainConstants.addPersistent("Drive kP", Constants.Drive.kP).withWidget(BuiltInWidgets.kTextView).getEntry(); 
        DrivekI = DrivetrainConstants.addPersistent("Drive kI", Constants.Drive.kI).withWidget(BuiltInWidgets.kTextView).getEntry(); 
        DrivekD = DrivetrainConstants.addPersistent("Drive kD", Constants.Drive.kD).withWidget(BuiltInWidgets.kTextView).getEntry(); 
        DriveAutoBalanceP = DrivetrainConstants.addPersistent("Auto Balance P", Constants.Drive.autoBalanceP).withWidget(BuiltInWidgets.kTextView).getEntry(); 
        DriveRRTapeAlignP = DrivetrainConstants.addPersistent("RRTape Align P", Constants.Drive.RRTapeAlignP).withWidget(BuiltInWidgets.kTextView).getEntry(); 
        
        LeftYBlue1 = DrivetrainConstants.addPersistent("Left Y Blue Scoring Alignment Pos Val 1", Constants.Drive.leftYBlueScoringPos1).withWidget(BuiltInWidgets.kTextView).getEntry(); 
        LeftYBlue2 = DrivetrainConstants.addPersistent("Left Y Blue Scoring Alignment Pos Val 2", Constants.Drive.leftYBlueScoringPos2).withWidget(BuiltInWidgets.kTextView).getEntry(); 
        LeftYBlue3 = DrivetrainConstants.addPersistent("Left Y Blue Scoring Alignment Pos Val 3", Constants.Drive.leftYBlueScoringPos3).withWidget(BuiltInWidgets.kTextView).getEntry(); 
        
        MidYBlue1 = DrivetrainConstants.addPersistent("Mid Y Blue Scoring Alignment Pos Val 1", Constants.Drive.midYBlueScoringPos1).withWidget(BuiltInWidgets.kTextView).getEntry(); 
        MidYBlue2 = DrivetrainConstants.addPersistent("Mid Y Blue Scoring Alignment Pos Val 2", Constants.Drive.midYBlueScoringPos2).withWidget(BuiltInWidgets.kTextView).getEntry(); 
        MidYBlue3 = DrivetrainConstants.addPersistent("Mid Y Blue Scoring Alignment Pos Val 3", Constants.Drive.midYBlueScoringPos3).withWidget(BuiltInWidgets.kTextView).getEntry(); 
       
        RightYBlue1 = DrivetrainConstants.addPersistent("Right Y Blue Scoring Alignment Pos Val 1", Constants.Drive.rightYBlueScoringPos1).withWidget(BuiltInWidgets.kTextView).getEntry(); 
        RightYBlue2 = DrivetrainConstants.addPersistent("Right Y Blue Scoring Alignment Pos Val 2", Constants.Drive.rightYBlueScoringPos2).withWidget(BuiltInWidgets.kTextView).getEntry(); 
        RightYBlue3 = DrivetrainConstants.addPersistent("Right Y Blue Scoring Alignment Pos Val 3", Constants.Drive.rightYBlueScoringPos3).withWidget(BuiltInWidgets.kTextView).getEntry(); 
        
        LeftYRed1 = DrivetrainConstants.addPersistent("Left Y Red Scoring Alignment Pos Val 1", Constants.Drive.leftYRedScoringPos1).withWidget(BuiltInWidgets.kTextView).getEntry(); 
        LeftYRed2 = DrivetrainConstants.addPersistent("Left Y Red Scoring Alignment Pos Val 2", Constants.Drive.leftYRedScoringPos2).withWidget(BuiltInWidgets.kTextView).getEntry(); 
        LeftYRed3 = DrivetrainConstants.addPersistent("Left Y Red Scoring Alignment Pos Val 3", Constants.Drive.leftYRedScoringPos3).withWidget(BuiltInWidgets.kTextView).getEntry(); 
        
        MidYRed1 = DrivetrainConstants.addPersistent("Mid Y Red Scoring Alignment Pos Val 1", Constants.Drive.midYRedScoringPos1).withWidget(BuiltInWidgets.kTextView).getEntry(); 
        MidYRed2 = DrivetrainConstants.addPersistent("Mid Y Red Scoring Alignment Pos Val 2", Constants.Drive.midYRedScoringPos2).withWidget(BuiltInWidgets.kTextView).getEntry(); 
        MidYRed3 = DrivetrainConstants.addPersistent("Mid Y Red Scoring Alignment Pos Val 3", Constants.Drive.midYRedScoringPos3).withWidget(BuiltInWidgets.kTextView).getEntry(); 
        
        RightYRed1 = DrivetrainConstants.addPersistent("Right Y Red Scoring Alignment Pos Val 1", Constants.Drive.rightYRedScoringPos1).withWidget(BuiltInWidgets.kTextView).getEntry(); 
        RightYRed2 = DrivetrainConstants.addPersistent("Right Y Red Scoring Alignment Pos Val 2", Constants.Drive.rightYRedScoringPos2).withWidget(BuiltInWidgets.kTextView).getEntry(); 
        RightYRed3 = DrivetrainConstants.addPersistent("Right Y Red Scoring Alignment Pos Val 3", Constants.Drive.rightYRedScoringPos3).withWidget(BuiltInWidgets.kTextView).getEntry(); 
        
        //Elevator Constants
        ElevatorForwardLim = ElevatorConstants.addPersistent("Forward Limit (in)", Constants.Elevator.forwardLimitInches).withWidget(BuiltInWidgets.kTextView).getEntry(); 
        ElevatorReverseLim = ElevatorConstants.addPersistent("Reverse Limit (in)", Constants.Elevator.reverseLimitInches).withWidget(BuiltInWidgets.kTextView).getEntry(); 
        ElevatorkP = ElevatorConstants.addPersistent("Elevator kP", Constants.Elevator.kP).withWidget(BuiltInWidgets.kTextView).getEntry(); 
        ElevatorkI = ElevatorConstants.addPersistent("Elevator kI", Constants.Elevator.kI).withWidget(BuiltInWidgets.kTextView).getEntry(); 
        ElevatorkD = ElevatorConstants.addPersistent("Elevator kD", Constants.Elevator.kD).withWidget(BuiltInWidgets.kTextView).getEntry(); 
        ElevatorPeakForwardOut = ElevatorConstants.addPersistent("Peak Forward Output", Constants.Elevator.peakOutForward).withWidget(BuiltInWidgets.kTextView).getEntry(); 
        ElevatorPeakReverseOut = ElevatorConstants.addPersistent("Peak Reverse Output", Constants.Elevator.peakOutReverse).withWidget(BuiltInWidgets.kTextView).getEntry(); 
        ElevatorIntakeHighPos = ElevatorConstants.addPersistent("Intake High Pos", Constants.Elevator.intakeHighPos).withWidget(BuiltInWidgets.kTextView).getEntry(); 
        ElevatorIntakePos = ElevatorConstants.addPersistent("Intake Pos", Constants.Elevator.intakePos).withWidget(BuiltInWidgets.kTextView).getEntry(); 
        ElevatorBottomPos = ElevatorConstants.addPersistent("Bottom Pos", Constants.Elevator.bottomPos).withWidget(BuiltInWidgets.kTextView).getEntry(); 
        ElevatorScoreHighPos = ElevatorConstants.addPersistent("Score High Pos", Constants.Elevator.scoreHighPos).withWidget(BuiltInWidgets.kTextView).getEntry(); 
        ElevatorScoreMidPos = ElevatorConstants.addPersistent("Score Mid Pos", Constants.Elevator.scoreMidPos).withWidget(BuiltInWidgets.kTextView).getEntry(); 
        ElevatorStowPos = ElevatorConstants.addPersistent("Stow Pos", Constants.Elevator.stowPos).withWidget(BuiltInWidgets.kTextView).getEntry(); 

        //Wrist Constants
        WristkP = WristConstants.addPersistent("Wrist kP", Constants.Wrist.kP).withWidget(BuiltInWidgets.kTextView).getEntry(); 
        WristkI= WristConstants.addPersistent("Wrist kI", Constants.Wrist.kI).withWidget(BuiltInWidgets.kTextView).getEntry(); 
        WristkD = WristConstants.addPersistent("Wrist kD", Constants.Wrist.kD).withWidget(BuiltInWidgets.kTextView).getEntry(); 
        WristForwardLimit = WristConstants.addPersistent("Forward Limit (in)", Constants.Wrist.forwardLimit).withWidget(BuiltInWidgets.kTextView).getEntry(); 
        WristReverseLimit = WristConstants.addPersistent("Reverse Limit (in)", Constants.Wrist.reverseLimit).withWidget(BuiltInWidgets.kTextView).getEntry(); 
        WristIntakeBackPos = WristConstants.addPersistent("Intake Back Pos", Constants.Wrist.intakeBackPos).withWidget(BuiltInWidgets.kTextView).getEntry(); 
        WristIntakeUpPos = WristConstants.addPersistent("Intake Up Pos", Constants.Wrist.intakeUpPos).withWidget(BuiltInWidgets.kTextView).getEntry(); 
        WristIntakeCubePos = WristConstants.addPersistent("Intake Cube Pos", Constants.Wrist.intakeCubePos).withWidget(BuiltInWidgets.kTextView).getEntry(); 
        WristIntakeScoreHighPos = WristConstants.addPersistent("Intake Score High Pos", Constants.Wrist.scoreHighPos).withWidget(BuiltInWidgets.kTextView).getEntry(); 
        WristIntakeScoreMidPos = WristConstants.addPersistent("Intake Score Mid Pos", Constants.Wrist.scoreMidPos).withWidget(BuiltInWidgets.kTextView).getEntry(); 
        WristIntakeScoreLowPos = WristConstants.addPersistent("Intake Score Low Pos", Constants.Wrist.scoreLowPos).withWidget(BuiltInWidgets.kTextView).getEntry(); 
        WristCarryPos = WristConstants.addPersistent("Carry pos", Constants.Wrist.carryPos).withWidget(BuiltInWidgets.kTextView).getEntry(); 
        WristShootPos = WristConstants.addPersistent("Shoot pos", Constants.Wrist.shootPos).withWidget(BuiltInWidgets.kTextView).getEntry(); 
        WristLoadPos = WristConstants.addPersistent("Load Pos", Constants.Wrist.loadPos).withWidget(BuiltInWidgets.kTextView).getEntry(); 
        WristStowPos = WristConstants.addPersistent("Stow Pos", Constants.Wrist.stowPos).withWidget(BuiltInWidgets.kTextView).getEntry(); 

        //Pivot Constants
        PivotkP = PivotConstants.addPersistent("Pivot kP", Constants.Pivot.kP).withWidget(BuiltInWidgets.kTextView).getEntry(); 
        PivotkI = PivotConstants.addPersistent("Pivot kI", Constants.Pivot.kI).withWidget(BuiltInWidgets.kTextView).getEntry(); 
        PivotkD = PivotConstants.addPersistent("Pivot kD", Constants.Pivot.kD).withWidget(BuiltInWidgets.kTextView).getEntry(); 
        PivotForwardLimit = PivotConstants.addPersistent("Forward Limit", Constants.Pivot.forwardLimit).withWidget(BuiltInWidgets.kTextView).getEntry(); 
        PivotReverseLimit = PivotConstants.addPersistent("Reverse Limit", Constants.Pivot.reverseLimit).withWidget(BuiltInWidgets.kTextView).getEntry(); 
        PivotIntakeBackPos = PivotConstants.addPersistent("Intake Back Pos", Constants.Pivot.intakeBackPos).withWidget(BuiltInWidgets.kTextView).getEntry(); 
        PivotScoreHighPos = PivotConstants.addPersistent("Score High pos", Constants.Pivot.scoreHighPos).withWidget(BuiltInWidgets.kTextView).getEntry(); 
        PivotCubePos = PivotConstants.addPersistent("Cube Pos", Constants.Pivot.cubePos).withWidget(BuiltInWidgets.kTextView).getEntry(); 
        PivotIntakeHighPos = PivotConstants.addPersistent("Intake high pos", Constants.Pivot.intakeHighPos).withWidget(BuiltInWidgets.kTextView).getEntry(); 
        PivotLoadPos = PivotConstants.addPersistent("Load Pos", Constants.Pivot.loadPos).withWidget(BuiltInWidgets.kTextView).getEntry(); 
        PivotZero = PivotConstants.addPersistent("Zero", Constants.Pivot.zero).withWidget(BuiltInWidgets.kTextView).getEntry(); 
        PivotShootHighPos = PivotConstants.addPersistent("Shoot high Pos", Constants.Pivot.shootHighPos).withWidget(BuiltInWidgets.kTextView).getEntry(); 
        PivotStowPos = PivotConstants.addPersistent("Stow Pos", Constants.Pivot.stowPos).withWidget(BuiltInWidgets.kTextView).getEntry(); 
        PivotLoadDoublePos = PivotConstants.addPersistent("Load Double Pos", Constants.Pivot.loadDoublePos).withWidget(BuiltInWidgets.kTextView).getEntry(); 

        //Grasper Constants
        GrasperkP = GrasperConstants.addPersistent("Grasper kP", Constants.Grasper.kP).withWidget(BuiltInWidgets.kTextView).getEntry(); 
        GrasperkI = GrasperConstants.addPersistent("Grasper kI", Constants.Grasper.kI).withWidget(BuiltInWidgets.kTextView).getEntry(); 
        GrasperkD = GrasperConstants.addPersistent("Grasper kD", Constants.Grasper.kD).withWidget(BuiltInWidgets.kTextView).getEntry(); 
        GrasperkF = GrasperConstants.addPersistent("Grasper kF", Constants.Grasper.kF).withWidget(BuiltInWidgets.kTextView).getEntry();
    } 
        @Override
        public void periodic() {
            Constants.Drive.MAX_SPEED_MULTIPLIER = DriveMultiplier.getDouble(0.5); 
            Constants.Drive.kP = DrivekP.getDouble(0.3); 
            Constants.Drive.kI = DrivekI.getDouble(0.0); 
            Constants.Drive.kD = DrivekD.getDouble(0.0); 
            Constants.Drive.autoBalanceP = DriveAutoBalanceP.getDouble(0.03); 
            Constants.Drive.RRTapeAlignP = DriveRRTapeAlignP.getDouble(0.03); 
            Constants.Drive.leftYBlueScoringPos1 = LeftYBlue1.getDouble(4.99); 
            Constants.Drive.leftYBlueScoringPos2 = LeftYBlue2.getDouble(3.30); 
            Constants.Drive.leftYBlueScoringPos3 = LeftYBlue3.getDouble(1.63); 
            Constants.Drive.midYBlueScoringPos1 = MidYBlue1.getDouble(4.47); 
            Constants.Drive.midYBlueScoringPos2 = MidYBlue2.getDouble(2.74); 
            Constants.Drive.midYBlueScoringPos3 = MidYBlue3.getDouble(1.06); 
            Constants.Drive.rightYBlueScoringPos1 = RightYBlue1.getDouble(3.89); 
            Constants.Drive.rightYBlueScoringPos2 = RightYBlue2.getDouble(2.20); 
            Constants.Drive.rightYBlueScoringPos3 = RightYBlue3.getDouble(0.49); 
            Constants.Drive.leftYRedScoringPos1 = LeftYRed1.getDouble(4.15); 
            Constants.Drive.leftYRedScoringPos2 = LeftYRed2.getDouble(5.84); 
            Constants.Drive.leftYRedScoringPos3 = LeftYRed3.getDouble(7.55); 
            Constants.Drive.midYRedScoringPos1 = MidYRed1.getDouble(3.57);    
            Constants.Drive.midYRedScoringPos2 = MidYRed2.getDouble(5.30); 
            Constants.Drive.midYRedScoringPos3 = MidYRed3.getDouble(6.98); 
            Constants.Drive.rightYRedScoringPos1 = RightYBlue1.getDouble(3.05); 
            Constants.Drive.rightYRedScoringPos2 = RightYBlue2.getDouble(4.74); 
            Constants.Drive.rightYRedScoringPos3 = RightYBlue3.getDouble(6.41); 

            Constants.Elevator.forwardLimitInches = ElevatorForwardLim.getDouble(40); 
            Constants.Elevator.reverseLimitInches = ElevatorReverseLim.getDouble(0); 
            Constants.Elevator.kP = ElevatorkP.getDouble(0.04); 
            Constants.Elevator.kI = ElevatorkI.getDouble(0.04); 
            Constants.Elevator.kD = ElevatorkD.getDouble(0.0); 
            Constants.Elevator.peakOutForward = ElevatorPeakForwardOut.getDouble(1); 
            Constants.Elevator.peakOutReverse = ElevatorPeakReverseOut.getDouble(-1); 
            Constants.Elevator.intakeHighPos = ElevatorIntakeHighPos.getDouble(12); 
            Constants.Elevator.intakePos = ElevatorIntakePos.getDouble(8.5); 
            Constants.Elevator.bottomPos = ElevatorBottomPos.getDouble(0.5); 
            Constants.Elevator.scoreHighPos = ElevatorScoreHighPos.getDouble(35); 
            Constants.Elevator.scoreMidPos = ElevatorScoreMidPos.getDouble(13.3); 
            Constants.Elevator.stowPos = ElevatorStowPos.getDouble(0); 

            Constants.Wrist.kP = WristkP.getDouble(0.03); 
            Constants.Wrist.kI = WristkI.getDouble(0.0); 
            Constants.Wrist.kD = WristkD.getDouble(0.0); 
            Constants.Wrist.forwardLimit = WristForwardLimit.getDouble(10.75); 
            Constants.Wrist.reverseLimit = WristReverseLimit.getDouble(-10.75); 
            Constants.Wrist.intakeBackPos = WristIntakeBackPos.getDouble(3.3); 
            Constants.Wrist.intakeUpPos = WristIntakeUpPos.getDouble(4.8); 
            Constants.Wrist.intakeCubePos = WristIntakeCubePos.getDouble(5.5); 
            Constants.Wrist.scoreHighPos = WristIntakeScoreHighPos.getDouble(-4.7); 
            Constants.Wrist.scoreMidPos = WristIntakeScoreMidPos.getDouble(-5.0); 
            Constants.Wrist.scoreLowPos = WristIntakeScoreLowPos.getDouble(-2.63); 
            Constants.Wrist.carryPos = WristCarryPos.getDouble(-1.0); 
            Constants.Wrist.shootHighPos = WristShootPos.getDouble(-2.0); 
            Constants.Wrist.loadPos = WristLoadPos.getDouble(0.8); 
            Constants.Wrist.stowPos = WristStowPos.getDouble(1.7); 

            Constants.Pivot.kP = PivotkP.getDouble(1.0); 
            Constants.Pivot.kI = PivotkI.getDouble(0.0); 
            Constants.Pivot.kD = PivotkD.getDouble(0.0); 
            Constants.Pivot.forwardLimit = PivotForwardLimit.getDouble(1000); 
            Constants.Pivot.reverseLimit = PivotReverseLimit.getDouble(-1000); 
            Constants.Pivot.intakeBackPos = PivotIntakeBackPos.getDouble(2.2); 
            Constants.Pivot.scoreHighPos = PivotScoreHighPos.getDouble(-1.2); 
            Constants.Pivot.cubePos = PivotCubePos.getDouble(-1.0); 
            Constants.Pivot.intakeHighPos = PivotIntakeHighPos.getDouble(2.0); 
            Constants.Pivot.loadPos = PivotLoadPos.getDouble(1.45);
            Constants.Pivot.loadDoublePos = PivotLoadDoublePos.getDouble(-0.7); 
            Constants.Pivot.zero = PivotZero.getDouble(0.96); 
            Constants.Pivot.shootHighPos = PivotShootHighPos.getDouble(-0.5); 
            Constants.Pivot.stowPos = PivotStowPos.getDouble(-0.7); 

            Constants.Grasper.kP = GrasperkP.getDouble(0.02); 
            Constants.Grasper.kI = GrasperkI.getDouble(0.17); 
            Constants.Grasper.kD = GrasperkD.getDouble(0.0); 
            Constants.Grasper.kF = GrasperkF.getDouble(0.0); 
    }

}
