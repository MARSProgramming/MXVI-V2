package frc.robot.util;
import java.util.Map;

import edu.wpi.first.math.util.Units;
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
    private ShuffleboardLayout DrivetrainConstants = Constant.getLayout("Drivetrain Const", BuiltInLayouts.kList).withSize(2,5).withPosition(0,0);
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

    private GenericEntry BlueBLOff; 
    private GenericEntry BlueBROff; 
    private GenericEntry BlueMLOff; 
    private GenericEntry BlueMROff; 
    private GenericEntry BlueTLOff; 
    private GenericEntry BlueTROff;
    private GenericEntry RedBLOff;
    private GenericEntry RedBROff;
    private GenericEntry RedMLOff;
    private GenericEntry RedMROff;
    private GenericEntry RedTlOff;
    private GenericEntry RedTROff;

    
    public void configureDashboard(){ 
        //Drive Constants
        DriveMultiplier = DrivetrainConstants.addPersistent("Max Speed Multiplier", Constants.Drive.MAX_SPEED_MULTIPLIER).withWidget(BuiltInWidgets.kTextView).getEntry(); 
        DrivekP = DrivetrainConstants.addPersistent("Drive kP", Constants.Drive.kP).withWidget(BuiltInWidgets.kTextView).getEntry(); 
        DrivekI = DrivetrainConstants.addPersistent("Drive kI", Constants.Drive.kI).withWidget(BuiltInWidgets.kTextView).getEntry(); 
        DrivekD = DrivetrainConstants.addPersistent("Drive kD", Constants.Drive.kD).withWidget(BuiltInWidgets.kTextView).getEntry(); 
        DriveAutoBalanceP = DrivetrainConstants.addPersistent("Auto Balance P", Constants.Drive.autoBalanceP).withWidget(BuiltInWidgets.kTextView).getEntry(); 
        DriveRRTapeAlignP = DrivetrainConstants.addPersistent("RRTape Align P", Constants.Drive.RRTapeAlignP).withWidget(BuiltInWidgets.kTextView).getEntry(); 
        
        BlueBLOff = DrivetrainConstants.addPersistent("Blue Bottom Left Offset", Units.metersToInches(Constants.Drive.BlueBottomLOffset)).withWidget(BuiltInWidgets.kTextView).getEntry(); 
        BlueBROff = DrivetrainConstants.addPersistent("Blue Bottom Right Offset", Units.metersToInches(Constants.Drive.BlueBottomROffset)).withWidget(BuiltInWidgets.kTextView).getEntry(); 
        BlueMLOff = DrivetrainConstants.addPersistent("Blue Mid Left Offset", Units.metersToInches(Constants.Drive.BlueMidLOffset)).withWidget(BuiltInWidgets.kTextView).getEntry(); 
        BlueMROff = DrivetrainConstants.addPersistent("Blue Mid Right Offset",Units.metersToInches(Constants.Drive.BlueMidROffset)).withWidget(BuiltInWidgets.kTextView).getEntry(); 
        BlueTLOff = DrivetrainConstants.addPersistent("Blue Top Left Offset", Units.metersToInches(Constants.Drive.BlueBottomLOffset)).withWidget(BuiltInWidgets.kTextView).getEntry(); 
        BlueTROff = DrivetrainConstants.addPersistent("Blue Top Right Offset", Units.metersToInches(Constants.Drive.BlueBottomROffset)).withWidget(BuiltInWidgets.kTextView).getEntry(); 
        RedBLOff = DrivetrainConstants.addPersistent("Red Bottom Left Offset", Units.metersToInches(Constants.Drive.RedBottomLOffset)).withWidget(BuiltInWidgets.kTextView).getEntry(); 
        RedBROff = DrivetrainConstants.addPersistent("Red Bottom Right Offset", Units.metersToInches(Constants.Drive.RedBottomROffset)).withWidget(BuiltInWidgets.kTextView).getEntry(); 
        RedMLOff = DrivetrainConstants.addPersistent("Red Mid Left Offset", Units.metersToInches(Constants.Drive.RedMidLOffset)).withWidget(BuiltInWidgets.kTextView).getEntry(); 
        RedMROff = DrivetrainConstants.addPersistent("Red Mid Right Offset", Units.metersToInches(Constants.Drive.RedMidROffset)).withWidget(BuiltInWidgets.kTextView).getEntry(); 
        RedTlOff = DrivetrainConstants.addPersistent("Red Top Left Offset", Units.metersToInches(Constants.Drive.RedTopLOffset)).withWidget(BuiltInWidgets.kTextView).getEntry(); 
        RedTROff = DrivetrainConstants.addPersistent("Red Top Right Offset", Units.metersToInches(Constants.Drive.RedTopROffset)).withWidget(BuiltInWidgets.kTextView).getEntry(); 
        
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
            Constants.Drive.BlueBottomLOffset = Units.inchesToMeters(BlueBLOff.getDouble(0.57)); 
            Constants.Drive.BlueBottomROffset = Units.inchesToMeters(BlueBROff.getDouble(0.57)); 
            Constants.Drive.BlueMidLOffset = Units.inchesToMeters(BlueMLOff.getDouble(0.54)); 
            Constants.Drive.BlueMidROffset = Units.inchesToMeters(BlueMROff.getDouble(0.56)); 
            Constants.Drive.BlueTopLOffset = Units.inchesToMeters(BlueTLOff.getDouble(0.58)); 
            Constants.Drive.BlueTopROffset = Units.inchesToMeters(BlueTROff.getDouble(0.52)); 
            Constants.Drive.RedBottomLOffset = Units.inchesToMeters(RedBLOff.getDouble(0.57)); 
            Constants.Drive.RedBottomROffset = Units.inchesToMeters(RedBROff.getDouble(0.57)); 
            Constants.Drive.RedMidLOffset = Units.inchesToMeters(RedMLOff.getDouble(0.56)); 
            Constants.Drive.RedMidROffset = Units.inchesToMeters(RedMROff.getDouble(0.54)); 
            Constants.Drive.RedTopLOffset = Units.inchesToMeters(RedTlOff.getDouble(0.52)); 
            Constants.Drive.RedTopROffset = Units.inchesToMeters(RedTROff.getDouble(0.58)); 
            
            
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
