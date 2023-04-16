package frc.robot.subsystems.MiniSystems;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

/*
 * Double stage slanted elevator moving the manipulator up and down
 * 
 * 2 Motors - IDs x, y
 * Gear Ratio - 49:1(tentative)
 * Constraints - Cannot move too far up or down
 * 
 * Controls:
 * position control through moveToPosition() method
 * manual movement through movePercentOutput() method
 */

public class Elevator extends SubsystemBase{

    private TalonFX master;
    private TalonFX follower;
    private final double kGearRatio = 20;
    private final double inchesPerRotation = 2 * Math.PI;
    private final double inchesToNativeUnits = 2048 * kGearRatio / inchesPerRotation;
    private final DigitalInput limitSwitch = new DigitalInput(Constants.Elevator.limitSwitchID);
    private boolean isLimitHit = false;
    private double adjustDouble = 0.0;
    public Elevator(){
        master = new TalonFX(Constants.Elevator.masterMotorID);
        follower = new TalonFX(Constants.Elevator.followerMotorID);
        master.configFactoryDefault();
        follower.configFactoryDefault();

        master.configForwardSoftLimitThreshold(Constants.Elevator.forwardLimitInches * inchesToNativeUnits);
        master.configForwardSoftLimitEnable(true);

        master.configReverseSoftLimitThreshold(Constants.Elevator.reverseLimitInches * inchesToNativeUnits);
        master.configReverseSoftLimitEnable(true);
        
        master.setNeutralMode(NeutralMode.Brake);
        follower.setNeutralMode(NeutralMode.Brake);
        master.setInverted(false);
        follower.setInverted(true);
        
        master.config_kP(0, Constants.Elevator.kP);
        master.config_kI(0, Constants.Elevator.kI);
        master.config_kD(0, Constants.Elevator.kD);

        master.configPeakOutputForward(Constants.Elevator.peakOutForward);
        master.configPeakOutputReverse(Constants.Elevator.peakOutReverse);

        master.setSelectedSensorPosition(0);
        follower.follow(master);
        
    }
    public CommandBase disableSoftLimits() {
      return runOnce(
        () -> {
          master.configForwardSoftLimitEnable(false);
          master.configReverseSoftLimitEnable(false);
        }
        ).withName("Disable Elevator Limits");
    }
    
    public CommandBase moveDoubleSetpointDown() {
        return runOnce(
          () -> {
            adjustDouble = -3;
          }
          );
    }

    public CommandBase resetDoubleSetpoint() {
        return runOnce(
          () -> {
            adjustDouble = 0;
          }
          );
    }

    public double getPosition(){
      return master.getSelectedSensorPosition() / inchesToNativeUnits;
    }
    public void goToBottom(){
      setPosition(Constants.Elevator.bottomPos);
    }
    public void goToIntake(){
      setPosition(Constants.Elevator.intakePos);
    }
    public void goToIntakeHigh(){
      setPosition(Constants.Elevator.intakeHighPos);
    }
    public void goToScoreHigh(){
      setPosition(Constants.Elevator.scoreHighPos);
    }
    public void goToScoreMid(){
      setPosition(Constants.Elevator.scoreMidPos);
    }
    public void goToScoreLow(){
        setPosition(Constants.Elevator.scoreLowPos);
    }
    public void goToCloseCubeIntake(){
        setPosition(Constants.Elevator.intakeCloseCubePos);
    }
    public void goToStow(){
      setPosition(Constants.Elevator.stowPos);
    }
    public void goToLoadDouble(){
      setPosition(Constants.Elevator.loadDoublePos + adjustDouble);
    }
    public void goToLoad(){
        setPosition(Constants.Elevator.loadPos);
      }
    public void setPosition(double inches){
      //if(isLimitHit){
        //master.set(ControlMode.PercentOutput, 0);
      //}
      //else{
        master.set(ControlMode.Position, inches * inchesToNativeUnits, DemandType.ArbitraryFeedForward, 0.05);
      //}
    }
    public void setPercentOutput(double v){
      if(isLimitHit && v < 0){
        master.set(ControlMode.PercentOutput, 0);
      }
      else{
        master.set(ControlMode.PercentOutput, v);
      }
    }

    public CommandBase runTestMode(DoubleSupplier d) {
        return runEnd(
          () -> {
            setPercentOutput(d.getAsDouble());
          },
          () -> {
            master.set(ControlMode.PercentOutput, 0);
          }
          ).withName("Test Elevator");
    }

    public CommandBase testSetpoint(DoubleSupplier d) {
        return runEnd(
          () -> {
            setPosition(d.getAsDouble());
          },
          () -> {
            master.set(ControlMode.PercentOutput, 0);
          }
          ).withName("Test Elevator Setpoints");
    }

    public CommandBase testBottomSetpoint() {
      return runEnd(
        () -> {
          goToBottom();
        },
        () -> {
          master.set(ControlMode.PercentOutput, 0);
        }
        ).withName("Test Elevator Setpoints");
  }
  
    public double distanceToSetpoint(double setpoint){
        return master.getSelectedSensorPosition() / inchesToNativeUnits - setpoint;
    }

    public double getElevatorVelocity(){
      return master.getSelectedSensorVelocity() / inchesToNativeUnits * 10;
    }

    public double getElevatorPosition(){
      return master.getSelectedSensorPosition() / inchesToNativeUnits;
    }

    public boolean isLimitHit(){
      return !limitSwitch.get();
    }

    public boolean atSetpoint(){
        return master.getClosedLoopError() < 0.2 * inchesToNativeUnits;
    }
    
    @Override
    public void periodic(){
      isLimitHit = isLimitHit();
      if(isLimitHit){
        master.setSelectedSensorPosition(0);
      }
    }
}
