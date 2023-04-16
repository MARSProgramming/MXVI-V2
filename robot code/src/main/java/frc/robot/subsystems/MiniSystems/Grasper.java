// This subsystem is the part of the robot that actually picks up cones and squares, with no sensor it will have to read va;ues to determine whether cargo has been picked up


package frc.robot.subsystems.MiniSystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;



public class Grasper extends SubsystemBase{
    private final double rotationsToNativeUnits = 2048;
    private XboxController mPilotRumble = new XboxController(0);
    private XboxController mCoPilotRumble = new XboxController(1);

    private TalonFX BeltController = new TalonFX(Constants.Grasper.motorID);
    public Grasper() {
        BeltController.configFactoryDefault();
        BeltController.setNeutralMode(NeutralMode.Coast);
        BeltController.setInverted(true);

        BeltController.config_kP(0, Constants.Grasper.kP);
        BeltController.config_kI(0, Constants.Grasper.kI);
        BeltController.config_kD(0, Constants.Grasper.kD);
        BeltController.config_kF(0, Constants.Grasper.kF);

        SupplyCurrentLimitConfiguration curr_lim = new SupplyCurrentLimitConfiguration(true, 20, 20, 0.1);
        BeltController.configSupplyCurrentLimit(curr_lim);
    }

    
    public double getGrasperVelocity(){
      return BeltController.getSelectedSensorVelocity() / rotationsToNativeUnits * 10;
    }

    public double getGrasperPosition(){
      return BeltController.getSelectedSensorPosition() / rotationsToNativeUnits;
    }

    public void RunGrasperEject(){
        BeltController.set(TalonFXControlMode.PercentOutput, -0.6);
    }

    public void RunGrasperStallcheck() {
        //if (UtilityFunctions.isStalling(BeltController.getSelectedSensorPosition(), 1000) && BeltController.getMotorOutputPercent() != 0) {
            //BeltController.set(TalonFXControlMode.PercentOutput, 0);
        //} else {
            BeltController.set(TalonFXControlMode.PercentOutput, 0.8);
        //}
    }

    public void setPercentOutput(double v){
        BeltController.set(ControlMode.PercentOutput, v);
    }

    public double getStatorCurrent(){
        return BeltController.getStatorCurrent();
    }

    public CommandBase setPercentOutputCommand(double v) {
      return runEnd(
        () -> {
          setPercentOutput(v);
        },
        () -> {
          BeltController.set(ControlMode.PercentOutput, 0);
        }
        ).withName("Test Grasper");
    }

    public CommandBase runTestMode() {
        return runEnd(
          () -> {
            RunGrasperStallcheck();
          },
          () -> {
            BeltController.set(ControlMode.PercentOutput, 0);
          }
          ).withName("Test Grasper");
    }

    public CommandBase runSpitMode() {
        return runEnd(
          () -> {
            RunGrasperEject();
          },
          () -> {
            BeltController.set(ControlMode.PercentOutput, 0);
          }
          ).withName("Test Outtake Grasper");
    }

    public CommandBase runTestCurrent() {
      return runEnd(
        () -> {
          BeltController.set(ControlMode.Current, 2.5);
        },
        () -> {
          BeltController.set(ControlMode.PercentOutput, 0);
        }
        ).withName("Test Outtake Grasper");
  }
    public void runSuck(){
        BeltController.set(ControlMode.Current, 2.5);
    }
    Timer mTimer = new Timer();
    @Override
    public void periodic(){
        if(BeltController.getStatorCurrent() > 80 && DriverStation.isTeleop() && BeltController.getMotorOutputPercent() > 0){
            mCoPilotRumble.setRumble(RumbleType.kBothRumble, 1);
            mPilotRumble.setRumble(RumbleType.kBothRumble, 1);
            runTestCurrent().schedule();
            mTimer.start();
        }
        else{
            if(mTimer.get() > 0.5){
                mCoPilotRumble.setRumble(RumbleType.kBothRumble, 0);
                mPilotRumble.setRumble(RumbleType.kBothRumble, 0);
                mTimer.stop();
                mTimer.reset();
            }
        }
        SmartDashboard.putNumber("stator", BeltController.getStatorCurrent());
        SmartDashboard.putNumber("supply", BeltController.getSupplyCurrent());
    }
}