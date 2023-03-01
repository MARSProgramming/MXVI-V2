// This subsystem is the part of the robot that actually picks up cones and squares, with no sensor it will have to read va;ues to determine whether cargo has been picked up


package frc.robot.subsystems.MiniSystems;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.util.UtilityFunctions;



public class Grasper extends SubsystemBase{

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

    public void RunGrasperEject(){
        BeltController.set(TalonFXControlMode.PercentOutput, -0.6);
    }

    public void RunGrasperStallcheck() {
        //if (UtilityFunctions.isStalling(BeltController.getSelectedSensorPosition(), 1000) && BeltController.getMotorOutputPercent() != 0) {
            //BeltController.set(TalonFXControlMode.PercentOutput, 0);
        //} else {
            BeltController.set(TalonFXControlMode.PercentOutput, 0.6);
        //}
    }

    public void setPercentOutput(double v){
        BeltController.set(ControlMode.PercentOutput, v);
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
    
    @Override
    public void periodic(){
      SmartDashboard.putNumber("Grasper Current", BeltController.getSupplyCurrent());
      SmartDashboard.putNumber("Error", BeltController.getClosedLoopError());
    }
}