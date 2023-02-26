// This subsystem is to pivot the arm, and has constraints for how far the arm can turn. 
//WristSubsystem: 
//Position Control via REV Encoder (limits?)
//Controls Y axis of end affecter (up/down)
//Gear ratio:
//ID:
//
package frc.robot.subsystems.MiniSystems;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXSimCollection;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.DutyCycleEncoderSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Arm extends SubsystemBase{
    
    private WPI_TalonFX mPivot = new WPI_TalonFX(Constants.Pivot.motorID);
    private WPI_TalonFX mWrist = new WPI_TalonFX(Constants.Wrist.motorID);
    private final DutyCycleEncoder mPivotEncoder = new DutyCycleEncoder(0);
    private final ProfiledPIDController mPivotController = new ProfiledPIDController(20, 0, 0, new TrapezoidProfile.Constraints(2, 5));

    // distance per pulse = (angle per revolution) / (pulses per revolution)
    //  = (2 * PI rads) / (2048 pulses)
    private static final int kCountsPerRevFalcon = 2048;  //Encoder counts per revolution of the motor shaft.
    private static final double kFalconEncoderDistPerPulse = 2.0 * Math.PI / kCountsPerRevFalcon;
    private static final int k100msPerSecond = 10;
    //private final double kRadianstoNativeUnits = 2048 / Math.PI / 2 * kPivotGearRatio;
    
    private static final int kCountsPerRevEncoder = 8192;  //Encoder counts per revolution of the motor shaft.
    private static final double kRevEncoderDistPerPulse = 2.0 * Math.PI / kCountsPerRevEncoder;

    /* Object for simulated inputs into Talon. */
    private final DutyCycleEncoderSim mPivotEncoderSim = new DutyCycleEncoderSim(mPivotEncoder);
    TalonFXSimCollection mPivotSim = mPivot.getSimCollection();
    TalonFXSimCollection mWristSim = mWrist.getSimCollection();

    private static final double kPivotGearRatio = 81; //TODO: what is the gear ratio of the Pivot?
    private static final double kPivotArmMass = 3.19; //Kilograms. TODO: what is the mass of pivot arm in kg?
    private static final double kPivotArmLength = Units.inchesToMeters(23); //TODO: what is the length of the pivot arm in inches?
  
    private static final double kWristGearRatio = 81; //TODO: what is the gear ratio of the Wrist?
    private static final double kGrasperMass = 1.75; // Kilograms. TODO: what is the mass of Grasper in kg?
    private static final double kGrasperLength = Units.inchesToMeters(8); //TODO: what is the length of the Grasper?
  
    private static final int k_pivot_min_angle = -360; 
    private static final int k_pivot_max_angle = 360; 
    private static final int k_wrist_min_angle = -360; 
    private static final int k_wrist_max_angle = 360; 
  
    SingleJointedArmSim m_PivotArmSim = new SingleJointedArmSim(
      DCMotor.getFalcon500(1),  //1 Falcon 500 controls the upper arm.
      kPivotGearRatio,
      SingleJointedArmSim.estimateMOI(kPivotArmLength+kGrasperLength, kPivotArmMass+kGrasperMass),
      kPivotArmLength+kGrasperLength,
      Units.degreesToRadians(k_pivot_min_angle),
      Units.degreesToRadians(k_pivot_max_angle),
      true,
      VecBuilder.fill(kFalconEncoderDistPerPulse) // Add noise with a std-dev of 1 tick
    );
  
    SingleJointedArmSim m_GripperArmSim = new SingleJointedArmSim(
      DCMotor.getFalcon500(1),  //1 Falcon 500 controls the upper arm.
      kWristGearRatio,
      SingleJointedArmSim.estimateMOI(kGrasperLength, kGrasperMass),
      kGrasperLength,
      Units.degreesToRadians(k_wrist_min_angle),
      Units.degreesToRadians(k_wrist_max_angle),
      true,
      VecBuilder.fill(kFalconEncoderDistPerPulse) // Add noise with a std-dev of 1 tick
    );
  
    // Create a Mechanism2d display of an Arm with a fixed ArmTower and moving double jointed Arm.
    private final Mechanism2d m_mech2d = new Mechanism2d(90, 90);
    private final MechanismRoot2d midNodeHome = m_mech2d.getRoot("Mid Node", 27.83, 0);
    private final MechanismLigament2d MidNode = midNodeHome.append(new MechanismLigament2d("Mid Cone Node", 34, 90, 10, new Color8Bit(Color.kWhite)));
    private final MechanismRoot2d highNodeHome = m_mech2d.getRoot("High Node", 10.58, 0);
    private final MechanismLigament2d HighNode = highNodeHome.append(new MechanismLigament2d("High Cone Node", 46, 90, 10, new Color8Bit(Color.kWhite)));
    private final MechanismRoot2d gridHome = m_mech2d.getRoot("Grid Home", 49.75, 0);
    private final MechanismLigament2d GridNode = gridHome.append(new MechanismLigament2d("Grid Wall", 49.75, 180, 50, new Color8Bit(Color.kWhite)));
    private final MechanismRoot2d dsHome = m_mech2d.getRoot("Double Substation Home", 49.75, 37);
    private final MechanismLigament2d DSRamp = dsHome.append(new MechanismLigament2d("Double Substation Ramp", 13.75, 180, 10, new Color8Bit(Color.kWhite)));
    private final MechanismRoot2d m_armPivot = m_mech2d.getRoot("ArmPivot", 65, 21.75);
    private final MechanismLigament2d m_arm_bottom =
        m_armPivot.append(
              new MechanismLigament2d(
                "Arm Bottom",
                Units.metersToInches(kPivotArmLength), 
                Units.radiansToDegrees(m_PivotArmSim.getAngleRads()), 
                10, 
                new Color8Bit(Color.kGold)));
    private final MechanismLigament2d m_arm_tower =
        m_armPivot.append(new MechanismLigament2d("ArmTower", 18, -90, 10, new Color8Bit(Color.kSilver)));
  
    private final MechanismLigament2d m_aframe_1 =
        m_armPivot.append(new MechanismLigament2d("aframe1", 24, -50, 10, new Color8Bit(Color.kSilver)));
    private final MechanismLigament2d m_bumper =
        gridHome.append(new MechanismLigament2d("Bumper", 30.5, 0, 60, new Color8Bit(Color.kRed)));
      private final MechanismLigament2d m_wrist =
      m_arm_bottom.append(
          new MechanismLigament2d(
              "Intake",
              Units.metersToInches(kGrasperLength),
              Units.radiansToDegrees(m_GripperArmSim.getAngleRads()),
              20,
              new Color8Bit(Color.kGray)));
    
    public Arm() {
        mPivot.configFactoryDefault();
        mPivot.setNeutralMode(NeutralMode.Brake);

        mWrist.configFactoryDefault();
        mWrist.setNeutralMode(NeutralMode.Brake);

        mWrist.configReverseSoftLimitThreshold(rotationToNativeUnits(Constants.Wrist.reverseLimit, kWristGearRatio));
        mWrist.configReverseSoftLimitEnable(true);

        mWrist.configForwardSoftLimitThreshold(rotationToNativeUnits(Constants.Wrist.forwardLimit, kWristGearRatio));
        mWrist.configForwardSoftLimitEnable(true);

        mWrist.config_kP(0, Constants.Wrist.kP);
        mWrist.config_kI(0, Constants.Wrist.kI);
        mWrist.config_kD(0, Constants.Wrist.kD);

        mPivotEncoder.reset();
        mPivotEncoder.setDistancePerRotation(Math.PI * 2.0);

        SmartDashboard.putData("Arm Sim", m_mech2d);
        //m_arm_tower.setColor(new Color8Bit(Color.kBlue));    

        SmartDashboard.putNumber("Pivot Setpoint (degrees)", 0);
      }

 @Override
 public void periodic() {
   /*
    * This will get the simulated sensor readings that we set
    * in the previous article while in simulation, but will use
    * real values on the robot itself.
    */
   SmartDashboard.putNumber("Pivot Motor Position (ticks)", mPivot.getSelectedSensorPosition());
   SmartDashboard.putNumber("Pivot Motor Position (deg)", Units.radiansToDegrees(nativeUnitsToRotationRad(mPivot.getSelectedSensorPosition(),kPivotGearRatio)));

   SmartDashboard.putNumber("Pivot Encoder Position", mPivotEncoder.getAbsolutePosition());
   SmartDashboard.putNumber("Pivot Encoder Distance per Rotation", mPivotEncoder.getDistancePerRotation());
   SmartDashboard.putNumber("Pivot Encoder Distance", mPivotEncoder.getDistance());

   SmartDashboard.putNumber("Wrist Position (ticks)", mWrist.getSelectedSensorPosition());
   SmartDashboard.putNumber("Wrist Position (deg)", Units.radiansToDegrees(nativeUnitsToRotationRad(mWrist.getSelectedSensorPosition(),kWristGearRatio)));

   mPivot.setVoltage(mPivotController.calculate(nativeUnitsToRotationRad(mPivot.getSelectedSensorPosition(),kPivotGearRatio), Units.degreesToRadians(SmartDashboard.getNumber("Pivot Setpoint (degrees)", 0))));
   //mPivot.setVoltage(mPivotController.calculate(nativeUnitsToRotationRad(mPivotEncoder.getAbsolutePosition(),1), Units.degreesToRadians(SmartDashboard.getNumber("Pivot Setpoint (degrees)", 0))));
 }

    public CommandBase runPivotManual(DoubleSupplier d) {
        return runEnd(
            () -> {
                mPivot.set(ControlMode.PercentOutput, d.getAsDouble());
              }, 
            () -> {
                mPivot.set(ControlMode.PercentOutput, 0.0);
              }
            ).withName("Test Pivot");
    }

    public CommandBase runWristManual(DoubleSupplier d) {
        return runEnd(
          () -> {
            mWrist.set(ControlMode.PercentOutput, d.getAsDouble());
          },
          () -> {
            mWrist.set(ControlMode.PercentOutput, 0.0);
          }
          ).withName("Test Wrist");
    }

@Override
public void simulationPeriodic() {
  SmartDashboard.putNumber("Sim Pivot Position (rad)", m_PivotArmSim.getAngleRads());
  SmartDashboard.putNumber("Sim Pivot Position (deg)", Units.radiansToDegrees(m_PivotArmSim.getAngleRads()));
  SmartDashboard.putNumber("Sim Wrist Position (deg)", Units.radiansToDegrees(m_GripperArmSim.getAngleRads()));

  /* Pass the robot battery voltage to the simulated Talon FXs */
  mPivotSim.setBusVoltage(RobotController.getBatteryVoltage());
  mWristSim.setBusVoltage(RobotController.getBatteryVoltage());

  /*
   * WPILib expects +V to be forward.
   * Positive motor output lead voltage is ccw. 
   */
  m_PivotArmSim.setInput(mPivotSim.getMotorOutputLeadVoltage());
  m_GripperArmSim.setInput(mWristSim.getMotorOutputLeadVoltage());

  /*
   * Advance the model by 20 ms. Note that if you are running this
   * subsystem in a separate thread or have changed the nominal
   * timestep of TimedRobot, this value needs to match it.
   */
  m_PivotArmSim.update(0.02);
  m_GripperArmSim.update(0.02);

  /*
   * Update all of our sensors.
   * WPILib's simulation class is assuming +V is forward
   */
  mPivotEncoderSim.setDistance(m_PivotArmSim.getAngleRads()/Math.PI/2.0);
//  mPivotEncoderSim.setDistance(Math.PI / -2.0);
  mPivotSim.setIntegratedSensorRawPosition(
                  rotationToNativeUnits(
                      m_PivotArmSim.getAngleRads(),
                      kPivotGearRatio
                  ));
  mPivotSim.setIntegratedSensorVelocity(
                  velocityToNativeUnits(
                      m_PivotArmSim.getVelocityRadPerSec(),
                      kPivotGearRatio
                  ));
  mWristSim.setIntegratedSensorRawPosition(
                  rotationToNativeUnits(
                      m_GripperArmSim.getAngleRads(),
                      kWristGearRatio
                  ));
  mWristSim.setIntegratedSensorVelocity(
                  velocityToNativeUnits(
                      m_GripperArmSim.getVelocityRadPerSec(),
                      kWristGearRatio
                  ));
    // SimBattery estimates loaded battery voltages
  RoboRioSim.setVInVoltage(
      BatterySim.calculateDefaultBatteryLoadedVoltage(m_PivotArmSim.getCurrentDrawAmps()));

  // Update the Mechanism Arm angle based on the simulated arm angle
  m_arm_bottom.setAngle(Units.radiansToDegrees(m_PivotArmSim.getAngleRads()));
  m_wrist.setAngle(Units.radiansToDegrees(m_GripperArmSim.getAngleRads()));
}

  private double nativeUnitsToRotationRad(double sensorCounts, double gearRatio){
    double motorRotations = (double)sensorCounts / kCountsPerRevFalcon;
    double jointRotations = motorRotations / gearRatio;
    double positionRads = jointRotations * 2 * Math.PI;
    return positionRads;
  }

  private int rotationToNativeUnits(double rotationRads, double gearRatio){
    double motorRotations = rotationRads / (2 * Math.PI) * gearRatio;
    int sensorCounts = (int)(motorRotations * kCountsPerRevFalcon);
    return sensorCounts;
  }

  private int velocityToNativeUnits(double velocityRadPerSecond, double gearRatio){
    double motorRotationsPerSecond = velocityRadPerSecond / (2 * Math.PI) * gearRatio;
    double motorRotationsPer100ms = motorRotationsPerSecond / k100msPerSecond;
    int sensorCountsPer100ms = (int)(motorRotationsPer100ms * kCountsPerRevFalcon);
    return sensorCountsPer100ms;
  }

}


