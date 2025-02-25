// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj.simulation.EncoderSim;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.util.Color8Bit;
import frc.robot.testingdashboard.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotMap;
import frc.robot.testingdashboard.TDNumber;
import frc.robot.testingdashboard.TDSendable;

public class Elevator extends SubsystemBase {
  private static Elevator m_elevator;

  private final double periodTime = 0.02;

  TDNumber m_targetAngle;
  TDNumber m_elevatorEncoderValueRotations;
  TDNumber m_elevatorEncoderValueDegrees;
  TDNumber m_elevatorCurrentOutput;
  TDNumber m_TDelevatorP;
  TDNumber m_TDelevatorI;
  TDNumber m_TDelevatorD;
  double m_elevatorP = Constants.ElevatorConstants.kElevatorP;
  double m_elevatorI = Constants.ElevatorConstants.kElevatorI;
  double m_elevatorD = Constants.ElevatorConstants.kElevatorD;
  private double m_elevatorLastAngle = 0;

  SparkFlex m_elevatorLeftSparkFlex;
  SparkFlex m_elevatorRightSparkFlex;

  SparkFlexConfig m_leftSparkFlexConfig;

  TDNumber m_elevatorLeftCurrentOutput;
  TDNumber m_elevatorRightCurrentOutput;

  SparkClosedLoopController m_elevatorClosedLoopController;
  SparkAbsoluteEncoder m_elevatorAbsoluteEncoder;

  ElevatorFeedforward m_elevatorFeedForwardController;
  TrapezoidProfile m_elevatorProfile;
  TrapezoidProfile.State m_elevatorState;
  TrapezoidProfile.State m_elevatorSetpoint;

  TDNumber m_targetShoulderAngle;
  TDNumber m_WoSShoulderEncoderValueRotations;
  TDNumber m_WoSShoulderEncoderValueDegrees;
  TDNumber m_WoSShoulderCurrentOutput;
  TDNumber m_TDshoulderP;
  TDNumber m_TDshoulderI;
  TDNumber m_TDshoulderD;
  double m_shoulderP = Constants.WoSConstants.kShoulderP;
  double m_shoulderI = Constants.WoSConstants.kShoulderI;
  double m_shoulderD = Constants.WoSConstants.kShoulderD;
  double m_shoulderkS = Constants.WoSConstants.kShoulderkS;
  double m_shoulderkG = Constants.WoSConstants.kShoulderkG;
  double m_shoulderkV = Constants.WoSConstants.kShoulderkV;
  private double m_shoulderLastAngle = 0;

  SparkMax m_WoSSparkMax;
  SparkMaxConfig m_SparkMaxConfig;

  SparkMax m_WoSShoulderSparkMax;
  SparkMaxConfig m_WoSShoulderSparkMaxConfig;
  SparkAbsoluteEncoder m_WoSShoulderAbsoluteEncoder;
  SparkClosedLoopController m_WoSShoulderClosedLoopController;

  ArmFeedforward m_WoSShoulderArmFeedForwardController;

  // This gearbox represents a gearbox containing 2 Vex 775pro motors.
  private final DCMotor m_elevatorMotor = DCMotor.getNEO(2);

  // Simulation classes help us simulate what's going on, including gravity.
  private final ElevatorSim m_elevatorSim =
      new ElevatorSim(m_elevatorMotor,
       20,
       5.0,
       0.10,
       0.0,
       1.0,
       true,
       0.0);
  private Encoder m_encoder;
  private EncoderSim m_encoderSim;
  private DCMotorSim m_motorSim;

  // Create a Mechanism2d visualization of the elevator
  private final Mechanism2d m_mech2d = new Mechanism2d(20, 50);
  private final MechanismRoot2d m_mech2dRoot = m_mech2d.getRoot("Elevator Root", 10, 0);
  private final MechanismLigament2d m_elevatorMech2d =
      m_mech2dRoot.append(
          new MechanismLigament2d("Elevator", m_elevatorSim.getPositionMeters() * 20, 90));
  
  /** Creates a new Elevator. */
  private Elevator() {
    super("Elevator");

    if (RobotMap.E_ENABLED) {
      // setup elevator
      m_elevatorLeftSparkFlex = new SparkFlex(RobotMap.E_LEFTMOTOR, MotorType.kBrushless);
      m_elevatorRightSparkFlex = new SparkFlex(RobotMap.E_RIGHTMOTOR, MotorType.kBrushless);

      m_leftSparkFlexConfig = new SparkFlexConfig();
      SparkFlexConfig rightElevatorSparkFlexConfig = new SparkFlexConfig();

      rightElevatorSparkFlexConfig.follow(m_elevatorLeftSparkFlex, true);

      m_TDelevatorP = new TDNumber(this, "Elevator PID", "P", Constants.ElevatorConstants.kElevatorP);
      m_TDelevatorI = new TDNumber(this, "Elevator PID", "I", Constants.ElevatorConstants.kElevatorI);
      m_TDelevatorD = new TDNumber(this, "Elevator PID", "D", Constants.ElevatorConstants.kElevatorD);

      m_leftSparkFlexConfig.closedLoop.pid(Constants.ElevatorConstants.kElevatorP, Constants.ElevatorConstants.kElevatorI,
          Constants.ElevatorConstants.kElevatorD);
      m_leftSparkFlexConfig.closedLoop.feedbackSensor(FeedbackSensor.kAbsoluteEncoder);
      m_leftSparkFlexConfig.closedLoop.positionWrappingEnabled(false);

      m_leftSparkFlexConfig.absoluteEncoder.positionConversionFactor(Constants.ElevatorConstants.kElevatorEncoderPositionFactor);
      m_leftSparkFlexConfig.absoluteEncoder.inverted(false);

      m_elevatorLeftSparkFlex.configure(m_leftSparkFlexConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

      m_elevatorClosedLoopController = m_elevatorLeftSparkFlex.getClosedLoopController();
      m_elevatorAbsoluteEncoder = m_elevatorLeftSparkFlex.getAbsoluteEncoder();

      m_elevatorFeedForwardController = new ElevatorFeedforward(Constants.ElevatorConstants.kElevatorkS, Constants.ElevatorConstants.kElevatorkG, Constants.ElevatorConstants.kElevatorkV);
      m_elevatorProfile = new TrapezoidProfile(new TrapezoidProfile.Constraints(
        Constants.ElevatorConstants.kElevatorMaxVelocity,
        Constants.ElevatorConstants.kElevatorMaxAcceleration
      ));
      m_elevatorSetpoint = new TrapezoidProfile.State(m_elevatorAbsoluteEncoder.getPosition(), 0.0);
      m_elevatorState = new TrapezoidProfile.State(m_elevatorAbsoluteEncoder.getPosition(), 0.0);

      m_targetAngle = new TDNumber(this, "Elevator Encoder Values", "Target Angle", getElevatorAngle());
      m_elevatorEncoderValueRotations = new TDNumber(this, "Elevator Encoder Values", "Rotations", getElevatorAngle() / Constants.ElevatorConstants.kElevatorEncoderPositionFactor);
      m_elevatorEncoderValueDegrees = new TDNumber(this, "Elevator Encoder Values", "Angle (degrees)", getElevatorAngle());
      m_elevatorLeftCurrentOutput = new TDNumber(this, "Current", "Left Elevator Output", m_elevatorLeftSparkFlex.getOutputCurrent());
      m_elevatorRightCurrentOutput = new TDNumber(this, "Current", "Right Elevator Output", m_elevatorRightSparkFlex.getOutputCurrent());

      
      // setup shoulder
      m_WoSShoulderSparkMax = new SparkMax(RobotMap.W_SHOULDERMOTOR, MotorType.kBrushless);
      m_WoSShoulderSparkMaxConfig = new SparkMaxConfig();

      m_TDshoulderP = new TDNumber(this, "WoS Shoulder PID", "P", Constants.WoSConstants.kShoulderP);
      m_TDshoulderI = new TDNumber(this, "WoS Shoulder PID", "I", Constants.WoSConstants.kShoulderI);
      m_TDshoulderD = new TDNumber(this, "WoS Shoulder PID", "D", Constants.WoSConstants.kShoulderD);

      m_WoSShoulderSparkMaxConfig.idleMode(IdleMode.kBrake);

      m_WoSShoulderSparkMaxConfig.closedLoop.pid(Constants.ElevatorConstants.kShoulderP, Constants.ElevatorConstants.kShoulderI,
          Constants.ElevatorConstants.kShoulderD);
      m_WoSShoulderSparkMaxConfig.closedLoop.feedbackSensor(FeedbackSensor.kAbsoluteEncoder);
      m_WoSShoulderSparkMaxConfig.closedLoop.positionWrappingEnabled(true);
      m_WoSShoulderSparkMaxConfig.closedLoop.positionWrappingMinInput(0);
      m_WoSShoulderSparkMaxConfig.closedLoop.positionWrappingMaxInput(Constants.ElevatorConstants.DEGREES_PER_REVOLUTION);

      m_WoSShoulderSparkMaxConfig.absoluteEncoder.positionConversionFactor(Constants.ElevatorConstants.kWoSShoulderEncoderPositionFactor);
      m_WoSShoulderSparkMaxConfig.absoluteEncoder.velocityConversionFactor(Constants.ElevatorConstants.kWoSShoulderEncoderVelocityFactor);
      m_WoSShoulderSparkMaxConfig.absoluteEncoder.inverted(false);

      m_WoSShoulderSparkMax.configure(m_WoSShoulderSparkMaxConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

      m_WoSShoulderAbsoluteEncoder = m_WoSShoulderSparkMax.getAbsoluteEncoder();
      m_WoSShoulderClosedLoopController = m_WoSShoulderSparkMax.getClosedLoopController();

      m_WoSShoulderArmFeedForwardController = new ArmFeedforward(m_shoulderkS, m_shoulderkG, m_shoulderkV);

      m_targetShoulderAngle = new TDNumber(this, "WoS Encoder Values", "Target Shoulder Angle", getShoulderAngle());
      m_WoSShoulderEncoderValueRotations = new TDNumber(this, "WoS Encoder Values", "Rotations", getShoulderAngle() / Constants.WoSConstants.kWoSShoulderEncoderPositionFactor);
      m_WoSShoulderEncoderValueDegrees = new TDNumber(this, "WoS Encoder Values", "Shoulder Angle (radians)", getShoulderAngle());
      m_WoSShoulderCurrentOutput = new TDNumber(Drive.getInstance(), "Current", "WoS Angle Output", m_WoSShoulderSparkMax.getOutputCurrent());

      m_encoder = new Encoder(1,2);
      m_encoderSim = new EncoderSim(m_encoder);
      m_motorSim = new DCMotorSim(LinearSystemId.createDCMotorSystem(6, 8), m_elevatorMotor);

      new TDSendable(this, "Elevator", "Position", m_mech2d);
    }
  }

  public static Elevator getInstance() {
    if (m_elevator == null) {
      m_elevator = new Elevator();
    }
    return m_elevator;
  }

  public void moveElevatorUp() {
    if (m_elevatorLeftSparkFlex != null) {
      m_elevatorLeftSparkFlex.set(Constants.ElevatorConstants.kElevatorSpeed);
      m_motorSim.setInputVoltage(Constants.ElevatorConstants.kElevatorSpeed * 12);
    }
  }

  public void moveElevatorDown() {
    if (m_elevatorLeftSparkFlex != null) {
      m_elevatorLeftSparkFlex.set(-Constants.ElevatorConstants.kElevatorSpeed);
      m_motorSim.setInputVoltage(-Constants.ElevatorConstants.kElevatorSpeed * 12);
    }
  }

  public void stopElevator() {
    if (m_elevatorLeftSparkFlex != null) {
      m_elevatorLeftSparkFlex.set(0.0);
      m_motorSim.setInputVoltage(0);
    }
  }

  public double getElevatorAngle() {
    return m_elevatorAbsoluteEncoder.getPosition();
  }

  public void setElevatorTargetAngle(double angle) {
    angle = MathUtil.clamp(angle,
                              Constants.ElevatorConstants.kElevatorLowerLimitDegrees, 
                              Constants.ElevatorConstants.kElevatorUpperLimitDegrees);
    if (angle != m_elevatorLastAngle) {
      m_targetAngle.set(angle);
      m_elevatorLastAngle = angle;
      m_elevatorSetpoint = new TrapezoidProfile.State(angle, 0.0);
    }
  }

  public void setElevatorTargetLevel(int level) {
    if (level < 1 || level > 4) return;
    setElevatorTargetAngle(Constants.ElevatorConstants.kElevatorLevels[level]);
  }

  public double getShoulderAngle() { 
    return m_WoSShoulderAbsoluteEncoder.getPosition() * Constants.ElevatorConstants.kWoSShoulderMotorToShoulderRatio;
  }

  public void setShoulderTargetAngle(double angle) {
    double setpoint = angle % Constants.ElevatorConstants.DEGREES_PER_REVOLUTION;
    setpoint = MathUtil.clamp(setpoint,
                              Constants.ElevatorConstants.kWoSShoulderLowerLimitDegrees, 
                              Constants.ElevatorConstants.kWoSShoulderUpperLimitDegrees);
    if (setpoint != m_shoulderLastAngle) {
      m_targetShoulderAngle.set(setpoint);
      m_shoulderLastAngle = setpoint;
    }
  }

  public void setShoulderTargetLevel(int level) {
    if (level < 1 || level > 4) return;
    setShoulderTargetAngle(Constants.ElevatorConstants.kWoSShoulderLevels[level]);
  }

  public void setShoulderSpeeds(double ShoulderRPM) {
      m_WoSSparkMax.getClosedLoopController().setReference(ShoulderRPM, ControlType.kVelocity);
  }

  public void spinShoulder(double shoulderSpeed){
    if (m_WoSShoulderSparkMax != null) {
      m_WoSShoulderSparkMax.set(shoulderSpeed);
    }
  }

  @Override
  public void periodic() {
    if (Constants.ElevatorConstants.kEnableElevatorPIDTuning &&
        m_elevatorLeftSparkFlex != null) {
      double tmp = m_TDelevatorP.get();
      boolean changed = false;
      if (tmp != m_elevatorP) {
        m_elevatorP = tmp;
        m_leftSparkFlexConfig.closedLoop.p(m_elevatorP);
        changed = true;
      }
      tmp = m_TDelevatorI.get();
      if (tmp != m_elevatorI) {
        m_elevatorI = tmp;
        changed = true;
        m_leftSparkFlexConfig.closedLoop.i(m_elevatorI);
      }
      tmp = m_TDelevatorD.get();
      if (tmp != m_elevatorD) {
        m_elevatorD = tmp;
        changed = true;
        m_leftSparkFlexConfig.closedLoop.d(m_elevatorD);
      }
      if(changed) {
        m_elevatorLeftSparkFlex.configure(m_leftSparkFlexConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
      }
    }
    if (Constants.ElevatorConstants.kEnableShoulderPIDTuning &&
        m_WoSShoulderSparkMax != null) {
      double tmp = m_TDshoulderP.get();
      boolean changed = false;
      if (tmp != m_shoulderP) {
        m_shoulderP = tmp;
        m_WoSShoulderSparkMaxConfig.closedLoop.p(m_shoulderP);
        changed = true;
      }
      tmp = m_TDshoulderI.get();
      if (tmp != m_shoulderI) {
        m_shoulderI = tmp;
        changed = true;
        m_WoSShoulderSparkMaxConfig.closedLoop.i(m_shoulderI);
      }
      tmp = m_TDshoulderD.get();
      if (tmp != m_shoulderD) {
        m_shoulderD = tmp;
        changed = true;
        m_WoSShoulderSparkMaxConfig.closedLoop.d(m_shoulderD);
      }
      if(changed) {
        m_WoSShoulderSparkMax.configure(m_WoSShoulderSparkMaxConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
      }
    }
    if (RobotMap.E_ENABLED) {
      m_elevatorLeftCurrentOutput.set(m_elevatorLeftSparkFlex.getOutputCurrent());
      m_elevatorRightCurrentOutput.set(m_elevatorRightSparkFlex.getOutputCurrent());
      m_elevatorEncoderValueDegrees.set(getElevatorAngle()/Constants.ElevatorConstants.kElevatorEncoderPositionFactor);
      m_elevatorEncoderValueRotations.set(getElevatorAngle());

      m_WoSShoulderCurrentOutput.set(m_WoSShoulderSparkMax.getOutputCurrent());
      m_WoSShoulderEncoderValueDegrees.set(getShoulderAngle()/Constants.WoSConstants.kWoSShoulderEncoderPositionFactor);
      m_WoSShoulderEncoderValueRotations.set(getShoulderAngle());

      double shoulderArbFeedforward = m_WoSShoulderArmFeedForwardController.calculate(m_shoulderLastAngle, 0.0);
      m_WoSShoulderClosedLoopController.setReference(m_shoulderLastAngle, ControlType.kPosition, ClosedLoopSlot.kSlot0, shoulderArbFeedforward);

      m_elevatorState = m_elevatorProfile.calculate(periodTime, m_elevatorState, m_elevatorSetpoint);
      double elevatorArbFeedforward = m_elevatorFeedForwardController.calculate(m_elevatorSetpoint.velocity);
      m_elevatorClosedLoopController.setReference(m_elevatorSetpoint.position, ControlType.kPosition, ClosedLoopSlot.kSlot0, elevatorArbFeedforward);
    }
    m_elevatorMech2d.setLength(m_elevatorSim.getPositionMeters() * 20);
    super.periodic();
  }

  /** Advance the simulation. */
  public void simulationPeriodic() {
    // In this method, we update our simulation of what our elevator is doing
    // First, we set our "inputs" (voltages)
    m_elevatorSim.setInput(m_motorSim.getInputVoltage());

    // Next, we update it. The standard loop time is 20ms.
    m_elevatorSim.update(0.020);

    // Finally, we set our simulated encoder's readings and simulated battery voltage
    m_encoderSim.setDistance(m_elevatorSim.getPositionMeters() * 10);
    // SimBattery estimates loaded battery voltages
    RoboRioSim.setVInVoltage(
        BatterySim.calculateDefaultBatteryLoadedVoltage(m_elevatorSim.getCurrentDrawAmps()));
  }

}
