// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.io.FileWriter;
import java.io.IOException;
import java.time.Instant;

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
import edu.wpi.first.units.BaseUnits;
import edu.wpi.first.units.measure.Voltage;
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
  TDNumber m_shoulderEncoderValueRotations;
  TDNumber m_shoulderEncoderValueDegrees;
  TDNumber m_shoulderCurrentOutput;
  TDNumber m_TDshoulderP;
  TDNumber m_TDshoulderI;
  TDNumber m_TDshoulderD;
  double m_shoulderP = Constants.ElevatorConstants.kShoulderP;
  double m_shoulderI = Constants.ElevatorConstants.kShoulderI;
  double m_shoulderD = Constants.ElevatorConstants.kShoulderD;
  double m_shoulderkS = Constants.ElevatorConstants.kShoulderkS;
  double m_shoulderkG = Constants.ElevatorConstants.kShoulderkG;
  double m_shoulderkV = Constants.ElevatorConstants.kShoulderkV;
  private double m_shoulderLastAngle = 0;
  
  private double r;
  private double curr_r;
  private boolean pid_running;

  SparkMax m_WoSSparkMax;
  SparkMaxConfig m_SparkMaxConfig;

  SparkMax m_shoulderSparkMax;
  SparkMaxConfig m_shoulderSparkMaxConfig;
  SparkAbsoluteEncoder m_shoulderAbsoluteEncoder;
  SparkClosedLoopController m_shoulderClosedLoopController;

  ArmFeedforward m_shoulderArmFeedForwardController;

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

      r = getElevatorAngle();
      pid_running = false;

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
      m_shoulderSparkMax = new SparkMax(RobotMap.E_SHOULDERMOTOR, MotorType.kBrushless);
      m_shoulderSparkMaxConfig = new SparkMaxConfig();

      m_TDshoulderP = new TDNumber(this, "WoS Shoulder PID", "P", Constants.ElevatorConstants.kShoulderP);
      m_TDshoulderI = new TDNumber(this, "WoS Shoulder PID", "I", Constants.ElevatorConstants.kShoulderI);
      m_TDshoulderD = new TDNumber(this, "WoS Shoulder PID", "D", Constants.ElevatorConstants.kShoulderD);

      m_shoulderSparkMaxConfig.idleMode(IdleMode.kBrake);

      m_shoulderSparkMaxConfig.closedLoop.pid(Constants.ElevatorConstants.kShoulderP, Constants.ElevatorConstants.kShoulderI,
          Constants.ElevatorConstants.kShoulderD);
      m_shoulderSparkMaxConfig.closedLoop.feedbackSensor(FeedbackSensor.kAbsoluteEncoder);
      m_shoulderSparkMaxConfig.closedLoop.positionWrappingEnabled(true);
      m_shoulderSparkMaxConfig.closedLoop.positionWrappingMinInput(0);
      m_shoulderSparkMaxConfig.closedLoop.positionWrappingMaxInput(Constants.ElevatorConstants.DEGREES_PER_REVOLUTION);

      m_shoulderSparkMaxConfig.absoluteEncoder.positionConversionFactor(Constants.ElevatorConstants.kShoulderEncoderPositionFactor);
      m_shoulderSparkMaxConfig.absoluteEncoder.velocityConversionFactor(Constants.ElevatorConstants.kShoulderEncoderVelocityFactor);
      m_shoulderSparkMaxConfig.absoluteEncoder.inverted(false);

      m_shoulderSparkMax.configure(m_shoulderSparkMaxConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

      m_shoulderAbsoluteEncoder = m_shoulderSparkMax.getAbsoluteEncoder();
      m_shoulderClosedLoopController = m_shoulderSparkMax.getClosedLoopController();

      m_shoulderArmFeedForwardController = new ArmFeedforward(m_shoulderkS, m_shoulderkG, m_shoulderkV);

      m_targetShoulderAngle = new TDNumber(this, "WoS Encoder Values", "Target Shoulder Angle", getShoulderAngle());
      m_shoulderEncoderValueRotations = new TDNumber(this, "WoS Encoder Values", "Rotations", getShoulderAngle() / Constants.ElevatorConstants.kShoulderEncoderPositionFactor);
      m_shoulderEncoderValueDegrees = new TDNumber(this, "WoS Encoder Values", "Shoulder Angle (radians)", getShoulderAngle());
      m_shoulderCurrentOutput = new TDNumber(Drive.getInstance(), "Current", "WoS Angle Output", m_shoulderSparkMax.getOutputCurrent());

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

  /**
	 * This function conducts an experiment used to identify parameters about the plant
	 * Outputs from this experiment will be filtered in MATLAB to obtain plant parameters
	 */
	public static void paramID() {
		// Define output array
		double[] output = new double[2000];
		
		Instant start = Instant.now();
	      
		for (int i = 0; i < output.length; i++) {
			//Wait until 1 ms has passed
			Instant end = Instant.now();
			Duration timeElapsed = Duration.between(start, end);
			while (timeElapsed.toNanos() / 1000000 == 0) {
				end = Instant.now();
				timeElapsed = Duration.between(start, end);
			}
			start = Instant.now();
			
			// Actuate plant
			if (i < 500) {
				m_elevatorLeftSparkFlex.setVoltage(6.0);
			} else if (i > 1000 && i < 1250){
				m_elevatorLeftSparkFlex.setVoltage(-6.0);
			} else {
				m_elevatorLeftSparkFlex.setVoltage(0.0);
			}
			
			// Read plant response
			output[i] = getElevatorAngle();
		}
		
		// Write to .csv for data processing
		try (FileWriter writer = new FileWriter("param.csv")) {
            for (int i = 0; i < output.length; i++) {
                writer.append(String.valueOf(output[i]));
                if (i < output.length - 1) {
                    writer.append(",");
                }
            }
            writer.append("\n");
            System.out.println("CSV file created: " + "param.csv");
        } catch (IOException e) {
            System.err.println("Error writing to file: " + e.getMessage());
        }
	}

  /**
	 * This function acts as a state space PI controller for the elevator
	 * Plant parameters: alpha, beta
	 * Other constants: calculated using method learned in Control Systems class.
	 * @param angle - the desired elevator setpoint
	 */
  public void elevatorToPosition(double angle) {
    // If elevator is already being driven to setpoint, simply update reference command
    if (pid_running) {
      r = angle;
      return;
    }
    pid_running = true;
    r = angle;
    curr_r = getElevatorAngle();

    // If elevator is at rest, start new Thread
    Thread elevatorThread = new Thread(() -> {
      // Initialize state variables
      // x1 = motor angle
      // x2 = motor velocity (which we presume starts at zero) - can be swapped for a getVelocity() api call
      // sigma = regulator (proportional control) state variable
      double x1 = getElevatorAngle();
      double x2 = 0.0;
      double sigma = 0.0;
      double y;

      // Plant parameters (TBD via parameter ID function)
      double alpha = 50;
      double beta = 20;

      // Max input allowed (Volts)
      double Umax = 12.0;

      // Bandwidth variables (control system tuning knobs)
	    // Lower values --> slower response, higher overshoot
      double lambda_e = 20.0;
      double lambda_r = 50.0;

      // Coefficient Matrix values
      double l1 = 2 * lambda_e - alpha;
      double l2 = lambda_e * lambda_e - 2 * alpha * lambda_e + alpha * alpha;

      double k1 = 1/beta * 3 * lambda_r *lambda_r;
      double k2 = 1/beta * (3*lambda_r - a);
      double k3 = 1/beta * lambda_r * lambda_r * lambda_r;
      double kappa = lambda_r / k3;
      
      boolean done = false;
      Instant start = Instant.now();

      while (!done) {
        Instant end = Instant.now();
        Duration timeElapsed = Duration.between(start, end);
        if (timeElapsed.toNanos() / 1000000 > 0.0) {
          if (curr_r < .999 * r) {
            curr_r += 500 * .001 * timeElapsed.toSeconds();
          } else if (curr_r > 1.001 * r) {
            curr_r -= 500 * .001 * timeElapsed.toSeconds();
          } else {
            curr_r = r;
          }
          start = Instant.now();
          y = getElevatorAngle();

          // Check if reference angle reached
          // *NOTE* - This assumes bandwidth variables are set so that system does not overshoot
          if (y > .999*r || y < 1.001*r) {
            m_elevatorLeftSparkFlex.setVoltage(0.0);
            done = true;
            pid_running = false;
            continue;
          }

          // Compute actuator input
          float uc = -k1*x1 - k2*x2-k3*sigma;
          float u;
          if (uc < -1*Umax) {
            u = -1*Umax;
          } else if (uc > Umax) {
            u = Umax;
          } else {
            u = uc;
          }

          //Actuate Elevator
          m_elevatorLeftSparkFlex.setVoltage(u);

          // Numerical state variable integration
          x1 = x1 + T * (x2 - l1*(x1 - y));
          x2 = x2 + T * (alpha*u - beta - l2*(x1 -y));
          sigma = sigma + T*(y - curr_r + kappa*(uc -u));
        }
      }
    });
    elevatorThread.start();
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
    return m_shoulderAbsoluteEncoder.getPosition() * Constants.ElevatorConstants.kShoulderMotorToShoulderRatio;
  }

  public void setShoulderTargetAngle(double angle) {
    double setpoint = angle % Constants.ElevatorConstants.DEGREES_PER_REVOLUTION;
    setpoint = MathUtil.clamp(setpoint,
                              Constants.ElevatorConstants.kShoulderLowerLimitDegrees, 
                              Constants.ElevatorConstants.kShoulderUpperLimitDegrees);
    if (setpoint != m_shoulderLastAngle) {
      m_targetShoulderAngle.set(setpoint);
      m_shoulderLastAngle = setpoint;
    }
  }

  public void setShoulderTargetLevel(int level) {
    if (level < 1 || level > 4) return;
    setShoulderTargetAngle(Constants.ElevatorConstants.kShoulderLevels[level]);
  }

  public void setShoulderSpeeds(double ShoulderRPM) {
      m_WoSSparkMax.getClosedLoopController().setReference(ShoulderRPM, ControlType.kVelocity);
  }

  public void spinShoulder(double shoulderSpeed){
    if (m_shoulderSparkMax != null) {
      m_shoulderSparkMax.set(shoulderSpeed);
    }
  }

  public void stopShoulder(){
    if (m_shoulderSparkMax != null) {
      m_shoulderSparkMax.set(0);
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
        m_shoulderSparkMax != null) {
      double tmp = m_TDshoulderP.get();
      boolean changed = false;
      if (tmp != m_shoulderP) {
        m_shoulderP = tmp;
        m_shoulderSparkMaxConfig.closedLoop.p(m_shoulderP);
        changed = true;
      }
      tmp = m_TDshoulderI.get();
      if (tmp != m_shoulderI) {
        m_shoulderI = tmp;
        changed = true;
        m_shoulderSparkMaxConfig.closedLoop.i(m_shoulderI);
      }
      tmp = m_TDshoulderD.get();
      if (tmp != m_shoulderD) {
        m_shoulderD = tmp;
        changed = true;
        m_shoulderSparkMaxConfig.closedLoop.d(m_shoulderD);
      }
      if(changed) {
        m_shoulderSparkMax.configure(m_shoulderSparkMaxConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
      }
    }
    if (RobotMap.E_ENABLED) {
      m_elevatorLeftCurrentOutput.set(m_elevatorLeftSparkFlex.getOutputCurrent());
      m_elevatorRightCurrentOutput.set(m_elevatorRightSparkFlex.getOutputCurrent());
      m_elevatorEncoderValueDegrees.set(getElevatorAngle()/Constants.ElevatorConstants.kElevatorEncoderPositionFactor);
      m_elevatorEncoderValueRotations.set(getElevatorAngle());

      m_shoulderCurrentOutput.set(m_shoulderSparkMax.getOutputCurrent());
      m_shoulderEncoderValueDegrees.set(getShoulderAngle()/Constants.ElevatorConstants.kShoulderEncoderPositionFactor);
      m_shoulderEncoderValueRotations.set(getShoulderAngle());

      double shoulderArbFeedforward = m_shoulderArmFeedForwardController.calculate(m_shoulderLastAngle, 0.0);
      m_shoulderClosedLoopController.setReference(m_shoulderLastAngle, ControlType.kPosition, ClosedLoopSlot.kSlot0, shoulderArbFeedforward);

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

  public void setShoulderVoltage(Voltage volts) {
    m_shoulderSparkMax.setVoltage(volts.in(BaseUnits.VoltageUnit));
  }

  public double getShoulderVoltage() {
    return m_shoulderSparkMax.get() * RobotController.getBatteryVoltage();
  }

  public double getShoulderPosition() {
    return m_shoulderSparkMax.getEncoder().getPosition();
  }

  public double getShoulderVelocity() {
    return m_shoulderSparkMax.getEncoder().getVelocity();
  }
}
