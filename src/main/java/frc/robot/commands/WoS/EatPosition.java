// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.WoS;

import frc.robot.testingdashboard.Command;
import frc.robot.subsystems.WoS;
import frc.robot.Constants;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class EatPosition extends Command {
  
  WoS m_WoS;
  /** Creates a new EatPosition. */
  public EatPosition() {
    super(WoS.getInstance(),"WoS","EatPosition");
    m_WoS = WoS.getInstance();
    addRequirements(m_WoS);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_WoS.setTargetAngle(Constants.WoSConstants.kEatAngle);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
