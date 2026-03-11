// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.ClimbSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ClimbDownCommand extends Command {
  /** Creates a new ClimbDownCommand. */
  public static ClimbSubsystem m_climbSubsystem;
  private static double m_target;
  public ClimbDownCommand(ClimbSubsystem climber, double targetPos) {

    m_climbSubsystem = climber;
    m_target = targetPos;
    // Use addRequirements() here to declare subsystem dependencies.

    addRequirements(m_climbSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_climbSubsystem.climbDown(m_target);
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
