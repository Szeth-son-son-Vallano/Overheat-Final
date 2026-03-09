// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.FuelSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class OutakeCommand extends Command {
  public static Double targetSpeed;
  public static FuelSubsystem m_fuelSubsystem;
  /** Creates a new OutakeCommand. */
  public OutakeCommand(FuelSubsystem fuelSubsysten, Double targetSpeed) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_fuelSubsystem = fuelSubsysten;
    addRequirements(m_fuelSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_fuelSubsystem.outake();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_fuelSubsystem.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
