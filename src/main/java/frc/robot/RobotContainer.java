// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Autos.TowerAuto;
import frc.robot.Commands.AlignCommand;
import frc.robot.Commands.ClimbDownCommand;
import frc.robot.Commands.ClimbUpCommand;
import frc.robot.Commands.DriveCommand;
import frc.robot.Commands.IntakeCommand;
import frc.robot.Commands.OutakeCommand;
import frc.robot.Commands.PassCommand;
import frc.robot.Commands.ReturnToZeroCommand;
import frc.robot.Commands.SetToMaxCommand;
import frc.robot.Commands.ShootCommand;
import frc.robot.Constants.ClimbConstants;
import frc.robot.Constants.FuelConstants;
import frc.robot.Constants.opConstants;
import frc.robot.Subsystems.ClimbSubsystem;
import frc.robot.Subsystems.DriveSubsystem;
import frc.robot.Subsystems.FuelSubsystem;
import frc.robot.Subsystems.VisionSubsystem;

public class RobotContainer {
  private final DriveSubsystem m_drive = new DriveSubsystem();
  private final FuelSubsystem m_fuel = new FuelSubsystem();
  private final ClimbSubsystem m_climb = new ClimbSubsystem();
  private VisionSubsystem m_vision;

  private Command towerAuto; 

  SendableChooser <Command> m_Chooser = new SendableChooser<>();
  
  public final CommandXboxController m_controller = 
  new CommandXboxController(opConstants.driveControllerID);

  private final CommandXboxController m_opController = 
  new CommandXboxController(opConstants.opControllerID);

  public RobotContainer() {
    m_vision = new VisionSubsystem(m_drive::addVisionMeasurement); //Add estConsumer to Vision
    towerAuto = new TowerAuto(m_vision, m_drive, m_fuel, m_climb);
    configureBindings();
    m_Chooser.setDefaultOption("Tower Auto", towerAuto);
    SmartDashboard.putData(m_Chooser);
  }

  private void configureBindings() {
    m_drive.setDefaultCommand(new DriveCommand(m_drive,
     () -> -m_controller.getLeftY()*(0.35 + (0.65 * m_controller.getRightTriggerAxis())),
     () -> -m_controller.getRightX(),
     () -> false)); //Drive Command
     
    m_controller.rightBumper().whileTrue(
      new ShootCommand(m_fuel, FuelConstants.targetSpeed)); //Shoot

    m_controller.leftBumper().whileTrue(
      new IntakeCommand(m_fuel, FuelConstants.targetSpeed)); //Intake

    m_controller.leftTrigger().whileTrue(
      new PassCommand(m_fuel, FuelConstants.targetSpeed)); //Pass

    m_controller.y().whileTrue(
      new OutakeCommand(m_fuel, FuelConstants.targetSpeed)); //Outake

    m_controller.povLeft().whileTrue(
      new AlignCommand(m_drive, m_vision)); //Auto Align

    m_opController.rightTrigger().whileTrue(
      new ClimbUpCommand(m_climb)); // Declimb

    m_opController.leftTrigger().whileTrue(
      new ClimbDownCommand(m_climb, ClimbConstants.climbL2Pos)); // L2 Climb


    m_opController.a().and(m_opController.x()).whileTrue(
      new ReturnToZeroCommand(m_climb, -ClimbConstants.climbUpPos +10)); 

    m_opController.b().and(m_opController.y()).whileTrue(
      new SetToMaxCommand(m_climb));

  }

  public Command getAutonomousCommand() {
    return m_Chooser.getSelected();
  }
}
