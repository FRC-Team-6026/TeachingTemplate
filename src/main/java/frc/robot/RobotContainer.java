// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.ArmDefaultHandling;
import frc.robot.commands.ArmPositionHandling;
import frc.robot.commands.TeleopSwerve;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Swerve;

public class RobotContainer {
  /* Controllers */
  private final XboxController driver = new XboxController(0);
  private final XboxController operator = new XboxController(1);

  /* Drive Controls */
  private final int translationAxis = XboxController.Axis.kLeftY.value;
  private final int strafeAxis = XboxController.Axis.kLeftX.value;
  private final int rotationAxis = XboxController.Axis.kRightX.value;

  /* Driver Buttons */
  private final JoystickButton zeroGyro =
  new JoystickButton(driver, XboxController.Button.kBack.value);
  private final JoystickButton robotCentricBumper =
  new JoystickButton(driver, XboxController.Button.kStart.value);
  private final JoystickButton resetOdometry = 
  new JoystickButton(driver, XboxController.Button.kA.value);
  private final JoystickButton xSwerve = 
  new JoystickButton(driver, XboxController.Button.kLeftBumper.value);
  private boolean robotCentric = false;

  /* Operator Buttons */
  private final JoystickButton topCone = 
  new JoystickButton(operator, XboxController.Button.kY.value);
  private final JoystickButton topCube = 
  new JoystickButton(operator, XboxController.Button.kX.value);
  private final JoystickButton midCone = 
  new JoystickButton(operator, XboxController.Button.kB.value);
  private final JoystickButton midCube = 
  new JoystickButton(operator, XboxController.Button.kA.value);

  private final JoystickButton subStation = 
  new JoystickButton(operator, XboxController.Button.kBack.value);
  private final JoystickButton floor = 
  new JoystickButton(operator, XboxController.Button.kStart.value);
  private final JoystickButton stow = 
  new JoystickButton(operator, XboxController.Button.kRightBumper.value);

  private final JoystickButton grabber = 
  new JoystickButton(operator, XboxController.Button.kLeftBumper.value);

  /* Subsystems */
  private final Swerve swerve = new Swerve(); 
  private final Arm arm = new Arm();

  public RobotContainer() {
    swerve.setDefaultCommand(
        new TeleopSwerve(
            swerve,
            () -> -driver.getRawAxis(translationAxis),
            () -> -driver.getRawAxis(strafeAxis),
            () -> -driver.getRawAxis(rotationAxis),
            () -> robotCentric));
  
    arm.setDefaultCommand(new ArmDefaultHandling(arm));

    configureBindings();    
  }

  private void configureBindings() {
    /* Driver Buttons */
    zeroGyro.onTrue(new InstantCommand(() -> swerve.zeroGyro()));
    robotCentricBumper.onTrue(new InstantCommand(() -> {
      robotCentric = !robotCentric;
      SmartDashboard.putBoolean("Is Robot Centric", robotCentric);
    }));
    resetOdometry.onTrue(new InstantCommand(() -> swerve.resetToAbsolute()));
    xSwerve.onTrue(new InstantCommand(() -> swerve.xPattern()));

    /* Operator Buttons */
    topCone.onTrue(new ArmPositionHandling(arm, Arm.GrabArmPositions.TopCone));
    topCube.onTrue(new ArmPositionHandling(arm, Arm.GrabArmPositions.TopCube));
    midCone.onTrue(new ArmPositionHandling(arm, Arm.GrabArmPositions.MidCone));
    midCube.onTrue(new ArmPositionHandling(arm, Arm.GrabArmPositions.MidCube));

    subStation.onTrue(new ArmPositionHandling(arm, Arm.GrabArmPositions.Substation));
    floor.onTrue(new ArmPositionHandling(arm, Arm.GrabArmPositions.Floor));
    stow.onTrue(new ArmPositionHandling(arm, Arm.GrabArmPositions.Stow));

    grabber.toggleOnTrue(new InstantCommand(() -> arm.openGrabber()));
    grabber.toggleOnFalse(new InstantCommand(() -> arm.closeGrabber()));

  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }

  public void teleopInit(){
    swerve.xPatternFalse();
    swerve.resetToAbsolute();
  }

  public void autoInit(){
    swerve.resetToAbsolute();
  }
}
