package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm;

public class ArmPositionHandling extends CommandBase {

    Arm _arm;
    Arm.GrabArmPositions position;

    public ArmPositionHandling(Arm _arm, Arm.GrabArmPositions position) {
        this._arm = _arm;
        this.position = position;
        addRequirements(_arm);
    }

    @Override
    public void initialize() {
        _arm.setDesiredPosition(position);
        _arm.runCommandTimer();
        _arm.disengageServo();
    }

    @Override
    public void execute() {
        _arm.compensationCalulations();
        _arm.goToDesiredRotationPosition();
        _arm.goToDesiredExtensionPosition();
    }

    @Override
    public void end(boolean interrupted) {
        _arm.stopNClearTimer();
        _arm.engageServo();
    }
    
    @Override
    public boolean isFinished() {
        return _arm.isFinished();
    }
}
