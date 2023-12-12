package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm;

public class ArmDefaultHandling extends CommandBase {

    Arm _arm;

    public ArmDefaultHandling(Arm _arm) {
        this._arm = _arm;
        addRequirements(_arm);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        _arm.compensationCalulations();
        _arm.goToDesiredRotation();
        _arm.goToDesiredExtension();
    }

    @Override
    public void end(boolean interrupted) {
    }
    
    @Override
    public boolean isFinished() {
        return false;
    }
}
