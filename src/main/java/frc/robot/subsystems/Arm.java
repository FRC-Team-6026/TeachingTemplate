package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.SparkMaxPIDController.ArbFFUnits;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.Items.SparkMax.SparkController;
import frc.lib.configs.Sparkmax.SparkControllerInfo;
import frc.robot.Constants;

public class Arm extends SubsystemBase{

    public final Solenoid _grabber;
    public final SparkController _rotation;
    public final SparkController _extension;
    public final Servo _ratchetServo;

    public final RelativeEncoder _rotationEncoder;
    public final SparkMaxPIDController _rotationController;
    public final RelativeEncoder _extensionEncoder;
    public final SparkMaxPIDController _extensionController;

    private double _rotationCompensation = 0;
    private double _extensionCompensation = 0;

    private double _desiredRotation = 0;
    private double _desiredExtension = 0;

    private Timer _commandTimer;
    private TrapezoidProfile _rotationProfile;
    private final TrapezoidProfile.Constraints _rotationConstraints;
    private TrapezoidProfile.State _rotationState;
    private TrapezoidProfile _extensionProfile;
    private final TrapezoidProfile.Constraints _extensionConstraints;
    private TrapezoidProfile.State _extensionState;
    
    public Arm(){
        this._grabber = new Solenoid(Constants.Setup.grabberID, PneumaticsModuleType.REVPH, 1);
        this._rotation = new SparkController(Constants.Setup.rotationId , new SparkControllerInfo().rotation());
        this._extension = new SparkController(Constants.Setup.extensionId, new SparkControllerInfo().extension());
        this._ratchetServo = new Servo(9);

        this._rotationEncoder = _rotation.sparkEncoder;
        this._rotationController = _rotation.sparkControl;
        this._extensionEncoder = _extension.sparkEncoder;
        this._extensionController = _extension.sparkControl;

        this._rotationConstraints = new TrapezoidProfile.Constraints(Constants.Arm.maxAutoPositionRotationDps, Constants.Arm.maxAutoPositionRotationDpsAcc);
        this._extensionConstraints = new TrapezoidProfile.Constraints(Constants.Arm.maxAutoPositionMaxIps, Constants.Arm.maxAutoPositionMaxIpsAcc);

        this._commandTimer = new Timer();
    }

    public void setDesiredPosition(GrabArmPositions position){
        _desiredRotation = position.rotation;
        _desiredExtension = position.extension;
        calculateProfiles();
    }

    public void goToDesiredRotation(){
        _rotationController.setReference(_desiredRotation, ControlType.kPosition, 1, _rotationCompensation, ArbFFUnits.kPercentOut);
    }

    public void goToDesiredExtension(){
        _extensionController.setReference(_desiredExtension, ControlType.kPosition, 1, _extensionCompensation, ArbFFUnits.kPercentOut);
    }

    public void goToDesiredRotationPosition(){
        var position = _rotationProfile.calculate(commandTime());
        _rotationController.setReference(position.position, ControlType.kPosition, 1, _rotationCompensation, ArbFFUnits.kPercentOut);
    }
    
    public void goToDesiredExtensionPosition(){
        var position = _extensionProfile.calculate(commandTime());
        _extensionController.setReference(position.position, ControlType.kPosition, 1, _extensionCompensation, ArbFFUnits.kPercentOut);
    }

    public void engageServo(){
        _ratchetServo.setAngle(30);
    }

    public void disengageServo(){
        _ratchetServo.setAngle(90);
    }

    public void openGrabber(){
        _grabber.set(true);
    }

    public void closeGrabber(){
        _grabber.set(false);
    }

    public void runCommandTimer(){
        _commandTimer.restart();
    }

    public void clearCommandTimer(){
        _commandTimer.reset();
    }

    public void stopNClearTimer(){
        _commandTimer.stop();
        _commandTimer.reset();
    }

    private double commandTime(){
        return _commandTimer.get() + Constants.CommandConstants.codeExecutionRateTime;
    }

    public void calculateInitialStates(){
        _rotationState = new TrapezoidProfile.State(_rotationEncoder.getPosition(), _rotationEncoder.getVelocity());
        _extensionState = new TrapezoidProfile.State(_extensionEncoder.getPosition(), _extensionEncoder.getVelocity());
    }

    public void calculateProfiles(){
        calculateInitialStates();
        TrapezoidProfile.State desiredRotationState = new TrapezoidProfile.State(_desiredRotation, 0.0);
        TrapezoidProfile.State desiredExtensionState = new TrapezoidProfile.State(_desiredExtension, 0.0);
        _rotationProfile = new TrapezoidProfile(_rotationConstraints, desiredRotationState, _rotationState);
        _extensionProfile = new TrapezoidProfile(_extensionConstraints, desiredExtensionState, _extensionState);
    }

    public boolean isFinished(){
        if(Math.abs(_desiredRotation - _rotationEncoder.getPosition()) < Constants.Arm.rotationPositionSettingToleranceDegrees && Math.abs(_desiredExtension - _extensionEncoder.getPosition()) < Constants.Arm.extensionPositionSettingToleranceInches){
            return true;
        }
        return false;
    }

    public void compensationCalulations(){
        //Compensation Calculations
        double centerOfGrav = (7.5+(0.254*_extensionEncoder.getPosition()));
        double cosineCompensation = Math.cos(Math.toRadians(_rotationEncoder.getPosition() - Constants.Arm.rotationOffsetinDegrees));
        double inLbTorqueR = (5 * centerOfGrav * cosineCompensation);
        double newtonMeterTorqueR = inLbTorqueR / 8.8507457673787;
        double motorOutputR = newtonMeterTorqueR / Constants.ConversionFactors.rotationGearRatio;
        _rotationCompensation = motorOutputR / Constants.Arm.rotationStallTorque; // output / stall torque

        double tensionFromSprings = (Constants.Arm.firstStageTension); //Spring force
        double inLBTorqueE = Constants.Arm.spoolRadius * tensionFromSprings;
        double newtonMeterTorqueE = inLBTorqueE / 8.8507457673787;
        double motorOutputE = newtonMeterTorqueE / Constants.ConversionFactors.extensionGearRatio;
        _extensionCompensation = - (motorOutputE / Constants.Arm.extensionStallTorque) * 0.8; // output / stall torque

        if(_extensionEncoder.getPosition() > 10.2){
            _extensionCompensation = _extensionCompensation * 0.05;
        }
    }

    public enum GrabArmPositions {
        Substation (179.6,1) {
            @Override
            public GrabArmPositions previous() {
                return Floor;
            };
        },
        TopCone(174,20.25),
        TopCube(181.7,15.7),
        MidCone(178,3.9),
        MidCube(197,2),
        Floor(5,16.25) {
            @Override
            public GrabArmPositions next() {
                return Substation;
            };
        },
        Stow(5,0);

        private final double rotation;
        private final double extension;

        GrabArmPositions(double rotation, double extension){
            this.rotation = rotation;
            this.extension = extension;
        }

        public GrabArmPositions next() {
            // No bounds checking required here, because the last instance overrides
            return values()[ordinal() + 1];
        }

        public GrabArmPositions previous() {
            // No bounds checking required here, because the first instance overrides
            return values()[ordinal() - 1];
        }
    }

}
