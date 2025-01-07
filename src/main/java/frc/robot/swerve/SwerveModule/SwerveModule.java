package frc.robot.swerve.SwerveModule;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.swerve.DriveConstants;
import frc.robot.util.debugging.LoggedTunableNumber;

public class SwerveModule extends SubsystemBase {

    // All the logged tunable numbers used //
    public static final LoggedTunableNumber driveP = new LoggedTunableNumber("Module/Drive/kP", DriveConstants.driveControllerConfig.kP());
    public static final LoggedTunableNumber driveD = new LoggedTunableNumber("Module/Drive/kD", DriveConstants.driveControllerConfig.kD());

    public static final LoggedTunableNumber driveS = new LoggedTunableNumber("Module/Drive/kS", DriveConstants.driveControllerConfig.kS());
    public static final LoggedTunableNumber driveV = new LoggedTunableNumber("Module/Drive/kV", DriveConstants.driveControllerConfig.kV());
    public static final LoggedTunableNumber driveA = new LoggedTunableNumber("Module/Drive/kA", DriveConstants.driveControllerConfig.kA());

    public static final LoggedTunableNumber azimuthP = new LoggedTunableNumber("Module/Azimuth/kP", DriveConstants.azimuthControllerConfig.kP());
    public static final LoggedTunableNumber azimuthD = new LoggedTunableNumber("Module/Azimuth/kD", DriveConstants.azimuthControllerConfig.kD());
    public static final LoggedTunableNumber azimuthS = new LoggedTunableNumber("Module/Azimuth/kS", DriveConstants.azimuthControllerConfig.kS());

    private Double velocitySetpointMPS = null;
    private Rotation2d azimuthSetpointAngle = null;
    private Double accelerationSetpointMPSS = null;

    private SwerveModuleState currentState = new SwerveModuleState();
    private SwerveModulePosition currentPosition = new SwerveModulePosition();

    private SwerveModuleHardware io;
    private SwerveModuleInputsAutoLogged inputs = new SwerveModuleInputsAutoLogged();

    private SimpleMotorFeedforward driveFeedforward = new SimpleMotorFeedforward(0, 0, 0);

    private String nameKey;

    public SwerveModule(SwerveModuleHardware config){
        this.io = (config);
        nameKey = "Module/" + io.getName();
    }

    public void periodic(){
        io.updateInputs(inputs);
        Logger.processInputs("Drive/"+ nameKey, inputs);

        currentState = new SwerveModuleState(inputs.driveVelocityMPS, inputs.azimuthPosistion);
        currentPosition = new SwerveModulePosition(inputs.drivePosistionM, inputs.azimuthPosistion);

        // Checks if setpoint is existing before setting a demand // 
        if(velocitySetpointMPS != null){

            if(accelerationSetpointMPSS != null){
                double feedforward = driveFeedforward.calculate(velocitySetpointMPS, accelerationSetpointMPSS);
                io.setDriveVelocity(velocitySetpointMPS, feedforward);
            }

            else{
                io.setDriveVelocity(velocitySetpointMPS, 0);
            }
        }

        if(azimuthSetpointAngle != null){
            io.setAzimuthPosistion(azimuthSetpointAngle.getRotations());
        }

        // Changes PID / FF gains // 
        LoggedTunableNumber.ifChanged(hashCode(), () -> {
            io.setDriveConstants(driveP.get(), driveD.get(), driveS.get(), driveV.get(), driveA.get());
        }, driveP, driveD, driveS, driveV, driveA);

        LoggedTunableNumber.ifChanged(hashCode(), () -> {
            io.setAzimuthConstants(azimuthP.get(), azimuthD.get(), azimuthS.get());
        }, azimuthP, azimuthD, azimuthS);

        LoggedTunableNumber.ifChanged(hashCode(), () -> {
            driveFeedforward = new SimpleMotorFeedforward(driveS.get(), driveV.get(), driveA.get());
        }, driveS, driveV, driveA);

    }

    /**
     * Reset the posistion of the azimuth encoder
     */
    public void resetAzimuthEncoder(){
        io.resetAzimuthPosistion();
    }

    /**
     * Run characterization for feedforward gains, for the swerve
     * Bind this to a button and let it run
     * @param inputVolts should be set to 4 or 6 volts
     */
    public void runLinearCharacterization(double inputVolts) {
        setAzimuthPosistion(new Rotation2d());
        setDriveVelocity(null);
        setDriveVolts(inputVolts);
    }

    /**
     * Set the desired velocity/posistion respectivly to the drive and azimuth
     * @param state desired state
     * @return set state
     */
    public SwerveModuleState setSwerveState(SwerveModuleState state){
        setDriveAcceleration(null);
        setDriveVelocity(state.speedMetersPerSecond);
        setAzimuthPosistion(state.angle);
        return new SwerveModuleState(velocitySetpointMPS, azimuthSetpointAngle);
    }

    public SwerveModuleState setSwerveStatewithAccel(SwerveModuleState state, double accel){
        setDriveAcceleration(accelerationSetpointMPSS);
        setDriveVelocity(state.speedMetersPerSecond);
        setAzimuthPosistion(state.angle);
        return new SwerveModuleState(velocitySetpointMPS, azimuthSetpointAngle);
    }

    public SwerveModuleState setAzimuthPosistion(SwerveModuleState state){
        setAzimuthPosistion(state.angle);
        setDriveVelocity(null);
        setDriveAcceleration(null);
        return new SwerveModuleState(0, azimuthSetpointAngle);
    }

    /**
     * Returns the velocity/posistion of the drive and azimuth respectively
     * @return current state of the swerve module
     */
    public SwerveModuleState getCurrentState(){
        return currentState;
    }

    /**
     * Returnds the distance traveled/posistion of the drive and azimuth respectively
     * @return current position of the swerve module
     */
    public SwerveModulePosition getCurrentPosistion(){
        return currentPosition;
    }

    /**
     * Set velocity demand to the drive motor
     * @param velocityDemand (units : m/s)
     */
    public void setDriveVelocity(Double velocityDemand){
        velocitySetpointMPS = velocityDemand;
    }

    /**
     * Set acceleration demand to the drive motor
     * @param accelDemand (units : m/s/s)
     */
    public void setDriveAcceleration(Double accelDemand){
        accelerationSetpointMPSS = accelDemand;
    }

    /**
     * Set voltage to the drive motor
     * @param volts (range: 0-12)
     */
    public void setDriveVolts(Double volts){
        io.setDriveVolts(volts);
    }

    /**
     * Set posistion to the azimuth motor
     * @param volts (units: rotations)
     */
    public void setAzimuthPosistion(Rotation2d posistionDemand){
        azimuthSetpointAngle = posistionDemand;
    }

    /**
     * Set voltage to the azimuth motor
     * @param volts (range: 0-12)
     */
    public void setAzimuthVolts(Double volts){
        io.setAzimuthVolts(volts);
    }

    public SwerveModuleInputsAutoLogged getInputs(){
        return inputs;
    }

    /**
     * Stop swerve module
     */
    public void stop(){
        io.setAzimuthVolts(0);
        io.setDriveVolts(0);
    }

}
