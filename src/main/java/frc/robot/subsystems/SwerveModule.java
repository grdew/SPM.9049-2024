package frc.robot.subsystems;


import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.ModuleConstants;
//import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
/*
 * após olhar um bom tempo nesse código, eu percebi que essa clase de subsystem, não é de fato um subsystem, mas meio que uma declaração de o que é um módulo swerve. 
 * essa declaração é usada no "swerveSubsystem"
 */


public class SwerveModule {

    private final CANcoder encoderabs;
    //private final StatusSignal<Double> varEncoderAbs;
    private final CANSparkMax driveMotor;
    private final CANSparkMax turningMotor;

    private final RelativeEncoder driveEncoder;
    private final RelativeEncoder turningEncoder;

    private final PIDController turningPidController;

    //private final boolean absoluteEncoderReversed;
    //private final double absoluteEncoderOffsetRad;
    //private final int idDoEncoder;

    public SwerveModule(int driveMotorId, int turningMotorId, boolean driveMotorReversed, boolean turningMotorReversed,
            int absoluteEncoderId, double absoluteEncoderOffset, boolean absoluteEncoderReversed) {

        //this.absoluteEncoderOffsetRad = absoluteEncoderOffset;
        //this.absoluteEncoderReversed = absoluteEncoderReversed;
        //this.idDoEncoder = absoluteEncoderId;
        encoderabs = new CANcoder(absoluteEncoderId, "rio");//definir a cancoder
        CANcoderConfiguration toApply = new CANcoderConfiguration();
        encoderabs.getConfigurator().apply(toApply);
        encoderabs.getAbsolutePosition().setUpdateFrequency(100);
        encoderabs.getAbsolutePosition().setUpdateFrequency(100);
        //varEncoderAbs = encoderabs.getAbsolutePosition();

        driveMotor = new CANSparkMax(driveMotorId, MotorType.kBrushless);//trocado para sparkmax em CAN ao invés de pwm
        turningMotor = new CANSparkMax(turningMotorId, MotorType.kBrushless);

        driveMotor.setInverted(driveMotorReversed);
        turningMotor.setInverted(turningMotorReversed);

        driveEncoder = driveMotor.getEncoder();
        turningEncoder = turningMotor.getEncoder();

        driveEncoder.setPositionConversionFactor(ModuleConstants.kDriveEncoderRot2Meter);
        driveEncoder.setVelocityConversionFactor(ModuleConstants.kDriveEncoderRPM2MeterPerSec);
        turningEncoder.setPositionConversionFactor(ModuleConstants.kTurningEncoderRot2Rad);
        turningEncoder.setVelocityConversionFactor(ModuleConstants.kTurningEncoderRPM2RadPerSec);

        turningPidController = new PIDController(ModuleConstants.kPTurning, 0, 0);
        turningPidController.enableContinuousInput(-Math.PI, Math.PI);
        
        resetEncoders(); 
    }

    public double getDrivePosition() {
        return driveEncoder.getPosition();
    }

    public double getTurningPosition() {
        return turningEncoder.getPosition();
    }

    public double getDriveVelocity() {
        return driveEncoder.getVelocity();
    }

    public double getTurningVelocity() {
        return turningEncoder.getVelocity();
    }

    public double getAbsoluteEncoderRad() {   //modificado para cancoders                                      
     return 0;
        /* 
        varEncoderAbs.refresh();
        var posicao = varEncoderAbs.getValue();
        double angle = (posicao / 4096) * 360;
        angle *= 2.0 * Math.PI;
        angle -= absoluteEncoderOffsetRad;
        if (idDoEncoder == 4){
        return 0;
        }else{
        return 0;// angle * (absoluteEncoderReversed ? -1.0 : 1.0);
        }//não funciona as cancoders pq eu devo ser burro*/
    }

    public void resetEncoders() {
        driveEncoder.setPosition(0);
        turningEncoder.setPosition(getAbsoluteEncoderRad());
    }

    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(getDriveVelocity(), new Rotation2d(getTurningPosition()));
    }

    public void setDesiredState(SwerveModuleState state) {
        if (Math.abs(state.speedMetersPerSecond) < 0.001) {
            stop();
            return;
        }
        state = SwerveModuleState.optimize(state, getPosition().angle);
        driveMotor.set(state.speedMetersPerSecond / DriveConstants.kPhysicalMaxSpeedMetersPerSecond);
        turningMotor.set(turningPidController.calculate(getTurningPosition(), state.angle.getRadians()));
        //SmartDashboard.putString("Swerve[" + absoluteEncoder.getChannel() + "] state", state.toString()); pra q depuração né kkkk
    }

    public void stop() {
        driveMotor.set(0);
        turningMotor.set(0);
    }
}
