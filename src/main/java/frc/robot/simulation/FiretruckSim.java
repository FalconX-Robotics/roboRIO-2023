package frc.robot.simulation;

import java.util.function.BiFunction;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.numbers.N4;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.NumericalIntegration;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.LinearSystemSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;

public class FiretruckSim extends LinearSystemSim<N4, N2, N2> {

    public static class ArmParameters {
        public final DCMotor m_gearbox;
        public final double m_gearing;
        public final double m_retractedMOI;
        public final double m_minAngleRads;
        public final double m_maxAngleRads;
        public final double m_armMassKg;
        public final double m_armInitialCOGDistanceMeters;

        public ArmParameters(DCMotor gearbox, double gearing, double retractedMOI,
                double minAngleRads, double maxAngleRads, double armMassKg, double m_armInitialCOGDistanceMeters) {
            this.m_gearbox = gearbox;
            this.m_gearing = gearing;
            this.m_retractedMOI = retractedMOI;
            this.m_minAngleRads = minAngleRads;
            this.m_maxAngleRads = maxAngleRads;
            this.m_armMassKg = armMassKg;
            this.m_armInitialCOGDistanceMeters = m_armInitialCOGDistanceMeters;
        }
    }

    public static class ExtenderParameters {
        public final DCMotor m_gearbox;
        public final double m_gearing;
        public final double m_carriageMassKg;
        public final double m_drumRadiusMeters;
        public final double m_minDistanceMeters;
        public final double m_maxDistanceMeters;
        public final double m_carriageInitialCOGDistanceMeters;

        public ExtenderParameters(DCMotor gearbox, double gearing, double carriageMassKg, double drumRadiusMeters,
                double minDistanceMeters, double maxDistanceMeters, double carriageInitialCOGDistanceMeters) {
            this.m_gearbox = gearbox;
            this.m_gearing = gearing;
            this.m_carriageMassKg = carriageMassKg;
            this.m_drumRadiusMeters = drumRadiusMeters;
            this.m_minDistanceMeters = minDistanceMeters;
            this.m_maxDistanceMeters = maxDistanceMeters;
            this.m_carriageInitialCOGDistanceMeters = carriageInitialCOGDistanceMeters;
        }
    }


    public static LinearSystem<N4, N2, N2> createFiretruckSystem(ArmParameters armParameters, ExtenderParameters extenderParameters) {
        // See Controls Engineering in the FIRST Robotics Competition by Tyler Veness
        double E_K_t = extenderParameters.m_gearbox.KtNMPerAmp;
        double E_G = extenderParameters.m_gearing;
        double E_R = extenderParameters.m_gearbox.rOhms;
        double E_r = extenderParameters.m_drumRadiusMeters;
        double E_m = extenderParameters.m_carriageMassKg;
        double E_K_v = extenderParameters.m_gearbox.KvRadPerSecPerVolt;
        
        return new LinearSystem<N4, N2, N2>(
            Matrix.mat(Nat.N4(), Nat.N4()).fill(
                0, 1, 0, 0,
                0, 0 /* handled in updateX */, 0, 0,
                0, 0, 0, 1,
                0, 
                    0, 
                    0,
                    - E_G * E_G * E_K_t / (E_R * E_r * E_r * E_m * E_K_v)
            ),
            Matrix.mat(Nat.N4(), Nat.N2()).fill()
                0, 0,
                0, 0,
                0, 0,
                0, E_G * E_K_t / (E_R * E_r * E_m)
            ),
            Matrix.mat(Nat.N2(), Nat.N4()).fill(
                1, 0, 0, 0,
                0, 0, 1, 0
            ),
            Matrix.mat(Nat.N2(), Nat.N2()).fill(
                0, 0,
                0, 0
            )
        );
    }

    private ArmParameters m_armParameters;
    private ExtenderParameters m_extenderParameters;
    private LinearSystem<N4, N2, N2> m_plant;

    public FiretruckSim(LinearSystem<N4, N2, N2> plant, ArmParameters armParameters,
            ExtenderParameters extenderParameters) {
        super(plant);
        this.m_plant = plant;
        this.m_armParameters = armParameters;
        this.m_extenderParameters = extenderParameters;
    }
    public FiretruckSim(ArmParameters armParameters, ExtenderParameters extenderParameters) {
        this(createFiretruckSystem(armParameters, extenderParameters), armParameters, extenderParameters);
    }


    public boolean wouldArmHitLowerLimit(double currentAngleRads) {
        return currentAngleRads <= m_armParameters.m_minAngleRads;
    }

    public boolean wouldArmHitUpperLimit(double currentAngleRads) {
        return currentAngleRads >= m_armParameters.m_maxAngleRads;
    }

    public boolean hasArmHitLowerLimit() {
        return wouldArmHitLowerLimit(getArmAngleRads());
    }

    public boolean hasArmHitUpperLimit() {
        return wouldArmHitUpperLimit(getArmAngleRads());
    }

    public double getArmAngleRads() {
        return m_y.get(0, 0);
    }

    private double getArmCOGDistanceMeters(double extenderDistanceMeters) {
        // D_fixed * (m_arm - m_carriage) + D_carriage * m_carriage = D_arm * m_arm;
        double fixedMassKg = m_armParameters.m_armMassKg - m_extenderParameters.m_carriageMassKg;
        double fixedCOGDistanceMeters = 
                (m_armParameters.m_armInitialCOGDistanceMeters * m_armParameters.m_armMassKg -
                m_extenderParameters.m_carriageMassKg * m_extenderParameters.m_carriageInitialCOGDistanceMeters) / 
                fixedMassKg;
        return fixedCOGDistanceMeters * fixedMassKg + extenderDistanceMeters * m_extenderParameters.m_carriageMassKg;
    }

    private double getArmMOI(double extenderDistanceMeters) {
        double extensionDeltaMeters = extenderDistanceMeters - m_extenderParameters.m_minDistanceMeters;
        return m_armParameters.m_retractedMOI +
                2 * m_extenderParameters.m_carriageInitialCOGDistanceMeters *
                        m_extenderParameters.m_carriageMassKg * extensionDeltaMeters +
                m_extenderParameters.m_carriageMassKg * extensionDeltaMeters * extensionDeltaMeters;
    }

    public boolean wouldExtenderHitInnerLimit(double currentDistanceMeters) {
        return currentDistanceMeters <= m_extenderParameters.m_minDistanceMeters;
    }

    public boolean wouldExtenderHitOuterLimit(double currentDistanceMeters) {
        return currentDistanceMeters >= m_extenderParameters.m_maxDistanceMeters;
    }

    public boolean hasExtenderHitInnerLimit() {
        return wouldExtenderHitInnerLimit(getExtenderDistanceMeters());
    }

    public boolean hasExtenderHitOuterLimit() {
        return wouldExtenderHitOuterLimit(getExtenderDistanceMeters());
    }

    public double getExtenderDistanceMeters() {
        return m_y.get(2, 0);
    }

    protected Matrix<N4, N1> updateX(Matrix<N4, N1> currentXhat, Matrix<N2, N1> currentU, double dtSeconds) {
        BiFunction<Matrix<N4, N1>, Matrix<N2, N1>, Matrix<N4, N1>> plantFunction =
            (Matrix<N4, N1> x, Matrix<N2, N1> u) -> {
                Matrix<N4, N1> xdot = m_plant.getA().times(x).plus(m_plant.getB().times(u));
                
                // See Controls Engineering in the FIRST Robotics Competition by Tyler Veness
                double A_G = m_armParameters.m_gearing;
                double A_K_t = m_armParameters.m_gearbox.KtNMPerAmp;
                double A_K_v = m_armParameters.m_gearbox.KvRadPerSecPerVolt;
                double A_R = m_armParameters.m_gearbox.rOhms;

                double A_J = getArmMOI(x.get(2, 0));

                // Set angular acceleration
                xdot.set(
                    1,
                    0,
                    - A_G * A_G * A_K_t / (A_K_v * A_R * A_J) * x.get(1, 0) +
                    A_G * A_K_t / (A_R * A_J) * u.get(0, 0)
                );

                double armCOGDistanceMeters = getArmCOGDistanceMeters(x.get(2, 0));
                // gravity
                xdot = xdot.plus(
                    VecBuilder.fill(
                        0,
                        - m_armParameters.m_armMassKg * 9.81 * armCOGDistanceMeters / A_J * Math.sin(x.get(0, 0)),
                        0,
                        9.81 * Math.cos(x.get(0, 0))
                    )
                );

                return xdot;
            };

        Matrix<N4, N1> updatedXhat = NumericalIntegration.rkdp(
            plantFunction,
            currentXhat,
            currentU,
            dtSeconds
        );

        if (wouldArmHitLowerLimit(updatedXhat.get(0, 0))) {
            updatedXhat.set(0, 0, m_armParameters.m_minAngleRads);
            updatedXhat.set(1, 0, 0);
        }

        if (wouldArmHitUpperLimit(updatedXhat.get(0, 0))) {
            updatedXhat.set(0, 0, m_armParameters.m_maxAngleRads);
            updatedXhat.set(1, 0, 0);
        }

        if (wouldExtenderHitInnerLimit(updatedXhat.get(2, 0))) {
            updatedXhat.set(2, 0, m_extenderParameters.m_minDistanceMeters);
            updatedXhat.set(3, 0, 0);
        }

        if (wouldExtenderHitOuterLimit(updatedXhat.get(2, 0))) {
            updatedXhat.set(2, 0, m_extenderParameters.m_maxDistanceMeters);
            updatedXhat.set(3, 0, 0);
        }
        return updatedXhat;
        
    }

   
    
}
