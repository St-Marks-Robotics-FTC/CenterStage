package org.firstinspires.ftc.teamcode.jankbot.prototype.LQR;

import org.apache.commons.math3.linear.LUDecomposition;
import org.apache.commons.math3.linear.MatrixUtils;
import org.apache.commons.math3.linear.RealMatrix;
import org.ejml.simple.SimpleMatrix;

public class LQR {
    private static SimpleMatrix a;
    private static SimpleMatrix b;
    private static SimpleMatrix Q;
    private static SimpleMatrix R;

    private static double dt = 0.02;

    public LQR(SimpleMatrix a, SimpleMatrix b, SimpleMatrix q, SimpleMatrix r, double dt) {
        this.a = a;
        this.b = b;
        this.Q = q;
        this.R = r;
        this.dt = dt;
    }

    public SimpleMatrix calculateK() {
        Discretization.discretizeAB(a, b, dt);
        RealMatrix P = solveDARE(
                LQRUtil.SimpleReal(Discretization.discA),
                LQRUtil.SimpleReal(Discretization.discB),
                LQRUtil.SimpleReal(Q),
                LQRUtil.SimpleReal(R));
        RealMatrix K = computeStateFeedbackGain(
                LQRUtil.SimpleReal(Discretization.discA),
                LQRUtil.SimpleReal(Discretization.discB),
                P,
                LQRUtil.SimpleReal(R));
        return LQRUtil.RealSimple(K);
    }

    public static RealMatrix solveDARE(RealMatrix A, RealMatrix B, RealMatrix Q, RealMatrix R) {
        int n = A.getRowDimension();
        RealMatrix identity = MatrixUtils.createRealIdentityMatrix(n);

        RealMatrix part1 = A.transpose().multiply(identity).multiply(A);
        RealMatrix part2 = A.transpose().multiply(identity).multiply(B);
        RealMatrix part3 = B.transpose().multiply(identity).multiply(B).add(R);
        RealMatrix part4 = B.transpose().multiply(identity).multiply(A);

        RealMatrix inversePart3 = new LUDecomposition(part3).getSolver().getInverse();

        RealMatrix P = new LUDecomposition(part3).getSolver().getInverse().multiply(part1.subtract(part2.multiply(inversePart3).multiply(part4))).add(Q);

        return P;
    }

    public static RealMatrix computeStateFeedbackGain(RealMatrix A, RealMatrix B, RealMatrix P, RealMatrix R) {
        RealMatrix term1 = B.transpose().multiply(P).multiply(B).add(R);
        RealMatrix inverseTerm1 = new LUDecomposition(term1).getSolver().getInverse();
        RealMatrix term2 = B.transpose().multiply(P).multiply(A);

        RealMatrix K = inverseTerm1.multiply(term2);

        return K;
    }
}