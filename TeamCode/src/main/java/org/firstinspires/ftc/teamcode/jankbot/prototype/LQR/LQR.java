package org.firstinspires.ftc.teamcode.jankbot.prototype.LQR;

import android.opengl.Matrix;

import org.apache.commons.math3.linear.LUDecomposition;
import org.apache.commons.math3.linear.MatrixUtils;
import org.apache.commons.math3.linear.QRDecomposition;
import org.apache.commons.math3.linear.RealMatrix;
import org.ejml.simple.SimpleMatrix;
import org.opencv.core.Mat;

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
        //A+BR^-1B^T(A^-1)^TQ
        RealMatrix z11 = A.add(B.multiply(MatrixUtils.inverse(R).multiply(B.transpose().multiply(MatrixUtils.inverse(A).transpose().multiply(Q)))));
        //-BR^-1B^T(A^-1)^T
        RealMatrix z12 = B.multiply(MatrixUtils.inverse(R).multiply(B.transpose().multiply(MatrixUtils.inverse(A).transpose()))).scalarMultiply(-1);
        //-(A^-1)^TQ
        RealMatrix z21 = MatrixUtils.inverse(A).transpose().multiply(Q).scalarMultiply(-1);
        //(A^-1)^T
        RealMatrix z22 = MatrixUtils.inverse(A).transpose();
        //create Z matrix
        RealMatrix z = MatrixUtils.createRealMatrix(A.getRowDimension()*2, A.getRowDimension()*2);
        //assign blocks
        z = assignBlock(0, 0, z11, z);
        z = assignBlock(0, A.getColumnDimension(), z12, z);
        z = assignBlock(A.getRowDimension(), 0, z21, z);
        z = assignBlock(A.getRowDimension(), A.getColumnDimension(), z22, z);
        //QR Decomp
        //Z = U*T
        //Z = U^-1*U*T*U
        //Z = U^-1*Zk*U
        //Z = U*Zk*U^T
        // where Zk is a converged upper triangular matrix
        RealMatrix U = new QRDecomposition(z).getQ();
        RealMatrix U11 = retrieveBlock(0, 0, A.getRowDimension(), U);
        RealMatrix U21 = retrieveBlock(A.getRowDimension(), 0, A.getRowDimension(), U);
        RealMatrix P = U21.multiply(MatrixUtils.inverse(U11));
        return P;
    }

    public static RealMatrix computeStateFeedbackGain(RealMatrix A, RealMatrix B, RealMatrix P, RealMatrix R) {
        RealMatrix term1 = B.transpose().multiply(P).multiply(B).add(R);
        RealMatrix inverseTerm1 = MatrixUtils.inverse(term1);
        RealMatrix term2 = B.transpose().multiply(P).multiply(A);
        RealMatrix K = inverseTerm1.multiply(term2);
        return K;
    }

    //assign a onto b
    //x = rows y = col
    public static RealMatrix assignBlock(int x, int y, RealMatrix a, RealMatrix b) {
        for (int i = x; i<a.getRowDimension()+x; i++) {
            for (int j = y; j<a.getColumnDimension()+y; j++) {
                b.setEntry(i, j, a.getEntry(i-x, j-y));
            }
        }
        return b;
    }
    public static RealMatrix retrieveBlock(int x, int y, int size, RealMatrix b) {
        RealMatrix a = MatrixUtils.createRealMatrix(size, size);
        for (int i = x; i<a.getRowDimension()+x; i++) {
            for (int j = y; j<a.getColumnDimension()+y; j++) {
                a.setEntry(i-x, j-y, b.getEntry(i, j));
            }
        }
        return a;
    }
}