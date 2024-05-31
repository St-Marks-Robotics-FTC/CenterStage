package org.firstinspires.ftc.teamcode.LebronBot.Opmodes.Testing.LQR;

import android.util.Log;

import org.apache.commons.math3.linear.MatrixUtils;
import org.apache.commons.math3.linear.QRDecomposition;
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
        //A+BR^-1B^T(A^-1)^TQ
        RealMatrix z11 = A.add(B.multiply(MatrixUtils.inverse(R).multiply(B.transpose().multiply(MatrixUtils.inverse(A).transpose().multiply(Q)))));
        Log.d("z11: ", Double.toString(z11.getEntry(0,0)));
        //-BR^-1B^T(A^-1)^T
        RealMatrix z12 = B.multiply(MatrixUtils.inverse(R).multiply(B.transpose().multiply(MatrixUtils.inverse(A).transpose()))).scalarMultiply(-1);
        Log.d("z12: ", Double.toString(z12.getEntry(0,0)));
        //-(A^-1)^TQ
        RealMatrix z21 = MatrixUtils.inverse(A).transpose().multiply(Q).scalarMultiply(-1);
        Log.d("z21: ", Double.toString(z21.getEntry(0,0)));
        //(A^-1)^T
        RealMatrix z22 = MatrixUtils.inverse(A).transpose();
        Log.d("z22: ", Double.toString(z22.getEntry(0,0)));
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
        Log.d("u11: ", Double.toString(U11.getEntry(0,0)));
        RealMatrix U21 = retrieveBlock(A.getRowDimension(), 0, A.getRowDimension(), U);
        Log.d("u21: ", Double.toString(U11.getEntry(0,0)));
        RealMatrix P = U21.multiply(MatrixUtils.inverse(U11));
        Log.d("P: ", Double.toString(P.getEntry(0,0)));
        return P;
    }

    public static RealMatrix computeStateFeedbackGain(RealMatrix A, RealMatrix B, RealMatrix P, RealMatrix R) {
        Log.d("A: ", Double.toString(A.getEntry(0,0)));
        Log.d("B:" , Double.toString(B.getEntry(0,0)));
        Log.d("B^t", Double.toString(B.transpose().getEntry(0,0)));
        Log.d("Pa: ", Double.toString(P.getEntry(0,0)));
        RealMatrix term1 = B.transpose().multiply(P).multiply(B).add(R);
        Log.d("term1: ", Double.toString(term1.getEntry(0,0)));
        RealMatrix inverseTerm1 = MatrixUtils.inverse(term1);
        RealMatrix term2 = B.transpose().multiply(P);
        Log.d("term2: ", Double.toString(term2.getEntry(0,0)));
        term2 = term2.multiply(A);
        Log.d("term2a: ", Double.toString(term2.getEntry(0,0)));
        RealMatrix K = inverseTerm1.multiply(term2);
        Log.d("K: ", Double.toString(K.getEntry(0,0)));
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