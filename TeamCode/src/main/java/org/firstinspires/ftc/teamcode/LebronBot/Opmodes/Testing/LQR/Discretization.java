package org.firstinspires.ftc.teamcode.LebronBot.Opmodes.Testing.LQR;

import android.util.Log;

import org.apache.commons.math3.linear.EigenDecomposition;
import org.apache.commons.math3.linear.MatrixUtils;
import org.apache.commons.math3.linear.RealMatrix;
import org.ejml.simple.SimpleMatrix;

public class Discretization {

    public static SimpleMatrix discA;
    public static SimpleMatrix discB;

    private Discretization() {
    }

    public static void discretizeAB(SimpleMatrix contA, SimpleMatrix contB, double dt) {
        //for reference dt should be about 20 ms
        discA = new SimpleMatrix(contA.numRows(), contA.numCols());
        discB = new SimpleMatrix(contB.numRows(), contB.numCols());
        int states = contA.numRows();
        int inputs = contB.numCols();
        SimpleMatrix M = new SimpleMatrix(states + inputs, states + inputs);
        for (int i = 0; i<states; i++) {
            for (int j = 0; j<states; j++) {
                M.set(i, j, contA.get(i, j));
            }
        }
//        for (int j = contA.numRows(); j<contA.numRows()+contB.numRows(); j++) {//cols(rows and columns are interchangable bc of square matrix)
//            for (int i = 0; i<states; i++) {// rows
//                M.set(i, j, contB.get(i, j-contA.numRows()));
//            }
//        }
        M.set(1,1, contB.get(0,0));
        Log.d("Ma: ", Double.toString(M.get(0,0)));
        Log.d("Mb: ", Double.toString(M.get(1,1)));
        SimpleMatrix phi = exp(M.scale(dt));
        Log.d("pha: ", Double.toString(phi.get(0,0)));
        Log.d("phb: ", Double.toString(phi.get(1,1)));
        discA.set(0,0, phi.get(0,0));
        discB.set(0,0, phi.get(1,1));
    }

    public static SimpleMatrix exp(SimpleMatrix x) {
        if (x.numRows()!=x.numCols()) {
            return x;
        }
        RealMatrix exp = LQRUtil.SimpleReal(x);
        EigenDecomposition eigenDecomposition = new EigenDecomposition(exp);
        RealMatrix result = eigenDecomposition.getV().multiply(
                MatrixUtils.createRealDiagonalMatrix(eigenDecomposition.getRealEigenvalues())
                        .scalarMultiply(Math.exp(1.0))
        ).multiply(eigenDecomposition.getV().transpose());
        SimpleMatrix ans = LQRUtil.RealSimple(result);
        return ans;
    }
}
