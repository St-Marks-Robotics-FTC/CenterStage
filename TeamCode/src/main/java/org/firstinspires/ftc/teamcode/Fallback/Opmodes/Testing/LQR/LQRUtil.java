package org.firstinspires.ftc.teamcode.Fallback.Opmodes.Testing.LQR;

import org.apache.commons.math3.linear.MatrixUtils;
import org.apache.commons.math3.linear.RealMatrix;
import org.ejml.simple.SimpleMatrix;

public class LQRUtil {

    public static RealMatrix SimpleReal(SimpleMatrix a) {
        RealMatrix b = MatrixUtils.createRealMatrix(a.numRows(), a.numCols());
        for (int i = 0; i<a.numRows(); i++) {
            for (int j = 0; j<a.numCols(); j++) {
                b.setEntry(i, j, a.get(i, j));
            }
        }
        return b;
    }
    public static SimpleMatrix RealSimple(RealMatrix a) {
        SimpleMatrix b = new SimpleMatrix(a.getRowDimension(), a.getColumnDimension());
        for (int i = 0; i<a.getRowDimension(); i++) {
            for (int j = 0; j<a.getColumnDimension(); j++) {
                b.set(i, j, a.getEntry(i, j));
            }
        }
        return b;
    }
}
