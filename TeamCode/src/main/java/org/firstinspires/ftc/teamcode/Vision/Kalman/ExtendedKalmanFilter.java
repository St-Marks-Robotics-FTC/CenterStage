package org.firstinspires.ftc.teamcode.Vision.Kalman;

import org.ejml.data.DMatrixRMaj;
import org.ejml.simple.SimpleMatrix;

public abstract class ExtendedKalmanFilter {
    // Q and R should most likely be diagonal matrices; only use non-diagonal matrices if have time
    // Note: even though x is a matrix, this really just means it is a single-row matrix, it is basically just a vector
    private SimpleMatrix Q; // process (predict step) noise cov matrix
    private SimpleMatrix R; // measurement (update step) noise cov matrix

    protected SimpleMatrix x;
    private SimpleMatrix P; // covariance error
    private long prevTime;

    private long deltaTime = 0;
    private double updateDelta = 0;

    public ExtendedKalmanFilter(DMatrixRMaj Q, DMatrixRMaj R, DMatrixRMaj initialEstimate) {
        this.Q = new SimpleMatrix(Q);
        this.R = new SimpleMatrix(R);
        // assume the error is 0 for now (idk if this is the right way to do it, or even if P is the right shape, will need to run Junit tests
        this.P = new SimpleMatrix(new DMatrixRMaj(3, 3));
        this.x = new SimpleMatrix(initialEstimate);
        this.prevTime = System.nanoTime();
    }

    public ExtendedKalmanFilter(DMatrixRMaj Q, DMatrixRMaj R, DMatrixRMaj initialEstimate, double updateDelta) {
        this(Q, R, initialEstimate);
        this.updateDelta = updateDelta;
    }

    /**
     * describes how the state evolves
     * @param u input vector
     * @return delta vector for state estimate based on u and current state estimate
     */
    public abstract SimpleMatrix f(DMatrixRMaj u);

    /**
     * @param u input vector
     * @return the jacobian of {@link #f} with respect to the state vector
     */
    public abstract SimpleMatrix F(DMatrixRMaj u);

    /**
     * Pretty much describes what the measurement should look like given the current state
     * @return innovation estimate based on measurement vector
     */
    public abstract SimpleMatrix h();

    /**
     * @return the jacobian of {@link #h} with respect to the state vector
     */
    public abstract SimpleMatrix H();

    /**
     * Runs the prediction step
     * @param u The input vector
     * @return The priori state estimate
     */
    public SimpleMatrix predict(DMatrixRMaj u) {
        // evolve the state according to the model
        // use the delta time to multiply by the state transition function
        this.x = this.x.plus(new SimpleMatrix(this.f(u)).scale(this.deltaTime));
        SimpleMatrix F = this.F(u);
        // update the error cov matrix based on F and previous error cov, as well as process noise; P should always increase during prediction and the main uncertainty comes from Q
        this.P = this.P.plus(F.mult(this.P).plus(F.transpose().mult(this.P).plus(this.Q)).scale(this.deltaTime));
        return this.x;
    }

    /**
     * Runs the update step
     * @param y The measurement/correction vector
     * @return The posteriori state estimate
     */
    public SimpleMatrix update(DMatrixRMaj y) {
        // jacobian of innovation function used for "linearization"
        SimpleMatrix H = this.H();
        // fun part: compute the kalman gain!
        SimpleMatrix K = this.P.mult(H.transpose()).mult(H.mult(this.P).mult(H).plus(this.R).invert()); // the hope is that the matrix inversion doesn't take much time complexity for a 3x3
        // apply the kalman gain with the innovation residual (error between actual measurements and "predicted" measurements) to obtain new state estimate
        this.x = this.x.plus(K.mult(new SimpleMatrix(y).minus(this.h())));
        // update the error cov once again; it SHOULD go down in this step if the sensors aren't literal garbage
        this.P = SimpleMatrix.identity(3).minus(K.mult(H)).mult(this.P);
        return this.x;
    }

    /**
     * Runs an iteration of the controller; in practice, the update step may or not be run
     * @param u The input vector
     * @param y The most recent measurement vector (could be cached, in which case it is ignored if the proper delta time has not elapsed)
     * @return The current state estimate
     */
    public SimpleMatrix iterate(DMatrixRMaj u, DMatrixRMaj y) {
        this.deltaTime = (System.nanoTime() - prevTime) / 1000; // dt is gonna be in terms of ms
        this.predict(u);
        if (this.deltaTime >= this.updateDelta) this.update(y);
        return this.x;
    }

    public SimpleMatrix getEstimate() {
        return this.x;
    }
}