package us.ihmc.euclid.tools;

import us.ihmc.euclid.matrix.Matrix3D;
import us.ihmc.euclid.matrix.interfaces.Matrix3DBasics;
import us.ihmc.euclid.matrix.interfaces.Matrix3DReadOnly;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DBasics;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.euclid.tuple4D.interfaces.QuaternionBasics;
import us.ihmc.euclid.tuple4D.interfaces.QuaternionReadOnly;

/**
 * Calculator for computing the eigenvalues and eigenvectors of a symmetric 3D matrix <tt>A</tt>.
 * Eigenvalues and eigenvectors have the following property:
 * 
 * <pre>
 * A*v = &lambda;*v
 * </pre>
 * 
 * where A is a square matrix and v is an eigenvector associated with the eigenvalue &lambda;.
 * 
 * @author Sylvain Bertrand
 */
public class SymmetricEigenDecomposition3D
{
   static final double sqrtTwoOverTwo = EuclidCoreTools.squareRoot(2.0) / 2.0;

   private final Matrix3D A_internal = new Matrix3D();
   private final Quaternion Qquat = new Quaternion();
   private final Eigen3DOutput output = new Eigen3DOutput();

   private int maxIterations = 25;
   private double tolerance = 1.0e-13;

   /**
    * Creates a new calculator ready to be used.
    */
   public SymmetricEigenDecomposition3D()
   {
   }

   /**
    * Sets the maximum number of iterations for the first stage of the decomposition.
    *
    * @param maxIterations the new maximum number of iterations for the next decompositions. Default
    *                      value is {@code 25}.
    */
   public void setMaxIterations(int maxIterations)
   {
      this.maxIterations = maxIterations;
   }

   /**
    * Sets the tolerance used internally, lower value means higher accuracy but more iterations.
    *
    * @param tolerance the new tolerance to used for the next decompositions. Default value
    *                  {@code 1.0e-13}.
    */
   public void setTolerance(double tolerance)
   {
      this.tolerance = tolerance;
   }

   /**
    * Performs a decomposition of the given symmetric matrix {@code A}.
    * <p>
    * If the given matrix is not symmetric, the decomposition automatically fails and returns
    * {@code false}.
    * </p>
    *
    * @param A the matrix to be decomposed. Not modified.
    * @return whether the algorithm succeeded or not.
    */
   public boolean decompose(Matrix3DReadOnly A)
   {
      if (!A.isMatrixSymmetric(tolerance))
         return false;

      double max = A.maxAbsElement();
      initialize(A, 1.0 / max);
      computeQ(A_internal);
      output.eigenValues.set(A_internal.getM00(), A_internal.getM11(), A_internal.getM22());
      sortEigenValues(output.eigenValues, Qquat);
      toMatrix3D(Qquat, output.eigenVector0, output.eigenVector1, output.eigenVector2);
      output.eigenValues.scale(max);
      return true;
   }

   private void initialize(Matrix3DReadOnly A, double scale)
   {
      double a00 = A.getM00() * scale;
      double a11 = A.getM11() * scale;
      double a22 = A.getM22() * scale;

      double offScale = 0.5 * scale;
      double a01 = offScale * (A.getM01() + A.getM10());
      double a02 = offScale * (A.getM02() + A.getM20());
      double a12 = offScale * (A.getM12() + A.getM21());

      A_internal.set(a00, a01, a02, a01, a11, a12, a02, a12, a22);
   }

   /**
    * Redirection to
    * {@link SingularValueDecomposition3D#computeV(Matrix3DBasics, QuaternionBasics, int, double)} to
    * decompose the matrix.
    * 
    * @param A the matrix to decompose. Modified.
    */
   private void computeQ(Matrix3DBasics A)
   {
      SingularValueDecomposition3D.computeV(A, Qquat, maxIterations, tolerance);
   }

   /**
    * Sorts the eigen values stored in {@code lambda} in descending order and update {@code Q} to
    * maintain the equality <tt>A = Q &Lambda; Q<sup>T</sup></tt>.
    */
   private static void sortEigenValues(Vector3DBasics lambda, QuaternionBasics Qquat)
   {
      double rho0 = Math.abs(lambda.getX());
      double rho1 = Math.abs(lambda.getY());
      double rho2 = Math.abs(lambda.getZ());

      double qx = Qquat.getX();
      double qy = Qquat.getY();
      double qz = Qquat.getZ();
      double qs = Qquat.getS();

      // @formatter:off
      if (rho0 > rho1)
      {
         if (rho0 > rho2)
         {
            if (rho1 > rho2)
            { // 0 > 1 > 2
              // Do nothing
            }
            else
            { // 0 > 2 > 1
               lambda.set(lambda.getX(), lambda.getZ(), lambda.getY());
               Qquat.setUnsafe(sqrtTwoOverTwo * (qs + qx),
                               sqrtTwoOverTwo * (qy + qz),
                               sqrtTwoOverTwo * (qz - qy),
                               sqrtTwoOverTwo * (qs - qx));
            }
         }
         else
         { // 2 > 0 > 1
            lambda.set(lambda.getZ(), lambda.getX(), lambda.getY());
            Qquat.setUnsafe(0.5 * (-qs + qx - qy + qz),
                            0.5 * (-qs + qx + qy - qz),
                            0.5 * (-qs - qx + qy + qz),
                            0.5 * ( qs + qx + qy + qz));
         }
      }
      else
      {
         if (rho1 > rho2)
         {
            if (rho0 > rho2)
            { // 1 > 0 > 2
               lambda.set(lambda.getY(), lambda.getX(), lambda.getZ());
               Qquat.setUnsafe(sqrtTwoOverTwo * (qx + qy),
                               sqrtTwoOverTwo * (qy - qx),
                               sqrtTwoOverTwo * (qs + qz),
                               sqrtTwoOverTwo * (qs - qz));
            }
            else
            { // 1 > 2 > 0
               lambda.set(lambda.getY(), lambda.getZ(), lambda.getX());
               Qquat.setUnsafe(0.5 * (qs + qx + qy - qz),
                               0.5 * (qs - qx + qy + qz),
                               0.5 * (qs + qx - qy + qz),
                               0.5 * (qs - qx - qy - qz));
            }
         }
         else
         { // 2 > 1 > 0 
            lambda.set(lambda.getZ(), lambda.getY(), lambda.getX());
            Qquat.setUnsafe(sqrtTwoOverTwo * (qx + qz),
                            sqrtTwoOverTwo * (qy - qs),
                            sqrtTwoOverTwo * (qz - qx),
                            sqrtTwoOverTwo * (qs + qy));
         }
      }
      // @formatter:on
   }

   private static void toMatrix3D(QuaternionReadOnly q, Vector3DBasics v0, Vector3DBasics v1, Vector3DBasics v2)
   {
      double qx = q.getX();
      double qy = q.getY();
      double qz = q.getZ();
      double qs = q.getS();

      double yy2 = 2.0 * qy * qy;
      double zz2 = 2.0 * qz * qz;
      double xx2 = 2.0 * qx * qx;
      double xy2 = 2.0 * qx * qy;
      double sz2 = 2.0 * qs * qz;
      double xz2 = 2.0 * qx * qz;
      double sy2 = 2.0 * qs * qy;
      double yz2 = 2.0 * qy * qz;
      double sx2 = 2.0 * qs * qx;

      double m00 = 1.0 - yy2 - zz2;
      double m01 = xy2 - sz2;
      double m02 = xz2 + sy2;
      double m10 = xy2 + sz2;
      double m11 = 1.0 - xx2 - zz2;
      double m12 = yz2 - sx2;
      double m20 = xz2 - sy2;
      double m21 = yz2 + sx2;
      double m22 = 1.0 - xx2 - yy2;
      v0.set(m00, m10, m20);
      v1.set(m01, m11, m21);
      v2.set(m02, m12, m22);
   }

   /**
    * Returns the i<sup>th</sup> eigenvector.
    * 
    * @param index the index&in;[0;2] of the eigenvector to get.
    * @return the i<sup>th</sup> eigenvector.
    */
   public Vector3D getEigenVector(int index)
   {
      return output.getEigenVector(index);
   }

   /**
    * Returns the three eigenvectors in a matrix format where each column represents an eigenvector.
    * 
    * @param eigenVectorsToPack the matrix used to store the eigenvectors. If {@code null}, a new
    *                           matrix is created and returned.
    * @return the eigenvectors in a matrix format.
    */
   public Matrix3DBasics getEigenVectors(Matrix3DBasics eigenVectorsToPack)
   {
      return output.getEigenVectors(eigenVectorsToPack);
   }

   /**
    * Returns the i<sup>th</sup> eigenvalue.
    * <p>
    * Note that the eigenvalues are sorted in descending order.
    * </p>
    * 
    * @param index the index&in;[0;2] of the eigenvalue to get.
    * @return the i<sup>th</sup> eigenvalue.
    */
   public double getEigenValue(int index)
   {
      return output.getEigenValue(index);
   }

   /**
    * Returns the three eigenvalues in a vector format.
    * <p>
    * Note that the eigenvalues are sorted in descending order.
    * </p>
    * 
    * @return the eigenvalues in a vector format.
    */
   public Vector3D getEigenValues()
   {
      return output.getEigenValues();
   }

   /**
    * Class used to package the result of the decomposition.
    *
    * @author Sylvain Bertrand
    */
   public static class Eigen3DOutput
   {
      private final Vector3D eigenValues = new Vector3D();
      private final Vector3D eigenVector0 = new Vector3D();
      private final Vector3D eigenVector1 = new Vector3D();
      private final Vector3D eigenVector2 = new Vector3D();
      private final Vector3D[] eigenVectors = {eigenVector0, eigenVector1, eigenVector2};

      /**
       * Performs a deep copy of {@code other} into {@code this}.
       *
       * @param other the other output to copy. Not modified.
       */
      public void set(Eigen3DOutput other)
      {
         eigenValues.set(other.eigenValues);
         eigenVector0.set(other.eigenVector0);
         eigenVector1.set(other.eigenVector1);
         eigenVector2.set(other.eigenVector2);
      }

      /**
       * Resets this output such that <tt>Q &Lambda; Q<sup>T</sup> = I</tt>.
       */
      public void setIdentity()
      {
         eigenValues.set(1.0, 1.0, 1.0);
         eigenVector0.set(1.0, 0.0, 0.0);
         eigenVector1.set(0.0, 1.0, 0.0);
         eigenVector2.set(0.0, 0.0, 1.0);
      }

      /**
       * Sets eigenvalues and eigenvectors to NaN.
       */
      public void setToNaN()
      {
         eigenValues.setToNaN();
         eigenVector0.setToNaN();
         eigenVector1.setToNaN();
         eigenVector2.setToNaN();
      }

      /**
       * Returns the i<sup>th</sup> eigenvector.
       * 
       * @param index the index&in;[0;2] of the eigenvector to get.
       * @return the i<sup>th</sup> eigenvector.
       */
      public Vector3D getEigenVector(int index)
      {
         return eigenVectors[index];
      }

      /**
       * Returns the three eigenvectors in a matrix format where each column represents an eigenvector.
       * 
       * @param eigenVectorsToPack the matrix used to store the eigenvectors. If {@code null}, a new
       *                           matrix is created and returned.
       * @return the eigenvectors in a matrix format.
       */
      public Matrix3DBasics getEigenVectors(Matrix3DBasics eigenVectorsToPack)
      {
         if (eigenVectorsToPack == null)
            eigenVectorsToPack = new Matrix3D();
         eigenVectorsToPack.setColumns(eigenVector0, eigenVector1, eigenVector2);
         return eigenVectorsToPack;
      }

      /**
       * Returns the i<sup>th</sup> eigenvalue.
       * <p>
       * Note that the eigenvalues are sorted in descending order.
       * </p>
       * 
       * @param index the index&in;[0;2] of the eigenvalue to get.
       * @return the i<sup>th</sup> eigenvalue.
       */
      public double getEigenValue(int index)
      {
         return eigenValues.getElement(index);
      }

      /**
       * Returns the three eigenvalues in a vector format.
       * <p>
       * Note that the eigenvalues are sorted in descending order.
       * </p>
       * 
       * @return the eigenvalues in a vector format.
       */
      public Vector3D getEigenValues()
      {
         return eigenValues;
      }
   }
}
