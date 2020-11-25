package us.ihmc.euclid.tools;

import us.ihmc.euclid.matrix.Matrix3D;
import us.ihmc.euclid.matrix.interfaces.Matrix3DBasics;
import us.ihmc.euclid.matrix.interfaces.Matrix3DReadOnly;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DBasics;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.euclid.tuple4D.interfaces.QuaternionBasics;
import us.ihmc.euclid.tuple4D.interfaces.QuaternionReadOnly;

public class SymmetricEigenDecomposition3D
{
   static final double sqrtTwoOverTwo = EuclidCoreTools.squareRoot(2.0) / 2.0;

   private final Matrix3D A_internal = new Matrix3D();
   private final Quaternion Qquat = new Quaternion();
   private final Eigen3DOutput output = new Eigen3DOutput();

   private int maxIterations = 25;
   private double tolerance = 1.0e-13;

   public SymmetricEigenDecomposition3D()
   {
   }

   public SymmetricEigenDecomposition3D(double tolerance)
   {
      setTolerance(tolerance);
   }

   public SymmetricEigenDecomposition3D(double tolerance, int maxIterations)
   {
      setTolerance(tolerance);
      setMaxIterations(maxIterations);
   }

   public void setMaxIterations(int maxIterations)
   {
      this.maxIterations = maxIterations;
   }

   public void setTolerance(double tolerance)
   {
      this.tolerance = tolerance;
   }

   public boolean decompose(Matrix3DReadOnly A)
   {
      if (!A.isMatrixSymmetric(tolerance))
         return false;

      A_internal.set(A);
      double max = A_internal.maxAbsElement();
      A_internal.scale(1.0 / max);
      computeQ(A_internal);
      output.eigenValues.set(A_internal.getM00(), A_internal.getM11(), A_internal.getM22());
      sortEigenValues(output.eigenValues, Qquat);
      toMatrix3D(Qquat, output.eigenVector0, output.eigenVector1, output.eigenVector2);
      output.eigenValues.scale(max);
      return true;
   }

   private void computeQ(Matrix3DBasics A)
   {
      SingularValueDecomposition3D.computeV(A, Qquat, maxIterations, tolerance);
   }

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

   public Matrix3DBasics getEigenVectors(Matrix3DBasics eigenVectorsToPack)
   {
      return output.getEigenVectors(eigenVectorsToPack);
   }

   public Vector3D getEigenVector(int index)
   {
      return output.getEigenVector(index);
   }

   public Vector3D getEigenValues()
   {
      return output.getEigenValues();
   }

   public double getEigenValue(int index)
   {
      return output.getEigenValue(index);
   }

   public static class Eigen3DOutput
   {
      private final Vector3D eigenValues = new Vector3D();
      private final Vector3D eigenVector0 = new Vector3D();
      private final Vector3D eigenVector1 = new Vector3D();
      private final Vector3D eigenVector2 = new Vector3D();
      private final Vector3D[] eigenVectors = {eigenVector0, eigenVector1, eigenVector2};

      public Matrix3DBasics getEigenVectors(Matrix3DBasics eigenVectorsToPack)
      {
         if (eigenVectorsToPack == null)
            eigenVectorsToPack = new Matrix3D();
         eigenVectorsToPack.setColumns(eigenVector0, eigenVector1, eigenVector2);
         return eigenVectorsToPack;
      }

      public Vector3D getEigenVector(int index)
      {
         return eigenVectors[index];
      }

      public Vector3D getEigenValues()
      {
         return eigenValues;
      }

      public double getEigenValue(int index)
      {
         return eigenValues.getElement(index);
      }
   }
}
