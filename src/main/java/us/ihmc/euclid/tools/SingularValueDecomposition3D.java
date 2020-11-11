package us.ihmc.euclid.tools;

import us.ihmc.euclid.Axis3D;
import us.ihmc.euclid.matrix.Matrix3D;
import us.ihmc.euclid.matrix.interfaces.Matrix3DBasics;
import us.ihmc.euclid.matrix.interfaces.Matrix3DReadOnly;
import us.ihmc.euclid.rotationConversion.RotationMatrixConversion;
import us.ihmc.euclid.tuple4D.Vector4D;
import us.ihmc.euclid.tuple4D.interfaces.Vector4DBasics;

/**
 * SVD decomposition of a 3D matrix <tt>A</tt> as:
 * 
 * <pre>
 * A = U W V<sup>T</sup>
 * </pre>
 * 
 * From paper: <i>Computing the Singular Value Decomposition of 3 × 3 matrices with minimal
 * branching and elementary floating point operations</i>
 * 
 * @author Sylvain Bertrand
 */
public class SingularValueDecomposition3D
{
   private final Matrix3D temp = new Matrix3D();

   /**
    * Givens rotation used in the Jacobi eigenvalue iterative algorithm.
    */
   private final Matrix3D Qmat = new Matrix3D();
   /**
    * The Givens rotation matrix <tt>Q</tt> as a quaternion.
    */
   private final Vector4D Qquat = new Vector4D();

   /**
    * The second rotation of the SVD decomposition expressed as a quaternion.
    */
   private final Vector4D Vquat = new Vector4D();
   private final Matrix3D Vmat = new Matrix3D();

   private double sigma0, sigma1, sigma2;

   private final Vector4D Uquat = new Vector4D();
   private final Matrix3D Umat = new Matrix3D();

   private int maxIterations = 10000;
   private double tolerance = 1.0e-9;

   public SingularValueDecomposition3D()
   {
   }

   public void compute(Matrix3DReadOnly A)
   {
      computeV(A);
      computeUW(A);
   }

   private void computeV(Matrix3DReadOnly A)
   {
      Matrix3D S = temp;
      S.setAndMultiplyInner(A);

      int iteration = 0;

      for (; iteration < maxIterations; iteration++)
      {
         // Find the element off-diagonal with the max absolute value.
         double a_01_abs = Math.abs(S.getM01());
         double a_02_abs = Math.abs(S.getM02());
         double a_12_abs = Math.abs(S.getM12());
         double maxValue;
         int p, q;

         if (a_01_abs > a_02_abs)
         {
            if (a_01_abs > a_12_abs)
            {
               p = 0;
               q = 1;
               maxValue = a_01_abs;
            }
            else
            {
               p = 1;
               q = 2;
               maxValue = a_12_abs;
            }
         }
         else
         {
            if (a_02_abs > a_12_abs)
            {
               p = 0;
               q = 2;
               maxValue = a_02_abs;
            }
            else
            {
               p = 1;
               q = 2;
               maxValue = a_12_abs;
            }
         }

         // If the max value found is less than the tolerance, we done!
         if (maxValue <= tolerance)
            break;

         approxGivensQuaternion(p, q, S, Qquat, Qmat);

         // Updating S as: S = Q^T S Q
         S.preMultiplyTransposeOther(Qmat);
         S.multiply(Qmat);

         if (iteration == 0)
            Vquat.set(Qquat);
         else
            QuaternionTools.multiply(Vquat, Qquat, Vquat);
      }

      if (iteration == 0)
      {
         Vmat.setIdentity();
         Vquat.set(0, 0, 0, 1);
      }
      else
      {
         double norm = RotationMatrixConversion.convertQuaternionToMatrix(Vquat.getX(), Vquat.getY(), Vquat.getZ(), Vquat.getS(), Vmat);
         Vquat.scale(1.0 / norm);
      }
   }

   private static final double gamma = 3.0 + 2.0 * Math.sqrt(2.0);
   private static final double cosPiOverEight = Math.cos(Math.PI / 8.0);
   private static final double sinPiOverEight = Math.sin(Math.PI / 8.0);

   private static void approxGivensQuaternion(int p, int q, Matrix3DReadOnly S, Vector4DBasics givensQuaternionToPack, Matrix3DBasics givensRotationToPack)
   {
      double s_pp = S.getElement(p, p);
      double s_pq = S.getElement(p, q);
      double s_qq = S.getElement(q, q);

      double ch = 2.0 * (s_pp - s_qq);
      double sh = s_pq;

      if (gamma * sh * sh < ch * ch)
      {
         /*
          * TODO The square root does not need to be accurate. In the paper, the authors suggest using the
          * SSE RSQRTPS, need to look into it.
          */
         double omega = 1.0 / Math.sqrt(ch * ch + sh * sh);
         ch *= omega;
         sh *= omega;
      }
      else
      {
         ch = cosPiOverEight;
         sh = sinPiOverEight;
      }

      if (p == 0)
      {
         if (q == 1)
         { // Rotation along the z-axis
            packGivensRotation(Axis3D.Z, ch, sh, givensQuaternionToPack, givensRotationToPack);
         }
         else
         { // Rotation along the y-axis
            packGivensRotation(Axis3D.Y, ch, sh, givensQuaternionToPack, givensRotationToPack);
         }
      }
      else
      { // Rotation along the x-axis
         packGivensRotation(Axis3D.X, ch, sh, givensQuaternionToPack, givensRotationToPack);
      }
   }

   private static void packGivensRotation(Axis3D rotationAxis, double ch, double sh, Vector4DBasics givensQuaternionToPack, Matrix3DBasics givensRotationToPack)
   {
      double ch2 = ch * ch;
      double sh2 = sh * sh;

      double diag_a = ch2 + sh2;
      double diag_b = (ch2 - sh2) / diag_a;
      double off_diag = 2.0 * ch * sh / diag_a;
      diag_a = 1.0;

      switch (rotationAxis)
      {
         case X:
            givensQuaternionToPack.set(sh, 0, 0, ch);
            givensRotationToPack.set(diag_a, 0, 0, 0, diag_b, -off_diag, 0, off_diag, diag_b);
            break;
         case Y:
            // Kind off an oddity that the rotation needs to be inverted. 
            // Best guess so far is that sh is computed from the s_02 element which is the location for the +sin(angle) element of a rotation along the y_axis, while the other 2 rotations uses the -sin(angle) element.
            givensQuaternionToPack.set(0, -sh, 0, ch);
            givensRotationToPack.set(diag_b, 0, -off_diag, 0, diag_a, 0, off_diag, 0, diag_b);
            break;
         case Z:
            givensQuaternionToPack.set(0, 0, sh, ch);
            givensRotationToPack.set(diag_b, -off_diag, 0, off_diag, diag_b, 0, 0, 0, diag_a);
            break;

         default:
            throw new IllegalStateException("Unexpected value for Axis3D: " + rotationAxis);
      }
   }

   private void computeUW(Matrix3DReadOnly A)
   {
      Matrix3D B = temp;
      Matrix3DTools.multiply(A, Vmat, B);
      sortBColumns(B, Vmat, Vquat);

      boolean isUquatInitialized = false;

      if (!EuclidCoreTools.isZero(B.getM10(), tolerance) || B.getM00() < 0.0)
      {
         qrGivensQuaternion(1, 0, B, Qquat, Qmat, tolerance);
         B.preMultiplyTransposeOther(Qmat);
         Uquat.set(Qquat);
         isUquatInitialized = true;
      }

      if (!EuclidCoreTools.isZero(B.getM20(), tolerance) || B.getM11() < 0.0)
      {
         qrGivensQuaternion(2, 0, B, Qquat, Qmat, tolerance);
         B.preMultiplyTransposeOther(Qmat);
         if (isUquatInitialized)
            QuaternionTools.multiply(Uquat, Qquat, Uquat);
         else
            Uquat.set(Qquat);
         isUquatInitialized = true;
      }

      if (!EuclidCoreTools.isZero(B.getM21(), tolerance) || B.getM11() < 0.0)
      {
         qrGivensQuaternion(2, 1, B, Qquat, Qmat, tolerance);
         B.preMultiplyTransposeOther(Qmat);
         if (isUquatInitialized)
            QuaternionTools.multiply(Uquat, Qquat, Uquat);
         else
            Uquat.set(Qquat);
         isUquatInitialized = true;
      }

      if (isUquatInitialized)
      {
         double norm = RotationMatrixConversion.convertQuaternionToMatrix(Uquat.getX(), Uquat.getY(), Uquat.getZ(), Uquat.getS(), Umat);
         Uquat.scale(1.0 / norm);
      }
      else
      {
         Umat.setIdentity();
         Uquat.set(0, 0, 0, 1);
      }

      sigma0 = B.getM00();
      sigma1 = B.getM11();
      sigma2 = B.getM22();
   }

   private static void qrGivensQuaternion(int p, int q, Matrix3DReadOnly B, Vector4DBasics givensQuaternionToPack, Matrix3DBasics givensRotationToPack,
                                          double epsilon)
   {
      double a1 = B.getElement(q, q);
      double a2 = B.getElement(p, q);

      double rho = Math.sqrt(a1 * a1 + a2 * a2);
      double ch;
      double sh;

      if (a1 < 0.0)
      {
         ch = rho > epsilon ? a2 : 0.0;
         sh = -a1 + Math.max(rho, epsilon);
      }
      else
      {
         ch = a1 + Math.max(rho, epsilon);
         sh = rho > epsilon ? a2 : 0.0;

      }

      double omega = 1.0 / Math.sqrt(ch * ch + sh * sh);
      ch *= omega;
      sh *= omega;

      if (q == 0)
      {
         if (p == 1)
         { // Rotation along the z-axis
            packGivensRotation(Axis3D.Z, ch, sh, givensQuaternionToPack, givensRotationToPack);
         }
         else // p == 2
         { // Rotation along the y-axis
            packGivensRotation(Axis3D.Y, ch, sh, givensQuaternionToPack, givensRotationToPack);
         }
      }
      else
      { // Rotation along the x-axis
         packGivensRotation(Axis3D.X, ch, sh, givensQuaternionToPack, givensRotationToPack);
      }
   }

   private static void sortBColumns(Matrix3DBasics B, Matrix3DBasics Vmat, Vector4DBasics Vquat)
   {
      double rho0 = EuclidCoreTools.normSquared(B.getM00(), B.getM10(), B.getM20());
      double rho1 = EuclidCoreTools.normSquared(B.getM01(), B.getM11(), B.getM21());
      double rho2 = EuclidCoreTools.normSquared(B.getM02(), B.getM12(), B.getM22());

      if (rho0 < rho1)
      {
         swapColumns(0, true, 1, B);
         swapColumns(0, true, 1, Vmat);
         swapElements(0, 1, Vquat);
         double tmp = rho0;
         rho0 = rho1;
         rho1 = tmp;
      }

      if (rho0 < rho2)
      {
         swapColumns(0, true, 2, B);
         swapColumns(0, true, 2, Vmat);
         swapElements(0, 2, Vquat);
         double tmp = rho0;
         rho0 = rho2;
         rho2 = tmp;
      }

      if (rho1 < rho2)
      {
         swapColumns(1, true, 2, B);
         swapColumns(1, true, 2, Vmat);
         swapElements(1, 2, Vquat);
      }
   }

   static void swapColumns(int col1, boolean negateCol1, int col2, Matrix3DBasics matrixToSwapColumns)
   {
      if (col2 <= col1)
         throw new IllegalArgumentException("col2 is expected to be strictly greater than col1");

      double r0, r1, r2;

      if (col1 == 0)
      {
         r0 = matrixToSwapColumns.getM00();
         r1 = matrixToSwapColumns.getM10();
         r2 = matrixToSwapColumns.getM20();

         if (negateCol1)
         {
            r0 = -r0;
            r1 = -r1;
            r2 = -r2;
         }

         if (col2 == 1)
         {
            matrixToSwapColumns.setM00(matrixToSwapColumns.getM01());
            matrixToSwapColumns.setM10(matrixToSwapColumns.getM11());
            matrixToSwapColumns.setM20(matrixToSwapColumns.getM21());

            matrixToSwapColumns.setM01(r0);
            matrixToSwapColumns.setM11(r1);
            matrixToSwapColumns.setM21(r2);
         }
         else // col2 == 2
         {
            matrixToSwapColumns.setM00(matrixToSwapColumns.getM02());
            matrixToSwapColumns.setM10(matrixToSwapColumns.getM12());
            matrixToSwapColumns.setM20(matrixToSwapColumns.getM22());

            matrixToSwapColumns.setM02(r0);
            matrixToSwapColumns.setM12(r1);
            matrixToSwapColumns.setM22(r2);
         }
      }
      else // col1 == 1 & col2 == 2
      {
         r0 = matrixToSwapColumns.getM01();
         r1 = matrixToSwapColumns.getM11();
         r2 = matrixToSwapColumns.getM21();

         if (negateCol1)
         {
            r0 = -r0;
            r1 = -r1;
            r2 = -r2;
         }

         matrixToSwapColumns.setM01(matrixToSwapColumns.getM02());
         matrixToSwapColumns.setM11(matrixToSwapColumns.getM12());
         matrixToSwapColumns.setM21(matrixToSwapColumns.getM22());

         matrixToSwapColumns.setM02(r0);
         matrixToSwapColumns.setM12(r1);
         matrixToSwapColumns.setM22(r2);
      }
   }

   private static final double sqrtTwoOverTwo = Math.sqrt(2.0) / 2.0;

   static void swapElements(int c1, int c2, Vector4DBasics quaternion)
   {
      if (c2 <= c1)
         throw new IllegalArgumentException("col2 is expected to be strictly greater than col1");

      double q1x = quaternion.getX();
      double q1y = quaternion.getY();
      double q1z = quaternion.getZ();
      double q1s = quaternion.getS();

      if (c1 == 0)
      {
         if (c2 == 1)
         {
            QuaternionTools.multiplyImpl(q1x, q1y, q1z, q1s, false, 0, 0, sqrtTwoOverTwo, sqrtTwoOverTwo, false, quaternion);
         }
         else // c2 == 2
         {
            QuaternionTools.multiplyImpl(q1x, q1y, q1z, q1s, false, 0, -sqrtTwoOverTwo, 0, sqrtTwoOverTwo, false, quaternion);
         }
      }
      else // c1 == 1 & c2 == 2
      {
         QuaternionTools.multiplyImpl(q1x, q1y, q1z, q1s, false, sqrtTwoOverTwo, 0, 0, sqrtTwoOverTwo, false, quaternion);
      }
   }

   public Matrix3D getUmat()
   {
      return Umat;
   }

   public Vector4D getUquat()
   {
      return Uquat;
   }

   public Matrix3D getWmat()
   {
      return new Matrix3D(sigma0, 0, 0, 0, sigma1, 0, 0, 0, sigma2);
   }

   public Matrix3D getVmat()
   {
      return Vmat;
   }

   public Vector4D getVquat()
   {
      return Vquat;
   }

   public double getTolerance()
   {
      return tolerance;
   }
}
