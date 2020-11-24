package us.ihmc.euclid.tools;

import us.ihmc.euclid.exceptions.SingularMatrixException;
import us.ihmc.euclid.matrix.Matrix3D;
import us.ihmc.euclid.matrix.interfaces.Matrix3DBasics;
import us.ihmc.euclid.matrix.interfaces.Matrix3DReadOnly;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.euclid.tuple4D.interfaces.QuaternionBasics;
import us.ihmc.euclid.tuple4D.interfaces.QuaternionReadOnly;

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
   static final double gamma = 3.0 + 2.0 * EuclidCoreTools.squareRoot(2.0);
   static final double cosPiOverEight = EuclidCoreTools.cos(Math.PI / 8.0);
   static final double sinPiOverEight = EuclidCoreTools.sin(Math.PI / 8.0);
   static final double sqrtTwoOverTwo = EuclidCoreTools.squareRoot(2.0) / 2.0;

   private final Matrix3D temp = new Matrix3D();
   private final SVD3DOutput output = new SVD3DOutput();

   private int maxIterations = 10000;
   private double tolerance = 1.0e-9;

   public SingularValueDecomposition3D()
   {
   }

   public void setMaxIterations(int maxIterations)
   {
      this.maxIterations = maxIterations;
   }

   public void setTolerance(double tolerance)
   {
      this.tolerance = tolerance;
   }

   public void decompose(Matrix3DReadOnly A)
   {
      computeV(A);
      computeUW(A);
   }

   public void transpose()
   {
      output.transpose();
   }

   private void computeV(Matrix3DReadOnly A)
   {
      Matrix3D S = temp;
      Matrix3DTools.multiplyTransposeLeft(A, A, S);

      int iteration = 0;

      output.V.setToZero();

      for (; iteration < maxIterations; iteration++)
      {
         // Find the element off-diagonal with the max absolute value.
         double a_01_abs = Math.abs(S.getM01());
         double a_02_abs = Math.abs(S.getM02());
         double a_12_abs = Math.abs(S.getM12());

         if (a_01_abs > a_02_abs)
         {
            if (a_01_abs > a_12_abs)
            {
               if (a_01_abs < tolerance)
                  break;
               approxGivensQuaternion(0, 1, S, output.V);
            }
            else
            {
               if (a_12_abs < tolerance)
                  break;
               approxGivensQuaternion(1, 2, S, output.V);
            }
         }
         else
         {
            if (a_02_abs > a_12_abs)
            {
               if (a_02_abs < tolerance)
                  break;
               approxGivensQuaternion(0, 2, S, output.V);
            }
            else
            {
               if (a_12_abs < tolerance)
                  break;
               approxGivensQuaternion(1, 2, S, output.V);
            }
         }
      }

      if (iteration > 0)
         output.V.normalize();
   }

   private static void approxGivensQuaternion(int p, int q, Matrix3DBasics SToUpdate, QuaternionBasics QToUpdate)
   {
      double s_pp, s_pq, s_qq;

      if (p == 0)
      {
         if (q == 1)
         {
            s_pp = SToUpdate.getM00();
            s_pq = SToUpdate.getM01();
            s_qq = SToUpdate.getM11();
         }
         else
         {
            s_pp = SToUpdate.getM00();
            s_pq = SToUpdate.getM02();
            s_qq = SToUpdate.getM22();
         }
      }
      else
      {
         s_pp = SToUpdate.getM11();
         s_pq = SToUpdate.getM12();
         s_qq = SToUpdate.getM22();
      }

      double ch = 2.0 * (s_pp - s_qq);
      double sh = s_pq;

      if (gamma * sh * sh < ch * ch)
      {
         /*
          * TODO The square root does not need to be accurate. In the paper, the authors suggest using the
          * SSE RSQRTPS, need to look into it.
          */
         double omega = 1.0 / EuclidCoreTools.squareRoot(ch * ch + sh * sh);
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
            prependGivensQuaternionZ(ch, sh, QToUpdate);
            applyJacobiGivensRotationZ(ch, sh, SToUpdate);
         }
         else
         { // Rotation along the y-axis
            prependGivensQuaternionY(ch, sh, QToUpdate);
            applyJacobiGivensRotationY(ch, sh, SToUpdate);
         }
      }
      else
      { // Rotation along the x-axis
         prependGivensQuaternionX(ch, sh, QToUpdate);
         applyJacobiGivensRotationX(ch, sh, SToUpdate);
      }
   }

   private void computeUW(Matrix3DReadOnly A)
   {
      Matrix3D B = temp;
      computeB(A, output.V, B);
      sortBColumns(B, output.V);
      output.U.setToZero();

      boolean isUquatInitialized = false;

      if (!EuclidCoreTools.isZero(B.getM10(), tolerance) || B.getM00() < 0.0)
      {
         qrGivensQuaternion(1, 0, B, output.U, tolerance);
         isUquatInitialized = true;
      }

      if (!EuclidCoreTools.isZero(B.getM20(), tolerance) || B.getM11() < 0.0)
      {
         qrGivensQuaternion(2, 0, B, output.U, tolerance);
         isUquatInitialized = true;
      }

      if (!EuclidCoreTools.isZero(B.getM21(), tolerance) || B.getM11() < 0.0)
      {
         qrGivensQuaternion(2, 1, B, output.U, tolerance);
         isUquatInitialized = true;
      }

      output.W.set(B.getM00(), B.getM11(), B.getM22());

      if (isUquatInitialized)
         output.U.normalize();
   }

   private static void computeB(Matrix3DReadOnly A, QuaternionReadOnly V, Matrix3DBasics BToPack)
   {
      double qx = V.getX();
      double qy = V.getY();
      double qz = V.getZ();
      double qs = V.getS();

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

      double b00 = A.getM00() * m00 + A.getM01() * m10 + A.getM02() * m20;
      double b01 = A.getM00() * m01 + A.getM01() * m11 + A.getM02() * m21;
      double b02 = A.getM00() * m02 + A.getM01() * m12 + A.getM02() * m22;
      double b10 = A.getM10() * m00 + A.getM11() * m10 + A.getM12() * m20;
      double b11 = A.getM10() * m01 + A.getM11() * m11 + A.getM12() * m21;
      double b12 = A.getM10() * m02 + A.getM11() * m12 + A.getM12() * m22;
      double b20 = A.getM20() * m00 + A.getM21() * m10 + A.getM22() * m20;
      double b21 = A.getM20() * m01 + A.getM21() * m11 + A.getM22() * m21;
      double b22 = A.getM20() * m02 + A.getM21() * m12 + A.getM22() * m22;
      BToPack.set(b00, b01, b02, b10, b11, b12, b20, b21, b22);
   }

   private static void qrGivensQuaternion(int p, int q, Matrix3DBasics B, QuaternionBasics UToUpdate, double epsilon)
   {
      double a1 = B.getElement(q, q);
      double a2 = B.getElement(p, q);

      double rho = EuclidCoreTools.squareRoot(a1 * a1 + a2 * a2);
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

      double omega = 1.0 / EuclidCoreTools.squareRoot(ch * ch + sh * sh);
      ch *= omega;
      sh *= omega;

      if (q == 0)
      {
         if (p == 1)
         { // Rotation along the z-axis
            appendGivensQuaternionZ(ch, sh, UToUpdate);
            applyQRGivensRotationZ(ch, sh, B);
         }
         else // p == 2
         { // Rotation along the y-axis
            appendGivensQuatenrionY(ch, sh, UToUpdate);
            applyQRGivensRotationY(ch, sh, B);
         }
      }
      else
      { // Rotation along the x-axis
         appendGivensQuaternionX(ch, sh, UToUpdate);
         applyQRGivensRotation(ch, sh, B);
      }
   }

   static void applyJacobiGivensRotationX(double ch, double sh, Matrix3DBasics S)
   {
      double ch2 = ch * ch;
      double sh2 = sh * sh;

      double diag_a = ch2 + sh2;
      double diag_b = (ch2 - sh2) / diag_a;
      double off_diag = 2.0 * ch * sh / diag_a;

      double s00 = S.getM00();
      double s11 = S.getM11();
      double s22 = S.getM22();
      double s01 = S.getM01();
      double s02 = S.getM02();
      double s12 = S.getM12();

      double qTs11 = diag_b * s11 + off_diag * s12;
      double qTs22 = diag_b * s22 - off_diag * s12;
      double qTs12 = diag_b * s12 + off_diag * s22;
      double qTs21 = diag_b * s12 - off_diag * s11;

      double qTsq11 = qTs11 * diag_b + qTs12 * off_diag;
      double qTsq22 = qTs22 * diag_b - qTs21 * off_diag;
      double qTsq01 = s01 * diag_b + s02 * off_diag;
      double qTsq02 = s02 * diag_b - s01 * off_diag;
      double qTsq12 = qTs12 * diag_b - qTs11 * off_diag;
      S.set(s00, qTsq01, qTsq02, qTsq01, qTsq11, qTsq12, qTsq02, qTsq12, qTsq22);
   }

   static void applyJacobiGivensRotationY(double ch, double sh, Matrix3DBasics S)
   {
      double ch2 = ch * ch;
      double sh2 = sh * sh;

      double diag_a = ch2 + sh2;
      double diag_b = (ch2 - sh2) / diag_a;
      double off_diag = 2.0 * ch * sh / diag_a;

      double s00 = S.getM00();
      double s11 = S.getM11();
      double s22 = S.getM22();
      double s01 = S.getM01();
      double s02 = S.getM02();
      double s12 = S.getM12();

      double qTs00 = diag_b * s00 + off_diag * s02;
      double qTs01 = diag_b * s01 + off_diag * s12;
      double qTs02 = diag_b * s02 + off_diag * s22;
      double qTs20 = diag_b * s02 - off_diag * s00;
      double qTs22 = diag_b * s22 - off_diag * s02;

      double qTsq00 = diag_b * qTs00 + off_diag * qTs02;
      double qTsq02 = diag_b * qTs02 - off_diag * qTs00;
      double qTsq12 = diag_b * s12 - off_diag * s01;
      double qTsq22 = diag_b * qTs22 - off_diag * qTs20;
      S.set(qTsq00, qTs01, qTsq02, qTs01, s11, qTsq12, qTsq02, qTsq12, qTsq22);
   }

   static void applyJacobiGivensRotationZ(double ch, double sh, Matrix3DBasics S)
   {
      double ch2 = ch * ch;
      double sh2 = sh * sh;

      double diag_a = ch2 + sh2;
      double diag_b = (ch2 - sh2) / diag_a;
      double off_diag = 2.0 * ch * sh / diag_a;

      double s00 = S.getM00();
      double s11 = S.getM11();
      double s22 = S.getM22();
      double s01 = S.getM01();
      double s02 = S.getM02();
      double s12 = S.getM12();

      double qTs00 = diag_b * s00 + off_diag * s01;
      double qTs01 = diag_b * s01 + off_diag * s11;
      double qTs02 = diag_b * s02 + off_diag * s12;
      double qTs10 = diag_b * s01 - off_diag * s00;
      double qTs11 = diag_b * s11 - off_diag * s01;
      double qTs12 = diag_b * s12 - off_diag * s02;
      double qTsq00 = diag_b * qTs00 + off_diag * qTs01;
      double qTsq01 = diag_b * qTs01 - off_diag * qTs00;
      double qTsq11 = diag_b * qTs11 - off_diag * qTs10;
      S.set(qTsq00, qTsq01, qTs02, qTsq01, qTsq11, qTs12, qTs02, qTs12, s22);
   }

   private static void applyQRGivensRotationZ(double ch, double sh, Matrix3DBasics BToUpdate)
   {
      double ch2 = ch * ch;
      double sh2 = sh * sh;

      double diag_a = ch2 + sh2;
      double diag_b = (ch2 - sh2) / diag_a;
      double off_diag = 2.0 * ch * sh / diag_a;
      double b00 = diag_b * BToUpdate.getM00() + off_diag * BToUpdate.getM10();
      double b01 = diag_b * BToUpdate.getM01() + off_diag * BToUpdate.getM11();
      double b02 = diag_b * BToUpdate.getM02() + off_diag * BToUpdate.getM12();
      double b10 = diag_b * BToUpdate.getM10() - off_diag * BToUpdate.getM00();
      double b11 = diag_b * BToUpdate.getM11() - off_diag * BToUpdate.getM01();
      double b12 = diag_b * BToUpdate.getM12() - off_diag * BToUpdate.getM02();
      BToUpdate.set(b00, b01, b02, b10, b11, b12, BToUpdate.getM20(), BToUpdate.getM21(), BToUpdate.getM22());
   }

   private static void applyQRGivensRotationY(double ch, double sh, Matrix3DBasics BToUpdate)
   {
      double ch2 = ch * ch;
      double sh2 = sh * sh;

      double diag_a = ch2 + sh2;
      double diag_b = (ch2 - sh2) / diag_a;
      double off_diag = 2.0 * ch * sh / diag_a;
      double b00 = diag_b * BToUpdate.getM00() + off_diag * BToUpdate.getM20();
      double b01 = diag_b * BToUpdate.getM01() + off_diag * BToUpdate.getM21();
      double b02 = diag_b * BToUpdate.getM02() + off_diag * BToUpdate.getM22();
      double b20 = diag_b * BToUpdate.getM20() - off_diag * BToUpdate.getM00();
      double b21 = diag_b * BToUpdate.getM21() - off_diag * BToUpdate.getM01();
      double b22 = diag_b * BToUpdate.getM22() - off_diag * BToUpdate.getM02();
      BToUpdate.set(b00, b01, b02, BToUpdate.getM10(), BToUpdate.getM11(), BToUpdate.getM12(), b20, b21, b22);
   }

   private static void applyQRGivensRotation(double ch, double sh, Matrix3DBasics BToUpdate)
   {
      double ch2 = ch * ch;
      double sh2 = sh * sh;

      double diag_a = ch2 + sh2;
      double diag_b = (ch2 - sh2) / diag_a;
      double off_diag = 2.0 * ch * sh / diag_a;
      double b10 = diag_b * BToUpdate.getM10() + off_diag * BToUpdate.getM20();
      double b11 = diag_b * BToUpdate.getM11() + off_diag * BToUpdate.getM21();
      double b12 = diag_b * BToUpdate.getM12() + off_diag * BToUpdate.getM22();
      double b20 = diag_b * BToUpdate.getM20() - off_diag * BToUpdate.getM10();
      double b21 = diag_b * BToUpdate.getM21() - off_diag * BToUpdate.getM11();
      double b22 = diag_b * BToUpdate.getM22() - off_diag * BToUpdate.getM12();
      BToUpdate.set(BToUpdate.getM00(), BToUpdate.getM01(), BToUpdate.getM02(), b10, b11, b12, b20, b21, b22);
   }

   private static void prependGivensQuaternionX(double ch, double sh, QuaternionBasics V)
   {
      double vx = V.getX();
      double vy = V.getY();
      double vz = V.getZ();
      double vs = V.getS();
      V.setUnsafe(vx * ch + vs * sh, vy * ch + vz * sh, vz * ch - vy * sh, vs * ch - vx * sh);
   }

   private static void prependGivensQuaternionY(double ch, double sh, QuaternionBasics V)
   {
      double vx = V.getX();
      double vy = V.getY();
      double vz = V.getZ();
      double vs = V.getS();
      V.setUnsafe(vz * sh + vx * ch, vy * ch - vs * sh, vz * ch - vx * sh, vs * ch + vy * sh);
   }

   private static void prependGivensQuaternionZ(double ch, double sh, QuaternionBasics V)
   {
      double vx = V.getX();
      double vy = V.getY();
      double vz = V.getZ();
      double vs = V.getS();
      V.setUnsafe(vx * ch + vy * sh, vy * ch - vx * sh, vz * ch + vs * sh, vs * ch - vz * sh);
   }

   private static void appendGivensQuaternionX(double ch, double sh, QuaternionBasics U)
   {
      double ux = U.getX();
      double uy = U.getY();
      double uz = U.getZ();
      double us = U.getS();
      U.setUnsafe(us * sh + ux * ch, uy * ch + uz * sh, uz * ch - uy * sh, us * ch - ux * sh);
   }

   private static void appendGivensQuatenrionY(double ch, double sh, QuaternionBasics U)
   {
      double ux = U.getX();
      double uy = U.getY();
      double uz = U.getZ();
      double us = U.getS();
      U.setUnsafe(ux * ch + uz * sh, uy * ch - us * sh, uz * ch - ux * sh, us * ch + uy * sh);
   }

   private static void appendGivensQuaternionZ(double ch, double sh, QuaternionBasics U)
   {
      double ux = U.getX();
      double uy = U.getY();
      double uz = U.getZ();
      double us = U.getS();
      U.setUnsafe(ux * ch + uy * sh, uy * ch - ux * sh, us * sh + uz * ch, us * ch - uz * sh);
   }

   static void sortBColumns(Matrix3DBasics B, QuaternionBasics Vquat)
   {
      double rho0 = EuclidCoreTools.normSquared(B.getM00(), B.getM10(), B.getM20());
      double rho1 = EuclidCoreTools.normSquared(B.getM01(), B.getM11(), B.getM21());
      double rho2 = EuclidCoreTools.normSquared(B.getM02(), B.getM12(), B.getM22());

      double qx = Vquat.getX();
      double qy = Vquat.getY();
      double qz = Vquat.getZ();
      double qs = Vquat.getS();

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
               B.set(B.getM00(), B.getM02(), -B.getM01(),
                     B.getM10(), B.getM12(), -B.getM11(),
                     B.getM20(), B.getM22(), -B.getM21());
               Vquat.setUnsafe(sqrtTwoOverTwo * (qs + qx),
                               sqrtTwoOverTwo * (qy + qz),
                               sqrtTwoOverTwo * (qz - qy),
                               sqrtTwoOverTwo * (qs - qx));
            }
         }
         else
         { // 2 > 0 > 1
            B.set(B.getM02(), B.getM00(), B.getM01(),
                  B.getM12(), B.getM10(), B.getM11(),
                  B.getM22(), B.getM20(), B.getM21());
            Vquat.setUnsafe(0.5 * (-qs + qx - qy + qz),
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
               B.set(B.getM01(), -B.getM00(), B.getM02(),
                     B.getM11(), -B.getM10(), B.getM12(),
                     B.getM21(), -B.getM20(), B.getM22());
               Vquat.setUnsafe(sqrtTwoOverTwo * (qx + qy),
                               sqrtTwoOverTwo * (qy - qx),
                               sqrtTwoOverTwo * (qs + qz),
                               sqrtTwoOverTwo * (qs - qz));
            }
            else
            { // 1 > 2 > 0
               B.set(B.getM01(), B.getM02(), B.getM00(),
                     B.getM11(), B.getM12(), B.getM10(),
                     B.getM21(), B.getM22(), B.getM20());
               Vquat.setUnsafe(0.5 * (qs + qx + qy - qz),
                               0.5 * (qs - qx + qy + qz),
                               0.5 * (qs + qx - qy + qz),
                               0.5 * (qs - qx - qy - qz));
            }
         }
         else
         { // 2 > 1 > 0 
            B.set(B.getM02(), B.getM01(), -B.getM00(),
                  B.getM12(), B.getM11(), -B.getM10(),
                  B.getM22(), B.getM21(), -B.getM20());
            Vquat.setUnsafe(sqrtTwoOverTwo * (qx + qz),
                            sqrtTwoOverTwo * (qy - qs),
                            sqrtTwoOverTwo * (qz - qx),
                            sqrtTwoOverTwo * (qs + qy));
         }
      }
      // @formatter:on
   }

   public SVD3DOutput getOutput()
   {
      return output;
   }

   public Quaternion getU()
   {
      return output.getU();
   }

   public Vector3D getW()
   {
      return output.getW();
   }

   public Matrix3DBasics getW(Matrix3DBasics W)
   {
      return output.getW(W);
   }

   public Quaternion getV()
   {
      return output.getV();
   }

   public double getTolerance()
   {
      return tolerance;
   }

   public static class SVD3DOutput
   {
      private final Quaternion U = new Quaternion();
      private final Vector3D W = new Vector3D();
      private final Quaternion V = new Quaternion();

      public void set(SVD3DOutput other)
      {
         U.set(other.U);
         W.set(other.W);
         V.set(other.V);
      }

      public void setIdentity()
      {
         U.setToZero();
         W.set(1.0, 1.0, 1.0);
         V.setToZero();
      }

      public void setToNaN()
      {
         U.setToNaN();
         W.setToNaN();
         V.setToNaN();
      }

      public void transpose()
      {
         double vx = V.getX();
         double vy = V.getY();
         double vz = V.getZ();
         double vs = V.getS();
         V.set(U);
         U.setUnsafe(vx, vy, vz, vs);
      }

      public void invert()
      {
         if (W.getX() < Matrix3DTools.EPS_INVERT || W.getY() < Matrix3DTools.EPS_INVERT || Math.abs(W.getZ()) < Matrix3DTools.EPS_INVERT)
            throw new SingularMatrixException(W.getX(), 0, 0, 0, W.getY(), 0, 0, 0, W.getZ());
         transpose();
         W.setX(1.0 / W.getX());
         W.setY(1.0 / W.getY());
         W.setZ(1.0 / W.getZ());
      }

      public Quaternion getU()
      {
         return U;
      }

      public Vector3D getW()
      {
         return W;
      }

      public Matrix3DBasics getW(Matrix3DBasics W)
      {
         if (W == null)
            W = new Matrix3D();
         W.setToDiagonal(this.W);
         return W;
      }

      public Quaternion getV()
      {
         return V;
      }

      @Override
      public String toString()
      {
         return "U = " + U + ", W = " + W + ", V = " + V;
      }
   }
}
