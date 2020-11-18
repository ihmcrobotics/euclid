package us.ihmc.euclid.tools;

import us.ihmc.euclid.matrix.Matrix3D;
import us.ihmc.euclid.matrix.RotationMatrix;
import us.ihmc.euclid.matrix.interfaces.CommonMatrix3DBasics;
import us.ihmc.euclid.matrix.interfaces.Matrix3DBasics;
import us.ihmc.euclid.matrix.interfaces.Matrix3DReadOnly;
import us.ihmc.euclid.matrix.interfaces.RotationMatrixBasics;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.euclid.tuple4D.Vector4D;
import us.ihmc.euclid.tuple4D.interfaces.QuaternionBasics;
import us.ihmc.euclid.tuple4D.interfaces.Tuple4DBasics;
import us.ihmc.euclid.tuple4D.interfaces.Tuple4DReadOnly;

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
   static final double gamma = 3.0 + 2.0 * Math.sqrt(2.0);
   static final double cosPiOverEight = Math.cos(Math.PI / 8.0);
   static final double sinPiOverEight = Math.sin(Math.PI / 8.0);
   static final double sqrtTwoOverTwo = Math.sqrt(2.0) / 2.0;

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

      output.Vquat.set(0, 0, 0, 1);

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
               approxGivensQuaternion(0, 1, S, output.Vquat);
            }
            else
            {
               if (a_12_abs < tolerance)
                  break;
               approxGivensQuaternion(1, 2, S, output.Vquat);
            }
         }
         else
         {
            if (a_02_abs > a_12_abs)
            {
               if (a_02_abs < tolerance)
                  break;
               approxGivensQuaternion(0, 2, S, output.Vquat);
            }
            else
            {
               if (a_12_abs < tolerance)
                  break;
               approxGivensQuaternion(1, 2, S, output.Vquat);
            }
         }
      }

      if (iteration > 0)
      {
         output.Vquat.normalize();
         toRotationMatrix(output.Vquat, output.Vmat);
      }
      else
      {
         output.Vmat.setIdentity();
      }
   }

   private static void approxGivensQuaternion(int p, int q, CommonMatrix3DBasics SToUpdate, Tuple4DBasics QToUpdate)
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
      Matrix3DTools.multiply(A, output.Vmat, B);
      sortBColumns(B, output.Vmat, output.Vquat);
      output.Uquat.set(0, 0, 0, 1);

      boolean isUquatInitialized = false;

      if (!EuclidCoreTools.isZero(B.getM10(), tolerance) || B.getM00() < 0.0)
      {
         qrGivensQuaternion(1, 0, B, output.Uquat, tolerance);
         isUquatInitialized = true;
      }

      if (!EuclidCoreTools.isZero(B.getM20(), tolerance) || B.getM11() < 0.0)
      {
         qrGivensQuaternion(2, 0, B, output.Uquat, tolerance);
         isUquatInitialized = true;
      }

      if (!EuclidCoreTools.isZero(B.getM21(), tolerance) || B.getM11() < 0.0)
      {
         qrGivensQuaternion(2, 1, B, output.Uquat, tolerance);
         isUquatInitialized = true;
      }

      output.sigma0 = B.getM00();
      output.sigma1 = B.getM11();
      output.sigma2 = B.getM22();

      if (isUquatInitialized)
      {
         output.Uquat.normalize();
         toRotationMatrix(output.Uquat, output.Umat);
      }
      else
      {
         output.Umat.setIdentity();
         output.Uquat.set(0, 0, 0, 1);
      }

   }

   private static void qrGivensQuaternion(int p, int q, CommonMatrix3DBasics B, Tuple4DBasics UToUpdate, double epsilon)
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

   static void applyJacobiGivensRotationX(double ch, double sh, CommonMatrix3DBasics S)
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

   static void applyJacobiGivensRotationY(double ch, double sh, CommonMatrix3DBasics S)
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

   static void applyJacobiGivensRotationZ(double ch, double sh, CommonMatrix3DBasics S)
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

   private static void applyQRGivensRotationZ(double ch, double sh, CommonMatrix3DBasics BToUpdate)
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

   private static void applyQRGivensRotationY(double ch, double sh, CommonMatrix3DBasics BToUpdate)
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

   private static void applyQRGivensRotation(double ch, double sh, CommonMatrix3DBasics BToUpdate)
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

   private static void prependGivensQuaternionX(double ch, double sh, Tuple4DBasics V)
   {
      double vx = V.getX();
      double vy = V.getY();
      double vz = V.getZ();
      double vs = V.getS();
      V.set(vx * ch + vs * sh, vy * ch + vz * sh, vz * ch - vy * sh, vs * ch - vx * sh);
   }

   private static void prependGivensQuaternionY(double ch, double sh, Tuple4DBasics V)
   {
      double vx = V.getX();
      double vy = V.getY();
      double vz = V.getZ();
      double vs = V.getS();
      V.set(vz * sh + vx * ch, vy * ch - vs * sh, vz * ch - vx * sh, vs * ch + vy * sh);
   }

   private static void prependGivensQuaternionZ(double ch, double sh, Tuple4DBasics V)
   {
      double vx = V.getX();
      double vy = V.getY();
      double vz = V.getZ();
      double vs = V.getS();
      V.set(vx * ch + vy * sh, vy * ch - vx * sh, vz * ch + vs * sh, vs * ch - vz * sh);
   }

   private static void appendGivensQuaternionX(double ch, double sh, Tuple4DBasics U)
   {
      double ux = U.getX();
      double uy = U.getY();
      double uz = U.getZ();
      double us = U.getS();
      U.set(us * sh + ux * ch, uy * ch + uz * sh, uz * ch - uy * sh, us * ch - ux * sh);
   }

   private static void appendGivensQuatenrionY(double ch, double sh, Tuple4DBasics U)
   {
      double ux = U.getX();
      double uy = U.getY();
      double uz = U.getZ();
      double us = U.getS();
      U.set(ux * ch + uz * sh, uy * ch - us * sh, uz * ch - ux * sh, us * ch + uy * sh);
   }

   private static void appendGivensQuaternionZ(double ch, double sh, Tuple4DBasics U)
   {
      double ux = U.getX();
      double uy = U.getY();
      double uz = U.getZ();
      double us = U.getS();
      U.set(ux * ch + uy * sh, uy * ch - ux * sh, us * sh + uz * ch, us * ch - uz * sh);
   }

   static void sortBColumns(CommonMatrix3DBasics B, CommonMatrix3DBasics Vmat, Tuple4DBasics Vquat)
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
               Vmat.set(Vmat.getM00(), Vmat.getM02(), -Vmat.getM01(),
                        Vmat.getM10(), Vmat.getM12(), -Vmat.getM11(),
                        Vmat.getM20(), Vmat.getM22(), -Vmat.getM21());
               Vquat.set(sqrtTwoOverTwo * (qs + qx),
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
            Vmat.set(Vmat.getM02(), Vmat.getM00(), Vmat.getM01(),
                     Vmat.getM12(), Vmat.getM10(), Vmat.getM11(),
                     Vmat.getM22(), Vmat.getM20(), Vmat.getM21());
            Vquat.set(0.5 * (-qs + qx - qy + qz),
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
               Vmat.set(Vmat.getM01(), -Vmat.getM00(), Vmat.getM02(),
                        Vmat.getM11(), -Vmat.getM10(), Vmat.getM12(),
                        Vmat.getM21(), -Vmat.getM20(), Vmat.getM22());
               Vquat.set(sqrtTwoOverTwo * (qx + qy),
                         sqrtTwoOverTwo * (qy - qx),
                         sqrtTwoOverTwo * (qs + qz),
                         sqrtTwoOverTwo * (qs - qz));
            }
            else
            { // 1 > 2 > 0
               B.set(B.getM01(), B.getM02(), B.getM00(),
                     B.getM11(), B.getM12(), B.getM10(),
                     B.getM21(), B.getM22(), B.getM20());
               Vmat.set(Vmat.getM01(), Vmat.getM02(), Vmat.getM00(),
                        Vmat.getM11(), Vmat.getM12(), Vmat.getM10(),
                        Vmat.getM21(), Vmat.getM22(), Vmat.getM20());
               Vquat.set(0.5 * (qs + qx + qy - qz),
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
            Vmat.set(Vmat.getM02(), Vmat.getM01(), -Vmat.getM00(),
                     Vmat.getM12(), Vmat.getM11(), -Vmat.getM10(),
                     Vmat.getM22(), Vmat.getM21(), -Vmat.getM20());
            Vquat.set(sqrtTwoOverTwo * (qx + qz),
                      sqrtTwoOverTwo * (qy - qs),
                      sqrtTwoOverTwo * (qz - qx),
                      sqrtTwoOverTwo * (qs + qy));
         }
      }
      // @formatter:off
   }

   private static void toRotationMatrix(Tuple4DReadOnly quaternion, CommonMatrix3DBasics rotationMatrix)
   {
      double qx = quaternion.getX();
      double qy = quaternion.getY();
      double qz = quaternion.getZ();
      double qs = quaternion.getS();

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
      if (rotationMatrix instanceof RotationMatrixBasics)
         ((RotationMatrixBasics) rotationMatrix).setUnsafe(m00, m01, m02, m10, m11, m12, m20, m21, m22);
      else
         rotationMatrix.set(m00, m01, m02, m10, m11, m12, m20, m21, m22);
   }

   public SVD3DOutput getOutput()
   {
      return output;
   }

   public Matrix3D getUMatrix()
   {
      return output.getUMatrix();
   }

   public Vector4D getUQuaternion()
   {
      return output.getUQuaternion();
   }

   public RotationMatrixBasics getU(RotationMatrixBasics U, boolean transpose)
   {
      return output.getU(U, transpose);
   }

   public QuaternionBasics getU(QuaternionBasics U, boolean conjugate)
   {
      return output.getU(U, conjugate);
   }

   public double getSigma0()
   {
      return output.getSigma0();
   }

   public double getSigma1()
   {
      return output.getSigma1();
   }

   public double getSigma2()
   {
      return output.getSigma2();
   }

   public Matrix3DBasics getW(Matrix3DBasics W)
   {
      return output.getW(W);
   }

   public Matrix3D getVMatrix()
   {
      return output.getVMatrix();
   }

   public Vector4D getVQuaternion()
   {
      return output.getVQuaternion();
   }

   public RotationMatrixBasics getV(RotationMatrixBasics V, boolean transpose)
   {
      return output.getV(V, transpose);
   }

   public QuaternionBasics getV(QuaternionBasics V, boolean conjugate)
   {
      return output.getV(V, conjugate);
   }

   public double getTolerance()
   {
      return tolerance;
   }

   public static class SVD3DOutput
   {
      private final Vector4D Uquat = new Vector4D();
      private final Matrix3D Umat = new Matrix3D();

      private double sigma0, sigma1, sigma2;

      private final Vector4D Vquat = new Vector4D();
      private final Matrix3D Vmat = new Matrix3D();

      public void set(SVD3DOutput other)
      {
         Uquat.set(other.Uquat);
         Umat.set(other.Umat);

         sigma0 = other.sigma0;
         sigma1 = other.sigma1;
         sigma2 = other.sigma2;

         Vquat.set(other.Vquat);
         Vmat.set(other.Vmat);
      }

      public void setIdentity()
      {
         Uquat.set(0, 0, 0, 1);
         Umat.setIdentity();

         sigma0 = 1.0;
         sigma1 = 1.0;
         sigma2 = 1.0;

         Vquat.set(0, 0, 0, 1);
         Vmat.setIdentity();
      }

      public void setToNaN()
      {
         Uquat.setToNaN();
         Umat.setToNaN();

         sigma0 = Double.NaN;
         sigma1 = Double.NaN;
         sigma2 = Double.NaN;

         Vquat.setToNaN();
         Vmat.setToNaN();
      }

      public void transpose()
      {
         {
            double vx = Vquat.getX();
            double vy = Vquat.getY();
            double vz = Vquat.getZ();
            double vs = Vquat.getS();
            Vquat.set(Uquat);
            Uquat.set(vx, vy, vz, vs);
         }

         {
            double v00 = Vmat.getM00();
            double v01 = Vmat.getM01();
            double v02 = Vmat.getM02();
            double v10 = Vmat.getM10();
            double v11 = Vmat.getM11();
            double v12 = Vmat.getM12();
            double v20 = Vmat.getM20();
            double v21 = Vmat.getM21();
            double v22 = Vmat.getM22();
            Vmat.set(Umat);
            Umat.set(v00, v01, v02, v10, v11, v12, v20, v21, v22);
         }
      }

      public void setSigmas(double sigma0, double sigma1, double sigma2)
      {
         this.sigma0 = sigma0;
         this.sigma1 = sigma1;
         this.sigma2 = sigma2;
      }

      public void setSigma0(double sigma0)
      {
         this.sigma0 = sigma0;
      }

      public void setSigma1(double sigma1)
      {
         this.sigma1 = sigma1;
      }

      public void setSigma2(double sigma2)
      {
         this.sigma2 = sigma2;
      }

      public Matrix3D getUMatrix()
      {
         return Umat;
      }

      public Vector4D getUQuaternion()
      {
         return Uquat;
      }

      public RotationMatrixBasics getU(RotationMatrixBasics U, boolean transpose)
      {
         if (U == null)
            U = new RotationMatrix();
         if (transpose)
            U.setUnsafe(Umat.getM00(), Umat.getM10(), Umat.getM20(), Umat.getM01(), Umat.getM11(), Umat.getM21(), Umat.getM02(), Umat.getM12(), Umat.getM22());
         else
            U.setUnsafe(Umat.getM00(), Umat.getM01(), Umat.getM02(), Umat.getM10(), Umat.getM11(), Umat.getM12(), Umat.getM20(), Umat.getM21(), Umat.getM22());
         return U;
      }

      public QuaternionBasics getU(QuaternionBasics U, boolean conjugate)
      {
         if (U == null)
            U = new Quaternion();
         if (conjugate)
            U.setUnsafe(-Uquat.getX(), -Uquat.getY(), -Uquat.getZ(), Uquat.getS());
         else
            U.setUnsafe(Uquat.getX(), Uquat.getY(), Uquat.getZ(), Uquat.getS());
         return U;
      }

      public double getSigma0()
      {
         return sigma0;
      }

      public double getSigma1()
      {
         return sigma1;
      }

      public double getSigma2()
      {
         return sigma2;
      }

      public Matrix3DBasics getW(Matrix3DBasics W)
      {
         if (W == null)
            W = new Matrix3D();
         W.setToDiagonal(sigma0, sigma1, sigma2);
         return W;
      }

      public Matrix3D getVMatrix()
      {
         return Vmat;
      }

      public Vector4D getVQuaternion()
      {
         return Vquat;
      }

      public RotationMatrixBasics getV(RotationMatrixBasics V, boolean transpose)
      {
         if (V == null)
            V = new RotationMatrix();
         if (transpose)
            V.setUnsafe(Vmat.getM00(), Vmat.getM10(), Vmat.getM20(), Vmat.getM01(), Vmat.getM11(), Vmat.getM21(), Vmat.getM02(), Vmat.getM12(), Vmat.getM22());
         else
            V.setUnsafe(Vmat.getM00(), Vmat.getM01(), Vmat.getM02(), Vmat.getM10(), Vmat.getM11(), Vmat.getM12(), Vmat.getM20(), Vmat.getM21(), Vmat.getM22());
         return V;
      }

      public QuaternionBasics getV(QuaternionBasics V, boolean conjugate)
      {
         if (V == null)
            V = new Quaternion();
         if (conjugate)
            V.setUnsafe(-Vquat.getX(), -Vquat.getY(), -Vquat.getZ(), Vquat.getS());
         else
            V.setUnsafe(Vquat.getX(), Vquat.getY(), Vquat.getZ(), Vquat.getS());
         return V;
      }
   }
}
