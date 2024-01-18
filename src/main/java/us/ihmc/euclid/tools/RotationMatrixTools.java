package us.ihmc.euclid.tools;

import us.ihmc.euclid.axisAngle.interfaces.AxisAngleReadOnly;
import us.ihmc.euclid.matrix.interfaces.CommonMatrix3DBasics;
import us.ihmc.euclid.matrix.interfaces.Matrix3DReadOnly;
import us.ihmc.euclid.matrix.interfaces.RotationMatrixBasics;
import us.ihmc.euclid.matrix.interfaces.RotationMatrixReadOnly;
import us.ihmc.euclid.orientation.interfaces.Orientation3DReadOnly;
import us.ihmc.euclid.rotationConversion.AxisAngleConversion;
import us.ihmc.euclid.rotationConversion.RotationMatrixConversion;
import us.ihmc.euclid.rotationConversion.RotationVectorConversion;
import us.ihmc.euclid.tuple2D.interfaces.Tuple2DBasics;
import us.ihmc.euclid.tuple2D.interfaces.Tuple2DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DBasics;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DBasics;
import us.ihmc.euclid.tuple4D.interfaces.QuaternionBasics;
import us.ihmc.euclid.tuple4D.interfaces.QuaternionReadOnly;
import us.ihmc.euclid.yawPitchRoll.interfaces.YawPitchRollReadOnly;

/**
 * Tools for performing operations on rotation matrices.
 *
 * @author Sylvain Bertrand
 */
public class RotationMatrixTools
{

   static final double EPS = 1.0e-12;

   private RotationMatrixTools()
   {
      // Suppresses default constructor, ensuring non-instantiability.
   }

   /**
    * Performs the multiplication: {@code m1} * {@code m2} and stores the result in
    * {@code matrixToPack}.
    * <p>
    * All the matrices can be the same object.
    * </p>
    *
    * @param m1           the first matrix. Not modified.
    * @param m2           the second matrix. Not modified.
    * @param matrixToPack the matrix in which the result is stored. Modified.
    */
   public static void multiply(RotationMatrixReadOnly m1, RotationMatrixReadOnly m2, CommonMatrix3DBasics matrixToPack)
   {
      multiplyImpl(m1, false, m2, false, matrixToPack);
   }

   /**
    * Performs the multiplication: {@code m1}<sup>T</sup> * {@code m2}<sup>T</sup> and stores the
    * result in {@code matrixToPack}.
    * <p>
    * All the matrices can be the same object.
    * </p>
    *
    * @param m1           the first matrix. Not modified.
    * @param m2           the second matrix. Not modified.
    * @param matrixToPack the matrix in which the result is stored. Modified.
    */
   public static void multiplyTransposeBoth(RotationMatrixReadOnly m1, RotationMatrixReadOnly m2, CommonMatrix3DBasics matrixToPack)
   {
      multiplyImpl(m1, true, m2, true, matrixToPack);
   }

   /**
    * Performs the multiplication: {@code m1}<sup>T</sup> * {@code m2} and stores the result in
    * {@code matrixToPack}.
    * <p>
    * All the matrices can be the same object.
    * </p>
    *
    * @param m1           the first matrix. Not modified.
    * @param m2           the second matrix. Not modified.
    * @param matrixToPack the matrix in which the result is stored. Modified.
    */
   public static void multiplyTransposeLeft(RotationMatrixReadOnly m1, RotationMatrixReadOnly m2, CommonMatrix3DBasics matrixToPack)
   {
      multiplyImpl(m1, true, m2, false, matrixToPack);
   }

   /**
    * Performs the multiplication: {@code m1} * {@code m2}<sup>T</sup> and stores the result in
    * {@code matrixToPack}.
    * <p>
    * All the matrices can be the same object.
    * </p>
    *
    * @param m1           the first matrix. Not modified.
    * @param m2           the second matrix. Not modified.
    * @param matrixToPack the matrix in which the result is stored. Modified.
    */
   public static void multiplyTransposeRight(RotationMatrixReadOnly m1, RotationMatrixReadOnly m2, CommonMatrix3DBasics matrixToPack)
   {
      multiplyImpl(m1, false, m2, true, matrixToPack);
   }

   /**
    * Performs the multiplication of {@code orientation1} and {@code orientation2} and stores the
    * result in {@code matrixToPack}.
    * <p>
    * More precisely, {@code orientation1} and {@code orientation2} are first converted to rotation
    * matrices, then an matrix multiplication is performed using the two first arguments as entry.
    * </p>
    * <p>
    * All three arguments can be the same object for in place operations.
    * </p>
    *
    * @param orientation1 the first orientation in the multiplication. Not modified.
    * @param inverse1     whether the first orientation should be inverted in the multiplication.
    * @param orientation2 the second orientation in the multiplication. Not modified.
    * @param inverse2     whether the second orientation should be inverted in the multiplication.
    * @param matrixToPack the rotation matrix in which the result is stored. Modified.
    */
   public static void multiply(Orientation3DReadOnly orientation1,
                               boolean inverse1,
                               Orientation3DReadOnly orientation2,
                               boolean inverse2,
                               CommonMatrix3DBasics matrixToPack)
   {
      if (orientation1 instanceof RotationMatrixReadOnly)
      {
         multiply((RotationMatrixReadOnly) orientation1, inverse1, orientation2, inverse2, matrixToPack);
         return;
      }

      if (orientation1.isZeroOrientation())
      {
         if (orientation2.isZeroOrientation())
         {
            matrixToPack.setIdentity();
         }
         else if (inverse2)
         {
            matrixToPack.set(orientation2);
            matrixToPack.transpose();
         }
         else
         {
            matrixToPack.set(orientation2);
         }

         return;
      }
      else if (orientation2.isZeroOrientation())
      {
         if (inverse1)
         {
            matrixToPack.set(orientation1);
            matrixToPack.transpose();
         }
         else
         {
            matrixToPack.set(orientation1);
         }

         return;
      }

      double b00, b01, b02, b10, b11, b12, b20, b21, b22;
      if (orientation2 instanceof RotationMatrixReadOnly b)
      { // In this case orientation2 might be the same object as matrixToPack, so let's save its components first.
         b00 = b.getM00();
         b01 = b.getM01();
         b02 = b.getM02();
         b10 = b.getM10();
         b11 = b.getM11();
         b12 = b.getM12();
         b20 = b.getM20();
         b21 = b.getM21();
         b22 = b.getM22();
      }
      else
      {
         matrixToPack.set(orientation2);
         b00 = matrixToPack.getM00();
         b01 = matrixToPack.getM01();
         b02 = matrixToPack.getM02();
         b10 = matrixToPack.getM10();
         b11 = matrixToPack.getM11();
         b12 = matrixToPack.getM12();
         b20 = matrixToPack.getM20();
         b21 = matrixToPack.getM21();
         b22 = matrixToPack.getM22();
      }

      // Now we can safely use the matrixToPack argument to convert the orientation1.
      matrixToPack.set(orientation1);
      double a00 = matrixToPack.getM00();
      double a01 = matrixToPack.getM01();
      double a02 = matrixToPack.getM02();
      double a10 = matrixToPack.getM10();
      double a11 = matrixToPack.getM11();
      double a12 = matrixToPack.getM12();
      double a20 = matrixToPack.getM20();
      double a21 = matrixToPack.getM21();
      double a22 = matrixToPack.getM22();
      multiplyImpl(a00, a01, a02, a10, a11, a12, a20, a21, a22, inverse1, b00, b01, b02, b10, b11, b12, b20, b21, b22, inverse2, matrixToPack);
   }

   /**
    * Performs the multiplication of {@code orientation1} and {@code orientation2} and stores the
    * result in {@code matrixToPack}.
    * <p>
    * More precisely, {@code orientation1} is first converted to a rotation matrix, then an matrix
    * multiplication is performed using the two first arguments as entry.
    * </p>
    * <p>
    * All three arguments can be the same object for in place operations.
    * </p>
    *
    * @param orientation1 the first orientation in the multiplication. Not modified.
    * @param inverse1     whether the first orientation should be inverted in the multiplication.
    * @param orientation2 the second orientation in the multiplication. Not modified.
    * @param inverse2     whether the second orientation should be inverted in the multiplication.
    * @param matrixToPack the rotation matrix in which the result is stored. Modified.
    */
   public static void multiply(Orientation3DReadOnly orientation1,
                               boolean inverse1,
                               RotationMatrixReadOnly orientation2,
                               boolean inverse2,
                               CommonMatrix3DBasics matrixToPack)
   {
      if (orientation1 instanceof RotationMatrixReadOnly)
      {
         multiplyImpl((RotationMatrixReadOnly) orientation1, inverse1, orientation2, inverse2, matrixToPack);
         return;
      }

      if (orientation1.isZeroOrientation())
      {
         if (orientation2.isZeroOrientation())
         {
            matrixToPack.setIdentity();
         }
         else if (inverse2)
         {
            matrixToPack.set(orientation2);
            matrixToPack.transpose();
         }
         else
         {
            matrixToPack.set(orientation2);
         }

         return;
      }
      else if (orientation2.isZeroOrientation())
      {
         if (inverse1)
         {
            matrixToPack.set(orientation1);
            matrixToPack.transpose();
         }
         else
         {
            matrixToPack.set(orientation1);
         }

         return;
      }

      // In this case orientation2 might be the same object as matrixToPack, so let's save its components first.
      double b00 = orientation2.getM00();
      double b01 = orientation2.getM01();
      double b02 = orientation2.getM02();
      double b10 = orientation2.getM10();
      double b11 = orientation2.getM11();
      double b12 = orientation2.getM12();
      double b20 = orientation2.getM20();
      double b21 = orientation2.getM21();
      double b22 = orientation2.getM22();
      // Now we can safely use the matrixToPack argument to convert the orientation1.
      matrixToPack.set(orientation1);
      double a00 = matrixToPack.getM00();
      double a01 = matrixToPack.getM01();
      double a02 = matrixToPack.getM02();
      double a10 = matrixToPack.getM10();
      double a11 = matrixToPack.getM11();
      double a12 = matrixToPack.getM12();
      double a20 = matrixToPack.getM20();
      double a21 = matrixToPack.getM21();
      double a22 = matrixToPack.getM22();
      multiplyImpl(a00, a01, a02, a10, a11, a12, a20, a21, a22, inverse1, b00, b01, b02, b10, b11, b12, b20, b21, b22, inverse2, matrixToPack);
   }

   /**
    * Performs the multiplication of {@code orientation1} and {@code orientation2} and stores the
    * result in {@code matrixToPack}.
    * <p>
    * More precisely, {@code orientation2} is first converted to a rotation matrix, then an matrix
    * multiplication is performed using the two first arguments as entry.
    * </p>
    * <p>
    * All three arguments can be the same object for in place operations.
    * </p>
    *
    * @param orientation1 the first orientation in the multiplication. Not modified.
    * @param inverse1     whether the first orientation should be inverted in the multiplication.
    * @param orientation2 the second orientation in the multiplication. Not modified.
    * @param inverse2     whether the second orientation should be inverted in the multiplication.
    * @param matrixToPack the rotation matrix in which the result is stored. Modified.
    */
   public static void multiply(RotationMatrixReadOnly orientation1,
                               boolean inverse1,
                               Orientation3DReadOnly orientation2,
                               boolean inverse2,
                               CommonMatrix3DBasics matrixToPack)
   {
      if (orientation2 instanceof RotationMatrixReadOnly)
      {
         multiplyImpl(orientation1, inverse1, (RotationMatrixReadOnly) orientation2, inverse2, matrixToPack);
         return;
      }

      if (orientation1.isZeroOrientation())
      {
         if (orientation2.isZeroOrientation())
         {
            matrixToPack.setIdentity();
         }
         else if (inverse2)
         {
            matrixToPack.set(orientation2);
            matrixToPack.transpose();
         }
         else
         {
            matrixToPack.set(orientation2);
         }

         return;
      }
      else if (orientation2.isZeroOrientation())
      {
         if (inverse1)
         {
            matrixToPack.set(orientation1);
            matrixToPack.transpose();
         }
         else
         {
            matrixToPack.set(orientation1);
         }

         return;
      }

      // In this case orientation1 might be the same object as matrixToPack, so let's save its components first.
      double a00 = orientation1.getM00();
      double a01 = orientation1.getM01();
      double a02 = orientation1.getM02();
      double a10 = orientation1.getM10();
      double a11 = orientation1.getM11();
      double a12 = orientation1.getM12();
      double a20 = orientation1.getM20();
      double a21 = orientation1.getM21();
      double a22 = orientation1.getM22();
      // Now we can safely use the matrixToPack argument to convert the orientation2.
      matrixToPack.set(orientation2);
      double b00 = matrixToPack.getM00();
      double b01 = matrixToPack.getM01();
      double b02 = matrixToPack.getM02();
      double b10 = matrixToPack.getM10();
      double b11 = matrixToPack.getM11();
      double b12 = matrixToPack.getM12();
      double b20 = matrixToPack.getM20();
      double b21 = matrixToPack.getM21();
      double b22 = matrixToPack.getM22();
      multiplyImpl(a00, a01, a02, a10, a11, a12, a20, a21, a22, inverse1, b00, b01, b02, b10, b11, b12, b20, b21, b22, inverse2, matrixToPack);
   }

   /**
    * Performs the multiplication of {@code a} and {@code b} and stores the result in
    * {@code matrixToPack}.
    * <p>
    * All three arguments can be the same object for in place operations.
    * </p>
    *
    * @param a            the first rotation matrix in the multiplication. Not modified.
    * @param transposeA   whether the first matrix should be transposed in the multiplication.
    * @param b            the second rotation matrix in the multiplication. Not modified.
    * @param transposeB   whether the second matrix should be transposed in the multiplication.
    * @param matrixToPack the rotation matrix in which the result is stored. Modified.
    */
   private static void multiplyImpl(RotationMatrixReadOnly a,
                                    boolean transposeA,
                                    RotationMatrixReadOnly b,
                                    boolean transposeB,
                                    CommonMatrix3DBasics matrixToPack)
   {
      if (a.isZeroOrientation())
      {
         if (b.isZeroOrientation())
            matrixToPack.setIdentity();
         else if (transposeB)
            matrixToPack.setAndTranspose(b);
         else
            matrixToPack.set(b);

         return;
      }
      else if (b.isZeroOrientation())
      {
         if (transposeA)
            matrixToPack.setAndTranspose(a);
         else
            matrixToPack.set(a);

         return;
      }

      multiplyImpl(a.getM00(),
                   a.getM01(),
                   a.getM02(),
                   a.getM10(),
                   a.getM11(),
                   a.getM12(),
                   a.getM20(),
                   a.getM21(),
                   a.getM22(),
                   transposeA,
                   b.getM00(),
                   b.getM01(),
                   b.getM02(),
                   b.getM10(),
                   b.getM11(),
                   b.getM12(),
                   b.getM20(),
                   b.getM21(),
                   b.getM22(),
                   transposeB,
                   matrixToPack);
   }

   private static void multiplyImpl(double a00,
                                    double a01,
                                    double a02,
                                    double a10,
                                    double a11,
                                    double a12,
                                    double a20,
                                    double a21,
                                    double a22,
                                    boolean transposeA,
                                    double b00,
                                    double b01,
                                    double b02,
                                    double b10,
                                    double b11,
                                    double b12,
                                    double b20,
                                    double b21,
                                    double b22,
                                    boolean transposeB,
                                    CommonMatrix3DBasics matrixToPack)
   {
      double c00, c01, c02, c10, c11, c12, c20, c21, c22;

      if (transposeA)
      {
         if (transposeB)
         {
            c00 = a00 * b00 + a10 * b01 + a20 * b02;
            c01 = a00 * b10 + a10 * b11 + a20 * b12;
            c02 = a00 * b20 + a10 * b21 + a20 * b22;
            c10 = a01 * b00 + a11 * b01 + a21 * b02;
            c11 = a01 * b10 + a11 * b11 + a21 * b12;
            c12 = a01 * b20 + a11 * b21 + a21 * b22;
            c20 = a02 * b00 + a12 * b01 + a22 * b02;
            c21 = a02 * b10 + a12 * b11 + a22 * b12;
            c22 = a02 * b20 + a12 * b21 + a22 * b22;
         }
         else
         {
            c00 = a00 * b00 + a10 * b10 + a20 * b20;
            c01 = a00 * b01 + a10 * b11 + a20 * b21;
            c02 = a00 * b02 + a10 * b12 + a20 * b22;
            c10 = a01 * b00 + a11 * b10 + a21 * b20;
            c11 = a01 * b01 + a11 * b11 + a21 * b21;
            c12 = a01 * b02 + a11 * b12 + a21 * b22;
            c20 = a02 * b00 + a12 * b10 + a22 * b20;
            c21 = a02 * b01 + a12 * b11 + a22 * b21;
            c22 = a02 * b02 + a12 * b12 + a22 * b22;
         }
      }
      else
      {
         if (transposeB)
         {
            c00 = a00 * b00 + a01 * b01 + a02 * b02;
            c01 = a00 * b10 + a01 * b11 + a02 * b12;
            c02 = a00 * b20 + a01 * b21 + a02 * b22;
            c10 = a10 * b00 + a11 * b01 + a12 * b02;
            c11 = a10 * b10 + a11 * b11 + a12 * b12;
            c12 = a10 * b20 + a11 * b21 + a12 * b22;
            c20 = a20 * b00 + a21 * b01 + a22 * b02;
            c21 = a20 * b10 + a21 * b11 + a22 * b12;
            c22 = a20 * b20 + a21 * b21 + a22 * b22;
         }
         else
         {
            c00 = a00 * b00 + a01 * b10 + a02 * b20;
            c01 = a00 * b01 + a01 * b11 + a02 * b21;
            c02 = a00 * b02 + a01 * b12 + a02 * b22;
            c10 = a10 * b00 + a11 * b10 + a12 * b20;
            c11 = a10 * b01 + a11 * b11 + a12 * b21;
            c12 = a10 * b02 + a11 * b12 + a12 * b22;
            c20 = a20 * b00 + a21 * b10 + a22 * b20;
            c21 = a20 * b01 + a21 * b11 + a22 * b21;
            c22 = a20 * b02 + a21 * b12 + a22 * b22;
         }
      }

      matrixToPack.set(c00, c01, c02, c10, c11, c12, c20, c21, c22);
   }

   /**
    * Prepend a rotation about the z-axis to {@code matrixOriginal} and stores the result in
    * {@code matrixToPack}.
    * <p>
    * All the matrices can be the same object.
    * </p>
    *
    * <pre>
    *                / cos(yaw) -sin(yaw) 0 \
    * matrixToPack = | sin(yaw)  cos(yaw) 0 | * matrixOriginal
    *                \    0         0     1 /
    * </pre>
    *
    * @param yaw            the angle to rotate about the z-axis.
    * @param matrixOriginal the matrix on which the yaw rotation is appended. Not modified.
    * @param matrixToPack   the matrix in which the result is stored. Modified.
    */
   public static void prependYawRotation(double yaw, Matrix3DReadOnly matrixOriginal, CommonMatrix3DBasics matrixToPack)
   {
      if (matrixOriginal.isIdentity())
      {
         RotationMatrixConversion.computeYawMatrix(yaw, matrixToPack);
         return;
      }

      double cYaw = EuclidCoreTools.cos(yaw);
      double sYaw = EuclidCoreTools.sin(yaw);

      double m00 = cYaw * matrixOriginal.getM00() - sYaw * matrixOriginal.getM10();
      double m01 = cYaw * matrixOriginal.getM01() - sYaw * matrixOriginal.getM11();
      double m02 = cYaw * matrixOriginal.getM02() - sYaw * matrixOriginal.getM12();
      double m10 = sYaw * matrixOriginal.getM00() + cYaw * matrixOriginal.getM10();
      double m11 = sYaw * matrixOriginal.getM01() + cYaw * matrixOriginal.getM11();
      double m12 = sYaw * matrixOriginal.getM02() + cYaw * matrixOriginal.getM12();
      double m20 = matrixOriginal.getM20();
      double m21 = matrixOriginal.getM21();
      double m22 = matrixOriginal.getM22();
      // TODO Should we switch to setUnsafe?
      matrixToPack.set(m00, m01, m02, m10, m11, m12, m20, m21, m22);
   }

   /**
    * Append a rotation about the z-axis to {@code matrixOriginal} and stores the result in
    * {@code matrixToPack}.
    * <p>
    * All the matrices can be the same object.
    * </p>
    *
    * <pre>
    *                                 / cos(yaw) -sin(yaw) 0 \
    * matrixToPack = matrixOriginal * | sin(yaw)  cos(yaw) 0 |
    *                                 \    0         0     1 /
    * </pre>
    *
    * @param matrixOriginal the matrix on which the yaw rotation is appended. Not modified.
    * @param yaw            the angle to rotate about the z-axis.
    * @param matrixToPack   the matrix in which the result is stored. Modified.
    */
   public static void appendYawRotation(Matrix3DReadOnly matrixOriginal, double yaw, CommonMatrix3DBasics matrixToPack)
   {
      if (matrixOriginal.isIdentity())
      {
         RotationMatrixConversion.computeYawMatrix(yaw, matrixToPack);
         return;
      }

      double cYaw = EuclidCoreTools.cos(yaw);
      double sYaw = EuclidCoreTools.sin(yaw);

      double m00 = cYaw * matrixOriginal.getM00() + sYaw * matrixOriginal.getM01();
      double m01 = -sYaw * matrixOriginal.getM00() + cYaw * matrixOriginal.getM01();
      double m02 = matrixOriginal.getM02();
      double m10 = cYaw * matrixOriginal.getM10() + sYaw * matrixOriginal.getM11();
      double m11 = -sYaw * matrixOriginal.getM10() + cYaw * matrixOriginal.getM11();
      double m12 = matrixOriginal.getM12();
      double m20 = cYaw * matrixOriginal.getM20() + sYaw * matrixOriginal.getM21();
      double m21 = -sYaw * matrixOriginal.getM20() + cYaw * matrixOriginal.getM21();
      double m22 = matrixOriginal.getM22();
      matrixToPack.set(m00, m01, m02, m10, m11, m12, m20, m21, m22);
   }

   /**
    * Prepend a rotation about the y-axis to {@code matrixOriginal} and stores the result in
    * {@code matrixToPack}.
    * <p>
    * All the matrices can be the same object.
    * </p>
    *
    * <pre>
    *                /  cos(pitch) 0 sin(pitch) \
    * matrixToPack = |      0      1     0      | * matrixOriginal
    *                \ -sin(pitch) 0 cos(pitch) /
    * </pre>
    *
    * @param pitch          the angle to rotate about the y-axis.
    * @param matrixOriginal the matrix on which the pitch rotation is appended. Not modified.
    * @param matrixToPack   the matrix in which the result is stored. Modified.
    */
   public static void prependPitchRotation(double pitch, Matrix3DReadOnly matrixOriginal, CommonMatrix3DBasics matrixToPack)
   {
      if (matrixOriginal.isIdentity())
      {
         RotationMatrixConversion.computePitchMatrix(pitch, matrixToPack);
         return;
      }

      double cPitch = EuclidCoreTools.cos(pitch);
      double sPitch = EuclidCoreTools.sin(pitch);

      double m00 = cPitch * matrixOriginal.getM00() + sPitch * matrixOriginal.getM20();
      double m01 = cPitch * matrixOriginal.getM01() + sPitch * matrixOriginal.getM21();
      double m02 = cPitch * matrixOriginal.getM02() + sPitch * matrixOriginal.getM22();
      double m10 = matrixOriginal.getM10();
      double m11 = matrixOriginal.getM11();
      double m12 = matrixOriginal.getM12();
      double m20 = -sPitch * matrixOriginal.getM00() + cPitch * matrixOriginal.getM20();
      double m21 = -sPitch * matrixOriginal.getM01() + cPitch * matrixOriginal.getM21();
      double m22 = -sPitch * matrixOriginal.getM02() + cPitch * matrixOriginal.getM22();
      matrixToPack.set(m00, m01, m02, m10, m11, m12, m20, m21, m22);
   }

   /**
    * Append a rotation about the y-axis to {@code matrixOriginal} and stores the result in
    * {@code matrixToPack}.
    * <p>
    * All the matrices can be the same object.
    * </p>
    *
    * <pre>
    *                                 /  cos(pitch) 0 sin(pitch) \
    * matrixToPack = matrixOriginal * |      0      1     0      |
    *                                 \ -sin(pitch) 0 cos(pitch) /
    * </pre>
    *
    * @param matrixOriginal the matrix on which the pitch rotation is appended. Not modified.
    * @param pitch          the angle to rotate about the y-axis.
    * @param matrixToPack   the matrix in which the result is stored. Modified.
    */
   public static void appendPitchRotation(Matrix3DReadOnly matrixOriginal, double pitch, CommonMatrix3DBasics matrixToPack)
   {
      if (matrixOriginal.isIdentity())
      {
         RotationMatrixConversion.computePitchMatrix(pitch, matrixToPack);
         return;
      }

      double cPitch = EuclidCoreTools.cos(pitch);
      double sPitch = EuclidCoreTools.sin(pitch);

      double m00 = cPitch * matrixOriginal.getM00() - sPitch * matrixOriginal.getM02();
      double m01 = matrixOriginal.getM01();
      double m02 = sPitch * matrixOriginal.getM00() + cPitch * matrixOriginal.getM02();
      double m10 = cPitch * matrixOriginal.getM10() - sPitch * matrixOriginal.getM12();
      double m11 = matrixOriginal.getM11();
      double m12 = sPitch * matrixOriginal.getM10() + cPitch * matrixOriginal.getM12();
      double m20 = cPitch * matrixOriginal.getM20() - sPitch * matrixOriginal.getM22();
      double m21 = matrixOriginal.getM21();
      double m22 = sPitch * matrixOriginal.getM20() + cPitch * matrixOriginal.getM22();
      matrixToPack.set(m00, m01, m02, m10, m11, m12, m20, m21, m22);
   }

   /**
    * Prepend a rotation about the x-axis to {@code matrixOriginal} and stores the result in
    * {@code matrixToPack}.
    * <p>
    * All the matrices can be the same object.
    * </p>
    *
    * <pre>
    *                / 1     0          0     \
    * matrixToPack = | 0 cos(roll) -sin(roll) | * matrixOriginal
    *                \ 0 sin(roll)  cos(roll) /
    * </pre>
    *
    * @param roll           the angle to rotate about the x-axis.
    * @param matrixOriginal the matrix on which the roll rotation is appended. Not modified.
    * @param matrixToPack   the matrix in which the result is stored. Modified.
    */
   public static void prependRollRotation(double roll, Matrix3DReadOnly matrixOriginal, CommonMatrix3DBasics matrixToPack)
   {
      if (matrixOriginal.isIdentity())
      {
         RotationMatrixConversion.computeRollMatrix(roll, matrixToPack);
         return;
      }

      double cRoll = EuclidCoreTools.cos(roll);
      double sRoll = EuclidCoreTools.sin(roll);

      double m00 = matrixOriginal.getM00();
      double m01 = matrixOriginal.getM01();
      double m02 = matrixOriginal.getM02();
      double m10 = cRoll * matrixOriginal.getM10() - sRoll * matrixOriginal.getM20();
      double m11 = cRoll * matrixOriginal.getM11() - sRoll * matrixOriginal.getM21();
      double m12 = cRoll * matrixOriginal.getM12() - sRoll * matrixOriginal.getM22();
      double m20 = sRoll * matrixOriginal.getM10() + cRoll * matrixOriginal.getM20();
      double m21 = sRoll * matrixOriginal.getM11() + cRoll * matrixOriginal.getM21();
      double m22 = sRoll * matrixOriginal.getM12() + cRoll * matrixOriginal.getM22();
      matrixToPack.set(m00, m01, m02, m10, m11, m12, m20, m21, m22);
   }

   /**
    * Append a rotation about the x-axis to {@code matrixOriginal} and stores the result in
    * {@code matrixToPack}.
    * <p>
    * All the matrices can be the same object.
    * </p>
    *
    * <pre>
    *                                 / 1     0          0     \
    * matrixToPack = matrixOriginal * | 0 cos(roll) -sin(roll) |
    *                                 \ 0 sin(roll)  cos(roll) /
    * </pre>
    *
    * @param matrixOriginal the matrix on which the roll rotation is appended. Not modified.
    * @param roll           the angle to rotate about the x-axis.
    * @param matrixToPack   the matrix in which the result is stored. Modified.
    */
   public static void appendRollRotation(Matrix3DReadOnly matrixOriginal, double roll, CommonMatrix3DBasics matrixToPack)
   {
      if (matrixOriginal.isIdentity())
      {
         RotationMatrixConversion.computeRollMatrix(roll, matrixToPack);
         return;
      }

      double cRoll = EuclidCoreTools.cos(roll);
      double sRoll = EuclidCoreTools.sin(roll);

      double m00 = matrixOriginal.getM00();
      double m01 = cRoll * matrixOriginal.getM01() + sRoll * matrixOriginal.getM02();
      double m02 = -sRoll * matrixOriginal.getM01() + cRoll * matrixOriginal.getM02();
      double m10 = matrixOriginal.getM10();
      double m11 = cRoll * matrixOriginal.getM11() + sRoll * matrixOriginal.getM12();
      double m12 = -sRoll * matrixOriginal.getM11() + cRoll * matrixOriginal.getM12();
      double m20 = matrixOriginal.getM20();
      double m21 = cRoll * matrixOriginal.getM21() + sRoll * matrixOriginal.getM22();
      double m22 = -sRoll * matrixOriginal.getM21() + cRoll * matrixOriginal.getM22();
      matrixToPack.set(m00, m01, m02, m10, m11, m12, m20, m21, m22);
   }

   /**
    * Rotates the given {@code tupleOriginal} by a rotation about the z-axis and stores the result in
    * {@code tupleTransformed}.
    * <p>
    * Both tuples can be the same object for performing in-place transformation.
    * </p>
    *
    * <pre>
    *                    / cos(yaw) -sin(yaw) 0 \
    * tupleTransformed = | sin(yaw)  cos(yaw) 0 | * tupleOriginal
    *                    \    0         0     1 /
    * </pre>
    *
    * @param yaw              the angle to rotate about the z-axis.
    * @param tupleOriginal    the tuple to be transformed. Not modified.
    * @param tupleTransformed the tuple in which the result is stored. Modified.
    */
   public static void applyYawRotation(double yaw, Tuple3DReadOnly tupleOriginal, Tuple3DBasics tupleTransformed)
   {
      double cYaw = EuclidCoreTools.cos(yaw);
      double sYaw = EuclidCoreTools.sin(yaw);

      double x = tupleOriginal.getX() * cYaw - tupleOriginal.getY() * sYaw;
      double y = tupleOriginal.getX() * sYaw + tupleOriginal.getY() * cYaw;
      double z = tupleOriginal.getZ();
      tupleTransformed.set(x, y, z);
   }

   /**
    * Rotates the given {@code tupleOriginal} by a rotation about the z-axis and stores the result in
    * {@code tupleTransformed}.
    * <p>
    * Both tuples can be the same object for performing in-place transformation.
    * </p>
    *
    * <pre>
    * tupleTransformed = / cos(yaw) -sin(yaw) \ * tupleOriginal
    *                    \ sin(yaw)  cos(yaw) /
    * </pre>
    *
    * @param yaw              the angle to rotate about the z-axis.
    * @param tupleOriginal    the tuple to be transformed. Not modified.
    * @param tupleTransformed the tuple in which the result is stored. Modified.
    */
   public static void applyYawRotation(double yaw, Tuple2DReadOnly tupleOriginal, Tuple2DBasics tupleTransformed)
   {
      double cYaw = EuclidCoreTools.cos(yaw);
      double sYaw = EuclidCoreTools.sin(yaw);

      double x = tupleOriginal.getX() * cYaw - tupleOriginal.getY() * sYaw;
      double y = tupleOriginal.getX() * sYaw + tupleOriginal.getY() * cYaw;
      tupleTransformed.set(x, y);
   }

   /**
    * Rotates the given {@code tupleOriginal} by a rotation about the y-axis and stores the result in
    * {@code tupleTransformed}.
    * <p>
    * Both tuples can be the same object for performing in-place transformation.
    * </p>
    *
    * <pre>
    *                    /  cos(pitch) 0 sin(pitch) \
    * tupleTransformed = |      0      1     0      | * tupleOriginal
    *                    \ -sin(pitch) 0 cos(pitch) /
    * </pre>
    *
    * @param pitch            the angle to rotate about the y-axis.
    * @param tupleOriginal    the tuple to be transformed. Not modified.
    * @param tupleTransformed the tuple in which the result is stored. Modified.
    */
   public static void applyPitchRotation(double pitch, Tuple3DReadOnly tupleOriginal, Tuple3DBasics tupleTransformed)
   {
      double cPitch = EuclidCoreTools.cos(pitch);
      double sPitch = EuclidCoreTools.sin(pitch);

      double x = tupleOriginal.getX() * cPitch + tupleOriginal.getZ() * sPitch;
      double y = tupleOriginal.getY();
      double z = -tupleOriginal.getX() * sPitch + tupleOriginal.getZ() * cPitch;
      tupleTransformed.set(x, y, z);
   }

   /**
    * Rotates the given {@code tupleOriginal} by a rotation about the x-axis and stores the result in
    * {@code tupleTransformed}.
    * <p>
    * Both tuples can be the same object for performing in-place transformation.
    * </p>
    *
    * <pre>
    *                    / 1     0          0     \
    * tupleTransformed = | 0 cos(roll) -sin(roll) | * tupleOriginal
    *                    \ 0 sin(roll)  cos(roll) /
    * </pre>
    *
    * @param roll             the angle to rotate about the x-axis.
    * @param tupleOriginal    the tuple to be transformed. Not modified.
    * @param tupleTransformed the tuple in which the result is stored. Modified.
    */
   public static void applyRollRotation(double roll, Tuple3DReadOnly tupleOriginal, Tuple3DBasics tupleTransformed)
   {
      double cRoll = EuclidCoreTools.cos(roll);
      double sRoll = EuclidCoreTools.sin(roll);

      double x = tupleOriginal.getX();
      double y = tupleOriginal.getY() * cRoll - tupleOriginal.getZ() * sRoll;
      double z = tupleOriginal.getY() * sRoll + tupleOriginal.getZ() * cRoll;
      tupleTransformed.set(x, y, z);
   }

   /**
    * Performs a linear interpolation in SO(3) from {@code r0} to {@code rf} given the percentage
    * {@code alpha}.
    * <p>
    * This is equivalent to but much more computationally expensive than the <i>Spherical Linear
    * Interpolation</i> performed with quaternions, see
    * {@link QuaternionBasics#interpolate(QuaternionReadOnly, QuaternionReadOnly, double)}.
    * </p>
    *
    * @param r0           the first rotation matrix used in the interpolation. Not modified.
    * @param rf           the second rotation matrix used in the interpolation. Not modified.
    * @param alpha        the percentage to use for the interpolation. A value of 0 will result in
    *                     setting {@code matrixToPack} to {@code r0}, while a value of 1 is equivalent
    *                     to setting {@code matrixToPack} to {@code rf}.
    * @param matrixToPack the rotation matrix in which the result of the interpolation is stored.
    *                     Modified.
    */
   public static void interpolate(RotationMatrixReadOnly r0, RotationMatrixReadOnly rf, double alpha, RotationMatrixBasics matrixToPack)
   {
      if (r0.isZeroOrientation() && rf.isZeroOrientation())
      {
         matrixToPack.setIdentity();
         return;
      }

      if (r0.containsNaN() || rf.containsNaN())
      {
         matrixToPack.setToNaN();
         return;
      }

      if (r0.epsilonEquals(rf, AxisAngleConversion.EPS))
      {
         matrixToPack.set(r0);
         return;
      }

      double m00 = r0.getM00() * rf.getM00() + r0.getM10() * rf.getM10() + r0.getM20() * rf.getM20();
      double m01 = r0.getM00() * rf.getM01() + r0.getM10() * rf.getM11() + r0.getM20() * rf.getM21();
      double m02 = r0.getM00() * rf.getM02() + r0.getM10() * rf.getM12() + r0.getM20() * rf.getM22();
      double m10 = r0.getM01() * rf.getM00() + r0.getM11() * rf.getM10() + r0.getM21() * rf.getM20();
      double m11 = r0.getM01() * rf.getM01() + r0.getM11() * rf.getM11() + r0.getM21() * rf.getM21();
      double m12 = r0.getM01() * rf.getM02() + r0.getM11() * rf.getM12() + r0.getM21() * rf.getM22();
      double m20 = r0.getM02() * rf.getM00() + r0.getM12() * rf.getM10() + r0.getM22() * rf.getM20();
      double m21 = r0.getM02() * rf.getM01() + r0.getM12() * rf.getM11() + r0.getM22() * rf.getM21();
      double m22 = r0.getM02() * rf.getM02() + r0.getM12() * rf.getM12() + r0.getM22() * rf.getM22();

      double angle, x, y, z; // variables for result

      x = m21 - m12;
      y = m02 - m20;
      z = m10 - m01;

      double s = EuclidCoreTools.norm(x, y, z);

      if (s > AxisAngleConversion.EPS)
      {
         double sin = 0.5 * s;
         double cos = 0.5 * (m00 + m11 + m22 - 1.0);
         angle = EuclidCoreTools.atan2(sin, cos);
         x /= s;
         y /= s;
         z /= s;
      }
      else
      {
         // otherwise this singularity is angle = 180
         angle = Math.PI;
         double xx = 0.50 * (m00 + 1.0);
         double yy = 0.50 * (m11 + 1.0);
         double zz = 0.50 * (m22 + 1.0);
         double xy = 0.25 * (m01 + m10);
         double xz = 0.25 * (m02 + m20);
         double yz = 0.25 * (m12 + m21);

         if (xx > yy && xx > zz)
         { // m00 is the largest diagonal term
            x = EuclidCoreTools.squareRoot(xx);
            y = xy / x;
            z = xz / x;
         }
         else if (yy > zz)
         { // m11 is the largest diagonal term
            y = EuclidCoreTools.squareRoot(yy);
            x = xy / y;
            z = yz / y;
         }
         else
         { // m22 is the largest diagonal term so base result on this
            z = EuclidCoreTools.squareRoot(zz);
            x = xz / z;
            y = yz / z;
         }
      }

      angle *= alpha;

      double sinTheta = EuclidCoreTools.sin(angle);
      double cosTheta = EuclidCoreTools.cos(angle);
      double t = 1.0 - cosTheta;

      double xz = x * z;
      double xy = x * y;
      double yz = y * z;

      m00 = t * x * x + cosTheta;
      m01 = t * xy - sinTheta * z;
      m02 = t * xz + sinTheta * y;
      m10 = t * xy + sinTheta * z;
      m11 = t * y * y + cosTheta;
      m12 = t * yz - sinTheta * x;
      m20 = t * xz - sinTheta * y;
      m21 = t * yz + sinTheta * x;
      m22 = t * z * z + cosTheta;

      double r00 = r0.getM00() * m00 + r0.getM01() * m10 + r0.getM02() * m20;
      double r01 = r0.getM00() * m01 + r0.getM01() * m11 + r0.getM02() * m21;
      double r02 = r0.getM00() * m02 + r0.getM01() * m12 + r0.getM02() * m22;
      double r10 = r0.getM10() * m00 + r0.getM11() * m10 + r0.getM12() * m20;
      double r11 = r0.getM10() * m01 + r0.getM11() * m11 + r0.getM12() * m21;
      double r12 = r0.getM10() * m02 + r0.getM11() * m12 + r0.getM12() * m22;
      double r20 = r0.getM20() * m00 + r0.getM21() * m10 + r0.getM22() * m20;
      double r21 = r0.getM20() * m01 + r0.getM21() * m11 + r0.getM22() * m21;
      double r22 = r0.getM20() * m02 + r0.getM21() * m12 + r0.getM22() * m22;

      matrixToPack.set(r00, r01, r02, r10, r11, r12, r20, r21, r22);
   }

   /**
    * Performs a Cross platform Angular Distance Calculation between Rotation Matrix and any other 3D
    * orientation systems.
    *
    * @param rotationMatrix the rotation matrix to be used for comparison. Not modified.
    * @param orientation3D  the orientation3D to be used for comparison. Not modified.
    * @return The angle between roationMatrix and orientation3D, contained in [0, <i>pi</i>].
    */
   public static double distance(RotationMatrixReadOnly rotationMatrix, Orientation3DReadOnly orientation3D)
   {
      if (orientation3D instanceof QuaternionReadOnly)
      {
         return distance(rotationMatrix, (QuaternionReadOnly) orientation3D);
      }
      if (orientation3D instanceof YawPitchRollReadOnly)
      {
         return distance(rotationMatrix, (YawPitchRollReadOnly) orientation3D);
      }
      if (orientation3D instanceof AxisAngleReadOnly)
      {
         return distance(rotationMatrix, (AxisAngleReadOnly) orientation3D);
      }
      if (orientation3D instanceof RotationMatrixReadOnly)
      {
         return distance(rotationMatrix, (RotationMatrixReadOnly) orientation3D);
      }
      else
      {
         throw new UnsupportedOperationException("Unsupported type: " + orientation3D.getClass().getSimpleName());
      }
   }

   /**
    * Computes and returns the distance between rotation matrix and Quaternion.
    *
    * @param rotationMatrix the rotation matrix to be used for comparison. Not modified.
    * @param quaternion     the quaternion to be used for comparison. Not modified.
    * @return the angle representing the distance between the two orientations. It is contained in [0,
    *       <i>pi</i>].
    */
   public static double distance(RotationMatrixReadOnly rotationMatrix, QuaternionReadOnly quaternion)
   {
      return QuaternionTools.distance(quaternion, rotationMatrix);
   }

   /**
    * Computes and returns the distance between rotation matrix and axis angle.
    *
    * @param rotationMatrix the rotation matrix to be used for comparison. Not modified.
    * @param axisAngle      the axisAngle to be used for comparison. Not modified.
    * @return the angle representing the distance between the two rotation matrices. It is contained in
    *       [0, <i>pi</i>].
    */
   public static double distance(RotationMatrixReadOnly rotationMatrix, AxisAngleReadOnly axisAngle)
   {
      return AxisAngleTools.distance(axisAngle, rotationMatrix);
   }

   /**
    * Computes and returns the distance between rotation matrix and yaw pitch roll.
    *
    * @param rotationMatrix the rotation matrix to be used for comparison. Not modified.
    * @param yawPitchRoll   the yawPitchRoll to be used for comparison. Not modified.
    * @return the angle representing the distance between the two rotation matrices. It is contained in
    *       [0, <i>pi</i>].
    */
   public static double distance(RotationMatrixReadOnly rotationMatrix, YawPitchRollReadOnly yawPitchRoll)
   {
      if (rotationMatrix.containsNaN() || yawPitchRoll.containsNaN())
      {
         return Double.NaN;
      }
      if (rotationMatrix.isZeroOrientation(EPS))
      {
         return YawPitchRollTools.angle(yawPitchRoll);
      }
      if (yawPitchRoll.isZeroOrientation(EPS))
      {
         return RotationMatrixTools.angle(rotationMatrix);
      }
      double yaw = yawPitchRoll.getYaw();
      double pitch = yawPitchRoll.getPitch();
      double roll = yawPitchRoll.getRoll();
      double m00, m01, m02, m10, m11, m12, m20, m21, m22;

      double cosc = EuclidCoreTools.cos(yaw);
      double sinc = EuclidCoreTools.sin(yaw);

      double cosb = EuclidCoreTools.cos(pitch);
      double sinb = EuclidCoreTools.sin(pitch);

      double cosa = EuclidCoreTools.cos(roll);
      double sina = EuclidCoreTools.sin(roll);

      // Introduction to Robotics, 2.64
      m00 = cosc * cosb;
      m01 = cosc * sinb * sina - sinc * cosa;
      m02 = cosc * sinb * cosa + sinc * sina;
      m10 = sinc * cosb;
      m11 = sinc * sinb * sina + cosc * cosa;
      m12 = sinc * sinb * cosa - cosc * sina;
      m20 = -sinb;
      m21 = cosb * sina;
      m22 = cosb * cosa;
      return RotationMatrixTools.distance(rotationMatrix, m00, m01, m02, m10, m11, m12, m20, m21, m22);
   }

   /**
    * Computes and returns the distance from the rotation matrix {@code m1} to {@code m2}.
    *
    * @param a the first rotation matrix. Not modified.
    * @param b the second rotation matrix. Not modified.
    * @return the angle representing the distance between the two rotation matrices. It is contained in
    *       [0, <i>pi</i>].
    */
   public static double distance(RotationMatrixReadOnly a, RotationMatrixReadOnly b)
   {
      double b00 = b.getM00();
      double b01 = b.getM01();
      double b02 = b.getM02();
      double b10 = b.getM10();
      double b11 = b.getM11();
      double b12 = b.getM12();
      double b20 = b.getM20();
      double b21 = b.getM21();
      double b22 = b.getM22();
      return distance(a, b00, b01, b02, b10, b11, b12, b20, b21, b22);
   }

   static double distance(RotationMatrixReadOnly a, double b00, double b01, double b02, double b10, double b11, double b12, double b20, double b21, double b22)
   {
      double m00 = a.getM00() * b00 + a.getM01() * b01 + a.getM02() * b02;
      double m01 = a.getM00() * b10 + a.getM01() * b11 + a.getM02() * b12;
      double m02 = a.getM00() * b20 + a.getM01() * b21 + a.getM02() * b22;
      double m10 = a.getM10() * b00 + a.getM11() * b01 + a.getM12() * b02;
      double m11 = a.getM10() * b10 + a.getM11() * b11 + a.getM12() * b12;
      double m12 = a.getM10() * b20 + a.getM11() * b21 + a.getM12() * b22;
      double m20 = a.getM20() * b00 + a.getM21() * b01 + a.getM22() * b02;
      double m21 = a.getM20() * b10 + a.getM21() * b11 + a.getM22() * b12;
      double m22 = a.getM20() * b20 + a.getM21() * b21 + a.getM22() * b22;

      return angle(m00, m01, m02, m10, m11, m12, m20, m21, m22);
   }

   // TODO Test me against Quaternion.angle or AxisAngle.angle - - - - - D O N E

   /**
    * Computes and returns the angular distance of given rotation matrix from origin.
    *
    * @param m the rotation matrix to be used for comparison. Not modified
    * @return the angle representing the distance from origin (zero orientation). It is contained in
    *       [0, <i>pi</i>].
    */
   public static double angle(RotationMatrixReadOnly m)
   {
      double m00 = m.getM00();
      double m01 = m.getM01();
      double m02 = m.getM02();
      double m10 = m.getM10();
      double m11 = m.getM11();
      double m12 = m.getM12();
      double m20 = m.getM20();
      double m21 = m.getM21();
      double m22 = m.getM22();
      return angle(m00, m01, m02, m10, m11, m12, m20, m21, m22);
   }

   /**
    * Computes and returns the angular distance of given rotation matrix from origin.
    *
    * @param m00 element at (0,0)
    * @param m01 element at (0,1)
    * @param m02 element at (0,2)
    * @param m10 element at (1,0)
    * @param m11 element at (1,1)
    * @param m12 element at (1,2)
    * @param m20 element at (2,0)
    * @param m21 element at (2,1)
    * @param m22 element at (2,2)
    * @return the angle representing the distance from origin (zero orientation). It is contained in
    *       [0, <i>pi</i>].
    */
   public static double angle(double m00, double m01, double m02, double m10, double m11, double m12, double m20, double m21, double m22)
   {
      double angle, x, y, z; // variables for result

      x = m21 - m12;
      y = m02 - m20;
      z = m10 - m01;

      double s = EuclidCoreTools.fastNorm(x, y, z);

      if (s > AxisAngleConversion.EPS)
      {
         double sin = 0.5 * s;
         double cos = 0.5 * (m00 + m11 + m22 - 1.0);
         angle = EuclidCoreTools.atan2(sin, cos);
      }
      else if (m00 + m11 + m22 > 3.0 - 1.0e-7)
      { // At this point, the matrix has to be identity.
         return 0.0;
      }
      else
      {
         // otherwise this singularity is angle = 180
         angle = Math.PI;
      }

      return angle;
   }

   /**
    * Computes the angular velocity from the finite difference of two orientations.
    *
    * @param previousOrientation   the orientation at the previous time step. Not modified.
    * @param currentOrientation    the orientation at the current time step. Not modified.
    * @param dt                    the time step.
    * @param angularVelocityToPack the vector used to store the angular velocity expressed in the orientation's local coordinates. Modified.
    * @see EuclidCoreTools#finiteDifference(Orientation3DReadOnly, Orientation3DReadOnly, double, Vector3DBasics)
    */
   public static void finiteDifference(RotationMatrixReadOnly previousOrientation,
                                       RotationMatrixReadOnly currentOrientation,
                                       double dt,
                                       Vector3DBasics angularVelocityToPack)
   {
      finiteDifference(previousOrientation.getM00(),
                       previousOrientation.getM01(),
                       previousOrientation.getM02(),
                       previousOrientation.getM10(),
                       previousOrientation.getM11(),
                       previousOrientation.getM12(),
                       previousOrientation.getM20(),
                       previousOrientation.getM21(),
                       previousOrientation.getM22(),
                       currentOrientation.getM00(),
                       currentOrientation.getM01(),
                       currentOrientation.getM02(),
                       currentOrientation.getM10(),
                       currentOrientation.getM11(),
                       currentOrientation.getM12(),
                       currentOrientation.getM20(),
                       currentOrientation.getM21(),
                       currentOrientation.getM22(),
                       dt,
                       angularVelocityToPack);
   }

   /**
    * Computes the angular velocity from the finite difference of two orientations.
    *
    * @param previousOrientation   the orientation at the previous time step. Not modified.
    * @param currentOrientation    the orientation at the current time step. Not modified.
    * @param dt                    the time step.
    * @param angularVelocityToPack the vector used to store the angular velocity expressed in the orientation's local coordinates. Modified.
    * @see EuclidCoreTools#finiteDifference(Orientation3DReadOnly, Orientation3DReadOnly, double, Vector3DBasics)
    */
   public static void finiteDifference(RotationMatrixReadOnly previousOrientation,
                                       QuaternionReadOnly currentOrientation,
                                       double dt,
                                       Vector3DBasics angularVelocityToPack)
   {
      finiteDifferenceImpl(previousOrientation,
                           currentOrientation.getX(),
                           currentOrientation.getY(),
                           currentOrientation.getZ(),
                           currentOrientation.getS(),
                           dt,
                           angularVelocityToPack);
   }

   /**
    * Computes the angular velocity from the finite difference of two orientations.
    *
    * @param previousOrientation   the orientation at the previous time step. Not modified.
    * @param currentOrientation    the orientation at the current time step. Not modified.
    * @param dt                    the time step.
    * @param angularVelocityToPack the vector used to store the angular velocity expressed in the orientation's local coordinates. Modified.
    * @see EuclidCoreTools#finiteDifference(Orientation3DReadOnly, Orientation3DReadOnly, double, Vector3DBasics)
    */
   public static void finiteDifference(RotationMatrixReadOnly previousOrientation,
                                       AxisAngleReadOnly currentOrientation,
                                       double dt,
                                       Vector3DBasics angularVelocityToPack)
   {
      double halfAngle = 0.5 * currentOrientation.getAngle();
      double sinHalfAngle = Math.sin(halfAngle);
      double xCurr = currentOrientation.getX() * sinHalfAngle;
      double yCurr = currentOrientation.getY() * sinHalfAngle;
      double zCurr = currentOrientation.getZ() * sinHalfAngle;
      double sCurr = Math.cos(halfAngle);
      finiteDifferenceImpl(previousOrientation, xCurr, yCurr, zCurr, sCurr, dt, angularVelocityToPack);
   }

   private static void finiteDifferenceImpl(RotationMatrixReadOnly previousOrientation,
                                            double xCurr,
                                            double yCurr,
                                            double zCurr,
                                            double sCurr,
                                            double dt,
                                            Vector3DBasics angularVelocityToPack)
   {
      double yy2 = 2.0 * yCurr * yCurr;
      double zz2 = 2.0 * zCurr * zCurr;
      double xx2 = 2.0 * xCurr * xCurr;
      double xy2 = 2.0 * xCurr * yCurr;
      double sz2 = 2.0 * sCurr * zCurr;
      double xz2 = 2.0 * xCurr * zCurr;
      double sy2 = 2.0 * sCurr * yCurr;
      double yz2 = 2.0 * yCurr * zCurr;
      double sx2 = 2.0 * sCurr * xCurr;

      double c00 = 1.0 - yy2 - zz2;
      double c01 = xy2 - sz2;
      double c02 = xz2 + sy2;
      double c10 = xy2 + sz2;
      double c11 = 1.0 - xx2 - zz2;
      double c12 = yz2 - sx2;
      double c20 = xz2 - sy2;
      double c21 = yz2 + sx2;
      double c22 = 1.0 - xx2 - yy2;

      finiteDifference(previousOrientation.getM00(),
                       previousOrientation.getM01(),
                       previousOrientation.getM02(),
                       previousOrientation.getM10(),
                       previousOrientation.getM11(),
                       previousOrientation.getM12(),
                       previousOrientation.getM20(),
                       previousOrientation.getM21(),
                       previousOrientation.getM22(),
                       c00,
                       c01,
                       c02,
                       c10,
                       c11,
                       c12,
                       c20,
                       c21,
                       c22,
                       dt,
                       angularVelocityToPack);
   }

   /**
    * Computes the angular velocity from the finite difference of two orientations.
    *
    * @param previousOrientation   the orientation at the previous time step. Not modified.
    * @param currentOrientation    the orientation at the current time step. Not modified.
    * @param dt                    the time step.
    * @param angularVelocityToPack the vector used to store the angular velocity expressed in the orientation's local coordinates. Modified.
    * @see EuclidCoreTools#finiteDifference(Orientation3DReadOnly, Orientation3DReadOnly, double, Vector3DBasics)
    */
   public static void finiteDifference(QuaternionReadOnly previousOrientation,
                                       RotationMatrixReadOnly currentOrientation,
                                       double dt,
                                       Vector3DBasics angularVelocityToPack)
   {

      finiteDifferenceImpl(previousOrientation.getX(),
                           previousOrientation.getY(),
                           previousOrientation.getZ(),
                           previousOrientation.getS(),
                           currentOrientation,
                           dt,
                           angularVelocityToPack);
   }

   /**
    * Computes the angular velocity from the finite difference of two orientations.
    *
    * @param previousOrientation   the orientation at the previous time step. Not modified.
    * @param currentOrientation    the orientation at the current time step. Not modified.
    * @param dt                    the time step.
    * @param angularVelocityToPack the vector used to store the angular velocity expressed in the orientation's local coordinates. Modified.
    * @see EuclidCoreTools#finiteDifference(Orientation3DReadOnly, Orientation3DReadOnly, double, Vector3DBasics)
    */
   public static void finiteDifference(AxisAngleReadOnly previousOrientation,
                                       RotationMatrixReadOnly currentOrientation,
                                       double dt,
                                       Vector3DBasics angularVelocityToPack)
   {
      double halfAngle = 0.5 * previousOrientation.getAngle();
      double sinHalfAngle = Math.sin(halfAngle);
      double xPrev = previousOrientation.getX() * sinHalfAngle;
      double yPrev = previousOrientation.getY() * sinHalfAngle;
      double zPrev = previousOrientation.getZ() * sinHalfAngle;
      double sPrev = Math.cos(halfAngle);
      finiteDifferenceImpl(xPrev, yPrev, zPrev, sPrev, currentOrientation, dt, angularVelocityToPack);
   }

   private static void finiteDifferenceImpl(double xPrev,
                                            double yPrev,
                                            double zPrev,
                                            double sPrev,
                                            RotationMatrixReadOnly currentOrientation,
                                            double dt,
                                            Vector3DBasics angularVelocityToPack)
   {
      double yy2 = 2.0 * yPrev * yPrev;
      double zz2 = 2.0 * zPrev * zPrev;
      double xx2 = 2.0 * xPrev * xPrev;
      double xy2 = 2.0 * xPrev * yPrev;
      double sz2 = 2.0 * sPrev * zPrev;
      double xz2 = 2.0 * xPrev * zPrev;
      double sy2 = 2.0 * sPrev * yPrev;
      double yz2 = 2.0 * yPrev * zPrev;
      double sx2 = 2.0 * sPrev * xPrev;

      double c00 = 1.0 - yy2 - zz2;
      double c01 = xy2 - sz2;
      double c02 = xz2 + sy2;
      double c10 = xy2 + sz2;
      double c11 = 1.0 - xx2 - zz2;
      double c12 = yz2 - sx2;
      double c20 = xz2 - sy2;
      double c21 = yz2 + sx2;
      double c22 = 1.0 - xx2 - yy2;

      finiteDifference(c00,
                       c01,
                       c02,
                       c10,
                       c11,
                       c12,
                       c20,
                       c21,
                       c22,
                       currentOrientation.getM00(),
                       currentOrientation.getM01(),
                       currentOrientation.getM02(),
                       currentOrientation.getM10(),
                       currentOrientation.getM11(),
                       currentOrientation.getM12(),
                       currentOrientation.getM20(),
                       currentOrientation.getM21(),
                       currentOrientation.getM22(),
                       dt,
                       angularVelocityToPack);
   }

   /**
    * Computes the angular velocity from the finite difference of two orientations.
    *
    * @param previousOrientation   the orientation at the previous time step. Not modified.
    * @param currentOrientation    the orientation at the current time step. Not modified.
    * @param dt                    the time step.
    * @param angularVelocityToPack the vector used to store the angular velocity expressed in the orientation's local coordinates. Modified.
    * @see EuclidCoreTools#finiteDifference(Orientation3DReadOnly, Orientation3DReadOnly, double, Vector3DBasics)
    */
   public static void finiteDifference(YawPitchRollReadOnly previousOrientation,
                                       RotationMatrixReadOnly currentOrientation,
                                       double dt,
                                       Vector3DBasics angularVelocityToPack)
   {
      double yawPrev = previousOrientation.getYaw();
      double pitchPrev = previousOrientation.getPitch();
      double rollPrev = previousOrientation.getRoll();

      double coscPrev = EuclidCoreTools.cos(yawPrev);
      double sincPrev = EuclidCoreTools.sin(yawPrev);

      double cosbPrev = EuclidCoreTools.cos(pitchPrev);
      double sinbPrev = EuclidCoreTools.sin(pitchPrev);

      double cosaPrev = EuclidCoreTools.cos(rollPrev);
      double sinaPrev = EuclidCoreTools.sin(rollPrev);

      // Introduction to Robotics, 2.64
      double p00 = coscPrev * cosbPrev;
      double p01 = coscPrev * sinbPrev * sinaPrev - sincPrev * cosaPrev;
      double p02 = coscPrev * sinbPrev * cosaPrev + sincPrev * sinaPrev;
      double p10 = sincPrev * cosbPrev;
      double p11 = sincPrev * sinbPrev * sinaPrev + coscPrev * cosaPrev;
      double p12 = sincPrev * sinbPrev * cosaPrev - coscPrev * sinaPrev;
      double p20 = -sinbPrev;
      double p21 = cosbPrev * sinaPrev;
      double p22 = cosbPrev * cosaPrev;

      finiteDifference(p00,
                       p01,
                       p02,
                       p10,
                       p11,
                       p12,
                       p20,
                       p21,
                       p22,
                       currentOrientation.getM00(),
                       currentOrientation.getM01(),
                       currentOrientation.getM02(),
                       currentOrientation.getM10(),
                       currentOrientation.getM11(),
                       currentOrientation.getM12(),
                       currentOrientation.getM20(),
                       currentOrientation.getM21(),
                       currentOrientation.getM22(),
                       dt,
                       angularVelocityToPack);
   }

   /**
    * Computes the angular velocity from the finite difference of two orientations.
    *
    * @param previousOrientation   the orientation at the previous time step. Not modified.
    * @param currentOrientation    the orientation at the current time step. Not modified.
    * @param dt                    the time step.
    * @param angularVelocityToPack the vector used to store the angular velocity expressed in the orientation's local coordinates. Modified.
    * @see EuclidCoreTools#finiteDifference(Orientation3DReadOnly, Orientation3DReadOnly, double, Vector3DBasics)
    */
   public static void finiteDifference(RotationMatrixReadOnly previousOrientation,
                                       YawPitchRollReadOnly currentOrientation,
                                       double dt,
                                       Vector3DBasics angularVelocityToPack)
   {
      double p00 = previousOrientation.getM00();
      double p01 = previousOrientation.getM01();
      double p02 = previousOrientation.getM02();
      double p10 = previousOrientation.getM10();
      double p11 = previousOrientation.getM11();
      double p12 = previousOrientation.getM12();
      double p20 = previousOrientation.getM20();
      double p21 = previousOrientation.getM21();
      double p22 = previousOrientation.getM22();

      finiteDifferenceImpl(p00, p01, p02, p10, p11, p12, p20, p21, p22, currentOrientation, dt, angularVelocityToPack);
   }

   static void finiteDifferenceImpl(double p00,
                                    double p01,
                                    double p02,
                                    double p10,
                                    double p11,
                                    double p12,
                                    double p20,
                                    double p21,
                                    double p22,
                                    YawPitchRollReadOnly currentOrientation,
                                    double dt,
                                    Vector3DBasics angularVelocityToPack)
   {
      double yawCurr = currentOrientation.getYaw();
      double pitchCurr = currentOrientation.getPitch();
      double rollCurr = currentOrientation.getRoll();

      double coscCurr = EuclidCoreTools.cos(yawCurr);
      double sincCurr = EuclidCoreTools.sin(yawCurr);

      double cosbCurr = EuclidCoreTools.cos(pitchCurr);
      double sinbCurr = EuclidCoreTools.sin(pitchCurr);

      double cosaCurr = EuclidCoreTools.cos(rollCurr);
      double sinaCurr = EuclidCoreTools.sin(rollCurr);

      // Introduction to Robotics, 2.64
      double c00 = coscCurr * cosbCurr;
      double c01 = coscCurr * sinbCurr * sinaCurr - sincCurr * cosaCurr;
      double c02 = coscCurr * sinbCurr * cosaCurr + sincCurr * sinaCurr;
      double c10 = sincCurr * cosbCurr;
      double c11 = sincCurr * sinbCurr * sinaCurr + coscCurr * cosaCurr;
      double c12 = sincCurr * sinbCurr * cosaCurr - coscCurr * sinaCurr;
      double c20 = -sinbCurr;
      double c21 = cosbCurr * sinaCurr;
      double c22 = cosbCurr * cosaCurr;

      finiteDifference(p00, p01, p02, p10, p11, p12, p20, p21, p22, c00, c01, c02, c10, c11, c12, c20, c21, c22, dt, angularVelocityToPack);
   }

   static void finiteDifference(double p00,
                                double p01,
                                double p02,
                                double p10,
                                double p11,
                                double p12,
                                double p20,
                                double p21,
                                double p22,
                                double c00,
                                double c01,
                                double c02,
                                double c10,
                                double c11,
                                double c12,
                                double c20,
                                double c21,
                                double c22,
                                double dt,
                                Vector3DBasics angularVelocityToPack)
   {
      // Multiply the transpose of the previous orientation by the current orientation.
      double m00 = p00 * c00 + p10 * c10 + p20 * c20;
      double m01 = p00 * c01 + p10 * c11 + p20 * c21;
      double m02 = p00 * c02 + p10 * c12 + p20 * c22;
      double m10 = p01 * c00 + p11 * c10 + p21 * c20;
      double m11 = p01 * c01 + p11 * c11 + p21 * c21;
      double m12 = p01 * c02 + p11 * c12 + p21 * c22;
      double m20 = p02 * c00 + p12 * c10 + p22 * c20;
      double m21 = p02 * c01 + p12 * c11 + p22 * c21;
      double m22 = p02 * c02 + p12 * c12 + p22 * c22;

      RotationVectorConversion.convertMatrixToRotationVector(m00, m01, m02, m10, m11, m12, m20, m21, m22, angularVelocityToPack);
      angularVelocityToPack.scale(1.0 / dt);
   }
}
