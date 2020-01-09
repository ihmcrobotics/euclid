package us.ihmc.euclid.tools;

import us.ihmc.euclid.matrix.RotationMatrix;
import us.ihmc.euclid.matrix.interfaces.RotationMatrixReadOnly;
import us.ihmc.euclid.orientation.interfaces.Orientation3DReadOnly;
import us.ihmc.euclid.rotationConversion.AxisAngleConversion;
import us.ihmc.euclid.rotationConversion.RotationMatrixConversion;
import us.ihmc.euclid.tuple2D.interfaces.Tuple2DBasics;
import us.ihmc.euclid.tuple2D.interfaces.Tuple2DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DBasics;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DReadOnly;
import us.ihmc.euclid.tuple4D.interfaces.QuaternionBasics;
import us.ihmc.euclid.tuple4D.interfaces.QuaternionReadOnly;

/**
 * Tools for performing operations on rotation matrices.
 * 
 * @author Sylvain Bertrand
 */
public class RotationMatrixTools
{
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
   public static void multiply(RotationMatrixReadOnly m1, RotationMatrixReadOnly m2, RotationMatrix matrixToPack)
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
   public static void multiplyTransposeBoth(RotationMatrixReadOnly m1, RotationMatrixReadOnly m2, RotationMatrix matrixToPack)
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
   public static void multiplyTransposeLeft(RotationMatrixReadOnly m1, RotationMatrixReadOnly m2, RotationMatrix matrixToPack)
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
   public static void multiplyTransposeRight(RotationMatrixReadOnly m1, RotationMatrixReadOnly m2, RotationMatrix matrixToPack)
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
   public static void multiply(Orientation3DReadOnly orientation1, boolean inverse1, Orientation3DReadOnly orientation2, boolean inverse2,
                               RotationMatrix matrixToPack)
   {
      if (orientation1 instanceof RotationMatrixReadOnly)
      {
         multiply((RotationMatrixReadOnly) orientation1, inverse1, orientation2, inverse2, matrixToPack);
         return;
      }

      if (orientation1.isZeroOrientation())
      {
         if (orientation2.isZeroOrientation())
            matrixToPack.setToZero();
         else if (inverse2)
            matrixToPack.setAndInvert(orientation2);
         else
            matrixToPack.set(orientation2);

         return;
      }
      else if (orientation2.isZeroOrientation())
      {
         if (inverse1)
            matrixToPack.setAndInvert(orientation1);
         else
            matrixToPack.set(orientation1);

         return;
      }

      double b00, b01, b02, b10, b11, b12, b20, b21, b22;
      if (orientation2 instanceof RotationMatrixReadOnly)
      { // In this case orientation2 might be the same object as matrixToPack, so let's save its components first.
         RotationMatrixReadOnly b = (RotationMatrixReadOnly) orientation2;
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
   public static void multiply(Orientation3DReadOnly orientation1, boolean inverse1, RotationMatrixReadOnly orientation2, boolean inverse2,
                               RotationMatrix matrixToPack)
   {
      if (orientation1 instanceof RotationMatrixReadOnly)
      {
         multiplyImpl((RotationMatrixReadOnly) orientation1, inverse1, orientation2, inverse2, matrixToPack);
         return;
      }

      if (orientation1.isZeroOrientation())
      {
         if (orientation2.isZeroOrientation())
            matrixToPack.setToZero();
         else if (inverse2)
            matrixToPack.setAndInvert(orientation2);
         else
            matrixToPack.set(orientation2);

         return;
      }
      else if (orientation2.isZeroOrientation())
      {
         if (inverse1)
            matrixToPack.setAndInvert(orientation1);
         else
            matrixToPack.set(orientation1);

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
   public static void multiply(RotationMatrixReadOnly orientation1, boolean inverse1, Orientation3DReadOnly orientation2, boolean inverse2,
                               RotationMatrix matrixToPack)
   {
      if (orientation2 instanceof RotationMatrixReadOnly)
      {
         multiplyImpl(orientation1, inverse1, (RotationMatrixReadOnly) orientation2, inverse2, matrixToPack);
         return;
      }

      if (orientation1.isZeroOrientation())
      {
         if (orientation2.isZeroOrientation())
            matrixToPack.setToZero();
         else if (inverse2)
            matrixToPack.setAndInvert(orientation2);
         else
            matrixToPack.set(orientation2);

         return;
      }
      else if (orientation2.isZeroOrientation())
      {
         if (inverse1)
            matrixToPack.setAndInvert(orientation1);
         else
            matrixToPack.set(orientation1);

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
   private static void multiplyImpl(RotationMatrixReadOnly a, boolean transposeA, RotationMatrixReadOnly b, boolean transposeB, RotationMatrix matrixToPack)
   {
      if (a.isZeroOrientation())
      {
         if (b.isZeroOrientation())
            matrixToPack.setToZero();
         else if (transposeB)
            matrixToPack.setAndInvert(b);
         else
            matrixToPack.set(b);

         return;
      }
      else if (b.isZeroOrientation())
      {
         if (transposeA)
            matrixToPack.setAndInvert(a);
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

   private static void multiplyImpl(double a00, double a01, double a02, double a10, double a11, double a12, double a20, double a21, double a22,
                                    boolean transposeA, double b00, double b01, double b02, double b10, double b11, double b12, double b20, double b21,
                                    double b22, boolean transposeB, RotationMatrix matrixToPack)
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
   public static void prependYawRotation(double yaw, RotationMatrixReadOnly matrixOriginal, RotationMatrix matrixToPack)
   {
      if (matrixOriginal.isZeroOrientation())
      {
         RotationMatrixConversion.computeYawMatrix(yaw, matrixToPack);
         return;
      }

      double cYaw = Math.cos(yaw);
      double sYaw = Math.sin(yaw);

      double m00 = cYaw * matrixOriginal.getM00() - sYaw * matrixOriginal.getM10();
      double m01 = cYaw * matrixOriginal.getM01() - sYaw * matrixOriginal.getM11();
      double m02 = cYaw * matrixOriginal.getM02() - sYaw * matrixOriginal.getM12();
      double m10 = sYaw * matrixOriginal.getM00() + cYaw * matrixOriginal.getM10();
      double m11 = sYaw * matrixOriginal.getM01() + cYaw * matrixOriginal.getM11();
      double m12 = sYaw * matrixOriginal.getM02() + cYaw * matrixOriginal.getM12();
      double m20 = matrixOriginal.getM20();
      double m21 = matrixOriginal.getM21();
      double m22 = matrixOriginal.getM22();
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
   public static void appendYawRotation(RotationMatrixReadOnly matrixOriginal, double yaw, RotationMatrix matrixToPack)
   {
      if (matrixOriginal.isZeroOrientation())
      {
         RotationMatrixConversion.computeYawMatrix(yaw, matrixToPack);
         return;
      }

      double cYaw = Math.cos(yaw);
      double sYaw = Math.sin(yaw);

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
   public static void prependPitchRotation(double pitch, RotationMatrixReadOnly matrixOriginal, RotationMatrix matrixToPack)
   {
      if (matrixOriginal.isZeroOrientation())
      {
         RotationMatrixConversion.computePitchMatrix(pitch, matrixToPack);
         return;
      }

      double cPitch = Math.cos(pitch);
      double sPitch = Math.sin(pitch);

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
   public static void appendPitchRotation(RotationMatrixReadOnly matrixOriginal, double pitch, RotationMatrix matrixToPack)
   {
      if (matrixOriginal.isZeroOrientation())
      {
         RotationMatrixConversion.computePitchMatrix(pitch, matrixToPack);
         return;
      }

      double cPitch = Math.cos(pitch);
      double sPitch = Math.sin(pitch);

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
   public static void prependRollRotation(double roll, RotationMatrixReadOnly matrixOriginal, RotationMatrix matrixToPack)
   {
      if (matrixOriginal.isZeroOrientation())
      {
         RotationMatrixConversion.computeRollMatrix(roll, matrixToPack);
         return;
      }

      double cRoll = Math.cos(roll);
      double sRoll = Math.sin(roll);

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
   public static void appendRollRotation(RotationMatrixReadOnly matrixOriginal, double roll, RotationMatrix matrixToPack)
   {
      if (matrixOriginal.isZeroOrientation())
      {
         RotationMatrixConversion.computeRollMatrix(roll, matrixToPack);
         return;
      }

      double cRoll = Math.cos(roll);
      double sRoll = Math.sin(roll);

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
      double cYaw = Math.cos(yaw);
      double sYaw = Math.sin(yaw);

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
      double cYaw = Math.cos(yaw);
      double sYaw = Math.sin(yaw);

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
      double cPitch = Math.cos(pitch);
      double sPitch = Math.sin(pitch);

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
      double cRoll = Math.cos(roll);
      double sRoll = Math.sin(roll);

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
   public static void interpolate(RotationMatrixReadOnly r0, RotationMatrixReadOnly rf, double alpha, RotationMatrix matrixToPack)
   {
      if (r0.isZeroOrientation() && rf.isZeroOrientation())
      {
         matrixToPack.setToZero();
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

      double s = EuclidCoreTools.squareRoot(EuclidCoreTools.normSquared(x, y, z));

      if (s > AxisAngleConversion.EPS)
      {
         double sin = 0.5 * s;
         double cos = 0.5 * (m00 + m11 + m22 - 1.0);
         angle = Math.atan2(sin, cos);
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

      double sinTheta = Math.sin(angle);
      double cosTheta = Math.cos(angle);
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
    * Computes and returns the distance from the rotation matrix {@code m1} to {@code m2}.
    *
    * @param m1 the first rotation matrix. Not modified.
    * @param m2 the second rotation matrix. Not modified.
    * @return the angle representing the distance between the two rotation matrices. It is contained in
    *         [0, <i>pi</i>].
    */
   public static double distance(RotationMatrixReadOnly m1, RotationMatrixReadOnly m2)
   {
      double m00 = m1.getM00() * m2.getM00() + m1.getM01() * m2.getM01() + m1.getM02() * m2.getM02();
      double m01 = m1.getM00() * m2.getM10() + m1.getM01() * m2.getM11() + m1.getM02() * m2.getM12();
      double m02 = m1.getM00() * m2.getM20() + m1.getM01() * m2.getM21() + m1.getM02() * m2.getM22();
      double m10 = m1.getM10() * m2.getM00() + m1.getM11() * m2.getM01() + m1.getM12() * m2.getM02();
      double m11 = m1.getM10() * m2.getM10() + m1.getM11() * m2.getM11() + m1.getM12() * m2.getM12();
      double m12 = m1.getM10() * m2.getM20() + m1.getM11() * m2.getM21() + m1.getM12() * m2.getM22();
      double m20 = m1.getM20() * m2.getM00() + m1.getM21() * m2.getM01() + m1.getM22() * m2.getM02();
      double m21 = m1.getM20() * m2.getM10() + m1.getM21() * m2.getM11() + m1.getM22() * m2.getM12();
      double m22 = m1.getM20() * m2.getM20() + m1.getM21() * m2.getM21() + m1.getM22() * m2.getM22();

      double angle, x, y, z; // variables for result

      x = m21 - m12;
      y = m02 - m20;
      z = m10 - m01;

      double s = EuclidCoreTools.fastNorm(x, y, z);

      if (s > AxisAngleConversion.EPS)
      {
         double sin = 0.5 * s;
         double cos = 0.5 * (m00 + m11 + m22 - 1.0);
         angle = Math.atan2(sin, cos);
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
}
