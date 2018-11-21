package us.ihmc.euclid.tools;

import us.ihmc.euclid.exceptions.NotAMatrix2DException;
import us.ihmc.euclid.exceptions.SingularMatrixException;
import us.ihmc.euclid.matrix.interfaces.CommonMatrix3DBasics;
import us.ihmc.euclid.matrix.interfaces.Matrix3DBasics;
import us.ihmc.euclid.matrix.interfaces.Matrix3DReadOnly;
import us.ihmc.euclid.matrix.interfaces.RotationMatrixReadOnly;
import us.ihmc.euclid.matrix.interfaces.RotationScaleMatrixReadOnly;
import us.ihmc.euclid.tuple2D.interfaces.Tuple2DBasics;
import us.ihmc.euclid.tuple2D.interfaces.Tuple2DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DBasics;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DReadOnly;
import us.ihmc.euclid.tuple4D.interfaces.Vector4DBasics;
import us.ihmc.euclid.tuple4D.interfaces.Vector4DReadOnly;

/**
 * Tools for performing operations on 3D matrices.
 * 
 * @author Sylvain Bertrand
 *
 */
public abstract class Matrix3DTools
{
   static final double EPS_INVERT = 1.0e-16;

   /**
    * Performs an in-place inversion of the given matrix such that: m = m<sup>-1</sup>.
    *
    * @param matrixToInvert the matrix to invert. Modified.
    * @return {@code true} if the inversion succeeds, {@code false} if the matrix is singular.
    */
   public static boolean invert(Matrix3DBasics matrixToInvert)
   {
      return invert(matrixToInvert, matrixToInvert);
   }

   /**
    * Computes the inverse of {@code matrix} and stores the result in {@code inverseToPack}.
    * <p>
    * The matrices can be the same object.
    * </p>
    *
    * @param matrix the matrix to compute the inverse of. Not modified.
    * @param inverseToPack the result to pack. Modified.
    * @return {@code true} if the inversion succeeds, {@code false} if the matrix is singular.
    */
   public static boolean invert(Matrix3DReadOnly matrix, Matrix3DBasics inverseToPack)
   {
      double det = matrix.determinant();

      if (Math.abs(det) >= EPS_INVERT)
      {
         det = 1.0 / det;
         double m00 = (matrix.getM11() * matrix.getM22() - matrix.getM21() * matrix.getM12()) * det;
         double m01 = -(matrix.getM01() * matrix.getM22() - matrix.getM21() * matrix.getM02()) * det;
         double m02 = (matrix.getM01() * matrix.getM12() - matrix.getM11() * matrix.getM02()) * det;
         double m10 = -(matrix.getM10() * matrix.getM22() - matrix.getM20() * matrix.getM12()) * det;
         double m11 = (matrix.getM00() * matrix.getM22() - matrix.getM20() * matrix.getM02()) * det;
         double m12 = -(matrix.getM00() * matrix.getM12() - matrix.getM10() * matrix.getM02()) * det;
         double m20 = (matrix.getM10() * matrix.getM21() - matrix.getM20() * matrix.getM11()) * det;
         double m21 = -(matrix.getM00() * matrix.getM21() - matrix.getM20() * matrix.getM01()) * det;
         double m22 = (matrix.getM00() * matrix.getM11() - matrix.getM10() * matrix.getM01()) * det;
         inverseToPack.set(m00, m01, m02, m10, m11, m12, m20, m21, m22);
         return true;
      }

      return false;
   }

   /**
    * Performs the multiplication: {@code m1} * {@code m2} and stores the result in
    * {@code matrixToPack}.
    * <p>
    * All the matrices can be the same object.
    * </p>
    *
    * @param m1 the first matrix. Not modified.
    * @param m2 the second matrix. Not modified.
    * @param matrixToPack the matrix in which the result is stored. Modified.
    */
   public static void multiply(Matrix3DReadOnly m1, Matrix3DReadOnly m2, Matrix3DBasics matrixToPack)
   {
      double m00 = m1.getM00() * m2.getM00() + m1.getM01() * m2.getM10() + m1.getM02() * m2.getM20();
      double m01 = m1.getM00() * m2.getM01() + m1.getM01() * m2.getM11() + m1.getM02() * m2.getM21();
      double m02 = m1.getM00() * m2.getM02() + m1.getM01() * m2.getM12() + m1.getM02() * m2.getM22();
      double m10 = m1.getM10() * m2.getM00() + m1.getM11() * m2.getM10() + m1.getM12() * m2.getM20();
      double m11 = m1.getM10() * m2.getM01() + m1.getM11() * m2.getM11() + m1.getM12() * m2.getM21();
      double m12 = m1.getM10() * m2.getM02() + m1.getM11() * m2.getM12() + m1.getM12() * m2.getM22();
      double m20 = m1.getM20() * m2.getM00() + m1.getM21() * m2.getM10() + m1.getM22() * m2.getM20();
      double m21 = m1.getM20() * m2.getM01() + m1.getM21() * m2.getM11() + m1.getM22() * m2.getM21();
      double m22 = m1.getM20() * m2.getM02() + m1.getM21() * m2.getM12() + m1.getM22() * m2.getM22();
      matrixToPack.set(m00, m01, m02, m10, m11, m12, m20, m21, m22);
   }

   /**
    * Performs the multiplication: {@code m1}<sup>T</sup> * {@code m2}<sup>T</sup> and stores the
    * result in {@code matrixToPack}.
    * <p>
    * All the matrices can be the same object.
    * <p>
    *
    * @param m1 the first matrix. Not modified.
    * @param m2 the second matrix. Not modified.
    * @param matrixToPack the matrix in which the result is stored. Modified.
    */
   public static void multiplyTransposeBoth(Matrix3DReadOnly m1, Matrix3DReadOnly m2, Matrix3DBasics matrixToPack)
   {
      double m00 = m1.getM00() * m2.getM00() + m1.getM10() * m2.getM01() + m1.getM20() * m2.getM02();
      double m01 = m1.getM00() * m2.getM10() + m1.getM10() * m2.getM11() + m1.getM20() * m2.getM12();
      double m02 = m1.getM00() * m2.getM20() + m1.getM10() * m2.getM21() + m1.getM20() * m2.getM22();
      double m10 = m1.getM01() * m2.getM00() + m1.getM11() * m2.getM01() + m1.getM21() * m2.getM02();
      double m11 = m1.getM01() * m2.getM10() + m1.getM11() * m2.getM11() + m1.getM21() * m2.getM12();
      double m12 = m1.getM01() * m2.getM20() + m1.getM11() * m2.getM21() + m1.getM21() * m2.getM22();
      double m20 = m1.getM02() * m2.getM00() + m1.getM12() * m2.getM01() + m1.getM22() * m2.getM02();
      double m21 = m1.getM02() * m2.getM10() + m1.getM12() * m2.getM11() + m1.getM22() * m2.getM12();
      double m22 = m1.getM02() * m2.getM20() + m1.getM12() * m2.getM21() + m1.getM22() * m2.getM22();
      matrixToPack.set(m00, m01, m02, m10, m11, m12, m20, m21, m22);
   }

   /**
    * Performs the multiplication: {@code m1}<sup>-1</sup> * {@code m2}<sup>-1</sup> and stores the
    * result in {@code matrixToPack}.
    * <p>
    * All the matrices can be the same object.
    * <p>
    *
    * @param m1 the first matrix. Not modified.
    * @param m2 the second matrix. Not modified.
    * @param matrixToPack the matrix in which the result is stored. Modified.
    * @throws SingularMatrixException if the matrix {@code m2} * {@code m1} is not invertible.
    */
   public static void multiplyInvertBoth(Matrix3DReadOnly m1, Matrix3DReadOnly m2, Matrix3DBasics matrixToPack)
   {
      multiply(m2, m1, matrixToPack);
      boolean success = invert(matrixToPack);
      if (!success)
         throw new SingularMatrixException(matrixToPack);
   }

   /**
    * Performs the multiplication: {@code m1}<sup>T</sup> * {@code m2} and stores the result in
    * {@code matrixToPack}.
    * <p>
    * All the matrices can be the same object.
    * <p>
    *
    * @param m1 the first matrix. Not modified.
    * @param m2 the second matrix. Not modified.
    * @param matrixToPack the matrix in which the result is stored. Modified.
    */
   public static void multiplyTransposeLeft(Matrix3DReadOnly m1, Matrix3DReadOnly m2, Matrix3DBasics matrixToPack)
   {
      double m00 = m1.getM00() * m2.getM00() + m1.getM10() * m2.getM10() + m1.getM20() * m2.getM20();
      double m01 = m1.getM00() * m2.getM01() + m1.getM10() * m2.getM11() + m1.getM20() * m2.getM21();
      double m02 = m1.getM00() * m2.getM02() + m1.getM10() * m2.getM12() + m1.getM20() * m2.getM22();
      double m10 = m1.getM01() * m2.getM00() + m1.getM11() * m2.getM10() + m1.getM21() * m2.getM20();
      double m11 = m1.getM01() * m2.getM01() + m1.getM11() * m2.getM11() + m1.getM21() * m2.getM21();
      double m12 = m1.getM01() * m2.getM02() + m1.getM11() * m2.getM12() + m1.getM21() * m2.getM22();
      double m20 = m1.getM02() * m2.getM00() + m1.getM12() * m2.getM10() + m1.getM22() * m2.getM20();
      double m21 = m1.getM02() * m2.getM01() + m1.getM12() * m2.getM11() + m1.getM22() * m2.getM21();
      double m22 = m1.getM02() * m2.getM02() + m1.getM12() * m2.getM12() + m1.getM22() * m2.getM22();
      matrixToPack.set(m00, m01, m02, m10, m11, m12, m20, m21, m22);
   }

   /**
    * Performs the multiplication: {@code m1}<sup>-1</sup> * {@code m2} and stores the result in
    * {@code matrixToPack}.
    * <p>
    * All the matrices can be the same object.
    * <p>
    *
    * @param m1 the first matrix. Not modified.
    * @param m2 the second matrix. Not modified.
    * @param matrixToPack the matrix in which the result is stored. Modified.
    * @throws SingularMatrixException if {@code m1} is not invertible.
    */
   public static void multiplyInvertLeft(Matrix3DReadOnly m1, Matrix3DReadOnly m2, Matrix3DBasics matrixToPack)
   {
      double det = m1.determinant();
      if (Math.abs(det) < EPS_INVERT)
         throw new SingularMatrixException(m1);

      det = 1.0 / det;
      double invM00 = (m1.getM11() * m1.getM22() - m1.getM21() * m1.getM12()) * det;
      double invM01 = -(m1.getM01() * m1.getM22() - m1.getM21() * m1.getM02()) * det;
      double invM02 = (m1.getM01() * m1.getM12() - m1.getM11() * m1.getM02()) * det;
      double invM10 = -(m1.getM10() * m1.getM22() - m1.getM20() * m1.getM12()) * det;
      double invM11 = (m1.getM00() * m1.getM22() - m1.getM20() * m1.getM02()) * det;
      double invM12 = -(m1.getM00() * m1.getM12() - m1.getM10() * m1.getM02()) * det;
      double invM20 = (m1.getM10() * m1.getM21() - m1.getM20() * m1.getM11()) * det;
      double invM21 = -(m1.getM00() * m1.getM21() - m1.getM20() * m1.getM01()) * det;
      double invM22 = (m1.getM00() * m1.getM11() - m1.getM10() * m1.getM01()) * det;

      double m00 = invM00 * m2.getM00() + invM01 * m2.getM10() + invM02 * m2.getM20();
      double m01 = invM00 * m2.getM01() + invM01 * m2.getM11() + invM02 * m2.getM21();
      double m02 = invM00 * m2.getM02() + invM01 * m2.getM12() + invM02 * m2.getM22();
      double m10 = invM10 * m2.getM00() + invM11 * m2.getM10() + invM12 * m2.getM20();
      double m11 = invM10 * m2.getM01() + invM11 * m2.getM11() + invM12 * m2.getM21();
      double m12 = invM10 * m2.getM02() + invM11 * m2.getM12() + invM12 * m2.getM22();
      double m20 = invM20 * m2.getM00() + invM21 * m2.getM10() + invM22 * m2.getM20();
      double m21 = invM20 * m2.getM01() + invM21 * m2.getM11() + invM22 * m2.getM21();
      double m22 = invM20 * m2.getM02() + invM21 * m2.getM12() + invM22 * m2.getM22();
      matrixToPack.set(m00, m01, m02, m10, m11, m12, m20, m21, m22);
   }

   /**
    * Performs the multiplication: {@code m1}<sup>-1</sup> * {@code m2} and stores the result in
    * {@code matrixToPack}.
    * <p>
    * {@code m2} and {@code matrixToPack} can be the same object. {@code m1} and {@code m2} can be the
    * same object.
    * <p>
    * <p>
    * This operation uses the property: <br>
    * R<sup>-1</sup> = R<sup>T</sup> </br>
    * of a rotation matrix preventing to actually compute the inverse of the rotation matrix.
    * </p>
    *
    * @param m1 the first matrix. Not modified.
    * @param m2 the second matrix. Not modified.
    * @param matrixToPack the matrix in which the result is stored. Modified.
    */
   public static void multiplyInvertLeft(RotationMatrixReadOnly m1, Matrix3DReadOnly m2, Matrix3DBasics matrixToPack)
   {
      multiplyTransposeLeft(m1, m2, matrixToPack);
   }

   /**
    * Performs the multiplication: {@code m1}<sup>-1</sup> * {@code m2} and stores the result in
    * {@code matrixToPack}.
    * <p>
    * {@code m2} and {@code matrixToPack} can be the same object. {@code m1} and {@code m2} can be the
    * same object.
    * <p>
    * <p>
    * This operation uses the property: <br>
    * (R * S)<sup>-1</sup> = S<sup>-1</sup> * R<sup>T</sup> </br>
    * of the rotation-scale matrix preventing to actually compute its inverse.
    * </p>
    *
    * @param m1 the first matrix. Not modified.
    * @param m2 the second matrix. Not modified.
    * @param matrixToPack the matrix in which the result is stored. Modified.
    */
   public static void multiplyInvertLeft(RotationScaleMatrixReadOnly m1, Matrix3DReadOnly m2, Matrix3DBasics matrixToPack)
   {
      multiplyInvertLeft(m1.getRotationMatrix(), m2, matrixToPack);
      matrixToPack.scaleRows(1.0 / m1.getScaleX(), 1.0 / m1.getScaleY(), 1.0 / m1.getScaleZ());
   }

   /**
    * Performs the multiplication: {@code m1} * {@code m2}<sup>T</sup> and stores the result in
    * {@code matrixToPack}.
    * <p>
    * All the matrices can be the same object.
    * <p>
    *
    * @param m1 the first matrix. Not modified.
    * @param m2 the second matrix. Not modified.
    * @param matrixToPack the matrix in which the result is stored. Modified.
    */
   public static void multiplyTransposeRight(Matrix3DReadOnly m1, Matrix3DReadOnly m2, Matrix3DBasics matrixToPack)
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
      matrixToPack.set(m00, m01, m02, m10, m11, m12, m20, m21, m22);
   }

   /**
    * Performs the multiplication: {@code m1} * {@code m2}<sup>-1</sup> and stores the result in
    * {@code matrixToPack}.
    * <p>
    * All the matrices can be the same object.
    * <p>
    *
    * @param m1 the first matrix. Not modified.
    * @param m2 the second matrix. Not modified.
    * @param matrixToPack the matrix in which the result is stored. Modified.
    * @throws SingularMatrixException if {@code m2} is not invertible.
    */
   public static void multiplyInvertRight(Matrix3DReadOnly m1, Matrix3DReadOnly m2, Matrix3DBasics matrixToPack)
   {
      double det = m2.determinant();
      if (Math.abs(det) < EPS_INVERT)
         throw new SingularMatrixException(m2);

      det = 1.0 / det;
      double invM00 = (m2.getM11() * m2.getM22() - m2.getM21() * m2.getM12()) * det;
      double invM01 = -(m2.getM01() * m2.getM22() - m2.getM21() * m2.getM02()) * det;
      double invM02 = (m2.getM01() * m2.getM12() - m2.getM11() * m2.getM02()) * det;
      double invM10 = -(m2.getM10() * m2.getM22() - m2.getM20() * m2.getM12()) * det;
      double invM11 = (m2.getM00() * m2.getM22() - m2.getM20() * m2.getM02()) * det;
      double invM12 = -(m2.getM00() * m2.getM12() - m2.getM10() * m2.getM02()) * det;
      double invM20 = (m2.getM10() * m2.getM21() - m2.getM20() * m2.getM11()) * det;
      double invM21 = -(m2.getM00() * m2.getM21() - m2.getM20() * m2.getM01()) * det;
      double invM22 = (m2.getM00() * m2.getM11() - m2.getM10() * m2.getM01()) * det;

      double m00 = m1.getM00() * invM00 + m1.getM01() * invM10 + m1.getM02() * invM20;
      double m01 = m1.getM00() * invM01 + m1.getM01() * invM11 + m1.getM02() * invM21;
      double m02 = m1.getM00() * invM02 + m1.getM01() * invM12 + m1.getM02() * invM22;
      double m10 = m1.getM10() * invM00 + m1.getM11() * invM10 + m1.getM12() * invM20;
      double m11 = m1.getM10() * invM01 + m1.getM11() * invM11 + m1.getM12() * invM21;
      double m12 = m1.getM10() * invM02 + m1.getM11() * invM12 + m1.getM12() * invM22;
      double m20 = m1.getM20() * invM00 + m1.getM21() * invM10 + m1.getM22() * invM20;
      double m21 = m1.getM20() * invM01 + m1.getM21() * invM11 + m1.getM22() * invM21;
      double m22 = m1.getM20() * invM02 + m1.getM21() * invM12 + m1.getM22() * invM22;
      matrixToPack.set(m00, m01, m02, m10, m11, m12, m20, m21, m22);
   }

   /**
    * Performs the multiplication: {@code m1} * {@code m2}<sup>-1</sup> and stores the result in
    * {@code matrixToPack}.
    * <p>
    * {@code m1} and {@code matrixToPack} can be the same object. {@code m1} and {@code m2} can be the
    * same object.
    * <p>
    * <p>
    * This operation uses the property: <br>
    * R<sup>-1</sup> = R<sup>T</sup> </br>
    * of a rotation matrix preventing to actually compute the inverse of the rotation matrix.
    * </p>
    *
    * @param m1 the first matrix. Not modified.
    * @param m2 the second matrix. Not modified.
    * @param matrixToPack the matrix in which the result is stored. Modified.
    */
   public static void multiplyInvertRight(Matrix3DReadOnly m1, RotationMatrixReadOnly m2, Matrix3DBasics matrixToPack)
   {
      multiplyTransposeRight(m1, m2, matrixToPack);
   }

   /**
    * Performs the multiplication: {@code m1} * {@code m2}<sup>-1</sup> and stores the result in
    * {@code matrixToPack}.
    * <p>
    * {@code m1} and {@code matrixToPack} can be the same object. {@code m1} and {@code m2} can be the
    * same object.
    * <p>
    * <p>
    * This operation uses the property: <br>
    * (R * S)<sup>-1</sup> = S<sup>-1</sup> * R<sup>T</sup> </br>
    * of the rotation-scale matrix preventing to actually compute its inverse.
    * </p>
    *
    * @param m1 the first matrix. Not modified.
    * @param m2 the second matrix. Not modified.
    * @param matrixToPack the matrix in which the result is stored. Modified.
    */
   public static void multiplyInvertRight(Matrix3DReadOnly m1, RotationScaleMatrixReadOnly m2, Matrix3DBasics matrixToPack)
   {
      matrixToPack.set(m1);
      matrixToPack.scaleColumns(1.0 / m2.getScaleX(), 1.0 / m2.getScaleY(), 1.0 / m2.getScaleZ());
      multiplyTransposeRight(matrixToPack, m2.getRotationMatrix(), matrixToPack);
   }

   /**
    * Orthonormalization of the given matrix using the
    * <a href="https://en.wikipedia.org/wiki/Gram%E2%80%93Schmidt_process"> Gram-Schmidt method</a>.
    *
    * @param matrixToNormalize the matrix to normalize. Modified.
    */
   public static void normalize(CommonMatrix3DBasics matrixToNormalize)
   {
      double m00 = matrixToNormalize.getM00();
      double m01 = matrixToNormalize.getM01();
      double m02 = matrixToNormalize.getM02();
      double m10 = matrixToNormalize.getM10();
      double m11 = matrixToNormalize.getM11();
      double m12 = matrixToNormalize.getM12();
      double m20 = matrixToNormalize.getM20();
      double m21 = matrixToNormalize.getM21();
      double m22 = matrixToNormalize.getM22();

      double xdoty = m00 * m01 + m10 * m11 + m20 * m21;
      double xdotx = m00 * m00 + m10 * m10 + m20 * m20;
      double tmp = xdoty / xdotx;

      m01 -= tmp * m00;
      m11 -= tmp * m10;
      m21 -= tmp * m20;

      double zdoty = m02 * m01 + m12 * m11 + m22 * m21;
      double zdotx = m02 * m00 + m12 * m10 + m22 * m20;
      double ydoty = m01 * m01 + m11 * m11 + m21 * m21;

      tmp = zdotx / xdotx;
      double tmp1 = zdoty / ydoty;

      m02 -= tmp * m00 + tmp1 * m01;
      m12 -= tmp * m10 + tmp1 * m11;
      m22 -= tmp * m20 + tmp1 * m21;

      // Compute orthogonalized vector magnitudes and normalize
      double invMagX = 1.0 / EuclidCoreTools.norm(m00, m10, m20);
      double invMagY = 1.0 / EuclidCoreTools.norm(m01, m11, m21);
      double invMagZ = 1.0 / EuclidCoreTools.norm(m02, m12, m22);

      m00 *= invMagX;
      m01 *= invMagY;
      m02 *= invMagZ;
      m10 *= invMagX;
      m11 *= invMagY;
      m12 *= invMagZ;
      m20 *= invMagX;
      m21 *= invMagY;
      m22 *= invMagZ;
      matrixToNormalize.set(m00, m01, m02, m10, m11, m12, m20, m21, m22);
   }

   /**
    * Performs a transformation of {@code tupleOriginal} using the given matrix and stores the result
    * in {@code tupleTransformed}:
    * <p>
    * {@code tupleTransformed} = {@code matrix} * {@code tupleOriginal}.
    * </p>
    * <p>
    * Both tuples can be the same instance to perform in-place transformation.
    * </p>
    *
    * @param matrix the matrix used to transform {@code tupleOriginal}. Not modified.
    * @param tupleOriginal the original tuple to use for the transformation. Not modified.
    * @param tupleTransformed the tuple used to store the result of the transformation. Modified.
    */
   public static void transform(Matrix3DReadOnly matrix, Tuple3DReadOnly tupleOriginal, Tuple3DBasics tupleTransformed)
   {
      double x = matrix.getM00() * tupleOriginal.getX() + matrix.getM01() * tupleOriginal.getY() + matrix.getM02() * tupleOriginal.getZ();
      double y = matrix.getM10() * tupleOriginal.getX() + matrix.getM11() * tupleOriginal.getY() + matrix.getM12() * tupleOriginal.getZ();
      double z = matrix.getM20() * tupleOriginal.getX() + matrix.getM21() * tupleOriginal.getY() + matrix.getM22() * tupleOriginal.getZ();
      tupleTransformed.set(x, y, z);
   }

   /**
    * Performs a transformation of {@code tupleOriginal} using the given matrix and add the result to
    * {@code tupleTransformed}:
    * <p>
    * {@code tupleTransformed} = {@code tupleTransformed} + {@code matrix} * {@code tupleOriginal}.
    * </p>
    * <p>
    * Both tuples can be the same instance to perform in-place transformation.
    * </p>
    *
    * @param matrix the matrix used to transform {@code tupleOriginal}. Not modified.
    * @param tupleOriginal the original tuple to use for the transformation. Not modified.
    * @param tupleTransformed the tuple to which the result of the transformation is added to.
    *           Modified.
    */
   public static void addTransform(Matrix3DReadOnly matrix, Tuple3DReadOnly tupleOriginal, Tuple3DBasics tupleTransformed)
   {
      double x = matrix.getM00() * tupleOriginal.getX() + matrix.getM01() * tupleOriginal.getY() + matrix.getM02() * tupleOriginal.getZ();
      double y = matrix.getM10() * tupleOriginal.getX() + matrix.getM11() * tupleOriginal.getY() + matrix.getM12() * tupleOriginal.getZ();
      double z = matrix.getM20() * tupleOriginal.getX() + matrix.getM21() * tupleOriginal.getY() + matrix.getM22() * tupleOriginal.getZ();
      tupleTransformed.add(x, y, z);
   }

   /**
    * Performs a transformation of {@code tupleOriginal} using the given matrix and subtract the result
    * to {@code tupleTransformed}:
    * <p>
    * {@code tupleTransformed} = {@code tupleTransformed} - {@code matrix} * {@code tupleOriginal}.
    * </p>
    * <p>
    * Both tuples can be the same instance to perform in-place transformation.
    * </p>
    *
    * @param matrix the matrix used to transform {@code tupleOriginal}. Not modified.
    * @param tupleOriginal the original tuple to use for the transformation. Not modified.
    * @param tupleTransformed the tuple to which the result of the transformation is added to.
    *           Modified.
    */
   public static void subTransform(Matrix3DReadOnly matrix, Tuple3DReadOnly tupleOriginal, Tuple3DBasics tupleTransformed)
   {
      double x = matrix.getM00() * tupleOriginal.getX() + matrix.getM01() * tupleOriginal.getY() + matrix.getM02() * tupleOriginal.getZ();
      double y = matrix.getM10() * tupleOriginal.getX() + matrix.getM11() * tupleOriginal.getY() + matrix.getM12() * tupleOriginal.getZ();
      double z = matrix.getM20() * tupleOriginal.getX() + matrix.getM21() * tupleOriginal.getY() + matrix.getM22() * tupleOriginal.getZ();
      tupleTransformed.sub(x, y, z);
   }

   /**
    * Performs a transformation of {@code tupleOriginal} using the given matrix and stores the result
    * in {@code tupleTransformed}:
    * <p>
    * {@code tupleTransformed} = {@code matrix} * {@code tupleOriginal}.
    * </p>
    * <p>
    * Before the transformation is performed, if {@code checkIfTransformInXYPlane} equals true, this
    * verify that the matrix is a 2D transformation matrix using
    * {@link Matrix3DReadOnly#checkIfMatrix2D()}.
    * </p>
    * <p>
    * Both tuples can be the same instance to perform in-place transformation.
    * </p>
    *
    * @param matrix the matrix used to transform {@code tupleOriginal}. Not modified.
    * @param tupleOriginal the original tuple to use for the transformation. Not modified.
    * @param tupleTransformed the tuple used to stored the result of the transformation. Modified.
    * @param checkIfTransformInXYPlane whether {@link Matrix3DReadOnly#checkIfMatrix2D()} needs to be
    *           called on the matrix.
    * @throws NotAMatrix2DException if the matrix is not a 2D matrix and
    *            {@code checkIfTransformInXYPlane} is {@code true}.
    */
   public static void transform(Matrix3DReadOnly matrix, Tuple2DReadOnly tupleOriginal, Tuple2DBasics tupleTransformed, boolean checkIfTransformInXYPlane)
   {
      if (checkIfTransformInXYPlane)
         matrix.checkIfMatrix2D();
      double x = matrix.getM00() * tupleOriginal.getX() + matrix.getM01() * tupleOriginal.getY();
      double y = matrix.getM10() * tupleOriginal.getX() + matrix.getM11() * tupleOriginal.getY();
      tupleTransformed.set(x, y);
   }

   /**
    * Performs a transformation on the vector part of {@code vectorOriginal} using the given matrix and
    * stores the result in {@code vectorTransformed}:
    * <p>
    * {@code vectorTransformed.s} = {@code vectorOriginal.s}. {@code vectorTransformed.xyz} =
    * {@code matrix} * {@code vectorOriginal.xyz}.
    * </p>
    * <p>
    * Both vectors can be the same instance to perform in-place transformation.
    * </p>
    *
    * @param matrix the matrix used to transform {@code tupleOriginal}. Not modified.
    * @param vectorOriginal the original vector to use for the transformation. Not modified.
    * @param vectorTransformed the vector used to stored the result of the transformation. Modified.
    * @throws NotAMatrix2DException if the matrix is not a 2D matrix and
    *            {@code checkIfTransformInXYPlane} is {@code true}.
    */
   public static void transform(Matrix3DReadOnly matrix, Vector4DReadOnly vectorOriginal, Vector4DBasics vectorTransformed)
   {
      double x = matrix.getM00() * vectorOriginal.getX() + matrix.getM01() * vectorOriginal.getY() + matrix.getM02() * vectorOriginal.getZ();
      double y = matrix.getM10() * vectorOriginal.getX() + matrix.getM11() * vectorOriginal.getY() + matrix.getM12() * vectorOriginal.getZ();
      double z = matrix.getM20() * vectorOriginal.getX() + matrix.getM21() * vectorOriginal.getY() + matrix.getM22() * vectorOriginal.getZ();
      vectorTransformed.set(x, y, z, vectorOriginal.getS());
   }

   /**
    * Performs a transformation of {@code matrixOriginal} using {@code matrix} and stores the result in
    * {@code matrixTransformed}:
    * <p>
    * {@code matrixTransformed} = {@code matrix} * {@code matrixOriginal} *
    * {@code matrix}<sup>-1</sup>.
    * </p>
    * <p>
    * WARNING: <b> This is different from concatenating orientations.</b>
    * </p>
    * <p>
    * {@code matrixOriginal} and {@code matrixTransformed} can be the same instance to perform in-place
    * transformation.
    * </p>
    *
    * @param matrix the matrix used to transform {@code matrixOriginal}. Not modified.
    * @param matrixOriginal the original matrix to use for the transformation. Not modified.
    * @param matrixTransformed the matrix used to stored the result of the transformation. Modified.
    */
   public static void transform(Matrix3DReadOnly matrix, Matrix3DReadOnly matrixOriginal, Matrix3DBasics matrixTransformed)
   {
      multiply(matrix, matrixOriginal, matrixTransformed);
      multiplyInvertRight(matrixTransformed, matrix, matrixTransformed);
   }

   /**
    * Undoes the transformation of {@code tupleOriginal} using the given matrix and stores the result
    * in {@code tupleTransformed}:
    * <p>
    * {@code tupleTransformed} = {@code matrix}<sup>-1</sup> * {@code tupleOriginal}.
    * </p>
    * <p>
    * Both tuples can be the same instance to perform in-place transformation.
    * </p>
    *
    * @param matrix the matrix used to transform {@code tupleOriginal}. Not modified.
    * @param tupleOriginal the original tuple to use for the transformation. Not modified.
    * @param tupleTransformed the tuple used to store the result of the transformation. Modified.
    * @throws SingularMatrixException if {@code matrix} is not invertible.
    */
   public static void inverseTransform(Matrix3DReadOnly matrix, Tuple3DReadOnly tupleOriginal, Tuple3DBasics tupleTransformed)
   {
      double det = matrix.determinant();
      if (Math.abs(det) < EPS_INVERT)
         throw new SingularMatrixException(matrix);

      det = 1.0 / det;
      double invM00 = (matrix.getM11() * matrix.getM22() - matrix.getM21() * matrix.getM12()) * det;
      double invM01 = -(matrix.getM01() * matrix.getM22() - matrix.getM21() * matrix.getM02()) * det;
      double invM02 = (matrix.getM01() * matrix.getM12() - matrix.getM11() * matrix.getM02()) * det;
      double invM10 = -(matrix.getM10() * matrix.getM22() - matrix.getM20() * matrix.getM12()) * det;
      double invM11 = (matrix.getM00() * matrix.getM22() - matrix.getM20() * matrix.getM02()) * det;
      double invM12 = -(matrix.getM00() * matrix.getM12() - matrix.getM10() * matrix.getM02()) * det;
      double invM20 = (matrix.getM10() * matrix.getM21() - matrix.getM20() * matrix.getM11()) * det;
      double invM21 = -(matrix.getM00() * matrix.getM21() - matrix.getM20() * matrix.getM01()) * det;
      double invM22 = (matrix.getM00() * matrix.getM11() - matrix.getM10() * matrix.getM01()) * det;

      double x = invM00 * tupleOriginal.getX() + invM01 * tupleOriginal.getY() + invM02 * tupleOriginal.getZ();
      double y = invM10 * tupleOriginal.getX() + invM11 * tupleOriginal.getY() + invM12 * tupleOriginal.getZ();
      double z = invM20 * tupleOriginal.getX() + invM21 * tupleOriginal.getY() + invM22 * tupleOriginal.getZ();
      tupleTransformed.set(x, y, z);
   }

   /**
    * Undoes the transformation of {@code tupleOriginal} using the given matrix and stores the result
    * in {@code tupleTransformed}:
    * <p>
    * {@code tupleTransformed} = {@code matrix}<sup>-1</sup> * {@code tupleOriginal}.
    * </p>
    * <p>
    * Before the transformation is performed, if {@code checkIfTransformInXYPlane} equals true, this
    * verify that the matrix is a 2D transformation matrix using
    * {@link Matrix3DReadOnly#checkIfMatrix2D()}.
    * </p>
    * <p>
    * Both tuples can be the same instance to perform in-place transformation.
    * </p>
    *
    * @param matrix the matrix used to transform {@code tupleOriginal}. Not modified.
    * @param tupleOriginal the original tuple to use for the transformation. Not modified.
    * @param tupleTransformed the tuple used to stored the result of the transformation. Modified.
    * @param checkIfTransformInXYPlane whether {@link Matrix3DReadOnly#checkIfMatrix2D()} needs to be
    *           called on the matrix.
    * @throws NotAMatrix2DException if the matrix is not a 2D matrix and
    *            {@code checkIfTransformInXYPlane} is {@code true}.
    * @throws SingularMatrixException if {@code matrix} is not invertible.
    */
   public static void inverseTransform(Matrix3DReadOnly matrix, Tuple2DReadOnly tupleOriginal, Tuple2DBasics tupleTransformed,
                                       boolean checkIfTransformInXYPlane)
   {
      boolean isMatrix2D = matrix.isMatrix2D();

      if (checkIfTransformInXYPlane || isMatrix2D)
      {
         if (!isMatrix2D)
            throw new NotAMatrix2DException(matrix);

         // Compute only the determinant of the sub matrix that transforms in the XY plane.
         double det = matrix.getM00() * matrix.getM11() - matrix.getM10() * matrix.getM01(); //determinant(matrix);
         if (Math.abs(det) < EPS_INVERT)
            throw new SingularMatrixException(matrix);

         det = 1.0 / det;
         double invM00 = matrix.getM11() * det;
         double invM01 = -matrix.getM01() * det;
         double invM10 = -matrix.getM10() * det;
         double invM11 = matrix.getM00() * det;

         double x = invM00 * tupleOriginal.getX() + invM01 * tupleOriginal.getY();
         double y = invM10 * tupleOriginal.getX() + invM11 * tupleOriginal.getY();
         tupleTransformed.set(x, y);
      }
      else
      {
         double det = matrix.determinant();
         if (Math.abs(det) < EPS_INVERT)
            throw new SingularMatrixException(matrix);

         det = 1.0 / det;
         double invM00 = (matrix.getM11() * matrix.getM22() - matrix.getM21() * matrix.getM12()) * det;
         double invM01 = -(matrix.getM01() * matrix.getM22() - matrix.getM21() * matrix.getM02()) * det;
         double invM10 = -(matrix.getM10() * matrix.getM22() - matrix.getM20() * matrix.getM12()) * det;
         double invM11 = (matrix.getM00() * matrix.getM22() - matrix.getM20() * matrix.getM02()) * det;

         double x = invM00 * tupleOriginal.getX() + invM01 * tupleOriginal.getY();
         double y = invM10 * tupleOriginal.getX() + invM11 * tupleOriginal.getY();
         tupleTransformed.set(x, y);
      }
   }

   /**
    * Undoes the transformation on the vector part of {@code vectorOriginal} using the given matrix and
    * stores the result in {@code vectorTransformed}:
    * <p>
    * {@code vectorTransformed.s} = {@code vectorOriginal.s}. {@code vectorTransformed.xyz} =
    * {@code matrix}<sup>-1</sup> * {@code vectorOriginal.xyz}.
    * </p>
    * <p>
    * Both vectors can be the same instance to perform in-place transformation.
    * </p>
    *
    * @param matrix the matrix used to transform {@code tupleOriginal}. Not modified.
    * @param vectorOriginal the original vector to use for the transformation. Not modified.
    * @param vectorTransformed the vector used to stored the result of the transformation. Modified.
    * @throws NotAMatrix2DException if the matrix is not a 2D matrix and
    *            {@code checkIfTransformInXYPlane} is {@code true}.
    * @throws SingularMatrixException if {@code matrix} is not invertible.
    */
   public static void inverseTransform(Matrix3DReadOnly matrix, Vector4DReadOnly vectorOriginal, Vector4DBasics vectorTransformed)
   {
      double det = matrix.determinant();
      if (Math.abs(det) < EPS_INVERT)
         throw new SingularMatrixException(matrix);

      det = 1.0 / det;
      double invM00 = (matrix.getM11() * matrix.getM22() - matrix.getM21() * matrix.getM12()) * det;
      double invM01 = -(matrix.getM01() * matrix.getM22() - matrix.getM21() * matrix.getM02()) * det;
      double invM02 = (matrix.getM01() * matrix.getM12() - matrix.getM11() * matrix.getM02()) * det;
      double invM10 = -(matrix.getM10() * matrix.getM22() - matrix.getM20() * matrix.getM12()) * det;
      double invM11 = (matrix.getM00() * matrix.getM22() - matrix.getM20() * matrix.getM02()) * det;
      double invM12 = -(matrix.getM00() * matrix.getM12() - matrix.getM10() * matrix.getM02()) * det;
      double invM20 = (matrix.getM10() * matrix.getM21() - matrix.getM20() * matrix.getM11()) * det;
      double invM21 = -(matrix.getM00() * matrix.getM21() - matrix.getM20() * matrix.getM01()) * det;
      double invM22 = (matrix.getM00() * matrix.getM11() - matrix.getM10() * matrix.getM01()) * det;

      double x = invM00 * vectorOriginal.getX() + invM01 * vectorOriginal.getY() + invM02 * vectorOriginal.getZ();
      double y = invM10 * vectorOriginal.getX() + invM11 * vectorOriginal.getY() + invM12 * vectorOriginal.getZ();
      double z = invM20 * vectorOriginal.getX() + invM21 * vectorOriginal.getY() + invM22 * vectorOriginal.getZ();
      vectorTransformed.set(x, y, z, vectorOriginal.getS());
   }

   /**
    * Undoes the transformation on {@code matrixOriginal} using {@code matrix} and stores the result in
    * {@code matrixTransformed}:
    * <p>
    * {@code matrixTransformed} = {@code matrix}<sup>-1</sup> * {@code matrixOriginal} *
    * {@code matrix}.
    * </p>
    * <p>
    * WARNING: <b> This is different from concatenating orientations.</b>
    * </p>
    * <p>
    * {@code matrixOriginal} and {@code matrixTransformed} can be the same instance to perform in-place
    * transformation.
    * </p>
    *
    * @param matrix the matrix used to transform {@code matrixOriginal}. Not modified.
    * @param matrixOriginal the original matrix to use for the transformation. Not modified.
    * @param matrixTransformed the matrix used to stored the result of the transformation. Modified.
    */
   public static void inverseTransform(Matrix3DReadOnly matrix, Matrix3DReadOnly matrixOriginal, Matrix3DBasics matrixTransformed)
   {
      multiplyInvertLeft(matrix, matrixOriginal, matrixTransformed);
      multiply(matrixTransformed, matrix, matrixTransformed);
   }

   /**
    * Create an {@linkplain ArrayIndexOutOfBoundsException} for a bad column index.
    *
    * @param maxColumnIndex maximum column index allowed.
    * @param column the bad column index.
    * @return the exception
    */
   public static ArrayIndexOutOfBoundsException columnOutOfBoundsException(int maxColumnIndex, int column)
   {
      return new ArrayIndexOutOfBoundsException("column should be in [0, " + maxColumnIndex + "], but is: " + column);
   }

   /**
    * Create an {@linkplain ArrayIndexOutOfBoundsException} for a bad row index.
    *
    * @param maxRowIndex the maximum row index allowed.
    * @param row the bad row index.
    * @return the exception
    */
   public static ArrayIndexOutOfBoundsException rowOutOfBoundsException(int maxRowIndex, int row)
   {
      return new ArrayIndexOutOfBoundsException("row should be in [0, " + maxRowIndex + "], but is: " + row);
   }
}
