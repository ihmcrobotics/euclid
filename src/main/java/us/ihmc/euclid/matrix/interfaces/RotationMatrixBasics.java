package us.ihmc.euclid.matrix.interfaces;

import us.ihmc.euclid.exceptions.NotARotationMatrixException;
import us.ihmc.euclid.interfaces.Transformable;
import us.ihmc.euclid.orientation.interfaces.Orientation3DBasics;
import us.ihmc.euclid.orientation.interfaces.Orientation3DReadOnly;
import us.ihmc.euclid.rotationConversion.RotationMatrixConversion;
import us.ihmc.euclid.tools.RotationMatrixTools;
import us.ihmc.euclid.transform.interfaces.Transform;

/**
 * Write and read interface used for 3-by-3 rotation matrices.
 * <p>
 * A rotation matrix is used to represent a 3D orientation through its 9 coefficients. A rotation
 * matrix has to comply to several constraints:
 * <ul>
 * <li>each column of the matrix represents a unitary vector,
 * <li>each row of the matrix represents a unitary vector,
 * <li>every pair of columns of the matrix represents two orthogonal vectors,
 * <li>every pair of rows of the matrix represents two orthogonal vectors,
 * <li>the matrix determinant is equal to {@code 1}.
 * </ul>
 * A rotation matrix has the nice property <i>R<sup>T</sup> = R<sup>-1</sup></i>.
 * </p>
 *
 * @author Sylvain Bertrand
 */
public interface RotationMatrixBasics extends CommonMatrix3DBasics, RotationMatrixReadOnly, Orientation3DBasics, Transformable
{
   /** {@inheritDoc} */
   @Override
   default boolean containsNaN()
   {
      return RotationMatrixReadOnly.super.containsNaN();
   }

   /**
    * Sets this rotation matrix to identity representing a 'zero' rotation.
    */
   @Override
   default void setToZero()
   {
      setIdentity();
   }

   /**
    * Transposes this matrix: m = m<sup>T</sup>.
    */
   default void transpose()
   {
      set(getM00(), getM10(), getM20(), getM01(), getM11(), getM21(), getM02(), getM12(), getM22());
   }

   /**
    * Inverts this rotation matrix.
    * <p>
    * This operation uses the property: <br>
    * R<sup>-1</sup> = R<sup>T</sup> </br>
    * of a rotation matrix preventing to actually compute the inverse of the matrix.
    * </p>
    * <p>
    * This is equivalent to {@code this.transpose()}.
    * </p>
    */
   @Override
   default void invert()
   {
      transpose();
   }

   /** {@inheritDoc} */
   @Override
   default void normalize()
   {
      CommonMatrix3DBasics.super.normalize();
   }

   /**
    * Sets the 9 coefficients of this rotation matrix without performing any checks on the data
    * provided.
    * <p>
    * This method is meant for internal usage. Prefer using
    * {@link #set(double, double, double, double, double, double, double, double, double)} or
    * {@link #setAndNormalize(double, double, double, double, double, double, double, double, double)}.
    * </p>
    *
    * @param m00 the new 1st row 1st column coefficient for this matrix.
    * @param m01 the new 1st row 2nd column coefficient for this matrix.
    * @param m02 the new 1st row 3rd column coefficient for this matrix.
    * @param m10 the new 2nd row 1st column coefficient for this matrix.
    * @param m11 the new 2nd row 2nd column coefficient for this matrix.
    * @param m12 the new 2nd row 3rd column coefficient for this matrix.
    * @param m20 the new 3rd row 1st column coefficient for this matrix.
    * @param m21 the new 3rd row 2nd column coefficient for this matrix.
    * @param m22 the new 3rd row 3rd column coefficient for this matrix.
    */
   public void setUnsafe(double m00, double m01, double m02, double m10, double m11, double m12, double m20, double m21, double m22);

   /**
    * {@inheritDoc}
    *
    * @throws NotARotationMatrixException if the resulting matrix is not a rotation matrix.
    */
   @Override
   default void set(double m00, double m01, double m02, double m10, double m11, double m12, double m20, double m21, double m22)
   {
      setUnsafe(m00, m01, m02, m10, m11, m12, m20, m21, m22);
      if (!isIdentity())
         checkIfRotationMatrix();
   }

   /**
    * Sets the 9 coefficients of this rotation matrix and then normalizes {@code this}.
    *
    * @param m00 the new 1st row 1st column coefficient for this matrix.
    * @param m01 the new 1st row 2nd column coefficient for this matrix.
    * @param m02 the new 1st row 3rd column coefficient for this matrix.
    * @param m10 the new 2nd row 1st column coefficient for this matrix.
    * @param m11 the new 2nd row 2nd column coefficient for this matrix.
    * @param m12 the new 2nd row 3rd column coefficient for this matrix.
    * @param m20 the new 3rd row 1st column coefficient for this matrix.
    * @param m21 the new 3rd row 2nd column coefficient for this matrix.
    * @param m22 the new 3rd row 3rd column coefficient for this matrix.
    * @throws NotARotationMatrixException if the normalization failed.
    */
   default void setAndNormalize(double m00, double m01, double m02, double m10, double m11, double m12, double m20, double m21, double m22)
   {
      setUnsafe(m00, m01, m02, m10, m11, m12, m20, m21, m22);
      normalize();
   }

   /**
    * Sets this rotation matrix to equal the 3D matrix {@code matrix} and then normalizes {@code this}.
    *
    * @param matrix the matrix to copy the values from. Not modified.
    * @throws NotARotationMatrixException if the normalization failed.
    */
   default void setAndNormalize(Matrix3DReadOnly matrix)
   {
      setAndNormalize(matrix.getM00(),
                      matrix.getM01(),
                      matrix.getM02(),
                      matrix.getM10(),
                      matrix.getM11(),
                      matrix.getM12(),
                      matrix.getM20(),
                      matrix.getM21(),
                      matrix.getM22());
   }

   /**
    * Sets this rotation matrix to equal the given {@code other} and then normalizes {@code this}.
    *
    * @param other the rotation matrix to copy the values from. Not modified.
    * @throws NotARotationMatrixException if the normalization failed.
    */
   default void setAndNormalize(RotationMatrixReadOnly other)
   {
      set(other);
      normalize();
   }

   /**
    * {@inheritDoc}
    * <p>
    * If the argument implements {@link RotationMatrixReadOnly}, a redirection
    * {@link #set(RotationMatrixReadOnly)} is done.
    * </p>
    */
   @Override
   default void set(Matrix3DReadOnly other)
   {
      if (other instanceof RotationMatrixReadOnly)
         set((RotationMatrixReadOnly) other);
      else
         CommonMatrix3DBasics.super.set(other);
   }

   /**
    * {@inheritDoc}
    * <p>
    * If the argument implements {@link RotationMatrixReadOnly}, a redirection
    * {@link #set(RotationMatrixReadOnly)} is done.
    * </p>
    */
   @Override
   default void set(Orientation3DReadOnly orientation3DReadOnly)
   {
      if (orientation3DReadOnly instanceof RotationMatrixReadOnly)
         set((RotationMatrixReadOnly) orientation3DReadOnly);
      else
         orientation3DReadOnly.get(this);
   }

   /**
    * Sets this rotation matrix to {@code other} and copies the dirty and identity flags for the other
    * matrix.
    * 
    * @param other the other rotation matrix to copy.
    */
   void set(RotationMatrixReadOnly other);

   /**
    * Sets this rotation matrix to the invert of the given {@code matrix}.
    * <p>
    * This operation uses the property: <br>
    * R<sup>-1</sup> = R<sup>T</sup> </br>
    * of a rotation matrix preventing to actually compute the inverse of the matrix.
    * </p>
    *
    * @param matrix the matrix to copy the values from. Not modified.
    * @throws NotARotationMatrixException if {@code matrix} is not a rotation matrix.
    */
   default void setAndInvert(Matrix3DReadOnly matrix)
   {
      setAndTranspose(matrix);
   }

   /**
    * Sets this rotation matrix to the invert of the given one {@code other}.
    * <p>
    * This operation uses the property: <br>
    * R<sup>-1</sup> = R<sup>T</sup> </br>
    * of a rotation matrix preventing to actually compute the inverse of the matrix.
    * </p>
    *
    * @param other the matrix to copy the values from. Not modified.
    */
   default void setAndInvert(RotationMatrixReadOnly other)
   {
      setAndTranspose(other);
   }

   /**
    * Sets this matrix to equal the other matrix and then transposes this.
    * <p>
    * this = other<sup>T</sup>
    * </p>
    *
    * @param other the other matrix used to update this matrix. Not modified.
    */
   default void setAndTranspose(Matrix3DReadOnly other)
   {
      set(other);
      transpose();
   }

   @Override
   default void setAxisAngle(double x, double y, double z, double angle)
   {
      RotationMatrixConversion.convertAxisAngleToMatrix(x, y, z, angle, this);
   }

   @Override
   default void setRotationVector(double x, double y, double z)
   {
      RotationMatrixConversion.convertRotationVectorToMatrix(x, y, z, this);
   }

   @Override
   default void setQuaternion(double x, double y, double z, double s)
   {
      RotationMatrixConversion.convertQuaternionToMatrix(x, y, z, s, this);
   }

   @Override
   default void setYawPitchRoll(double yaw, double pitch, double roll)
   {
      RotationMatrixConversion.convertYawPitchRollToMatrix(yaw, pitch, roll, this);
   }

   @Override
   default void setRotationMatrix(double m00, double m01, double m02, double m10, double m11, double m12, double m20, double m21, double m22)
   {
      setUnsafe(m00, m01, m02, m10, m11, m12, m20, m21, m22);
   }

   /**
    * Sets this rotation matrix to represent a counter clockwise rotation around the z-axis of an angle
    * {@code yaw}.
    *
    * <pre>
    *        / cos(yaw) -sin(yaw) 0 \
    * this = | sin(yaw)  cos(yaw) 0 |
    *        \    0         0     1 /
    * </pre>
    *
    * @param yaw the angle to rotate about the z-axis.
    */
   @Override
   default void setToYawOrientation(double yaw)
   {
      RotationMatrixConversion.computeYawMatrix(yaw, this);
   }

   /**
    * Sets this rotation matrix to represent a counter clockwise rotation around the y-axis of an angle
    * {@code pitch}.
    *
    * <pre>
    *        /  cos(pitch) 0 sin(pitch) \
    * this = |      0      1     0      |
    *        \ -sin(pitch) 0 cos(pitch) /
    * </pre>
    *
    * @param pitch the angle to rotate about the y-axis.
    */
   @Override
   default void setToPitchOrientation(double pitch)
   {
      RotationMatrixConversion.computePitchMatrix(pitch, this);
   }

   /**
    * Sets this rotation matrix to represent a counter clockwise rotation around the x-axis of an angle
    * {@code roll}.
    *
    * <pre>
    *        / 1     0          0     \
    * this = | 0 cos(roll) -sin(roll) |
    *        \ 0 sin(roll)  cos(roll) /
    * </pre>
    *
    * @param roll the angle to rotate about the x-axis.
    */
   @Override
   default void setToRollOrientation(double roll)
   {
      RotationMatrixConversion.computeRollMatrix(roll, this);
   }

   /**
    * Performs a matrix multiplication on this.
    * <p>
    * this = this * other
    * </p>
    *
    * @param other the other matrix to multiply this by. Not modified.
    */
   default void multiply(RotationMatrixReadOnly other)
   {
      RotationMatrixTools.multiply(this, other, this);
   }

   @Override
   default void append(Orientation3DReadOnly orientation)
   {
      RotationMatrixTools.multiply(this, false, orientation, false, this);
   }

   /**
    * Performs a matrix multiplication on this.
    * <p>
    * this = this<sup>T</sup> * other
    * </p>
    *
    * @param other the other matrix to multiply this by. Not modified.
    */
   default void multiplyTransposeThis(RotationMatrixReadOnly other)
   {
      RotationMatrixTools.multiplyTransposeLeft(this, other, this);
   }

   /**
    * Performs a matrix multiplication on this.
    * <p>
    * this = this * other<sup>T</sup>
    * </p>
    *
    * @param other the other matrix to multiply this by. Not modified.
    */
   default void multiplyTransposeOther(RotationMatrixReadOnly other)
   {
      RotationMatrixTools.multiplyTransposeRight(this, other, this);
   }

   @Override
   default void appendInvertOther(Orientation3DReadOnly orientation)
   {
      RotationMatrixTools.multiply(this, false, orientation, true, this);
   }

   /**
    * Performs a matrix multiplication on this.
    * <p>
    * this = this<sup>T</sup> * other<sup>T</sup>
    * </p>
    *
    * @param other the other matrix to multiply this by. Not modified.
    */
   default void multiplyTransposeBoth(RotationMatrixReadOnly other)
   {
      RotationMatrixTools.multiplyTransposeBoth(this, other, this);
   }

   /**
    * Append a rotation about the z-axis to this rotation matrix.
    *
    * <pre>
    *               / cos(yaw) -sin(yaw) 0 \
    * this = this * | sin(yaw)  cos(yaw) 0 |
    *               \    0         0     1 /
    * </pre>
    *
    * @param yaw the angle to rotate about the z-axis.
    */
   @Override
   default void appendYawRotation(double yaw)
   {
      RotationMatrixTools.appendYawRotation(this, yaw, this);
   }

   /**
    * Append a rotation about the y-axis to this rotation matrix.
    *
    * <pre>
    *               /  cos(pitch) 0 sin(pitch) \
    * this = this * |      0      1     0      |
    *               \ -sin(pitch) 0 cos(pitch) /
    * </pre>
    *
    * @param pitch the angle to rotate about the y-axis.
    */
   @Override
   default void appendPitchRotation(double pitch)
   {
      RotationMatrixTools.appendPitchRotation(this, pitch, this);
   }

   /**
    * Append a rotation about the x-axis to this rotation matrix.
    *
    * <pre>
    *               / 1     0          0     \
    * this = this * | 0 cos(roll) -sin(roll) |
    *               \ 0 sin(roll)  cos(roll) /
    * </pre>
    *
    * @param roll the angle to rotate about the x-axis.
    */
   @Override
   default void appendRollRotation(double roll)
   {
      RotationMatrixTools.appendRollRotation(this, roll, this);
   }

   /**
    * Performs a matrix multiplication on this.
    * <p>
    * this = other * this
    * </p>
    *
    * @param other the other matrix to multiply this by. Not modified.
    */
   default void preMultiply(RotationMatrixReadOnly other)
   {
      RotationMatrixTools.multiply(other, this, this);
   }

   @Override
   default void prepend(Orientation3DReadOnly orientation)
   {
      RotationMatrixTools.multiply(orientation, false, this, false, this);
   }

   /**
    * Performs a matrix multiplication on this.
    * <p>
    * this = other * this<sup>T</sup>
    * </p>
    *
    * @param other the other matrix to multiply this by. Not modified.
    */
   default void preMultiplyTransposeThis(RotationMatrixReadOnly other)
   {
      RotationMatrixTools.multiplyTransposeRight(other, this, this);
   }

   /**
    * Performs a matrix multiplication on this.
    * <p>
    * this = other<sup>T</sup> * this
    * </p>
    *
    * @param other the other matrix to multiply this by. Not modified.
    */
   default void preMultiplyTransposeOther(RotationMatrixReadOnly other)
   {
      RotationMatrixTools.multiplyTransposeLeft(other, this, this);
   }

   @Override
   default void prependInvertOther(Orientation3DReadOnly orientation)
   {
      RotationMatrixTools.multiply(orientation, true, this, false, this);
   }

   /**
    * Performs a matrix multiplication on this.
    * <p>
    * this = other<sup>T</sup> * this<sup>T</sup>
    * </p>
    *
    * @param other the other matrix to multiply this by. Not modified.
    */
   default void preMultiplyTransposeBoth(RotationMatrixReadOnly other)
   {
      RotationMatrixTools.multiplyTransposeBoth(other, this, this);
   }

   /**
    * Prepend a rotation about the z-axis to this rotation matrix.
    *
    * <pre>
    *        / cos(yaw) -sin(yaw) 0 \
    * this = | sin(yaw)  cos(yaw) 0 | * this
    *        \    0         0     1 /
    * </pre>
    *
    * @param yaw the angle to rotate about the z-axis.
    */
   @Override
   default void prependYawRotation(double yaw)
   {
      RotationMatrixTools.prependYawRotation(yaw, this, this);
   }

   /**
    * Prepend a rotation about the y-axis to this rotation matrix.
    *
    * <pre>
    *        /  cos(pitch) 0 sin(pitch) \
    * this = |      0      1     0      | * this
    *        \ -sin(pitch) 0 cos(pitch) /
    * </pre>
    *
    * @param pitch the angle to rotate about the y-axis.
    */
   @Override
   default void prependPitchRotation(double pitch)
   {
      RotationMatrixTools.prependPitchRotation(pitch, this, this);
   }

   /**
    * Append a rotation about the x-axis to this rotation matrix.
    *
    * <pre>
    *        / 1     0          0     \
    * this = | 0 cos(roll) -sin(roll) | * this
    *        \ 0 sin(roll)  cos(roll) /
    * </pre>
    *
    * @param roll the angle to rotate about the x-axis.
    */
   @Override
   default void prependRollRotation(double roll)
   {
      RotationMatrixTools.prependRollRotation(roll, this, this);
   }

   /**
    * Performs a linear interpolation in SO(3) from {@code this} to {@code rf} given the percentage
    * {@code alpha}.
    * <p>
    * This is equivalent to but much more computationally expensive than the <i>Spherical Linear
    * Interpolation</i> performed with quaternions.
    * </p>
    *
    * @param rf    the other rotation matrix used for the interpolation. Not modified.
    * @param alpha the percentage used for the interpolation. A value of 0 will result in not modifying
    *              this rotation matrix, while a value of 1 is equivalent to setting this rotation
    *              matrix to {@code rf}.
    */
   default void interpolate(RotationMatrixReadOnly rf, double alpha)
   {
      interpolate(this, rf, alpha);
   }

   /**
    * Performs a linear interpolation in SO(3) from {@code r0} to {@code rf} given the percentage
    * {@code alpha}.
    * <p>
    * This is equivalent to but much more computationally expensive than the <i>Spherical Linear
    * Interpolation</i> performed with quaternions.
    * </p>
    *
    * @param r0    the first rotation matrix used in the interpolation. Not modified.
    * @param rf    the second rotation matrix used in the interpolation. Not modified.
    * @param alpha the percentage to use for the interpolation. A value of 0 will result in setting
    *              this rotation matrix to {@code r0}, while a value of 1 is equivalent to setting this
    *              rotation matrix to {@code rf}.
    */
   default void interpolate(RotationMatrixReadOnly r0, RotationMatrixReadOnly rf, double alpha)
   {
      RotationMatrixTools.interpolate(r0, rf, alpha, this);
   }

   /**
    * {@inheritDoc}
    * <p>
    * this = R * this where 'R' is the 3-by-3 matrix representing the rotation part of the
    * {@code transform}.
    * </p>
    * <p>
    * Note: the transformation of a {@code RotationMatrix} strongly differs from the transformation of
    * a {@link Matrix3DBasics}.
    * </p>
    */
   @Override
   default void applyTransform(Transform transform)
   {
      transform.transform(this);
   }

   /**
    * {@inheritDoc}
    * <p>
    * this = R<sup>T</sup> * this where 'R' is the 3-by-3 matrix representing the rotation part of the
    * {@code transform}.
    * </p>
    * <p>
    * Note: the transformation of a {@code RotationMatrix} strongly differs from the transformation of
    * a {@link Matrix3DBasics}.
    * </p>
    */
   @Override
   default void applyInverseTransform(Transform transform)
   {
      transform.inverseTransform(this);
   }
}
