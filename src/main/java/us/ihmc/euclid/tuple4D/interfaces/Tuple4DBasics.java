package us.ihmc.euclid.tuple4D.interfaces;

import org.ejml.data.DMatrix;

import us.ihmc.euclid.interfaces.Clearable;
import us.ihmc.euclid.interfaces.Transformable;
import us.ihmc.euclid.tools.EuclidCoreTools;

/**
 * Write and read interface for a 4 dimensional tuple.
 * <p>
 * A tuple 4D represents what is commonly called a quaternion. Although from definition, a
 * quaternion does not necessarily represent an 3D orientation, in this library the classes
 * implementing {@link QuaternionReadOnly} and {@link QuaternionBasics} represent unit-quaternions
 * meant to represent 3D orientations. The classes implementing {@link Vector4DReadOnly} and
 * {@link Vector4DBasics} are used to represent generic quaternions.
 * </p>
 * <p>
 * The write interface for a 4D tuple is more restricted than for the 2D and 3D tuples to improve
 * data safety for the classes representing unit-quaternions.
 * </p>
 * <p>
 * When describing a 4D tuple, its 4 components are often gathered in two groups: the scalar part
 * {@code s} and the vector part ({@code x}, {@code y}, {@code z}).
 * </p>
 * <p>
 * Note on the difference between applying a 3D transform on a quaternion and a 4D vector:
 * <ul>
 * <li>When transformed by a homogeneous transformation matrix, a quaternion is only pre-multiplied
 * by the rotation part of the transform, resulting in concatenating the orientations of the
 * transform and the quaternion.
 * <li>When transformed by a homogeneous transformation matrix, a 4D vector scalar part {@code s}
 * remains unchanged. The vector part ({@code x}, {@code y}, {@code z}) is scaled and rotated, and
 * translated by {@code s} times the translation part of the transform. Note that for {@code s = 0},
 * a 4D vector behaves as a 3D vector, and for {@code s = 1} it behaves as a 3D point.
 * </ul>
 * </p>
 *
 * @author Sylvain Bertrand
 */
public interface Tuple4DBasics extends Tuple4DReadOnly, Clearable, Transformable
{
   /**
    * Sets this tuple's components to {@code x}, {@code y}, {@code z}, and {@code s}.
    *
    * @param x the new value for the x-component of this tuple.
    * @param y the new value for the y-component of this tuple.
    * @param z the new value for the z-component of this tuple.
    * @param s the new value for the s-component of this tuple.
    */
   void set(double x, double y, double z, double s);

   /**
    * Normalizes this tuple such that its norm is equal to 1 after calling this method and its
    * direction remains unchanged.
    * <p>
    * Edge cases:
    * <ul>
    * <li>if this tuple contains {@link Double#NaN}, this method is ineffective.
    * </ul>
    * </p>
    */
   void normalize();

   @Override
   default void setToNaN()
   {
      set(Double.NaN, Double.NaN, Double.NaN, Double.NaN);
   }

   /** {@inheritDoc} */
   @Override
   default boolean containsNaN()
   {
      return Tuple4DReadOnly.super.containsNaN();
   }

   /**
    * Sets each component of this tuple to its absolute value.
    */
   default void absolute()
   {
      set(Math.abs(getX()), Math.abs(getY()), Math.abs(getZ()), Math.abs(getS()));
   }

   /**
    * Changes the sign of each component of this tuple.
    */
   default void negate()
   {
      set(-getX(), -getY(), -getZ(), -getS());
   }

   /**
    * Sets this tuple to {@code tupleReadOnly}.
    *
    * @param other the other tuple to copy the values from. Not modified.
    */
   default void set(Tuple4DReadOnly other)
   {
      set(other.getX(), other.getY(), other.getZ(), other.getS());
   }

   /**
    * Sets this tuple's components {@code x}, {@code y}, {@code z}, {@code s} in order from the given
    * array {@code tupleArray}.
    *
    * @param tupleArray the array containing the new values for this tuple's components. Not modified.
    */
   default void set(double[] tupleArray)
   {
      set(tupleArray[0], tupleArray[1], tupleArray[2], tupleArray[3]);
   }

   /**
    * Sets this tuple's components {@code x}, {@code y}, {@code z}, {@code s} in order from the given
    * array {@code tupleArray}.
    *
    * @param startIndex the first index to start reading from in the array.
    * @param tupleArray the array containing the new values for this tuple's components. Not modified.
    */
   default void set(int startIndex, double[] tupleArray)
   {
      set(tupleArray[startIndex++], tupleArray[startIndex++], tupleArray[startIndex++], tupleArray[startIndex]);
   }

   /**
    * Sets this tuple's components {@code x}, {@code y}, {@code z}, {@code s} in order from the given
    * array {@code tupleArray}.
    *
    * @param tupleArray the array containing the new values for this tuple's components. Not modified.
    */
   default void set(float[] tupleArray)
   {
      set(tupleArray[0], tupleArray[1], tupleArray[2], tupleArray[3]);
   }

   /**
    * Sets this tuple's components {@code x}, {@code y}, {@code z}, {@code s} in order from the given
    * array {@code tupleArray}.
    *
    * @param startIndex the first index to start reading from in the array.
    * @param tupleArray the array containing the new values for this tuple's components. Not modified.
    */
   default void set(int startIndex, float[] tupleArray)
   {
      set(tupleArray[startIndex++], tupleArray[startIndex++], tupleArray[startIndex++], tupleArray[startIndex]);
   }

   /**
    * Sets this tuple's components {@code x}, {@code y}, {@code z}, {@code s} in order from the given
    * column vector starting to read from its first row index.
    *
    * @param matrix the column vector containing the new values for this tuple's components. Not
    *               modified.
    */
   default void set(DMatrix matrix)
   {
      EuclidCoreTools.checkMatrixMinimumSize(4, 1, matrix);
      set(matrix.unsafe_get(0, 0), matrix.unsafe_get(1, 0), matrix.unsafe_get(2, 0), matrix.unsafe_get(3, 0));
   }

   /**
    * Sets this tuple's components {@code x}, {@code y}, {@code z}, {@code s} in order from the given
    * column vector starting to read from {@code startRow}.
    *
    * @param startRow the first row index to start reading in the dense-matrix.
    * @param matrix   the column vector containing the new values for this tuple's components. Not
    *                 modified.
    */
   default void set(int startRow, DMatrix matrix)
   {
      EuclidCoreTools.checkMatrixMinimumSize(startRow + 4, 1, matrix);
      set(matrix.unsafe_get(startRow++, 0), matrix.unsafe_get(startRow++, 0), matrix.unsafe_get(startRow++, 0), matrix.unsafe_get(startRow, 0));
   }

   /**
    * Sets this tuple's components {@code x}, {@code y}, {@code z}, {@code s} in order from the given
    * matrix starting to read from {@code startRow} at the column index {@code column}.
    *
    * @param startRow the first row index to start reading in the dense-matrix.
    * @param column   the column index to read in the dense-matrix.
    * @param matrix   the column vector containing the new values for this tuple's components. Not
    *                 modified.
    */
   default void set(int startRow, int column, DMatrix matrix)
   {
      EuclidCoreTools.checkMatrixMinimumSize(startRow + 4, column + 1, matrix);
      set(matrix.unsafe_get(startRow++, column),
          matrix.unsafe_get(startRow++, column),
          matrix.unsafe_get(startRow++, column),
          matrix.unsafe_get(startRow, column));
   }

   /**
    * Sets this tuple to {@code other} and then calls {@link #absolute()}.
    *
    * @param other the other tuple to copy the values from. Not modified.
    */
   default void setAndAbsolute(Tuple4DReadOnly other)
   {
      set(Math.abs(other.getX()), Math.abs(other.getY()), Math.abs(other.getZ()), Math.abs(other.getS()));
   }

   /**
    * Sets this tuple to {@code other} and then calls {@link #negate()}.
    *
    * @param other the other tuple to copy the values from. Not modified.
    */
   default void setAndNegate(Tuple4DReadOnly other)
   {
      set(-other.getX(), -other.getY(), -other.getZ(), -other.getS());
   }

   /**
    * Sets this tuple to {@code other} and then calls {@link #normalize()}.
    *
    * @param other the other tuple to copy the values from. Not modified.
    */
   default void setAndNormalize(Tuple4DReadOnly other)
   {
      set(other);
      normalize();
   }
}