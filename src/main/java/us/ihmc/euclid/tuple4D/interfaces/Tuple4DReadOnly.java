package us.ihmc.euclid.tuple4D.interfaces;

import org.ejml.data.DMatrix;

import us.ihmc.euclid.interfaces.EuclidGeometry;
import us.ihmc.euclid.tools.EuclidCoreIOTools;
import us.ihmc.euclid.tools.EuclidCoreTools;
import us.ihmc.euclid.tools.TupleTools;

/**
 * Read-only interface for a 4 dimensional tuple.
 * <p>
 * A tuple 4D represents what is commonly called a quaternion. Although from definition, a
 * quaternion does not necessarily represent an 3D orientation, in this library the classes
 * implementing {@link QuaternionReadOnly} and {@link QuaternionBasics} represent unit-quaternions
 * meant to represent 3D orientations. The classes implementing {@link Vector4DReadOnly} and
 * {@link Vector4DBasics} are used to represent generic quaternions.
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
public interface Tuple4DReadOnly extends EuclidGeometry
{
   /**
    * Returns the x-component of this tuple.
    *
    * @return the x-component.
    */
   double getX();

   /**
    * Returns the y-component of this tuple.
    *
    * @return the y-component.
    */
   double getY();

   /**
    * Returns the z-component of this tuple.
    *
    * @return the z-component.
    */
   double getZ();

   /**
    * Returns the s-component of this tuple.
    *
    * @return the s-component.
    */
   double getS();

   /**
    * Returns the x-component of this tuple.
    *
    * @return the x-component.
    */
   default float getX32()
   {
      return (float) getX();
   }

   /**
    * Returns the y-component of this tuple.
    *
    * @return the y-component.
    */
   default float getY32()
   {
      return (float) getY();
   }

   /**
    * Returns the z-component of this tuple.
    *
    * @return the z-component.
    */
   default float getZ32()
   {
      return (float) getZ();
   }

   /**
    * Returns the s-component of this tuple.
    *
    * @return the s-component.
    */
   default float getS32()
   {
      return (float) getS();
   }

   /**
    * Tests if this tuple contains a {@link Double#NaN}.
    *
    * @return {@code true} if this tuple contains a {@link Double#NaN}, {@code false} otherwise.
    */
   default boolean containsNaN()
   {
      return EuclidCoreTools.containsNaN(getX(), getY(), getZ(), getS());
   }

   /**
    * Selects a component of this tuple based on {@code index} and returns its value.
    * <p>
    * For an {@code index} value going from 0 up to 3, the corresponding components are {@code x},
    * {@code y}, {@code z}, and {@code s}, respectively.
    * </p>
    *
    * @param index the index of the component to get.
    * @return the value of the component.
    * @throws IndexOutOfBoundsException if {@code index} &notin; [0, 3].
    */
   default double getElement(int index)
   {
      switch (index)
      {
         case 0:
            return getX();
         case 1:
            return getY();
         case 2:
            return getZ();
         case 3:
            return getS();
         default:
            throw new IndexOutOfBoundsException(Integer.toString(index));
      }
   }

   /**
    * Selects a component of this tuple based on {@code index} and returns its value.
    * <p>
    * For an {@code index} value going from 0 up to 3, the corresponding components are {@code x},
    * {@code y}, {@code z}, and {@code s}, respectively.
    * </p>
    *
    * @param index the index of the component to get.
    * @return the value of the component.
    * @throws IndexOutOfBoundsException if {@code index} &notin; [0, 3].
    */
   default float getElement32(int index)
   {
      switch (index)
      {
         case 0:
            return getX32();
         case 1:
            return getY32();
         case 2:
            return getZ32();
         case 3:
            return getS32();
         default:
            throw new IndexOutOfBoundsException(Integer.toString(index));
      }
   }

   /**
    * Packs the components {@code x}, {@code y}, {@code z}, {@code s} in order in an array starting
    * from its first index.
    *
    * @param tupleArrayToPack the array in which this tuple is stored. Modified.
    */
   default void get(double[] tupleArrayToPack)
   {
      get(0, tupleArrayToPack);
   }

   /**
    * Packs the components {@code x}, {@code y}, {@code z}, {@code s} in order in an array starting
    * from {@code startIndex}.
    *
    * @param startIndex       the index in the array where the first component is stored.
    * @param tupleArrayToPack the array in which this tuple is stored. Modified.
    */
   default void get(int startIndex, double[] tupleArrayToPack)
   {
      tupleArrayToPack[startIndex++] = getX();
      tupleArrayToPack[startIndex++] = getY();
      tupleArrayToPack[startIndex++] = getZ();
      tupleArrayToPack[startIndex] = getS();
   }

   /**
    * Packs the components {@code x}, {@code y}, {@code z}, {@code s} in order in an array starting
    * from its first index.
    *
    * @param tupleArrayToPack the array in which this tuple is stored. Modified.
    */
   default void get(float[] tupleArrayToPack)
   {
      get(0, tupleArrayToPack);
   }

   /**
    * Packs the components {@code x}, {@code y}, {@code z}, {@code s} in order in an array starting
    * from {@code startIndex}.
    *
    * @param startIndex       the index in the array where the first component is stored.
    * @param tupleArrayToPack the array in which this tuple is stored. Modified.
    */
   default void get(int startIndex, float[] tupleArrayToPack)
   {
      tupleArrayToPack[startIndex++] = getX32();
      tupleArrayToPack[startIndex++] = getY32();
      tupleArrayToPack[startIndex++] = getZ32();
      tupleArrayToPack[startIndex] = getS32();
   }

   /**
    * Packs the components {@code x}, {@code y}, {@code z}, {@code s} in order in a column vector
    * starting from its first row index.
    *
    * @param tupleMatrixToPack the array in which this tuple is stored. Modified.
    */
   default void get(DMatrix tupleMatrixToPack)
   {
      get(0, 0, tupleMatrixToPack);
   }

   /**
    * Packs the components {@code x}, {@code y}, {@code z}, {@code s} in order in a column vector
    * starting from {@code startRow}.
    *
    * @param startRow          the first row index to start writing in the matrix.
    * @param tupleMatrixToPack the column vector in which this tuple is stored. Modified.
    */
   default void get(int startRow, DMatrix tupleMatrixToPack)
   {
      get(startRow, 0, tupleMatrixToPack);
   }

   /**
    * Packs the components {@code x}, {@code y}, {@code z}, {@code s} in order in a column vector
    * starting from {@code startRow} at the column index {@code column}.
    *
    * @param startRow          the first row index to start writing in the matrix.
    * @param column            the column index to write in the matrix.
    * @param tupleMatrixToPack the matrix in which this tuple is stored. Modified.
    */
   default void get(int startRow, int column, DMatrix tupleMatrixToPack)
   {
      EuclidCoreTools.checkMatrixMinimumSize(startRow + 4, column + 1, tupleMatrixToPack);
      tupleMatrixToPack.unsafe_set(startRow++, column, getX());
      tupleMatrixToPack.unsafe_set(startRow++, column, getY());
      tupleMatrixToPack.unsafe_set(startRow++, column, getZ());
      tupleMatrixToPack.unsafe_set(startRow, column, getS());
   }

   /**
    * Calculates and returns the norm of this tuple.
    * <p>
    * norm = &radic;(x<sup>2</sup> + y<sup>2</sup> + z<sup>2</sup> + s<sup>2</sup>)
    * </p>
    *
    * @return the norm's value of this tuple.
    */
   default double norm()
   {
      return EuclidCoreTools.squareRoot(normSquared());
   }

   /**
    * Calculates and returns the square of the norm of this tuple.
    * <p>
    * norm<sup>2</sup> = x<sup>2</sup> + y<sup>2</sup> + z<sup>2</sup> + s<sup>2</sup>
    * </p>
    * <p>
    * This method is usually preferred over {@link #norm()} when calculation speed matters and
    * knowledge of the actual norm does not, i.e. when comparing several tuples by theirs norm.
    * </p>
    *
    * @return the norm's value of this tuple.
    */
   default double normSquared()
   {
      return dot(this);
   }

   /**
    * Calculates and returns the value of the dot product of this tuple with {@code other}.
    * <p>
    * For instance, the dot product of two tuples p and q is defined as: <br>
    * p . q = &sum;<sub>i=1:4</sub>(p<sub>i</sub> * q<sub>i</sub>)
    * </p>
    *
    * @param other the other vector used for the dot product. Not modified.
    * @return the value of the dot product.
    */
   default double dot(Tuple4DReadOnly other)
   {
      return getX() * other.getX() + getY() * other.getY() + getZ() * other.getZ() + getS() * other.getS();
   }

   /** {@inheritDoc} */
   @Override
   default boolean equals(EuclidGeometry geometry)
   {
      if (geometry == this)
         return true;
      if (geometry == null)
         return false;
      if (!(geometry instanceof Tuple4DReadOnly))
         return false;
      Tuple4DReadOnly other = (Tuple4DReadOnly) geometry;
      return getX() == other.getX() && getY() == other.getY() && getZ() == other.getZ() && getS() == other.getS();
   }

   /** {@inheritDoc} */
   @Override
   default boolean epsilonEquals(EuclidGeometry geometry, double epsilon)
   {
      if (!(geometry instanceof Tuple4DReadOnly))
         return false;

      Tuple4DReadOnly other = (Tuple4DReadOnly) geometry;
      return TupleTools.epsilonEquals(this, other, epsilon);
   }

   /**
    * Provides a {@code String} representation of this tuple4D as follows: (x, y, z, s).
    *
    * @param format the format to use for each number.
    * @return the {@code String} representing this tuple4D.
    */
   @Override
   default String toString(String format)
   {
      return EuclidCoreIOTools.getTuple4DString(format, this);
   }

}