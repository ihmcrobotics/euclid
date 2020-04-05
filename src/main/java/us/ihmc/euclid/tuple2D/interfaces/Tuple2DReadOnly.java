package us.ihmc.euclid.tuple2D.interfaces;

import org.ejml.data.DenseMatrix64F;

import us.ihmc.euclid.tools.EuclidCoreTools;
import us.ihmc.euclid.tools.TupleTools;

/**
 * Read-only interface for a 2 dimensional tuple.
 * <p>
 * A tuple is an abstract geometry object holding onto the common math between a 2D point and
 * vector.
 * </p>
 * <p>
 * Although a point and vector hold onto the same type of information, the distinction is made
 * between them as they represent different geometry objects and are typically not handled the same
 * way:
 * <ul>
 * <li>a point represents the coordinate of a location in space. A notable difference with a vector
 * is that the distance between two points has a physical meaning. When a point is transformed with
 * a homogeneous transformation matrix, a point's coordinates are susceptible to be scaled, rotated,
 * and translated.
 * <li>a vector is not constrained to a location in space. Instead, a vector represents some
 * physical quantity that has a direction and a magnitude such as: a velocity, a force, the
 * translation from one point to another, etc. When a vector is transformed with a homogeneous
 * transformation matrix, its components are susceptible to be scaled and rotated, but never to be
 * translated.
 * </ul>
 * </p>
 *
 * @author Sylvain Bertrand
 */
public interface Tuple2DReadOnly
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
    * Tests if this tuple contains a {@link Double#NaN}.
    *
    * @return {@code true} if this tuple contains a {@link Double#NaN}, {@code false} otherwise.
    */
   default boolean containsNaN()
   {
      return EuclidCoreTools.containsNaN(getX(), getY());
   }

   /**
    * Selects a component of this tuple based on {@code index} and returns its value.
    * <p>
    * For an {@code index} of 0, the corresponding component is {@code x}, while for 1 it is {@code y}.
    * </p>
    *
    * @param index the index of the component to get.
    * @return the value of the component.
    * @throws IndexOutOfBoundsException if {@code index} &notin; [0, 1].
    */
   default double getElement(int index)
   {
      switch (index)
      {
         case 0:
            return getX();
         case 1:
            return getY();
         default:
            throw new IndexOutOfBoundsException(Integer.toString(index));
      }
   }

   /**
    * Selects a component of this tuple based on {@code index} and returns its value. For an
    * {@code index} of 0, the corresponding component is {@code x}, while for 1 it is {@code y}.
    *
    * @param index the index of the component to get.
    * @return the value of the component.
    * @throws IndexOutOfBoundsException if {@code index} &notin; [0, 1].
    */
   default float getElement32(int index)
   {
      switch (index)
      {
         case 0:
            return getX32();
         case 1:
            return getY32();
         default:
            throw new IndexOutOfBoundsException(Integer.toString(index));
      }
   }

   /**
    * Packs the components {@code x}, {@code y} in order in an array starting from its first index.
    *
    * @param tupleArrayToPack the array in which this tuple is stored. Modified.
    */
   default void get(double[] tupleArrayToPack)
   {
      get(0, tupleArrayToPack);
   }

   /**
    * Packs the components {@code x}, {@code y} in order in an array starting from {@code startIndex}.
    *
    * @param startIndex       the index in the array where the first component is stored.
    * @param tupleArrayToPack the array in which this tuple is stored. Modified.
    */
   default void get(int startIndex, double[] tupleArrayToPack)
   {
      tupleArrayToPack[startIndex++] = getX();
      tupleArrayToPack[startIndex] = getY();
   }

   /**
    * Packs the components {@code x}, {@code y} in order in an array starting from its first index.
    *
    * @param tupleArrayToPack the array in which this tuple is stored. Modified.
    */
   default void get(float[] tupleArrayToPack)
   {
      get(0, tupleArrayToPack);
   }

   /**
    * Packs the components {@code x}, {@code y} in order in an array starting from {@code startIndex}.
    *
    * @param startIndex       the index in the array where the first component is stored.
    * @param tupleArrayToPack the array in which this tuple is stored. Modified.
    */
   default void get(int startIndex, float[] tupleArrayToPack)
   {
      tupleArrayToPack[startIndex++] = getX32();
      tupleArrayToPack[startIndex] = getY32();
   }

   /**
    * Packs the components {@code x}, {@code y} in order in a column vector starting from its first row
    * index.
    *
    * @param tupleMatrixToPack the array in which this tuple is stored. Modified.
    */
   default void get(DenseMatrix64F tupleMatrixToPack)
   {
      get(0, 0, tupleMatrixToPack);
   }

   /**
    * Packs the components {@code x}, {@code y} in order in a column vector starting from
    * {@code startRow}.
    *
    * @param startRow          the first row index to start writing in the dense-matrix.
    * @param tupleMatrixToPack the column vector in which this tuple is stored. Modified.
    */
   default void get(int startRow, DenseMatrix64F tupleMatrixToPack)
   {
      get(startRow, 0, tupleMatrixToPack);
   }

   /**
    * Packs the components {@code x}, {@code y} in order in a column vector starting from
    * {@code startRow} at the column index {@code column}.
    *
    * @param startRow          the first row index to start writing in the dense-matrix.
    * @param column            the column index to write in the dense-matrix.
    * @param tupleMatrixToPack the matrix in which this tuple is stored. Modified.
    */
   default void get(int startRow, int column, DenseMatrix64F tupleMatrixToPack)
   {
      EuclidCoreTools.checkMatrixMinimumSize(startRow + 2, column + 1, tupleMatrixToPack);
      tupleMatrixToPack.unsafe_set(startRow++, column, getX());
      tupleMatrixToPack.unsafe_set(startRow, column, getY());
   }

   /**
    * Tests on a per component basis if this tuple is equal to the given {@code other} to an
    * {@code epsilon}.
    *
    * @param other   the other tuple to compare against this. Not modified.
    * @param epsilon the tolerance to use when comparing each component.
    * @return {@code true} if the two tuples are equal, {@code false} otherwise.
    */
   default boolean epsilonEquals(Tuple2DReadOnly other, double epsilon)
   {
      return TupleTools.epsilonEquals(this, other, epsilon);
   }

   /**
    * Tests on a per component basis, if this tuple is exactly equal to {@code other}.
    *
    * @param other the other tuple to compare against this. Not modified.
    * @return {@code true} if the two tuples are exactly equal component-wise, {@code false} otherwise.
    */
   default boolean equals(Tuple2DReadOnly other)
   {
      if (other == this)
         return true;
      else if (other == null)
         return false;
      else
         return getX() == other.getX() && getY() == other.getY();
   }
}