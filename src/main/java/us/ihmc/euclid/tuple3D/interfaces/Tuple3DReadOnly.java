package us.ihmc.euclid.tuple3D.interfaces;

import org.ejml.data.DMatrix;

import us.ihmc.euclid.Axis3D;
import us.ihmc.euclid.interfaces.EuclidGeometry;
import us.ihmc.euclid.tools.EuclidCoreIOTools;
import us.ihmc.euclid.tools.EuclidCoreTools;
import us.ihmc.euclid.tools.TupleTools;

/**
 * Read-only interface for a 3 dimensional tuple.
 * <p>
 * A tuple is an abstract geometry object holding onto the common math between a 3D point and
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
public interface Tuple3DReadOnly extends EuclidGeometry
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
    * Tests if this tuple contains a {@link Double#NaN}.
    *
    * @return {@code true} if this tuple contains a {@link Double#NaN}, {@code false} otherwise.
    */
   default boolean containsNaN()
   {
      return EuclidCoreTools.containsNaN(getX(), getY(), getZ());
   }

   /**
    * Selects and returns the component of this tuple corresponding to the given {@code axis}.
    *
    * @param axis the axis of the component to get.
    * @return the value of the component.
    */
   default double getElement(Axis3D axis)
   {
      return axis.extract(this);
   }

   /**
    * Selects a component of this tuple based on {@code index} and returns its value.
    * <p>
    * For an {@code index} of 0, the corresponding component is {@code x}, 1 it is {@code y}, 2 it is
    * {@code z}.
    * </p>
    *
    * @param index the index of the component to get.
    * @return the value of the component.
    * @throws IndexOutOfBoundsException if {@code index} &notin; [0, 2].
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
         default:
            throw new IndexOutOfBoundsException(Integer.toString(index));
      }
   }

   /**
    * Selects a component of this tuple based on {@code index} and returns its value.
    * <p>
    * For an {@code index} of 0, the corresponding component is {@code x}, 1 it is {@code y}, 2 it is
    * {@code z}.
    * </p>
    *
    * @param index the index of the component to get.
    * @return the value of the component.
    * @throws IndexOutOfBoundsException if {@code index} &notin; [0, 2].
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
         default:
            throw new IndexOutOfBoundsException(Integer.toString(index));
      }
   }

   /**
    * Packs the components {@code x}, {@code y}, {@code z} in order in an array starting from its first
    * index.
    *
    * @param tupleArrayToPack the array in which this tuple is stored. Modified.
    */
   default void get(double[] tupleArrayToPack)
   {
      get(0, tupleArrayToPack);
   }

   /**
    * Packs the components {@code x}, {@code y}, {@code z} in order in an array starting from
    * {@code startIndex}.
    *
    * @param startIndex       the index in the array where the first component is stored.
    * @param tupleArrayToPack the array in which this tuple is stored. Modified.
    */
   default void get(int startIndex, double[] tupleArrayToPack)
   {
      tupleArrayToPack[startIndex++] = getX();
      tupleArrayToPack[startIndex++] = getY();
      tupleArrayToPack[startIndex] = getZ();
   }

   /**
    * Packs the components {@code x}, {@code y}, {@code z} in order in an array starting from its first
    * index.
    *
    * @param tupleArrayToPack the array in which this tuple is stored. Modified.
    */
   default void get(float[] tupleArrayToPack)
   {
      get(0, tupleArrayToPack);
   }

   /**
    * Packs the components {@code x}, {@code y}, {@code z} in order in an array starting from
    * {@code startIndex}.
    *
    * @param startIndex       the index in the array where the first component is stored.
    * @param tupleArrayToPack the array in which this tuple is stored. Modified.
    */
   default void get(int startIndex, float[] tupleArrayToPack)
   {
      tupleArrayToPack[startIndex++] = getX32();
      tupleArrayToPack[startIndex++] = getY32();
      tupleArrayToPack[startIndex] = getZ32();
   }

   /**
    * Packs the components {@code x}, {@code y}, {@code z} in order in a column vector starting from
    * its first row index.
    *
    * @param tupleMatrixToPack the array in which this tuple is stored. Modified.
    */
   default void get(DMatrix tupleMatrixToPack)
   {
      get(0, 0, tupleMatrixToPack);
   }

   /**
    * Packs the components {@code x}, {@code y}, {@code z} in order in a column vector starting from
    * {@code startRow}.
    *
    * @param startRow          the first row index to start writing in the matrix.
    * @param tupleMatrixToPack the column vector in which this tuple is stored. Modified.
    */
   default void get(int startRow, DMatrix tupleMatrixToPack)
   {
      get(startRow, 0, tupleMatrixToPack);
   }

   /**
    * Packs the components {@code x}, {@code y}, {@code z} in order in a column vector starting from
    * {@code startRow} at the column index {@code column}.
    *
    * @param startRow          the first row index to start writing in the matrix.
    * @param column            the column index to write in the matrix.
    * @param tupleMatrixToPack the matrix in which this tuple is stored. Modified.
    */
   default void get(int startRow, int column, DMatrix tupleMatrixToPack)
   {
      EuclidCoreTools.checkMatrixMinimumSize(startRow + 3, column + 1, tupleMatrixToPack);
      tupleMatrixToPack.unsafe_set(startRow++, column, getX());
      tupleMatrixToPack.unsafe_set(startRow++, column, getY());
      tupleMatrixToPack.unsafe_set(startRow, column, getZ());
   }

   /**
    * Calculates and returns the norm of this tuple.
    * <p>
    * norm = &radic;(x<sup>2</sup> + y<sup>2</sup> + z<sup>2</sup>)
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
    * norm<sup>2</sup> = x<sup>2</sup> + y<sup>2</sup> + z<sup>2</sup>
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
    * Calculates the norm of the difference between {@code this} and {@code other}.
    * <p>
    * |{@code this} - {@code other}| = &radic;[({@code this.x} - {@code other.x})<sup>2</sup> +
    * ({@code this.y} - {@code other.y})<sup>2</sup> + ({@code this.z} - {@code other.z})<sup>2</sup>]
    * </p>
    *
    * @param other the other tuple to compare to. Not modified.
    * @return the norm squared of the difference.
    */
   default double differenceNorm(Tuple3DReadOnly other)
   {
      return EuclidCoreTools.squareRoot(differenceNormSquared(other));
   }

   /**
    * Calculates the norm squared of the difference between {@code this} and {@code other}.
    * <p>
    * |{@code this} - {@code other}|<sup>2</sup> = ({@code this.x} - {@code other.x})<sup>2</sup> +
    * ({@code this.y} - {@code other.y})<sup>2</sup> + ({@code this.z} - {@code other.z})<sup>2</sup>
    * </p>
    *
    * @param other the other tuple to compare to. Not modified.
    * @return the norm squared of the difference.
    */
   default double differenceNormSquared(Tuple3DReadOnly other)
   {
      double dx = getX() - other.getX();
      double dy = getY() - other.getY();
      double dz = getZ() - other.getZ();
      return EuclidCoreTools.normSquared(dx, dy, dz);
   }

   /**
    * Calculates and returns the value of the dot product of this tuple with {@code other}.
    * <p>
    * For instance, the dot product of two tuples p and q is defined as: <br>
    * p . q = &sum;<sub>i=1:3</sub>(p<sub>i</sub> * q<sub>i</sub>)
    * </p>
    *
    * @param other the other tuple used for the dot product. Not modified.
    * @return the value of the dot product.
    */
   default double dot(Tuple3DReadOnly other)
   {
      return getX() * other.getX() + getY() * other.getY() + getZ() * other.getZ();
   }

   /** {@inheritDoc} */
   @Override
   default boolean epsilonEquals(EuclidGeometry geometry, double epsilon)
   {
      if (geometry == this)
         return true;
      if (geometry == null)
         return false;
      if (!(geometry instanceof Tuple3DReadOnly))
         return false;
      Tuple3DReadOnly other = (Tuple3DReadOnly) geometry;
      return TupleTools.epsilonEquals(this, other, epsilon);
   }

   /** {@inheritDoc} */
   @Override
   default boolean equals(EuclidGeometry geometry)
   {
      if (geometry == this)
         return true;
      if (geometry == null)
         return false;
      if (!(geometry instanceof Tuple3DReadOnly))
         return false;
      Tuple3DReadOnly other = (Tuple3DReadOnly) geometry;
      if (!EuclidCoreTools.equals(getX(), other.getX()))
         return false;
      if (!EuclidCoreTools.equals(getY(), other.getY()))
         return false;
      if (!EuclidCoreTools.equals(getZ(), other.getZ()))
         return false;
      return true;
   }

   /**
    * Gets a representative {@code String} of this tuple 3D given a specific format to use.
    * <p>
    * Using the default format {@link EuclidCoreIOTools#DEFAULT_FORMAT}, this provides a {@code String}
    * as follows:
    *
    * <pre>
    * (-0.558, -0.380,  0.130 )
    * </pre>
    * </p>
    */
   @Override
   default String toString(String format)
   {
      return EuclidCoreIOTools.getTuple3DString(format, this);
   }
}