package us.ihmc.euclid.tuple2D;

import us.ihmc.euclid.interfaces.GeometryObject;
import us.ihmc.euclid.tools.EuclidCoreIOTools;
import us.ihmc.euclid.tools.EuclidHashCodeTools;
import us.ihmc.euclid.tuple2D.interfaces.Point2DBasics;
import us.ihmc.euclid.tuple2D.interfaces.Tuple2DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DReadOnly;

/**
 * A 2D point represents the 2D coordinates of a location on the XY-plane.
 * <p>
 * This version of 2D point uses double precision fields to save the value of each component. It is
 * meant for garbage free usage.
 * </p>
 *
 * @author Sylvain Bertrand
 */
public class Point2D implements Point2DBasics, GeometryObject<Point2D>
{
   /** The x-coordinate. */
   private double x;
   /** The y-coordinate. */
   private double y;

   /**
    * Creates a new point and initializes it coordinates to zero.
    */
   public Point2D()
   {
      setToZero();
   }

   /**
    * Creates a new point and initializes it with the given coordinates.
    *
    * @param x the x-coordinate.
    * @param y the y-coordinate.
    */
   public Point2D(double x, double y)
   {
      set(x, y);
   }

   /**
    * Creates a new point and initializes its component {@code x}, {@code y} in order from the given
    * array.
    *
    * @param pointArray the array containing this point's coordinates. Not modified.
    */
   public Point2D(double[] pointArray)
   {
      set(pointArray);
   }

   /**
    * Creates a new point and initializes it to {@code other}.
    *
    * @param other the tuple to copy the coordinates from. Not modified.
    */
   public Point2D(Tuple2DReadOnly other)
   {
      set(other);
   }

   /**
    * Creates a new point and initializes it to {@code tuple3DReadOnly} x and y components.
    *
    * @param tuple3DReadOnly the tuple to copy the coordinates from. Not modified.
    */
   public Point2D(Tuple3DReadOnly tuple3DReadOnly)
   {
      set(tuple3DReadOnly);
   }

   /**
    * Sets this point to {@code other}.
    *
    * @param other the other point to copy the values from. Not modified.
    */
   @Override
   public void set(Point2D other)
   {
      Point2DBasics.super.set(other);
   }

   /**
    * Sets the x-coordinate of this point.
    *
    * @param x the x-coordinate.
    */
   @Override
   public void setX(double x)
   {
      this.x = x;
   }

   /**
    * Sets the y-coordinate of this point.
    *
    * @param y the y-coordinate.
    */
   @Override
   public void setY(double y)
   {
      this.y = y;
   }

   /**
    * Returns the value of the x-coordinate of this point.
    *
    * @return the x-coordinate's value.
    */
   @Override
   public double getX()
   {
      return x;
   }

   /**
    * Returns the value of the y-coordinate of this point.
    *
    * @return the y-coordinate's value.
    */
   @Override
   public double getY()
   {
      return y;
   }

   /**
    * Tests if the given {@code object}'s class is the same as this, in which case the method returns
    * {@link #equals(Tuple2DReadOnly)}, it returns {@code false} otherwise.
    *
    * @param object the object to compare against this. Not modified.
    * @return {@code true} if {@code object} and this are exactly equal, {@code false} otherwise.
    */
   @Override
   public boolean equals(Object object)
   {
      if (object instanceof Tuple2DReadOnly)
         return equals((Tuple2DReadOnly) object);
      else
         return false;
   }

   /**
    * Tests on a per component basis if this point is equal to the given {@code other} to an
    * {@code epsilon}.
    *
    * @param other   the other point to compare against this. Not modified.
    * @param epsilon the tolerance to use when comparing each component.
    * @return {@code true} if the two tuples are equal, {@code false} otherwise.
    */
   @Override
   public boolean epsilonEquals(Point2D other, double epsilon)
   {
      return Point2DBasics.super.epsilonEquals(other, epsilon);
   }

   /**
    * Tests if {@code this} and {@code other} represent the same point 2D to an {@code epsilon}.
    * <p>
    * Two points are considered geometrically equal if they are at a distance of less than or equal to
    * {@code epsilon}.
    * </p>
    * <p>
    * Note that {@code this.geometricallyEquals(other, epsilon) == true} does not necessarily imply
    * {@code this.epsilonEquals(other, epsilon)} and vice versa.
    * </p>
    *
    * @param other   the other point 2D to compare against this. Not modified.
    * @param epsilon the maximum distance that the two points can be spaced and still considered equal.
    * @return {@code true} if the two points represent the same geometry, {@code false} otherwise.
    */
   @Override
   public boolean geometricallyEquals(Point2D other, double epsilon)
   {
      return Point2DBasics.super.geometricallyEquals(other, epsilon);
   }

   /**
    * Provides a {@code String} representation of this point 2D as follows: (x, y).
    *
    * @return the {@code String} representing this point 2D.
    */
   @Override
   public String toString()
   {
      return EuclidCoreIOTools.getTuple2DString(this);
   }

   /**
    * Calculates and returns a hash code value from the value of each component of this point 2D.
    *
    * @return the hash code value for this point 2D.
    */
   @Override
   public int hashCode()
   {
      long bits = 1L;
      bits = EuclidHashCodeTools.addToHashCode(bits, x);
      bits = EuclidHashCodeTools.addToHashCode(bits, y);
      return EuclidHashCodeTools.toIntHashCode(bits);
   }
}
