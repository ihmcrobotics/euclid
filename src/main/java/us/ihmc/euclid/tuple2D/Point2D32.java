package us.ihmc.euclid.tuple2D;

import us.ihmc.euclid.tools.EuclidCoreIOTools;
import us.ihmc.euclid.tools.EuclidHashCodeTools;
import us.ihmc.euclid.tuple2D.interfaces.Point2DBasics;
import us.ihmc.euclid.tuple2D.interfaces.Tuple2DReadOnly;

/**
 * A 2D point represents the 2D coordinates of a location on the XY-plane.
 * <p>
 * This version of 2D point uses single precision fields to save the value of each component. It is
 * meant for garbage free usage and for situations where heap memory is limited. When memory is not
 * a constraint, the use of {@link Point2D} is preferable.
 * </p>
 *
 * @author Sylvain Bertrand
 */
public class Point2D32 implements Point2DBasics
{
   /** The x-coordinate. */
   private float x;
   /** The y-coordinate. */
   private float y;

   /**
    * Creates a new point and initializes it coordinates to zero.
    */
   public Point2D32()
   {
      setToZero();
   }

   /**
    * Creates a new point and initializes it with the given coordinates.
    *
    * @param x the x-coordinate.
    * @param y the y-coordinate.
    */
   public Point2D32(float x, float y)
   {
      set(x, y);
   }

   /**
    * Creates a new point and initializes its component {@code x}, {@code y} in order from the given
    * array.
    *
    * @param pointArray the array containing this point's coordinates. Not modified.
    */
   public Point2D32(float[] pointArray)
   {
      set(pointArray);
   }

   /**
    * Creates a new point and initializes it to {@code other}.
    *
    * @param other the tuple to copy the coordinates from. Not modified.
    */
   public Point2D32(Tuple2DReadOnly other)
   {
      set(other);
   }

   /**
    * Sets the x-coordinate of this point.
    *
    * @param x the x-coordinate.
    */
   @Override
   public void setX(double x)
   {
      this.x = (float) x;
   }

   /**
    * Sets the y-coordinate of this point.
    *
    * @param y the y-coordinate.
    */
   @Override
   public void setY(double y)
   {
      this.y = (float) y;
   }

   /**
    * Sets the x-coordinate of this point.
    *
    * @param x the x-coordinate.
    */
   public void setX(float x)
   {
      this.x = x;
   }

   /**
    * Sets the y-coordinate of this point.
    *
    * @param y the y-coordinate.
    */
   public void setY(float y)
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
    * Returns the value of the x-coordinate of this point.
    *
    * @return the x-coordinate's value.
    */
   @Override
   public float getX32()
   {
      return x;
   }

   /**
    * Returns the value of the y-coordinate of this point.
    *
    * @return the y-coordinate's value.
    */
   @Override
   public float getY32()
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
      return EuclidHashCodeTools.toIntHashCode(x, y);
   }
}
