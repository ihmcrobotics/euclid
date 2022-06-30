package us.ihmc.euclid.tuple3D;

import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.tools.EuclidCoreIOTools;
import us.ihmc.euclid.tools.EuclidHashCodeTools;
import us.ihmc.euclid.tuple2D.interfaces.Tuple2DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Point3DBasics;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DReadOnly;

/**
 * A 3D point represents the 3D coordinates of a location in space.
 * <p>
 * This version of 3D point uses double precision fields to save the value of each component. It is
 * meant for garbage free usage.
 * </p>
 *
 * @author Sylvain Bertrand
 */
public class Point3D implements Point3DBasics, Settable<Point3D>
{
   /** The x-coordinate. */
   private double x;
   /** The y-coordinate. */
   private double y;
   /** The z-coordinate. */
   private double z;

   /**
    * Creates a new point and initializes it coordinates to zero.
    */
   public Point3D()
   {
      setToZero();
   }

   /**
    * Creates a new point and initializes it with the given coordinates.
    *
    * @param x the x-coordinate.
    * @param y the y-coordinate.
    * @param z the z-coordinate.
    */
   public Point3D(double x, double y, double z)
   {
      set(x, y, z);
   }

   /**
    * Creates a new point and initializes its component {@code x}, {@code y}, {@code z} in order from
    * the given array.
    *
    * @param pointArray the array containing this point's coordinates. Not modified.
    */
   public Point3D(double[] pointArray)
   {
      set(pointArray);
   }

   /**
    * Creates a new point and initializes its x and y components to {@code tuple2DReadOnly}.
    *
    * @param tuple2DReadOnly the tuple to copy the coordinates from. Not modified.
    */
   public Point3D(Tuple2DReadOnly tuple2DReadOnly)
   {
      set(tuple2DReadOnly);
   }

   /**
    * Creates a new point and initializes it to {@code other}.
    *
    * @param other the tuple to copy the coordinates from. Not modified.
    */
   public Point3D(Tuple3DReadOnly other)
   {
      set(other);
   }

   /**
    * Sets this point to {@code other}.
    *
    * @param other the other point to copy the values from. Not modified.
    */
   @Override
   public void set(Point3D other)
   {
      Point3DBasics.super.set(other);
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
    * Sets the z-coordinate of this point.
    *
    * @param z the z-coordinate.
    */
   @Override
   public void setZ(double z)
   {
      this.z = z;
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
    * Returns the value of the z-coordinate of this point.
    *
    * @return the z-coordinate's value.
    */
   @Override
   public double getZ()
   {
      return z;
   }

   /**
    * Tests if the given {@code object}'s class is the same as this, in which case the method returns
    * {@link #equals(Tuple3DReadOnly)}, it returns {@code false} otherwise.
    *
    * @param object the object to compare against this. Not modified.
    * @return {@code true} if {@code object} and this are exactly equal, {@code false} otherwise.
    */
   @Override
   public boolean equals(Object object)
   {
      if (object instanceof Tuple3DReadOnly)
         return equals((Tuple3DReadOnly) object);
      else
         return false;
   }

   /**
    * Provides a {@code String} representation of this point 3D as follows: (x, y, z).
    *
    * @return the {@code String} representing this point 3D.
    */
   @Override
   public String toString()
   {
      return toString(EuclidCoreIOTools.DEFAULT_FORMAT);
   }

   /**
    * Calculates and returns a hash code value from the value of each component of this point 3D.
    *
    * @return the hash code value for this point 3D.
    */
   @Override
   public int hashCode()
   {
      return EuclidHashCodeTools.toIntHashCode(x, y, z);
   }
}
