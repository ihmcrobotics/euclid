package us.ihmc.euclid.tuple2D;

import us.ihmc.euclid.interfaces.EuclidGeometry;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.tools.EuclidCoreIOTools;
import us.ihmc.euclid.tools.EuclidHashCodeTools;
import us.ihmc.euclid.tuple2D.interfaces.Tuple2DReadOnly;
import us.ihmc.euclid.tuple2D.interfaces.Vector2DBasics;

/**
 * A 2D vector represents a physical quantity with a magnitude and a direction in the XY-plane. For
 * instance, it can be used to represent a 2D velocity, force, or translation from one 2D point to
 * another.
 * <p>
 * This version of 2D vector uses single precision fields to save the value of each component. It is
 * meant for garbage free usage and for situations where heap memory is limited. When memory is not
 * a constraint, the use of {@link Vector2D} is preferable.
 * </p>
 *
 * @author Sylvain Bertrand
 */
public class Vector2D32 implements Vector2DBasics, Settable<Vector2D32>
{
   /** The x-component. */
   private float x;
   /** The y-component. */
   private float y;

   /**
    * Creates a new vector and initializes it components to zero.
    */
   public Vector2D32()
   {
      setToZero();
   }

   /**
    * Creates a new vector and initializes it with the given components.
    *
    * @param x the x-component.
    * @param y the y-component.
    */
   public Vector2D32(float x, float y)
   {
      set(x, y);
   }

   /**
    * Creates a new vector and initializes its component {@code x}, {@code y} in order from the given
    * array.
    *
    * @param vectorArray the array containing this vector's components. Not modified.
    */
   public Vector2D32(float[] vectorArray)
   {
      set(vectorArray);
   }

   /**
    * Creates a new vector and initializes it to {@code other}.
    *
    * @param other the tuple to copy the components from. Not modified.
    */
   public Vector2D32(Tuple2DReadOnly other)
   {
      set(other);
   }

   /**
    * Sets this vector to {@code other}.
    *
    * @param other the other vector to copy the values from. Not modified.
    */
   @Override
   public void set(Vector2D32 other)
   {
      Vector2DBasics.super.set(other);
   }

   /**
    * Sets the x-component of this vector.
    *
    * @param x the x-component.
    */
   @Override
   public void setX(double x)
   {
      this.x = (float) x;
   }

   /**
    * Sets the y-component of this vector.
    *
    * @param y the y-component.
    */
   @Override
   public void setY(double y)
   {
      this.y = (float) y;
   }

   /**
    * Sets the x-component of this vector.
    *
    * @param x the x-component.
    */
   public void setX(float x)
   {
      this.x = x;
   }

   /**
    * Sets the y-component of this vector.
    *
    * @param y the y-component.
    */
   public void setY(float y)
   {
      this.y = y;
   }

   /**
    * Returns the value of the x-component of this vector.
    *
    * @return the x-component's value.
    */
   @Override
   public double getX()
   {
      return x;
   }

   /**
    * Returns the value of the y-component of this vector.
    *
    * @return the y-component's value.
    */
   @Override
   public double getY()
   {
      return y;
   }

   /**
    * Returns the value of the x-component of this vector.
    *
    * @return the x-component's value.
    */
   @Override
   public float getX32()
   {
      return x;
   }

   /**
    * Returns the value of the y-component of this vector.
    *
    * @return the y-component's value.
    */
   @Override
   public float getY32()
   {
      return y;
   }

   /**
    * Tests if the given {@code object}'s class is the same as this, in which case the method returns
    * {@link #equals(EuclidGeometry)}, it returns {@code false} otherwise.
    *
    * @param object the object to compare against this. Not modified.
    * @return {@code true} if {@code object} and this are exactly equal, {@code false} otherwise.
    */
   @Override
   public boolean equals(Object object)
   {
      if (object instanceof Tuple2DReadOnly)
         return equals((EuclidGeometry) object);
      else
         return false;
   }

   /**
    * Provides a {@code String} representation of this vector 2D as follows: (x, y).
    *
    * @return the {@code String} representing this vector 2D.
    */
   @Override
   public String toString()
   {
      return toString(EuclidCoreIOTools.DEFAULT_FORMAT);
   }

   /**
    * Calculates and returns a hash code value from the value of each component of this vector 2D.
    *
    * @return the hash code value for this vector 2D.
    */
   @Override
   public int hashCode()
   {
      return EuclidHashCodeTools.toIntHashCode(x, y);
   }
}
