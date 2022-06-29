package us.ihmc.euclid.tuple2D;

import us.ihmc.euclid.Axis2D;
import us.ihmc.euclid.tools.EuclidCoreIOTools;
import us.ihmc.euclid.tools.EuclidCoreTools;
import us.ihmc.euclid.tools.EuclidHashCodeTools;
import us.ihmc.euclid.tuple2D.interfaces.Tuple2DReadOnly;
import us.ihmc.euclid.tuple2D.interfaces.UnitVector2DBasics;
import us.ihmc.euclid.tuple2D.interfaces.UnitVector2DReadOnly;
import us.ihmc.euclid.tuple3D.UnitVector3D;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DReadOnly;

/**
 * Implementation for a 2 dimensional unit-length vector.
 * <p>
 * This unit vector shares the same API as a regular vector 2D while ensuring it is normalized when
 * accessing directly or indirectly its individual components, i.e. when invoking either
 * {@link #getX()} or {@link #getY()}.
 * </p>
 * <p>
 * When the values of this vector are set to zero, the next time it is normalized it will be reset
 * to (1.0, 0.0).
 * </p>
 *
 * @author Sylvain Bertrand
 */
public class UnitVector2D implements UnitVector2DBasics
{
   /** Tolerance used on this vector's components to identify if it can be normalized. */
   public static final double ZERO_TEST_EPSILON = UnitVector3D.ZERO_TEST_EPSILON;

   /** The dirty flag for this unit vector indicating whether it needs to be normalized or not. */
   private boolean dirty = true;
   /** The x-component. */
   private double x;
   /** The y-component. */
   private double y;

   /**
    * Creates a new unit vector and initializes it to {@link Axis2D#X}.
    */
   public UnitVector2D()
   {
      setToZero();
   }

   /**
    * Creates a new unit vector and initializes it with the given components.
    *
    * @param x the x-component.
    * @param y the y-component.
    */
   public UnitVector2D(double x, double y)
   {
      set(x, y);
   }

   /**
    * Creates a new unit vector and initializes its component {@code x}, {@code y} in order from the
    * given array.
    *
    * @param vectorArray the array containing this vector's components. Not modified.
    */
   public UnitVector2D(double[] vectorArray)
   {
      set(vectorArray);
   }

   /**
    * Creates a new unit vector and initializes its x and y components to {@code tuple3DReadOnly}.
    *
    * @param tuple3DReadOnly the tuple to copy the coordinates from. Not modified.
    */
   public UnitVector2D(Tuple3DReadOnly tuple3DReadOnly)
   {
      set(tuple3DReadOnly);
   }

   /**
    * Creates a new unit vector and initializes it to {@code other}.
    *
    * @param other the tuple to copy the components from. Not modified.
    */
   public UnitVector2D(Tuple2DReadOnly other)
   {
      set(other);
   }

   /** {@inheritDoc} */
   @Override
   public void absolute()
   {
      x = Math.abs(x);
      y = Math.abs(y);
   }

   /** {@inheritDoc} */
   @Override
   public void negate()
   {
      x = -x;
      y = -y;
   }

   /** {@inheritDoc} */
   @Override
   public void normalize()
   {
      if (dirty)
      {
         if (EuclidCoreTools.areAllZero(x, y, ZERO_TEST_EPSILON))
         {
            setToZero();
         }
         else
         {
            double lengthInverse = 1.0 / EuclidCoreTools.fastNorm(x, y);
            x *= lengthInverse;
            y *= lengthInverse;
         }
         dirty = false;
      }
   }

   /** {@inheritDoc} */
   @Override
   public void markAsDirty()
   {
      dirty = true;
   }

   /** {@inheritDoc} */
   @Override
   public boolean isDirty()
   {
      return dirty;
   }

   /** {@inheritDoc} */
   @Override
   public void set(UnitVector2DReadOnly other)
   {
      x = other.getRawX();
      y = other.getRawY();
      dirty = other.isDirty();
   }

   /** {@inheritDoc} */
   @Override
   public void setX(double x)
   {
      if (this.x != x)
      {
         this.x = x;
         markAsDirty();
      }
   }

   /** {@inheritDoc} */
   @Override
   public void setY(double y)
   {
      if (this.y != y)
      {
         this.y = y;
         markAsDirty();
      }
   }

   /** {@inheritDoc} */
   @Override
   public double getRawX()
   {
      return x;
   }

   /** {@inheritDoc} */
   @Override
   public double getRawY()
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
         return UnitVector2DBasics.super.equals((Tuple2DReadOnly) object);
      else
         return false;
   }

   /**
    * Provides a {@code String} representation of this unit vector 2D as follows: (x, y).
    *
    * @return the {@code String} representing this unit vector 2D.
    */
   @Override
   public String toString()
   {
      return toString(EuclidCoreIOTools.DEFAULT_FORMAT);
   }

   /**
    * Calculates and returns a hash code value from the value of each component of this unit vector 2D.
    *
    * @return the hash code value for this unit vector 2D.
    */
   @Override
   public int hashCode()
   {
      return EuclidHashCodeTools.toIntHashCode(getX(), getY());
   }
}
