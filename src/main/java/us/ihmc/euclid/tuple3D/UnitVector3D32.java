package us.ihmc.euclid.tuple3D;

import us.ihmc.euclid.tools.EuclidCoreIOTools;
import us.ihmc.euclid.tools.EuclidCoreTools;
import us.ihmc.euclid.tools.EuclidHashCodeTools;
import us.ihmc.euclid.tuple2D.interfaces.Tuple2DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.UnitVector3DBasics;
import us.ihmc.euclid.tuple3D.interfaces.UnitVector3DReadOnly;

/**
 * Implementation for a 3 dimensional unit-length vector.
 * <p>
 * This unit vector shares the same API as a regular vector 3D while ensuring it is normalized when
 * accessing directly or indirectly its individual components, i.e. when invoking either
 * {@link #getX()}, {@link #getY()}, or {@link #getZ()}.
 * </p>
 * <p>
 * When the values of this vector are set to zero, the next time it is normalized it will be reset
 * to (1.0, 0.0, 0.0).
 * </p>
 * <p>
 * This version of 3D vector uses single precision fields to save the value of each component.
 * </p>
 *
 * @author Sylvain Bertrand
 */
public class UnitVector3D32 implements UnitVector3DBasics
{
   /** Tolerance used on this vector's components to identify if it can be normalized. */
   public static final double ZERO_TEST_EPSILON = 1.0e-16;

   /** The dirty flag for this unit vector indicating whether it needs to be normalized or not. */
   private boolean dirty = true;
   /** The x-component. */
   private float x;
   /** The y-component. */
   private float y;
   /** The z-component. */
   private float z;

   /**
    * Creates a new unit vector and initializes it components to zero.
    */
   public UnitVector3D32()
   {
      setToZero();
   }

   /**
    * Creates a new unit vector and initializes it with the given components.
    *
    * @param x the x-component.
    * @param y the y-component.
    * @param z the z-component.
    */
   public UnitVector3D32(float x, float y, float z)
   {
      set(x, y, z);
   }

   /**
    * Creates a new unit vector and initializes its component {@code x}, {@code y}, {@code z} in order
    * from the given array.
    *
    * @param vectorArray the array containing this vector's components. Not modified.
    */
   public UnitVector3D32(float[] vectorArray)
   {
      set(vectorArray);
   }

   /**
    * Creates a new unit vector and initializes its x and y components to {@code tuple2DReadOnly}.
    *
    * @param tuple2DReadOnly the tuple to copy the coordinates from. Not modified.
    */
   public UnitVector3D32(Tuple2DReadOnly tuple2DReadOnly)
   {
      set(tuple2DReadOnly);
   }

   /**
    * Creates a new unit vector and initializes it to {@code other}.
    *
    * @param other the tuple to copy the components from. Not modified.
    */
   public UnitVector3D32(Tuple3DReadOnly other)
   {
      set(other);
   }

   /** {@inheritDoc} */
   @Override
   public void absolute()
   {
      x = Math.abs(x);
      y = Math.abs(y);
      z = Math.abs(z);
   }

   /** {@inheritDoc} */
   @Override
   public void negate()
   {
      x = -x;
      y = -y;
      z = -z;
   }

   /** {@inheritDoc} */
   @Override
   public void normalize()
   {
      if (dirty)
      {
         if (EuclidCoreTools.areAllZero(x, y, z, ZERO_TEST_EPSILON))
         {
            setToZero();
         }
         else
         {
            float lengthInverse = (float) (1.0 / EuclidCoreTools.fastNorm(x, y, z));
            x *= lengthInverse;
            y *= lengthInverse;
            z *= lengthInverse;
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
   public void set(UnitVector3DReadOnly other)
   {
      x = (float) other.getRawX();
      y = (float) other.getRawY();
      z = (float) other.getRawZ();
      dirty = other.isDirty();
   }

   /** {@inheritDoc} */
   @Override
   public void setX(double x)
   {
      if (this.x != x)
      {
         this.x = (float) x;
         markAsDirty();
      }
   }

   /** {@inheritDoc} */
   @Override
   public void setY(double y)
   {
      if (this.y != y)
      {
         this.y = (float) y;
         markAsDirty();
      }
   }

   /** {@inheritDoc} */
   @Override
   public void setZ(double z)
   {
      if (this.z != z)
      {
         this.z = (float) z;
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

   /** {@inheritDoc} */
   @Override
   public double getRawZ()
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
         return UnitVector3DBasics.super.equals((Tuple3DReadOnly) object);
      else
         return false;
   }

   /**
    * Provides a {@code String} representation of this unit vector 3D as follows: (x, y, z).
    *
    * @return the {@code String} representing this unit vector 3D.
    */
   @Override
   public String toString()
   {
      return toString(EuclidCoreIOTools.DEFAULT_FORMAT);
   }

   /**
    * Calculates and returns a hash code value from the value of each component of this unit vector 3D.
    *
    * @return the hash code value for this unit vector 3D.
    */
   @Override
   public int hashCode()
   {
      return EuclidHashCodeTools.toIntHashCode(x, y, z);
   }
}
