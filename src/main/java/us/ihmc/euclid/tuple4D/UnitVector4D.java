package us.ihmc.euclid.tuple4D;

import us.ihmc.euclid.Axis4D;
import us.ihmc.euclid.interfaces.EuclidGeometry;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.tools.EuclidCoreIOTools;
import us.ihmc.euclid.tools.EuclidCoreTools;
import us.ihmc.euclid.tools.EuclidHashCodeTools;
import us.ihmc.euclid.tuple4D.interfaces.Tuple4DReadOnly;
import us.ihmc.euclid.tuple4D.interfaces.UnitVector4DBasics;
import us.ihmc.euclid.tuple4D.interfaces.UnitVector4DReadOnly;

/**
 * Implementation for a 3 dimensional unit-length vector.
 * <p>
 * This unit vector shares the same API as a regular vector 4D while ensuring it is normalized when
 * accessing directly or indirectly its individual components, i.e. when invoking either
 * {@link #getX()}, {@link #getY()}, {@link #getZ()}, or {@link #getS()}.
 * </p>
 * <p>
 * When the values of this vector are set to zero, the next time it is normalized it will be reset
 * to (1.0, 0.0, 0.0, 0.0).
 * </p>
 *
 * @author Sylvain Bertrand
 */
public class UnitVector4D implements UnitVector4DBasics, Settable<UnitVector4D>
{
   /** Tolerance used on this vector's components to identify if it can be normalized. */
   public static final double ZERO_TEST_EPSILON = 1.0e-16;

   /** The dirty flag for this unit vector indicating whether it needs to be normalized or not. */
   private boolean dirty = true;
   /** The x-component. */
   private double x;
   /** The y-component. */
   private double y;
   /** The z-component. */
   private double z;
   /** The s-component. */
   private double s;

   /**
    * Creates a new unit vector and initializes it to {@link Axis4D#X}.
    */
   public UnitVector4D()
   {
      setToZero();
   }

   /**
    * Creates a new unit vector and initializes it with the given components.
    *
    * @param x the x-component.
    * @param y the y-component.
    * @param z the z-component.
    * @param s the s-component.
    */
   public UnitVector4D(double x, double y, double z, double s)
   {
      set(x, y, z, s);
   }

   /**
    * Creates a new unit vector and initializes its component {@code x}, {@code y}, {@code z},
    * {@code s} in order from the given array.
    *
    * @param vectorArray the array containing this vector's components. Not modified.
    */
   public UnitVector4D(double[] vectorArray)
   {
      set(vectorArray);
   }

   /**
    * Creates a new unit vector and initializes it to {@code other}.
    *
    * @param other the tuple to copy the components from. Not modified.
    */
   public UnitVector4D(Tuple4DReadOnly other)
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
      s = Math.abs(s);
   }

   /** {@inheritDoc} */
   @Override
   public void negate()
   {
      x = -x;
      y = -y;
      z = -z;
      s = -s;
   }

   /** {@inheritDoc} */
   @Override
   public void normalize()
   {
      if (dirty)
      {
         if (EuclidCoreTools.areAllZero(x, y, z, s, ZERO_TEST_EPSILON))
         {
            setToZero();
         }
         else
         {
            double lengthInverse = 1.0 / EuclidCoreTools.fastNorm(x, y, z, s);
            x *= lengthInverse;
            y *= lengthInverse;
            z *= lengthInverse;
            s *= lengthInverse;
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
   public void set(UnitVector4D other)
   {
      set((UnitVector4DReadOnly) other);
   }

   /** {@inheritDoc} */
   @Override
   public void set(UnitVector4DReadOnly other)
   {
      x = other.getRawX();
      y = other.getRawY();
      z = other.getRawZ();
      s = other.getRawS();
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
   public void setZ(double z)
   {
      if (this.z != z)
      {
         this.z = z;
         markAsDirty();
      }
   }

   /** {@inheritDoc} */
   @Override
   public void setS(double s)
   {
      if (this.s != s)
      {
         this.s = s;
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

   /** {@inheritDoc} */
   @Override
   public double getRawS()
   {
      return s;
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
      if (object instanceof Tuple4DReadOnly)
         return equals((EuclidGeometry) object);
      else
         return false;
   }

   /**
    * Provides a {@code String} representation of this unit vector 4D as follows: (x, y, z).
    *
    * @return the {@code String} representing this unit vector 4D.
    */
   @Override
   public String toString()
   {
      return toString(EuclidCoreIOTools.DEFAULT_FORMAT);
   }

   /**
    * Calculates and returns a hash code value from the value of each component of this unit vector 4D.
    *
    * @return the hash code value for this unit vector 4D.
    */
   @Override
   public int hashCode()
   {
      return EuclidHashCodeTools.toIntHashCode(getX(), getY(), getZ(), getS());
   }
}
