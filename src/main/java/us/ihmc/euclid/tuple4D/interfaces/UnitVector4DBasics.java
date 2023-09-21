package us.ihmc.euclid.tuple4D.interfaces;

import us.ihmc.euclid.Axis4D;

/**
 * Write and read interface for 3 dimensional unit-length vector.
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
public interface UnitVector4DBasics extends UnitVector4DReadOnly, Vector4DBasics
{
   /**
    * {@inheritDoc}
    * <p>
    * Invoking this setter with a new value will result in marking this unit vector as dirty.
    * </p>
    */
   @Override
   void setX(double x);

   /**
    * {@inheritDoc}
    * <p>
    * Invoking this setter with a new value will result in marking this unit vector as dirty.
    * </p>
    */
   @Override
   void setY(double y);

   /**
    * {@inheritDoc}
    * <p>
    * Invoking this setter with a new value will result in marking this unit vector as dirty.
    * </p>
    */
   @Override
   void setZ(double z);

   /**
    * {@inheritDoc}
    * <p>
    * Invoking this setter with a new value will result in marking this unit vector as dirty.
    * </p>
    */
   @Override
   void setS(double s);

   /** {@inheritDoc} */
   @Override
   default double getX()
   {
      normalize();
      return getRawX();
   }

   /** {@inheritDoc} */
   @Override
   default double getY()
   {
      normalize();
      return getRawY();
   }

   /** {@inheritDoc} */
   @Override
   default double getZ()
   {
      normalize();
      return getRawZ();
   }

   /** {@inheritDoc} */
   @Override
   default double getS()
   {
      normalize();
      return getRawS();
   }

   /**
    * Marks this unit vector as dirty.
    * <p>
    * When a unit vector is marked as dirty, the next call to either {@link #getX()}, {@link #getY()},
    * or {@link #getZ()} will perform a normalization of this unit vector.
    * </p>
    */
   public void markAsDirty();

   /** {@inheritDoc} */
   @Override
   void negate();

   /** {@inheritDoc} */
   @Override
   void absolute();

   /** {@inheritDoc} */
   @Override
   void normalize();

   /**
    * Sets this unit vector to {@code other} while keeping track of the dirty flag property.
    *
    * @param other the other unit vector. Not modified.
    */
   void set(UnitVector4DReadOnly other);

   /**
    * Sets this unit vector to {@link Axis4D#X}.
    */
   @Override
   default void setToZero()
   {
      set(Axis4D.X);
   }

   /**
    * {@inheritDoc}
    * <p>
    * If the argument implements {@link UnitVector4DReadOnly}, a redirection to
    * {@link #set(UnitVector4DReadOnly)} is done.
    * </p>
    */
   @Override
   default void set(Tuple4DReadOnly tupleReadOnly)
   {
      if (tupleReadOnly instanceof UnitVector4DReadOnly)
         set((UnitVector4DReadOnly) tupleReadOnly);
      else
         Vector4DBasics.super.set(tupleReadOnly);
   }

   /**
    * {@inheritDoc}
    * <p>
    * If the argument implements {@link UnitVector4DReadOnly}, a redirection
    * {@link #set(UnitVector4DReadOnly)} is done.
    * </p>
    */
   @Override
   default void setAndNegate(Tuple4DReadOnly tupleReadOnly)
   {
      if (tupleReadOnly instanceof UnitVector4DReadOnly)
      {
         set((UnitVector4DReadOnly) tupleReadOnly);
         negate();
      }
      else
      {
         Vector4DBasics.super.setAndNegate(tupleReadOnly);
      }
   }

   /**
    * {@inheritDoc}
    * <p>
    * If the argument implements {@link UnitVector4DReadOnly}, a redirection
    * {@link #set(UnitVector4DReadOnly)} is done.
    * </p>
    */
   @Override
   default void setAndAbsolute(Tuple4DReadOnly other)
   {
      if (other instanceof UnitVector4DReadOnly)
      {
         set((UnitVector4DReadOnly) other);
         absolute();
      }
      else
      {
         Vector4DBasics.super.setAndAbsolute(other);
      }
   }

   /**
    * {@inheritDoc}
    * <p>
    * This operation is performed without normalizing this vector.
    * </p>
    */
   @Override
   default void addX(double x)
   {
      setX(getRawX() + x);
   }

   /**
    * {@inheritDoc}
    * <p>
    * This operation is performed without normalizing this vector.
    * </p>
    */
   @Override
   default void addY(double y)
   {
      setY(getRawY() + y);
   }

   /**
    * {@inheritDoc}
    * <p>
    * This operation is performed without normalizing this vector.
    * </p>
    */
   @Override
   default void addZ(double z)
   {
      setZ(getRawZ() + z);
   }

   /**
    * {@inheritDoc}
    * <p>
    * This operation is performed without normalizing this vector.
    * </p>
    */
   @Override
   default void addS(double s)
   {
      setZ(getRawS() + s);
   }

   /**
    * {@inheritDoc}
    * <p>
    * This operation is performed without normalizing this vector.
    * </p>
    */
   @Override
   default void add(double x, double y, double z, double s)
   {
      set(getRawX() + x, getRawY() + y, getRawZ() + z, getRawS() + s);
   }

   /**
    * {@inheritDoc}
    * <p>
    * This operation is performed without normalizing this vector.
    * </p>
    */
   @Override
   default void subX(double x)
   {
      setX(getRawX() - x);
   }

   /**
    * {@inheritDoc}
    * <p>
    * This operation is performed without normalizing this vector.
    * </p>
    */
   @Override
   default void subY(double y)
   {
      setY(getRawY() - y);
   }

   /**
    * {@inheritDoc}
    * <p>
    * This operation is performed without normalizing this vector.
    * </p>
    */
   @Override
   default void subZ(double z)
   {
      setZ(getRawZ() - z);
   }

   /**
    * {@inheritDoc}
    * <p>
    * This operation is performed without normalizing this vector.
    * </p>
    */
   @Override
   default void subS(double s)
   {
      setZ(getRawS() - s);
   }

   /**
    * {@inheritDoc}
    * <p>
    * This operation is performed without normalizing this vector.
    * </p>
    */
   @Override
   default void sub(double x, double y, double z, double s)
   {
      set(getRawX() - x, getRawY() - y, getRawZ() - z, getRawS() - s);
   }

   /**
    * This method does nothing with a unit vector.
    */
   @Override
   default void scale(double scalar)
   {
   }

   /**
    * Redirection to {@link #add(Tuple4DReadOnly)} as a unit vector cannot be scaled.
    */
   @Override
   default void scaleAdd(double scalar, Tuple4DReadOnly other)
   {
      Vector4DBasics.super.add(other);
   }

   /**
    * {@inheritDoc}
    * <p>
    * This operation is performed without normalizing this vector.
    * </p>
    */
   @Override
   default void scaleAdd(double scalar, Tuple4DReadOnly tuple1, Tuple4DReadOnly tuple2)
   {
      if (tuple1 == this)
      {
         add(tuple1, tuple2);
      }
      else if (tuple2 == this)
      {
         double x = scalar * tuple1.getX() + getRawX();
         double y = scalar * tuple1.getY() + getRawY();
         double z = scalar * tuple1.getZ() + getRawZ();
         double s = scalar * tuple1.getS() + getRawS();
         set(x, y, z, s);
      }
      else
      {
         Vector4DBasics.super.scaleAdd(scalar, tuple1, tuple2);
      }
   }

   /**
    * Redirection to {@link #sub(Tuple4DReadOnly)} as a unit vector cannot be scaled.
    */
   @Override
   default void scaleSub(double scalar, Tuple4DReadOnly other)
   {
      Vector4DBasics.super.sub(other);
   }

   /**
    * {@inheritDoc}
    * <p>
    * This operation is performed without normalizing this vector.
    * </p>
    */
   @Override
   default void scaleSub(double scalar, Tuple4DReadOnly tuple1, Tuple4DReadOnly tuple2)
   {
      if (tuple1 == this)
      {
         sub(tuple1, tuple2);
      }
      else if (tuple2 == this)
      {
         double x = scalar * tuple1.getX() - getRawX();
         double y = scalar * tuple1.getY() - getRawY();
         double z = scalar * tuple1.getZ() - getRawZ();
         double s = scalar * tuple1.getS() - getRawS();
         set(x, y, z, s);
      }
      else
      {
         Vector4DBasics.super.scaleSub(scalar, tuple1, tuple2);
      }
   }
}
