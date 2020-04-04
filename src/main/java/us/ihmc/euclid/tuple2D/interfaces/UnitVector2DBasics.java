package us.ihmc.euclid.tuple2D.interfaces;

import us.ihmc.euclid.Axis2D;

/**
 * Write and read interface for 2 dimensional unit-length vector.
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
public interface UnitVector2DBasics extends UnitVector2DReadOnly, Vector2DBasics
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

   /**
    * Marks this unit vector as dirty.
    * <p>
    * When a unit vector is marked as dirty, the next call to either {@link #getX()} or {@link #getY()}
    * will perform a normalization of this unit vector.
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
   void set(UnitVector2DReadOnly other);

   /**
    * Sets this unit vector to (1.0, 0.0).
    */
   @Override
   default void setToZero()
   {
      set(Axis2D.X);
   }

   /**
    * {@inheritDoc}
    * <p>
    * If the argument implements {@link UnitVector2DReadOnly}, a redirection to
    * {@link #set(UnitVector2DReadOnly)} is done.
    * </p>
    */
   @Override
   default void set(Tuple2DReadOnly tupleReadOnly)
   {
      if (tupleReadOnly instanceof UnitVector2DReadOnly)
         set((UnitVector2DReadOnly) tupleReadOnly);
      else
         Vector2DBasics.super.set(tupleReadOnly);
   }

   /**
    * {@inheritDoc}
    * <p>
    * If the argument implements {@link UnitVector2DReadOnly}, a redirection
    * {@link #set(UnitVector2DReadOnly)} is done.
    * </p>
    */
   @Override
   default void setAndNegate(Tuple2DReadOnly tupleReadOnly)
   {
      if (tupleReadOnly instanceof UnitVector2DReadOnly)
      {
         set((UnitVector2DReadOnly) tupleReadOnly);
         negate();
      }
      else
      {
         Vector2DBasics.super.setAndNegate(tupleReadOnly);
      }
   }

   /**
    * {@inheritDoc}
    * <p>
    * If the argument implements {@link UnitVector2DReadOnly}, a redirection
    * {@link #set(UnitVector2DReadOnly)} is done.
    * </p>
    */
   @Override
   default void setAndAbsolute(Tuple2DReadOnly other)
   {
      if (other instanceof UnitVector2DReadOnly)
      {
         set((UnitVector2DReadOnly) other);
         absolute();
      }
      else
      {
         Vector2DBasics.super.setAndAbsolute(other);
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
   default void add(double x, double y)
   {
      set(getRawX() + x, getRawY() + y);
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
   default void sub(double x, double y)
   {
      set(getRawX() - x, getRawY() - y);
   }

   /**
    * This method does nothing with a unit vector.
    */
   @Override
   default void scale(double scalar)
   {
   }

   /**
    * Redirection to {@link #add(Tuple2DReadOnly)} as a unit vector cannot be scaled.
    */
   @Override
   default void scaleAdd(double scalar, Tuple2DReadOnly other)
   {
      Vector2DBasics.super.add(other);
   }

   /**
    * {@inheritDoc}
    * <p>
    * This operation is performed without normalizing this vector.
    * </p>
    */
   @Override
   default void scaleAdd(double scalar, Tuple2DReadOnly tuple1, Tuple2DReadOnly tuple2)
   {
      if (tuple1 == this)
      {
         add(tuple1, tuple2);
      }
      else if (tuple2 == this)
      {
         double x = scalar * tuple1.getX() + getRawX();
         double y = scalar * tuple1.getY() + getRawY();
         set(x, y);
      }
      else
      {
         Vector2DBasics.super.scaleAdd(scalar, tuple1, tuple2);
      }
   }

   /**
    * Redirection to {@link #sub(Tuple2DReadOnly)} as a unit vector cannot be scaled.
    */
   @Override
   default void scaleSub(double scalar, Tuple2DReadOnly other)
   {
      Vector2DBasics.super.sub(other);
   }

   /**
    * {@inheritDoc}
    * <p>
    * This operation is performed without normalizing this vector.
    * </p>
    */
   @Override
   default void scaleSub(double scalar, Tuple2DReadOnly tuple1, Tuple2DReadOnly tuple2)
   {
      if (tuple1 == this)
      {
         sub(tuple1, tuple2);
      }
      else if (tuple2 == this)
      {
         double x = scalar * tuple1.getX() - getRawX();
         double y = scalar * tuple1.getY() - getRawY();
         set(x, y);
      }
      else
      {
         Vector2DBasics.super.scaleSub(scalar, tuple1, tuple2);
      }
   }

   /**
    * This method does nothing with a unit vector as a unit vector cannot be scaled.
    *
    * @return always {@code false};
    */
   @Override
   default boolean clipToMaxLength(double maxLength)
   {
      return false;
   }
}
