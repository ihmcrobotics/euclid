package us.ihmc.euclid.tuple2D.interfaces;

import us.ihmc.euclid.tuple3D.interfaces.UnitVector3DReadOnly;

/**
 * Read-only interface for 2 dimensional unit-length vector.
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
public interface UnitVector2DReadOnly extends Vector2DReadOnly
{
   /** Tolerance used on this vector's components to identify if it can be normalized. */
   public static final double ZERO_TEST_EPSILON = UnitVector3DReadOnly.ZERO_TEST_EPSILON;

   /**
    * {@inheritDoc}
    * <p>
    * Before returning the value of the x-component of this unit vector, it is first normalized if
    * marked as dirty.
    * </p>
    */
   @Override
   double getX();

   /**
    * {@inheritDoc}
    * <p>
    * Before returning the value of the y-component of this unit vector, it is first normalized if
    * marked as dirty.
    * </p>
    */
   @Override
   double getY();

   /**
    * Unsafe getter to access the internal value for the x-component of this unit vector while
    * by-passing the normalization step.
    * <p>
    * This getter is destined to internal API and advanced users only.
    * </p>
    *
    * @return the unnormalized value for the x-component of this unit vector.
    */
   double getRawX();

   /**
    * Unsafe getter to access the internal value for the y-component of this unit vector while
    * by-passing the normalization step.
    * <p>
    * This getter is destined to internal API and advanced users only.
    * </p>
    *
    * @return the unnormalized value for the y-component of this unit vector.
    */
   double getRawY();

   /**
    * Returns the state of this unit vector dirty flag.
    * <p>
    * This unit vector is marked as dirty to indicate its values have been updated and not yet
    * normalized.
    * </p>
    *
    * @return the current value of the dirty flag.
    */
   boolean isDirty();

   /**
    * @return 1.0
    */
   @Override
   default double length()
   {
      return 1.0;
   }

   /**
    * @return 1.0
    */
   @Override
   default double lengthSquared()
   {
      return 1.0;
   }
}
