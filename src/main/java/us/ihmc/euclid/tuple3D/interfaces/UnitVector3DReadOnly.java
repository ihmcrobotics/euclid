package us.ihmc.euclid.tuple3D.interfaces;

/**
 * Read-only interface for 3 dimensional unit-length vector.
 * <p>
 * This unit vector shares the same API as a regular vector 3D while ensuring it is normalized when
 * accessing directly or indirectly its individual components, i.e. when invoking either
 * {@link #getX()}, {@link #getY()}, or {@link #getZ()}.
 * </p>
 * <p>
 * When the values of this vector are set to zero, the next time it is normalized it will be reset
 * to (1.0, 0.0, 0.0).
 * </p>
 *
 * @author Sylvain Bertrand
 */
public interface UnitVector3DReadOnly extends Vector3DReadOnly
{
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
    * {@inheritDoc}
    * <p>
    * Before returning the value of the z-component of this unit vector, it is first normalized if
    * marked as dirty.
    * </p>
    */
   @Override
   double getZ();

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
    * Unsafe getter to access the internal value for the z-component of this unit vector while
    * by-passing the normalization step.
    * <p>
    * This getter is destined to internal API and advanced users only.
    * </p>
    *
    * @return the unnormalized value for the z-component of this unit vector.
    */
   double getRawZ();

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
   default double norm()
   {
      return 1.0;
   }

   /**
    * @return 1.0
    */
   @Override
   default double normSquared()
   {
      return 1.0;
   }
}
