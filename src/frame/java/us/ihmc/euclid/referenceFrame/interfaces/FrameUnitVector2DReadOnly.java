package us.ihmc.euclid.referenceFrame.interfaces;

import us.ihmc.euclid.tuple2D.interfaces.UnitVector2DReadOnly;

/**
 * Read-only interface for 2 dimensional unit-length vector expressed in given reference frame.
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
public interface FrameUnitVector2DReadOnly extends FrameVector2DReadOnly, UnitVector2DReadOnly
{
   @Override
   default boolean geometricallyEquals(Object object, double epsilon)
   {
      return FrameVector2DReadOnly.super.geometricallyEquals(object, epsilon);
   }
}
