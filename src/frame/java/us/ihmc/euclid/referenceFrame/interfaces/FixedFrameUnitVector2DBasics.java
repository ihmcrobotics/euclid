package us.ihmc.euclid.referenceFrame.interfaces;

import us.ihmc.euclid.interfaces.EuclidGeometry;
import us.ihmc.euclid.referenceFrame.exceptions.ReferenceFrameMismatchException;
import us.ihmc.euclid.tuple2D.interfaces.UnitVector2DBasics;
import us.ihmc.euclid.tuple2D.interfaces.UnitVector2DReadOnly;

/**
 * Write and read interface for 3 dimensional unit-length vector expressed in a constant reference
 * frame, i.e. the reference frame of this object cannot be changed via this interface.
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
public interface FixedFrameUnitVector2DBasics extends FixedFrameVector2DBasics, FrameUnitVector2DReadOnly, UnitVector2DBasics
{
   /**
    * Sets this frame unit vector to {@code other}.
    *
    * @param other the other frame unit vector to set this to. Not modified.
    * @throws ReferenceFrameMismatchException if {@code other} is not expressed in the same frame as
    *                                         {@code this}.
    */
   default void set(FrameUnitVector2DReadOnly other)
   {
      checkReferenceFrameMatch(other);
      set((UnitVector2DReadOnly) other);
   }
   
   @Override
   default boolean geometricallyEquals(EuclidGeometry geometry, double epsilon)
   {
      return FixedFrameVector2DBasics.super.geometricallyEquals(geometry, epsilon);
   }
}
