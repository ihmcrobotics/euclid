package us.ihmc.euclid.referenceFrame.interfaces;

import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple4D.interfaces.QuaternionReadOnly;

/**
 * Read-only interface for a quaternion expressed in a given reference frame.
 * <p>
 * In addition to representing a {@link QuaternionReadOnly}, a {@link ReferenceFrame} is associated
 * to a {@code FrameQuaternionReadOnly}. This allows, for instance, to enforce, at runtime, that
 * operations on quaternions occur in the same coordinate system. Also, via the method
 * {@link FrameChangeable#changeFrame(ReferenceFrame)}, one can easily calculates the value of a
 * quaternion in different reference frames.
 * </p>
 * <p>
 * Because a {@code FrameQuaternionReadOnly} extends {@code QuaternionReadOnly}, it is compatible
 * with methods only requiring {@code QuaternionReadOnly}. However, these methods do NOT assert that
 * the operation occur in the proper coordinate system. Use this feature carefully and always prefer
 * using methods requiring {@code FrameQuaternionReadOnly}.
 * </p>
 */
public interface FrameQuaternionReadOnly extends FrameTuple4DReadOnly, FrameOrientation3DReadOnly, QuaternionReadOnly
{
   /**
    * Tests if {@code this} and {@code other} represent the same orientation to an {@code epsilon}.
    * <p>
    * Note that {@code this.geometricallyEquals(other, epsilon) == true} does not necessarily imply
    * that the 2 orientations are of the same type nor that they are equal on a per-component bases.
    * Returns false if the reference frame does not match.
    * </p>
    *
    * @param object  the object to compare against this. Not modified.
    * @param epsilon the maximum angle for the two orientations to be considered equal.
    * @return {@code true} if the two orientations represent the same geometry, {@code false}
    *         otherwise.
    */
   @Override
   default boolean geometricallyEquals(Object object, double epsilon)
   {
      return FrameOrientation3DReadOnly.super.geometricallyEquals(object, epsilon);
   }
}
