package us.ihmc.euclid.referenceFrame.interfaces;

import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.exceptions.ReferenceFrameMismatchException;
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
    * Computes and returns the distance from this quaternion to {@code other}.
    *
    * @param other the other quaternion to measure the distance. Not modified.
    * @return the angle representing the distance between the two quaternions. It is contained in [0,
    *         2<i>pi</i>]
    * @throws ReferenceFrameMismatchException if reference frame of {@code this} and {@code other} do
    *                                         not match.
    */
   default double distance(FrameQuaternionReadOnly other)
   {
      checkReferenceFrameMatch(other);
      return QuaternionReadOnly.super.distance(other);
   }

   /**
    * Compares {@code this} to {@code other} to determine if the two frame quaternions are
    * geometrically similar, i.e. the magnitude of their difference is less than or equal to
    * {@code epsilon}.
    *
    * @param other   the frame quaternion to compare to. Not modified.
    * @param epsilon the tolerance of the comparison.
    * @return {@code true} if the two frame quaternions represent the same geometry, {@code false}
    *         otherwise.
    * @throws ReferenceFrameMismatchException if {@code other} is not expressed in the same reference
    *                                         frame as {@code this}.
    */
   default boolean geometricallyEquals(FrameQuaternionReadOnly other, double epsilon)
   {
      checkReferenceFrameMatch(other);
      return QuaternionReadOnly.super.geometricallyEquals(other, epsilon);
   }
}
