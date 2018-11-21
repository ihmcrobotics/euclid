package us.ihmc.euclid.referenceFrame.interfaces;

import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple4D.interfaces.QuaternionBasics;
import us.ihmc.euclid.tuple4D.interfaces.QuaternionReadOnly;

/**
 * Write and read interface for a quaternion expressed in a changeable reference frame, i.e. the
 * reference frame in which this quaternion is expressed can be changed.
 * <p>
 * In addition to representing a {@link QuaternionBasics}, a {@link ReferenceFrame} is associated to
 * a {@code FrameQuaternionBasics}. This allows, for instance, to enforce, at runtime, that
 * operations on quaternions occur in the same coordinate system. Also, via the method
 * {@link #changeFrame(ReferenceFrame)}, one can easily calculates the value of a quaternion in
 * different reference frames.
 * </p>
 * <p>
 * Because a {@code FrameQuaternionBasics} extends {@code QuaternionBasics}, it is compatible with
 * methods only requiring {@code QuaternionBasics}. However, these methods do NOT assert that the
 * operation occur in the proper coordinate system. Use this feature carefully and always prefer
 * using methods requiring {@code FrameQuaternionBasics}.
 * </p>
 */
public interface FrameQuaternionBasics extends FixedFrameQuaternionBasics, FrameTuple4DBasics, FrameOrientation3DBasics, FrameChangeable
{
   /**
    * Sets this frame tuple to {@code quaternionReadOnly} and sets its current frame to
    * {@code referenceFrame}.
    *
    * @param referenceFrame the new reference frame for this frame tuple.
    * @param quaternionReadOnly the quaternion to copy the values from. Not modified.
    */
   default void setIncludingFrame(ReferenceFrame referenceFrame, QuaternionReadOnly quaternionReadOnly)
   {
      setReferenceFrame(referenceFrame);
      set(quaternionReadOnly);
   }

   /**
    * Sets this frame tuple to {@code other}.
    *
    * @param other the other frame tuple to copy the values and reference frame from. Not modified.
    */
   default void setIncludingFrame(FrameQuaternionReadOnly other)
   {
      setIncludingFrame(other.getReferenceFrame(), other);
   }
}
