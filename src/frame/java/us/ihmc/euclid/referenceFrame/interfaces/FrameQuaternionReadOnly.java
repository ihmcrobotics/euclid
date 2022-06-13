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

}
