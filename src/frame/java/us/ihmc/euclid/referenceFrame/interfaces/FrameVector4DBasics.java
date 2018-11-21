package us.ihmc.euclid.referenceFrame.interfaces;

import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple4D.interfaces.Vector4DBasics;

/**
 * Write and read interface for a 4D vector expressed in a changeable reference frame, i.e. the
 * reference frame in which this vector is expressed can be changed.
 * <p>
 * In addition to representing a {@link Vector4DBasics}, a {@link ReferenceFrame} is associated to a
 * {@code FrameVector4DBasics}. This allows, for instance, to enforce, at runtime, that operations
 * on vectors occur in the same coordinate system.
 * </p>
 * <p>
 * Because a {@code FrameVector4DBasics} extends {@code Vector4DBasics}, it is compatible with
 * methods only requiring {@code Vector4DBasics}. However, these methods do NOT assert that the
 * operation occur in the proper coordinate system. Use this feature carefully and always prefer
 * using methods requiring {@code FrameVector4DBasics}.
 * </p>
 */
public interface FrameVector4DBasics extends FixedFrameVector4DBasics, FrameTuple4DBasics, FrameChangeable
{

}
