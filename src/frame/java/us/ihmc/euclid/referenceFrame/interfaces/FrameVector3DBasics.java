package us.ihmc.euclid.referenceFrame.interfaces;

import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DBasics;

/**
 * Write and read interface for a 3D vector expressed in a changeable reference frame, i.e. the
 * reference frame in which this vector is expressed can be changed.
 * <p>
 * In addition to representing a {@link Vector3DBasics}, a {@link ReferenceFrame} is associated to a
 * {@code FrameVector3DBasics}. This allows, for instance, to enforce, at runtime, that operations
 * on vectors occur in the same coordinate system.
 * </p>
 * <p>
 * Because a {@code FrameVector3DBasics} extends {@code Vector3DBasics}, it is compatible with
 * methods only requiring {@code Vector3DBasics}. However, these methods do NOT assert that the
 * operation occur in the proper coordinate system. Use this feature carefully and always prefer
 * using methods requiring {@code FrameVector3DBasics}.
 * </p>
 */
public interface FrameVector3DBasics extends FixedFrameVector3DBasics, FrameTuple3DBasics, FrameChangeable
{

}
