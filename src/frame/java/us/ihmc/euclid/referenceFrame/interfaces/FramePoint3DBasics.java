package us.ihmc.euclid.referenceFrame.interfaces;

import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple3D.interfaces.Point3DBasics;

/**
 * Write and read interface for a 3D point expressed in a changeable reference frame, i.e. the
 * reference frame in which this point is expressed can be changed.
 * <p>
 * In addition to representing a {@link Point3DBasics}, a {@link ReferenceFrame} is associated to a
 * {@code FramePoint3DBasics}. This allows, for instance, to enforce, at runtime, that operations on
 * points occur in the same coordinate system.
 * </p>
 * <p>
 * Because a {@code FramePoint3DBasics} extends {@code Point3DBasics}, it is compatible with methods
 * only requiring {@code Point3DBasics}. However, these methods do NOT assert that the operation
 * occur in the proper coordinate system. Use this feature carefully and always prefer using methods
 * requiring {@code FramePoint3DBasics}.
 * </p>
 */
public interface FramePoint3DBasics extends FixedFramePoint3DBasics, FrameTuple3DBasics, FrameChangeable
{

}
