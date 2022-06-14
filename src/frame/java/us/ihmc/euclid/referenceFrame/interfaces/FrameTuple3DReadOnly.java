package us.ihmc.euclid.referenceFrame.interfaces;

import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.tools.EuclidFrameIOTools;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DReadOnly;

/**
 * Read-only interface for a 3D tuple expressed in a given reference frame.
 * <p>
 * In addition to representing a {@link Tuple3DReadOnly}, a {@link ReferenceFrame} is associated to
 * a {@code FrameTuple3DReadOnly}. This allows, for instance, to enforce, at runtime, that
 * operations on tuples occur in the same coordinate system.
 * </p>
 * <p>
 * Because a {@code FrameTuple3DReadOnly} extends {@code Tuple3DReadOnly}, it is compatible with
 * methods only requiring {@code Tuple3DReadOnly}. However, these methods do NOT assert that the
 * operation occur in the proper coordinate system. Use this feature carefully and always prefer
 * using methods requiring {@code FrameTuple3DReadOnly}.
 * </p>
 */
public interface FrameTuple3DReadOnly extends Tuple3DReadOnly, EuclidFrameGeometry
{
   @Override
   default String toString(String format)
   {
      return EuclidFrameIOTools.getFrameTuple3DString(format, this);
   }
}
