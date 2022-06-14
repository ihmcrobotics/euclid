package us.ihmc.euclid.referenceFrame.interfaces;

import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.tools.EuclidFrameIOTools;
import us.ihmc.euclid.tuple2D.interfaces.Tuple2DReadOnly;

/**
 * Read-only interface for a 2D tuple expressed in a given reference frame.
 * <p>
 * In addition to representing a {@link Tuple2DReadOnly}, a {@link ReferenceFrame} is associated to
 * a {@code FrameTuple2DReadOnly}. This allows, for instance, to enforce, at runtime, that
 * operations on tuples occur in the same coordinate system.
 * </p>
 * <p>
 * Because a {@code FrameTuple2DReadOnly} extends {@code Tuple2DReadOnly}, it is compatible with
 * methods only requiring {@code Tuple2DReadOnly}. However, these methods do NOT assert that the
 * operation occur in the proper coordinate system. Use this feature carefully and always prefer
 * using methods requiring {@code FrameTuple2DReadOnly}.
 * </p>
 */
public interface FrameTuple2DReadOnly extends Tuple2DReadOnly, EuclidFrameGeometry
{
   /**
    * Provides a {@code String} representation of this tuple2D. as follows: (x, y)-referenceFrame.
    *
    * @return a string representation of this tuple2D.
    */
   @Override
   default String toString(String format)
   {
      return EuclidFrameIOTools.getFrameTuple2DString(format, this);
   }
}
