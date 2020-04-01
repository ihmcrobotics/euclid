package us.ihmc.euclid.referenceFrame.interfaces;

import us.ihmc.euclid.shape.primitives.interfaces.PointShape3DBasics;

/**
 * Read and write interface for a point shape 3D expressed in a constant reference frame, i.e. the
 * reference frame of this object cannot be changed via this interface.
 * <p>
 * A point shape 3D is represented by its position.
 * </p>
 *
 * @author Sylvain Bertrand
 */
public interface FixedFramePointShape3DBasics extends PointShape3DBasics, FramePointShape3DReadOnly, FixedFrameShape3DBasics, FixedFramePoint3DBasics
{
   /**
    * Returns {@code null} as this shape is not defined by a pose.
    */
   @Override
   default FixedFrameShape3DPoseBasics getPose()
   {
      return null;
   }
}
