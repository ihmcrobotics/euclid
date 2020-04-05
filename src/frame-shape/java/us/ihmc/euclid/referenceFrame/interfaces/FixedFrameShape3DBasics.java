package us.ihmc.euclid.referenceFrame.interfaces;

import us.ihmc.euclid.shape.primitives.interfaces.Shape3DBasics;

/**
 * Read and write interface for representing a 3D shape expressed in a constant reference frame,
 * i.e. the reference frame of this object cannot be changed via this interface.
 *
 * @author Sylvain Bertrand
 */
public interface FixedFrameShape3DBasics extends Shape3DBasics, FrameShape3DReadOnly
{
   /** {@inheritDoc} */
   @Override
   FixedFrameShape3DPoseBasics getPose();
}
