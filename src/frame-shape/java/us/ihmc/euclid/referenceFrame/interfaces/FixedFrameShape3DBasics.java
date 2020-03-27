package us.ihmc.euclid.referenceFrame.interfaces;

import us.ihmc.euclid.shape.primitives.interfaces.Shape3DBasics;

public interface FixedFrameShape3DBasics extends Shape3DBasics, FrameShape3DReadOnly
{
   @Override
   FixedFrameShape3DPoseBasics getPose();
}
