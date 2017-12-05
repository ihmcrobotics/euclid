package us.ihmc.euclid.referenceFrame;

import us.ihmc.euclid.geometry.LineSegment3D;
import us.ihmc.euclid.referenceFrame.interfaces.FrameLineSegment3DReadOnly;

public class FrameLineSegment3D extends FrameGeometryObject<FrameLineSegment3D, LineSegment3D> implements FrameLineSegment3DReadOnly
{
   public FrameLineSegment3D(LineSegment3D segment)
   {
      super(segment);
   }
}
