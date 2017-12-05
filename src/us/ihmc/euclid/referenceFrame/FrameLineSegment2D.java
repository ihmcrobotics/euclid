package us.ihmc.euclid.referenceFrame;

import us.ihmc.euclid.geometry.LineSegment2D;
import us.ihmc.euclid.referenceFrame.interfaces.FrameLineSegment2DReadOnly;

public class FrameLineSegment2D extends FrameGeometryObject<FrameLineSegment2D, LineSegment2D> implements FrameLineSegment2DReadOnly
{
   FrameLineSegment2D(LineSegment2D segment)
   {
      super(segment);
   }
}