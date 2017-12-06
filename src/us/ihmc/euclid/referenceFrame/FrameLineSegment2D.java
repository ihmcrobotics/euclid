package us.ihmc.euclid.referenceFrame;

import us.ihmc.euclid.geometry.LineSegment2D;
import us.ihmc.euclid.geometry.interfaces.LineSegment2DBasics;
import us.ihmc.euclid.geometry.interfaces.LineSegment2DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameLineSegment2DReadOnly;
import us.ihmc.euclid.tuple2D.interfaces.Point2DReadOnly;

public class FrameLineSegment2D extends FrameGeometryObject<FrameLineSegment2D, LineSegment2D> implements FrameLineSegment2DReadOnly, LineSegment2DBasics
{
   /** The line segment. */
   private final LineSegment2D lineSegment;
   
   public FrameLineSegment2D(LineSegment2DReadOnly segment)
   {
      this(ReferenceFrame.getWorldFrame(), segment);
   }

   public FrameLineSegment2D(ReferenceFrame referenceFrame, LineSegment2DReadOnly segment)
   {
      super(referenceFrame, new LineSegment2D(segment));
      lineSegment = getGeometryObject();
   }

   @Override
   public Point2DReadOnly getFirstEndpoint()
   {
      return lineSegment.getFirstEndpoint();
   }

   @Override
   public Point2DReadOnly getSecondEndpoint()
   {
      return lineSegment.getSecondEndpoint();
   }

   @Override
   public void setFirstEndpoint(double firstEndpointX, double firstEndpointY)
   {
      lineSegment.setFirstEndpoint(firstEndpointX, firstEndpointY);
   }

   @Override
   public void setSecondEndpoint(double secondEndpointX, double secondEndpointY)
   {
      lineSegment.setSecondEndpoint(secondEndpointX, secondEndpointY);
   }

   @Override
   public void shift(boolean shiftToLeft, double distanceToShift)
   {
      lineSegment.shift(shiftToLeft, distanceToShift);
   }
}