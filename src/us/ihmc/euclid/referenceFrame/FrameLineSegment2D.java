package us.ihmc.euclid.referenceFrame;

import us.ihmc.euclid.geometry.LineSegment2D;
import us.ihmc.euclid.geometry.interfaces.LineSegment2DBasics;
import us.ihmc.euclid.geometry.interfaces.LineSegment2DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameLineSegment2DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint2DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameTuple2DReadOnly;

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
   public FramePoint2DReadOnly getFirstEndpoint()
   {
      return new FramePoint2D(getReferenceFrame(), lineSegment.getFirstEndpoint());
   }

   @Override
   public FramePoint2DReadOnly getSecondEndpoint()
   {
      return new FramePoint2D(getReferenceFrame(), lineSegment.getSecondEndpoint());
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
   
   public void setFirstEndpoint(FramePoint2DReadOnly firstEndpoint)
   {
      checkReferenceFrameMatch(firstEndpoint);
      LineSegment2DBasics.super.setFirstEndpoint(firstEndpoint);
   }
   
   public void setSecondEndpoint(FramePoint2DReadOnly secondEndpoint)
   {
      checkReferenceFrameMatch(secondEndpoint);
      LineSegment2DBasics.super.setSecondEndpoint(secondEndpoint);
   }

   @Override
   public void shift(boolean shiftToLeft, double distanceToShift)
   {
      lineSegment.shift(shiftToLeft, distanceToShift);
   }
   
   public void translate(FrameTuple2DReadOnly translation)
   {
      checkReferenceFrameMatch(translation);
      LineSegment2DBasics.super.translate(translation);
   }
}