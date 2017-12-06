package us.ihmc.euclid.referenceFrame;

import us.ihmc.euclid.geometry.LineSegment3D;
import us.ihmc.euclid.geometry.interfaces.LineSegment3DBasics;
import us.ihmc.euclid.geometry.interfaces.LineSegment3DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameLineSegment3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;

public class FrameLineSegment3D extends FrameGeometryObject<FrameLineSegment3D, LineSegment3D> implements FrameLineSegment3DReadOnly, LineSegment3DBasics
{
   /** The line segment. */
   private final LineSegment3D lineSegment;
   
   public FrameLineSegment3D(LineSegment3DReadOnly segment)
   {
      this(ReferenceFrame.getWorldFrame(), segment);
   }
   
   public FrameLineSegment3D(ReferenceFrame referenceFrame, LineSegment3DReadOnly segment)
   {
      super(referenceFrame, new LineSegment3D(segment));
      lineSegment = getGeometryObject();
   }

   @Override
   public Point3DReadOnly getFirstEndpoint()
   {
      return lineSegment.getFirstEndpoint();
   }

   @Override
   public Point3DReadOnly getSecondEndpoint()
   {
      return lineSegment.getSecondEndpoint();
   }

   @Override
   public void setFirstEndpoint(double firstEndpointX, double firstEndpointY, double firstEndpointZ)
   {
      lineSegment.setFirstEndpoint(firstEndpointX, firstEndpointY, firstEndpointZ);
   }

   @Override
   public void setSecondEndpoint(double secondEndpointX, double secondEndpointY, double secondEndpointZ)
   {
      lineSegment.setSecondEndpoint(secondEndpointX, secondEndpointY, secondEndpointZ);
   }
}
