package us.ihmc.euclid.referenceFrame;

import us.ihmc.euclid.geometry.LineSegment3D;
import us.ihmc.euclid.geometry.interfaces.LineSegment3DBasics;
import us.ihmc.euclid.geometry.interfaces.LineSegment3DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameLineSegment3DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint3DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameTuple3DReadOnly;

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
   public FramePoint3DReadOnly getFirstEndpoint()
   {
      return new FramePoint3D(getReferenceFrame(), lineSegment.getFirstEndpoint());
   }

   @Override
   public FramePoint3DReadOnly getSecondEndpoint()
   {
      return new FramePoint3D(getReferenceFrame(), lineSegment.getSecondEndpoint());
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
   
   /**
    * Translates this line segment by the given (x, y, z) contained in {@code translation}.
    * <p>
    * Note that the length and direction of this line segment remains unchanged.
    * </p>
    *
    * @param translation the translation to add to each endpoint of this line segment. Not modified.
    */
   public void translate(FrameTuple3DReadOnly translation)
   {
      checkReferenceFrameMatch(translation);
      LineSegment3DBasics.super.translate(translation);
   }
}
