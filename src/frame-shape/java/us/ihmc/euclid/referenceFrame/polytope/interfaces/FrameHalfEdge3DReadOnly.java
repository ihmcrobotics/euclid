package us.ihmc.euclid.referenceFrame.polytope.interfaces;

import us.ihmc.euclid.referenceFrame.interfaces.FrameLineSegment3DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint3DReadOnly;
import us.ihmc.euclid.shape.convexPolytope.interfaces.HalfEdge3DReadOnly;

public interface FrameHalfEdge3DReadOnly extends HalfEdge3DReadOnly, FrameLineSegment3DReadOnly
{
   @Override
   FrameVertex3DReadOnly getOrigin();

   @Override
   FrameVertex3DReadOnly getDestination();

   default FramePoint3DReadOnly getFirstEndpoint()
   {
      return getOrigin();
   }

   @Override
   default FramePoint3DReadOnly getSecondEndpoint()
   {
      return getDestination();
   }

   @Override
   FrameHalfEdge3DReadOnly getTwin();

   @Override
   FrameHalfEdge3DReadOnly getNext();

   @Override
   FrameHalfEdge3DReadOnly getPrevious();

   default double distanceFromSupportLine(FramePoint3DReadOnly point)
   {
      checkReferenceFrameMatch(point);
      return HalfEdge3DReadOnly.super.distanceFromSupportLine(point);
   }

   default boolean equals(FrameHalfEdge3DReadOnly other)
   {
      if (other == this)
         return true;
      else if (other == null)
         return false;
      else if ((getOrigin() == null) != (other.getOrigin() == null))
         return false;
      else if ((getDestination() == null) != (other.getDestination() == null))
         return false;
      else
         return FrameLineSegment3DReadOnly.super.equals(other);
   }
}
