package us.ihmc.euclid.referenceFrame.polytope.interfaces;

import java.util.Collection;

import us.ihmc.euclid.referenceFrame.interfaces.FramePoint3DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVector3DReadOnly;
import us.ihmc.euclid.shape.convexPolytope.interfaces.Vertex3DReadOnly;

public interface FrameVertex3DReadOnly extends Vertex3DReadOnly, FramePoint3DReadOnly
{
   Collection<? extends FrameHalfEdge3DReadOnly> getAssociatedEdges();

   FrameHalfEdge3DReadOnly getAssociatedEdge(int index);

   default boolean isEdgeAssociated(FrameHalfEdge3DReadOnly edgeToCheck)
   {
      checkReferenceFrameMatch(edgeToCheck);
      return Vertex3DReadOnly.super.isEdgeAssociated(edgeToCheck);
   }

   default FrameHalfEdge3DReadOnly getEdgeTo(Vertex3DReadOnly destination)
   {
      return (FrameHalfEdge3DReadOnly) Vertex3DReadOnly.super.getEdgeTo(destination);
   }

   default FrameHalfEdge3DReadOnly getEdgeTo(FrameVertex3DReadOnly destination)
   {
      checkReferenceFrameMatch(destination);
      return (FrameHalfEdge3DReadOnly) Vertex3DReadOnly.super.getEdgeTo(destination);
   }

   default double dot(FrameVector3DReadOnly vector)
   {
      return Vertex3DReadOnly.super.dot(vector);
   }

   default boolean equals(FrameVertex3DReadOnly other)
   {
      return FramePoint3DReadOnly.super.equals(other);
   }
}
