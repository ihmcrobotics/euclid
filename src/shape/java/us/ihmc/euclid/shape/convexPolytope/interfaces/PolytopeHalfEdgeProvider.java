package us.ihmc.euclid.shape.convexPolytope.interfaces;

import us.ihmc.euclid.shape.convexPolytope.HalfEdge3DBasics;
import us.ihmc.euclid.shape.convexPolytope.Vertex3DBasics;

public interface PolytopeHalfEdgeProvider
{
   HalfEdge3DBasics getHalfEdge(Vertex3DBasics origin, Vertex3DBasics destination);

   HalfEdge3DBasics getHalfEdge();

   HalfEdge3DBasics getHalfEdge(HalfEdge3DReadOnly polytopeHalfEdge);
}
