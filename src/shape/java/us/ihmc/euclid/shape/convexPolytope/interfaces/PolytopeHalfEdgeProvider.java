package us.ihmc.euclid.shape.convexPolytope.interfaces;

import us.ihmc.euclid.shape.convexPolytope.HalfEdge3DBasics;
import us.ihmc.euclid.shape.convexPolytope.Vertex3D;

public interface PolytopeHalfEdgeProvider
{
   HalfEdge3DBasics getHalfEdge(Vertex3D origin, Vertex3D destination);

   HalfEdge3DBasics getHalfEdge();
}
