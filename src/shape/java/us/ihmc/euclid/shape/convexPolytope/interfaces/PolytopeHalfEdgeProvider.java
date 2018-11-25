package us.ihmc.euclid.shape.convexPolytope.interfaces;

import us.ihmc.euclid.shape.convexPolytope.HalfEdge3D;
import us.ihmc.euclid.shape.convexPolytope.Vertex3D;

public interface PolytopeHalfEdgeProvider
{
   HalfEdge3D getHalfEdge(Vertex3D origin, Vertex3D destination);

   HalfEdge3D getHalfEdge();
}
