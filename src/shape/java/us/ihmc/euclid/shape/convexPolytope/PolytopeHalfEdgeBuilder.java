package us.ihmc.euclid.shape.convexPolytope;

import us.ihmc.euclid.shape.convexPolytope.interfaces.PolytopeHalfEdgeProvider;

public class PolytopeHalfEdgeBuilder implements PolytopeHalfEdgeProvider
{

   @Override
   public HalfEdge3D getHalfEdge(Vertex3D origin, Vertex3D destination)
   {
      return new HalfEdge3D((Vertex3D) origin, (Vertex3D) destination);
   }

   @Override
   public HalfEdge3D getHalfEdge()
   {
      return new HalfEdge3D();
   }
}
