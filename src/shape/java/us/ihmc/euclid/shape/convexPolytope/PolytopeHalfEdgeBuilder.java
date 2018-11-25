package us.ihmc.euclid.shape.convexPolytope;

import us.ihmc.euclid.shape.convexPolytope.interfaces.HalfEdge3DReadOnly;
import us.ihmc.euclid.shape.convexPolytope.interfaces.PolytopeHalfEdgeProvider;

public class PolytopeHalfEdgeBuilder implements PolytopeHalfEdgeProvider
{

   @Override
   public HalfEdge3D getHalfEdge(Vertex3DBasics origin, Vertex3DBasics destination)
   {
      return new HalfEdge3D((Vertex3D) origin, (Vertex3D) destination);
   }

   @Override
   public HalfEdge3D getHalfEdge()
   {
      return new HalfEdge3D();
   }

   @Override
   public HalfEdge3D getHalfEdge(HalfEdge3DReadOnly polytopeHalfEdge)
   {
      return new HalfEdge3D(polytopeHalfEdge);
   }
}
