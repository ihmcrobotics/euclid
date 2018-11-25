package us.ihmc.euclid.shape.convexPolytope;

import us.ihmc.euclid.shape.convexPolytope.interfaces.PolytopeHalfEdgeProvider;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;

/**
 * This class defines a polytope face. A face is defined by the set of edges that bound it.
 * 
 * @author Apoorv S
 *
 */
public class Face3D extends Face3DBasics
{
   private final PolytopeHalfEdgeBuilder halfEdgeBuilder = new PolytopeHalfEdgeBuilder();

   public Face3D()
   {
      super();
   }

   @Override
   protected PolytopeHalfEdgeProvider getHalfEdgeProvider()
   {
      return halfEdgeBuilder;
   }

   @Override
   public HalfEdge3D getEdge(int index)
   {
      return (HalfEdge3D) super.getEdge(index);
   }

   @Override
   public HalfEdge3D getFirstVisibleEdge(Point3DReadOnly vertex)
   {
      return (HalfEdge3D) super.getFirstVisibleEdge(vertex);
   }
}
