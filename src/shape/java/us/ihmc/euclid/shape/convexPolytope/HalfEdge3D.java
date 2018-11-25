package us.ihmc.euclid.shape.convexPolytope;

import us.ihmc.euclid.shape.convexPolytope.interfaces.HalfEdge3DReadOnly;
import us.ihmc.euclid.shape.convexPolytope.interfaces.PolytopeHalfEdgeProvider;

/**
 * This class implements a doubly connected edge list
 * (https://en.wikipedia.org/wiki/Doubly_connected_edge_list) for storing polytope information A
 * half edge is completely described by its origin, destination and twin edge The face, previous
 * half edge and next half edge are stored for readability of code and should not be used for any
 * geometrical operations An attempt is made to update the twin in case the edge is modified to
 * ensure that the relation remains consistent
 * 
 * @author Apoorv S
 */
public class HalfEdge3D extends HalfEdge3DBasics
{
   private final PolytopeHalfEdgeBuilder halfEdgeBuilder = new PolytopeHalfEdgeBuilder();

   public HalfEdge3D()
   {
      super();
   }

   /**
    * Creates a new edge at the same location. References to origin / destination vertices, twin / next
    * / previous edges and associated is not preserved
    * 
    * @param edge
    */
   public HalfEdge3D(HalfEdge3DReadOnly edge)
   {
      super(new Vertex3D(edge.getOriginVertex()), new Vertex3D(edge.getDestinationVertex()));
   }

   public HalfEdge3D(Vertex3D origin, Vertex3D destination)
   {
      super(origin, destination);
   }

   public HalfEdge3D(Vertex3D originVertex, Vertex3D destinationVertex, HalfEdge3D twinEdge,
                           HalfEdge3D nextHalfEdge, HalfEdge3D previousHalfEdge, Face3D face)
   {
      super(originVertex, destinationVertex, twinEdge, nextHalfEdge, previousHalfEdge, face);
   }

   public HalfEdge3D(HalfEdge3D twinEdge, Face3D face)
   {
      super(twinEdge, face);
   }

   @Override
   public HalfEdge3D getNextHalfEdge()
   {
      return (HalfEdge3D) super.getNextHalfEdge();
   }

   @Override
   protected PolytopeHalfEdgeProvider getHalfEdgeProvider()
   {
      return halfEdgeBuilder;
   }
}
