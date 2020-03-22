package us.ihmc.euclid.shape.convexPolytope;

/**
 * Implementation of a half-edge 3D that belongs to a convex polytope 3D.
 * <p>
 * This is part of a Doubly Connected Edge List data structure
 * <a href="https://en.wikipedia.org/wiki/Doubly_connected_edge_list"> link</a>.
 * </p>
 *
 * @author Apoorv Shrivastava
 * @author Sylvain Bertrand
 */
public class HalfEdge3D extends AbstractHalfEdge3D<Vertex3D, HalfEdge3D, Face3D>
{
   /**
    * Creates a new half-edge and initializes its origin and destination.
    *
    * @param origin      the vertex the half-edge starts from. Not modified, reference saved.
    * @param destination the vertex the half-edge ends at. Not modified, reference saved.
    */
   public HalfEdge3D(Vertex3D origin, Vertex3D destination)
   {
      super(origin, destination);
   }
}
