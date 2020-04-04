package us.ihmc.euclid.shape.convexPolytope.interfaces;

/**
 * Factory for creating a specific type of half-edge.
 *
 * @author Sylvain Bertrand
 * @param <Vertex> the final type used for representing a vertex.
 * @param <Edge>   the final type used for representing a half-edge.
 */
public interface HalfEdge3DFactory<Vertex extends Vertex3DReadOnly, Edge extends HalfEdge3DReadOnly>
{
   /**
    * Creates and initializes a new half-edge.
    *
    * @param origin      the vertex the half-edge starts from. Not modified, reference saved.
    * @param destination the vertex the half-edge ends at. Not modified, reference saved.
    * @return the new half-edge.
    */
   Edge newInstance(Vertex origin, Vertex destination);
}
