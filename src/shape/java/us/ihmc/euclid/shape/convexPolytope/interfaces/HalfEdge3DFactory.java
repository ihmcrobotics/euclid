package us.ihmc.euclid.shape.convexPolytope.interfaces;

public interface HalfEdge3DFactory<Vertex extends Vertex3DReadOnly, Edge extends HalfEdge3DReadOnly>
{
   Edge newInstance(Vertex origin, Vertex destination);
}
