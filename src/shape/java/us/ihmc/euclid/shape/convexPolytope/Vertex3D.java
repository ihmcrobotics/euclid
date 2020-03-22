package us.ihmc.euclid.shape.convexPolytope;

import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;

/**
 * Implementation of a vertex 3D that belongs to a convex polytope 3D.
 * <p>
 * This is part of a Doubly Connected Edge List data structure
 * <a href="https://en.wikipedia.org/wiki/Doubly_connected_edge_list"> link</a>.
 * </p>
 *
 * @author Apoorv Shrivastava
 * @author Sylvain Bertrand
 */
public class Vertex3D extends AbstractVertex3D<Vertex3D, HalfEdge3D, Face3D>
{
   /**
    * Creates a new vertex and initializes its coordinates.
    *
    * @param x the x-coordinate of this vertex.
    * @param y the y-coordinate of this vertex.
    * @param z the z-coordinate of this vertex.
    */
   public Vertex3D(double x, double y, double z)
   {
      super(x, y, z);
   }

   /**
    * Creates a new vertex and initializes its coordinates.
    *
    * @param position the initial position for this vertex. Not modified.
    */
   public Vertex3D(Point3DReadOnly position)
   {
      super(position);
   }
}
