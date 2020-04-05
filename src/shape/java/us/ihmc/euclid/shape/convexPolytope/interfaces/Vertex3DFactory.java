package us.ihmc.euclid.shape.convexPolytope.interfaces;

import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;

/**
 * Factory for creating a specific type of vertex.
 *
 * @author Sylvain Bertrand
 * @param <Vertex> the final type used for representing a vertex.
 */
public interface Vertex3DFactory<Vertex extends Vertex3DReadOnly>
{
   /**
    * Creates and initializes a new face.
    *
    * @param position the vertex position. Not modified.
    * @return the new vertex.
    */
   Vertex newInstance(Point3DReadOnly position);
}
