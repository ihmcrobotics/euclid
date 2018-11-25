package us.ihmc.euclid.shape.convexPolytope.interfaces;

import us.ihmc.euclid.shape.convexPolytope.Vertex3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;

public interface PolytopeVertexProvider
{
   Vertex3D getVertex();

   Vertex3D getVertex(double x, double y, double z);

   Vertex3D getVertex(double coords[]);

   Vertex3D getVertex(Point3DReadOnly vertexToAdd);
}
