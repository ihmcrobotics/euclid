package us.ihmc.euclid.shape.convexPolytope.interfaces;

import us.ihmc.euclid.shape.convexPolytope.Vertex3DBasics;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;

public interface PolytopeVertexProvider
{
   Vertex3DBasics getVertex();

   Vertex3DBasics getVertex(double x, double y, double z);

   Vertex3DBasics getVertex(double coords[]);

   Vertex3DBasics getVertex(Point3DReadOnly vertexToAdd);
}
