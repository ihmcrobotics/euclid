package us.ihmc.euclid.shape.convexPolytope.interfaces;

import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;

public interface Vertex3DFactory<Vertex extends Vertex3DReadOnly>
{
   Vertex newInstance(Point3DReadOnly position);
}
