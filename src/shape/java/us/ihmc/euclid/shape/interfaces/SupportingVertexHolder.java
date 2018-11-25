package us.ihmc.euclid.shape.interfaces;

import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;

public interface SupportingVertexHolder
{
   /**
    * Get a support vertex in the direction specified
    * 
    * @param supportDirection the direction to search in
    * @return the spatial location of the supporting vertex
    */
   Point3DReadOnly getSupportingVertex(Vector3DReadOnly supportDirection);
}
