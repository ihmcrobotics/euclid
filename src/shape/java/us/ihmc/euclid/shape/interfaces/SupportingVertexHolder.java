package us.ihmc.euclid.shape.interfaces;

import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DBasics;
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
   default Point3DReadOnly getSupportingVertex(Vector3DReadOnly supportDirection)
   {
      Point3D supportingVertex = new Point3D();
      getSupportingVertex(supportDirection, supportingVertex);
      return supportingVertex;
   }

   boolean getSupportingVertex(Vector3DReadOnly supportDirection, Point3DBasics supportingVertexToPack);
}
