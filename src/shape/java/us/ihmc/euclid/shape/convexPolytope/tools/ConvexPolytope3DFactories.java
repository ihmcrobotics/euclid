package us.ihmc.euclid.shape.convexPolytope.tools;

import us.ihmc.euclid.shape.convexPolytope.Face3D;
import us.ihmc.euclid.shape.convexPolytope.HalfEdge3D;
import us.ihmc.euclid.shape.convexPolytope.Vertex3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;

public class ConvexPolytope3DFactories
{
   /**
    * Creates a new face such that:
    * <ul>
    * <li>the given {@code vertex} belongs to the new face;
    * <li>the new face has an edge that is the twin of the given {@code twinEdge};
    * <li>the new face's normal points away from the given {@code pointBelowFace}.
    * </ul>
    * 
    * @param pointBelowFace the coordinates of a location that is under the face. Not modified.
    * @param vertex a vertex of the new face.
    * @param twinEdge the edge which twin's associated face is the new face.
    * @param epsilon tolerance used when testing if a vertex should be added or not.
    * @return the new face.
    */
   public static Face3D newFace3DFromVertexAndTwinEdge(Point3DReadOnly pointBelowFace, Vertex3D vertex, HalfEdge3D twinEdge, double epsilon)
   { // TODO the epsilon should probably be set to zero here as otherwise we would probably not create the face.
      Vector3D initialNormal = new Vector3D();
      initialNormal.sub(vertex, pointBelowFace);

      Face3D face = new Face3D(initialNormal);
      face.addVertex(twinEdge.getDestination(), epsilon);
      face.addVertex(twinEdge.getOrigin(), epsilon);
      face.addVertex(vertex, epsilon);
      HalfEdge3D faceFirstEdge = face.getEdge(0);

      // TODO Remove once tested
      assert twinEdge.getOrigin() == faceFirstEdge.getDestination() && twinEdge.getDestination() == faceFirstEdge.getOrigin();

      twinEdge.setTwinEdge(faceFirstEdge);
      faceFirstEdge.setTwinEdge(twinEdge);

      return face;
   }
}
