package us.ihmc.euclid.shape.convexPolytope.tools;

import us.ihmc.euclid.shape.convexPolytope.Face3D;
import us.ihmc.euclid.shape.convexPolytope.HalfEdge3D;
import us.ihmc.euclid.shape.convexPolytope.Vertex3D;
import us.ihmc.euclid.tuple3D.Vector3D;

public class ConvexPolytope3DFactories
{
   /**
    * Creates a new face such that:
    * <ul>
    * <li>the given {@code vertex} belongs to the new face;
    * <li>the new face has an edge that is the twin of the given {@code twinEdge};
    * <li>the new face's normal can be computed using the direction of the given twin-edge.
    * </ul>
    * 
    * @param vertex a vertex of the new face.
    * @param twinEdge the edge which twin's associated face is the new face.
    * @param epsilon tolerance used when testing if a vertex should be added or not.
    * @return the new face.
    */
   public static Face3D newFace3DFromVertexAndTwinEdge(Vertex3D vertex, HalfEdge3D twinEdge, double epsilon)
   { // TODO the epsilon should probably be set to zero here as otherwise we would probably not create the face.
      Vertex3D v1 = twinEdge.getDestination();
      Vertex3D v2 = twinEdge.getOrigin();
      Vertex3D v3 = vertex;

      // Estimate the face's normal based on its vertices and knowing the expecting ordering based on the twin-edge: v1, v2, then v3.
      Vector3D initialNormal = EuclidPolytopeTools.crossProductOfLineSegment3Ds(v1, v2, v2, v3);
      // As the vertices are clock-wise ordered the cross-product of 2 successive edges should be negated to obtain the face's normal.
      initialNormal.negate();

      Face3D face = new Face3D(initialNormal);

      face.addVertex(v1, epsilon);
      face.addVertex(v2, epsilon);
      face.addVertex(v3, epsilon);

      HalfEdge3D faceFirstEdge = face.getEdge(0);

      // TODO Remove once tested
      assert twinEdge.getOrigin() == faceFirstEdge.getDestination() && twinEdge.getDestination() == faceFirstEdge.getOrigin();

      twinEdge.setTwinEdge(faceFirstEdge);
      faceFirstEdge.setTwinEdge(twinEdge);

      return face;
   }
}
