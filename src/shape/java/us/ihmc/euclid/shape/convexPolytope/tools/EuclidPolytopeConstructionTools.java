package us.ihmc.euclid.shape.convexPolytope.tools;

import java.util.ArrayList;
import java.util.Collection;
import java.util.List;

import us.ihmc.euclid.shape.convexPolytope.Face3D;
import us.ihmc.euclid.shape.convexPolytope.HalfEdge3D;
import us.ihmc.euclid.shape.convexPolytope.Vertex3D;
import us.ihmc.euclid.tuple3D.Vector3D;

public class EuclidPolytopeConstructionTools
{
   public static final double DEFAULT_CONSTRUCTION_EPSILON = 1.0e-10;

   /**
    * Computes the faces containing the given {@code vertex} as follows:
    * <ul>
    * <li>if the vertex is in the plane of a silhouette edge's face, the face is expanded to include
    * the new vertex;
    * <li>otherwise, a new face is created from the vertex and the silhouette edge.
    * </ul>
    * 
    * @param vertex faces are modified and/or created to include this vertex.
    * @param silhouetteEdges the contour visible from the vertex. Each edge is expected to be
    *           associated with either a hidden face or an in-plane face.
    * @param inPlaneFaces the list of faces for which the vertex is considered to lie in the face's
    *           support plane. These faces are expanded to include the new vertex.
    * @param epsilon tolerance used for testing edge-cases such as equivalent vertices, vertex lying on
    *           a line, etc.
    * @return the list of new faces that were created in the the process.
    */
   public static List<Face3D> computeVertexNeighborFaces(Vertex3D vertex, Collection<HalfEdge3D> silhouetteEdges, Collection<Face3D> inPlaneFaces,
                                                         double epsilon)
   {
      List<Face3D> newFaces = new ArrayList<>();
   
      for (HalfEdge3D silhouetteEdge : silhouetteEdges)
      { // Modify/Create the faces that are to contain the new vertex. The faces will take care of updating the edges.
         if (inPlaneFaces.contains(silhouetteEdge.getFace()))
         { // The face has to be extended to include the new vertex
            silhouetteEdge.getFace().addVertex(vertex);
         }
         else
         { // Creating a new face.
            newFaces.add(newFace3DFromVertexAndTwinEdge(vertex, silhouetteEdge, epsilon));
         }
      }
   
      for (HalfEdge3D startingFrom : vertex.getAssociatedEdges())
      { // Going through the new edges and associating the twins.
         HalfEdge3D endingTo = startingFrom.getDestination().getEdgeTo(vertex);
   
         startingFrom.setTwin(endingTo);
         endingTo.setTwin(startingFrom);
      }
   
      return newFaces;
   }

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
   
      Face3D face = new Face3D(initialNormal, epsilon);
   
      face.addVertex(v1);
      face.addVertex(v2);
      face.addVertex(v3);
   
      HalfEdge3D faceFirstEdge = face.getEdge(0);
   
      twinEdge.setTwin(faceFirstEdge);
      faceFirstEdge.setTwin(twinEdge);
   
      return face;
   }

}
