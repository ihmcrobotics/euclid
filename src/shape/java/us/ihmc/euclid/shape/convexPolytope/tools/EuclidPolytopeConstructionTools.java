package us.ihmc.euclid.shape.convexPolytope.tools;

import java.util.ArrayList;
import java.util.Collection;
import java.util.List;

import us.ihmc.euclid.shape.convexPolytope.Face3D;
import us.ihmc.euclid.shape.convexPolytope.HalfEdge3D;
import us.ihmc.euclid.shape.convexPolytope.Vertex3D;

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
            silhouetteEdge.getFace().addVertex(vertex, epsilon);
         }
         else
         { // Creating a new face.
            newFaces.add(ConvexPolytope3DFactories.newFace3DFromVertexAndTwinEdge(vertex, silhouetteEdge, epsilon));
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

}
