package us.ihmc.euclid.shape.convexPolytope.interfaces;

import java.util.List;

public interface PolytopeListener
{
   void attachPolytope(ConvexPolytope3DReadOnly polytopeToAttach);

   void updateAll();

   void updateEdges();

   void updateVertices();

   void updateFaces();

   void updateVisibleSilhouette(List<? extends HalfEdge3DReadOnly> visibleEdges);

   void udpateVisibleEdgeSeed(HalfEdge3DReadOnly visibleEdgeSeed);

   void updateOnFaceList(List<? extends Face3DReadOnly> onFaceList);

   void updateVisibleFaceList(List<? extends Face3DReadOnly> visibleFaceList);
}
