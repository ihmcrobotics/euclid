package us.ihmc.euclid.shape.collision.shapeModifier;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

import us.ihmc.euclid.geometry.interfaces.LineSegment3DReadOnly;
import us.ihmc.euclid.geometry.tools.EuclidGeometryTools;
import us.ihmc.euclid.shape.convexPolytope.interfaces.ConvexPolytope3DReadOnly;
import us.ihmc.euclid.shape.convexPolytope.interfaces.Face3DReadOnly;
import us.ihmc.euclid.shape.convexPolytope.interfaces.HalfEdge3DReadOnly;
import us.ihmc.euclid.shape.convexPolytope.interfaces.Vertex3DReadOnly;
import us.ihmc.euclid.shape.convexPolytope.tools.EuclidPolytopeTools;
import us.ihmc.euclid.shape.tools.EuclidShapeTools;
import us.ihmc.euclid.tools.TupleTools;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DBasics;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;

public class ConvexPolytope3DSTPBoundingVolume extends SphereTorusPatchesBoundingVolume<ConvexPolytope3DReadOnly>
{
   private final Point3D faceSphereCenter = new Point3D();
   private final Point3D edgeTorusCenter = new Point3D();
   private final Vector3D edgeTorusAxis = new Vector3D();

   public ConvexPolytope3DSTPBoundingVolume()
   {
   }

   @Override
   protected double findMaximumEdgeLengthSquared()
   {
      double maximumEdgeLengthSquared = 0.0;

      for (int faceIndex = 0; faceIndex < shape3D.getNumberOfFaces(); faceIndex++)
      {
         Face3DReadOnly face = shape3D.getFace(faceIndex);
         Vertex3DReadOnly firstVertex = face.getVertex(0);

         for (int vertexIndex = 1; vertexIndex < face.getNumberOfEdges(); vertexIndex++)
         {
            maximumEdgeLengthSquared = Math.max(firstVertex.distanceSquared(face.getVertex(vertexIndex)), maximumEdgeLengthSquared);
         }
      }
      return maximumEdgeLengthSquared;
   }

   @Override
   public boolean getSupportingVertex(Vector3DReadOnly supportDirection, Point3DBasics supportingVertexToPack)
   {
      Vertex3DReadOnly bestVertex = shape3D.getSupportingVertex(supportDirection);

      if (bestVertex == null)
         return false;

      List<STPFace3D> bestSTPFaces = findBestFace(supportDirection, bestVertex);

      if (getFaceSupportingVertex(supportDirection, bestSTPFaces, bestVertex, supportingVertexToPack))
         return true;

      if (getBestEdgeSupportingVertex(supportDirection, bestSTPFaces, bestVertex, supportingVertexToPack))
         return true;

      EuclidShapeTools.supportingVertexSphere3D(supportDirection, bestVertex, smallRadius, supportingVertexToPack);
      return true;
   }

   private List<STPFace3D> findBestFace(Vector3DReadOnly supportDirection, Vertex3DReadOnly bestVertex)
   {
      Face3DReadOnly bestFace = bestVertex.getAssociatedEdge(0).getFace();
      double bestFaceDot = bestFace.getNormal().dot(supportDirection);

      for (int i = 1; i < bestVertex.getNumberOfAssociatedEdges(); i++)
      {
         HalfEdge3DReadOnly candidateEdge = bestVertex.getAssociatedEdge(i);
         Face3DReadOnly candidateFace = candidateEdge.getFace();
         double candidateDot = candidateFace.getNormal().dot(supportDirection);

         if (candidateDot > bestFaceDot)
         {
            bestFaceDot = candidateDot;
            bestFace = candidateFace;
         }
      }

      return STPFace3D.newSTPFace3Ds(bestFace);
   }

   private boolean getFaceSupportingVertex(Vector3DReadOnly supportDirection, List<STPFace3D> bestFaces, Vertex3DReadOnly bestVertex,
                                           Point3DBasics supportingVertexToPack)
   {
      for (STPFace3D stpFace : bestFaces)
      {
         if (!stpFace.contains(bestVertex))
            continue;

         if (getTriangleSupportingVertex(supportDirection, stpFace, supportingVertexToPack))
            return true;
      }
      return false;
   }

   private boolean getTriangleSupportingVertex(Vector3DReadOnly supportDirection, STPFace3D face, Point3DBasics supportingVertexToPack)
   {
      EuclidGeometryTools.sphere3DPositionFromThreePoints(face.v0, face.v1, face.v2, largeRadius - smallRadius, faceSphereCenter);
      EuclidShapeTools.supportingVertexSphere3D(supportDirection, faceSphereCenter, largeRadius, supportingVertexToPack);
      return face.isPointDirectlyAboveOrBelow(supportingVertexToPack);
   }

   private boolean getBestEdgeSupportingVertex(Vector3DReadOnly supportDirection, List<STPFace3D> bestFaces, Vertex3DReadOnly bestVertex,
                                               Point3DBasics supportingVertexToPack)
   {
      STPHalfEdge3D bestEdge = null;
      double bestEdgeDot = Double.NEGATIVE_INFINITY;

      for (STPFace3D face : bestFaces)
      {
         if (!face.contains(bestVertex))
            continue;

         for (STPHalfEdge3D edge : face.getEdges())
         {
            if (!edge.contains(bestVertex))
               continue;

            double candidateDot = TupleTools.dot(edge.midpoint(), supportDirection);

            if (candidateDot > bestEdgeDot)
            {
               bestEdgeDot = candidateDot;
               bestEdge = edge;
            }
         }
      }

      return getEdgeSupportingVertex(supportDirection, bestEdge, supportingVertexToPack);
   }

   private boolean getEdgeSupportingVertex(Vector3DReadOnly supportDirection, STPHalfEdge3D edge, Point3DBasics supportingVertexToPack)
   {
      edgeTorusCenter.add(edge.getFirstEndpoint(), edge.getSecondEndpoint());
      edgeTorusCenter.scale(0.5);
      edgeTorusAxis.sub(edge.getSecondEndpoint(), edge.getFirstEndpoint());

      double radius = largeRadius - smallRadius;
      double torusRadius = Math.sqrt(radius * radius - 0.25 * edgeTorusAxis.lengthSquared());
      double torusTubeRadius = largeRadius;

      EuclidShapeTools.innerSupportingVertexTorus3D(supportDirection, edgeTorusCenter, edgeTorusAxis, torusRadius, torusTubeRadius, supportingVertexToPack);

      if (!edge.isBetweenEndpoints(supportingVertexToPack))
         return false;
      if (edge.face.isPointDirectlyAboveOrBelow(supportingVertexToPack))
         return false;
      if (edge.twin != null && edge.twin.face.isPointDirectlyAboveOrBelow(supportingVertexToPack))
         return false;
      return true;
   }

   private static class STPFace3D
   {
      private final Face3DReadOnly owner;
      private final Vertex3DReadOnly v0, v1, v2;
      private final STPHalfEdge3D e0, e1, e2;
      private final List<STPHalfEdge3D> edges;

      public static List<STPFace3D> newSTPFace3Ds(Face3DReadOnly owner)
      {
         List<STPFace3D> faces = new ArrayList<>();

         STPFace3D previousFace = new STPFace3D(owner, owner.getVertex(0), owner.getVertex(1), owner.getVertex(2));
         faces.add(previousFace);

         for (int i = 1; i < owner.getNumberOfEdges() - 2; i++)
         {
            Vertex3DReadOnly v0 = owner.getVertex(0);
            Vertex3DReadOnly v1 = owner.getVertex(i + 1);
            Vertex3DReadOnly v2 = owner.getVertex(i + 2);
            STPFace3D currentFace = new STPFace3D(owner, v0, v1, v2);

            currentFace.e0.twin = previousFace.e2;
            previousFace.e2.twin = currentFace.e0;

            faces.add(currentFace);
         }

         return faces;
      }

      public STPFace3D(Face3DReadOnly face3D, Vertex3DReadOnly v0, Vertex3DReadOnly v1, Vertex3DReadOnly v2)
      {
         owner = face3D;
         this.v0 = v0;
         this.v1 = v1;
         this.v2 = v2;
         e0 = new STPHalfEdge3D(v0, v1);
         e1 = new STPHalfEdge3D(v1, v2);
         e2 = new STPHalfEdge3D(v2, v0);
         e0.face = this;
         e1.face = this;
         e2.face = this;
         edges = Arrays.asList(e0, e1, e2);
      }

      public boolean contains(Vertex3DReadOnly query)
      {
         return v0 == query || v1 == query || v2 == query;
      }

      /**
       * Tests whether the query is located directly above or below this face, such its projection would
       * be located inside this face.
       *
       * @param query the coordinates of the query. Not modified.
       * @return {@code true} if the query is located either directly above or below this face,
       *         {@code false} otherwise.
       */
      public boolean isPointDirectlyAboveOrBelow(Point3DReadOnly query)
      {
         if (canObserverSeeEdge(query, e0))
            return false;
         if (canObserverSeeEdge(query, e1))
            return false;
         if (canObserverSeeEdge(query, e2))
            return false;
         return true;
      }

      public boolean canObserverSeeEdge(Point3DReadOnly observer, LineSegment3DReadOnly edge)
      {
         return EuclidPolytopeTools.isPoint3DOnLeftSideOfLine3D(observer, edge.getFirstEndpoint(), edge.getSecondEndpoint(), owner.getNormal());
      }

      public List<STPHalfEdge3D> getEdges()
      {
         return edges;
      }
   }

   private static class STPHalfEdge3D implements LineSegment3DReadOnly
   {
      private Vertex3DReadOnly origin, destination;
      private STPFace3D face;
      private STPHalfEdge3D twin;

      public STPHalfEdge3D(Vertex3DReadOnly origin, Vertex3DReadOnly destination)
      {
         this.origin = origin;
         this.destination = destination;
      }

      public boolean contains(Vertex3DReadOnly query)
      {
         return origin == query || destination == query;
      }

      @Override
      public Vertex3DReadOnly getFirstEndpoint()
      {
         return origin;
      }

      @Override
      public Vertex3DReadOnly getSecondEndpoint()
      {
         return destination;
      }
   }
}
