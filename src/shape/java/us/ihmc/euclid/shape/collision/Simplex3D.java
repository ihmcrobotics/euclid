package us.ihmc.euclid.shape.collision;

import java.util.ArrayList;
import java.util.List;
import java.util.stream.Collectors;

import us.ihmc.euclid.geometry.tools.EuclidGeometryTools;
import us.ihmc.euclid.shape.convexPolytope.ConvexPolytope3D;
import us.ihmc.euclid.shape.convexPolytope.Face3D;
import us.ihmc.euclid.shape.convexPolytope.HalfEdge3D;
import us.ihmc.euclid.shape.convexPolytope.Vertex3D;
import us.ihmc.euclid.shape.convexPolytope.interfaces.ConvexPolytopeFeature3D;
import us.ihmc.euclid.shape.convexPolytope.interfaces.Vertex3DReadOnly;
import us.ihmc.euclid.shape.convexPolytope.tools.EuclidPolytopeTools;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DBasics;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DBasics;

public class Simplex3D implements ConvexPolytopeFeature3D
{
   private final double constructionEpsilon;
   private ConvexPolytope3D polytope;

   public Simplex3D(double constructionEpsilon)
   {
      this.constructionEpsilon = constructionEpsilon;
      polytope = new ConvexPolytope3D(constructionEpsilon);
   }

   public boolean addVertex(Point3DReadOnly vertexOnShapeA, Point3DReadOnly vertexOnShapeB)
   {
      return addVertex(new DifferenceVertex3D(vertexOnShapeA, vertexOnShapeB));
   }

   public boolean addVertex(DifferenceVertex3D newVertex)
   {
      return polytope.addVertex(newVertex);
   }

   @Override
   public boolean getSupportVectorDirectionTo(Point3DReadOnly point, Vector3DBasics supportVectorToPack)
   {
      return polytope.getSupportVectorDirectionTo(point, supportVectorToPack);
   }

   public void reduceToFeature(ConvexPolytopeFeature3D smallestFeature)
   {
      List<DifferenceVertex3D> vertices = new ArrayList<>();
      for (Vertex3DReadOnly vertex : smallestFeature.getVertices())
      {
         DifferenceVertex3D differenceVertex = (DifferenceVertex3D) vertex;
         differenceVertex.clearAssociatedEdgeList();
         vertices.add(differenceVertex);
      }

      polytope = new ConvexPolytope3D(constructionEpsilon);
      polytope.addVertices(vertices);
   }

   public void getCollidingPointsOnSimplex(Point3DReadOnly point, Point3DBasics pointOnA, Point3DBasics pointOnB)
   {
      ConvexPolytopeFeature3D member = getSmallestFeature(point);
      // Assuming linearity between the simplex and polytope points
      if (member instanceof Face3D)
      {
         Face3D face = (Face3D) member;
         Point3D projection = new Point3D();
         EuclidGeometryTools.orthogonalProjectionOnPlane3D(point, face.getCentroid(), face.getNormal(), projection);
         double[] coords = EuclidPolytopeTools.barycentricCoordinates(face, projection);

         pointOnA.setToZero();
         pointOnB.setToZero();

         for (int i = 0; i < face.getNumberOfEdges(); i++)
         {
            DifferenceVertex3D vertex = (DifferenceVertex3D) face.getEdge(i).getOrigin();

            pointOnA.scaleAdd(coords[i], vertex.getVertexOnShapeA(), pointOnA);
            pointOnB.scaleAdd(coords[i], vertex.getVertexOnShapeB(), pointOnB);
         }
      }
      else if (member instanceof HalfEdge3D)
      {
         HalfEdge3D edge = (HalfEdge3D) member;
         DifferenceVertex3D simplexVertex1 = (DifferenceVertex3D) edge.getOrigin();
         DifferenceVertex3D simplexVertex2 = (DifferenceVertex3D) edge.getDestination();
         double percentage = EuclidGeometryTools.percentageAlongLineSegment3D(point, simplexVertex1, simplexVertex2);

         pointOnA.interpolate(simplexVertex1.getVertexOnShapeA(), simplexVertex2.getVertexOnShapeA(), percentage);
         pointOnB.interpolate(simplexVertex1.getVertexOnShapeB(), simplexVertex2.getVertexOnShapeB(), percentage);
      }
      else if (member instanceof DifferenceVertex3D)
      {
         DifferenceVertex3D vertex = (DifferenceVertex3D) member;
         pointOnA.set(vertex.getVertexOnShapeA());
         pointOnB.set(vertex.getVertexOnShapeB());
      }
      else
      {
         throw new RuntimeException("Unhandled simplex member " + member.getClass());
      }
   }

   public boolean isPointInside(Point3DReadOnly pointToCheck)
   {
      return polytope.isPointInside(pointToCheck);
   }

   public ConvexPolytope3D getPolytope()
   {
      return polytope;
   }

   @Override
   public double distance(Point3DReadOnly point)
   {
      return polytope.distance(point);
   }

   @Override
   public ConvexPolytopeFeature3D getSmallestFeature(Point3DReadOnly point)
   {
      return polytope.getSmallestFeature(point);
   }

   @Override
   public List<Vertex3D> getVertices()
   {
      return polytope.getVertices();
   }

   public List<DifferenceVertex3D> getDifferenceVertices()
   {
      return polytope.getVertices().stream().map(v -> (DifferenceVertex3D) v).collect(Collectors.toList());
   }
}
