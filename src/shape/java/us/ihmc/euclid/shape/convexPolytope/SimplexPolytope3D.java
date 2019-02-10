package us.ihmc.euclid.shape.convexPolytope;

import java.util.ArrayList;
import java.util.List;

import us.ihmc.euclid.geometry.tools.EuclidGeometryTools;
import us.ihmc.euclid.shape.convexPolytope.interfaces.Simplex3D;
import us.ihmc.euclid.shape.convexPolytope.interfaces.Vertex3DReadOnly;
import us.ihmc.euclid.shape.convexPolytope.tools.EuclidPolytopeTools;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DBasics;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DBasics;

public class SimplexPolytope3D implements Simplex3D
{
   private ConvexPolytope3D polytope = new ConvexPolytope3D();
   private List<SimplexVertex3D> vertices = new ArrayList<>();
   private final Point3D projection = new Point3D();

   public SimplexPolytope3D()
   {
      super();
   }

   public SimplexVertex3D addVertex(Vertex3DReadOnly vertexOnPolytopeA, Vertex3DReadOnly vertexOnPolytopeB)
   {
      SimplexVertex3D newVertex = new SimplexVertex3D();
      newVertex.set(vertexOnPolytopeA, vertexOnPolytopeB);
      polytope.addVertex(newVertex);
      return newVertex;
   }

   public void clear()
   {
      vertices.clear();
      polytope.clear();
   }

   public boolean isPointInside(Point3DReadOnly pointToCheck, double epsilon)
   {
      return polytope.isPointInside(pointToCheck, epsilon);
   }

   @Override
   public double distance(Point3DReadOnly point)
   {
      return polytope.distance(point);
   }

   @Override
   public boolean getSupportVectorDirectionTo(Point3DReadOnly point, Vector3DBasics supportVectorToPack)
   {
      return polytope.getSupportVectorDirectionTo(point, supportVectorToPack);
   }

   public boolean isEmpty()
   {
      return polytope.isEmpty();
   }

   @Override
   public Simplex3D getSmallestSimplexMemberReference(Point3DReadOnly point)
   {
      return polytope.getSmallestSimplexMemberReference(point);
   }

   @Override
   public String toString()
   {
      return polytope.toString();
   }

   public ConvexPolytope3D getPolytope()
   {
      return polytope;
   }

   public void getCollidingPointsOnSimplex(Point3DReadOnly point, Point3DBasics pointOnA, Point3DBasics pointOnB)
   {
      Simplex3D member = getSmallestSimplexMemberReference(point);
      // Assuming linearity between the simplex and polytope points
      if (member instanceof Face3D)
      {
         Face3D face = (Face3D) member;

         EuclidGeometryTools.orthogonalProjectionOnPlane3D(point, face.getCentroid(), face.getNormal(), projection);
         double[] coords = EuclidPolytopeTools.barycentricCoordinates(face, projection);

         pointOnA.setToZero();
         pointOnB.setToZero();

         for (int i = 0; i < face.getNumberOfEdges(); i++)
         {
            SimplexVertex3D vertex = (SimplexVertex3D) face.getEdge(i).getOrigin();

            pointOnA.scaleAdd(coords[i], vertex.getVertexOnPolytopeA(), pointOnA);
            pointOnB.scaleAdd(coords[i], vertex.getVertexOnPolytopeB(), pointOnB);
         }
      }
      else if (member instanceof HalfEdge3D)
      {
         SimplexVertex3D simplexVertex1 = (SimplexVertex3D) ((HalfEdge3D) member).getOrigin();
         Vertex3DReadOnly polytopeAVertex1 = simplexVertex1.getVertexOnPolytopeA();
         Vertex3DReadOnly polytopeBVertex1 = simplexVertex1.getVertexOnPolytopeB();
         SimplexVertex3D simplexVertex2 = (SimplexVertex3D) ((HalfEdge3D) member).getDestination();
         Vertex3DReadOnly polytopeAVertex2 = simplexVertex2.getVertexOnPolytopeA();
         Vertex3DReadOnly polytopeBVertex2 = simplexVertex2.getVertexOnPolytopeB();
         double percentage = EuclidGeometryTools.percentageAlongLineSegment3D(point, simplexVertex1, simplexVertex2);
         pointOnA.interpolate(polytopeAVertex1, polytopeAVertex2, percentage);
         pointOnB.interpolate(polytopeBVertex1, polytopeBVertex2, percentage);
      }
      else if (member instanceof SimplexVertex3D)
      {
         // TODO fix this nasty type casting
         pointOnA.set(((SimplexVertex3D) member).getVertexOnPolytopeA());
         pointOnB.set(((SimplexVertex3D) member).getVertexOnPolytopeB());
      }
      else
      {
         throw new RuntimeException("Unhandled simplex member " + member.getClass());
      }
   }
}
