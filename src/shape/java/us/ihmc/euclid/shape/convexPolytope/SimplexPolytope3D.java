package us.ihmc.euclid.shape.convexPolytope;

import java.util.ArrayList;
import java.util.List;

import org.ejml.data.DenseMatrix64F;
import org.ejml.ops.CommonOps;

import us.ihmc.euclid.geometry.tools.EuclidGeometryTools;
import us.ihmc.euclid.shape.convexPolytope.interfaces.Simplex3D;
import us.ihmc.euclid.shape.convexPolytope.interfaces.Vertex3DReadOnly;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DBasics;

public class SimplexPolytope3D implements Simplex3D
{
   private double epsilon = 1.0e-12;
   private ConvexPolytope3D polytope = new ConvexPolytope3D();
   private List<SimplexVertex3D> vertices = new ArrayList<>();
   private final Vector3D basisVector1 = new Vector3D();
   private final Vector3D basisVector2 = new Vector3D();
   private final Vector3D pointVector = new Vector3D();
   private final Point3D projection = new Point3D();
   private final DenseMatrix64F basis = new DenseMatrix64F(3, 2);
   private final DenseMatrix64F basisInverse = new DenseMatrix64F(2, 3);
   private final DenseMatrix64F vector = new DenseMatrix64F(3, 1);
   private final DenseMatrix64F coordinates = new DenseMatrix64F(2, 1);

   public SimplexPolytope3D()
   {
      super();
   }

   public void setEpsilon(double epsilon)
   {
      this.epsilon = epsilon;
   }

   public void addVertex(Vertex3DReadOnly vertexOnPolytopeA, Vertex3DReadOnly vertexOnPolytopeB)
   {
      addVertex(vertexOnPolytopeA, vertexOnPolytopeB, epsilon);
   }

   public void addVertex(Vertex3DReadOnly vertexOnPolytopeA, Vertex3DReadOnly vertexOnPolytopeB, double epsilon)
   {
      SimplexVertex3D newVertex = new SimplexVertex3D();
      newVertex.set(vertexOnPolytopeA, vertexOnPolytopeB);
      polytope.addVertex(newVertex, epsilon);
   }

   public void clear()
   {
      vertices.clear();
      polytope.clear();
   }

   public boolean isInteriorPoint(Point3DReadOnly pointToCheck, double epsilon)
   {
      return polytope.isInteriorPoint(pointToCheck, epsilon);
   }

   @Override
   public double distance(Point3DReadOnly point)
   {
      return polytope.distance(point);
   }

   @Override
   public void getSupportVectorDirectionTo(Point3DReadOnly point, Vector3DBasics supportVectorToPack)
   {
      polytope.getSupportVectorDirectionTo(point, supportVectorToPack);
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

   public void getCollidingPointsOnSimplex(Point3DReadOnly point, Point3D pointOnA, Point3D pointOnB)
   {
      Simplex3D member = getSmallestSimplexMemberReference(point);
      // Assuming linearity between the simplex and polytope points
      if (member instanceof Face3D)
      {
         // TODO fix this nasty type casting
         SimplexVertex3D simplexVertex1 = (SimplexVertex3D) ((Face3D) member).getEdge(0).getOrigin();
         Vertex3DReadOnly polytopeAVertex1 = simplexVertex1.getVertexOnPolytopeA();
         Vertex3DReadOnly polytopeBVertex1 = simplexVertex1.getVertexOnPolytopeB();
         SimplexVertex3D simplexVertex2 = (SimplexVertex3D) ((Face3D) member).getEdge(0).getDestination();
         Vertex3DReadOnly polytopeAVertex2 = simplexVertex2.getVertexOnPolytopeA();
         Vertex3DReadOnly polytopeBVertex2 = simplexVertex2.getVertexOnPolytopeB();
         SimplexVertex3D simplexVertex3 = (SimplexVertex3D) ((Face3D) member).getEdge(1).getDestination();
         Vertex3DReadOnly polytopeAVertex3 = simplexVertex3.getVertexOnPolytopeA();
         Vertex3DReadOnly polytopeBVertex3 = simplexVertex3.getVertexOnPolytopeB();

         // Computing the coordinate vector for the face basis (using the first two edges as the basis)
         EuclidGeometryTools.orthogonalProjectionOnPlane3D(point, simplexVertex2, ((Face3D) member).getFaceNormal(), projection);
         for (int i = 0; i < 3; i++)
         {
            basis.set(i, 0, simplexVertex1.getElement(i) - simplexVertex2.getElement(i));
            basis.set(i, 1, simplexVertex3.getElement(i) - simplexVertex2.getElement(i));
            vector.set(i, 0, projection.getElement(i) - simplexVertex2.getElement(i));
         }
         CommonOps.pinv(basis, basisInverse);
         CommonOps.mult(basisInverse, vector, coordinates);
         setByInterpolation(pointOnA, polytopeAVertex1, polytopeAVertex2, polytopeAVertex3, coordinates.get(0, 0), coordinates.get(1, 0));
         setByInterpolation(pointOnB, polytopeBVertex1, polytopeBVertex2, polytopeBVertex3, coordinates.get(0, 0), coordinates.get(1, 0));
      }
      else if (member instanceof HalfEdge3D)
      {
         // TODO fix this nasty type casting
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

   private void setByInterpolation(Point3D pointOnA, Vertex3DReadOnly polytopeAVertex1, Vertex3DReadOnly polytopeAVertex2, Vertex3DReadOnly polytopeAVertex3,
                                   double a, double b)
   {
      basisVector1.sub(polytopeAVertex1, polytopeAVertex2);
      basisVector2.sub(polytopeAVertex3, polytopeAVertex2);
      pointVector.setAndScale(a, basisVector1);
      pointVector.scaleAdd(b, basisVector2, pointVector);
      pointOnA.add(pointVector, polytopeAVertex2);
   }
}
