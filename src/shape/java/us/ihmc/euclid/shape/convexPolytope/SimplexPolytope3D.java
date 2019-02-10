package us.ihmc.euclid.shape.convexPolytope;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

import org.ejml.data.DenseMatrix64F;

import us.ihmc.euclid.geometry.tools.EuclidGeometryTools;
import us.ihmc.euclid.shape.convexPolytope.interfaces.Simplex3D;
import us.ihmc.euclid.shape.convexPolytope.interfaces.Vertex3DReadOnly;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DBasics;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DBasics;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DBasics;

public class SimplexPolytope3D implements Simplex3D
{
   private ConvexPolytope3D polytope = new ConvexPolytope3D();
   private List<SimplexVertex3D> vertices = new ArrayList<>();
   private final Point3D projection = new Point3D();
   private final DenseMatrix64F verticesMatrix = new DenseMatrix64F(3, 3);
   private final DenseMatrix64F projectionMatrix = new DenseMatrix64F(3, 1);
   private final DenseMatrix64F weigths = new DenseMatrix64F(3, 1);

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
      polytope.getFaces().forEach(face -> {
         Vector3D normalExpected = new Vector3D();
         normalExpected.sub(face.getCentroid(), polytope.getCentroid());
         assert face.getNormal().dot(normalExpected) > 0.0;
         face.updateNormal();
         face.updateCentroidAndArea();
         face.updateVertices();
         
         assert face.getNormal().dot(normalExpected) > 0.0;
      });

      Simplex3D member = getSmallestSimplexMemberReference(point);
      // Assuming linearity between the simplex and polytope points
      if (member instanceof Face3D)
      {
         Face3D face = (Face3D) member;

         SimplexVertex3D simplexVertex1 = (SimplexVertex3D) face.getEdge(0).getOrigin();
         SimplexVertex3D simplexVertex2 = (SimplexVertex3D) face.getEdge(1).getOrigin();
         SimplexVertex3D simplexVertex3 = (SimplexVertex3D) face.getEdge(2).getOrigin();

         Vertex3DReadOnly polytopeAVertex1 = simplexVertex1.getVertexOnPolytopeA();
         Vertex3DReadOnly polytopeAVertex2 = simplexVertex2.getVertexOnPolytopeA();
         Vertex3DReadOnly polytopeAVertex3 = simplexVertex3.getVertexOnPolytopeA();

         Vertex3DReadOnly polytopeBVertex1 = simplexVertex1.getVertexOnPolytopeB();
         Vertex3DReadOnly polytopeBVertex2 = simplexVertex2.getVertexOnPolytopeB();
         Vertex3DReadOnly polytopeBVertex3 = simplexVertex3.getVertexOnPolytopeB();

         Point3D testPoint = new Point3D();
         testPoint.sub(polytopeAVertex1, polytopeBVertex1);
         assert testPoint.epsilonEquals(simplexVertex1, 1.0e-12);
         testPoint.sub(polytopeAVertex2, polytopeBVertex2);
         assert testPoint.epsilonEquals(simplexVertex2, 1.0e-12);
         testPoint.sub(polytopeAVertex3, polytopeBVertex3);
         assert testPoint.epsilonEquals(simplexVertex3, 1.0e-12);

         double[] lambdas = new double[3];
         projectOriginOntoFace(simplexVertex1, simplexVertex2, simplexVertex3, projection, lambdas);
         System.out.println(Arrays.toString(lambdas));

         pointOnA.setAndScale(lambdas[0], polytopeAVertex1);
         pointOnA.scaleAdd(lambdas[1], polytopeAVertex2, pointOnA);
         pointOnA.scaleAdd(lambdas[2], polytopeAVertex3, pointOnA);

         pointOnB.setAndScale(lambdas[0], polytopeBVertex1);
         pointOnB.scaleAdd(lambdas[1], polytopeBVertex2, pointOnB);
         pointOnB.scaleAdd(lambdas[2], polytopeBVertex3, pointOnB);

         //         // Computing the coordinate vector for the face basis (using the first two edges as the basis)
         //         EuclidGeometryTools.orthogonalProjectionOnPlane3D(point, face.getCentroid(), face.getNormal(), projection);
         //         assert projection.epsilonEquals(polytope.getClosestFace(point).orthogonalProjection(point), 1.0e-12);
         //         assert face.isPointDirectlyAboveOrBelow(point);
         //
         //         simplexVertex1.get(0, 0, verticesMatrix);
         //         simplexVertex2.get(0, 1, verticesMatrix);
         //         simplexVertex3.get(0, 2, verticesMatrix);
         //         projection.get(projectionMatrix);
         //         UnrolledInverseFromMinor.inv3(verticesMatrix, verticesMatrix, 1.0);
         //         CommonOps.mult(verticesMatrix, projectionMatrix, weigths);
         //
         //         System.out.println(new Point3D(weigths.data));
         //
         //         pointOnA.setAndScale(weigths.get(0), polytopeAVertex1);
         //         pointOnA.scaleAdd(weigths.get(1), polytopeAVertex2, pointOnA);
         //         pointOnA.scaleAdd(weigths.get(2), polytopeAVertex3, pointOnA);
         //
         //         pointOnB.setAndScale(weigths.get(0), polytopeBVertex1);
         //         pointOnB.scaleAdd(weigths.get(1), polytopeBVertex2, pointOnB);
         //         pointOnB.scaleAdd(weigths.get(2), polytopeBVertex3, pointOnB);
         //
         //         Point3D testPoint = new Point3D();
         //         testPoint.sub(pointOnA, pointOnB);
         //         assert testPoint.epsilonEquals(projection, 1.0e-12);
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

   private final Vector3D u = new Vector3D();
   private final Vector3D v = new Vector3D();
   private final Vector3D tempVector3 = new Vector3D();
   private final Vector3D tempVector4 = new Vector3D();
   private final Vector3D tempNormalVector1 = new Vector3D();

   private void projectOriginOntoFace(Point3DReadOnly vertexOne, Point3DReadOnly vertexTwo, Point3DReadOnly vertexThree, Tuple3DBasics closestPointToOrigin,
                                      double[] lambdas)
   {
      // Using barycentric coordinates as described in https://www.cs.ubc.ca/~heidrich/Papers/JGT.05.pdf
      u.sub(vertexTwo, vertexOne);
      v.sub(vertexThree, vertexOne);

      tempNormalVector1.cross(u, v);
      double fourASquared = tempNormalVector1.dot(tempNormalVector1);

      //TODO: Magic number for checking affinely Dependent...
      // Probably a better way to check this than just the area of the triangle.
      // Something relative. 
      //      this.affinelyDependent = fourASquared < 1e-10;

      double oneOver4ASquared = 1.0 / (fourASquared);

      tempVector3.set(vertexOne);
      tempVector3.scale(-1.0); //w

      tempVector4.cross(u, tempVector3);
      double lambdaThree = tempVector4.dot(tempNormalVector1) * oneOver4ASquared;

      tempVector4.cross(tempVector3, v);
      double lambdaTwo = tempVector4.dot(tempNormalVector1) * oneOver4ASquared;

      double lambdaOne = 1.0 - lambdaTwo - lambdaThree;

      lambdas[0] = lambdaOne;
      lambdas[1] = lambdaTwo;
      lambdas[2] = lambdaThree;

      closestPointToOrigin.set(0.0, 0.0, 0.0);

      u.set(vertexOne);
      u.scale(lambdaOne);
      closestPointToOrigin.add(u);

      u.set(vertexTwo);
      u.scale(lambdaTwo);
      closestPointToOrigin.add(u);

      u.set(vertexThree);
      u.scale(lambdaThree);
      closestPointToOrigin.add(u);
   }
}
