package us.ihmc.euclid.shape.collision.epa;

import static us.ihmc.euclid.shape.collision.gjk.GJKTools.*;

import java.util.ArrayList;
import java.util.List;

import us.ihmc.euclid.Axis;
import us.ihmc.euclid.axisAngle.AxisAngle;
import us.ihmc.euclid.geometry.tools.EuclidGeometryTools;
import us.ihmc.euclid.matrix.RotationMatrix;
import us.ihmc.euclid.shape.collision.gjk.GJKVertex3D;
import us.ihmc.euclid.shape.collision.gjk.GJKTools.ProjectedTriangleSignedAreaCalculator;
import us.ihmc.euclid.shape.collision.interfaces.SupportingVertexHolder;
import us.ihmc.euclid.shape.convexPolytope.tools.EuclidPolytopeTools;
import us.ihmc.euclid.tools.EuclidCoreTools;
import us.ihmc.euclid.tools.TupleTools;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;

public class EPATools
{

   public enum BarycentricCoordinatesOutput
   {
      INSIDE, OUTSIDE, AFFINELY_DEPENDENT
   };

   public static BarycentricCoordinatesOutput barycentricCoordinatesFrom2Simplex(Point3DReadOnly s1, Point3DReadOnly s2, Point3DReadOnly s3, double epsilon,
                                                                       double[] lambdasToPack)
   {
      double s1x = s1.getX(), s1y = s1.getY(), s1z = s1.getZ();
      double s2x = s2.getX(), s2y = s2.getY(), s2z = s2.getZ();
      double s3x = s3.getX(), s3y = s3.getY(), s3z = s3.getZ();

      double p0x, p0y, p0z;
      { // Computing the projection p0 of the origin (0, 0, 0) onto the face as p0 = (s1.n) / (n.n) n
         double s2s1x3D = s1x - s2x;
         double s2s1y3D = s1y - s2y;
         double s2s1z3D = s1z - s2z;

         double s3s1x3D = s1x - s3x;
         double s3s1y3D = s1y - s3y;
         double s3s1z3D = s1z - s3z;

         // Computing the face's normal as n = (s2 - s1) x (s3 - s1)
         double nx = s2s1y3D * s3s1z3D - s2s1z3D * s3s1y3D;
         double ny = s2s1z3D * s3s1x3D - s2s1x3D * s3s1z3D;
         double nz = s2s1x3D * s3s1y3D - s2s1y3D * s3s1x3D;

         double distanceFromPlane = TupleTools.dot(nx, ny, nz, s1) / EuclidCoreTools.normSquared(nx, ny, nz);
         p0x = distanceFromPlane * nx;
         p0y = distanceFromPlane * ny;
         p0z = distanceFromPlane * nz;
      }

      double muMax = 0.0, muMaxAbs = 0.0; // mu = signed area of the triangle (s1, s2, s3)

      // Finding muMax and the coordinates that generate it.
      // As described in the paper, the 3D triangle is projected on each Cartesian plane.
      // The projected 2D triangle with the largest area is selected to pursue further computation.
      double mu, muAbs;
      ProjectedTriangleSignedAreaCalculator calculator = yzTriangleAreaCalculator;

      // Area of the triangle (s1, s2, s3) projected onto the YZ-plane.
      mu = calculator.compute(s1x, s1y, s1z, s2x, s2y, s2z, s3x, s3y, s3z);
      muAbs = Math.abs(mu);

      muMax = mu;
      muMaxAbs = muAbs;

      // Area of the triangle (s1, s2, s3) projected onto the XZ-plane.
      mu = zxTriangleAreaCalculator.compute(s1x, s1y, s1z, s2x, s2y, s2z, s3x, s3y, s3z);
      muAbs = Math.abs(mu);

      if (muAbs > muMaxAbs)
      {
         muMax = mu;
         muMaxAbs = muAbs;
         calculator = zxTriangleAreaCalculator;
      }

      // Area of the triangle (s1, s2, s3) projected onto the XY-plane.
      mu = xyTriangleAreaCalculator.compute(s1x, s1y, s1z, s2x, s2y, s2z, s3x, s3y, s3z);
      muAbs = Math.abs(mu);

      if (muAbs > muMaxAbs)
      {
         muMax = mu;
         muMaxAbs = muAbs;
         calculator = xyTriangleAreaCalculator;
      }

      if (Math.abs(muMax) < epsilon)
         return BarycentricCoordinatesOutput.AFFINELY_DEPENDENT;

      double C1 = calculator.compute(p0x, p0y, p0z, s2x, s2y, s2z, s3x, s3y, s3z); // = area of the triangle (p0, s2, s3) projected onto the selected Cartesian plane.
      double C2 = calculator.compute(s1x, s1y, s1z, p0x, p0y, p0z, s3x, s3y, s3z); // = area of the triangle (s1, p0, s3) projected onto the selected Cartesian plane.
      double C3 = calculator.compute(s1x, s1y, s1z, s2x, s2y, s2z, p0x, p0y, p0z); // = area of the triangle (s1, s2, p0) projected onto the selected Cartesian plane.

      if (compareSigns(muMax, C1) && compareSigns(muMax, C2) && compareSigns(muMax, C3))
      {
         lambdasToPack[0] = C1 / muMax;
         lambdasToPack[1] = C2 / muMax;
         lambdasToPack[2] = C3 / muMax;
         return BarycentricCoordinatesOutput.INSIDE;
      }
      else
      {
         double normSquared = Double.POSITIVE_INFINITY;
         boolean isAlmostInside = true;

         if (compareSigns(muMax, -C1))
         {
            if (Math.abs(C1) > epsilon)
               isAlmostInside = false;

            double[] candidateOutput = barycentricCoordinatesFrom1Simplex(s2, s3);
            double lambda2 = candidateOutput[0];
            double lambda3 = candidateOutput[1];
            p0x = lambda2 * s2x + lambda3 * s3x;
            p0y = lambda2 * s2y + lambda3 * s3y;
            p0z = lambda2 * s2z + lambda3 * s3z;
            double candidateNormSquared = EuclidCoreTools.normSquared(p0x, p0y, p0z);
            lambdasToPack[0] = 0.0;
            lambdasToPack[1] = lambda2;
            lambdasToPack[2] = lambda3;
            normSquared = candidateNormSquared;
         }

         if (compareSigns(muMax, -C2))
         {
            if (Math.abs(C2) > epsilon)
               isAlmostInside = false;

            double[] candidateOutput = barycentricCoordinatesFrom1Simplex(s1, s3);
            double lambda1 = candidateOutput[0];
            double lambda3 = candidateOutput[1];
            p0x = lambda1 * s1x + lambda3 * s3x;
            p0y = lambda1 * s1y + lambda3 * s3y;
            p0z = lambda1 * s1z + lambda3 * s3z;
            double candidateNormSquared = EuclidCoreTools.normSquared(p0x, p0y, p0z);
            if (candidateNormSquared < normSquared)
            {
               lambdasToPack[0] = lambda1;
               lambdasToPack[1] = 0.0;
               lambdasToPack[2] = lambda3;
               normSquared = candidateNormSquared;
            }
         }

         if (compareSigns(muMax, -C3))
         {
            if (Math.abs(C3) > epsilon)
               isAlmostInside = false;

            double[] candidateOutput = barycentricCoordinatesFrom1Simplex(s1, s2);
            double lambda1 = candidateOutput[0];
            double lambda2 = candidateOutput[1];
            p0x = lambda1 * s1x + lambda2 * s2x;
            p0y = lambda1 * s1y + lambda2 * s2y;
            p0z = lambda1 * s1z + lambda2 * s2z;
            double candidateNormSquared = EuclidCoreTools.normSquared(p0x, p0y, p0z);
            if (candidateNormSquared < normSquared)
            {
               lambdasToPack[0] = lambda1;
               lambdasToPack[1] = lambda2;
               lambdasToPack[2] = 0.0;
               normSquared = candidateNormSquared;
            }
         }

         return isAlmostInside ? BarycentricCoordinatesOutput.INSIDE : BarycentricCoordinatesOutput.OUTSIDE;
      }
   }

   public static double[] barycentricCoordinatesFrom1Simplex(Point3DReadOnly s1, Point3DReadOnly s2)
   {
      double s1x = s1.getX(), s1y = s1.getY(), s1z = s1.getZ();
      double s2x = s2.getX(), s2y = s2.getY(), s2z = s2.getZ();

      double p0x, p0y, p0z;
      { // Computing the projection p0 of the origin (0, 0, 0) onto the edge as p0 = s2 - (s2.t) / (t.t) t
        // Computing the edge's direction as t = (s2 - s1)
         double tx = s2x - s1x;
         double ty = s2y - s1y;
         double tz = s2z - s1z;

         double param = -TupleTools.dot(tx, ty, tz, s2) / EuclidCoreTools.normSquared(tx, ty, tz);
         p0x = param * tx + s2x;
         p0y = param * ty + s2y;
         p0z = param * tz + s2z;
      }

      double muMax = 0.0; // mu = length of the edge (s1, s2)
      double s1Max, s2Max, p0Max;

      { // Finding muMax and the coordinates that generate it.
        // Length of the edge (s1, s2) projected onto the X-axis.
         s1Max = s1x;
         s2Max = s2x;
         p0Max = p0x;
         muMax = s1x - s2x;

         // Length of the edge (s1, s2) projected onto the Y-axis.
         double mu = s1y - s2y;

         if (Math.abs(mu) > Math.abs(muMax))
         {
            s1Max = s1y;
            s2Max = s2y;
            p0Max = p0y;
            muMax = mu;
         }

         // Length of the edge (s1, s2) projected onto the Z-axis.
         mu = s1z - s2z;

         if (Math.abs(mu) > Math.abs(muMax))
         {
            s1Max = s1z;
            s2Max = s2z;
            p0Max = p0z;
            muMax = mu;
         }
      }

      double C1 = -(s2Max - p0Max);

      if (compareSigns(muMax, C1))
      {
         double C2 = s1Max - p0Max;

         if (compareSigns(muMax, C2))
         { // The projection in between the edge endpoints. Computing the barycentric coordinates.
            return new double[] {C1 / muMax, C2 / muMax};
         }
         else
         {
            return new double[] {0.0, 1.0};
         }
      }
      else
      {
         return new double[] {1.0, 0.0};
      }
   }

   public static List<EPAFace3D> newEPAPolytopeFromGJKSimplex(SupportingVertexHolder shapeA, SupportingVertexHolder shapeB, GJKVertex3D[] gjkVertices,
                                                              double epsilon)
   {
      List<EPAFace3D> epaPolytope = new ArrayList<>();
   
      if (gjkVertices == null)
      {
         return null;
      }
      else if (gjkVertices.length == 4)
      {
         EPAVertex3D y0 = new EPAVertex3D(gjkVertices[0]);
         EPAVertex3D y1 = new EPAVertex3D(gjkVertices[1]);
         EPAVertex3D y2 = new EPAVertex3D(gjkVertices[2]);
         EPAVertex3D y3 = new EPAVertex3D(gjkVertices[3]);
   
         // Estimate the face's normal based on its vertices and knowing the expecting ordering based on the twin-edge: v1, v2, then v3.
         Vector3D n = EuclidPolytopeTools.crossProductOfLineSegment3Ds(y0, y1, y1, y2);
         // As the vertices are clockwise ordered the cross-product of 2 successive edges should be negated to obtain the face's normal.
         n.negate();
   
         if (EuclidGeometryTools.isPoint3DAbovePlane3D(y3, y0, n))
         {
            EPAFace3D f0 = new EPAFace3D(y3, y0, y1, epsilon);
            if (f0.isTriangleAffinelyDependent())
               return null;
            EPAFace3D f1 = new EPAFace3D(y3, y1, y2, epsilon);
            if (f1.isTriangleAffinelyDependent())
               return null;
            EPAFace3D f2 = new EPAFace3D(y3, y2, y0, epsilon);
            if (f2.isTriangleAffinelyDependent())
               return null;
            EPAFace3D f3 = new EPAFace3D(y0, y2, y1, epsilon);
            if (f3.isTriangleAffinelyDependent())
               return null;
   
            f0.getEdge1().setTwin(f3.getEdge2()); // e01 <-> e10
            f3.getEdge0().setTwin(f2.getEdge1()); // e02 <-> e20
            f2.getEdge2().setTwin(f0.getEdge0()); // e03 <-> e30
   
            f1.getEdge1().setTwin(f3.getEdge1()); // e12 <-> e21
            f0.getEdge2().setTwin(f1.getEdge0()); // e13 <-> e31
            f1.getEdge2().setTwin(f2.getEdge0()); // e23 <-> e32
   
            epaPolytope.add(f0);
            epaPolytope.add(f1);
            epaPolytope.add(f2);
            epaPolytope.add(f3);
         }
         else
         {
            EPAFace3D f0 = new EPAFace3D(y3, y1, y0, epsilon);
            if (f0.isTriangleAffinelyDependent())
               return null;
            EPAFace3D f1 = new EPAFace3D(y3, y2, y1, epsilon);
            if (f1.isTriangleAffinelyDependent())
               return null;
            EPAFace3D f2 = new EPAFace3D(y3, y0, y2, epsilon);
            if (f2.isTriangleAffinelyDependent())
               return null;
            EPAFace3D f3 = new EPAFace3D(y0, y1, y2, epsilon);
            if (f3.isTriangleAffinelyDependent())
               return null;
   
            f3.getEdge0().setTwin(f0.getEdge1()); // e01 <-> e10
            f2.getEdge1().setTwin(f3.getEdge2()); // e02 <-> e20
            f0.getEdge2().setTwin(f2.getEdge0()); // e03 <-> e30
   
            f3.getEdge1().setTwin(f1.getEdge1()); // e12 <-> e21
            f1.getEdge2().setTwin(f0.getEdge0()); // e13 <-> e31
            f2.getEdge2().setTwin(f1.getEdge0()); // e23 <-> e32
   
            epaPolytope.add(f0);
            epaPolytope.add(f1);
            epaPolytope.add(f2);
            epaPolytope.add(f3);
         }
      }
      else if (gjkVertices.length == 3)
      {
         EPAVertex3D y0 = new EPAVertex3D(gjkVertices[0]);
         EPAVertex3D y1 = new EPAVertex3D(gjkVertices[1]);
         EPAVertex3D y2 = new EPAVertex3D(gjkVertices[2]);
   
         // Estimate the face's normal based on its vertices and knowing the expecting ordering based on the twin-edge: v1, v2, then v3.
         Vector3D n = EuclidPolytopeTools.crossProductOfLineSegment3Ds(y0, y1, y1, y2);
         // As the vertices are clockwise ordered the cross-product of 2 successive edges should be negated to obtain the face's normal.
         n.negate();
   
         Point3DReadOnly vertexA, vertexB;
         EPAVertex3D y3, y4;
         vertexA = shapeA.getSupportingVertex(n);
         n.negate();
         vertexB = shapeB.getSupportingVertex(n);
         y3 = new EPAVertex3D(vertexA, vertexB);
         vertexA = shapeA.getSupportingVertex(n);
         n.negate();
         vertexB = shapeB.getSupportingVertex(n);
         y4 = new EPAVertex3D(vertexA, vertexB);
   
         if (EuclidPolytopeTools.tetrahedronContainsOrigin(y0, y1, y2, y3))
         {
            EPAFace3D f0 = new EPAFace3D(y3, y0, y1, epsilon);
            if (f0.isTriangleAffinelyDependent())
               return null;
            EPAFace3D f1 = new EPAFace3D(y3, y1, y2, epsilon);
            if (f1.isTriangleAffinelyDependent())
               return null;
            EPAFace3D f2 = new EPAFace3D(y3, y2, y0, epsilon);
            if (f2.isTriangleAffinelyDependent())
               return null;
            EPAFace3D f3 = new EPAFace3D(y0, y2, y1, epsilon);
            if (f3.isTriangleAffinelyDependent())
               return null;
   
            f0.getEdge1().setTwin(f3.getEdge2()); // e01 <-> e10
            f3.getEdge0().setTwin(f2.getEdge1()); // e02 <-> e20
            f2.getEdge2().setTwin(f0.getEdge0()); // e03 <-> e30
   
            f1.getEdge1().setTwin(f3.getEdge1()); // e12 <-> e21
            f0.getEdge2().setTwin(f1.getEdge0()); // e13 <-> e31
            f1.getEdge2().setTwin(f2.getEdge0()); // e23 <-> e32
   
            epaPolytope.add(f0);
            epaPolytope.add(f1);
            epaPolytope.add(f2);
            epaPolytope.add(f3);
         }
         else if (EuclidPolytopeTools.tetrahedronContainsOrigin(y0, y1, y2, y4))
         {
            EPAFace3D f0 = new EPAFace3D(y4, y1, y0, epsilon);
            if (f0.isTriangleAffinelyDependent())
               return null;
            EPAFace3D f1 = new EPAFace3D(y4, y2, y1, epsilon);
            if (f1.isTriangleAffinelyDependent())
               return null;
            EPAFace3D f2 = new EPAFace3D(y4, y0, y2, epsilon);
            if (f2.isTriangleAffinelyDependent())
               return null;
            EPAFace3D f3 = new EPAFace3D(y0, y1, y2, epsilon);
            if (f3.isTriangleAffinelyDependent())
               return null;
   
            f3.getEdge0().setTwin(f0.getEdge1()); // e01 <-> e10
            f2.getEdge1().setTwin(f3.getEdge2()); // e02 <-> e20
            f0.getEdge2().setTwin(f2.getEdge0()); // e04 <-> e40
   
            f3.getEdge1().setTwin(f1.getEdge1()); // e12 <-> e21
            f1.getEdge2().setTwin(f0.getEdge0()); // e14 <-> e41
            f2.getEdge2().setTwin(f1.getEdge0()); // e24 <-> e42
   
            epaPolytope.add(f0);
            epaPolytope.add(f1);
            epaPolytope.add(f2);
            epaPolytope.add(f3);
         }
         else
         {
            EPAFace3D f0 = new EPAFace3D(y4, y1, y0, epsilon);
            if (f0.isTriangleAffinelyDependent())
               return null;
            EPAFace3D f1 = new EPAFace3D(y4, y2, y1, epsilon);
            if (f1.isTriangleAffinelyDependent())
               return null;
            EPAFace3D f2 = new EPAFace3D(y4, y0, y2, epsilon);
            if (f2.isTriangleAffinelyDependent())
               return null;
            EPAFace3D f3 = new EPAFace3D(y3, y0, y1, epsilon);
            if (f3.isTriangleAffinelyDependent())
               return null;
            EPAFace3D f4 = new EPAFace3D(y3, y1, y2, epsilon);
            if (f4.isTriangleAffinelyDependent())
               return null;
            EPAFace3D f5 = new EPAFace3D(y3, y2, y0, epsilon);
            if (f5.isTriangleAffinelyDependent())
               return null;
   
            f3.getEdge0().setTwin(f5.getEdge2()); // e30 <-> e03
            f4.getEdge0().setTwin(f3.getEdge2()); // e31 <-> e13
            f5.getEdge0().setTwin(f4.getEdge2()); // e32 <-> e23
   
            f2.getEdge0().setTwin(f0.getEdge2()); // e40 <-> e04
            f0.getEdge0().setTwin(f1.getEdge2()); // e41 <-> e14
            f1.getEdge0().setTwin(f2.getEdge2()); // e42 <-> e24
   
            f3.getEdge1().setTwin(f0.getEdge1()); // e01 <-> e10
            f2.getEdge1().setTwin(f5.getEdge1()); // e02 <-> e20
            f4.getEdge1().setTwin(f1.getEdge1()); // e12 <-> e21
   
            epaPolytope.add(f0);
            epaPolytope.add(f1);
            epaPolytope.add(f2);
            epaPolytope.add(f3);
            epaPolytope.add(f4);
            epaPolytope.add(f5);
         }
      }
      else if (gjkVertices.length == 2)
      {
         EPAVertex3D y0 = new EPAVertex3D(gjkVertices[0]);
         EPAVertex3D y1 = new EPAVertex3D(gjkVertices[1]);
   
         Vector3D d = new Vector3D();
         d.sub(y1, y0);
   
         Vector3DReadOnly axis = Axis.X;
         double coord = Math.abs(d.getX());
         double yAbs = Math.abs(d.getY());
         double zAbs = Math.abs(d.getZ());
   
         if (yAbs > coord)
         {
            coord = yAbs;
            axis = Axis.Y;
         }
         if (zAbs > coord)
         {
            axis = Axis.Z;
         }
   
         Vector3D v1 = new Vector3D();
         v1.cross(d, axis);
         RotationMatrix r = new RotationMatrix(new AxisAngle(d, 2.0 / 3.0 * Math.PI));
         Vector3D v2 = new Vector3D();
         Vector3D v3 = new Vector3D();
         r.transform(v1, v2);
         r.transform(v2, v3);
   
         Point3DReadOnly vertexA, vertexB;
         EPAVertex3D y2, y3, y4;
         vertexA = shapeA.getSupportingVertex(v1);
         v1.negate();
         vertexB = shapeB.getSupportingVertex(v1);
         y2 = new EPAVertex3D(vertexA, vertexB);
         vertexA = shapeA.getSupportingVertex(v2);
         v2.negate();
         vertexB = shapeB.getSupportingVertex(v2);
         y3 = new EPAVertex3D(vertexA, vertexB);
         vertexA = shapeA.getSupportingVertex(v3);
         v3.negate();
         vertexB = shapeB.getSupportingVertex(v3);
         y4 = new EPAVertex3D(vertexA, vertexB);
   
         if (EuclidPolytopeTools.tetrahedronContainsOrigin(y0, y2, y3, y4))
         {
            // Building the faces such that clockwise winding
            EPAFace3D f0 = new EPAFace3D(y0, y2, y3, epsilon);
            if (f0.isTriangleAffinelyDependent())
               return null;
            EPAFace3D f1 = new EPAFace3D(y0, y3, y4, epsilon);
            if (f1.isTriangleAffinelyDependent())
               return null;
            EPAFace3D f2 = new EPAFace3D(y0, y4, y2, epsilon);
            if (f2.isTriangleAffinelyDependent())
               return null;
            EPAFace3D f3 = new EPAFace3D(y2, y4, y3, epsilon);
            if (f3.isTriangleAffinelyDependent())
               return null;
   
            f0.getEdge0().setTwin(f2.getEdge2()); // e02 <-> e20
            f1.getEdge0().setTwin(f0.getEdge2()); // e03 <-> e30
            f2.getEdge0().setTwin(f1.getEdge2()); // e04 <-> e40
   
            f0.getEdge1().setTwin(f3.getEdge2()); // e23 <-> e32
            f3.getEdge0().setTwin(f2.getEdge1()); // e24 <-> e42
            f1.getEdge1().setTwin(f3.getEdge1()); // e34 <-> e43
   
            epaPolytope.add(f0);
            epaPolytope.add(f1);
            epaPolytope.add(f2);
            epaPolytope.add(f3);
         }
         else if (EuclidPolytopeTools.tetrahedronContainsOrigin(y1, y2, y3, y4))
         {
            // Building the faces such that clockwise winding
            EPAFace3D f0 = new EPAFace3D(y1, y3, y2, epsilon);
            if (f0.isTriangleAffinelyDependent())
               return null;
            EPAFace3D f1 = new EPAFace3D(y1, y4, y3, epsilon);
            if (f1.isTriangleAffinelyDependent())
               return null;
            EPAFace3D f2 = new EPAFace3D(y1, y2, y4, epsilon);
            if (f2.isTriangleAffinelyDependent())
               return null;
            EPAFace3D f3 = new EPAFace3D(y2, y3, y4, epsilon);
            if (f3.isTriangleAffinelyDependent())
               return null;
   
            f2.getEdge0().setTwin(f0.getEdge2()); // e12 <-> e21
            f0.getEdge0().setTwin(f1.getEdge2()); // e13 <-> e31
            f1.getEdge0().setTwin(f2.getEdge2()); // e14 <-> e41
   
            f3.getEdge0().setTwin(f0.getEdge1()); // e23 <-> e32
            f2.getEdge1().setTwin(f3.getEdge2()); // e24 <-> e42
            f3.getEdge1().setTwin(f1.getEdge1()); // e34 <-> e43
   
            epaPolytope.add(f0);
            epaPolytope.add(f1);
            epaPolytope.add(f2);
            epaPolytope.add(f3);
         }
         else
         {
            EPAFace3D f0 = new EPAFace3D(y0, y2, y3, epsilon);
            if (f0.isTriangleAffinelyDependent())
               return null;
            EPAFace3D f1 = new EPAFace3D(y0, y3, y4, epsilon);
            if (f1.isTriangleAffinelyDependent())
               return null;
            EPAFace3D f2 = new EPAFace3D(y0, y4, y2, epsilon);
            if (f2.isTriangleAffinelyDependent())
               return null;
            EPAFace3D f3 = new EPAFace3D(y1, y3, y2, epsilon);
            if (f3.isTriangleAffinelyDependent())
               return null;
            EPAFace3D f4 = new EPAFace3D(y1, y4, y3, epsilon);
            if (f4.isTriangleAffinelyDependent())
               return null;
            EPAFace3D f5 = new EPAFace3D(y1, y2, y4, epsilon);
            if (f5.isTriangleAffinelyDependent())
               return null;
   
            f0.getEdge0().setTwin(f2.getEdge2()); // e02 <-> e20
            f1.getEdge0().setTwin(f0.getEdge2()); // e03 <-> e30
            f2.getEdge0().setTwin(f1.getEdge2()); // e04 <-> e40
   
            f5.getEdge0().setTwin(f3.getEdge2()); // e12 <-> e21
            f3.getEdge0().setTwin(f4.getEdge2()); // e13 <-> e31
            f4.getEdge0().setTwin(f5.getEdge2()); // e14 <-> e41
   
            f0.getEdge1().setTwin(f3.getEdge1()); // e23 <-> e32
            f5.getEdge1().setTwin(f2.getEdge1()); // e24 <-> e42
            f1.getEdge1().setTwin(f4.getEdge1()); // e34 <-> e43
   
            f0.getEdge1().setTwin(f3.getEdge1()); // e23 <-> e32
            f5.getEdge1().setTwin(f2.getEdge1()); // e24 <-> e42
            f1.getEdge1().setTwin(f4.getEdge1()); // e34 <-> e43
   
            epaPolytope.add(f0);
            epaPolytope.add(f1);
            epaPolytope.add(f2);
            epaPolytope.add(f3);
            epaPolytope.add(f4);
            epaPolytope.add(f5);
         }
      }
      else if (gjkVertices.length == 1)
      {
         // Supposedly this case only occurs when 2 shapes are only touching with 0-depth.
         return null;
      }
   
      return epaPolytope;
   }

   public static void silhouette(EPAEdge3D edge, Point3DReadOnly observer, List<EPAEdge3D> silhouetteToPack)
   {
      if (edge == null || edge.getFace().isObsolete())
         return;
   
      if (!edge.getFace().canObserverSeeFace(observer))
      {
         silhouetteToPack.add(edge);
      }
      else
      {
         edge.getFace().markObsolete();
         silhouette(edge.getNext().getTwin(), observer, silhouetteToPack);
         silhouette(edge.getPrevious().getTwin(), observer, silhouetteToPack);
      }
   }
}
