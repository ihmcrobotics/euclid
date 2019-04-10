package us.ihmc.euclid.shape.collision;

import us.ihmc.euclid.Axis;
import us.ihmc.euclid.shape.collision.interfaces.EuclidShape3DCollisionResultBasics;
import us.ihmc.euclid.shape.collision.interfaces.SupportingVertexHolder;
import us.ihmc.euclid.shape.convexPolytope.tools.EuclidPolytopeTools;
import us.ihmc.euclid.shape.primitives.interfaces.Shape3DReadOnly;
import us.ihmc.euclid.tools.EuclidCoreIOTools;
import us.ihmc.euclid.tools.EuclidCoreTools;
import us.ihmc.euclid.tools.EuclidHashCodeTools;
import us.ihmc.euclid.tools.Matrix3DFeatures;
import us.ihmc.euclid.tools.TupleTools;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DBasics;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;

// From "Improving the GJK algorithm for faster and more reliable distance queries between convex
// objects."
// TODO Make the initial supportDirection be settable to reduce number of iterations when possible.
public class GilbertJohnsonKeerthiCollisionDetector
{
   private static final boolean VERBOSE = false;
   private double epsilon = 1.0e-16;
   private double epsilonTriangleNormalSwitch = 1.0e-6;
   private int maxIterations = 500;
   private GJKSimplex3D simplex = null;
   private int numberOfIterations = 0;
   private final Vector3D supportDirection = new Vector3D();
   private final Vector3D initialSupportDirection = new Vector3D(Axis.Y);

   public GilbertJohnsonKeerthiCollisionDetector()
   {
   }

   public EuclidShape3DCollisionResult evaluateCollision(Shape3DReadOnly shapeA, Shape3DReadOnly shapeB)
   {
      EuclidShape3DCollisionResult result = new EuclidShape3DCollisionResult();
      evaluateCollision(shapeA, shapeB, result);
      return result;
   }

   public void evaluateCollision(Shape3DReadOnly shapeA, Shape3DReadOnly shapeB, EuclidShape3DCollisionResult resultToPack)
   {
      evaluateCollision(shapeA, shapeB, resultToPack);
      resultToPack.setShapeA(shapeA);
      resultToPack.setShapeB(shapeB);
   }

   public EuclidShape3DCollisionResult evaluateCollision(SupportingVertexHolder shapeA, SupportingVertexHolder shapeB)
   {
      EuclidShape3DCollisionResult result = new EuclidShape3DCollisionResult();
      evaluateCollision(shapeA, shapeB, result);
      return result;
   }

   public boolean evaluateCollision(SupportingVertexHolder shapeA, SupportingVertexHolder shapeB, EuclidShape3DCollisionResultBasics resultToPack)
   {
      GJKSimplex3D previousOutput = new GJKSimplex3D();

      supportDirection.set(initialSupportDirection);
      Point3DReadOnly vertexA = shapeA.getSupportingVertex(supportDirection);
      supportDirection.negate();
      Point3DReadOnly vertexB = shapeB.getSupportingVertex(supportDirection);

      boolean areColliding = false;

      if (vertexA == null || vertexB == null)
      {
         simplex = null;
         areColliding = false;
      }
      else
      {
         double closestPointNormSquared = 1.0;

         for (int i = 0; i < maxIterations; i++)
         {
            numberOfIterations = i;
            GJKVertex3D newVertex = new GJKVertex3D(vertexA, vertexB);

            if (previousOutput.contains(newVertex))
            {
               simplex = previousOutput;
               areColliding = false;
               if (VERBOSE)
                  System.out.println("New vertex belongs to simplex, terminating.");
               break;
            }

            if (Math.abs(closestPointNormSquared - newVertex.dot(supportDirection)) <= epsilon * closestPointNormSquared)
            {
               simplex = previousOutput;
               areColliding = false;
               if (VERBOSE)
                  System.out.println("Progression under tolerance, terminating.");
               break;
            }

            GJKSimplex3D output = nextBestSimplex(previousOutput.vertices, newVertex);

            if (output == null)
            { // End of process
               simplex = previousOutput;
               areColliding = false;
               if (VERBOSE)
                  System.out.println("Closest simplex is null, terminating.");
               break;
            }

            if (output.closestPointNormSquared() >= previousOutput.closestPointNormSquared())
            {
               simplex = previousOutput;
               areColliding = false;
               if (VERBOSE)
                  System.out.println("No progression, terminating.");
               break;
            }

            closestPointNormSquared = output.closestPointNormSquared();

            if (output.size() == 4 || closestPointNormSquared <= epsilon * output.maxNormSquared())
            { // End of process
               simplex = output;
               areColliding = true;
               if (VERBOSE)
                  System.out.println("Collision detected, terminating.");
               break;
            }

            if (closestPointNormSquared < epsilonTriangleNormalSwitch && output.vertices.length == 3)
               supportDirection.set(output.getTriangleNormal());
            else
               supportDirection.setAndNegate(output.closestPoint());
            vertexA = shapeA.getSupportingVertex(supportDirection);
            supportDirection.negate();
            vertexB = shapeB.getSupportingVertex(supportDirection);

            previousOutput = output;
         }
      }

      if (areColliding)
      {
         resultToPack.setToNaN();
         resultToPack.setShapesAreColliding(areColliding);
      }
      else if (simplex == null)
      {
         resultToPack.setToNaN();
      }
      else
      {
         resultToPack.setDistance(simplex.closestPointNorm());
         resultToPack.setShapeA(null);
         resultToPack.setShapeB(null);
         simplex.computePointOnA(resultToPack.getPointOnA());
         simplex.computePointOnB(resultToPack.getPointOnB());
         resultToPack.getNormalOnA().setToNaN();
         resultToPack.getNormalOnB().setToNaN();
      }

      if (VERBOSE)
         System.out.println("Number of iterations: " + numberOfIterations);

      return areColliding;
   }

   public void setInitialSupportDirection(Vector3DReadOnly initialSupportDirection)
   {
      this.initialSupportDirection.set(initialSupportDirection);
   }

   public void setMaxIterations(int maxIterations)
   {
      this.maxIterations = maxIterations;
   }

   public void setTerminalConditionEpsilon(double epsilon)
   {
      this.epsilon = epsilon;
   }

   public double getTerminalConditionEpsilon()
   {
      return epsilon;
   }

   public int getNumberOfIterations()
   {
      return numberOfIterations;
   }

   public GJKSimplex3D getSimplex()
   {
      return simplex;
   }

   public GJKVertex3D[] getSimplexVertices()
   {
      if (simplex == null)
         return null;
      else
         return simplex.vertices;
   }

   private static GJKSimplex3D nextBestSimplex(GJKVertex3D[] oldVertices, GJKVertex3D newVertex)
   {
      if (oldVertices.length == 3)
         return nextBestSimplexFrom3Simplex(newVertex, oldVertices[2], oldVertices[1], oldVertices[0]);
      else if (oldVertices.length == 2)
         return nextBestSimplexFrom2Simplex(newVertex, oldVertices[1], oldVertices[0]);
      else if (oldVertices.length == 1)
         return nextBestSimplexFrom1Simplex(newVertex, oldVertices[0]);
      else
         return new GJKSimplex3D(newVertex);
   }

   private static GJKSimplex3D nextBestSimplexFrom3Simplex(GJKVertex3D s1, GJKVertex3D s2, GJKVertex3D s3, GJKVertex3D s4)
   {
      double s1x = s1.getX(), s1y = s1.getY(), s1z = s1.getZ();
      double s2x = s2.getX(), s2y = s2.getY(), s2z = s2.getZ();
      double s3x = s3.getX(), s3y = s3.getY(), s3z = s3.getZ();
      double s4x = s4.getX(), s4y = s4.getY(), s4z = s4.getZ();

      double C41 = -Matrix3DFeatures.determinant(s2x, s3x, s4x, s2y, s3y, s4y, s2z, s3z, s4z);
      double C42 = +Matrix3DFeatures.determinant(s1x, s3x, s4x, s1y, s3y, s4y, s1z, s3z, s4z);
      double C43 = -Matrix3DFeatures.determinant(s1x, s2x, s4x, s1y, s2y, s4y, s1z, s2z, s4z);
      double C44 = +Matrix3DFeatures.determinant(s1x, s2x, s3x, s1y, s2y, s3y, s1z, s2z, s3z);
      double detM = C41 + C42 + C43 + C44;

      if (compareSigns(detM, C41) && compareSigns(detM, C42) && compareSigns(detM, C43) && compareSigns(detM, C44))
      {
         double[] lambdas = {C41 / detM, C42 / detM, C43 / detM, C44 / detM};
         GJKVertex3D[] supportVertices = {s1, s2, s3, s4};
         return new GJKSimplex3D(supportVertices, lambdas);
      }
      else
      {
         double d = Double.POSITIVE_INFINITY;
         GJKSimplex3D output = null;

         if (compareSigns(detM, -C42))
         {
            GJKSimplex3D candidateOutput = nextBestSimplexFrom2Simplex(s1, s3, s4);
            if (candidateOutput != null)
            {
               double candidateNorm = candidateOutput.closestPointNormSquared();
               if (candidateNorm < d)
               {
                  output = candidateOutput;
                  d = candidateNorm;
               }
            }
         }

         if (compareSigns(detM, -C43))
         {
            GJKSimplex3D candidateOutput = nextBestSimplexFrom2Simplex(s1, s2, s4);
            if (candidateOutput != null)
            {
               double candidateNorm = candidateOutput.closestPointNormSquared();
               if (candidateNorm < d)
               {
                  output = candidateOutput;
                  d = candidateNorm;
               }
            }
         }

         if (compareSigns(detM, -C44))
         {
            GJKSimplex3D candidateOutput = nextBestSimplexFrom2Simplex(s1, s2, s3);
            if (candidateOutput != null)
            {
               double candidateNorm = candidateOutput.closestPointNormSquared();
               if (candidateNorm < d)
               {
                  output = candidateOutput;
                  d = candidateNorm;
               }
            }
         }

         return output;
      }
   }

   private static GJKSimplex3D nextBestSimplexFrom2Simplex(GJKVertex3D s1, GJKVertex3D s2, GJKVertex3D s3)
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

      double C1 = calculator.compute(p0x, p0y, p0z, s2x, s2y, s2z, s3x, s3y, s3z); // = area of the triangle (p0, s2, s3) projected onto the selected Cartesian plane.
      double C2 = calculator.compute(s1x, s1y, s1z, p0x, p0y, p0z, s3x, s3y, s3z); // = area of the triangle (s1, p0, s3) projected onto the selected Cartesian plane.
      double C3 = calculator.compute(s1x, s1y, s1z, s2x, s2y, s2z, p0x, p0y, p0z); // = area of the triangle (s1, s2, p0) projected onto the selected Cartesian plane.

      if (compareSigns(muMax, C1) && compareSigns(muMax, C2) && compareSigns(muMax, C3))
      { // The projection p0 is inside the face. Computing the barycentric coordinates.
         double[] lambdas = {C1 / muMax, C2 / muMax, C3 / muMax};
         GJKVertex3D[] supportingVerices = {s1, s2, s3};
         return new GJKSimplex3D(supportingVerices, lambdas);
      }
      else
      { // The projection p0 is outside the face, identifying the closest edge knowing that s1 was just added, so it cannot be rejected.
         double d = Double.POSITIVE_INFINITY;
         GJKSimplex3D output = null;

         if (compareSigns(muMax, -C2))
         {
            GJKSimplex3D candidateOutput = nextBestSimplexFrom1Simplex(s1, s3);
            double candidateNorm = candidateOutput.closestPointNormSquared();
            output = candidateOutput;
            d = candidateNorm;
         }

         if (compareSigns(muMax, -C3))
         {
            GJKSimplex3D candidateOutput = nextBestSimplexFrom1Simplex(s1, s2);
            double candidateNorm = candidateOutput.closestPointNormSquared();
            if (candidateNorm < d)
            {
               output = candidateOutput;
               d = candidateNorm;
            }
         }

         return output;
      }
   }

   static final ProjectedTriangleSignedAreaCalculator yzTriangleAreaCalculator = (ax, ay, az, bx, by, bz, cx, cy, cz) -> triangleSignedArea(ay, az, by, bz, cy,
                                                                                                                                            cz);
   static final ProjectedTriangleSignedAreaCalculator zxTriangleAreaCalculator = (ax, ay, az, bx, by, bz, cx, cy, cz) -> triangleSignedArea(az, ax, bz, bx, cz,
                                                                                                                                            cx);
   static final ProjectedTriangleSignedAreaCalculator xyTriangleAreaCalculator = (ax, ay, az, bx, by, bz, cx, cy, cz) -> triangleSignedArea(ax, ay, bx, by, cx,
                                                                                                                                            cy);

   static interface ProjectedTriangleSignedAreaCalculator
   {
      double compute(double ax, double ay, double az, double bx, double by, double bz, double cx, double cy, double cz);
   }

   private static double triangleSignedArea(double ax, double ay, double bx, double by, double cx, double cy)
   {
      return ax * (by - cy) + bx * (cy - ay) + cx * (ay - by);
   }

   private static GJKSimplex3D nextBestSimplexFrom1Simplex(GJKVertex3D s1, GJKVertex3D s2)
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
            double[] lambdas = {C1 / muMax, C2 / muMax};
            GJKVertex3D[] supportVertices = {s1, s2};
            return new GJKSimplex3D(supportVertices, lambdas);
         }
         else
         { // The projection is outside, since s1 is the new vertex we automatically reject s2.
            return new GJKSimplex3D(s1);
         }
      }
      else
      { // The projection is outside, since s1 is the new vertex we automatically reject s2.
         return new GJKSimplex3D(s1);
      }
   }

   static boolean compareSigns(double a, double b)
   {
      if (a > 0.0 && b >= 0.0)
         return true;
      else if (a < -0.0 && b <= -0.0)
         return true;
      else
         return false;
   }

   public static class GJKSimplex3D
   {
      private final GJKVertex3D[] vertices;
      private final double[] barycentricCoordinates;

      private final Point3D closestPointToOrigin;
      private final double distanceFromOriginSquared;

      public GJKSimplex3D()
      {
         vertices = new GJKVertex3D[0];
         barycentricCoordinates = new double[0];
         closestPointToOrigin = null;
         distanceFromOriginSquared = Double.NaN;
      }

      public GJKSimplex3D(GJKVertex3D supportVertex)
      {
         vertices = new GJKVertex3D[1];
         barycentricCoordinates = new double[1];
         vertices[0] = supportVertex;
         barycentricCoordinates[0] = 1.0;
         closestPointToOrigin = new Point3D(supportVertex);
         distanceFromOriginSquared = supportVertex.distanceFromOriginSquared();
      }

      public GJKSimplex3D(GJKVertex3D[] supportVertices, double[] barycentricCoordinates)
      {
         this.vertices = supportVertices;
         this.barycentricCoordinates = barycentricCoordinates;

         closestPointToOrigin = new Point3D();
         for (int i = 0; i < size(); i++)
            closestPointToOrigin.scaleAdd(barycentricCoordinates[i], supportVertices[i], closestPointToOrigin);
         distanceFromOriginSquared = closestPointToOrigin.distanceFromOriginSquared();
      }

      public boolean contains(GJKVertex3D query)
      {
         for (GJKVertex3D vertex : vertices)
         {
            if (query.equals(vertex))
               return true;
         }
         return false;
      }

      public Point3D closestPoint()
      {
         return closestPointToOrigin;
      }

      private double distanceFromOrigin = Double.NaN;

      public double closestPointNorm()
      {
         if (Double.isNaN(distanceFromOrigin))
            distanceFromOrigin = Math.sqrt(closestPointNormSquared());
         return distanceFromOrigin;
      }

      public double closestPointNormSquared()
      {
         return distanceFromOriginSquared;
      }

      public Vector3D getTriangleNormal()
      {
         if (vertices.length != 3)
            return null;
         Vector3D n = EuclidPolytopeTools.crossProductOfLineSegment3Ds(vertices[0], vertices[1], vertices[0], vertices[2]);
         if (TupleTools.dot(n, closestPointToOrigin) > 0.0)
            n.negate();
         return n;
      }

      public Point3D computePointOnA()
      {
         Point3D pointOnA = new Point3D();
         if (computePointOnA(pointOnA))
            return pointOnA;
         else
            return null;
      }

      public boolean computePointOnA(Point3DBasics pointOnAToPack)
      {
         if (size() == 0)
         {
            return false;
         }
         else if (size() == 1)
         {
            pointOnAToPack.set(vertices[0].getVertexOnShapeA());
            return true;
         }
         else
         {
            pointOnAToPack.setAndScale(barycentricCoordinates[0], vertices[0].getVertexOnShapeA());
            for (int i = 1; i < size(); i++)
               pointOnAToPack.scaleAdd(barycentricCoordinates[i], vertices[i].getVertexOnShapeA(), pointOnAToPack);
            return true;
         }
      }

      public Point3D computePointOnB()
      {
         Point3D pointOnB = new Point3D();
         if (computePointOnB(pointOnB))
            return pointOnB;
         else
            return null;
      }

      public boolean computePointOnB(Point3DBasics pointOnBToPack)
      {
         if (size() == 0)
         {
            return false;
         }
         else if (size() == 1)
         {
            pointOnBToPack.set(vertices[0].getVertexOnShapeB());
            return true;
         }
         else
         {
            pointOnBToPack.setAndScale(barycentricCoordinates[0], vertices[0].getVertexOnShapeB());
            for (int i = 1; i < size(); i++)
               pointOnBToPack.scaleAdd(barycentricCoordinates[i], vertices[i].getVertexOnShapeB(), pointOnBToPack);
            return true;
         }
      }

      private double maxNormSquared = Double.NEGATIVE_INFINITY;

      public double maxNormSquared()
      {
         if (maxNormSquared == Double.NEGATIVE_INFINITY)
         {
            for (int i = 0; i < size(); i++)
            {
               maxNormSquared = Math.max(maxNormSquared, vertices[i].distanceFromOriginSquared());
            }
         }

         return maxNormSquared;
      }

      public int size()
      {
         return vertices.length;
      }
   }

   public static class GJKVertex3D implements Point3DReadOnly
   {
      private final double x, y, z;
      private final Point3DReadOnly vertexOnShapeA;
      private final Point3DReadOnly vertexOnShapeB;

      public GJKVertex3D(Point3DReadOnly vertexOnShapeA, Point3DReadOnly vertexOnShapeB)
      {
         this.vertexOnShapeA = vertexOnShapeA;
         this.vertexOnShapeB = vertexOnShapeB;
         x = vertexOnShapeA.getX() - vertexOnShapeB.getX();
         y = vertexOnShapeA.getY() - vertexOnShapeB.getY();
         z = vertexOnShapeA.getZ() - vertexOnShapeB.getZ();
      }

      public Point3DReadOnly getVertexOnShapeA()
      {
         return vertexOnShapeA;
      }

      public Point3DReadOnly getVertexOnShapeB()
      {
         return vertexOnShapeB;
      }

      public double dot(Tuple3DReadOnly tuple3D)
      {
         return TupleTools.dot(this, tuple3D);
      }

      @Override
      public double getX()
      {
         return x;
      }

      @Override
      public double getY()
      {
         return y;
      }

      @Override
      public double getZ()
      {
         return z;
      }

      @Override
      public int hashCode()
      {
         long bits = 1L;
         bits = EuclidHashCodeTools.addToHashCode(bits, x);
         bits = EuclidHashCodeTools.addToHashCode(bits, y);
         bits = EuclidHashCodeTools.addToHashCode(bits, z);
         return EuclidHashCodeTools.toIntHashCode(bits);
      }

      @Override
      public boolean equals(Object object)
      {
         if (object == this)
         {
            return true;
         }
         else if (object instanceof Tuple3DReadOnly)
         {
            return equals((Tuple3DReadOnly) object);
         }
         else
         {
            return false;
         }
      }

      @Override
      public String toString()
      {
         return "GJK Vertex 3D: " + EuclidCoreIOTools.getTuple3DString(this);
      }
   }
}
