package us.ihmc.euclid.shape.collision;

import us.ihmc.euclid.Axis;
import us.ihmc.euclid.shape.primitives.interfaces.Shape3DReadOnly;
import us.ihmc.euclid.tools.EuclidCoreTools;
import us.ihmc.euclid.tools.Matrix3DFeatures;
import us.ihmc.euclid.tools.TupleTools;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;

// From "Improving the GJK algorithm for faster and more reliable distance queries between convex
// objects."
public class GilbertJohnsonKeerthiCollisionDetector
{
   private double epsilon = 1.0e-16;
   private int maxIterations = 500;
   private boolean latestCollisionTestResult;
   private ClosestSimplexFeature simplex = null;
   private int numberOfIterations = 0;
   private final Vector3D supportDirection = new Vector3D();

   public GilbertJohnsonKeerthiCollisionDetector()
   {
   }

   public Shape3DCollisionTestResult doShapeCollisionTest(Shape3DReadOnly shapeA, Shape3DReadOnly shapeB)
   {
      boolean areShapesColliding = doCollisionTest(shapeA, shapeB);
      Shape3DCollisionTestResult result = new Shape3DCollisionTestResult();
      if (simplex == null)
         return null;
      result.setToNaN();
      packResult(shapeA, shapeB, result, areShapesColliding);
      return result;
   }

   public void doShapeCollisionTest(Shape3DReadOnly shapeA, Shape3DReadOnly shapeB, Shape3DCollisionTestResult result)
   {
      boolean areShapesColliding = doCollisionTest(shapeA, shapeB);
      result.setToNaN();

      if (simplex == null)
         return;

      packResult(shapeA, shapeB, result, areShapesColliding);
   }

   public boolean doCollisionTest(SupportingVertexHolder shapeA, SupportingVertexHolder shapeB)
   {
      ClosestSimplexFeature previousOutput = new ClosestSimplexFeature();

      supportDirection.set(Axis.Y);
      Point3DReadOnly vertexA = shapeA.getSupportingVertex(supportDirection);
      supportDirection.negate();
      Point3DReadOnly vertexB = shapeB.getSupportingVertex(supportDirection);

      if (vertexA == null || vertexB == null)
      {
         simplex = null;
         latestCollisionTestResult = false;
         return false;
      }

      double closestPointNormSquared = 1.0;

      for (int i = 0; i < maxIterations; i++)
      {
         numberOfIterations = i;
         DifferenceVertex3D newVertex = new DifferenceVertex3D(vertexA, vertexB);

         if (simplexContainsVertex(previousOutput.supportVertices, newVertex, epsilon))
         {
            simplex = previousOutput;
            latestCollisionTestResult = false;
            break;
         }

         if (Math.abs(closestPointNormSquared - newVertex.dot(supportDirection)) <= epsilon * closestPointNormSquared)
         {
            simplex = previousOutput;
            latestCollisionTestResult = false;
            break;
         }

         ClosestSimplexFeature output = computeClosestFeatureBasedOnSignedVolume(previousOutput.supportVertices, newVertex);

         if (output == null)
         { // End of process
            simplex = previousOutput;
            latestCollisionTestResult = false;
            break;
         }

         if (output.closestPointNormSquared() >= previousOutput.closestPointNormSquared())
         {
            simplex = previousOutput;
            latestCollisionTestResult = false;
            break;
         }

         closestPointNormSquared = output.closestPointNormSquared();

         if (output.size() == 4 || closestPointNormSquared <= epsilon * output.maxNormSquared())
         { // End of process
            simplex = output;
            latestCollisionTestResult = true;
            break;
         }

         supportDirection.setAndNegate(output.closestPoint());
         vertexA = shapeA.getSupportingVertex(supportDirection);
         supportDirection.negate();
         vertexB = shapeB.getSupportingVertex(supportDirection);

         previousOutput = output;
      }

      return latestCollisionTestResult;
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

   void packResult(Shape3DReadOnly shapeA, Shape3DReadOnly shapeB, Shape3DCollisionTestResult result, boolean areShapesColliding)
   {
      result.setShapesAreColliding(areShapesColliding);
      result.setShapeA(shapeA);
      result.setShapeB(shapeB);

      if (areShapesColliding)
         return;

      result.setDistance(simplex.closestPointNorm());
      result.getPointOnA().set(getClosestPointOnA());
      result.getPointOnB().set(getClosestPointOnB());
   }

   public double getDistance()
   {
      if (simplex == null)
         return Double.NaN;
      else
         return simplex.closestPointNorm();
   }

   public Vector3DReadOnly getSeparationVector()
   {
      if (simplex == null || latestCollisionTestResult)
         return null;

      Vector3D separationVector = new Vector3D();
      separationVector.set(simplex.closestPoint());
      return separationVector;
   }

   public Point3DReadOnly getClosestPointOnA()
   {
      if (simplex == null || latestCollisionTestResult)
         return null;

      return simplex.computePointOnA();
   }

   public Point3DReadOnly getClosestPointOnB()
   {
      if (simplex == null || latestCollisionTestResult)
         return null;

      return simplex.computePointOnB();
   }

   public DifferenceVertex3D[] getSimplexVertices()
   {
      if (simplex == null)
         return null;
      else
         return simplex.supportVertices;
   }

   private static boolean simplexContainsVertex(DifferenceVertex3D[] simplex, DifferenceVertex3D query, double epsilon)
   {
      for (DifferenceVertex3D vertex : simplex)
      {
         if (query.epsilonEquals(vertex, epsilon))
            return true;
      }
      return false;
   }

   public static ClosestSimplexFeature computeClosestFeatureBasedOnSignedVolume(DifferenceVertex3D[] oldVertices, DifferenceVertex3D newVertex)
   {
      if (oldVertices.length == 3)
         return signedVolumeFor3Simplex(newVertex, oldVertices[2], oldVertices[1], oldVertices[0]);
      else if (oldVertices.length == 2)
         return signedVolumeFor2Simplex(newVertex, oldVertices[1], oldVertices[0]);
      else if (oldVertices.length == 1)
         return signedVolumeFor1Simplex(newVertex, oldVertices[0]);
      else
         return new ClosestSimplexFeature(newVertex);
   }

   public static ClosestSimplexFeature signedVolumeFor3Simplex(DifferenceVertex3D s1, DifferenceVertex3D s2, DifferenceVertex3D s3, DifferenceVertex3D s4)
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
         DifferenceVertex3D[] supportVertices = {s1, s2, s3, s4};
         return new ClosestSimplexFeature(supportVertices, lambdas);
      }
      else
      {
         double d = Double.POSITIVE_INFINITY;
         ClosestSimplexFeature output = null;

         if (compareSigns(detM, -C42))
         {
            ClosestSimplexFeature candidateOutput = signedVolumeFor2Simplex(s1, s3, s4);
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
            ClosestSimplexFeature candidateOutput = signedVolumeFor2Simplex(s1, s2, s4);
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
            ClosestSimplexFeature candidateOutput = signedVolumeFor2Simplex(s1, s2, s3);
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

   public static ClosestSimplexFeature signedVolumeFor2Simplex(DifferenceVertex3D s1, DifferenceVertex3D s2, DifferenceVertex3D s3)
   {
      double s1x3D = s1.getX(), s1y3D = s1.getY(), s1z3D = s1.getZ();
      double s2x3D = s2.getX(), s2y3D = s2.getY(), s2z3D = s2.getZ();
      double s3x3D = s3.getX(), s3y3D = s3.getY(), s3z3D = s3.getZ();

      double s2s1x3D = s1x3D - s2x3D;
      double s2s1y3D = s1y3D - s2y3D;
      double s2s1z3D = s1z3D - s2z3D;

      double s3s1x3D = s1x3D - s3x3D;
      double s3s1y3D = s1y3D - s3y3D;
      double s3s1z3D = s1z3D - s3z3D;

      // Computing the face's normal as n = (s2 - s1) x (s3 - s1)
      double nx = s2s1y3D * s3s1z3D - s2s1z3D * s3s1y3D;
      double ny = s2s1z3D * s3s1x3D - s2s1x3D * s3s1z3D;
      double nz = s2s1x3D * s3s1y3D - s2s1y3D * s3s1x3D;

      // Computing the projection p0 of the origin (0, 0, 0) onto the face as p0 = (s1.n) / (n.n) n
      double distanceFromPlane = TupleTools.dot(nx, ny, nz, s1) / EuclidCoreTools.normSquared(nx, ny, nz);
      double p0x3D = distanceFromPlane * nx;
      double p0y3D = distanceFromPlane * ny;
      double p0z3D = distanceFromPlane * nz;

      double muMax = 0.0; // mu = area of the triangle (s1, s2, s3)
      double s1x, s2x, s3x, p0x;
      double s1y, s2y, s3y, p0y;

      // Finding muMax and the coordinates that generate it.
      // As described in the paper, the 3D triangle is projected on each Cartesian plane. The projected 2D triangle with the largest area is selected to pursue further computation.
      double mu;

      // Area of the triangle (s1, s2, s3) projected onto the YZ-plane.
      mu = s1y3D * (s2z3D - s3z3D) + s2y3D * (s3z3D - s1z3D) + s3y3D * (s1z3D - s2z3D);

      muMax = mu;
      s1x = s1y3D;
      s2x = s2y3D;
      s3x = s3y3D;
      p0x = p0y3D;
      s1y = s1z3D;
      s2y = s2z3D;
      s3y = s3z3D;
      p0y = p0z3D;

      // Area of the triangle (s1, s2, s3) projected onto the XZ-plane.
      mu = s1z3D * (s2x3D - s3x3D) + s2z3D * (s3x3D - s1x3D) + s3z3D * (s1x3D - s2x3D);

      if (Math.abs(mu) > Math.abs(muMax))
      {
         muMax = mu;
         s1x = s1z3D;
         s2x = s2z3D;
         s3x = s3z3D;
         p0x = p0z3D;
         s1y = s1x3D;
         s2y = s2x3D;
         s3y = s3x3D;
         p0y = p0x3D;
      }

      // Area of the triangle (s1, s2, s3) projected onto the XY-plane.
      mu = s1x3D * (s2y3D - s3y3D) + s2x3D * (s3y3D - s1y3D) + s3x3D * (s1y3D - s2y3D);

      if (Math.abs(mu) > Math.abs(muMax))
      {
         muMax = mu;
         s1x = s1x3D;
         s2x = s2x3D;
         s3x = s3x3D;
         p0x = p0x3D;
         s1y = s1y3D;
         s2y = s2y3D;
         s3y = s3y3D;
         p0y = p0y3D;
      }

      double C1 = p0x * (s2y - s3y) + s2x * (s3y - p0y) + s3x * (p0y - s2y); // = area of the triangle (p0, s2, s3) projected onto the selected Cartesian plane.
      double C2 = s1x * (p0y - s3y) + p0x * (s3y - s1y) + s3x * (s1y - p0y); // = area of the triangle (s1, p0, s3) projected onto the selected Cartesian plane.
      double C3 = s1x * (s2y - p0y) + s2x * (p0y - s1y) + p0x * (s1y - s2y); // = area of the triangle (s1, s2, p0) projected onto the selected Cartesian plane.

      if (compareSigns(muMax, C1) && compareSigns(muMax, C2) && compareSigns(muMax, C3))
      { // The projection p0 is inside the face. Computing the barycentric coordinates.
         double[] lambdas = {C1 / muMax, C2 / muMax, C3 / muMax};
         DifferenceVertex3D[] supportingVerices = {s1, s2, s3};
         return new ClosestSimplexFeature(supportingVerices, lambdas);
      }
      else
      { // The projection p0 is outside the face, identifying the closest edge knowing that s1 was just added, so it cannot be rejected.
         double d = Double.POSITIVE_INFINITY;
         ClosestSimplexFeature output = null;

         if (compareSigns(muMax, -C2))
         {
            ClosestSimplexFeature candidateOutput = signedVolumeFor1Simplex(s1, s3);
            double candidateNorm = candidateOutput.closestPointNormSquared();
            output = candidateOutput;
            d = candidateNorm;
         }

         if (compareSigns(muMax, -C3))
         {
            ClosestSimplexFeature candidateOutput = signedVolumeFor1Simplex(s1, s2);
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

   public static ClosestSimplexFeature signedVolumeFor1Simplex(DifferenceVertex3D s1, DifferenceVertex3D s2)
   {
      double s1x3D = s1.getX(), s1y3D = s1.getY(), s1z3D = s1.getZ();
      double s2x3D = s2.getX(), s2y3D = s2.getY(), s2z3D = s2.getZ();

      // Computing the edge's direction as t = (s2 - s1)
      double tx = s2x3D - s1x3D;
      double ty = s2y3D - s1y3D;
      double tz = s2z3D - s1z3D;
      // Computing the projection p0 of the origin (0, 0, 0) onto the edge as p0 = s2 - (s2.t) / (t.t) t
      double param = -TupleTools.dot(tx, ty, tz, s2) / EuclidCoreTools.normSquared(tx, ty, tz);
      double p0x3D = param * tx + s2x3D;
      double p0y3D = param * ty + s2y3D;
      double p0z3D = param * tz + s2z3D;

      double muMax = 0.0; // mu = length of the edge (s1, s2)
      double s1Max, s2Max, p0Max;

      { // Finding muMax and the coordinates that generate it.
        // Length of the edge (s1, s2) projected onto the X-axis.
         s1Max = s1x3D;
         s2Max = s2x3D;
         p0Max = p0x3D;
         muMax = s1x3D - s2x3D;

         // Length of the edge (s1, s2) projected onto the Y-axis.
         double mu = s1y3D - s2y3D;

         if (Math.abs(mu) > Math.abs(muMax))
         {
            s1Max = s1y3D;
            s2Max = s2y3D;
            p0Max = p0y3D;
            muMax = mu;
         }

         // Length of the edge (s1, s2) projected onto the Z-axis.
         mu = s1z3D - s2z3D;

         if (Math.abs(mu) > Math.abs(muMax))
         {
            s1Max = s1z3D;
            s2Max = s2z3D;
            p0Max = p0z3D;
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
            DifferenceVertex3D[] supportVertices = {s1, s2};
            return new ClosestSimplexFeature(supportVertices, lambdas);
         }
         else
         { // The projection is outside, since s1 is the new vertex we automatically reject s2.
            return new ClosestSimplexFeature(s1);
         }
      }
      else
      { // The projection is outside, since s1 is the new vertex we automatically reject s2.
         return new ClosestSimplexFeature(s1);
      }
   }

   public static boolean compareSigns(double a, double b)
   {
      if (a > 0.0 && b >= 0.0)
         return true;
      else if (a < -0.0 && b <= -0.0)
         return true;
      else
         return false;
   }

   public static class ClosestSimplexFeature
   {
      private final DifferenceVertex3D[] supportVertices;
      private final double[] barycentricCoordinates;

      private final Point3D closestPointToOrigin;
      private final double distanceFromOriginSquared;

      public ClosestSimplexFeature()
      {
         supportVertices = new DifferenceVertex3D[0];
         barycentricCoordinates = new double[0];
         closestPointToOrigin = null;
         distanceFromOriginSquared = Double.NaN;
      }

      public ClosestSimplexFeature(DifferenceVertex3D supportVertex)
      {
         supportVertices = new DifferenceVertex3D[1];
         barycentricCoordinates = new double[1];
         supportVertices[0] = supportVertex;
         barycentricCoordinates[0] = 1.0;
         closestPointToOrigin = new Point3D(supportVertex);
         distanceFromOriginSquared = supportVertex.distanceFromOriginSquared();
      }

      public ClosestSimplexFeature(DifferenceVertex3D[] supportVertices, double[] barycentricCoordinates)
      {
         this.supportVertices = supportVertices;
         this.barycentricCoordinates = barycentricCoordinates;

         closestPointToOrigin = new Point3D();
         for (int i = 0; i < size(); i++)
            closestPointToOrigin.scaleAdd(barycentricCoordinates[i], supportVertices[i], closestPointToOrigin);
         distanceFromOriginSquared = closestPointToOrigin.distanceFromOriginSquared();
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

      public Point3D computePointOnA()
      {
         if (size() == 0)
         {
            return null;
         }
         else if (size() == 1)
         {
            return new Point3D(supportVertices[0].getVertexOnShapeA());
         }
         else
         {
            Point3D pointOnA = new Point3D();
            for (int i = 0; i < size(); i++)
               pointOnA.scaleAdd(barycentricCoordinates[i], supportVertices[i].getVertexOnShapeA(), pointOnA);
            return pointOnA;
         }
      }

      public Point3D computePointOnB()
      {
         if (size() == 0)
         {
            return null;
         }
         else if (size() == 1)
         {
            return new Point3D(supportVertices[0].getVertexOnShapeB());
         }
         else
         {
            Point3D pointOnB = new Point3D();
            for (int i = 0; i < size(); i++)
               pointOnB.scaleAdd(barycentricCoordinates[i], supportVertices[i].getVertexOnShapeB(), pointOnB);
            return pointOnB;
         }
      }

      private double maxNormSquared = Double.NEGATIVE_INFINITY;

      public double maxNormSquared()
      {
         if (maxNormSquared == Double.NEGATIVE_INFINITY)
         {
            for (int i = 0; i < size(); i++)
            {
               maxNormSquared = Math.max(maxNormSquared, supportVertices[i].distanceFromOriginSquared());
            }
         }

         return maxNormSquared;
      }

      public int size()
      {
         return supportVertices.length;
      }
   }
}
