package us.ihmc.euclid.shape.collision.gjk;

import us.ihmc.euclid.tools.EuclidCoreTools;
import us.ihmc.euclid.tools.Matrix3DFeatures;
import us.ihmc.euclid.tools.TupleTools;

/**
 * This class provides the tools needed for the Gilbert-Johnson-Keerthi algorithm used for collision
 * detection.
 *
 * @author Sylvain Bertrand
 * @see GilbertJohnsonKeerthiCollisionDetector
 */
public class GJKTools
{
   /**
    * Functional interface for computing the signed area of a 3D triangle represented by the vertices
    * {@code a}, {@code b}, and {@code c}.
    *
    * @author Sylvain Bertrand
    */
   public static interface ProjectedTriangleSignedAreaCalculator
   {
      /**
       * Calculates the signed area of the 3D triangle.
       * <p>
       * The sign of the area depends on the winding of the vertices: positive for a counter-clockwise
       * ordering, negative otherwise.
       * </p>
       *
       * @param ax the x-coordinate of the first triangle vertex.
       * @param ay the y-coordinate of the first triangle vertex.
       * @param az the z-coordinate of the first triangle vertex.
       * @param bx the x-coordinate of the second triangle vertex.
       * @param by the y-coordinate of the second triangle vertex.
       * @param bz the z-coordinate of the second triangle vertex.
       * @param cx the x-coordinate of the third triangle vertex.
       * @param cy the y-coordinate of the third triangle vertex.
       * @param cz the z-coordinate of the third triangle vertex.
       * @return the signed area of the triangle.
       */
      double compute(double ax, double ay, double az, double bx, double by, double bz, double cx, double cy, double cz);
   }

   /**
    * Calculator for computing the area of a 3D triangle projected onto the YZ-plane.
    * <p>
    * The value returned by this calculated is actually two times the triangle area.
    * </p>
    */
   public static final ProjectedTriangleSignedAreaCalculator yzTriangleAreaCalculator = (ax, ay, az, bx, by, bz, cx, cy,
                                                                                         cz) -> triangleSignedArea(ay, az, by, bz, cy, cz);
   /**
    * Calculator for computing the area of a 3D triangle projected onto the ZX-plane.
    * <p>
    * The value returned by this calculated is actually two times the triangle area.
    * </p>
    */
   public static final ProjectedTriangleSignedAreaCalculator zxTriangleAreaCalculator = (ax, ay, az, bx, by, bz, cx, cy,
                                                                                         cz) -> triangleSignedArea(az, ax, bz, bx, cz, cx);
   /**
    * Calculator for computing the area of a 3D triangle projected onto the XY-plane.
    * <p>
    * The value returned by this calculated is actually two times the triangle area.
    * </p>
    */
   public static final ProjectedTriangleSignedAreaCalculator xyTriangleAreaCalculator = (ax, ay, az, bx, by, bz, cx, cy,
                                                                                         cz) -> triangleSignedArea(ax, ay, bx, by, cx, cy);

   /**
    * Calculates the signed area of a 2D triangle.
    * <p>
    * The sign of the area depends on the winding of the vertices: positive for a counter-clockwise
    * ordering, negative otherwise.
    * </p>
    *
    * @param ax the x-coordinate of the first triangle vertex.
    * @param ay the y-coordinate of the first triangle vertex.
    * @param bx the x-coordinate of the second triangle vertex.
    * @param by the y-coordinate of the second triangle vertex.
    * @param cx the x-coordinate of the third triangle vertex.
    * @param cy the y-coordinate of the third triangle vertex.
    * @return {@code 2.0 * area}.
    */
   public static double triangleSignedArea(double ax, double ay, double bx, double by, double cx, double cy)
   {
      return ax * (by - cy) + bx * (cy - ay) + cx * (ay - by);
   }

   /**
    * Finds and returns the smallest simplex that belongs to the simplex defined by the given
    * {@code oldVertices} and {@code newVertex} and that is the closest to the origin.
    * <p>
    * As well as finding the smallest closest simplex, the projection of the origin onto it, its
    * distance to the origin, and its barycentric coordinates are also computed.
    * </p>
    * <p>
    * This method is an implementation of the <i>Signed Volumes distance sub-algorithm</i> introduced
    * in: <a href="https://dl.acm.org/citation.cfm?id=3083724">Improving the GJK algorithm for faster
    * and more reliable distance queries between convex objects</a>
    * </p>
    *
    * @param oldVertices the vertices that may be filtered out. The array should contain at most 3
    *                    vertices. Not modified.
    * @param newVertex   the vertex that should not be filtered out by this method. Not modified.
    * @return the smallest simplex that is the closest to the origin.
    */
   public static GJKSimplex3D simplexClosestToOrigin(GJKVertex3D[] oldVertices, GJKVertex3D newVertex)
   {
      if (oldVertices.length == 3)
         return simplexClosestToOriginFrom3Simplex(newVertex, oldVertices[2], oldVertices[1], oldVertices[0]);
      else if (oldVertices.length == 2)
         return simplexClosestToOriginFrom2Simplex(newVertex, oldVertices[1], oldVertices[0]);
      else if (oldVertices.length == 1)
         return simplexClosestToOriginFrom1Simplex(newVertex, oldVertices[0]);
      else
         return new GJKSimplex3D(newVertex);
   }

   /**
    * Finds and returns the smallest simplex that belongs to the tetrahedron, defined by the given
    * vertices, that is the closest to the origin.
    * <p>
    * As well as finding the smallest closest simplex, the projection of the origin onto it, its
    * distance to the origin, and its barycentric coordinates are also computed.
    * </p>
    * <p>
    * This method is an implementation of the <i>Sub-routine for 3-simplex</i> introduced in:
    * <a href="https://dl.acm.org/citation.cfm?id=3083724">Improving the GJK algorithm for faster and
    * more reliable distance queries between convex objects</a>
    * </p>
    *
    * @param s1 the first vertex of the tetrahedron. <b>This method assumes that this vertex should not
    *           be filtered out</b>. Not modified.
    * @param s2 the second vertex of the tetrahedron. Not modified.
    * @param s3 the third vertex of the tetrahedron. Not modified.
    * @param s4 the fourth vertex of the tetrahedron. Not modified.
    * @return the smallest simplex that is the closest to the origin.
    */
   public static GJKSimplex3D simplexClosestToOriginFrom3Simplex(GJKVertex3D s1, GJKVertex3D s2, GJKVertex3D s3, GJKVertex3D s4)
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

         double zeroTestEpsilon = 1.0e-13;

         if (compareSigns(detM, -C42))
         {
            if (EuclidCoreTools.isZero(detM, zeroTestEpsilon) && EuclidCoreTools.isZero(C42, zeroTestEpsilon))
               return null;

            GJKSimplex3D candidateOutput = simplexClosestToOriginFrom2Simplex(s1, s3, s4);
            if (candidateOutput != null)
            {
               double candidateNorm = candidateOutput.getDistanceSquaredToOrigin();
               if (candidateNorm < d)
               {
                  output = candidateOutput;
                  d = candidateNorm;
               }
            }
         }

         if (compareSigns(detM, -C43))
         {
            if (EuclidCoreTools.isZero(detM, zeroTestEpsilon) && EuclidCoreTools.isZero(C43, zeroTestEpsilon))
               return null;

            GJKSimplex3D candidateOutput = simplexClosestToOriginFrom2Simplex(s1, s2, s4);
            if (candidateOutput != null)
            {
               double candidateNorm = candidateOutput.getDistanceSquaredToOrigin();
               if (candidateNorm < d)
               {
                  output = candidateOutput;
                  d = candidateNorm;
               }
            }
         }

         if (compareSigns(detM, -C44))
         {
            if (EuclidCoreTools.isZero(detM, zeroTestEpsilon) && EuclidCoreTools.isZero(C44, zeroTestEpsilon))
               return null;

            GJKSimplex3D candidateOutput = simplexClosestToOriginFrom2Simplex(s1, s2, s3);
            if (candidateOutput != null)
            {
               double candidateNorm = candidateOutput.getDistanceSquaredToOrigin();
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

   /**
    * Finds and returns the smallest simplex that belongs to the 3D triangle, defined by the given
    * vertices, that is the closest to the origin.
    * <p>
    * As well as finding the smallest closest simplex, the projection of the origin onto it, its
    * distance to the origin, and its barycentric coordinates are also computed.
    * </p>
    * <p>
    * This method is an implementation of the <i>Sub-routine for 2-simplex</i> introduced in:
    * <a href="https://dl.acm.org/citation.cfm?id=3083724">Improving the GJK algorithm for faster and
    * more reliable distance queries between convex objects</a>
    * </p>
    *
    * @param s1 the first vertex of the triangle. <b>This method assumes that this vertex should not be
    *           filtered out</b>. Not modified.
    * @param s2 the second vertex of the triangle. Not modified.
    * @param s3 the third vertex of the triangle. Not modified.
    * @return the smallest simplex that is the closest to the origin.
    */
   public static GJKSimplex3D simplexClosestToOriginFrom2Simplex(GJKVertex3D s1, GJKVertex3D s2, GJKVertex3D s3)
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
         if (Math.abs(C1) < 1.0e-16 && Math.abs(C2) < 1.0e-16 && Math.abs(C3) < 1.0e-16)
            return null;

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
            GJKSimplex3D candidateOutput = simplexClosestToOriginFrom1Simplex(s1, s3);
            double candidateNorm = candidateOutput.getDistanceSquaredToOrigin();
            output = candidateOutput;
            d = candidateNorm;
         }

         if (compareSigns(muMax, -C3))
         {
            GJKSimplex3D candidateOutput = simplexClosestToOriginFrom1Simplex(s1, s2);
            double candidateNorm = candidateOutput.getDistanceSquaredToOrigin();
            if (candidateNorm < d)
            {
               output = candidateOutput;
               d = candidateNorm;
            }
         }

         return output;
      }
   }

   /**
    * Finds and returns the smallest simplex that belongs to the 3D line segment, defined by the given
    * vertices, that is the closest to the origin.
    * <p>
    * As well as finding the smallest closest simplex, the projection of the origin onto it, its
    * distance to the origin, and its barycentric coordinates are also computed.
    * </p>
    * <p>
    * This method is an implementation of the <i>Sub-routine for 1-simplex</i> introduced in:
    * <a href="https://dl.acm.org/citation.cfm?id=3083724">Improving the GJK algorithm for faster and
    * more reliable distance queries between convex objects</a>
    * </p>
    *
    * @param s1 the first vertex of the line segment. <b>This method assumes that this vertex should
    *           not be filtered out</b>. Not modified.
    * @param s2 the second vertex of the line segment. Not modified.
    * @return the smallest simplex that is the closest to the origin.
    */
   public static GJKSimplex3D simplexClosestToOriginFrom1Simplex(GJKVertex3D s1, GJKVertex3D s2)
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

   /**
    * Compares the signs of {@code a} and {@code b} and returns {@code true} if they share the same
    * sign, {@code false} otherwise.
    * <p>
    * This sign comparison is adapted to handle {@link Double#NaN} and edge-cases of the GJK algorithm
    * and should not be used outside this context.
    * </p>
    *
    * @param a the first value.
    * @param b the second value.
    * @return {@code true} if the two values share the same sign, {@code false} otherwise.
    */
   public static boolean compareSigns(double a, double b)
   {
      if (a > 0.0 && b >= 0.0)
         return true;
      else if (a < -0.0 && b <= -0.0)
         return true;
      else
         return false;
   }
}
