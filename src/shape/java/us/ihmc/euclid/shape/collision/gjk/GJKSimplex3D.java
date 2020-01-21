package us.ihmc.euclid.shape.collision.gjk;

import us.ihmc.euclid.shape.convexPolytope.tools.EuclidPolytopeTools;
import us.ihmc.euclid.tools.EuclidCoreTools;
import us.ihmc.euclid.tools.TupleTools;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DBasics;

/**
 * Simplex 3D used in the Gilbert-Johnson-Keerthi algorithm.
 * <p>
 * A simplex is either:
 * <ul>
 * <li>a point, a.k.a 0-simplex.
 * <li>a line segment, a.k.a 1-simplex.
 * <li>a triangle, a.k.a 2-simplex.
 * <li>a tetrahedron, a.k.a 3-simplex.
 * </ul>
 * </p>
 *
 * @author Sylvain Bertrand
 * @see GilbertJohnsonKeerthiCollisionDetector
 */
public class GJKSimplex3D
{
   /** The vertices composing this simplex. */
   private final GJKVertex3D[] vertices;
   /**
    * The barycentric coordinates of {@code closestPointToOrigin}. See:
    * <a href="https://en.wikipedia.org/wiki/Barycentric_coordinate_system">link</a>.
    */
   private final double[] barycentricCoordinates;
   /** Location of the point on this simplex that is the closest to the origin. */
   private final Point3D closestPointToOrigin;
   /** The square of the distance between this simplex and the origin. */
   private final double distanceFromOriginSquared;

   /** The distance between this simplex and the origin, evaluated upon request only. */
   private double distanceFromOrigin = Double.NaN;
   /**
    * The square of the norm of the vertex of this simplex that is the farthest from origin.
    */
   private double maxDistanceSquaredFromOrigin = Double.NEGATIVE_INFINITY;

   /**
    * Creates a new empty simplex, i.e. no vertices.
    */
   public GJKSimplex3D()
   {
      vertices = new GJKVertex3D[0];
      barycentricCoordinates = new double[0];
      closestPointToOrigin = null;
      distanceFromOriginSquared = Double.NaN;
   }

   /**
    * Creates a new 0-simplex.
    *
    * @param vertex the vertex defining this simplex. Not modified, reference saved.
    */
   public GJKSimplex3D(GJKVertex3D vertex)
   {
      vertices = new GJKVertex3D[1];
      barycentricCoordinates = new double[1];
      vertices[0] = vertex;
      barycentricCoordinates[0] = 1.0;
      closestPointToOrigin = new Point3D(vertex);
      distanceFromOriginSquared = vertex.distanceFromOriginSquared();
   }

   /**
    * Creates a new simplex given its vertices and the barycentric coordinates of the closest point to
    * the origin.
    *
    * @param vertices               the simplex vertices. Not modified, reference saved.
    * @param barycentricCoordinates the barycentric coordinates of the closest point to the origin on
    *                               this simplex. Not modified, reference saved.
    */
   public GJKSimplex3D(GJKVertex3D[] vertices, double[] barycentricCoordinates)
   {
      this.vertices = vertices;
      this.barycentricCoordinates = barycentricCoordinates;

      closestPointToOrigin = new Point3D();
      for (int i = 0; i < getNumberOfVertices(); i++)
         closestPointToOrigin.scaleAdd(barycentricCoordinates[i], vertices[i], closestPointToOrigin);
      distanceFromOriginSquared = closestPointToOrigin.distanceFromOriginSquared();
   }

   /**
    * Tests whether the query equals one of this simplex vertices.
    *
    * @param query the vertex to evaluate. Not modified.
    * @return {@code true} if this simplex has a vertex that is equal to the query, {@code false}
    *         otherwise.
    */
   public boolean contains(GJKVertex3D query)
   {
      for (GJKVertex3D vertex : vertices)
      {
         if (query.equals(vertex))
            return true;
      }
      return false;
   }

   /**
    * Gets the coordinates of the point on this simplex that is the closest to the origin.
    *
    * @return the reference to the closest point to the origin.
    */
   public Point3D getClosestPointToOrigin()
   {
      return closestPointToOrigin;
   }

   /**
    * Computes if needed and returns the distance between this simplex and the origin.
    *
    * @return the distance to the origin.
    */
   public double getDistanceToOrigin()
   {
      if (Double.isNaN(distanceFromOrigin))
         distanceFromOrigin = EuclidCoreTools.squareRoot(getDistanceSquaredToOrigin());
      return distanceFromOrigin;
   }

   /**
    * Gets the square of the distance between this simplex and the origin.
    *
    * @return the distance squared to the origin.
    */
   public double getDistanceSquaredToOrigin()
   {
      return distanceFromOriginSquared;
   }

   /**
    * Gets the square of the norm of the vertex of this simplex that is the farthest from origin.
    *
    * @return the max distance squared to the origin.
    */
   public double getMaxDistanceSquaredToOrigin()
   {
      if (maxDistanceSquaredFromOrigin == Double.NEGATIVE_INFINITY)
      {
         for (int i = 0; i < getNumberOfVertices(); i++)
         {
            maxDistanceSquaredFromOrigin = Math.max(maxDistanceSquaredFromOrigin, vertices[i].distanceFromOriginSquared());
         }
      }

      return maxDistanceSquaredFromOrigin;
   }

   /**
    * When this simplex is a triangle, its computes and returns its normal vector, returns {@code null}
    * if this simplex is not triangle.
    *
    * @return the triangle normal if this is a 2-simplex, {@code null} otherwise.
    */
   public Vector3D getTriangleNormal()
   {
      if (vertices.length != 3)
         return null;
      Vector3D n = EuclidPolytopeTools.crossProductOfLineSegment3Ds(vertices[0], vertices[1], vertices[0], vertices[2]);
      if (TupleTools.dot(n, closestPointToOrigin) > 0.0)
         n.negate();
      return n;
   }

   /**
    * Computes the point on the shape A that corresponds to this simplex closest point to the origin.
    * <p>
    * WARNING: This method generates garbage.
    * </p>
    *
    * @return the point on the shape A.
    */
   public Point3D computePointOnA()
   {
      Point3D pointOnA = new Point3D();
      if (computePointOnA(pointOnA))
         return pointOnA;
      else
         return null;
   }

   /**
    * Computes the point on the shape A that corresponds to this simplex closest point to the origin.
    *
    * @param pointOnAToPack the point used to store the result. Modified.
    * @return whether this method succeeded or not.
    */
   public boolean computePointOnA(Point3DBasics pointOnAToPack)
   {
      if (getNumberOfVertices() == 0)
      {
         return false;
      }
      else if (getNumberOfVertices() == 1)
      {
         pointOnAToPack.set(vertices[0].getVertexOnShapeA());
         return true;
      }
      else
      {
         pointOnAToPack.setAndScale(barycentricCoordinates[0], vertices[0].getVertexOnShapeA());
         for (int i = 1; i < getNumberOfVertices(); i++)
            pointOnAToPack.scaleAdd(barycentricCoordinates[i], vertices[i].getVertexOnShapeA(), pointOnAToPack);
         return true;
      }
   }

   /**
    * Computes the point on the shape B that corresponds to this simplex closest point to the origin.
    * <p>
    * WARNING: This method generates garbage.
    * </p>
    *
    * @return the point on the shape B.
    */
   public Point3D computePointOnB()
   {
      Point3D pointOnB = new Point3D();
      if (computePointOnB(pointOnB))
         return pointOnB;
      else
         return null;
   }

   /**
    * Computes the point on the shape B that corresponds to this simplex closest point to the origin.
    *
    * @param pointOnBToPack the point used to store the result. Modified.
    * @return whether this method succeeded or not.
    */
   public boolean computePointOnB(Point3DBasics pointOnBToPack)
   {
      if (getNumberOfVertices() == 0)
      {
         return false;
      }
      else if (getNumberOfVertices() == 1)
      {
         pointOnBToPack.set(vertices[0].getVertexOnShapeB());
         return true;
      }
      else
      {
         pointOnBToPack.setAndScale(barycentricCoordinates[0], vertices[0].getVertexOnShapeB());
         for (int i = 1; i < getNumberOfVertices(); i++)
            pointOnBToPack.scaleAdd(barycentricCoordinates[i], vertices[i].getVertexOnShapeB(), pointOnBToPack);
         return true;
      }
   }

   /**
    * Gets the number of vertices composing this simplex.
    *
    * @return the number of vertices.
    */
   public int getNumberOfVertices()
   {
      return vertices.length;
   }

   /**
    * Gets this simplex vertices.
    *
    * @return the vertices composing this simplex.
    */
   public GJKVertex3D[] getVertices()
   {
      return vertices;
   }
}