package us.ihmc.euclid.shape.collision.epa;

import static us.ihmc.euclid.shape.collision.epa.EPATools.barycentricCoordinatesFrom2Simplex;

import java.util.Arrays;
import java.util.List;

import us.ihmc.euclid.geometry.BoundingBox3D;
import us.ihmc.euclid.geometry.interfaces.BoundingBox3DReadOnly;
import us.ihmc.euclid.geometry.tools.EuclidGeometryTools;
import us.ihmc.euclid.shape.collision.epa.EPATools.BarycentricCoordinatesOutput;
import us.ihmc.euclid.shape.convexPolytope.interfaces.Face3DReadOnly;
import us.ihmc.euclid.shape.convexPolytope.interfaces.HalfEdge3DReadOnly;
import us.ihmc.euclid.shape.convexPolytope.tools.EuclidPolytopeTools;
import us.ihmc.euclid.shape.tools.EuclidShapeIOTools;
import us.ihmc.euclid.tools.EuclidCoreTools;
import us.ihmc.euclid.tools.EuclidHashCodeTools;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DBasics;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;

/**
 * Triangle face 3D that belongs to a polytope used in the Expanding Polytope algorithm.
 * 
 * @author Sylvain Bertrand
 * @see ExpandingPolytopeAlgorithm
 */
public class EPAFace3D implements Comparable<EPAFace3D>, Face3DReadOnly
{
   /** The vertices composing this face. */
   private final EPAVertex3D v0, v1, v2;
   /** The edges composing this face. */
   private final EPAHalfEdge3D e0, e1, e2;
   /** Location of the point on this face that is the closest to the origin. */
   private final Point3DReadOnly closestPointToOrigin;
   /**
    * The barycentric coordinates of {@code closestPointToOrigin}. See:
    * <a href="https://en.wikipedia.org/wiki/Barycentric_coordinate_system">link</a>.
    */
   private final double lambda0, lambda1, lambda2;
   /** Whether this triangle face is affinely dependent. */
   private final boolean isTriangleAffinelyDependent;
   /** Whether the projection of the origin onto this face is located inside. */
   private final boolean isClosestPointInternal;
   /** The square of the distance between this simplex and the origin. */
   private final double distanceFromOriginSquared;
   /** This face normal. It points towards the outside of the polytope. */
   private final Vector3D normal;

   /** Whether this face has been discarded and is no longer part of a polytope. */
   private boolean obsolete = false;
   /** The distance between this simplex and the origin, evaluated upon request only. */
   private double distanceFromOrigin = Double.NaN;

   /**
    * Creates a new face from one of its edge's twin and a vertex.
    * <p>
    * This face winding is determined to be consistent with the given {@code twin}, i.e. the new face
    * edge linked to it as twin is oriented in opposite direction.
    * </p>
    * 
    * @param vertex  one of the new face vertex. Not modified, reference saved.
    * @param twin    the twin of one of the new face's edges. Not modified, reference saved.
    * @param epsilon tolerance used notably for determining whether the new triangle face is affinely
    *                dependent or not.
    * @return the new face linked to the given {@code twin}.
    */
   public static EPAFace3D fromVertexAndTwinEdge(EPAVertex3D vertex, EPAHalfEdge3D twin, double epsilon)
   {
      EPAVertex3D v0 = twin.getDestination();
      EPAVertex3D v1 = twin.getOrigin();
      EPAVertex3D v2 = vertex;
      EPAFace3D face = new EPAFace3D(v0, v1, v2, epsilon);
      face.e0.setTwin(twin);
      return face;
   }

   /**
    * Creates a new face from 3 given vertices.
    * <p>
    * The winding of the new face is based on the ordering of the given vertices.
    * </p>
    * 
    * @param v0      the first vertex of the new face. Not modified.
    * @param v1      the second vertex of the new face. Not modified.
    * @param v2      the third vertex of the new face. Not modified.
    * @param epsilon tolerance used notably for determining whether the new triangle face is affinely
    *                dependent or not.
    */
   public EPAFace3D(EPAVertex3D v0, EPAVertex3D v1, EPAVertex3D v2, double epsilon)
   {
      this.v0 = v0;
      this.v1 = v1;
      this.v2 = v2;
      e0 = new EPAHalfEdge3D(v0, v1, this);
      e1 = new EPAHalfEdge3D(v1, v2, this);
      e2 = new EPAHalfEdge3D(v2, v0, this);

      e0.setNext(e1);
      e1.setNext(e2);
      e2.setNext(e0);

      e0.setPrevious(e2);
      e1.setPrevious(e0);
      e2.setPrevious(e1);

      normal = EuclidPolytopeTools.crossProductOfLineSegment3Ds(v1, v0, v1, v2);

      double[] lambdas = new double[3];
      BarycentricCoordinatesOutput output = barycentricCoordinatesFrom2Simplex(v0, v1, v2, epsilon, lambdas);
      isTriangleAffinelyDependent = output == BarycentricCoordinatesOutput.AFFINELY_DEPENDENT;

      if (!isTriangleAffinelyDependent())
      {
         lambda0 = lambdas[0];
         lambda1 = lambdas[1];
         lambda2 = lambdas[2];

         isClosestPointInternal = output == BarycentricCoordinatesOutput.INSIDE;

         Point3D point = new Point3D();
         point.setAndScale(lambda0, v0);
         point.scaleAdd(lambda1, v1, point);
         point.scaleAdd(lambda2, v2, point);
         closestPointToOrigin = point;
         distanceFromOriginSquared = closestPointToOrigin.distanceFromOriginSquared();
      }
      else
      {
         lambda0 = Double.NaN;
         lambda1 = Double.NaN;
         lambda2 = Double.NaN;
         isClosestPointInternal = false;
         closestPointToOrigin = null;
         distanceFromOriginSquared = Double.NaN;
      }
   }

   /**
    * Tests whether the query equals one of this face vertices.
    * 
    * @param query the vertex to evaluate. Not modified.
    * @return {@code true} if this face has a vertex that is equal to the query, {@code false}
    *         otherwise.
    */
   public boolean contains(EPAVertex3D query)
   {
      return v0.equals(query) || v1.equals(query) || v2.equals(query);
   }

   /** {@inheritDoc} */
   @Override
   public boolean canObserverSeeFace(Point3DReadOnly observer)
   {
      // The following test was suggested in the original algorithm.
      // However, it appears to not be robust to edge-cases where the new vertex would not be above 
      // the current entry.
      // if (TupleTools.dot(closestPoint, observer) >= normSquared)
      //    return true;
      return EuclidGeometryTools.isPoint3DAbovePlane3D(observer, v0, normal);
   }

   /**
    * Gets the coordinates of the point on this simplex that is the closest to the origin.
    * 
    * @return the reference to the closest point to the origin.
    */
   public Point3DReadOnly getClosestPointToOrigin()
   {
      return closestPointToOrigin;
   }

   /**
    * Whether the projection of the origin onto this face is located inside.
    *
    * @return {@code true} if the projection of the origin is internal to this face.
    */
   public boolean isClosestPointInternal()
   {
      return isClosestPointInternal;
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
    * Whether this triangle face is affinely dependent.
    * 
    * @return {@code true} if this face is affinely dependent, {@code false} if it is sane.
    */
   public boolean isTriangleAffinelyDependent()
   {
      return isTriangleAffinelyDependent;
   }

   /**
    * Marks this face as obsolete indicating that it is no longer part of a polytope and that it can be
    * discarded.
    * <p>
    * This method also marks the face's edges as obsolete.
    * </p>
    */
   public void markObsolete()
   {
      obsolete = true;
      e0.markObsolete();
      e1.markObsolete();
      e2.markObsolete();
   }

   /**
    * Whether this face has been discarded and is no longer part of a polytope.
    *
    * @return {@code true} if this face has been marked as obsolete.
    */
   public boolean isObsolete()
   {
      return obsolete;
   }

   /**
    * Computes the point on the shape A that corresponds to this simplex closest point to the origin.
    * 
    * @param pointOnAToPack the point used to store the result. Modified.
    */
   public void computePointOnA(Point3DBasics pointOnAToPack)
   {
      pointOnAToPack.setAndScale(lambda0, v0.getVertexOnShapeA());
      pointOnAToPack.scaleAdd(lambda1, v1.getVertexOnShapeA(), pointOnAToPack);
      pointOnAToPack.scaleAdd(lambda2, v2.getVertexOnShapeA(), pointOnAToPack);
   }

   /**
    * Computes the point on the shape B that corresponds to this simplex closest point to the origin.
    * 
    * @param pointOnBToPack the point used to store the result. Modified.
    */
   public void computePointOnB(Point3DBasics pointOnBToPack)
   {
      pointOnBToPack.setAndScale(lambda0, v0.getVertexOnShapeB());
      pointOnBToPack.scaleAdd(lambda1, v1.getVertexOnShapeB(), pointOnBToPack);
      pointOnBToPack.scaleAdd(lambda2, v2.getVertexOnShapeB(), pointOnBToPack);
   }

   /**
    * Compares this face to {@code other} based on their distance to the origin.
    */
   @Override
   public int compareTo(EPAFace3D other)
   {
      if (getDistanceSquaredToOrigin() == other.getDistanceSquaredToOrigin())
         return 0;
      if (getDistanceSquaredToOrigin() > other.getDistanceSquaredToOrigin())
         return 1;
      return -1;
   }

   /**
    * Gets the first edge of this face.
    * 
    * @return this face's first edge.
    */
   public EPAHalfEdge3D getEdge0()
   {
      return e0;
   }

   /**
    * Gets the second edge of this face.
    * 
    * @return this face's second edge.
    */
   public EPAHalfEdge3D getEdge1()
   {
      return e1;
   }

   /**
    * Gets the third edge of this face.
    * 
    * @return this face's third edge.
    */
   public EPAHalfEdge3D getEdge2()
   {
      return e2;
   }

   /** {@inheritDoc} */
   @Override
   public double getArea()
   {
      return EuclidGeometryTools.triangleArea(v0, v1, v2);
   }

   /** {@inheritDoc} */
   @Override
   public BoundingBox3DReadOnly getBoundingBox()
   {
      BoundingBox3D boundingBox3D = new BoundingBox3D();
      boundingBox3D.setToNaN();
      boundingBox3D.updateToIncludePoint(v0);
      boundingBox3D.updateToIncludePoint(v1);
      boundingBox3D.updateToIncludePoint(v2);
      return boundingBox3D;
   }

   /** {@inheritDoc} */
   @Override
   public Point3DReadOnly getCentroid()
   {
      return EuclidGeometryTools.averagePoint3Ds(Arrays.asList(v0, v1, v2));
   }

   /** {@inheritDoc} */
   @Override
   public List<? extends HalfEdge3DReadOnly> getEdges()
   {
      return Arrays.asList(e0, e1, e2);
   }

   /** {@inheritDoc} */
   @Override
   public Vector3DReadOnly getNormal()
   {
      return normal;
   }

   /**
    * Tests if the given {@code object}'s class is the same as this, in which case the method returns
    * {@link #equals(Face3DReadOnly)}, it returns {@code false} otherwise.
    *
    * @param object the object to compare against this. Not modified.
    * @return {@code true} if {@code object} and this are exactly equal, {@code false} otherwise.
    */
   @Override
   public boolean equals(Object object)
   {
      if (object instanceof Face3DReadOnly)
         return equals((Face3DReadOnly) object);
      else
         return false;
   }

   /**
    * Calculates and returns a hash code value from the value of each component of this face 3D.
    *
    * @return the hash code value for this face 3D.
    */
   @Override
   public int hashCode()
   {
      long hash = 1L;
      hash = EuclidHashCodeTools.combineHashCode(hash, v0.hashCode());
      hash = EuclidHashCodeTools.combineHashCode(hash, v1.hashCode());
      hash = EuclidHashCodeTools.combineHashCode(hash, v2.hashCode());
      return EuclidHashCodeTools.toIntHashCode(hash);
   }

   /**
    * Provides a {@code String} representation of this face 3D as follows:
    *
    * <pre>
    * EPA Face 3D: centroid: ( 2.621, -0.723, -1.355 ), normal: ( 0.903, -0.202,  0.378 ), area:  0.180, number of edges: 3
    *    [( 2.590, -0.496, -1.161 ); ( 2.746, -0.536, -1.554 )]
    *    [( 2.746, -0.536, -1.554 ); ( 2.651, -0.950, -1.549 )]
    *    [( 2.651, -0.950, -1.549 ); ( 2.590, -0.496, -1.161 )]
    * </pre>
    * 
    * @return the {@code String} representing this face 3D.
    */
   @Override
   public String toString()
   {
      return "EPA " + EuclidShapeIOTools.getFace3DString(this);
   }
}