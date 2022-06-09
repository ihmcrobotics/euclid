package us.ihmc.euclid.shape.primitives.interfaces;

import us.ihmc.euclid.geometry.BoundingBox3D;
import us.ihmc.euclid.geometry.interfaces.BoundingBox3DBasics;
import us.ihmc.euclid.geometry.interfaces.BoundingBox3DReadOnly;
import us.ihmc.euclid.shape.collision.interfaces.SupportingVertexHolder;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DBasics;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DBasics;

/**
 * Read-only interface for representing a 3D shape.
 *
 * @author Sylvain Bertrand
 */
public interface Shape3DReadOnly extends SupportingVertexHolder
{
   /**
    * Gets the position of this shape centroid.
    *
    * @return this shape centroid.
    */
   Point3DReadOnly getCentroid();

   /**
    * Returns the volume for this shape.
    * 
    * @return the volume in cubic meters.
    */
   double getVolume();

   /**
    * Tests whether this shape contains at least one {@link Double#NaN} or not.
    *
    * @return {@code true} if this shape contains {@link Double#NaN}, {@code false} otherwise.
    */
   boolean containsNaN();

   /**
    * Evaluates the collision state between a point {@code pointToCheck} and this shape.
    *
    * @param pointToCheck                the coordinates of the query to be evaluated. Not modified.
    * @param closestPointOnSurfaceToPack the closest point to the query that lies on this shape
    *                                    surface. Modified.
    * @param normalAtClosestPointToPack  the surface normal at the closest point to the query. The
    *                                    normal points toward outside the shape. Modified.
    * @return {@code true} if the query is inside this shape or exactly on its surface, {@code false}
    *         otherwise.
    */
   boolean evaluatePoint3DCollision(Point3DReadOnly pointToCheck, Point3DBasics closestPointOnSurfaceToPack, Vector3DBasics normalAtClosestPointToPack);

   /**
    * Calculates the minimum distance between a point and this shape.
    * <p>
    * Note that if the point is inside this shape, this method returns 0.0.
    * </p>
    *
    * @param point the coordinates of the query. Not modified.
    * @return the value of the distance between the point and this shape.
    */
   default double distance(Point3DReadOnly point)
   {
      return Math.max(0.0, signedDistance(point));
   }

   /**
    * Returns minimum distance between the point and this shape.
    * <p>
    * The returned value is negative if the point is inside the shape.
    * </p>
    *
    * @param point the coordinates of the query. Not modified.
    * @return the distance between the query and the shape, it is negative if the point is inside the
    *         shape.
    */
   double signedDistance(Point3DReadOnly point);

   /**
    * Tests whether the given point is inside this shape or on its surface.
    *
    * @param query the coordinates of the query. Not modified.
    * @return true if the point is inside or on the surface, false otherwise.
    */
   default boolean isPointInside(Point3DReadOnly query)
   {
      return isPointInside(query, 0.0);
   }

   /**
    * Tests if the {@code query} is located inside this shape given the tolerance {@code epsilon}.
    * <p>
    * <ul>
    * <li>if {@code epsilon > 0}, the size of this shape is increased by shifting its surface/faces by
    * a distance of {@code epsilon} toward the outside.
    * <li>if {@code epsilon > 0}, the size of this shape is reduced by shifting its surface/faces by a
    * distance of {@code epsilon} toward the inside.
    * </ul>
    * </p>
    *
    * @param query   the coordinates of the query. Not modified.
    * @param epsilon the tolerance to use for this test.
    * @return {@code true} if the query is considered to be inside this shape, {@code false} otherwise.
    */
   boolean isPointInside(Point3DReadOnly query, double epsilon);

   /**
    * Computes the orthogonal projection of a point on this shape.
    * <p>
    * WARNING: This method generates garbage.
    * </p>
    * <p>
    * Edge cases:
    * <ul>
    * <li>If the query is inside the shape, the method fails and returns {@code null}.
    * </ul>
    * </p>
    *
    * @param pointToProject the point to project on this shape. Not modified.
    * @return the projection if this method succeeds, {@code null} otherwise.
    */
   default Point3DBasics orthogonalProjectionCopy(Point3DReadOnly pointToProject)
   {
      Point3D projection = new Point3D();

      if (orthogonalProjection(pointToProject, projection))
         return projection;
      else
         return null;
   }

   /**
    * Computes the orthogonal projection of a point on this shape.
    * <p>
    * Edge cases:
    * <ul>
    * <li>If the query is inside the shape, the method fails and returns {@code false}.
    * </ul>
    * </p>
    *
    * @param pointToProject the point to project on this shape. Modified.
    * @return whether the method succeeded or not.
    */
   default boolean orthogonalProjection(Point3DBasics pointToProject)
   {
      return orthogonalProjection(pointToProject, pointToProject);
   }

   /**
    * Computes the orthogonal projection of a point this shape.
    * <p>
    * Edge cases:
    * <ul>
    * <li>If the query is inside the shape, the method fails and returns {@code false}.
    * </ul>
    * </p>
    *
    * @param pointToProject   the coordinate of the point to compute the projection of. Not modified.
    * @param projectionToPack point in which the projection of the point onto this shape is stored.
    *                         Modified.
    * @return whether the method succeeded or not.
    */
   boolean orthogonalProjection(Point3DReadOnly pointToProject, Point3DBasics projectionToPack);

   /**
    * Gets the tightest axis-aligned bounding box that contains this shape.
    * <p>
    * WARNING: The default implementation of this method generates garbage.
    * </p>
    *
    * @return the bounding box.
    */
   default BoundingBox3DReadOnly getBoundingBox()
   {
      BoundingBox3D boundingBox3D = new BoundingBox3D();
      getBoundingBox(boundingBox3D);
      return boundingBox3D;
   }

   /**
    * Gets the tightest axis-aligned bounding box that contains this shape.
    *
    * @param boundingBoxToPack the bounding box to pack. Modified.
    */
   void getBoundingBox(BoundingBox3DBasics boundingBoxToPack);

   /**
    * Gets the convex property for this shape.
    *
    * @return {@code true} if this shape's implementation is convex, {@code false} otherwise.
    */
   boolean isConvex();

   /**
    * Gets whether this shape is a primitive shape or not.
    * <p>
    * A primitive shape is a simple geometric shape such as a sphere or a box. Transformation
    * operations on primitive shapes are cheap.
    * </p>
    *
    * @return {@code true} if this shape's implementation is a primitive, {@code false} otherwise.
    */
   boolean isPrimitive();

   /**
    * Indicates whether this geometry relies on a 3D pose, i.e. position and orientation, to defined in
    * space.
    * <p>
    * For instance, a 3D box is defined by its size and 3D pose, so
    * {@link Box3DReadOnly#isDefinedByPose()} is {@code true}, while a 3D sphere is defined by its
    * radius and position, so {@link Sphere3DReadOnly#isDefinedByPose()} is {@code false}.
    * </p>
    * <p>
    * For shapes defined by a pose, performing queries result in extra computation as an additional
    * transformation is needed. By using this information, algorithms can be adjusted to minimize their
    * number of operations.
    * </p>
    *
    * @return {@code true} if this geometry uses a 3D pose to position itself in space, {@code false}
    *         otherwise.
    */
   boolean isDefinedByPose();

   /**
    * Gets the pose defining this shape only if it is part of the geometry definition, i.e.
    * {@code this.isDefinedByPose() == true}, otherwise this method returns {@code null}.
    *
    * @return the pose defining this shape, or {@code null} if this shape is not defined with a pose.
    */
   Shape3DPoseReadOnly getPose();

   /**
    * Creates and returns a deep copy of this shape.
    *
    * @return a copy of this shape.
    */
   Shape3DBasics copy();
}
