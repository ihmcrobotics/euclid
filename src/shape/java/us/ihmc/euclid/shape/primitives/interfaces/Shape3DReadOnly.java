package us.ihmc.euclid.shape.primitives.interfaces;

import us.ihmc.euclid.shape.collision.SupportingVertexHolder;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DBasics;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DBasics;

public interface Shape3DReadOnly extends SupportingVertexHolder
{
   static final double IS_INSIDE_EPS = 1.0e-12;

   boolean containsNaN();

   /**
    * Evaluates the query point {@code pointToCheck}:
    * <ul>
    * <li>tests if the query is located inside this shape,
    * <li>calculates the coordinates of the closest point to the query and on laying on the shape
    * surface,
    * <li>calculates the normal of the shape surface at the coordinates of the closest point to the
    * query.
    * </ul>
    *
    * @param pointToCheck the coordinates of the query to be evaluated. Not modified.
    * @param closestPointOnSurfaceToPack the closest point to the query that lies on the shape surface.
    *           Modified.
    * @param normalAtClosestPointToPack the surface normal at the closest point to the query. The
    *           normal points toward outside the shape. Modified.
    * @return {@code true} if the query is inside this shape or exactly on its surface, {@code false}
    *         otherwise.
    */
   boolean doPoint3DCollisionTest(Point3DReadOnly pointToCheck, Point3DBasics closestPointOnSurfaceToPack, Vector3DBasics normalAtClosestPointToPack);

   /**
    * Calculates the minimum distance between the point and this shape.
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
    * @param query the coordinates of the query. Not modified.
    * @param epsilon the tolerance to use for this test.
    * @return {@code true} if the query is considered to be inside this shape, {@code false} otherwise.
    */
   boolean isPointInside(Point3DReadOnly query, double epsilon);

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
    * @param pointToProject the coordinate of the point to compute the projection of. Not modified.
    * @param projectionToPack point in which the projection of the point onto this shape is stored.
    *           Modified.
    * @return whether the method succeeded or not.
    */
   boolean orthogonalProjection(Point3DReadOnly pointToProject, Point3DBasics projectionToPack);
}
