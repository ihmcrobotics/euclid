package us.ihmc.euclid.referenceFrame.interfaces;

import us.ihmc.euclid.geometry.interfaces.BoundingBox3DBasics;
import us.ihmc.euclid.referenceFrame.FrameBoundingBox3D;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.exceptions.ReferenceFrameMismatchException;
import us.ihmc.euclid.shape.primitives.interfaces.Shape3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Point3DBasics;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DBasics;

/**
 * Read-only interface for representing a 3D shape expressed in a given reference frame.
 *
 * @author Sylvain Bertrand
 */
public interface FrameShape3DReadOnly extends Shape3DReadOnly, SupportingFrameVertexHolder
{
   /**
    * {@inheritDoc}
    * <p>
    * Note that the centroid is also the position of this cylinder.
    * </p>
    */
   @Override
   FramePoint3DReadOnly getCentroid();

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
    * @throws ReferenceFrameMismatchException if any of the frame arguments is not expressed in the
    *                                         same reference frame as {@code this}.
    */
   default boolean evaluatePoint3DCollision(Point3DReadOnly pointToCheck, FixedFramePoint3DBasics closestPointOnSurfaceToPack,
                                            FixedFrameVector3DBasics normalAtClosestPointToPack)
   {
      checkReferenceFrameMatch(closestPointOnSurfaceToPack);
      checkReferenceFrameMatch(normalAtClosestPointToPack);
      return evaluatePoint3DCollision(pointToCheck, (Point3DBasics) closestPointOnSurfaceToPack, (Vector3DBasics) normalAtClosestPointToPack);
   }

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
   default boolean evaluatePoint3DCollision(Point3DReadOnly pointToCheck, FramePoint3DBasics closestPointOnSurfaceToPack,
                                            FrameVector3DBasics normalAtClosestPointToPack)
   {
      closestPointOnSurfaceToPack.setReferenceFrame(getReferenceFrame());
      normalAtClosestPointToPack.setReferenceFrame(getReferenceFrame());
      return evaluatePoint3DCollision(pointToCheck, (Point3DBasics) closestPointOnSurfaceToPack, (Vector3DBasics) normalAtClosestPointToPack);
   }

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
    * @throws ReferenceFrameMismatchException if the frame argument is not expressed in the same
    *                                         reference frame as {@code this}.
    */
   default boolean evaluatePoint3DCollision(FramePoint3DReadOnly pointToCheck, Point3DBasics closestPointOnSurfaceToPack,
                                            Vector3DBasics normalAtClosestPointToPack)
   {
      checkReferenceFrameMatch(pointToCheck);
      return evaluatePoint3DCollision((Point3DReadOnly) pointToCheck, closestPointOnSurfaceToPack, normalAtClosestPointToPack);
   }

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
    * @throws ReferenceFrameMismatchException if any of the arguments is not expressed in the same
    *                                         reference frame as {@code this}.
    */
   default boolean evaluatePoint3DCollision(FramePoint3DReadOnly pointToCheck, FixedFramePoint3DBasics closestPointOnSurfaceToPack,
                                            FixedFrameVector3DBasics normalAtClosestPointToPack)
   {
      checkReferenceFrameMatch(pointToCheck);
      return evaluatePoint3DCollision((Point3DReadOnly) pointToCheck, closestPointOnSurfaceToPack, normalAtClosestPointToPack);
   }

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
    * @throws ReferenceFrameMismatchException if {@code pointToCheck} is not expressed in the same
    *                                         reference frame as {@code this}.
    */
   default boolean evaluatePoint3DCollision(FramePoint3DReadOnly pointToCheck, FramePoint3DBasics closestPointOnSurfaceToPack,
                                            FrameVector3DBasics normalAtClosestPointToPack)
   {
      checkReferenceFrameMatch(pointToCheck);
      return evaluatePoint3DCollision((Point3DReadOnly) pointToCheck, closestPointOnSurfaceToPack, normalAtClosestPointToPack);
   }

   /**
    * Calculates the minimum distance between a point and this shape.
    * <p>
    * Note that if the point is inside this shape, this method returns 0.0.
    * </p>
    *
    * @param point the coordinates of the query. Not modified.
    * @return the value of the distance between the point and this shape.
    * @throws ReferenceFrameMismatchException if the argument is not expressed in the same reference
    *                                         frame as {@code this}.
    */
   default double distance(FramePoint3DReadOnly point)
   {
      checkReferenceFrameMatch(point);
      return distance((Point3DReadOnly) point);
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
    * @throws ReferenceFrameMismatchException if the argument is not expressed in the same reference
    *                                         frame as {@code this}.
    */
   default double signedDistance(FramePoint3DReadOnly point)
   {
      checkReferenceFrameMatch(point);
      return signedDistance((Point3DReadOnly) point);
   }

   /**
    * Tests whether the given point is inside this shape or on its surface.
    *
    * @param query the coordinates of the query. Not modified.
    * @return true if the point is inside or on the surface, false otherwise.
    * @throws ReferenceFrameMismatchException if the argument is not expressed in the same reference
    *                                         frame as {@code this}.
    */
   default boolean isPointInside(FramePoint3DReadOnly query)
   {
      checkReferenceFrameMatch(query);
      return isPointInside((Point3DReadOnly) query);
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
    * @throws ReferenceFrameMismatchException if the argument is not expressed in the same reference
    *                                         frame as {@code this}.
    */
   default boolean isPointInside(FramePoint3DReadOnly query, double epsilon)
   {
      checkReferenceFrameMatch(query);
      return isPointInside((Point3DReadOnly) query, epsilon);
   }

   /** {@inheritDoc} */
   @Override
   default FramePoint3DBasics orthogonalProjectionCopy(Point3DReadOnly pointToProject)
   {
      FramePoint3D projection = new FramePoint3D();

      if (orthogonalProjection(pointToProject, projection))
         return projection;
      else
         return null;
   }

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
    * @throws ReferenceFrameMismatchException if the argument is not expressed in the same reference
    *                                         frame as {@code this}.
    */
   default FramePoint3DBasics orthogonalProjectionCopy(FramePoint3DReadOnly pointToProject)
   {
      checkReferenceFrameMatch(pointToProject);
      return orthogonalProjectionCopy((Point3DReadOnly) pointToProject);
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
    * @throws ReferenceFrameMismatchException if the argument is not expressed in the same reference
    *                                         frame as {@code this}.
    */
   default boolean orthogonalProjection(FixedFramePoint3DBasics pointToProject)
   {
      return orthogonalProjection(pointToProject, pointToProject);
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
   default boolean orthogonalProjection(FramePoint3DBasics pointToProject)
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
    * @throws ReferenceFrameMismatchException if the frame argument is not expressed in the same
    *                                         reference frame as {@code this}.
    */
   default boolean orthogonalProjection(Point3DReadOnly pointToProject, FixedFramePoint3DBasics projectionToPack)
   {
      checkReferenceFrameMatch(projectionToPack);
      return orthogonalProjection(pointToProject, (Point3DBasics) projectionToPack);
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
   default boolean orthogonalProjection(Point3DReadOnly pointToProject, FramePoint3DBasics projectionToPack)
   {
      projectionToPack.setReferenceFrame(getReferenceFrame());
      return orthogonalProjection(pointToProject, (Point3DBasics) projectionToPack);
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
    * @throws ReferenceFrameMismatchException if the frame argument is not expressed in the same
    *                                         reference frame as {@code this}.
    */
   default boolean orthogonalProjection(FramePoint3DReadOnly pointToProject, Point3DBasics projectionToPack)
   {
      checkReferenceFrameMatch(pointToProject);
      return orthogonalProjection((Point3DReadOnly) pointToProject, projectionToPack);
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
    * @throws ReferenceFrameMismatchException if any of the arguments is not expressed in the same
    *                                         reference frame as {@code this}.
    */
   default boolean orthogonalProjection(FramePoint3DReadOnly pointToProject, FixedFramePoint3DBasics projectionToPack)
   {
      checkReferenceFrameMatch(pointToProject);
      return orthogonalProjection((Point3DReadOnly) pointToProject, projectionToPack);
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
    * @throws ReferenceFrameMismatchException if {@code pointToProject} is not expressed in the same
    *                                         reference frame as {@code this}.
    */
   default boolean orthogonalProjection(FramePoint3DReadOnly pointToProject, FramePoint3DBasics projectionToPack)
   {
      checkReferenceFrameMatch(pointToProject);
      return orthogonalProjection((Point3DReadOnly) pointToProject, projectionToPack);
   }

   /** {@inheritDoc} */
   @Override
   default FrameBoundingBox3DReadOnly getBoundingBox()
   {
      FrameBoundingBox3D boundingBox3D = new FrameBoundingBox3D();
      getBoundingBox(boundingBox3D);
      return boundingBox3D;
   }

   /**
    * Gets the tightest axis-aligned bounding box that contains this shape in the given reference
    * frame.
    *
    * @param destinationFrame the reference frame in which the bounding is to be evaluated.
    * @return the bounding box.
    */
   default FrameBoundingBox3DReadOnly getBoundingBox(ReferenceFrame destinationFrame)
   {
      FrameBoundingBox3D boundingBox3D = new FrameBoundingBox3D();
      getBoundingBox(destinationFrame, boundingBox3D);
      return boundingBox3D;
   }

   /** {@inheritDoc} */
   @Override
   default void getBoundingBox(BoundingBox3DBasics boundingBoxToPack)
   {
      getBoundingBox(getReferenceFrame(), boundingBoxToPack);
   }

   /**
    * Gets the tightest axis-aligned bounding box that contains this shape in the reference frame in
    * which the bounding box is expressed.
    *
    * @param boundingBoxToPack the bounding box to pack. Modified.
    */
   default void getBoundingBox(FixedFrameBoundingBox3DBasics boundingBoxToPack)
   {
      getBoundingBox(boundingBoxToPack.getReferenceFrame(), boundingBoxToPack);
   }

   /**
    * Gets the tightest axis-aligned bounding box that contains this shape.
    *
    * @param boundingBoxToPack the bounding box to pack. Modified.
    */
   default void getBoundingBox(FrameBoundingBox3DBasics boundingBoxToPack)
   {
      getBoundingBox(getReferenceFrame(), boundingBoxToPack);
   }

   /**
    * Gets the tightest axis-aligned bounding box that contains this shape in the given reference
    * frame.
    *
    * @param destinationFrame  the reference frame in which the bounding is to be evaluated.
    * @param boundingBoxToPack the bounding box to pack. Modified.
    */
   void getBoundingBox(ReferenceFrame destinationFrame, BoundingBox3DBasics boundingBoxToPack);

   /**
    * Gets the tightest axis-aligned bounding box that contains this shape in the given reference
    * frame.
    *
    * @param destinationFrame  the reference frame in which the bounding is to be evaluated.
    * @param boundingBoxToPack the bounding box to pack. Modified.
    */
   default void getBoundingBox(ReferenceFrame destinationFrame, FrameBoundingBox3DBasics boundingBoxToPack)
   {
      boundingBoxToPack.setReferenceFrame(destinationFrame);
      getBoundingBox(destinationFrame, (BoundingBox3DBasics) boundingBoxToPack);
   }

   /**
    * Returns {@code null} as this shape is not defined by a pose.
    */
   @Override
   FrameShape3DPoseReadOnly getPose();

   /** {@inheritDoc} */
   @Override
   FixedFrameShape3DBasics copy();
}
