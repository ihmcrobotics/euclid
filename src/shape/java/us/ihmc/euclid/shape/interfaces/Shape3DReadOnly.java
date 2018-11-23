package us.ihmc.euclid.shape.interfaces;

import us.ihmc.euclid.geometry.interfaces.Pose3DBasics;
import us.ihmc.euclid.interfaces.Transformable;
import us.ihmc.euclid.matrix.interfaces.RotationMatrixReadOnly;
import us.ihmc.euclid.orientation.interfaces.Orientation3DBasics;
import us.ihmc.euclid.shape.CollisionTestResult;
import us.ihmc.euclid.transform.interfaces.RigidBodyTransformBasics;
import us.ihmc.euclid.tuple3D.interfaces.Point3DBasics;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DBasics;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DBasics;
import us.ihmc.euclid.yawPitchRoll.YawPitchRoll;

public interface Shape3DReadOnly
{
   static final double IS_INSIDE_EPS = 1.0e-12;

   Shape3DPoseReadOnly getPose();

   /**
    * Gets the read-only reference to the orientation of this shape.
    *
    * @return the orientation of this shape.
    */
   default RotationMatrixReadOnly getOrientation()
   {
      return getPose().getShapeOrientation();
   }

   /**
    * Gets the read-only reference of the position of this shape.
    *
    * @return the position of this shape.
    */
   default Point3DReadOnly getPosition()
   {
      return getPose().getShapePosition();
   }

   default boolean containsNaN()
   {
      return getPose().containsNaN();
   }

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

   default CollisionTestResult doCollisionTest(Shape3DReadOnly otherShape)
   {
      CollisionTestResult collisionTestResult = new CollisionTestResult();
      doCollisionTest(otherShape, collisionTestResult);
      return collisionTestResult;
   }

   default void doCollisionTest(Shape3DReadOnly otherShape, CollisionTestResult result)
   {
      throw new UnsupportedOperationException("This shape does not support shape-to-shape collision: " + getClass().getSimpleName());
   }

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
   default boolean isInsideOrOnSurface(Point3DReadOnly query)
   {
      return isInsideEpsilon(query, IS_INSIDE_EPS);
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
   boolean isInsideEpsilon(Point3DReadOnly query, double epsilon);

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

   /**
    * Packs the orientation of this shape into {@code orientationToPack}.
    *
    * @param orientationToPack used to pack the orientation of this shape. Modified.
    */
   default void getOrientation(Orientation3DBasics orientationToPack)
   {
      orientationToPack.set(getOrientation());
   }

   /**
    * Computes and returns the pitch angle from the yaw-pitch-roll representation of this shape
    * orientation.
    * <p>
    * WARNING: the Euler angles or yaw-pitch-roll representation is sensitive to gimbal lock and is
    * sometimes undefined.
    * </p>
    *
    * @return the pitch angle around the y-axis.
    */
   default double getOrientationPitch()
   {
      return getOrientation().getPitch();
   }

   /**
    * Computes and returns the roll angle from the yaw-pitch-roll representation of this shape
    * orientation.
    * <p>
    * WARNING: the Euler angles or yaw-pitch-roll representation is sensitive to gimbal lock and is
    * sometimes undefined.
    * </p>
    *
    * @return the roll angle around the x-axis.
    */
   default double getOrientationRoll()
   {
      return getOrientation().getRoll();
   }

   /**
    * Computes and returns the yaw angle from the yaw-pitch-roll representation of this shape
    * orientation.
    * <p>
    * WARNING: the Euler angles or yaw-pitch-roll representation is sensitive to gimbal lock and is
    * sometimes undefined.
    * </p>
    *
    * @return the yaw angle around the z-axis.
    */
   default double getOrientationYaw()
   {
      return getOrientation().getYaw();
   }

   /**
    * Computes and packs the orientation of this shape into the given yaw-pitch-roll angles.
    * <p>
    * WARNING: the Euler angles or yaw-pitch-roll representation is sensitive to gimbal lock and is
    * sometimes undefined.
    * </p>
    *
    * @param yawPitchRollToPack the array in which the yaw-pitch-roll angles are stored. Modified.
    * @deprecated Use a {@link YawPitchRoll} that can be set to {@link #getOrientation()}.
    */
   @Deprecated
   default void getOrientationYawPitchRoll(double[] yawPitchRollToPack)
   {
      getOrientation().getYawPitchRoll(yawPitchRollToPack);
   }

   /**
    * Packs the pose of this shape into the given {@code poseToPack}.
    *
    * @param poseToPack the pose in which the position and orientation of this shape are stored.
    *           Modified.
    */
   default void getPose(Pose3DBasics poseToPack)
   {
      poseToPack.set(getPosition(), getOrientation());
   }

   /**
    * Packs the pose of this shape into the given {@code transformToPack}.
    *
    * @param transformToPack the rigid-body transform in which the position and orientation of this
    *           shape are stored. Modified.
    */
   default void getPose(RigidBodyTransformBasics transformToPack)
   {
      transformToPack.set(getOrientation(), getPosition());
   }

   /**
    * Packs the position of this shape.
    *
    * @param tupleToPack the tuple in which the position of this shape is stored. Modified.
    */
   default void getPosition(Tuple3DBasics tupleToPack)
   {
      tupleToPack.set(getPosition());
   }

   /**
    * Gets the x-coordinate of the position of this shape.
    *
    * @return the x-coordinate of this shape position.
    */
   default double getPositionX()
   {
      return getPosition().getX();
   }

   /**
    * Gets the y-coordinate of the position of this shape.
    *
    * @return the y-coordinate of this shape position.
    */
   default double getPositionY()
   {
      return getPosition().getY();
   }

   /**
    * Gets the z-coordinate of the position of this shape.
    *
    * @return the z-coordinate of this shape position.
    */
   default double getPositionZ()
   {
      return getPosition().getZ();
   }

   /**
    * Changes the given {@code transformable} from being expressed in world to being expressed in this
    * shape local coordinates.
    *
    * @param transformable the transformable to change the coordinates in which it is expressed.
    *           Modified.
    */
   default void transformToLocal(Transformable transformable)
   {
      transformable.applyInverseTransform(getPose());
   }

   /**
    * Changes the given {@code transformable} from being expressed in this shape local coordinates to
    * being expressed in world.
    *
    * @param transformable the transformable to change the coordinates in which it is expressed.
    *           Modified.
    */
   default void transformToWorld(Transformable transformable)
   {
      transformable.applyTransform(getPose());
   }
}
